import argparse
import csv
import json
import math
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
import sys
import threading
import time

from booster_robotics_sdk_python import (
    B1JointCnt,
    B1LocoClient,
    B1LowCmdPublisher,
    B1LowStateSubscriber,
    ChannelFactory,
    LowCmd,
    LowCmdType,
    MotorCmd,
    RobotMode,
)


GRAVITY = 9.80665

# Booster T1 SERIAL LowCmd index order. Body joints match
# example/low_level/data/t1_default_kpkd_jointorque.csv (BOOSTER_T1_CFG base
# gains). The IMU calibration task then overlays the lying pose and the gain
# scales from robot_learning/envs/t1_booster/t1_imu_calibration_cfg.py.
DEFAULT_JOINT_NAMES = [
    "Head_Yaw",
    "Head_Pitch",
    "Left_Shoulder_Pitch",
    "Left_Shoulder_Roll",
    "Left_Elbow_Pitch",
    "Left_Elbow_Yaw",
    "Right_Shoulder_Pitch",
    "Right_Shoulder_Roll",
    "Right_Elbow_Pitch",
    "Right_Elbow_Yaw",
    "Waist",
    "Left_Hip_Pitch",
    "Left_Hip_Roll",
    "Left_Hip_Yaw",
    "Left_Knee_Pitch",
    "Left_Ankle_Pitch",
    "Left_Ankle_Roll",
    "Right_Hip_Pitch",
    "Right_Hip_Roll",
    "Right_Hip_Yaw",
    "Right_Knee_Pitch",
    "Right_Ankle_Pitch",
    "Right_Ankle_Roll",
]

DEFAULT_BASE_GAINS_CSV = (
    Path(__file__).resolve().parent / "data" / "t1_default_kpkd_jointorque.csv"
)
DEFAULT_COMMAND_CSV = (
    Path(__file__).resolve().parent / "data" / "t1_imu_calibration_command.csv"
)

# Lying pose from T1_IMU_CALIBRATION_LYING_POSE.
IMU_CALIBRATION_LYING_POSE = {
    "Left_Hip_Pitch": -1.001578,
    "Right_Hip_Pitch": -1.001578,
    "Left_Hip_Roll": -0.032290,
    "Right_Hip_Roll": 0.032290,
    "Left_Hip_Yaw": 0.073587,
    "Right_Hip_Yaw": -0.073587,
    "Left_Knee_Pitch": 2.30,
    "Right_Knee_Pitch": 2.30,
    "Left_Ankle_Pitch": 0.040022,
    "Right_Ankle_Pitch": 0.040022,
    "Left_Ankle_Roll": -0.003340,
    "Right_Ankle_Roll": 0.003340,
    "Waist": 0.0,
    "Left_Shoulder_Pitch": -1.40,
    "Right_Shoulder_Pitch": -1.40,
    "Left_Shoulder_Roll": -1.45,
    "Right_Shoulder_Roll": 1.45,
    "Left_Elbow_Pitch": 1.57,
    "Right_Elbow_Pitch": 1.57,
    "Left_Elbow_Yaw": -0.15,
    "Right_Elbow_Yaw": 0.15,
}

# Gain scales from T1TrainDRSStepIMUPoseCalibrationEnvCfg.
IMU_CALIB_JOINT_STIFFNESS_SCALE = 0.10
IMU_CALIB_JOINT_DAMPING_SCALE = 0.22
IMU_CALIB_SHOULDER_STIFFNESS_SCALE = 0.20
IMU_CALIB_SHOULDER_DAMPING_SCALE = 0.05
IMU_CALIB_ELBOW_STIFFNESS_SCALE = 0.36
IMU_CALIB_ELBOW_DAMPING_SCALE = 0.14

SHOULDER_JOINTS = {
    "Left_Shoulder_Pitch",
    "Left_Shoulder_Roll",
    "Right_Shoulder_Pitch",
    "Right_Shoulder_Roll",
}
ELBOW_JOINTS = {
    "Left_Elbow_Pitch",
    "Left_Elbow_Yaw",
    "Right_Elbow_Pitch",
    "Right_Elbow_Yaw",
}

# Soft head hold; body rows come from the CSV + IMU calibration overlay.
DEFAULT_Q = [0.0] * B1JointCnt
DEFAULT_KP = [5.0, 5.0] + [0.0] * (B1JointCnt - 2)
DEFAULT_KD = [0.1, 0.1] + [0.0] * (B1JointCnt - 2)
DEFAULT_TAU_LIMIT = [0.0] * B1JointCnt


@dataclass(frozen=True)
class LowStateSnapshot:
    time_sec: float
    rpy: tuple
    gyro: tuple
    acc: tuple
    motor_q: tuple
    motor_dq: tuple
    motor_tau_est: tuple


@dataclass(frozen=True)
class BiasEstimate:
    time_sec: float
    ready: bool
    history_size: int
    gyro_bias: tuple
    acc_bias: tuple
    mean_abs_joint_velocity: float


@dataclass(frozen=True)
class CommandConfig:
    low_cmd: LowCmd
    target_q: tuple


class LowStateBuffer:
    def __init__(self, proprio_source):
        self.proprio_source = proprio_source
        self._lock = threading.Lock()
        self._latest = None

    def handle_low_state(self, low_state_msg):
        imu = low_state_msg.imu_state
        motors = (
            low_state_msg.motor_state_parallel
            if self.proprio_source == "parallel"
            else low_state_msg.motor_state_serial
        )
        motor_q = [0.0] * B1JointCnt
        motor_dq = [0.0] * B1JointCnt
        motor_tau_est = [0.0] * B1JointCnt

        for i, motor in enumerate(motors):
            if i >= B1JointCnt:
                break
            motor_q[i] = motor.q
            motor_dq[i] = motor.dq
            motor_tau_est[i] = motor.tau_est

        snapshot = LowStateSnapshot(
            time_sec=time.time(),
            rpy=tuple(imu.rpy),
            gyro=tuple(imu.gyro),
            acc=tuple(imu.acc),
            motor_q=tuple(motor_q),
            motor_dq=tuple(motor_dq),
            motor_tau_est=tuple(motor_tau_est),
        )

        with self._lock:
            self._latest = snapshot

    def latest(self):
        with self._lock:
            return self._latest


class BaselineBiasPolicy:
    def __init__(self, history_steps):
        self.history_steps = history_steps

    @staticmethod
    def observation_vector(history):
        values = []
        for obs in history:
            values.extend(obs.rpy)
            values.extend(obs.gyro)
            values.extend(obs.acc)
            values.extend(obs.motor_q)
            values.extend(obs.motor_dq)
            values.extend(obs.motor_tau_est)
        return values

    def estimate(self, history):
        if not history:
            return BiasEstimate(
                time_sec=time.time(),
                ready=False,
                history_size=0,
                gyro_bias=(0.0, 0.0, 0.0),
                acc_bias=(0.0, 0.0, 0.0),
                mean_abs_joint_velocity=0.0,
            )

        gyro_bias = tuple(
            sum(obs.gyro[i] for obs in history) / len(history) for i in range(3)
        )
        acc_mean = tuple(
            sum(obs.acc[i] for obs in history) / len(history) for i in range(3)
        )
        expected_acc = expected_gravity_from_rpy(history[-1].rpy)
        acc_bias = tuple(acc_mean[i] - expected_acc[i] for i in range(3))
        mean_abs_joint_velocity = sum(
            abs(dq) for obs in history for dq in obs.motor_dq
        ) / (len(history) * B1JointCnt)

        return BiasEstimate(
            time_sec=history[-1].time_sec,
            ready=len(history) >= self.history_steps,
            history_size=len(history),
            gyro_bias=gyro_bias,
            acc_bias=acc_bias,
            mean_abs_joint_velocity=mean_abs_joint_velocity,
        )


def expected_gravity_from_rpy(rpy):
    roll, pitch, _ = rpy
    sr = math.sin(roll)
    cr = math.cos(roll)
    sp = math.sin(pitch)
    cp = math.cos(pitch)
    return (-GRAVITY * sp, GRAVITY * sr * cp, GRAVITY * cr * cp)


def default_log_path():
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path.home() / "t1_imu_logs" / f"imu_calibration_{timestamp}.csv"


def _empty_command_arrays():
    return {
        "q": [0.0] * B1JointCnt,
        "kp": [5.0, 5.0] + [0.0] * (B1JointCnt - 2),
        "kd": [0.1, 0.1] + [0.0] * (B1JointCnt - 2),
        "tau_limit": [0.0] * B1JointCnt,
    }


def load_command_csv(path):
    """Load T1 q/kp/kd rows keyed by joint_name into SERIAL LowCmd order."""
    path = Path(path).expanduser()
    name_to_index = {name: index for index, name in enumerate(DEFAULT_JOINT_NAMES)}
    arrays = _empty_command_arrays()
    missing_required = set(DEFAULT_JOINT_NAMES[2:])  # head is optional
    seen = set()

    with path.open(newline="") as csv_file:
        reader = csv.DictReader(csv_file)
        required_columns = {
            "joint_name",
            "default_qpos_rad",
            "kp",
            "kd",
        }
        if reader.fieldnames is None or not required_columns.issubset(reader.fieldnames):
            raise ValueError(
                f"{path} must include columns joint_name, default_qpos_rad, kp, kd"
            )

        for row in reader:
            joint_name = row["joint_name"].strip()
            if joint_name not in name_to_index:
                aliases = {
                    "AAHead_yaw": "Head_Yaw",
                    "Head_pitch": "Head_Pitch",
                }
                joint_name = aliases.get(joint_name, joint_name)
            if joint_name not in name_to_index:
                continue

            index = name_to_index[joint_name]
            if joint_name in seen:
                raise ValueError(f"Duplicate joint_name '{joint_name}' in {path}")
            seen.add(joint_name)
            missing_required.discard(joint_name)

            arrays["q"][index] = float(row["default_qpos_rad"])
            arrays["kp"][index] = float(row["kp"])
            arrays["kd"][index] = float(row["kd"])
            if "tau_limit_nm" in row and row["tau_limit_nm"] not in (None, ""):
                arrays["tau_limit"][index] = float(row["tau_limit_nm"])

    if missing_required:
        missing = ", ".join(sorted(missing_required))
        raise ValueError(f"{path} is missing required joints: {missing}")
    return arrays


def _gain_scales_for_joint(joint_name):
    if joint_name in SHOULDER_JOINTS:
        return IMU_CALIB_SHOULDER_STIFFNESS_SCALE, IMU_CALIB_SHOULDER_DAMPING_SCALE
    if joint_name in ELBOW_JOINTS:
        return IMU_CALIB_ELBOW_STIFFNESS_SCALE, IMU_CALIB_ELBOW_DAMPING_SCALE
    if joint_name in ("Head_Yaw", "Head_Pitch"):
        return 1.0, 1.0
    return IMU_CALIB_JOINT_STIFFNESS_SCALE, IMU_CALIB_JOINT_DAMPING_SCALE


def build_imu_calibration_command(base_gains_csv=DEFAULT_BASE_GAINS_CSV):
    """Compose lying-pose targets with IMU-calibration-scaled base gains."""
    arrays = load_command_csv(base_gains_csv)
    name_to_index = {name: index for index, name in enumerate(DEFAULT_JOINT_NAMES)}

    for joint_name, q_value in IMU_CALIBRATION_LYING_POSE.items():
        if joint_name not in name_to_index:
            raise ValueError(f"Unknown IMU calibration pose joint '{joint_name}'")
        arrays["q"][name_to_index[joint_name]] = float(q_value)

    for joint_name, index in name_to_index.items():
        stiffness_scale, damping_scale = _gain_scales_for_joint(joint_name)
        arrays["kp"][index] *= stiffness_scale
        arrays["kd"][index] *= damping_scale

    return arrays


def write_command_csv(path, arrays, source_config="t1_imu_calibration_cfg+BOOSTER_T1_CFG"):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(
            csv_file,
            fieldnames=[
                "joint_index",
                "joint_name",
                "default_qpos_rad",
                "kp",
                "kd",
                "tau_limit_nm",
                "source_config",
            ],
            lineterminator="\n",
        )
        writer.writeheader()
        for index, joint_name in enumerate(DEFAULT_JOINT_NAMES):
            writer.writerow(
                {
                    "joint_index": index,
                    "joint_name": joint_name,
                    "default_qpos_rad": round(arrays["q"][index], 6),
                    "kp": round(arrays["kp"][index], 6),
                    "kd": round(arrays["kd"][index], 6),
                    "tau_limit_nm": round(arrays["tau_limit"][index], 6),
                    "source_config": source_config,
                }
            )


def apply_imu_calibration_defaults(
    base_gains_csv=DEFAULT_BASE_GAINS_CSV, command_csv=DEFAULT_COMMAND_CSV
):
    arrays = build_imu_calibration_command(base_gains_csv)
    write_command_csv(command_csv, arrays)
    DEFAULT_Q[:] = arrays["q"]
    DEFAULT_KP[:] = arrays["kp"]
    DEFAULT_KD[:] = arrays["kd"]
    DEFAULT_TAU_LIMIT[:] = arrays["tau_limit"]
    return arrays


apply_imu_calibration_defaults()


def load_command_config(path):
    config = {
        "cmd_type": "SERIAL",
        "q": list(DEFAULT_Q),
        "kp": list(DEFAULT_KP),
        "kd": list(DEFAULT_KD),
        "tau_limit": list(DEFAULT_TAU_LIMIT),
        "dq": 0.0,
        "tau": 0.0,
        "weight": 0.0,
    }
    if path is None:
        return config

    config_path = Path(path).expanduser()
    suffix = config_path.suffix.lower()
    if suffix == ".csv":
        config.update(load_command_csv(config_path))
        return config

    with config_path.open() as config_file:
        user_config = json.load(config_file)
    config.update(user_config)
    return config


def validate_joint_vector(name, values):
    if len(values) != B1JointCnt:
        raise ValueError(f"{name} must contain {B1JointCnt} values, got {len(values)}")
    return [float(value) for value in values]


def build_command_config(config):
    q = validate_joint_vector("q", config["q"])
    kp = validate_joint_vector("kp", config["kp"])
    kd = validate_joint_vector("kd", config["kd"])
    dq = float(config.get("dq", 0.0))
    tau = float(config.get("tau", 0.0))
    weight = float(config.get("weight", 0.0))
    cmd_type_name = str(config.get("cmd_type", "SERIAL")).upper()

    if cmd_type_name == "PARALLEL":
        cmd_type = LowCmdType.PARALLEL
    elif cmd_type_name == "SERIAL":
        cmd_type = LowCmdType.SERIAL
    else:
        raise ValueError("cmd_type must be SERIAL or PARALLEL")

    low_cmd = LowCmd()
    low_cmd.cmd_type = cmd_type
    low_cmd.motor_cmd = [MotorCmd() for _ in range(B1JointCnt)]

    for i in range(B1JointCnt):
        low_cmd.motor_cmd[i].q = q[i]
        low_cmd.motor_cmd[i].dq = dq
        low_cmd.motor_cmd[i].tau = tau
        low_cmd.motor_cmd[i].kp = kp[i]
        low_cmd.motor_cmd[i].kd = kd[i]
        low_cmd.motor_cmd[i].weight = weight

    return CommandConfig(low_cmd=low_cmd, target_q=tuple(q))


def set_command_positions(low_cmd, q_values):
    for i, q in enumerate(q_values):
        low_cmd.motor_cmd[i].q = q


def ramp_positions(current_q, target_q, max_delta):
    return tuple(
        current_q[i] + max(-max_delta, min(max_delta, target_q[i] - current_q[i]))
        for i in range(B1JointCnt)
    )


class CsvLogger:
    def __init__(self, path):
        self.file = None
        self.writer = None
        if path is None:
            return

        path = Path(path).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        self.file = path.open("w", newline="")
        self.writer = csv.writer(self.file)
        self.writer.writerow(
            [
                "time_sec",
                "history_size",
                "ready",
                "roll_rad",
                "pitch_rad",
                "yaw_rad",
                "gyro_x",
                "gyro_y",
                "gyro_z",
                "acc_x",
                "acc_y",
                "acc_z",
                "gyro_bias_x",
                "gyro_bias_y",
                "gyro_bias_z",
                "acc_bias_x",
                "acc_bias_y",
                "acc_bias_z",
                "mean_abs_joint_velocity",
            ]
        )
        print(f"Logging calibration observations to {path}")

    def write(self, snapshot, estimate):
        if self.writer is None:
            return
        self.writer.writerow(
            [
                f"{snapshot.time_sec:.6f}",
                estimate.history_size,
                int(estimate.ready),
                *[f"{value:.9f}" for value in snapshot.rpy],
                *[f"{value:.9f}" for value in snapshot.gyro],
                *[f"{value:.9f}" for value in snapshot.acc],
                *[f"{value:.9f}" for value in estimate.gyro_bias],
                *[f"{value:.9f}" for value in estimate.acc_bias],
                f"{estimate.mean_abs_joint_velocity:.9f}",
            ]
        )

    def close(self):
        if self.file is not None:
            self.file.close()
            self.file = None
            self.writer = None


class KeyboardStop:
    def __init__(self):
        self.event = threading.Event()
        self.thread = None

    def start(self):
        self.thread = threading.Thread(target=self._read_stdin, daemon=True)
        self.thread.start()

    def _read_stdin(self):
        while not self.event.is_set():
            line = sys.stdin.readline()
            if line == "":
                return
            command = line.strip().lower()
            if command in ("x", "stop", "estop", "e-stop", "damping"):
                self.event.set()
                return


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Run a 50 Hz IMU calibration loop using constant low-level joint "
            "targets/gains and a history-based bias estimator."
        )
    )
    parser.add_argument(
        "network_interface",
        nargs="?",
        help="Optional DDS network interface or local IP address.",
    )
    parser.add_argument(
        "--config",
        help=(
            "Optional command config CSV/JSON. Default is the IMU calibration "
            f"lying pose with scaled gains written to {DEFAULT_COMMAND_CSV.name} "
            f"(base gains from {DEFAULT_BASE_GAINS_CSV.name})."
        ),
    )
    parser.add_argument(
        "--enable-control",
        action="store_true",
        help="Actually publish LowCmd messages. Without this, the script only observes.",
    )
    parser.add_argument(
        "--manual-custom-mode",
        action="store_true",
        help=(
            "Do not call ChangeMode(kCustom); assumes you will switch the robot "
            "to Custom mode manually."
        ),
    )
    parser.add_argument(
        "--return-mode",
        choices=("prepare", "damping", "none"),
        default="prepare",
        help="Mode to request on exit after API custom-mode control. Default: prepare.",
    )
    parser.add_argument(
        "--disable-keyboard-stop",
        action="store_true",
        help="Disable the runtime 'x'/'stop' + Enter software stop listener.",
    )
    parser.add_argument(
        "--no-confirm",
        action="store_true",
        help="Skip the Enter prompt before publishing control commands.",
    )
    parser.add_argument(
        "--control-dt",
        type=float,
        default=0.02,
        help="Observation/control timestep in seconds. Default: 0.02.",
    )
    parser.add_argument(
        "--ramp-time",
        type=float,
        default=2.0,
        help="Seconds to ramp from measured joint positions to target. Default: 2.0.",
    )
    parser.add_argument(
        "--max-joint-velocity",
        type=float,
        default=0.5,
        help="Maximum commanded target-position change, rad/s. Default: 0.5.",
    )
    parser.add_argument(
        "--max-abs-roll-pitch",
        type=float,
        default=2.0,
        help=(
            "Stop control if abs(roll) or abs(pitch) exceeds this. "
            "Default: 2.0 (allows the IMU-calibration lying pose)."
        ),
    )
    parser.add_argument(
        "--state-timeout",
        type=float,
        default=5.0,
        help="Seconds to wait for low-state data before enabling control. Default: 5.0.",
    )
    parser.add_argument(
        "--history-steps",
        type=int,
        default=100,
        help=(
            "Number of observations used by the policy. Default: 100 "
            "(matches the T1 IMU calibration supervised history window)."
        ),
    )
    parser.add_argument(
        "--max-steps",
        type=int,
        default=0,
        help="Stop after this many loop steps. Use 0 for unlimited. Default: 0.",
    )
    parser.add_argument(
        "--print-period",
        type=float,
        default=0.5,
        help="Seconds between terminal prints. Default: 0.5.",
    )
    parser.add_argument(
        "--proprio-source",
        choices=("serial", "parallel"),
        default="serial",
        help="LowState motor_state source used for proprioception. Default: serial.",
    )
    parser.add_argument(
        "--log",
        nargs="?",
        const=str(default_log_path()),
        default=str(default_log_path()),
        help="CSV log path. Default: ~/t1_imu_logs/imu_calibration_<timestamp>.csv.",
    )
    parser.add_argument(
        "--no-log",
        action="store_true",
        help="Do not create a CSV log file.",
    )

    args = parser.parse_args()
    if args.control_dt <= 0.0:
        parser.error("--control-dt must be positive")
    if args.ramp_time < 0.0:
        parser.error("--ramp-time must be 0 or greater")
    if args.max_joint_velocity <= 0.0:
        parser.error("--max-joint-velocity must be positive")
    if args.max_abs_roll_pitch <= 0.0:
        parser.error("--max-abs-roll-pitch must be positive")
    if args.state_timeout <= 0.0:
        parser.error("--state-timeout must be positive")
    if args.history_steps <= 0:
        parser.error("--history-steps must be positive")
    if args.max_steps < 0:
        parser.error("--max-steps must be 0 or greater")
    return args


def main():
    args = parse_args()
    config = load_command_config(args.config)
    command_config = build_command_config(config)
    low_cmd = command_config.low_cmd

    if args.network_interface:
        ChannelFactory.Instance().Init(0, args.network_interface)
    else:
        ChannelFactory.Instance().Init(0)

    state_buffer = LowStateBuffer(args.proprio_source)
    subscriber = B1LowStateSubscriber(state_buffer.handle_low_state)
    publisher = B1LowCmdPublisher()
    subscriber.InitChannel()
    publisher.InitChannel()
    client = None
    if (
        args.enable_control
        and (
            not args.manual_custom_mode
            or not args.disable_keyboard_stop
            or args.return_mode != "none"
        )
    ):
        client = B1LocoClient()
        client.Init()

    policy = BaselineBiasPolicy(args.history_steps)
    history = deque(maxlen=args.history_steps)
    logger = CsvLogger(None if args.no_log else args.log)
    control_started = False
    keyboard_stop = KeyboardStop()
    keyboard_stop_requested = False

    mode = "CONTROL ENABLED" if args.enable_control else "observe only"
    print(
        f"Running IMU calibration policy at {args.control_dt:.3f}s dt "
        f"({mode}); history_steps={args.history_steps}"
    )
    print(
        "Command config: "
        + (str(Path(args.config).expanduser()) if args.config else str(DEFAULT_COMMAND_CSV))
    )
    print(
        f"Target q sample (L hip/knee/ankle pitch): "
        f"{command_config.target_q[11]:.3f}, "
        f"{command_config.target_q[14]:.3f}, "
        f"{command_config.target_q[15]:.3f}"
    )
    obs_dim_per_step = 9 + 3 * B1JointCnt
    print(
        f"Observation shape: {args.history_steps} x {obs_dim_per_step} "
        f"({args.history_steps * obs_dim_per_step} flat values)"
    )
    print(f"Low-state channel: {subscriber.GetChannelName()}")
    print(f"Low-command channel: {publisher.GetChannelName()}")

    if args.enable_control:
        print(
            "Control checklist: robot in PREP/Prepare, lying/supine on a stable "
            "surface matching the IMU calibration pose, clear area, ready to "
            "switch to Custom mode."
        )
        wait_start = time.time()
        snapshot = state_buffer.latest()
        while snapshot is None and time.time() - wait_start < args.state_timeout:
            time.sleep(0.01)
            snapshot = state_buffer.latest()
        if snapshot is None:
            raise RuntimeError("No low-state data received; refusing to enable control")

        if (
            abs(snapshot.rpy[0]) > args.max_abs_roll_pitch
            or abs(snapshot.rpy[1]) > args.max_abs_roll_pitch
        ):
            raise RuntimeError(
                "Initial roll/pitch is outside the safety threshold; refusing control"
        )

        if not args.no_confirm:
            input("Press ENTER to publish an initial hold command and start control...")

        current_target_q = tuple(snapshot.motor_q)
        set_command_positions(low_cmd, current_target_q)
        publisher.Write(low_cmd)
        control_started = True

        if not args.manual_custom_mode:
            result = client.ChangeMode(RobotMode.kCustom)
            if result != 0:
                raise RuntimeError(f"ChangeMode(kCustom) failed with code {result}")
            print("Requested RobotMode.kCustom")
        else:
            print("Manual custom mode selected; switch the robot to Custom mode now.")
            if not args.no_confirm:
                input("Press ENTER after the robot is in Custom mode...")

        if not args.disable_keyboard_stop:
            keyboard_stop.start()
            print("Software stop armed: type 'x' or 'stop' then ENTER to request Damping.")
    else:
        current_target_q = command_config.target_q

    step = 0
    last_print = 0.0
    next_time = time.monotonic()
    max_joint_delta = args.max_joint_velocity * args.control_dt
    if args.ramp_time > 0.0:
        needed_ramp_delta = (
            max(
                abs(command_config.target_q[i] - current_target_q[i])
                for i in range(B1JointCnt)
            )
            * args.control_dt
            / args.ramp_time
        )
        ramp_joint_delta = min(max_joint_delta, needed_ramp_delta)
    else:
        ramp_joint_delta = max_joint_delta

    try:
        while args.max_steps == 0 or step < args.max_steps:
            now_mono = time.monotonic()
            if now_mono < next_time:
                time.sleep(next_time - now_mono)
            next_time += args.control_dt

            snapshot = state_buffer.latest()
            if snapshot is None:
                if time.time() - last_print >= args.print_period:
                    print("Waiting for low-state data...", flush=True)
                    last_print = time.time()
                continue

            history.append(snapshot)
            estimate = policy.estimate(history)

            if args.enable_control:
                if keyboard_stop.event.is_set():
                    keyboard_stop_requested = True
                    print("Keyboard software stop requested; stopping control")
                    break

                if (
                    abs(snapshot.rpy[0]) > args.max_abs_roll_pitch
                    or abs(snapshot.rpy[1]) > args.max_abs_roll_pitch
                ):
                    print("Roll/pitch safety threshold exceeded; stopping control")
                    break

                current_target_q = ramp_positions(
                    current_target_q, command_config.target_q, ramp_joint_delta
                )
                set_command_positions(low_cmd, current_target_q)
                publisher.Write(low_cmd)

            logger.write(snapshot, estimate)

            if time.time() - last_print >= args.print_period:
                ready = "ready" if estimate.ready else "warming"
                print(
                    f"{ready} n={estimate.history_size} "
                    f"gyro_bias=({estimate.gyro_bias[0]: .5f}, "
                    f"{estimate.gyro_bias[1]: .5f}, "
                    f"{estimate.gyro_bias[2]: .5f}) "
                    f"acc_bias=({estimate.acc_bias[0]: .5f}, "
                    f"{estimate.acc_bias[1]: .5f}, "
                    f"{estimate.acc_bias[2]: .5f}) "
                    f"mean_abs_dq={estimate.mean_abs_joint_velocity: .5f}",
                    flush=True,
                )
                last_print = time.time()

            step += 1
    except KeyboardInterrupt:
        print("\nStopped")
    finally:
        if control_started and args.return_mode != "none" and client is not None:
            if keyboard_stop_requested or args.return_mode == "damping":
                return_mode = RobotMode.kDamping
            else:
                return_mode = RobotMode.kPrepare
            result = client.ChangeMode(return_mode)
            if result != 0:
                print(f"Return mode request failed with code {result}")
        subscriber.CloseChannel()
        publisher.CloseChannel()
        logger.close()


if __name__ == "__main__":
    main()
