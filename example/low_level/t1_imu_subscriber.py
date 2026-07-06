import argparse
import csv
from datetime import datetime
from pathlib import Path
import time

from booster_robotics_sdk_python import ChannelFactory, B1LowStateSubscriber


def default_log_path():
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path.home() / "t1_imu_logs" / f"t1_imu_{timestamp}.csv"


class ImuLogger:
    def __init__(self, print_period, log_path):
        self.print_period = print_period
        self.last_print = 0.0
        self.log_file = None
        self.writer = None

        if log_path is not None:
            log_path = Path(log_path).expanduser()
            log_path.parent.mkdir(parents=True, exist_ok=True)
            self.log_file = log_path.open("w", newline="")
            self.writer = csv.writer(self.log_file)
            self.writer.writerow(
                [
                    "time_sec",
                    "roll_rad",
                    "pitch_rad",
                    "yaw_rad",
                    "gyro_x",
                    "gyro_y",
                    "gyro_z",
                    "acc_x",
                    "acc_y",
                    "acc_z",
                ]
            )
            self.log_file.flush()
            print(f"Logging IMU data to {log_path}")

    def close(self):
        if self.log_file is not None:
            self.log_file.close()

    def handle_low_state(self, low_state_msg):
        now = time.time()
        imu = low_state_msg.imu_state
        row = [
            f"{now:.6f}",
            f"{imu.rpy[0]:.9f}",
            f"{imu.rpy[1]:.9f}",
            f"{imu.rpy[2]:.9f}",
            f"{imu.gyro[0]:.9f}",
            f"{imu.gyro[1]:.9f}",
            f"{imu.gyro[2]:.9f}",
            f"{imu.acc[0]:.9f}",
            f"{imu.acc[1]:.9f}",
            f"{imu.acc[2]:.9f}",
        ]

        if self.writer is not None:
            self.writer.writerow(row)

        if now - self.last_print < self.print_period:
            return

        self.last_print = now
        print(
            "rpy: "
            f"{imu.rpy[0]: .6f}, {imu.rpy[1]: .6f}, {imu.rpy[2]: .6f} | "
            "gyro: "
            f"{imu.gyro[0]: .6f}, {imu.gyro[1]: .6f}, {imu.gyro[2]: .6f} | "
            "acc: "
            f"{imu.acc[0]: .6f}, {imu.acc[1]: .6f}, {imu.acc[2]: .6f}",
            flush=True,
        )


def parse_args():
    parser = argparse.ArgumentParser(
        description="Subscribe to T1/B1 low-state IMU data and optionally log it."
    )
    parser.add_argument(
        "network_interface",
        nargs="?",
        help="Optional DDS network interface or local IP address.",
    )
    parser.add_argument(
        "--print-period",
        type=float,
        default=0.2,
        help="Seconds between terminal prints. Default: 0.2.",
    )
    parser.add_argument(
        "--log",
        nargs="?",
        const=str(default_log_path()),
        default=str(default_log_path()),
        help="CSV log path. Default: ~/t1_imu_logs/t1_imu_<timestamp>.csv.",
    )
    parser.add_argument(
        "--no-log",
        action="store_true",
        help="Print only; do not create a CSV log file.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    log_path = None if args.no_log else args.log
    imu_logger = ImuLogger(args.print_period, log_path)

    if args.network_interface:
        ChannelFactory.Instance().Init(0, args.network_interface)
    else:
        ChannelFactory.Instance().Init(0)

    subscriber = B1LowStateSubscriber(imu_logger.handle_low_state)
    subscriber.InitChannel()
    print(f"Listening for IMU data on {subscriber.GetChannelName()}")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        subscriber.CloseChannel()
        imu_logger.close()
        print("\nStopped")


if __name__ == "__main__":
    main()
