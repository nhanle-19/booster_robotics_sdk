# Booster Robotics SDK - T1 Quickstart

This fork is set up around the basic workflow needed to reach and test a
Booster T1 from a laptop, then run SDK examples on the robot motion board.

## 1. Connect To The T1

Connect to the same network as the robot, then SSH to the motion board:

```bash
ssh master@<booster-host>
```

Use the robot hostname, mDNS name, or a local SSH alias for `<booster-host>`.
Keep robot IP addresses in your local shell history or SSH config, not in this
README.

The useful board should have the SDK repo:

```bash
hostname
ls ~/booster_robotics_sdk
```

## 2. Verify Robot Data

On the robot:

```bash
cd ~/booster_robotics_sdk
python3 -c "import booster_robotics_sdk_python; print('SDK OK')"
python3 example/low_level/low_level_subscriber.py
```

If the subscriber prints IMU or low-state values, DDS communication with the
robot is working.

For ROS 2 checks:

```bash
source /opt/booster/BoosterRos2Interface/install/setup.bash
ros2 topic list
```

Good signs include:

```text
/low_state
/joint_ctrl
```

## 3. Install Or Build The Python SDK

If the Python import fails, install the prebuilt package:

```bash
pip install booster_robotics_sdk_python --user
```

Or build the Python binding from this repo:

```bash
cd ~/booster_robotics_sdk
mkdir -p build
cd build
cmake .. -DBUILD_PYTHON_BINDING=on
make -j$(nproc)
sudo make install
```

## 4. Run The T1 IMU Logger

Copy the local script to the robot from the laptop:

```bash
scp /home/furustm/booster_robotics_sdk/example/low_level/t1_imu_subscriber.py master@<booster-host>:~/booster_robotics_sdk/example/low_level/
```

Run it on the robot:

```bash
ssh master@<booster-host>
cd ~/booster_robotics_sdk
python3 example/low_level/t1_imu_subscriber.py
```

Expected output:

```text
Logging IMU data to /home/master/t1_imu_logs/t1_imu_YYYYMMDD_HHMMSS.csv
Listening for IMU data on rt/low_state
rpy: ...
Reached 10.00s of logged IMU data (... datapoints); stopped logging to /home/master/t1_imu_logs/t1_imu_YYYYMMDD_HHMMSS.csv; exiting
```

`rpy` means roll, pitch, yaw in radians. `acc_z` near `9.8` while standing
still is normal because it includes gravity.

By default, each CSV file records 10 seconds of IMU data from the start of logging
and then the logger exits automatically.

Useful options:

```bash
python3 example/low_level/t1_imu_subscriber.py --print-period 1.0
python3 example/low_level/t1_imu_subscriber.py --no-log
python3 example/low_level/t1_imu_subscriber.py --log ~/my_t1_log.csv
python3 example/low_level/t1_imu_subscriber.py --log-duration 20
python3 example/low_level/t1_imu_subscriber.py --max-log-samples 0
```

Find the newest log on the robot:

```bash
ssh master@<booster-host> 'ls -t ~/t1_imu_logs/*.csv | head -1'
```

Copy all IMU logs back to the laptop:

```bash
mkdir -p /home/furustm/booster_robotics_sdk/logs
scp master@<booster-host>:~/t1_imu_logs/*.csv /home/furustm/booster_robotics_sdk/logs/
```

Plot the retrieved logs:

```bash
cd /home/furustm/booster_robotics_sdk
python3 plot_imu.py
python3 plot_imu.py --average-line-only
```

This saves:

```text
plot/acceleration_x.png
plot/acceleration_y.png
plot/acceleration_z.png
plot/gyro_roll_angular_velocity.png
plot/gyro_pitch_angular_velocity.png
plot/gyro_yaw_angular_velocity.png
```

## 5. Run The IMU Calibration Policy Scaffold

The calibration scaffold observes low-state data at 50 Hz, builds an IMU plus
proprioceptive history window, estimates a baseline IMU bias, and can publish a
constant low-level joint target with custom `kp`/`kd`.

Start in observe-only mode first:

```bash
cd ~/booster_robotics_sdk
python3 example/low_level/imu_calibration_policy.py --max-steps 500
```

When the robot is ready for custom low-level control, explicitly enable
publishing. The script waits for low-state data, asks for ENTER, publishes an
initial measured-position hold command, requests `RobotMode.kCustom`, then ramps
to the configured target:

```bash
python3 example/low_level/imu_calibration_policy.py <networkInterface> --enable-control
```

If you want to switch the robot to Custom mode yourself, use:

```bash
python3 example/low_level/imu_calibration_policy.py <networkInterface> --enable-control --manual-custom-mode
```

The loop defaults to `--control-dt 0.02` and `--history-steps 100`. Default joint
targets/`kp`/`kd` match the T1 IMU calibration task: the lying pose from
`T1_IMU_CALIBRATION_LYING_POSE`, with base gains from
`example/low_level/data/t1_default_kpkd_jointorque.csv` scaled by the task's
stiffness/damping factors. The composed command table is written to
`example/low_level/data/t1_imu_calibration_command.csv`.

Override with either that CSV or a JSON command config:

```bash
python3 example/low_level/imu_calibration_policy.py <networkInterface> \
  --enable-control \
  --config example/low_level/data/t1_imu_calibration_command.csv
```

```json
{
  "cmd_type": "SERIAL",
  "q": [0.0, 0.0],
  "kp": [5.0, 5.0],
  "kd": [0.1, 0.1],
  "dq": 0.0,
  "tau": 0.0,
  "weight": 0.0
}
```

Expand each JSON array to 23 values before using it on the robot.

Safety defaults:

```text
--ramp-time 2.0
--max-joint-velocity 0.5
--max-abs-roll-pitch 2.0
--return-mode prepare
```

During control, type `x` or `stop` and press ENTER to request a software stop
through `RobotMode.kDamping`. This is not a hardware e-stop; it still depends on
the Python process, DDS/network communication, and the robot accepting the mode
request. Keep a physical stop/safety operator available.

## 6. Run The Control Example

The high-level locomotion client is interactive:

```bash
cd ~/booster_robotics_sdk
ip -br addr
python3 example/high_level/b1_loco_example_client.py <networkInterface>
```

Use the interface name or local network address for the robot network, for
example `eth0`, `wlan0`, or the board's address on that network.

Common commands in that prompt include:

```text
mp    prepare mode
md    damping mode
mw    walking mode
stop  stop motion
```

Use a clear test area and keep the robot supported or ready to stop before
sending motion commands.

## 7. Local Development Notes

Compile-check the T1 IMU logger locally:

```bash
python3 -m py_compile example/low_level/t1_imu_subscriber.py
python3 -m py_compile example/low_level/imu_calibration_policy.py
```

Only copy files to the robot after local syntax checks pass.
