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
Reached 200 logged datapoints; stopped logging to /home/master/t1_imu_logs/t1_imu_YYYYMMDD_HHMMSS.csv
```

`rpy` means roll, pitch, yaw in radians. `acc_z` near `9.8` while standing
still is normal because it includes gravity.

By default, each CSV file records 200 IMU datapoints from the start of logging
and then closes automatically.

Useful options:

```bash
python3 example/low_level/t1_imu_subscriber.py --print-period 1.0
python3 example/low_level/t1_imu_subscriber.py --no-log
python3 example/low_level/t1_imu_subscriber.py --log ~/my_t1_log.csv
python3 example/low_level/t1_imu_subscriber.py --max-log-samples 0
```

Find the newest log on the robot:

```bash
ssh master@<booster-host> 'ls -t ~/t1_imu_logs/*.csv | head -1'
```

Copy all IMU logs back to the laptop:

```bash
mkdir -p /home/furustm/booster_robotics_sdk/t1_imu_logs
scp master@<booster-host>:~/t1_imu_logs/*.csv /home/furustm/booster_robotics_sdk/t1_imu_logs/
```

## 5. Run The Control Example

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

## 6. Local Development Notes

Compile-check the T1 IMU logger locally:

```bash
python3 -m py_compile example/low_level/t1_imu_subscriber.py
```

Only copy files to the robot after local syntax checks pass.
