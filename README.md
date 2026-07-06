# Booster Robotics SDK - T1 Quickstart

This fork is set up around the basic workflow needed to reach and test a
Booster T1 from a laptop, then run SDK examples on the robot motion board.

## 1. Connect To The T1

Connect to the same network as the robot, then SSH to the motion board:

```bash
ssh master@192.168.50.159
```

If that IP changes, scan your local subnet from the laptop and try the SSH
hosts that respond:

```bash
ip -br addr
for i in $(seq 1 254); do ping -c 1 -W 1 192.168.50.$i >/dev/null && echo 192.168.50.$i; done
ssh master@192.168.50.X
```

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
scp /home/furustm/booster_robotics_sdk/example/low_level/t1_imu_subscriber.py master@192.168.50.159:~/booster_robotics_sdk/example/low_level/
```

Run it on the robot:

```bash
ssh master@192.168.50.159
cd ~/booster_robotics_sdk
python3 example/low_level/t1_imu_subscriber.py
```

Expected output:

```text
Logging IMU data to /home/master/t1_imu_logs/t1_imu_YYYYMMDD_HHMMSS.csv
Listening for IMU data on rt/low_state
rpy: ...
```

`rpy` means roll, pitch, yaw in radians. `acc_z` near `9.8` while standing
still is normal because it includes gravity.

Useful options:

```bash
python3 example/low_level/t1_imu_subscriber.py --print-period 1.0
python3 example/low_level/t1_imu_subscriber.py --no-log
python3 example/low_level/t1_imu_subscriber.py --log ~/my_t1_log.csv
```

Copy logs back to the laptop:

```bash
scp master@192.168.50.159:~/t1_imu_logs/*.csv /home/furustm/booster_robotics_sdk/
```

## 5. Run The Control Example

The high-level locomotion client is interactive:

```bash
cd ~/booster_robotics_sdk
ip -br addr
python3 example/high_level/b1_loco_example_client.py <networkInterface>
```

Use the interface name or local IP for the robot network, for example `eth0`,
`wlan0`, or the board's `192.168.x.x` address.

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
