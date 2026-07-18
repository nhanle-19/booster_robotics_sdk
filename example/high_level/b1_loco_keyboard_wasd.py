#!/usr/bin/env python3
import csv
from datetime import datetime
from pathlib import Path
import select
import sys
import termios
import time
import tty

from booster_robotics_sdk_python import B1LocoClient, ChannelFactory, RobotMode


STEP = 0.025
YAW_STEP = 0.05
MAX_LINEAR = 0.5
MAX_YAW = 0.8
SEND_HZ = 5.0
LOG_HZ = 100.0


def clamp(value, low, high):
    return max(low, min(high, value))


class RawTerminal:
    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.old_settings = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self

    def __exit__(self, exc_type, exc, tb):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)


def read_key(timeout):
    ready, _, _ = select.select([sys.stdin], [], [], timeout)
    if not ready:
        return None
    return sys.stdin.read(1)


def print_status(vx, vy, vyaw, ret=None):
    suffix = "" if ret is None or ret == 0 else f" ret={ret}"
    print(f"\rvx={vx:+.3f} vy={vy:+.3f} vyaw={vyaw:+.3f}{suffix}      ", end="", flush=True)


class CommandLogger:
    def __init__(self):
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.path = Path("state_log") / f"b1_loco_command_{stamp}.csv"
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.file = self.path.open("w", newline="", buffering=1)
        self.writer = csv.writer(self.file)
        self.writer.writerow(
            ["time_sec", "wall_time_s", "vx_mps", "vy_mps", "vyaw_radps", "last_move_ret"]
        )

    def write(self, start_time, vx, vy, vyaw, ret):
        now = time.monotonic()
        self.writer.writerow(
            [
                f"{now - start_time:.6f}",
                f"{time.time():.6f}",
                f"{vx:.6f}",
                f"{vy:.6f}",
                f"{vyaw:.6f}",
                "" if ret is None else ret,
            ]
        )

    def close(self):
        self.file.close()


def main():
    network_interface = sys.argv[1] if len(sys.argv) > 1 else ""

    if network_interface:
        ChannelFactory.Instance().Init(0, network_interface)
    else:
        ChannelFactory.Instance().Init(0)

    client = B1LocoClient()
    client.Init()

    print("Switching to Prepare, then Walking...")
    ret = client.ChangeMode(RobotMode.kPrepare)
    if ret != 0:
        print(f"ChangeMode(kPrepare) returned {ret}; continuing.")
    time.sleep(1.0)
    ret = client.ChangeMode(RobotMode.kWalking)
    if ret != 0:
        print(f"ChangeMode(kWalking) returned {ret}; continuing.")
    time.sleep(1.0)

    vx = 0.0
    vy = 0.0
    vyaw = 0.0
    period = 1.0 / SEND_HZ
    log_period = 1.0 / LOG_HZ
    next_send = 0.0
    next_log = 0.0
    start_time = time.monotonic()
    last_ret = None
    logger = CommandLogger()

    print(f"Logging commands to {logger.path}")
    print("Controls: w/s vx, a/d vy, q/e yaw, space/x stop, Esc exits.")
    print_status(vx, vy, vyaw)

    try:
        with RawTerminal():
            while True:
                now = time.monotonic()
                next_tick = min(next_send, next_log)
                key = read_key(max(0.0, next_tick - now))
                changed = False

                if key in ("\x1b", "\x03"):
                    break
                if key == "w":
                    vx = clamp(round(vx + STEP, 6), -MAX_LINEAR, MAX_LINEAR)
                    changed = True
                elif key == "s":
                    vx = clamp(round(vx - STEP, 6), -MAX_LINEAR, MAX_LINEAR)
                    changed = True
                elif key == "a":
                    vy = clamp(round(vy + STEP, 6), -MAX_LINEAR, MAX_LINEAR)
                    changed = True
                elif key == "d":
                    vy = clamp(round(vy - STEP, 6), -MAX_LINEAR, MAX_LINEAR)
                    changed = True
                elif key == "q":
                    vyaw = clamp(round(vyaw + YAW_STEP, 6), -MAX_YAW, MAX_YAW)
                    changed = True
                elif key == "e":
                    vyaw = clamp(round(vyaw - YAW_STEP, 6), -MAX_YAW, MAX_YAW)
                    changed = True
                elif key in (" ", "x"):
                    vx = 0.0
                    vy = 0.0
                    vyaw = 0.0
                    changed = True

                now = time.monotonic()
                if changed or now >= next_send:
                    last_ret = client.Move(vx, vy, vyaw)
                    print_status(vx, vy, vyaw, last_ret)
                    next_send = now + period
                if now >= next_log:
                    logger.write(start_time, vx, vy, vyaw, last_ret)
                    next_log = now + log_period
    finally:
        print("\nStopping...")
        try:
            client.Move(0.0, 0.0, 0.0)
        except KeyboardInterrupt:
            pass
        logger.write(start_time, 0.0, 0.0, 0.0, last_ret)
        logger.close()
        print(f"Command log saved to {logger.path}")
        print("Stopped.")


if __name__ == "__main__":
    main()
