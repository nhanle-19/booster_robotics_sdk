import argparse
import csv
import os
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", str(Path("/tmp") / "matplotlib"))

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


TIME_COL = 0
GYRO_ROLL_COL = 4
GYRO_PITCH_COL = 5
GYRO_YAW_COL = 6
ACC_X_COL = 7
ACC_Y_COL = 8
ACC_Z_COL = 9
ACC_HORIZONTAL_Y_LIMITS = (-1.0, 1.0)
ACC_Z_Y_LIMITS = (9.0, 11.3)
GYRO_Y_LIMITS = (-0.01, 0.01)

PLOTS = [
    {
        "column": ACC_X_COL,
        "filename": "acceleration_x.png",
        "title": "Acceleration X",
        "ylabel": "acc_x (m/s^2)",
        "limits": ACC_HORIZONTAL_Y_LIMITS,
    },
    {
        "column": ACC_Y_COL,
        "filename": "acceleration_y.png",
        "title": "Acceleration Y",
        "ylabel": "acc_y (m/s^2)",
        "limits": ACC_HORIZONTAL_Y_LIMITS,
    },
    {
        "column": ACC_Z_COL,
        "filename": "acceleration_z.png",
        "title": "Acceleration Z",
        "ylabel": "acc_z (m/s^2)",
        "limits": ACC_Z_Y_LIMITS,
    },
    {
        "column": GYRO_ROLL_COL,
        "filename": "gyro_roll_angular_velocity.png",
        "title": "Gyro Roll Angular Velocity",
        "ylabel": "gyro_x (rad/s)",
        "limits": GYRO_Y_LIMITS,
    },
    {
        "column": GYRO_PITCH_COL,
        "filename": "gyro_pitch_angular_velocity.png",
        "title": "Gyro Pitch Angular Velocity",
        "ylabel": "gyro_y (rad/s)",
        "limits": GYRO_Y_LIMITS,
    },
    {
        "column": GYRO_YAW_COL,
        "filename": "gyro_yaw_angular_velocity.png",
        "title": "Gyro Yaw Angular Velocity",
        "ylabel": "gyro_z (rad/s)",
        "limits": GYRO_Y_LIMITS,
    },
]


def read_imu_csv(path, value_column):
    times = []
    values = []

    with path.open(newline="") as csv_file:
        reader = csv.reader(csv_file)
        next(reader, None)

        for row in reader:
            if len(row) <= value_column:
                continue
            times.append(float(row[TIME_COL]))
            values.append(float(row[value_column]))

    if times:
        start_time = times[0]
        times = [time_value - start_time for time_value in times]

    return times, values


def plot_series(log_files, output_path, title, y_label, value_column, y_limits=None):
    plt.figure(figsize=(12, 6))

    for log_file in log_files:
        times, values = read_imu_csv(log_file, value_column)
        if not times:
            print(f"Skipping empty log: {log_file}")
            continue

        plt.plot(times, values, linewidth=1.2, label=log_file.stem)

    plt.title(title)
    plt.xlabel("Time since log start (s)")
    plt.ylabel(y_label)
    if y_limits is not None:
        plt.ylim(*y_limits)
    plt.grid(True, alpha=0.3)
    plt.legend(fontsize="small")
    plt.tight_layout()
    plt.savefig(output_path, dpi=160)
    plt.close()
    print(f"Saved {output_path}")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Plot T1 IMU CSV logs from a folder into PNG files."
    )
    parser.add_argument(
        "--logs-dir",
        default="logs",
        help="Folder containing IMU CSV logs. Default: logs",
    )
    parser.add_argument(
        "--plot-dir",
        default="plot",
        help="Folder where PNG plots will be saved. Default: plot",
    )
    parser.add_argument(
        "--autoscale",
        action="store_true",
        help="Autoscale plot y-axes instead of using fixed IMU ranges.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    logs_dir = Path(args.logs_dir).expanduser()
    plot_dir = Path(args.plot_dir).expanduser()
    plot_dir.mkdir(parents=True, exist_ok=True)

    log_files = sorted(logs_dir.glob("*.csv"))
    if not log_files:
        raise SystemExit(f"No CSV logs found in {logs_dir}")

    for plot_config in PLOTS:
        plot_series(
            log_files,
            plot_dir / plot_config["filename"],
            plot_config["title"],
            plot_config["ylabel"],
            plot_config["column"],
            None if args.autoscale else plot_config["limits"],
        )


if __name__ == "__main__":
    main()
