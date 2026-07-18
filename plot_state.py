import argparse
import csv
import os
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", str(Path("/tmp") / "matplotlib"))

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


PLOTS = [
    {
        "column": "vx_mps",
        "filename": "velocity_x.png",
        "title": "Commanded Forward Velocity",
        "ylabel": "vx (m/s)",
    },
    {
        "column": "vy_mps",
        "filename": "velocity_y.png",
        "title": "Commanded Lateral Velocity",
        "ylabel": "vy (m/s)",
    },
    {
        "column": "vyaw_radps",
        "filename": "yaw_velocity.png",
        "title": "Commanded Yaw Velocity",
        "ylabel": "vyaw (rad/s)",
    },
]


def read_state_csv(path, value_column):
    times = []
    values = []

    with path.open(newline="") as csv_file:
        reader = csv.DictReader(csv_file)
        if reader.fieldnames is None:
            return times, values
        required = {"time_sec", value_column}
        if not required.issubset(reader.fieldnames):
            missing = ", ".join(sorted(required.difference(reader.fieldnames)))
            print(f"Skipping {path}: missing {missing}")
            return times, values

        for row in reader:
            try:
                times.append(float(row["time_sec"]))
                values.append(float(row[value_column]))
            except (TypeError, ValueError):
                continue

    if times:
        start_time = times[0]
        times = [time_value - start_time for time_value in times]

    return times, values


def plot_series(log_files, output_path, title, y_label, value_column):
    plt.figure(figsize=(12, 6))
    plotted = False

    for log_file in log_files:
        times, values = read_state_csv(log_file, value_column)
        if not times:
            print(f"Skipping empty log: {log_file}")
            continue

        plt.plot(times, values, linewidth=1.2, label=log_file.stem)
        plotted = True

    if not plotted:
        plt.close()
        print(f"No data for {title}")
        return

    plt.title(title)
    plt.xlabel("Time since log start (s)")
    plt.ylabel(y_label)
    plt.grid(True, alpha=0.3)
    plt.legend(fontsize="small")
    plt.tight_layout()
    plt.savefig(output_path, dpi=160)
    plt.close()
    print(f"Saved {output_path}")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Plot commanded velocity CSV logs from a state_log folder."
    )
    parser.add_argument(
        "--logs-dir",
        default="state_log",
        help="Folder containing b1_loco_command CSV logs. Default: state_log",
    )
    parser.add_argument(
        "--plot-dir",
        default="plot_state",
        help="Folder where PNG plots will be saved. Default: plot_state",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    logs_dir = Path(args.logs_dir).expanduser()
    plot_dir = Path(args.plot_dir).expanduser()
    plot_dir.mkdir(parents=True, exist_ok=True)

    log_files = sorted(logs_dir.glob("b1_loco_command_*.csv"))
    if not log_files:
        raise SystemExit(
            f"No b1_loco_command_*.csv logs found in {logs_dir}. "
            "Copy command logs from the robot's ~/booster_robotics_sdk/state_log/ folder first."
        )

    for plot_config in PLOTS:
        plot_series(
            log_files,
            plot_dir / plot_config["filename"],
            plot_config["title"],
            plot_config["ylabel"],
            plot_config["column"],
        )


if __name__ == "__main__":
    main()
