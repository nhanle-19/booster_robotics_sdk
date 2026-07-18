import argparse
import csv
import math
import os
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", str(Path("/tmp") / "matplotlib"))

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


TIME_COL = 0
TIME_FIELD = "time_sec"
ROLL_FIELD = "roll_rad"
PITCH_FIELD = "pitch_rad"
YAW_FIELD = "yaw_rad"
ACC_X_FIELD = "acc_x"
ACC_Y_FIELD = "acc_y"
ACC_Z_FIELD = "acc_z"
GYRO_ROLL_COL = 4
GYRO_PITCH_COL = 5
GYRO_YAW_COL = 6
ACC_X_COL = 7
ACC_Y_COL = 8
ACC_Z_COL = 9
ACC_HORIZONTAL_Y_LIMITS = (-1.0, 1.0)
ACC_Z_Y_LIMITS = (9.0, 11.3)
GYRO_Y_LIMITS = (-0.01, 0.01)
WORLD_ANGLE_Y_LIMITS = (-3.14159, 3.14159)

WORLD_ANGLE_PLOTS = [
    {
        "column": ROLL_FIELD,
        "filename": "world_roll_angle.png",
        "title": "World Roll Angle",
        "ylabel": "roll (rad)",
        "limits": WORLD_ANGLE_Y_LIMITS,
    },
    {
        "column": PITCH_FIELD,
        "filename": "world_pitch_angle.png",
        "title": "World Pitch Angle",
        "ylabel": "pitch (rad)",
        "limits": WORLD_ANGLE_Y_LIMITS,
    },
    {
        "column": YAW_FIELD,
        "filename": "world_yaw_angle.png",
        "title": "World Yaw Angle",
        "ylabel": "yaw (rad)",
        "limits": WORLD_ANGLE_Y_LIMITS,
    },
]

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


def parse_color_groups(color_group_args):
    groups = []
    for group_arg in color_group_args:
        for group in group_arg.split(","):
            group = group.strip()
            if group:
                groups.append(group)
    return groups


def match_color_group(title, color_groups):
    title_lower = title.lower()
    return next(
        (
            group
            for group in color_groups
            if group.lower() in title_lower
        ),
        None,
    )


def build_log_colors(log_files, color_groups):
    colors = {}
    group_colors = {}
    next_color_index = 0

    for log_file in log_files:
        matched_group = match_color_group(log_file.stem, color_groups)

        if matched_group is not None:
            if matched_group not in group_colors:
                group_colors[matched_group] = f"C{next_color_index % 10}"
                next_color_index += 1
            colors[log_file] = group_colors[matched_group]
        else:
            colors[log_file] = f"C{next_color_index % 10}"
            next_color_index += 1

    return colors


def build_fill_logs(log_files, color_groups):
    return {
        log_file
        for log_file in log_files
        if match_color_group(log_file.stem, color_groups) is not None
    }


def append_missing_groups(color_groups, fill_groups):
    all_groups = list(color_groups)
    seen_groups = {group.lower() for group in all_groups}
    for group in fill_groups:
        group_key = group.lower()
        if group_key not in seen_groups:
            all_groups.append(group)
            seen_groups.add(group_key)
    return all_groups


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


def read_imu_csv_field(path, value_field):
    times = []
    values = []

    with path.open(newline="") as csv_file:
        reader = csv.DictReader(csv_file)
        if reader.fieldnames is None:
            print(f"Skipping {path}: missing CSV header")
            return times, values

        required_fields = {TIME_FIELD, value_field}
        if not required_fields.issubset(reader.fieldnames):
            missing = ", ".join(sorted(required_fields.difference(reader.fieldnames)))
            print(f"Skipping {path}: missing {missing}")
            return times, values

        for row in reader:
            try:
                time_value = float(row[TIME_FIELD])
                value = float(row[value_field])
            except (TypeError, ValueError):
                continue

            times.append(time_value)
            values.append(value)

    if times:
        start_time = times[0]
        times = [time_value - start_time for time_value in times]

    return times, values


def read_total_acceleration_magnitudes(path):
    magnitudes = []

    with path.open(newline="") as csv_file:
        reader = csv.DictReader(csv_file)
        if reader.fieldnames is None:
            print(f"Skipping {path}: missing CSV header")
            return magnitudes

        required_fields = {ACC_X_FIELD, ACC_Y_FIELD, ACC_Z_FIELD}
        if not required_fields.issubset(reader.fieldnames):
            missing = ", ".join(sorted(required_fields.difference(reader.fieldnames)))
            print(f"Skipping {path}: missing {missing}")
            return magnitudes

        for row in reader:
            try:
                acc_x = float(row[ACC_X_FIELD])
                acc_y = float(row[ACC_Y_FIELD])
                acc_z = float(row[ACC_Z_FIELD])
            except (TypeError, ValueError):
                continue

            magnitudes.append(math.sqrt(acc_x**2 + acc_y**2 + acc_z**2))

    return magnitudes


def print_total_average_acceleration(log_files):
    total_sum = 0.0
    total_count = 0

    print("Average total acceleration sqrt(acc_x^2 + acc_y^2 + acc_z^2):")

    for log_file in log_files:
        magnitudes = read_total_acceleration_magnitudes(log_file)
        if not magnitudes:
            print(f"  {log_file.stem}: no parseable acceleration data")
            continue

        average_magnitude = sum(magnitudes) / len(magnitudes)
        total_sum += sum(magnitudes)
        total_count += len(magnitudes)
        print(f"  {log_file.stem}: {average_magnitude:.6f} m/s^2")

    if total_count:
        print(f"  overall: {total_sum / total_count:.6f} m/s^2")
    else:
        print("  overall: no parseable acceleration data")


def plot_series(
    log_files,
    output_path,
    title,
    y_label,
    value_column,
    y_limits=None,
    average_line_only=False,
    log_colors=None,
    fill_logs=None,
    fill_alpha=0.12,
):
    plt.figure(figsize=(12, 6))
    fill_segments = []

    for log_file in log_files:
        times, values = read_imu_csv(log_file, value_column)
        if not times:
            print(f"Skipping empty log: {log_file}")
            continue

        color = log_colors.get(log_file) if log_colors is not None else None
        fill_requested = fill_logs is not None and log_file in fill_logs
        if average_line_only:
            mean_value = sum(values) / len(values)
            duration = max(times) if max(times) > 0.0 else 1.0
            plt.hlines(
                mean_value,
                0.0,
                duration,
                colors=color,
                linestyles="--",
                linewidth=1.8,
                label=f"{log_file.stem} avg={mean_value:.5g}",
            )
            if fill_requested:
                fill_segments.append(([0.0, duration], [mean_value, mean_value], color))
        else:
            plt.plot(times, values, linewidth=1.2, label=log_file.stem, color=color)
            if fill_requested:
                fill_segments.append((times, values, color))

    if fill_segments:
        line_y_limits = plt.ylim()
        for fill_times, fill_values, fill_color in fill_segments:
            plt.fill_between(
                fill_times,
                fill_values,
                0.0,
                color=fill_color,
                alpha=fill_alpha,
                linewidth=0,
                zorder=0,
            )
        if y_limits is None:
            plt.ylim(*line_y_limits)

    plt.title(f"{title} Average" if average_line_only else title)
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


def plot_field_series(
    log_files,
    output_path,
    title,
    y_label,
    value_field,
    y_limits=None,
    average_line_only=False,
    log_colors=None,
    fill_logs=None,
    fill_alpha=0.12,
):
    plt.figure(figsize=(12, 6))
    fill_segments = []
    plotted = False

    for log_file in log_files:
        times, values = read_imu_csv_field(log_file, value_field)
        if not times:
            print(f"Skipping log with no parseable {title} data: {log_file}")
            continue

        color = log_colors.get(log_file) if log_colors is not None else None
        fill_requested = fill_logs is not None and log_file in fill_logs
        if average_line_only:
            mean_value = sum(values) / len(values)
            duration = max(times) if max(times) > 0.0 else 1.0
            plt.hlines(
                mean_value,
                0.0,
                duration,
                colors=color,
                linestyles="--",
                linewidth=1.8,
                label=f"{log_file.stem} avg={mean_value:.5g}",
            )
            if fill_requested:
                fill_segments.append(([0.0, duration], [mean_value, mean_value], color))
        else:
            plt.plot(times, values, linewidth=1.2, label=log_file.stem, color=color)
            if fill_requested:
                fill_segments.append((times, values, color))

        plotted = True

    if not plotted:
        plt.close()
        output_path.unlink(missing_ok=True)
        print(f"No data for {title}")
        return

    if fill_segments:
        line_y_limits = plt.ylim()
        for fill_times, fill_values, fill_color in fill_segments:
            plt.fill_between(
                fill_times,
                fill_values,
                0.0,
                color=fill_color,
                alpha=fill_alpha,
                linewidth=0,
                zorder=0,
            )
        if y_limits is None:
            plt.ylim(*line_y_limits)

    plt.title(f"{title} Average" if average_line_only else title)
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
        default="imu_logs",
        help="Folder containing IMU CSV logs. Default: imu_logs",
    )
    parser.add_argument(
        "--plot-dir",
        default="plot_imu",
        help="Folder where PNG plots will be saved. Default: plot_imu",
    )
    parser.add_argument(
        "--autoscale",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Autoscale plot y-axes instead of using fixed IMU ranges. Default: enabled",
    )
    parser.add_argument(
        "--average-line-only",
        action="store_true",
        help="Plot one horizontal average line per CSV file instead of raw samples.",
    )
    parser.add_argument(
        "--color-group",
        action="append",
        default=[],
        metavar="WORD",
        help=(
            "Use the same color for every CSV title containing WORD. "
            "Can be repeated or comma-separated."
        ),
    )
    parser.add_argument(
        "--fill-color-group",
        action="append",
        default=[],
        metavar="WORD",
        help=(
            "Fill only CSV titles containing WORD with a translucent group color. "
            "Can be repeated or comma-separated."
        ),
    )
    parser.add_argument(
        "--fill-alpha",
        type=float,
        default=0.12,
        help="Transparency for --fill-color-group fills. Default: 0.12",
    )
    parser.add_argument(
        "--world-angle-only",
        action="store_true",
        help="Only generate the IMU world-angle plots from roll, pitch, and yaw.",
    )
    args = parser.parse_args()
    if not 0.0 <= args.fill_alpha <= 1.0:
        parser.error("--fill-alpha must be between 0.0 and 1.0")
    return args


def main():
    args = parse_args()
    logs_dir = Path(args.logs_dir).expanduser()
    plot_dir = Path(args.plot_dir).expanduser()
    plot_dir.mkdir(parents=True, exist_ok=True)

    log_files = sorted(logs_dir.glob("*.csv"))
    if not log_files:
        raise SystemExit(f"No CSV logs found in {logs_dir}")

    color_groups = parse_color_groups(args.color_group)
    fill_groups = parse_color_groups(args.fill_color_group)
    log_colors = build_log_colors(log_files, append_missing_groups(color_groups, fill_groups))
    fill_logs = build_fill_logs(log_files, fill_groups) if fill_groups else None

    print_total_average_acceleration(log_files)

    for plot_config in WORLD_ANGLE_PLOTS:
        plot_field_series(
            log_files,
            plot_dir / plot_config["filename"],
            plot_config["title"],
            plot_config["ylabel"],
            plot_config["column"],
            None if args.autoscale else plot_config["limits"],
            args.average_line_only,
            log_colors,
            fill_logs,
            args.fill_alpha,
        )

    if args.world_angle_only:
        return

    for plot_config in PLOTS:
        plot_series(
            log_files,
            plot_dir / plot_config["filename"],
            plot_config["title"],
            plot_config["ylabel"],
            plot_config["column"],
            None if args.autoscale else plot_config["limits"],
            args.average_line_only,
            log_colors,
            fill_logs,
            args.fill_alpha,
        )


if __name__ == "__main__":
    main()
