"""Decode MPU6050/F9P IMU records into the canonical physical-unit CSV.

Accepted inputs are the protocol-v3 typed ``IMU,...`` serial stream and its
canonical IMU CSV. Processing is streaming so long recordings do not need to
fit in memory.
"""

from __future__ import annotations

import argparse
import csv
import pathlib

import matplotlib.pyplot as plt
import numpy as np

TOOLS_DIR = pathlib.Path(__file__).resolve().parent
DATA_DIR = TOOLS_DIR.parent / "data"
G0 = 9.80665
ACCEL_SCALE = G0 / 16384.0
GYRO_SCALE = 3600.0 / 131.0
RAW_NAMES = ("ax_raw", "ay_raw", "az_raw", "temp_raw", "gx_raw", "gy_raw", "gz_raw")
OUTPUT_COLUMNS = [
    "sample", "gps_week", "gps_tow_us", "time_valid", "timer_us", "time_s", "dt_s",
    *RAW_NAMES, "ax_m_s2", "ay_m_s2", "az_m_s2", "temp_deg_c",
    "gx_deg_h", "gy_deg_h", "gz_deg_h",
]


class BlockMeanCollector:
    def __init__(self, block_rows: int) -> None:
        self.block_rows = block_rows
        self.pending: list[list[float]] = []
        self.blocks: list[np.ndarray] = []

    def add(self, row: list[float]) -> None:
        self.pending.append(row)
        if len(self.pending) >= self.block_rows:
            self.blocks.append(np.mean(np.asarray(self.pending), axis=0))
            self.pending.clear()

    def finish(self) -> np.ndarray:
        if self.pending:
            self.blocks.append(np.mean(np.asarray(self.pending), axis=0))
            self.pending.clear()
        return np.vstack(self.blocks) if self.blocks else np.empty((0, 8))


def parse_typed(row: list[str]) -> dict[str, int] | None:
    if len(row) != 13 or row[0] != "IMU":
        return None
    try:
        values = [int(value.strip()) for value in row[1:]]
    except ValueError:
        return None
    names = ("sample", "gps_week", "gps_tow_us", "time_valid", "timer_us", *RAW_NAMES)
    return dict(zip(names, values))


def parse_named(row: list[str], header: list[str]) -> dict[str, int] | None:
    if len(row) != len(header):
        return None
    fields = dict(zip(header, row))
    required = {"sample", "gps_week", "gps_tow_us", "time_valid", "timer_us", *RAW_NAMES}
    if not required.issubset(fields):
        return None
    try:
        return {name: int(float(fields[name])) for name in required}
    except ValueError:
        return None


def decoded_row(sample: dict[str, int], first_timer_us: int,
                previous_timer_us: int | None) -> tuple[list[int | float], list[float]]:
    timer_us = sample["timer_us"]
    dt_s = (timer_us - previous_timer_us) / 1e6 if previous_timer_us is not None else 0.0
    time_s = (timer_us - first_timer_us) / 1e6
    ax, ay, az, temp, gx, gy, gz = (sample[name] for name in RAW_NAMES)
    physical = [ax * ACCEL_SCALE, ay * ACCEL_SCALE, az * ACCEL_SCALE,
                temp / 340.0 + 36.53,
                gx * GYRO_SCALE, gy * GYRO_SCALE, gz * GYRO_SCALE]
    row: list[int | float] = [
        sample["sample"], sample["gps_week"], sample["gps_tow_us"], sample["time_valid"],
        timer_us, time_s, dt_s, ax, ay, az, temp, gx, gy, gz, *physical,
    ]
    plot_row = [time_s, physical[0], physical[1], physical[2],
                physical[4], physical[5], physical[6], physical[3]]
    return row, plot_row


def plot_channels(path: pathlib.Path, values: np.ndarray, block_seconds: float) -> None:
    if values.size == 0:
        raise ValueError("No decoded samples available for plotting")
    hours = (values[:, 0] - values[0, 0]) / 3600.0
    channels = [
        (1, "Acceleration X", "m/s²", "#1f77b4"),
        (2, "Acceleration Y", "m/s²", "#ff7f0e"),
        (3, "Acceleration Z", "m/s²", "#2ca02c"),
        (4, "Gyroscope X", "deg/h", "#d62728"),
        (5, "Gyroscope Y", "deg/h", "#9467bd"),
        (6, "Gyroscope Z", "deg/h", "#8c564b"),
        (7, "Temperature", "°C", "#e41a1c"),
    ]
    figure, axes = plt.subplots(7, 1, figsize=(15, 17), sharex=True, constrained_layout=True)
    for axis, (column, title, unit, color) in zip(axes, channels):
        axis.plot(hours, values[:, column], color=color, linewidth=0.9)
        axis.set_ylabel(unit); axis.set_title(title, loc="left", fontsize=10); axis.grid(True, alpha=0.30)
    axes[-1].set_xlabel("Time after recording start (h)")
    figure.suptitle(f"MPU6050/F9P IMU data ({block_seconds:g} s block means)", fontsize=15)
    figure.savefig(path, dpi=180); plt.close(figure)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=pathlib.Path, help="Raw serial log or IMU CSV")
    parser.add_argument("--output-csv", type=pathlib.Path, help="Canonical decoded CSV path")
    parser.add_argument("--plot", type=pathlib.Path, help="Seven-channel PNG path")
    parser.add_argument("--rate", type=float, default=100.0)
    parser.add_argument("--plot-block-seconds", type=float, default=1.0)
    args = parser.parse_args()
    if not args.csv.exists():
        raise SystemExit(f"Input file not found: {args.csv}")
    if args.rate <= 0 or args.plot_block_seconds <= 0:
        raise SystemExit("--rate and --plot-block-seconds must be positive")

    output_dir = DATA_DIR / "decoded"
    output_csv = args.output_csv or output_dir / f"{args.csv.stem}_physical.csv"
    plot_path = args.plot or output_dir / f"{args.csv.stem}_7channel.png"
    output_csv.parent.mkdir(parents=True, exist_ok=True); plot_path.parent.mkdir(parents=True, exist_ok=True)
    if output_csv.resolve() == args.csv.resolve():
        raise SystemExit("Input and output CSV paths must be different")

    collector = BlockMeanCollector(max(1, round(args.rate * args.plot_block_seconds)))
    header: list[str] | None = None
    first_timer_us = previous_timer_us = None
    first_sample = last_sample = total = invalid = 0

    with args.csv.open("r", encoding="utf-8-sig", errors="replace", newline="") as source, \
            output_csv.open("w", encoding="utf-8", newline="") as target:
        writer = csv.writer(target, lineterminator="\n"); writer.writerow(OUTPUT_COLUMNS)
        for source_row in csv.reader(source):
            if not source_row or source_row[0].strip().startswith("#"):
                continue
            row = [item.strip() for item in source_row]
            if row[0] == "sample":
                header = row; continue
            sample = parse_typed(row) if row[0] == "IMU" else (parse_named(row, header) if header else None)
            if sample is None:
                invalid += 1; continue
            if first_timer_us is None:
                first_timer_us = sample["timer_us"]; first_sample = sample["sample"]
            output, plot_row = decoded_row(sample, first_timer_us, previous_timer_us)
            previous_timer_us = sample["timer_us"]; last_sample = sample["sample"]
            writer.writerow(output); collector.add(plot_row); total += 1
            if total % 250000 == 0:
                target.flush(); print(f"Decoded {total:,} rows", flush=True)

    if total == 0:
        raise SystemExit("No valid IMU rows found")
    plot_channels(plot_path, collector.finish(), args.plot_block_seconds)
    expected = last_sample - first_sample + 1
    print(f"Decoded rows: {total:,}; sequence difference: {expected - total:,}; invalid input rows: {invalid:,}")
    print(f"CSV: {output_csv.resolve()}"); print(f"Plot: {plot_path.resolve()}")


if __name__ == "__main__":
    main()
