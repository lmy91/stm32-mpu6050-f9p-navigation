"""Headless capture for the MPU6050/F9P synchronized serial protocol.

The STM32 PA9 stream is decoded into separate IMU and GNSS CSV files. Satellite
sky-view records are counted but are not duplicated into the navigation CSV.
"""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import pathlib
import time

import serial

TOOLS_DIR = pathlib.Path(__file__).resolve().parent
DATA_DIR = TOOLS_DIR.parent / "data"
G0 = 9.80665
ACCEL_SCALE = G0 / 16384.0
GYRO_SCALE = 3600.0 / 131.0

IMU_COLUMNS = [
    "sample", "gps_week", "gps_tow_us", "time_valid", "timer_us", "time_s", "dt_s",
    "ax_raw", "ay_raw", "az_raw", "temp_raw", "gx_raw", "gy_raw", "gz_raw",
    "ax_m_s2", "ay_m_s2", "az_m_s2", "temp_deg_c", "gx_deg_h", "gy_deg_h", "gz_deg_h",
]
GNSS_COLUMNS = [
    "gps_week", "gps_tow_ms", "time_valid", "rx_timer_us", "fix", "num_sv",
    "flags", "flags2", "carr_soln", "gnss_fix_ok", "diff_soln", "lat_deg", "lon_deg",
    "hmsl_m", "h_acc_m", "v_acc_m", "vel_n_m_s", "vel_e_m_s", "vel_d_m_s",
    "ground_speed_m_s", "s_acc_m_s", "pdop",
]


def parse_imu(parts: list[str], first_timer_us: int | None,
              last_timer_us: int | None) -> tuple[list[int | float], int, int] | None:
    if len(parts) != 13 or parts[0] != "IMU":
        return None
    try:
        sample, week, tow_us, valid, timer_us, ax, ay, az, temp, gx, gy, gz = (
            int(value) for value in parts[1:]
        )
    except ValueError:
        return None
    if first_timer_us is None:
        first_timer_us = timer_us
    time_s = (timer_us - first_timer_us) / 1e6
    dt_s = 0.0 if last_timer_us is None else (timer_us - last_timer_us) / 1e6
    physical = [
        ax * ACCEL_SCALE, ay * ACCEL_SCALE, az * ACCEL_SCALE,
        temp / 340.0 + 36.53,
        gx * GYRO_SCALE, gy * GYRO_SCALE, gz * GYRO_SCALE,
    ]
    row: list[int | float] = [
        sample, week, tow_us, valid, timer_us, time_s, dt_s,
        ax, ay, az, temp, gx, gy, gz, *physical,
    ]
    return row, first_timer_us, timer_us


def parse_gnss(parts: list[str]) -> list[int | float] | None:
    if len(parts) != 21 or parts[0] != "GNSS":
        return None
    try:
        values = [int(value) for value in parts[1:]]
        (week, tow_ms, valid, rx_timer_us, fix, num_sv, flags, flags2, carr_soln,
         lat_e7, lon_e7, hmsl_mm, h_acc_mm, v_acc_mm, vel_n, vel_e, vel_d,
         ground, s_acc_mms, pdop_x100) = values
    except ValueError:
        return None
    gnss_fix_ok = flags & 0x01
    diff_soln = (flags >> 1) & 0x01
    return [
        week, tow_ms, valid, rx_timer_us, fix, num_sv, flags, flags2, carr_soln,
        gnss_fix_ok, diff_soln, lat_e7 / 1e7, lon_e7 / 1e7, hmsl_mm / 1000.0,
        h_acc_mm / 1000.0, v_acc_mm / 1000.0,
        vel_n / 1000.0, vel_e / 1000.0, vel_d / 1000.0,
        ground / 1000.0, s_acc_mms / 1000.0, pdop_x100 / 100.0,
    ]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port", help="STM32 PA9 USB-TTL port, for example COM7")
    parser.add_argument("--baud", type=int, default=460800)
    parser.add_argument("--hours", type=float, default=0.0,
                        help="Stop after this many hours; 0 means until Ctrl+C")
    parser.add_argument("--output-dir", type=pathlib.Path, default=DATA_DIR / "decoded")
    parser.add_argument("--imu-output", type=pathlib.Path, help="IMU CSV path")
    parser.add_argument("--gnss-output", type=pathlib.Path, help="GNSS CSV path")
    parser.add_argument("--raw-output", type=pathlib.Path,
                        help="Optional file containing the complete unmodified serial stream")
    args = parser.parse_args()

    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    imu_path = args.imu_output or args.output_dir / f"imu_gnss_time_{stamp}.csv"
    gnss_path = args.gnss_output or args.output_dir / f"gnss_nav_{stamp}.csv"
    for path in (imu_path, gnss_path, args.raw_output):
        if path is not None:
            path.parent.mkdir(parents=True, exist_ok=True)
    resolved = [path.resolve() for path in (imu_path, gnss_path) if path is not None]
    if len(set(resolved)) != len(resolved):
        raise SystemExit("IMU and GNSS output paths must be different")

    deadline = time.monotonic() + args.hours * 3600.0 if args.hours > 0 else None
    imu_rows = gnss_rows = sat_rows = invalid = lost = 0
    first_timer_us = last_timer_us = previous_sample = None
    started = time.monotonic()
    raw_stream = None

    print(f"Capturing {args.port} at {args.baud} baud")
    print(f"IMU CSV  -> {imu_path.resolve()}")
    print(f"GNSS CSV -> {gnss_path.resolve()}")
    if args.raw_output:
        print(f"Raw stream -> {args.raw_output.resolve()}")
    print("Press Ctrl+C to stop safely.")

    try:
        if args.raw_output:
            raw_stream = args.raw_output.open("w", encoding="ascii", newline="")
        with serial.Serial(args.port, args.baud, timeout=1) as port, \
                imu_path.open("w", encoding="utf-8", newline="") as imu_stream, \
                gnss_path.open("w", encoding="utf-8", newline="") as gnss_stream:
            imu_writer = csv.writer(imu_stream, lineterminator="\n")
            gnss_writer = csv.writer(gnss_stream, lineterminator="\n")
            imu_writer.writerow(IMU_COLUMNS); gnss_writer.writerow(GNSS_COLUMNS)
            port.reset_input_buffer()
            # Discard the first fragment because opening a continuous stream can
            # begin in the middle of a line.
            port.readline()
            try:
                while deadline is None or time.monotonic() < deadline:
                    serial_bytes = port.readline()
                    if not serial_bytes:
                        continue
                    line = serial_bytes.decode("ascii", errors="replace").strip()
                    if raw_stream is not None:
                        raw_stream.write(line + "\n")
                    if not line or line.startswith("#"):
                        continue
                    parts = line.split(",")
                    if parts[0] == "IMU":
                        parsed = parse_imu(parts, first_timer_us, last_timer_us)
                        if parsed is None:
                            invalid += 1; continue
                        row, first_timer_us, last_timer_us = parsed
                        sample = int(row[0])
                        if previous_sample is not None and sample > previous_sample + 1:
                            lost += sample - previous_sample - 1
                        previous_sample = sample
                        imu_writer.writerow(row); imu_rows += 1
                    elif parts[0] == "GNSS":
                        row = parse_gnss(parts)
                        if row is None:
                            invalid += 1; continue
                        gnss_writer.writerow(row); gnss_rows += 1
                    elif parts[0] in {"SAT", "SAT_END"}:
                        sat_rows += 1
                    else:
                        invalid += 1

                    if imu_rows and imu_rows % 500 == 0:
                        imu_stream.flush(); gnss_stream.flush()
                        if raw_stream is not None: raw_stream.flush()
                        elapsed = time.monotonic() - started
                        print(f"IMU {imu_rows:,}, GNSS {gnss_rows:,}, lost {lost:,}, "
                              f"invalid {invalid:,}, {elapsed / 60:.1f} min", end="\r")
            except KeyboardInterrupt:
                pass
            finally:
                imu_stream.flush(); gnss_stream.flush()
                if raw_stream is not None: raw_stream.flush()
    finally:
        if raw_stream is not None:
            raw_stream.close()

    elapsed = time.monotonic() - started
    print(f"\nSaved IMU {imu_rows:,}, GNSS {gnss_rows:,}, satellite records {sat_rows:,} "
          f"({elapsed / 3600.0:.3f} h)")
    print(f"Lost IMU frames: {lost:,}; invalid lines: {invalid:,}")


if __name__ == "__main__":
    main()
