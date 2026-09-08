"""Headless capture for the MPU6050/F9P synchronized serial protocol.

The STM32 PA9 stream is decoded into separate IMU, navigation, and RXM-RAWX
CSV files. Sky-view records are counted but not duplicated into navigation CSV.
"""

from __future__ import annotations

import argparse
import contextlib
import csv
import datetime as dt
import pathlib
import struct
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
RAWX_COLUMNS = [
    "gps_week", "rcv_tow_s", "rx_timer_us", "leap_s", "rec_stat",
    "epoch_num_meas", "epoch_total_meas", "gnss_id", "sv_id", "sig_id",
    "freq_id", "signal", "frequency_mhz", "pseudorange_m",
    "carrier_phase_cycles", "doppler_hz", "locktime_ms", "cno_dbhz",
    "pr_stdev_m", "cp_stdev_cycles", "do_stdev_hz", "pr_valid",
    "cp_valid", "half_cycle", "sub_half_cycle",
]
LOG_TYPES = ("imu", "gnss", "rawx")

SIGNALS = {
    (0, 0): ("GPS_L1CA", 1575.42), (0, 3): ("GPS_L2CL", 1227.60),
    (0, 4): ("GPS_L2CM", 1227.60), (1, 0): ("SBAS_L1CA", 1575.42),
    (2, 0): ("GAL_E1C", 1575.42), (2, 1): ("GAL_E1B", 1575.42),
    (2, 5): ("GAL_E5bI", 1207.14), (2, 6): ("GAL_E5bQ", 1207.14),
    (3, 0): ("BDS_B1I_D1", 1561.098), (3, 1): ("BDS_B1I_D2", 1561.098),
    (3, 2): ("BDS_B2I_D1", 1207.14), (3, 3): ("BDS_B2I_D2", 1207.14),
    (5, 0): ("QZSS_L1CA", 1575.42), (5, 4): ("QZSS_L2CM", 1227.60),
    (5, 5): ("QZSS_L2CL", 1227.60),
}


def signal_name_frequency(gnss_id: int, sig_id: int,
                          freq_id: int) -> tuple[str, float | str]:
    if gnss_id == 6 and sig_id in (0, 2):
        channel = freq_id - 7
        if sig_id == 0:
            return "GLO_L1OF", 1602.0 + channel * 0.5625
        return "GLO_L2OF", 1246.0 + channel * 0.4375
    return SIGNALS.get((gnss_id, sig_id),
                       (f"GNSS{gnss_id}_SIG{sig_id}", ""))


def _float_from_hex(value: str, size: int) -> float:
    bits = int(value, 16)
    return struct.unpack("<d" if size == 8 else "<f",
                         bits.to_bytes(size, "little"))[0]


def parse_rawx_header(parts: list[str]) -> dict[str, int | float] | None:
    if len(parts) != 8 or parts[0] != "RAWX":
        return None
    try:
        return {
            "gps_week": int(parts[1]), "rcv_tow_s": _float_from_hex(parts[2], 8),
            "leap_s": int(parts[3]), "rec_stat": int(parts[4]),
            "num_meas": int(parts[5]), "total_meas": int(parts[6]),
            "rx_timer_us": int(parts[7]),
        }
    except (ValueError, OverflowError, struct.error):
        return None


def parse_rawx_measurement(parts: list[str], epoch: dict[str, int | float] | None
                           ) -> list[int | float | str] | None:
    if epoch is None or len(parts) != 14 or parts[0] != "RAWX_MEAS":
        return None
    try:
        gnss_id, sv_id, sig_id, freq_id = (int(v) for v in parts[1:5])
        pr = _float_from_hex(parts[5], 8); cp = _float_from_hex(parts[6], 8)
        doppler = _float_from_hex(parts[7], 4)
        lock_ms, cno, pr_std, cp_std, do_std, trk = (int(v) for v in parts[8:])
    except (ValueError, OverflowError, struct.error):
        return None
    signal, frequency = signal_name_frequency(gnss_id, sig_id, freq_id)
    cp_sigma: float | str = "" if cp_std == 15 else cp_std * 0.004
    return [
        epoch["gps_week"], epoch["rcv_tow_s"], epoch["rx_timer_us"],
        epoch["leap_s"], epoch["rec_stat"], epoch["num_meas"],
        epoch["total_meas"], gnss_id, sv_id, sig_id, freq_id, signal, frequency,
        pr, cp, doppler, lock_ms, cno, 0.01 * (2 ** pr_std), cp_sigma,
        0.002 * (2 ** do_std), trk & 1, (trk >> 1) & 1,
        (trk >> 2) & 1, (trk >> 3) & 1,
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


def create_session_directory(parent: pathlib.Path, stamp: str) -> pathlib.Path:
    parent.mkdir(parents=True, exist_ok=True)
    for index in range(1000):
        name = stamp if index == 0 else f"{stamp}_{index:02d}"
        directory = parent / name
        try:
            directory.mkdir()
            return directory
        except FileExistsError:
            continue
    raise OSError(f"cannot create a unique session directory for {stamp}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port", help="STM32 PA9 USB-TTL port, for example COM7")
    parser.add_argument("--baud", type=int, default=460800)
    parser.add_argument("--hours", type=float, default=0.0,
                        help="Stop after this many hours; 0 means until Ctrl+C")
    parser.add_argument("--output-dir", type=pathlib.Path, default=DATA_DIR / "decoded")
    parser.add_argument("--save", nargs="*", choices=LOG_TYPES,
                        default=list(LOG_TYPES), metavar="TYPE",
                        help="CSV types to save: imu gnss rawx; default: all")
    parser.add_argument("--raw-output", type=pathlib.Path,
                        help="Optional file containing the complete unmodified serial stream")
    args = parser.parse_args()

    selected = set(args.save)
    stamp = dt.datetime.now().strftime("%Y%m%d%H%M%S")
    session_dir = create_session_directory(args.output_dir, stamp) if selected else None
    paths = {
        "imu": session_dir / "imu.csv" if session_dir and "imu" in selected else None,
        "gnss": session_dir / "gnss.csv" if session_dir and "gnss" in selected else None,
        "rawx": session_dir / "rawx.csv" if session_dir and "rawx" in selected else None,
    }
    if args.raw_output is not None:
        args.raw_output.parent.mkdir(parents=True, exist_ok=True)

    deadline = time.monotonic() + args.hours * 3600.0 if args.hours > 0 else None
    imu_rows = gnss_rows = rawx_rows = sat_rows = invalid = lost = 0
    rawx_epoch = None; rawx_seen_header = False
    first_timer_us = last_timer_us = previous_sample = None
    started = time.monotonic()

    print(f"Capturing {args.port} at {args.baud} baud")
    if session_dir is not None: print(f"Session   -> {session_dir.resolve()}")
    for label, kind in (("IMU CSV ", "imu"), ("GNSS CSV", "gnss"),
                        ("RAWX CSV", "rawx")):
        path = paths[kind]
        print(f"{label} -> {path.resolve() if path else 'disabled'}")
    if args.raw_output:
        print(f"Raw stream -> {args.raw_output.resolve()}")
    print("Press Ctrl+C to stop safely.")

    with contextlib.ExitStack() as stack:
        port = stack.enter_context(serial.Serial(args.port, args.baud, timeout=1))
        streams: dict[str, object] = {}
        writers: dict[str, csv.writer] = {}
        columns = {"imu": IMU_COLUMNS, "gnss": GNSS_COLUMNS, "rawx": RAWX_COLUMNS}
        for kind, path in paths.items():
            if path is None: continue
            stream = stack.enter_context(path.open("w", encoding="utf-8", newline=""))
            writer = csv.writer(stream, lineterminator="\n")
            writer.writerow(columns[kind]); streams[kind] = stream; writers[kind] = writer
        raw_stream = (stack.enter_context(args.raw_output.open("w", encoding="ascii", newline=""))
                      if args.raw_output else None)
        rows_since_flush = 0
        port.reset_input_buffer()
        # Discard the first fragment because opening a continuous stream can
        # begin in the middle of a line.
        port.readline()
        try:
            while deadline is None or time.monotonic() < deadline:
                serial_bytes = port.readline()
                if not serial_bytes:
                    continue
                try:
                    line = serial_bytes.decode("ascii").strip()
                except UnicodeDecodeError:
                    invalid += 1; continue
                if raw_stream is not None:
                    raw_stream.write(line + "\n")
                if not line or line.startswith("#"):
                    continue
                parts = line.split(",")
                wrote_row = False
                if parts[0] == "IMU":
                    parsed = parse_imu(parts, first_timer_us, last_timer_us)
                    if parsed is None:
                        invalid += 1; continue
                    row, first_timer_us, last_timer_us = parsed
                    sample = int(row[0])
                    if previous_sample is not None:
                        delta = sample - previous_sample
                        if delta > 1: lost += delta - 1
                        elif delta <= 0: invalid += 1
                    previous_sample = sample
                    if "imu" in writers: writers["imu"].writerow(row); wrote_row = True
                    imu_rows += 1
                elif parts[0] == "GNSS":
                    row = parse_gnss(parts)
                    if row is None:
                        invalid += 1; continue
                    if "gnss" in writers: writers["gnss"].writerow(row); wrote_row = True
                    gnss_rows += 1
                elif parts[0] == "SAT" and len(parts) == 10:
                    sat_rows += 1
                elif parts[0] == "SAT_END" and len(parts) == 5:
                    sat_rows += 1
                elif parts[0] == "RAWX":
                    rawx_epoch = parse_rawx_header(parts)
                    if rawx_epoch is None: invalid += 1
                    else: rawx_seen_header = True
                elif parts[0] == "RAWX_MEAS":
                    if rawx_epoch is None and not rawx_seen_header:
                        continue
                    row = parse_rawx_measurement(parts, rawx_epoch)
                    if row is None:
                        invalid += 1; continue
                    if "rawx" in writers: writers["rawx"].writerow(row); wrote_row = True
                    rawx_rows += 1
                elif parts[0] == "RAWX_END" and len(parts) == 2:
                    rawx_epoch = None
                else:
                    invalid += 1

                if wrote_row:
                    rows_since_flush += 1
                if rows_since_flush >= 100:
                    for stream in streams.values(): stream.flush()
                    if raw_stream is not None: raw_stream.flush()
                    rows_since_flush = 0
                if imu_rows and imu_rows % 500 == 0:
                    elapsed = time.monotonic() - started
                    print(f"IMU {imu_rows:,}, GNSS {gnss_rows:,}, lost {lost:,}, "
                          f"invalid {invalid:,}, {elapsed / 60:.1f} min", end="\r")
        except KeyboardInterrupt:
            pass
        finally:
            for stream in streams.values(): stream.flush()
            if raw_stream is not None: raw_stream.flush()

    elapsed = time.monotonic() - started
    print(f"\nSaved IMU {imu_rows:,}, GNSS {gnss_rows:,}, RAWX {rawx_rows:,}, satellite records {sat_rows:,} "
          f"({elapsed / 3600.0:.3f} h)")
    print(f"Lost IMU frames: {lost:,}; invalid lines: {invalid:,}")


if __name__ == "__main__":
    main()
