"""Headless capture for the MPU6050/F9P synchronized serial protocol.

The STM32 PA9 stream is decoded into separate IMU, navigation, and RXM-RAWX
CSV files. On Raspberry Pi, a second receive-only UART can simultaneously save
the original ZED-F9P byte stream as f9p.ubx for later RINEX conversion.
"""

from __future__ import annotations

import argparse
import contextlib
import csv
import datetime as dt
import json
import pathlib
import struct
import sys
import threading
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

# Observability counters emitted once per PPS in the "# sync" line. The four
# "d_" columns are unsigned-32 deltas vs. the previous # sync (first row = 0);
# backlog = interrupt_count - sample_count (in-flight captures not yet emitted).
SYNC_COLUMNS = [
    "unix_ms", "pps",
    "sample_count", "interrupt_count", "interrupt_overruns",
    "cc2_overcapture", "dt_gap_count", "i2c_errors",
    "d_interrupt_overruns", "d_cc2_overcapture", "d_dt_gap_count", "d_i2c_errors",
    "backlog",
]
SYNC_COUNTERS = ("sample_count", "interrupt_count", "interrupt_overruns",
                 "cc2_overcapture", "dt_gap_count", "i2c_errors")

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


def parse_satellite(parts: list[str]) -> dict[str, int] | None:
    """Parse one protocol-v3 NAV-SAT row for the live sky plot."""
    if len(parts) != 10 or parts[0] != "SAT":
        return None
    try:
        week, tow_ms, valid, gnss_id, sv_id, cno, elev, azim, used = (
            int(value) for value in parts[1:])
    except ValueError:
        return None
    if (not 0 <= week <= 65535 or not 0 <= tow_ms <= 604_800_000 or
            valid not in (0, 1) or not 0 <= gnss_id <= 15 or
            not 0 <= sv_id <= 255 or not 0 <= cno <= 255 or
            not -90 <= elev <= 90 or not -360 <= azim <= 360 or used not in (0, 1)):
        return None
    return {"gps_week": week, "gps_tow_ms": tow_ms, "time_valid": valid,
            "gnss_id": gnss_id, "sv_id": sv_id, "cno_dbhz": cno,
            "elev_deg": elev, "azim_deg": azim % 360, "used": used}


def parse_satellite_end(parts: list[str]) -> tuple[int, int, int, int] | None:
    if len(parts) != 5 or parts[0] != "SAT_END":
        return None
    try:
        week, tow_ms, valid, count = (int(value) for value in parts[1:])
    except ValueError:
        return None
    if (not 0 <= week <= 65535 or not 0 <= tow_ms <= 604_800_000 or
            valid not in (0, 1) or not 0 <= count <= 255):
        return None
    return week, tow_ms, valid, count


def parse_sync(line: str) -> dict[str, int] | None:
    """Parse one "# sync,key=value,..." diagnostic line into its counters."""
    if not line.startswith("# sync,"):
        return None
    fields: dict[str, str] = {}
    for item in line[len("# sync,"):].split(","):
        if "=" in item:
            key, value = item.split("=", 1)
            fields[key] = value
    try:
        return {name: int(fields[name]) for name in SYNC_COUNTERS} | {
            "pps": int(fields.get("pps", "0"))}
    except (KeyError, ValueError):
        return None


def u32_delta(current: int, previous: int) -> int:
    """Unsigned 32-bit difference to survive firmware counter wrap."""
    return (current - previous) & 0xFFFFFFFF


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


def imu_live_sample(row: list[int | float]) -> dict[str, int | float]:
    """Build the compact one-sample IMU payload published to the web UI."""
    return {
        "sample": int(row[0]),
        "gps_week": int(row[1]),
        "gps_tow_us": int(row[2]),
        "time_valid": int(row[3]),
        "timer_us": int(row[4]),
        "ax_m_s2": float(row[14]),
        "ay_m_s2": float(row[15]),
        "az_m_s2": float(row[16]),
        "temp_deg_c": float(row[17]),
        # CSV keeps deg/h for Allan and navigation compatibility.  The live
        # dashboard uses the more readable deg/s requested for monitoring.
        "gx_deg_s": float(row[18]) / 3600.0,
        "gy_deg_s": float(row[19]) / 3600.0,
        "gz_deg_s": float(row[20]) / 3600.0,
    }


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


class CsvRecorder:
    """Open and close one timestamped recording session without closing UART."""

    def __init__(self, parent: pathlib.Path, selected: set[str]):
        self.parent = parent
        self.selected = selected
        self.stack: contextlib.ExitStack | None = None
        self.streams: dict[str, object] = {}
        self.writers: dict[str, csv.writer] = {}
        self.session_dir: pathlib.Path | None = None
        self.rows_since_flush = 0

    @property
    def active(self) -> bool:
        return self.stack is not None

    def start(self) -> pathlib.Path | None:
        if self.active or not self.selected:
            return self.session_dir
        stamp = dt.datetime.now().strftime("%Y%m%d%H%M%S")
        self.session_dir = create_session_directory(self.parent, stamp)
        self.stack = contextlib.ExitStack()
        columns = {"imu": IMU_COLUMNS, "gnss": GNSS_COLUMNS, "rawx": RAWX_COLUMNS}
        try:
            for kind in LOG_TYPES:
                if kind not in self.selected:
                    continue
                path = self.session_dir / f"{kind}.csv"
                stream = self.stack.enter_context(
                    path.open("w", encoding="utf-8", newline=""))
                writer = csv.writer(stream, lineterminator="\n")
                writer.writerow(columns[kind])
                stream.flush()
                self.streams[kind] = stream
                self.writers[kind] = writer
            # sync.csv is an always-on diagnostic, independent of --save.
            sync_path = self.session_dir / "sync.csv"
            sync_stream = self.stack.enter_context(
                sync_path.open("w", encoding="utf-8", newline=""))
            sync_writer = csv.writer(sync_stream, lineterminator="\n")
            sync_writer.writerow(SYNC_COLUMNS)
            sync_stream.flush()
            self.streams["sync"] = sync_stream
            self.writers["sync"] = sync_writer
        except Exception:
            self.stop()
            raise
        self.rows_since_flush = 0
        return self.session_dir

    def stop(self) -> None:
        if self.stack is not None:
            for stream in self.streams.values():
                stream.flush()
            self.stack.close()
        self.stack = None
        self.streams = {}
        self.writers = {}
        self.rows_since_flush = 0

    def write(self, kind: str, row: list[int | float | str]) -> bool:
        writer = self.writers.get(kind)
        if writer is None:
            return False
        writer.writerow(row)
        self.rows_since_flush += 1
        if kind in ("gnss", "sync"):
            # The LAN dashboard follows the live state file, while immediate
            # GNSS flush also keeps the on-disk navigation result current.
            # sync.csv is one row per second and should stay visible too.
            self.streams[kind].flush()
        if self.rows_since_flush >= 100:
            for stream in self.streams.values():
                stream.flush()
            self.rows_since_flush = 0
        return True


class F9pUbxTap:
    """Continuously drain a receive-only F9P UART and record it on demand."""

    def __init__(self, port_name: str, baud: int):
        self.port_name = port_name
        self.baud = baud
        self._port = None
        self._stream = None
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._error: str | None = None
        self._received_bytes = 0
        self._recorded_bytes = 0
        self._last_rx_unix_ms: int | None = None
        self._bytes_since_flush = 0
        self._last_flush = time.monotonic()

    @property
    def active(self) -> bool:
        with self._lock:
            return self._stream is not None

    @property
    def error(self) -> str | None:
        with self._lock:
            return self._error

    def open(self) -> None:
        if self._port is not None:
            return
        self._port = serial.Serial(self.port_name, self.baud, timeout=0.2)
        self._port.reset_input_buffer()
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._run, name="f9p-ubx-tap", daemon=True)
        self._thread.start()

    def start_recording(self, session_dir: pathlib.Path) -> pathlib.Path:
        path = session_dir / "f9p.ubx"
        stream = path.open("xb", buffering=1024 * 1024)
        with self._lock:
            if self._stream is not None:
                stream.close()
                raise RuntimeError("F9P UBX recording is already active")
            if self._error is not None:
                stream.close()
                raise OSError(self._error)
            self._recorded_bytes = 0
            self._bytes_since_flush = 0
            self._last_flush = time.monotonic()
            self._stream = stream
        return path

    def stop_recording(self) -> None:
        with self._lock:
            stream = self._stream
            self._stream = None
        if stream is not None:
            stream.flush()
            stream.close()

    def snapshot(self) -> dict[str, object]:
        with self._lock:
            return {
                "port": self.port_name,
                "baud": self.baud,
                "connected": self._port is not None and self._error is None,
                "recording": self._stream is not None,
                "received_bytes": self._received_bytes,
                "recorded_bytes": self._recorded_bytes,
                "last_rx_unix_ms": self._last_rx_unix_ms,
                "error": self._error,
            }

    def _record_data(self, data: bytes) -> None:
        """Store one received chunk; split out to make file behavior testable."""
        now_ms = round(time.time() * 1000)
        with self._lock:
            self._received_bytes += len(data)
            self._last_rx_unix_ms = now_ms
            if self._stream is not None:
                self._stream.write(data)
                self._recorded_bytes += len(data)
                self._bytes_since_flush += len(data)
                if (self._bytes_since_flush >= 65536 or
                        time.monotonic() - self._last_flush >= 1.0):
                    self._stream.flush()
                    self._bytes_since_flush = 0
                    self._last_flush = time.monotonic()

    def _run(self) -> None:
        try:
            while not self._stop.is_set():
                assert self._port is not None
                waiting = self._port.in_waiting
                data = self._port.read(min(max(waiting, 1), 65536))
                if data:
                    self._record_data(data)
        except (OSError, serial.SerialException) as error:
            with self._lock:
                self._error = str(error)

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        self.stop_recording()
        if self._port is not None:
            self._port.close()
        self._port = None
        self._thread = None


def write_runtime_state(path: pathlib.Path | None, payload: dict[str, object]) -> None:
    """Atomically publish volatile live state for the read-only web service."""
    if path is None:
        return
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_name(path.name + ".tmp")
        temporary.write_text(json.dumps(payload, ensure_ascii=False,
                                        separators=(",", ":")), encoding="utf-8")
        temporary.replace(path)
    except OSError as error:
        print(f"Warning: cannot update runtime state {path}: {error}", file=sys.stderr,
              flush=True)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port", help="STM32 PA9 USB-TTL port, for example COM7")
    parser.add_argument("--baud", type=int, default=460800)
    parser.add_argument("--hours", type=float, default=0.0,
                        help="Stop after this many hours; 0 means until Ctrl+C")
    parser.add_argument("--status-interval", type=float, default=5.0,
                        help="Print one live status line every N seconds; default: 5")
    parser.add_argument("--output-dir", type=pathlib.Path, default=DATA_DIR / "decoded")
    parser.add_argument("--save", nargs="*", choices=LOG_TYPES,
                        default=list(LOG_TYPES), metavar="TYPE",
                        help="CSV types to save: imu gnss rawx; default: all")
    parser.add_argument("--record-control", type=pathlib.Path,
                        help="Service mode: record only while this file exists")
    parser.add_argument("--state-file", type=pathlib.Path,
                        help="Atomically publish live state JSON for a dashboard")
    parser.add_argument("--ntrip-control", type=pathlib.Path,
                        help="Volatile JSON NTRIP config; RTCM is forwarded on this UART")
    parser.add_argument("--ubx-port",
                        help="Receive-only F9P UART to save as f9p.ubx while recording")
    parser.add_argument("--ubx-baud", type=int, default=115200,
                        help="Baud rate of --ubx-port; default: 115200")
    args = parser.parse_args()

    ntrip = None
    parse_bridge_report = None
    if args.ntrip_control is not None:
        # capture_serial.py is also a standalone PC tool. Load the Pi-only
        # NTRIP bridge only when its control file is explicitly configured.
        sys.path.insert(0, str(TOOLS_DIR.parent))
        from raspberry_pi5.ntrip_client import (  # pylint: disable=import-outside-toplevel
            NtripController, parse_bridge_report as parse_rtcm_report,
        )
        ntrip = NtripController(args.ntrip_control)
        parse_bridge_report = parse_rtcm_report

    selected = set(args.save)
    recorder = CsvRecorder(args.output_dir, selected)
    ubx_tap = F9pUbxTap(args.ubx_port, args.ubx_baud) if args.ubx_port else None
    controlled_recording = args.record_control is not None
    if controlled_recording:
        args.record_control.parent.mkdir(parents=True, exist_ok=True)
        requested = args.record_control.exists()
    else:
        requested = bool(selected)
    deadline = time.monotonic() + args.hours * 3600.0 if args.hours > 0 else None
    imu_rows = gnss_rows = rawx_rows = sat_rows = invalid = lost = 0
    rawx_epoch = None; rawx_seen_header = False
    first_timer_us = last_timer_us = previous_sample = None
    prev_sync: dict[str, int] | None = None
    latest_sync_diag: dict[str, int] | None = None
    started = time.monotonic()
    started_unix = time.time()
    last_status = started
    last_state = 0.0
    last_status_imu = 0
    latest_imu: dict[str, int | float] | None = None
    latest_imu_unix_ms: int | None = None
    latest_gnss: dict[str, int | float] | None = None
    latest_gnss_unix_ms: int | None = None
    satellite_epoch: list[dict[str, int]] = []
    satellite_epoch_key: tuple[int, int, int] | None = None
    latest_satellites: list[dict[str, int]] = []
    latest_satellites_unix_ms: int | None = None
    recording_error: str | None = None

    def start_recording() -> pathlib.Path:
        session = recorder.start()
        if session is None:
            raise OSError("no CSV type was selected")
        try:
            if ubx_tap is not None:
                ubx_tap.start_recording(session)
        except Exception:
            recorder.stop()
            raise
        return session

    def stop_recording() -> pathlib.Path | None:
        session = recorder.session_dir
        if ubx_tap is not None:
            ubx_tap.stop_recording()
        recorder.stop()
        return session

    print(f"Position service on {args.port} at {args.baud} baud")
    if ubx_tap is not None:
        print(f"F9P UBX tap on {args.ubx_port} at {args.ubx_baud} baud")
    if controlled_recording:
        print(f"Recording control -> {args.record_control.resolve()}")
        print("Recording starts disabled unless the control file already exists.")
    if recorder.active:
        print(f"Recording session -> {recorder.session_dir.resolve()}")
    print("Press Ctrl+C to stop safely.")

    def publish_state(service_active: bool = True) -> None:
        now_unix = time.time()
        write_runtime_state(args.state_file, {
            "protocol_version": 3,
            "service_active": service_active,
            "recording": recorder.active,
            "session": recorder.session_dir.name if recorder.active and recorder.session_dir else None,
            "updated_unix_ms": round(now_unix * 1000),
            "started_unix_ms": round(started_unix * 1000),
            "imu_updated_unix_ms": latest_imu_unix_ms,
            "imu": latest_imu,
            "gnss_updated_unix_ms": latest_gnss_unix_ms,
            "gnss": latest_gnss,
            "satellites_updated_unix_ms": latest_satellites_unix_ms,
            "satellites": latest_satellites,
            "recording_error": recording_error,
            "ubx": ubx_tap.snapshot() if ubx_tap is not None else {
                "connected": False, "recording": False,
                "received_bytes": 0, "recorded_bytes": 0, "error": "",
            },
            "ntrip": ntrip.snapshot() if ntrip is not None else {
                "requested": False, "connected": False, "phase": "未配置",
                "error": "", "forwarded_bytes": 0, "bridge_ready": False,
            },
            "counts": {"imu": imu_rows, "gnss": gnss_rows, "rawx": rawx_rows,
                       "sat": sat_rows, "lost": lost, "invalid": invalid,
                       "sync_diag": latest_sync_diag},
        })

    try:
        if ubx_tap is not None:
            ubx_tap.open()
        if requested:
            start_recording()
        if recorder.active:
            print(f"Recording session -> {recorder.session_dir.resolve()}")

        with serial.Serial(args.port, args.baud, timeout=1, write_timeout=0.5) as port:
            port.reset_input_buffer()
            # Discard the first fragment because opening a continuous stream can
            # begin in the middle of a line.
            port.readline()
            while deadline is None or time.monotonic() < deadline:
                if ntrip is not None:
                    ntrip.sync_control()
                    try:
                        ntrip.pump_serial(port)
                    except serial.SerialException as error:
                        ntrip.disable(f"树莓派UART下发失败：{error}")
                if controlled_recording:
                    requested = args.record_control.exists()
                    if requested and not recorder.active:
                        try:
                            session = start_recording()
                            recording_error = None
                            print(f"Recording started -> {session.resolve()}", flush=True)
                        except OSError as error:
                            recording_error = str(error)
                            args.record_control.unlink(missing_ok=True)
                            print(f"Recording start failed: {error}", file=sys.stderr,
                                  flush=True)
                        publish_state()
                    elif not requested and recorder.active:
                        finished = stop_recording()
                        recording_error = None
                        print(f"Recording stopped -> {finished.resolve()}", flush=True)
                        publish_state()
                if ubx_tap is not None and ubx_tap.error is not None and recorder.active:
                    recording_error = f"F9P UBX UART failed: {ubx_tap.error}"
                    if controlled_recording:
                        args.record_control.unlink(missing_ok=True)
                    stop_recording()
                    print(recording_error, file=sys.stderr, flush=True)
                    publish_state()
                serial_bytes = port.readline()
                status_now = time.monotonic()
                if (args.status_interval > 0 and
                        status_now - last_status >= args.status_interval):
                    status_elapsed = status_now - last_status
                    imu_hz = (imu_rows - last_status_imu) / status_elapsed
                    wall_clock = dt.datetime.now().strftime("%H:%M:%S")
                    ubx_status = ubx_tap.snapshot() if ubx_tap else {
                        "received_bytes": 0, "recorded_bytes": 0}
                    print(
                        f"[{wall_clock}] IMU={imu_rows} {imu_hz:.2f}Hz "
                        f"GNSS={gnss_rows} RAWX={rawx_rows} SAT={sat_rows} "
                        f"UBXrx/save={ubx_status['received_bytes']}/"
                        f"{ubx_status['recorded_bytes']}B "
                        f"lost={lost} invalid={invalid} "
                        f"REC={'ON' if recorder.active else 'OFF'} "
                        f"t={(status_now - started) / 60.0:.1f}min",
                        end="\r" if sys.stdout.isatty() else "\n",
                        flush=True,
                    )
                    last_status = status_now
                    last_status_imu = imu_rows
                if status_now - last_state >= 1.0:
                    publish_state()
                    last_state = status_now
                if not serial_bytes:
                    continue
                try:
                    line = serial_bytes.decode("ascii").strip()
                except UnicodeDecodeError:
                    invalid += 1; continue
                if not line:
                    continue
                if line.startswith("#RTCM,") and ntrip is not None:
                    report = parse_bridge_report(line)
                    if report is None:
                        invalid += 1
                    else:
                        ntrip.update_bridge(report)
                    continue
                if line.startswith("# sync,"):
                    sync = parse_sync(line)
                    if sync is not None:
                        if prev_sync is not None:
                            deltas = {
                                "d_interrupt_overruns": u32_delta(
                                    sync["interrupt_overruns"],
                                    prev_sync["interrupt_overruns"]),
                                "d_cc2_overcapture": u32_delta(
                                    sync["cc2_overcapture"],
                                    prev_sync["cc2_overcapture"]),
                                "d_dt_gap_count": u32_delta(
                                    sync["dt_gap_count"],
                                    prev_sync["dt_gap_count"]),
                                "d_i2c_errors": u32_delta(
                                    sync["i2c_errors"], prev_sync["i2c_errors"]),
                            }
                        else:
                            deltas = {"d_interrupt_overruns": 0, "d_cc2_overcapture": 0,
                                      "d_dt_gap_count": 0, "d_i2c_errors": 0}
                        backlog = (sync["interrupt_count"] -
                                   sync["sample_count"]) & 0xFFFFFFFF
                        row = [round(time.time() * 1000), sync["pps"],
                               sync["sample_count"], sync["interrupt_count"],
                               sync["interrupt_overruns"], sync["cc2_overcapture"],
                               sync["dt_gap_count"], sync["i2c_errors"],
                               deltas["d_interrupt_overruns"],
                               deltas["d_cc2_overcapture"],
                               deltas["d_dt_gap_count"],
                               deltas["d_i2c_errors"], backlog]
                        recorder.write("sync", row)
                        latest_sync_diag = {**sync, **deltas, "backlog": backlog}
                        prev_sync = sync
                    continue
                if line.startswith("#"):
                    continue
                parts = line.split(",")
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
                    recorder.write("imu", row)
                    imu_rows += 1
                    latest_imu = imu_live_sample(row)
                    latest_imu_unix_ms = round(time.time() * 1000)
                elif parts[0] == "GNSS":
                    row = parse_gnss(parts)
                    if row is None:
                        invalid += 1; continue
                    recorder.write("gnss", row)
                    latest_gnss = dict(zip(GNSS_COLUMNS, row))
                    latest_gnss_unix_ms = round(time.time() * 1000)
                    gnss_rows += 1
                    publish_state()
                    last_state = status_now
                elif parts[0] == "SAT":
                    satellite = parse_satellite(parts)
                    if satellite is None:
                        invalid += 1; continue
                    key = (satellite["gps_week"], satellite["gps_tow_ms"],
                           satellite["time_valid"])
                    if satellite_epoch_key != key:
                        satellite_epoch = []
                        satellite_epoch_key = key
                    satellite_epoch.append(satellite)
                    sat_rows += 1
                elif parts[0] == "SAT_END":
                    end = parse_satellite_end(parts)
                    if end is None:
                        invalid += 1; continue
                    key = end[:3]
                    if satellite_epoch_key == key and len(satellite_epoch) == end[3]:
                        latest_satellites = satellite_epoch
                        latest_satellites_unix_ms = round(time.time() * 1000)
                        publish_state()
                        last_state = status_now
                    else:
                        invalid += 1
                    satellite_epoch = []
                    satellite_epoch_key = None
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
                    recorder.write("rawx", row)
                    rawx_rows += 1
                elif parts[0] == "RAWX_END" and len(parts) == 2:
                    rawx_epoch = None
                else:
                    invalid += 1

    except KeyboardInterrupt:
        pass
    finally:
        stop_recording()
        if ubx_tap is not None:
            ubx_tap.close()
        if ntrip is not None:
            ntrip.close()
        publish_state(service_active=False)

    elapsed = time.monotonic() - started
    print(f"\nProcessed IMU {imu_rows:,}, GNSS {gnss_rows:,}, RAWX {rawx_rows:,}, satellite records {sat_rows:,} "
          f"({elapsed / 3600.0:.3f} h)")
    print(f"Lost IMU frames: {lost:,}; invalid lines: {invalid:,}")


if __name__ == "__main__":
    main()
