"""Bounded hardware smoke test using the real Qt serial/NTRIP code.

Close the GUI before running. No CSV, raw folder, credentials or coordinates
are written. --bnc enables real RTCM delivery; without it this is read-only.
"""
from __future__ import annotations

import argparse
import json
import os
import pathlib
import sys
import time

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "host"))

import serial
from PyQt5 import QtWidgets
from imu_serial_qt import NavigationMonitor
from ntrip_rtcm import read_bnc_endpoint


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port", help="STM32 USB-TTL port, e.g. COM7")
    parser.add_argument("--seconds", type=float, default=40)
    parser.add_argument("--bnc", type=pathlib.Path, help="private config used only in memory")
    parser.add_argument("--stall-gui", action="store_true", help="pause GUI processing for 0.5 s every 10 s")
    parser.add_argument("--require-fixed", action="store_true", help="also require an RTK fixed final state")
    args = parser.parse_args()
    if not 1 <= args.seconds <= 300:
        parser.error("seconds must be between 1 and 300")
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    monitor = NavigationMonitor()
    port = serial.Serial(port=None, baudrate=460800, timeout=0, write_timeout=0)
    port.dtr = False; port.rts = False; port.port = args.port
    endpoint = read_bnc_endpoint(args.bnc) if args.bnc else None
    started = False
    baseline = None
    client = None
    begin = time.monotonic()
    last_stall = begin
    next_progress = begin + 20
    try:
        port.open()
        monitor.serial_port = port
        monitor.discard_until_newline = True
        monitor._start_serial_worker(port)
        while time.monotonic() - begin < args.seconds:
            app.processEvents()
            if args.stall_gui and time.monotonic() - last_stall > 10:
                time.sleep(0.5)
                last_stall = time.monotonic()
            r = monitor.bridge_report
            if baseline is None and r is not None and r[1]: baseline = r[:]
            if endpoint and not started and baseline is not None:
                host, nport, mount, user, password = endpoint
                if not user:
                    raise ValueError("This smoke test requires an account in the selected BNC config")
                # Do not write test credentials to QSettings.
                class MemorySettings:
                    def value(self, key, default=None):
                        return {"ntrip_host": host, "ntrip_port": nport,
                                "ntrip_mount": mount, "ntrip_user": user}.get(key, default)
                monitor.settings = MemorySettings()
                monitor.ntrip_password = password
                monitor.toggle_ntrip()
                client = monitor.ntrip_client
                started = True
            time.sleep(0.003)
            if time.monotonic() >= next_progress:
                print(json.dumps({"progress_s": round(time.monotonic() - begin),
                                  "imu": monitor.total_imu, "lost": monitor.lost_imu,
                                  "reconnects": client.reconnects if client else None,
                                  "queue": client.frames.snapshot() if client else None,
                                  "fix": monitor.fix_label.text()}, ensure_ascii=True), flush=True)
                next_progress += 20
        result = {
            "seconds": round(time.monotonic() - begin, 2),
            "imu": monitor.total_imu, "gnss": monitor.total_gnss, "rawx": monitor.total_rawx,
            "imu_lost": monitor.lost_imu, "invalid_lines": monitor.invalid_lines,
            "fix": monitor.fix_label.text(), "gps_time": monitor.time_label.text(),
            "ntrip": monitor.ntrip_label.text(),
            "network_bytes": client.network_bytes if client else 0,
            "network_frames": client.frame_count if client else 0,
            "network_crc_errors": client.crc_errors if client else 0,
            "reconnects": client.reconnects if client else 0,
            "msm_rewritten_filtered": [client.msm_adapter.rewritten, client.msm_adapter.filtered] if client else [],
            "queue": client.frames.snapshot() if client else None,
            "serial_sent_outstanding_max_age": monitor.serial_worker.snapshot,
            "serial_max_assembly_sendwait_host_residence_s": monitor.serial_worker.timing_snapshot,
            "rtcm_message_types": dict(client.message_types) if client else {},
            "base_positions_ecef": dict(client.parser.base_positions) if client else {},
            "base_msm_epochs": dict(client.parser.msm_epochs) if client else {},
            "gnss_state": dict(monitor.gnss_diagnostics),
            "bridge_start": baseline, "bridge_end": monitor.bridge_report,
        }
        print(json.dumps(result, ensure_ascii=True, indent=2))
        if endpoint:
            r = monitor.bridge_report
            ok = (baseline is not None and r is not None and r[3] > baseline[3]
                  and r[7] > baseline[7] and r[4:6] == baseline[4:6]
                  and r[9] == baseline[9] and monitor.invalid_lines == 0
                  and monitor.lost_imu == 0 and client is not None
                  and client.reconnects == 0 and monitor.ntrip_client is client)
            if args.require_fixed:
                ok = ok and monitor.gnss_diagnostics.get("carr_soln") == 2
            return 0 if ok else 2
        return 0
    finally:
        monitor.disconnect_serial("test complete")
        # Join native threads before delivering deferred QObject deletion.
        workers = list(monitor.ntrip_workers)
        for worker in workers: worker.stop()
        for worker in workers:
            while not worker.wait(1000): pass
        app.processEvents()
        monitor.close(); app.processEvents()


if __name__ == "__main__":
    raise SystemExit(main())
