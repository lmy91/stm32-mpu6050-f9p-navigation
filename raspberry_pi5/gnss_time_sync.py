#!/usr/bin/env python3
"""Set Raspberry Pi wall time once from a fresh, valid F9P GNSS epoch."""

from __future__ import annotations

import argparse
import json
import pathlib
import time

GPS_EPOCH_UNIX_S = 315_964_800
GPS_WEEK_S = 604_800


def gnss_utc_unix_s(state: dict[str, object], now_monotonic_s: float,
                    max_age_s: float = 3.0) -> float:
    """Convert a fresh live-state GNSS epoch to current UTC Unix seconds."""
    gnss = state.get("gnss")
    if not isinstance(gnss, dict) or int(gnss.get("time_valid", 0)) != 1:
        raise ValueError("GNSS time is not valid")
    week = int(gnss["gps_week"])
    tow_ms = int(gnss["gps_tow_ms"])
    leap_s = int(state["gps_utc_leap_seconds"])
    updated = float(state["gnss_updated_monotonic_s"])
    age = now_monotonic_s - updated
    if not 2_000 <= week <= 4_095:
        raise ValueError(f"implausible GPS week: {week}")
    if not 0 <= tow_ms < GPS_WEEK_S * 1_000:
        raise ValueError(f"invalid GPS time of week: {tow_ms}")
    if not 0 <= leap_s <= 64:
        raise ValueError(f"invalid GPS-UTC leap seconds: {leap_s}")
    if not 0 <= age <= max_age_s:
        raise ValueError(f"GNSS state is stale: {age:.3f}s")
    epoch_utc = GPS_EPOCH_UNIX_S + week * GPS_WEEK_S + tow_ms / 1_000.0 - leap_s
    return epoch_utc + age


def read_state(path: pathlib.Path) -> dict[str, object]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError("live state root is not an object")
    return value


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--state-file", type=pathlib.Path,
                        default=pathlib.Path("/run/gnss-imu/live.json"))
    parser.add_argument("--threshold", type=float, default=2.0,
                        help="Only set the clock when absolute error exceeds seconds")
    parser.add_argument("--timeout", type=float, default=0.0,
                        help="Wait seconds for valid GNSS; 0 waits indefinitely")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    started = time.monotonic()
    last_error = "waiting for live GNSS state"
    while args.timeout <= 0 or time.monotonic() - started < args.timeout:
        try:
            target = gnss_utc_unix_s(read_state(args.state_file), time.monotonic())
            offset = target - time.time()
            if abs(offset) <= args.threshold:
                print(f"System time already agrees with GNSS (offset {offset:+.3f}s)")
                return 0
            if args.dry_run:
                print(f"Would set system time from GNSS (offset {offset:+.3f}s)")
                return 0
            time.clock_settime(time.CLOCK_REALTIME, target)
            print(f"System time set from GNSS (previous offset {offset:+.3f}s)")
            return 0
        except (FileNotFoundError, KeyError, TypeError, ValueError,
                json.JSONDecodeError) as error:
            last_error = str(error)
            time.sleep(1.0)
    print(f"No valid GNSS time before timeout: {last_error}")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
