# -*- coding: utf-8 -*-
"""Validate and summarize Qt ``rawx.csv`` files.

The input is the expanded UBX-RXM-RAWX stream written by the Qt recorder. The
tool never modifies that file. It validates epoch row counts, reports signal
quality, and finds carrier tracking discontinuities suitable for follow-up
inspection before tightly coupled processing.

Usage::

    python tools/decode_rawx.py rawx.csv
    python tools/decode_rawx.py rawx.csv --slip-threshold 1.5 --json summary.json

Signal identifiers follow u-blox F9 HPG 1.32 Interface Description, table 4.
``sigId`` is constellation-specific and must never be decoded on its own.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import sys
from collections import OrderedDict, defaultdict
from pathlib import Path

GPS_WEEK_SECONDS = 604800.0
DEFAULT_SLIP_THRESHOLD_CYCLES = 1.5

CONSTELLATIONS = {
    0: "GPS", 1: "SBAS", 2: "GAL", 3: "BDS", 4: "IMES",
    5: "QZSS", 6: "GLO", 7: "NavIC",
}

# UBX protocol (gnssId, sigId) -> (stable name, nominal frequency in MHz).
# GLONASS FDMA frequencies are calculated from freqId in signal_info().
SIGNALS = {
    (0, 0): ("GPS_L1CA", 1575.42),
    (0, 3): ("GPS_L2CL", 1227.60),
    (0, 4): ("GPS_L2CM", 1227.60),
    (0, 6): ("GPS_L5I", 1176.45),
    (0, 7): ("GPS_L5Q", 1176.45),
    (1, 0): ("SBAS_L1CA", 1575.42),
    (2, 0): ("GAL_E1C", 1575.42),
    (2, 1): ("GAL_E1B", 1575.42),
    (2, 3): ("GAL_E5aI", 1176.45),
    (2, 4): ("GAL_E5aQ", 1176.45),
    (2, 5): ("GAL_E5bI", 1207.14),
    (2, 6): ("GAL_E5bQ", 1207.14),
    (3, 0): ("BDS_B1I_D1", 1561.098),
    (3, 1): ("BDS_B1I_D2", 1561.098),
    (3, 2): ("BDS_B2I_D1", 1207.14),
    (3, 3): ("BDS_B2I_D2", 1207.14),
    (3, 5): ("BDS_B1C", 1575.42),
    (3, 7): ("BDS_B2a", 1176.45),
    (5, 0): ("QZSS_L1CA", 1575.42),
    (5, 1): ("QZSS_L1S", 1575.42),
    (5, 4): ("QZSS_L2CM", 1227.60),
    (5, 5): ("QZSS_L2CL", 1227.60),
    (5, 8): ("QZSS_L5I", 1176.45),
    (5, 9): ("QZSS_L5Q", 1176.45),
    (7, 0): ("NavIC_L5A", 1176.45),
}

REQUIRED_COLUMNS = {
    "gps_week", "rcv_tow_s", "rx_timer_us", "epoch_num_meas",
    "epoch_total_meas", "gnss_id", "sv_id", "sig_id", "freq_id",
    "pseudorange_m", "carrier_phase_cycles", "doppler_hz", "locktime_ms",
    "cno_dbhz", "pr_valid", "cp_valid", "half_cycle", "sub_half_cycle",
}


def signal_info(gnss_id: int, sig_id: int, freq_id: int = 0) -> tuple[str, float | None]:
    """Return the UBX signal name and carrier frequency in MHz."""
    if gnss_id == 6 and sig_id in (0, 2):
        channel = freq_id - 7
        if sig_id == 0:
            return "GLO_L1OF", 1602.0 + channel * 0.5625
        return "GLO_L2OF", 1246.0 + channel * 0.4375
    return SIGNALS.get((gnss_id, sig_id),
                       (f"GNSS{gnss_id}_SIG{sig_id}", None))


def signal_name(gnss_id: int, sig_id: int, freq_id: int = 0) -> str:
    return signal_info(gnss_id, sig_id, freq_id)[0]


def _int(row: dict[str, str], field: str) -> int:
    return int(row[field])


def _float(row: dict[str, str], field: str) -> float:
    value = float(row[field])
    if not math.isfinite(value):
        raise ValueError(f"{field} is not finite")
    return value


def decode(path: str | Path):
    """Read the CSV and return ordered ``((week, tow, timer), rows)`` epochs."""
    epochs = OrderedDict()
    with Path(path).open(newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        missing = REQUIRED_COLUMNS.difference(reader.fieldnames or ())
        if missing:
            raise ValueError("RAWX CSV 缺少字段：" + ", ".join(sorted(missing)))
        for line_number, row in enumerate(reader, 2):
            try:
                key = (_int(row, "gps_week"), _float(row, "rcv_tow_s"),
                       _int(row, "rx_timer_us"))
                # Parse fields now so malformed rows fail with a useful line.
                for field in ("epoch_num_meas", "epoch_total_meas", "gnss_id",
                              "sv_id", "sig_id", "freq_id", "locktime_ms",
                              "pr_valid", "cp_valid", "half_cycle",
                              "sub_half_cycle"):
                    _int(row, field)
                for field in ("pseudorange_m", "carrier_phase_cycles",
                              "doppler_hz", "cno_dbhz"):
                    _float(row, field)
            except (KeyError, TypeError, ValueError) as error:
                raise ValueError(f"RAWX CSV 第 {line_number} 行无效：{error}") from error
            epochs.setdefault(key, []).append(row)
    return list(epochs.items())


def validate_epochs(epochs) -> list[dict[str, object]]:
    """Return one issue record for every inconsistent or incomplete epoch."""
    issues = []
    for (week, tow, timer), rows in epochs:
        declared = {_int(row, "epoch_num_meas") for row in rows}
        totals = {_int(row, "epoch_total_meas") for row in rows}
        observation_keys = [
            (_int(row, "gnss_id"), _int(row, "sv_id"),
             _int(row, "sig_id"), _int(row, "freq_id"))
            for row in rows
        ]
        reasons = []
        if len(declared) != 1:
            reasons.append("epoch_num_meas 不一致")
        elif len(rows) != next(iter(declared)):
            reasons.append(f"声明 {next(iter(declared))} 条，实际 {len(rows)} 条")
        if len(totals) != 1:
            reasons.append("epoch_total_meas 不一致")
        elif declared and next(iter(totals)) < next(iter(declared)):
            reasons.append("total_meas 小于 num_meas")
        duplicate_count = len(observation_keys) - len(set(observation_keys))
        if duplicate_count:
            reasons.append(f"重复观测 {duplicate_count} 条")
        if reasons:
            issues.append({"gps_week": week, "rcv_tow_s": tow,
                           "rx_timer_us": timer, "reasons": reasons})
    return issues


def _tracking_key(row: dict[str, str]) -> tuple[int, int, int, int]:
    return (_int(row, "gnss_id"), _int(row, "sv_id"),
            _int(row, "sig_id"), _int(row, "freq_id"))


def detect_cycle_slips(epochs, threshold_cycles: float = DEFAULT_SLIP_THRESHOLD_CYCLES):
    """Find carrier tracking discontinuities and phase-Doppler outliers.

    This is a quality-control detector, not a final geodetic cycle-slip
    estimator. A lock-time decrease is reported as ``lock_reset``. Otherwise
    two adjacent 1 Hz epochs with valid carrier phase and finite Doppler are
    tested using ``d(phi)/dt = -Doppler``. Invalid carrier observations break
    continuity and are never converted into slip events.
    """
    if threshold_cycles <= 0:
        raise ValueError("周跳阈值必须大于 0")
    events = defaultdict(list)
    previous = {}
    for epoch_index, ((week, tow, _timer), rows) in enumerate(epochs):
        absolute_time = week * GPS_WEEK_SECONDS + tow
        for row in rows:
            key = _tracking_key(row)
            if _int(row, "cp_valid") != 1:
                previous.pop(key, None)
                continue
            cp = _float(row, "carrier_phase_cycles")
            doppler = _float(row, "doppler_hz")
            lock_ms = _int(row, "locktime_ms")
            if key in previous:
                prev_cp, prev_doppler, prev_time, prev_lock = previous[key]
                dt = absolute_time - prev_time
                if 0.5 <= dt <= 1.5:
                    residual = cp - (prev_cp - 0.5 * (prev_doppler + doppler) * dt)
                    if lock_ms < prev_lock:
                        events[key].append({
                            "epoch_index": epoch_index, "gps_week": week,
                            "rcv_tow_s": tow, "kind": "lock_reset",
                            "residual_cycles": residual, "dt_s": dt,
                            "previous_lock_ms": prev_lock, "lock_ms": lock_ms,
                        })
                    elif abs(residual) > threshold_cycles:
                        events[key].append({
                            "epoch_index": epoch_index, "gps_week": week,
                            "rcv_tow_s": tow, "kind": "phase_doppler_outlier",
                            "residual_cycles": residual, "dt_s": dt,
                            "previous_lock_ms": prev_lock, "lock_ms": lock_ms,
                        })
            previous[key] = (cp, doppler, absolute_time, lock_ms)
    return events


def count_retracks(valid_series):
    """Count valid carrier observation gaps longer than two seconds."""
    retracks = {}
    for key, times in valid_series.items():
        retracks[key] = sum(b - a > 2.0 for a, b in zip(times, times[1:]))
    return retracks


def analyze(path: str | Path,
            slip_threshold: float = DEFAULT_SLIP_THRESHOLD_CYCLES) -> dict[str, object]:
    epochs = decode(path)
    if not epochs:
        raise ValueError("RAWX CSV 为空")

    epoch_issues = validate_epochs(epochs)
    absolute_times = [week * GPS_WEEK_SECONDS + tow for (week, tow, _), _ in epochs]
    dts = [b - a for a, b in zip(absolute_times, absolute_times[1:]) if b > a]
    span_s = absolute_times[-1] - absolute_times[0]
    per_epoch = []
    valid_series = OrderedDict()
    flags = {"total": 0, "pr_valid": 0, "cp_valid": 0,
             "half_cycle_valid": 0, "sub_half_cycle": 0}
    unknown_signals = defaultdict(int)

    for ((week, tow, _timer), rows), absolute_time in zip(epochs, absolute_times):
        unique_signals = {}
        cnos = []
        for row in rows:
            key = _tracking_key(row)
            unique_signals[key] = row
            cnos.append(_float(row, "cno_dbhz"))
            flags["total"] += 1
            for field in ("pr_valid", "cp_valid"):
                flags[field] += _int(row, field) == 1
            flags["half_cycle_valid"] += _int(row, "half_cycle") == 1
            flags["sub_half_cycle"] += _int(row, "sub_half_cycle") == 1
            if _int(row, "cp_valid") == 1:
                valid_series.setdefault(key, []).append(absolute_time)
            name, _frequency = signal_info(key[0], key[2], key[3])
            if name.startswith("GNSS"):
                unknown_signals[(key[0], key[2])] += 1
        constellation_count = defaultdict(int)
        for gnss_id, _sv_id, _sig_id, _freq_id in unique_signals:
            constellation_count[CONSTELLATIONS.get(gnss_id, f"GNSS{gnss_id}")] += 1
        per_epoch.append({
            "week": week, "tow": tow, "nsig": len(unique_signals),
            "constellations": dict(constellation_count),
            "mean_cno": statistics.mean(cnos), "min_cno": min(cnos),
            "max_cno": max(cnos),
        })

    events = detect_cycle_slips(epochs, slip_threshold)
    lock_resets = sum(event["kind"] == "lock_reset"
                      for values in events.values() for event in values)
    phase_outliers = sum(event["kind"] == "phase_doppler_outlier"
                         for values in events.values() for event in values)
    retracks = count_retracks(valid_series)
    total_retracks = sum(retracks.values())

    print("=" * 68)
    print("RAWX 原始观测质量检查")
    print("=" * 68)
    first_week, first_tow, _ = epochs[0][0]
    last_week, last_tow, _ = epochs[-1][0]
    print(f"历元数              : {len(epochs)}")
    print(f"时间跨度            : {span_s:.3f} s  "
          f"(W{first_week} {first_tow:.3f} → W{last_week} {last_tow:.3f})")
    if dts:
        non_one_hz = sum(abs(dt - 1.0) > 0.05 for dt in dts)
        print(f"历元间隔(中位)      : {statistics.median(dts):.3f} s  "
              f"(非 1 Hz 间隔 {non_one_hz} 次)")
    print(f"历元完整性          : {len(epochs) - len(epoch_issues)}/{len(epochs)} 完整")
    print(f"观测总数            : {flags['total']}  "
          f"平均 {flags['total'] / len(epochs):.1f}/历元")
    print(f"有效标志            : PR {100*flags['pr_valid']/flags['total']:.1f}%  "
          f"CP {100*flags['cp_valid']/flags['total']:.1f}%")
    print(f"半周状态            : halfCyc有效 {flags['half_cycle_valid']}  "
          f"subHalfCyc {flags['sub_half_cycle']}")
    print(f"锁定时间回退        : {lock_resets} 次")
    print(f"相位-多普勒异常候选 : {phase_outliers} 次  "
          f"(|残差| > {slip_threshold:g} cycles，仅CP有效相邻历元)")
    print(f"有效载波重捕获      : {total_retracks} 次  (间隔 > 2 s)")

    constellation_totals = defaultdict(int)
    for item in per_epoch:
        for constellation, count in item["constellations"].items():
            constellation_totals[constellation] += count
    print("\n星座组成（观测条数 / 占比）:")
    for constellation, count in sorted(constellation_totals.items(),
                                       key=lambda item: -item[1]):
        print(f"  {constellation:6s} {count:8d}  "
              f"{100*count/flags['total']:5.1f}%")

    if epoch_issues:
        print("\n不完整/不一致历元（最多显示 10 个）:")
        for issue in epoch_issues[:10]:
            print(f"  W{issue['gps_week']} {issue['rcv_tow_s']:.3f}: "
                  + "；".join(issue["reasons"]))
    if unknown_signals:
        print("\n未知信号标识（原始ID已保留）:")
        for (gnss_id, sig_id), count in sorted(unknown_signals.items()):
            print(f"  gnssId={gnss_id}, sigId={sig_id}: {count} 条")

    event_ranking = sorted(events.items(), key=lambda item: -len(item[1]))[:10]
    if event_ranking:
        print("\n跟踪异常最多的信号（最多 10 个）:")
        for (gnss_id, sv_id, sig_id, freq_id), values in event_ranking:
            resets = sum(item["kind"] == "lock_reset" for item in values)
            outliers = len(values) - resets
            print(f"  {CONSTELLATIONS.get(gnss_id, gnss_id)} {sv_id:02d} "
                  f"{signal_name(gnss_id, sig_id, freq_id):12s}  "
                  f"锁定回退 {resets}  相位异常 {outliers}")

    bin_seconds = 300.0
    number_of_bins = max(1, math.ceil(max(span_s, 0.0) / bin_seconds))
    event_bins = [0] * number_of_bins
    for values in events.values():
        for event in values:
            event_time = event["gps_week"] * GPS_WEEK_SECONDS + event["rcv_tow_s"]
            index = min(number_of_bins - 1,
                        max(0, int((event_time - absolute_times[0]) / bin_seconds)))
            event_bins[index] += 1

    sample_step = max(1, len(per_epoch) // 600)
    sampled = per_epoch[::sample_step]
    return {
        "source": str(Path(path)),
        "n_epochs": len(epochs), "span_s": span_s,
        "median_epoch_interval_s": statistics.median(dts) if dts else None,
        "non_1hz_intervals": sum(abs(dt - 1.0) > 0.05 for dt in dts),
        "epoch_issue_count": len(epoch_issues), "epoch_issues": epoch_issues,
        "observation_count": flags["total"], "quality_flags": flags,
        "lock_resets": lock_resets, "phase_doppler_outliers": phase_outliers,
        "valid_carrier_retracks": total_retracks,
        "slip_threshold_cycles": slip_threshold,
        "unknown_signals": [
            {"gnss_id": key[0], "sig_id": key[1], "count": count}
            for key, count in sorted(unknown_signals.items())
        ],
        "series": {
            "elapsed_s": [item["week"] * GPS_WEEK_SECONDS + item["tow"]
                          - absolute_times[0] for item in sampled],
            "signal_count": [item["nsig"] for item in sampled],
            "mean_cno": [item["mean_cno"] for item in sampled],
            "min_cno": [item["min_cno"] for item in sampled],
            "max_cno": [item["max_cno"] for item in sampled],
        },
        "event_bin_center_minutes": [
            (index + 0.5) * bin_seconds / 60.0 for index in range(number_of_bins)
        ],
        "event_bin_counts": event_bins,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="检查 Qt rawx.csv 原始观测质量")
    parser.add_argument("csv", help="Qt 保存的 rawx.csv")
    parser.add_argument("--slip-threshold", type=float,
                        default=DEFAULT_SLIP_THRESHOLD_CYCLES,
                        help="相位-多普勒异常阈值，单位 cycles（默认 1.5）")
    parser.add_argument("--json", help="可选 JSON 汇总输出路径")
    args = parser.parse_args(argv)
    try:
        result = analyze(args.csv, args.slip_threshold)
        if args.json:
            with Path(args.json).open("w", encoding="utf-8") as stream:
                json.dump(result, stream, ensure_ascii=False, indent=2)
            print(f"\n已写入 JSON：{args.json}")
    except (OSError, ValueError, csv.Error) as error:
        print(f"错误：{error}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
