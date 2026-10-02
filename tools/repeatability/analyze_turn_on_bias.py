#!/usr/bin/env python3
"""Analyze fixed-pose IMU startup means after raw-domain TC and calibration.

Example (manual power-cycle sessions, described in TURN_ON_BIAS.md)::

    python tools/repeatability/analyze_turn_on_bias.py --experiment data/turn_on_bias/<experiment>

``pi-reboot-continuous-imu`` experiments measure segment-mean variation while
the IMU remains powered; their covariance must not initialize turn-on bias.
Dependencies: numpy, pandas, scipy, matplotlib (the existing Allan environment).
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from datetime import datetime, timezone
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.io import loadmat


ROOT = Path(__file__).resolve().parents[2]
G0 = 9.80665
UINT32 = 2**32
DEG_H_TO_RAD_S = math.pi / 180.0 / 3600.0
AXES = ["ax", "ay", "az", "gx", "gy", "gz"]
VALUE_COLUMNS = ["ax_m_s2", "ay_m_s2", "az_m_s2",
                 "gx_deg_h", "gy_deg_h", "gz_deg_h"]
SYNC_COUNTERS = ["interrupt_overruns", "cc2_overcapture", "dt_gap_count", "i2c_errors"]
COMPLETE_STATUSES = {"complete", "completed", "collected", "downloaded"}
CONTINUOUS_MODE = "pi-reboot-continuous-imu"
POWER_CYCLE_MODE = "imu-power-cycle"
plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def finite_array(value, shape: tuple, label: str) -> np.ndarray:
    arr = np.asarray(value, dtype=np.float64)
    if arr.size != math.prod(shape) or not np.isfinite(arr).all():
        raise ValueError(f"{label}: expected finite array with shape {shape}")
    return arr.reshape(shape)


def load_model(coeff: Path, calib: Path) -> dict:
    """Load and cross-check the same TC convention as the existing Allan tool."""
    if not coeff.is_file() or not calib.is_file():
        raise ValueError(f"calibration input missing: {coeff} / {calib}")
    if coeff.suffix.lower() == ".mat":
        metadata = loadmat(coeff, simplify_cells=True)
        full = finite_array(metadata["coef"], (6, 6), "coef")
        orders = finite_array(metadata.get("ord", metadata.get("axisOrder")), (6,), "ord")
        coeff_mat = coeff
    elif coeff.suffix.lower() == ".csv":
        frame = pd.read_csv(coeff)
        columns = {"axis", "order", "c0", "c1", "c2", "c3", "c4", "c5"}
        if not columns.issubset(frame.columns) or len(frame) != 6:
            raise ValueError("coefficient CSV requires exactly six axis rows and order,c0..c5")
        labels = [str(item).split("(")[0].strip().lower() for item in frame["axis"]]
        if labels != AXES:
            raise ValueError(f"coefficient CSV axis order must be {AXES}, got {labels}")
        full = finite_array(frame[["c5", "c4", "c3", "c2", "c1", "c0"]], (6, 6), "CSV coef")
        orders = finite_array(frame["order"], (6,), "CSV order")
        coeff_mat = coeff.with_suffix(".mat")
        if not coeff_mat.is_file():
            raise ValueError(f"coefficient CSV needs companion MAT metadata: {coeff_mat}")
        metadata = loadmat(coeff_mat, simplify_cells=True)
        if not np.allclose(full, finite_array(metadata["coef"], (6, 6), "MAT coef"),
                           rtol=1e-13, atol=1e-12):
            raise ValueError("CSV coefficients differ from companion MAT")
        mat_orders = finite_array(metadata.get("ord", metadata.get("axisOrder")), (6,), "MAT ord")
        if not np.array_equal(orders, mat_orders):
            raise ValueError("CSV orders differ from companion MAT")
    else:
        raise ValueError("--coeff must be a CSV with companion MAT, or a MAT file")
    if np.any(orders != np.floor(orders)) or np.any((orders < 0) | (orders > 5)):
        raise ValueError("temperature polynomial order must be an integer from 0 to 5")
    active = orders > 0
    for i, order in enumerate(orders.astype(int)):
        if order == 0 and np.any(full[i] != 0):
            raise ValueError(f"unfitted axis {AXES[i]} must contain all-zero coefficients")
        if order and np.any(full[i, :5-order] != 0):
            raise ValueError(f"axis {AXES[i]} contains terms above its declared order")
    if str(metadata.get("domain", "")) != "raw":
        raise ValueError("temperature metadata domain must explicitly be 'raw'")
    tref, tmin, tmax = [float(metadata[key]) for key in ("Tref", "Tmin", "Tmax")]
    if not np.isfinite([tref, tmin, tmax]).all() or not tmin <= tref <= tmax or tmin >= tmax:
        raise ValueError("temperature reference/range is invalid")
    poly = full.copy()
    poly[:, -1] = 0.0  # c0 is fitted metadata, NEVER a temperature correction.
    calibration = loadmat(calib, simplify_cells=True)["result"]
    required = ["ba", "Ca", "gyroBiasMean_deg_h", "tempCoeff", "Tref", "tcActive"]
    if not isinstance(calibration, dict) or any(key not in calibration for key in required):
        raise ValueError(f"calibration result requires {required}")
    if not np.allclose(poly, finite_array(calibration["tempCoeff"], (6, 6), "result.tempCoeff"),
                       rtol=1e-13, atol=1e-12):
        raise ValueError("calibration and coefficients use different temperature polynomials")
    if abs(float(calibration["Tref"]) - tref) > 1e-10:
        raise ValueError("calibration and coefficients use different Tref")
    cal_active = finite_array(calibration["tcActive"], (6,), "result.tcActive")
    if not np.isin(cal_active, [0, 1]).all() or not np.array_equal(cal_active.astype(bool), active):
        raise ValueError("calibration and coefficients use different active axes")
    for key, expected in (("tempFitMin", tmin), ("tempFitMax", tmax)):
        if key in calibration and abs(float(calibration[key]) - expected) > 1e-9:
            raise ValueError(f"calibration {key} differs from the temperature fit")
    if "tempDomain" in calibration and str(calibration["tempDomain"]) != "raw":
        raise ValueError("calibration temperature domain must be raw")
    ba = finite_array(calibration["ba"], (3,), "result.ba")
    ca = finite_array(calibration["Ca"], (3, 3), "result.Ca")
    gb = finite_array(calibration["gyroBiasMean_deg_h"], (3,), "result.gyroBiasMean_deg_h")
    if np.linalg.cond(ca) > 100:
        raise ValueError("calibration matrix is singular or ill-conditioned")
    return {"poly": poly, "active": active, "orders": orders.astype(int),
            "tref": tref, "tmin": tmin, "tmax": tmax, "ba": ba, "ca": ca, "gb": gb,
            "coeff": str(coeff), "coeff_mat": str(coeff_mat), "calib": str(calib),
            "input_sha256": {str(path): sha256(path) for path in {coeff, coeff_mat, calib}}}


def compensate(raw: np.ndarray, temperature: np.ndarray, model: dict) -> np.ndarray:
    result = raw.copy()
    delta = temperature - model["tref"]
    for i in np.flatnonzero(model["active"]):
        result[:, i] -= np.polyval(model["poly"][i], delta)
    result[:, :3] = (result[:, :3] - model["ba"]) @ model["ca"].T
    result[:, 3:] -= model["gb"]
    return result


def u32_difference(values: np.ndarray) -> np.ndarray:
    # int64 subtraction handles wrap while retaining exact uint32 values.
    return (np.diff(values.astype(np.int64)) & (UINT32 - 1)).astype(np.int64)


def sync_diagnostics(session: Path, first_sample: int, start_offset: int,
                     end_offset: int) -> tuple[dict, list[str], list[str]]:
    """Measure counter increments in the selected window, not lifetime totals."""
    metrics = {"sync_available": False, "sync_aligned": False}
    errors, warnings = [], []
    path = session / "sync.csv"
    if not path.is_file():
        return metrics, errors, ["missing_sync_csv"]
    try:
        frame = pd.read_csv(path)
        if len(frame) < 2:
            return metrics, errors, ["insufficient_sync_rows"]
        if "sample_count" not in frame:
            return metrics, errors, ["sync_missing_sample_count"]
        sample = pd.to_numeric(frame["sample_count"], errors="coerce").to_numpy(float)
        if not np.isfinite(sample).all() or np.any(sample != np.floor(sample)):
            return metrics, ["invalid_sync_sample_count"], warnings
        offset = (sample.astype(np.int64) - first_sample) & (UINT32 - 1)
        # Include only intervals whose BOTH endpoints are inside the window.
        # A delta in the first selected row may include a pre-window event.
        selected = frame.loc[(offset >= start_offset) & (offset <= end_offset)].copy()
        if len(selected) < 2:
            return metrics, errors, ["insufficient_aligned_sync_rows"]
        metrics.update(sync_available=True, sync_aligned=True, sync_rows=len(selected))
        for key in SYNC_COUNTERS:
            if key not in selected:
                warnings.append(f"sync_missing_{key}")
                continue
            values = pd.to_numeric(selected[key], errors="coerce").to_numpy(float)
            if not np.isfinite(values).all() or np.any(values != np.floor(values)):
                errors.append(f"invalid_sync_{key}")
                continue
            increments = u32_difference(values)
            if np.any(increments >= UINT32 // 2):
                errors.append(f"sync_counter_reset_{key}")
                continue
            total = int(increments.sum())
            metrics[f"sync_{key}_increment"] = total
            if total:
                errors.append(f"sync_{key}_increment={total}")
        if "backlog" in selected:
            metrics["sync_backlog_max"] = float(pd.to_numeric(selected["backlog"], errors="coerce").max())
    except (OSError, ValueError, pd.errors.ParserError) as exc:
        errors.append(f"sync_read_failed:{exc}")
    return metrics, errors, warnings


def block_means(times: np.ndarray, calibrated: np.ndarray, temperature: np.ndarray,
                window: float, block_seconds: float) -> list[dict]:
    """Fixed time bins avoid treating autocorrelated samples as independent."""
    rows = []
    count = max(1, int(math.ceil(window / block_seconds)))
    for i in range(count):
        begin, end = i * block_seconds, min((i + 1) * block_seconds, window)
        mask = (times >= begin) & (times < end)
        if mask.sum() < 2:
            continue
        row = {"block": i + 1, "start_s": begin, "end_s": end,
               "samples": int(mask.sum()), "temp_mean_c": float(temperature[mask].mean())}
        for axis, mean in zip(AXES, calibrated[mask].mean(axis=0)):
            row[f"{axis}_mean"] = float(mean)
        rows.append(row)
    return rows


def verify_input_files(session: Path, expected) -> tuple[dict, list[str]]:
    """Revalidate local files against the hashes recorded during download."""
    hashes, errors = {}, []
    if expected is not None and not isinstance(expected, list):
        return {"data_hashes_verified": False}, ["invalid_manifest_files_metadata"]
    if expected:
        for entry in expected:
            if not isinstance(entry, dict):
                errors.append("invalid_manifest_file_entry")
                continue
            name = entry.get("name", "")
            if not isinstance(name, str) or not name or Path(name).name != name or any(c in name for c in "\\/:"):
                errors.append("invalid_manifest_file_name")
                continue
            path = session / name
            if not path.is_file():
                errors.append(f"downloaded_file_missing:{name}")
                continue
            actual = sha256(path)
            hashes[name] = actual
            if actual.lower() != str(entry.get("sha256", "")).lower():
                errors.append(f"downloaded_file_hash_mismatch:{name}")
            if path.stat().st_size != entry.get("size"):
                errors.append(f"downloaded_file_size_mismatch:{name}")
        if "imu.csv" not in hashes:
            errors.append("manifest_files_missing_imu_csv")
    else:
        for name in ("imu.csv", "sync.csv"):
            if (session / name).is_file():
                hashes[name] = sha256(session / name)
    return {"data_hashes_verified": bool(expected) and not errors,
            "input_sha256_json": json.dumps(hashes, sort_keys=True)}, errors


def analyze_run(run: dict, experiment: Path, model: dict, args) -> tuple[dict, list[dict]]:
    row = {"index": run.get("index"), "attempt": run.get("attempt", 1),
           "acquisition_status": str(run.get("status", "unknown")),
           "local_session": run.get("local_session", ""), "valid": False,
           "power_cycle_verified": run.get("power_cycle_verified") is True}
    errors, warnings = [], []
    blocks = []
    if row["acquisition_status"].lower() not in COMPLETE_STATUSES:
        row["qc_errors"] = "acquisition_incomplete:" + row["acquisition_status"]
        row["qc_warnings"] = str(run.get("error", ""))
        return row, blocks
    if not row["local_session"]:
        row.update(qc_errors="missing_local_session", qc_warnings="")
        return row, blocks
    session = Path(row["local_session"])
    if not session.is_absolute():
        session = experiment / session
    row["local_session"] = str(session.resolve())
    try:
        file_metrics, file_errors = verify_input_files(session, run.get("files"))
        row.update(file_metrics)
        if file_errors:
            row.update(qc_errors=";".join(file_errors), qc_warnings="")
            return row, blocks
        frame = pd.read_csv(session / "imu.csv", usecols=["sample", "time_s", "temp_deg_c", *VALUE_COLUMNS])
        if len(frame) < 3:
            raise ValueError("fewer than three IMU rows")
        numeric = frame.apply(pd.to_numeric, errors="raise")
        if not np.isfinite(numeric.to_numpy(float)).all():
            raise ValueError("NaN/Inf in required IMU columns")
        sample_float = numeric["sample"].to_numpy(float)
        if np.any(sample_float != np.floor(sample_float)) or np.any((sample_float < 0) | (sample_float >= UINT32)):
            raise ValueError("sample must be uint32")
        samples = sample_float.astype(np.int64)
        times = numeric["time_s"].to_numpy(float)
        relative = times - times[0]
        dt = np.diff(times)
        if np.any(dt <= 0):
            raise ValueError("time_s is not strictly increasing; do not sort away resets")
        period = float(np.median(dt))
        row.update(recording_samples=len(frame), recording_duration_s=float(relative[-1]),
                   first_sample=int(samples[0]), last_sample=int(samples[-1]),
                   sample_rate_hz=1.0 / period,
                   recording_sample_gap_count=int(np.count_nonzero(u32_difference(samples) != 1)),
                   recording_time_gap_count=int(np.count_nonzero(dt > 1.5 * period)))
        # Require the acquisition to extend beyond the fixed window's endpoint.
        window_end = args.warmup_seconds + args.window_seconds
        if relative[-1] + 1.5 * period < window_end:
            errors.append("insufficient_recording_duration")
        mask = (relative >= args.warmup_seconds) & (relative < window_end)
        indices = np.flatnonzero(mask)
        if len(indices) < 3:
            raise ValueError("fewer than three samples in the fixed statistics window")
        first, last = int(indices[0]), int(indices[-1])
        wt = relative[mask] - args.warmup_seconds
        ws = samples[mask]
        temperature = numeric.loc[mask, "temp_deg_c"].to_numpy(float)
        raw = numeric.loc[mask, VALUE_COLUMNS].to_numpy(float)
        selected_diffs = u32_difference(ws)
        gaps = int(np.count_nonzero(selected_diffs != 1))
        time_gaps = int(np.count_nonzero(np.diff(wt) > 1.5 * period))
        covered = float((wt[-1] - wt[0] + period) / args.window_seconds)
        row.update(window_samples=len(indices), window_start_relative_s=float(relative[first]),
                   window_end_relative_s=float(relative[last]), window_coverage_ratio=covered,
                   window_sample_gap_count=gaps, window_time_gap_count=time_gaps,
                   temp_min_c=float(temperature.min()), temp_max_c=float(temperature.max()),
                   temp_mean_c=float(temperature.mean()), temp_std_c=float(temperature.std(ddof=1)),
                   temp_slope_c_min=float(np.polyfit(wt - wt.mean(), temperature, 1)[0] * 60.0),
                   sample_rate_actual_hz=float((len(indices)-1)/(wt[-1]-wt[0])))
        if gaps:
            errors.append(f"window_sample_gaps={gaps}")
        if time_gaps:
            errors.append(f"window_time_gaps={time_gaps}")
        if covered < args.min_coverage:
            errors.append(f"window_coverage<{args.min_coverage:g}")
        if args.expected_rate_hz and abs(row["sample_rate_actual_hz"] / args.expected_rate_hz - 1.0) > args.rate_tolerance:
            errors.append("sample_rate_out_of_tolerance")
        out_of_range = (temperature < model["tmin"] - 1e-9) | (temperature > model["tmax"] + 1e-9)
        row["temperature_out_of_range_samples"] = int(out_of_range.sum())
        if out_of_range.any():
            # No polynomial evaluation outside its fitted interval, even on
            # excluded rows. Raw metadata is preserved for diagnosing warmup.
            errors.append("temperature_outside_fitted_range")
        else:
            calibrated = compensate(raw, temperature, model)
            means = calibrated.mean(axis=0)
            stds = calibrated.std(axis=0, ddof=1)
            for i, axis in enumerate(AXES):
                row[f"{axis}_mean"] = float(means[i])
                row[f"{axis}_sample_std"] = float(stds[i])
                row[f"{axis}_raw_mean"] = float(raw[:, i].mean())
            row["accel_mean_norm_m_s2"] = float(np.linalg.norm(means[:3]))
            row["accel_std_vector_m_s2"] = float(np.linalg.norm(stds[:3]))
            row["gyro_std_vector_rad_s"] = float(np.linalg.norm(stds[3:]) * DEG_H_TO_RAD_S)
            centered_acc = calibrated[:, :3] - means[:3]
            row["accel_deviation_p95_m_s2"] = float(np.percentile(np.linalg.norm(centered_acc, axis=1), 95))
            if row["accel_std_vector_m_s2"] > args.max_accel_std:
                errors.append("accel_not_stationary")
            if row["gyro_std_vector_rad_s"] > args.max_gyro_std:
                errors.append("gyro_not_stationary")
            if abs(row["accel_mean_norm_m_s2"] - G0) > args.max_gravity_error:
                errors.append("accel_gravity_norm_out_of_tolerance")
            blocks = block_means(wt, calibrated, temperature, args.window_seconds, args.block_seconds)
            for block in blocks:
                block["run_index"] = row["index"]
                block["run_attempt"] = row["attempt"]
            if len(blocks) >= 2:
                bm = np.array([[b[f"{axis}_mean"] for axis in AXES] for b in blocks])
                for i, axis in enumerate(AXES):
                    row[f"{axis}_block_mean_std"] = float(bm[:, i].std(ddof=1))
                row["accel_first_last_block_difference_m_s2"] = float(np.linalg.norm(bm[-1, :3]-bm[0, :3]))
                if row["accel_first_last_block_difference_m_s2"] > args.max_block_drift:
                    warnings.append("accel_window_mean_drift")
            else:
                warnings.append("insufficient_blocks_for_stability")
            if abs(row["temp_slope_c_min"]) > args.max_temp_slope:
                warnings.append("temperature_still_changing")
        offset = (samples - samples[0]) & (UINT32 - 1)
        sync, sync_errors, sync_warnings = sync_diagnostics(session, int(samples[0]),
                                                           int(offset[first]), int(offset[last]))
        row.update(sync)
        errors.extend(sync_errors)
        warnings.extend(sync_warnings)
    except (OSError, KeyError, TypeError, ValueError, pd.errors.ParserError) as exc:
        errors.append(f"imu_read_or_validation_failed:{exc}")
    row["valid"] = not errors
    row["qc_errors"] = ";".join(errors)
    row["qc_warnings"] = ";".join(warnings)
    return row, blocks


def covariance(frame: pd.DataFrame, axes: list[str]) -> np.ndarray | None:
    if len(frame) < 2:
        return None
    return np.cov(frame[[f"{axis}_mean" for axis in axes]].to_numpy(float), rowvar=False, ddof=1)


def safe_json(value):
    """Keep output strict JSON: insufficient estimates are null, never NaN."""
    if isinstance(value, np.ndarray):
        return safe_json(value.tolist())
    if isinstance(value, np.generic):
        return safe_json(value.item())
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {str(key): safe_json(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [safe_json(item) for item in value]
    return value


def save_plots(summary: pd.DataFrame, blocks: pd.DataFrame, output: Path, mode: str) -> None:
    available = summary.dropna(subset=["ax_mean", "ay_mean", "az_mean"]) if "ax_mean" in summary else pd.DataFrame()
    fig, axes = plt.subplots(3, 1, figsize=(9, 7), sharex=True, constrained_layout=True)
    if not available.empty:
        valid = available["valid"].astype(bool).to_numpy()
        center = available.loc[valid, [f"{a}_mean" for a in AXES[:3]]].mean().to_numpy(float) if valid.any() else available[[f"{a}_mean" for a in AXES[:3]]].mean().to_numpy(float)
        for i, axis in enumerate(axes):
            x = pd.to_numeric(available["index"]).to_numpy()
            y = (available[f"{AXES[i]}_mean"].to_numpy(float)-center[i])*1000/G0
            axis.plot(x, y, "-", color="0.7", linewidth=0.8)
            axis.scatter(x[valid], y[valid], label="QC passed", color="#0072B2")
            if (~valid).any():
                axis.scatter(x[~valid], y[~valid], marker="x", color="#D55E00", label="QC failed")
            axis.set_ylabel(f"{AXES[i].upper()} relative mean (mg)")
            axis.grid(alpha=0.3)
        axes[0].legend()
    else:
        axes[1].text(.5, .5, "No calibrated means available", ha="center", transform=axes[1].transAxes)
    axes[0].set_title("Segment-mean variation (continuous IMU power)" if mode == CONTINUOUS_MODE else "Startup mean variation")
    axes[-1].set_xlabel("Run index")
    fig.savefig(output / "run_means.png", dpi=160)
    plt.close(fig)
    fig, axes = plt.subplots(1, 3, figsize=(11, 3.4), constrained_layout=True)
    if not available.empty:
        for i, axis in enumerate(axes):
            axis.scatter(available["temp_mean_c"], (available[f"{AXES[i]}_mean"]-center[i])*1000/G0,
                         c=np.where(available["valid"], "#0072B2", "#D55E00"))
            axis.set_xlabel("Window mean temperature (deg C)")
            axis.set_ylabel(f"{AXES[i].upper()} relative mean (mg)")
            axis.grid(alpha=.3)
    fig.savefig(output / "means_vs_temperature.png", dpi=160)
    plt.close(fig)
    fig, axes = plt.subplots(3, 1, figsize=(9, 7), sharex=True, constrained_layout=True)
    if not blocks.empty:
        for (index, attempt), subset in blocks.groupby(["run_index", "run_attempt"], sort=False):
            for i, axis in enumerate(axes):
                y = subset[f"{AXES[i]}_mean"].to_numpy(float)
                axis.plot((subset["start_s"]+subset["end_s"])/2, (y-y.mean())*1000/G0,
                          marker=".", label=f"Run {index}, attempt {attempt}")
    for i, axis in enumerate(axes):
        axis.set_ylabel(f"{AXES[i].upper()} block mean (mg)")
        axis.grid(alpha=.3)
    axes[0].set_title("Within-window block means (relative to each run's block average)")
    if not blocks.empty and blocks["run_index"].nunique() <= 10:
        axes[0].legend(ncol=3, fontsize=8)
    axes[-1].set_xlabel("Seconds after statistics window begins")
    fig.savefig(output / "window_stability.png", dpi=160)
    plt.close(fig)


def write_report(summary: pd.DataFrame, init: dict, model: dict, args, output: Path) -> None:
    continuous = init["mode"] == CONTINUOUS_MODE
    lines = ["# IMU 重启采集与均值重复性报告", "",
             f"实验模式：`{init['mode']}`。",
             "**IMU 连续通电，仅树莓派重启；结果是分段均值变化，不能作为 turn-on bias 重复性或滤波器初始零偏协方差。**" if continuous else
             "正式上电统计须有可核查的 IMU 断电、恢复供电记录。软件复位不等同于重新上电。",
             "", f"总轮数 {len(summary)}，质量合格 {init['valid_runs']} 轮。",
             f"统计窗口从首个已采集样本后 {args.warmup_seconds:g} s 开始，持续 {args.window_seconds:g} s。该时间零点是采集开始，并非 IMU 上电时刻。",
             f"原始域温补范围 {model['tmin']:.4f}～{model['tmax']:.4f} °C，参考温度 {model['tref']:.4f} °C。",
             "温补对每个样本先执行多项式补偿（不补 c0），再执行固定标定矩阵；超出拟合温区的统计窗口不做外推并判为无效。",
             "", "## 三轴重复性", "",
             "| 轴 | 1σ (m/s²) | 1σ (mg) |", "|---|---:|---:|"]
    for i, axis in enumerate(AXES[:3]):
        if init["accel_sigma_m_s2"] is None:
            lines.append(f"| {axis.upper()} | 不足两轮 | 不足两轮 |")
        else:
            lines.append(f"| {axis.upper()} | {init['accel_sigma_m_s2'][i]:.8g} | {init['accel_sigma_mg'][i]:.8g} |")
    lines += ["", "协方差按有效轮次均值、ddof=1 计算，单位为 (m/s²)²，**不再除以轮数**。",
              "它包含窗口均值噪声、残余温漂和微小姿态变化。段内分块均值仅辅助判断稳定性，不按独立白噪声假设扣除。",
              "固定姿态的重力投影未知，因此本报告不把静态加速度均值当成绝对零偏，也不生成绝对初始零偏均值。",
              "", "## 是否可用于滤波器初始化", "",
              f"`suitable_for_filter_initialization = {str(init['suitable_for_filter_initialization']).lower()}`。",
              "判定条件：正式 IMU 上电模式、每轮断电记录已验证、至少 30 轮有效数据、下载文件指纹和同步诊断可核查、全批次无显著质量警告。",
              "实际使用还需要确认刚性固定姿态、启动等待时间、冷热启动条件及滤波器零偏状态坐标系与本实验一致。"]
    if init["valid_runs"] < 30:
        lines += ["", "**本批次属于流程试跑或小样本检查，协方差估计波动大，不能作为正式初始不确定度。三轮数据的三轴协方差秩最多为 2。**"]
    if init["initialization_rejection_reasons"]:
        lines += ["", "未满足条件：" + "；".join(init["initialization_rejection_reasons"]) + "。"]
    lines += ["", "## 每轮质量结果", "", "| 轮次 | 尝试 | 状态 | 有效 | 错误 | 警告 |", "|---|---|---|---|---|---|"]
    for _, row in summary.iterrows():
        cells = [row["index"], row["attempt"], row["acquisition_status"], "是" if row["valid"] else "否", row.get("qc_errors", ""), row.get("qc_warnings", "")]
        lines.append("| " + " | ".join(str(cell).replace("|", "/").replace("\n", " ") for cell in cells) + " |")
    lines += ["", "不按均值偏离程度自动剔除轮次，全部采集结果与质量原因保留。",
              "不同轮次的姿态改变无法仅从加速度均值与真实零偏变化可靠区分，必须靠实验固定条件控制。",
              "", "## 输出文件", "",
              "- `runs_summary.csv`：每轮原始/补偿均值、采样率、温度、样本序列和同步诊断。",
              "- `bias_covariance.csv`：标定后加速度计三轴协方差。",
              "- `gyro_bias_covariance.csv`：辅助陀螺均值协方差，单位 (rad/s)²。",
              "- `filter_init.json`：估计、单位、坐标系、初始化适用性和配置指纹。",
              "- `manifest_snapshot.json`：本次分析开始时实验清单的逐字节快照，避免后续状态更新影响追溯。",
              "- `block_means.csv` / `window_stability.png`：段内固定时间分块均值。",
              "- `run_means.png` / `means_vs_temperature.png`：轮次和温度关系。"]
    (output / "重复性报告.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--experiment", type=Path, help="experiment containing manifest.json")
    parser.add_argument("--coeff", type=Path, default=ROOT / "data/calib24/temp_coeffs_raw.csv")
    parser.add_argument("--calib", type=Path, default=ROOT / "data/calib24/calib24_result_tempcomp_azgxgy.mat")
    parser.add_argument("--check-calibration", action="store_true", help="verify model and exit; no experiment required")
    parser.add_argument("--warmup-seconds", type=float, default=None)
    parser.add_argument("--window-seconds", type=float, default=None)
    parser.add_argument("--block-seconds", type=float, default=10)
    parser.add_argument("--expected-rate-hz", type=float, default=100)
    parser.add_argument("--rate-tolerance", type=float, default=.10)
    parser.add_argument("--min-coverage", type=float, default=.99)
    parser.add_argument("--max-accel-std", type=float, default=.15, help="vector sample std, m/s²")
    parser.add_argument("--max-gyro-std", type=float, default=.02, help="vector sample std, rad/s")
    parser.add_argument("--max-gravity-error", type=float, default=.5, help="abs(norm(mean accel)-g), m/s²")
    parser.add_argument("--max-block-drift", type=float, default=.03, help="first-last accel block mean difference, m/s²; warning only")
    parser.add_argument("--max-temp-slope", type=float, default=.2, help="absolute temperature slope, °C/min; warning only")
    parser.add_argument("--output", type=Path, help="defaults to experiment/analysis")
    args = parser.parse_args()
    try:
        model = load_model(args.coeff.resolve(), args.calib.resolve())
        if args.check_calibration:
            print(json.dumps({"status": "ok", "domain": "raw", "Tref": model["tref"],
                              "Tmin": model["tmin"], "Tmax": model["tmax"],
                              "active_axes": [AXES[i] for i in np.flatnonzero(model["active"])],
                              "order": model["orders"].tolist(), "compensate_c0": False}, ensure_ascii=False))
            return 0
        if args.experiment is None:
            parser.error("--experiment is required unless --check-calibration is used")
        experiment = args.experiment.resolve()
        manifest_bytes = (experiment / "manifest.json").read_bytes()
        manifest = json.loads(manifest_bytes.decode("utf-8-sig"))
        if not isinstance(manifest.get("runs"), list) or not manifest["runs"]:
            raise ValueError("manifest.runs must be a nonempty array")
        if any(not isinstance(run, dict) for run in manifest["runs"]):
            raise ValueError("manifest.runs entries must be objects")
        identities = [(run.get("index"), run.get("attempt", 1)) for run in manifest["runs"]]
        if any(not isinstance(value, int) or isinstance(value, bool) or value < 1
               for identity in identities for value in identity) or len(set(identities)) != len(identities):
            raise ValueError("each run needs a unique (positive integer index, positive integer attempt)")
        settings = manifest.get("settings", {})
        args.warmup_seconds = args.warmup_seconds if args.warmup_seconds is not None else float(settings.get("warmup_seconds", 30))
        args.window_seconds = args.window_seconds if args.window_seconds is not None else float(settings.get("window_seconds", 120))
        if not math.isfinite(args.warmup_seconds) or args.warmup_seconds < 0:
            raise ValueError("warmup must be finite and nonnegative")
        for name in ["window_seconds", "block_seconds", "expected_rate_hz", "rate_tolerance", "max_accel_std", "max_gyro_std", "max_gravity_error", "max_block_drift", "max_temp_slope"]:
            if not math.isfinite(getattr(args, name)) or getattr(args, name) <= 0:
                raise ValueError(f"{name} must be finite and positive")
        if not 0 < args.min_coverage <= 1:
            raise ValueError("min coverage must lie in (0,1]")
        mode = str(manifest.get("mode", "unknown"))
        output = (args.output or experiment / "analysis").resolve()
        output.mkdir(parents=True, exist_ok=True)
        snapshot_file = output / "manifest_snapshot.json"
        snapshot_file.write_bytes(manifest_bytes)
        rows, all_blocks = [], []
        for run in manifest["runs"]:
            row, blocks = analyze_run(run, experiment, model, args)
            rows.append(row)
            all_blocks.extend(blocks)
            print(f"run {row['index']} attempt {row['attempt']}: {'valid' if row['valid'] else 'invalid'} {row['qc_errors']}", flush=True)
        summary = pd.DataFrame(rows)
        blocks = pd.DataFrame(all_blocks, columns=["run_index", "run_attempt", "block", "start_s", "end_s",
                                                 "samples", "temp_mean_c", *[f"{axis}_mean" for axis in AXES]])
        valid = summary.loc[summary["valid"].astype(bool)]
        if valid["index"].duplicated().any():
            raise ValueError("multiple QC-valid attempts of one run cannot be counted as independent startups")
        summary.to_csv(output / "runs_summary.csv", index=False, encoding="utf-8-sig")
        blocks.to_csv(output / "block_means.csv", index=False, encoding="utf-8-sig")
        acc_cov = covariance(valid, AXES[:3])
        gyro_cov = covariance(valid, AXES[3:])
        if gyro_cov is not None:
            gyro_cov *= DEG_H_TO_RAD_S**2
        for name, axes, cov in [("bias_covariance.csv", AXES[:3], acc_cov), ("gyro_bias_covariance.csv", AXES[3:], gyro_cov)]:
            pd.DataFrame(cov if cov is not None else np.full((3, 3), np.nan), index=axes, columns=axes).to_csv(output / name, index_label="axis", encoding="utf-8-sig")
        reasons = []
        if mode != POWER_CYCLE_MODE:
            reasons.append("未执行经过验证的 IMU 真正断电再上电")
        if len(valid) < 30:
            reasons.append("有效轮数不足 30，属于试跑或小样本")
        if not len(valid) or not valid["power_cycle_verified"].all():
            reasons.append("有效轮次缺少可核查的断电上电记录")
        if not len(valid) or "sync_available" not in valid or not valid["sync_available"].fillna(False).all():
            reasons.append("同步诊断缺失或无法对齐")
        if not len(valid) or "data_hashes_verified" not in valid or not valid["data_hashes_verified"].fillna(False).all():
            reasons.append("下载数据缺少可核查指纹")
        if len(valid) and valid["qc_warnings"].str.len().gt(0).any():
            reasons.append("有效轮次存在质量或稳定性警告")
        acc_sigma = np.sqrt(np.maximum(np.diag(acc_cov), 0)) if acc_cov is not None else None
        gyro_sigma = np.sqrt(np.maximum(np.diag(gyro_cov), 0)) if gyro_cov is not None else None
        init = {"schema_version": 1, "generated_utc": datetime.now(timezone.utc).isoformat(),
                "mode": mode, "estimate_kind": "continuous_power_segment_mean_covariance" if mode == CONTINUOUS_MODE else "startup_mean_covariance",
                "suitable_for_filter_initialization": not reasons,
                "initialization_rejection_reasons": reasons, "valid_runs": len(valid), "total_runs": len(summary),
                "valid_run_indices": valid["index"].tolist(),
                "valid_run_attempts": valid[["index", "attempt"]].to_dict(orient="records"),
                "accel_state_coordinate_frame": "fixed calibrated accelerometer sensor frame [ax, ay, az]",
                "accel_covariance_unit": "(m/s^2)^2", "accel_sigma_unit": "m/s^2",
                "accel_covariance_m_s2_squared": acc_cov,
                "accel_covariance_diagonal_m_s2_squared": np.diag(acc_cov) if acc_cov is not None else None,
                "accel_sigma_m_s2": acc_sigma, "accel_sigma_mg": acc_sigma * 1000/G0 if acc_sigma is not None else None,
                "gyro_covariance_unit": "(rad/s)^2", "gyro_covariance_rad_s_squared": gyro_cov,
                "gyro_sigma_rad_s": gyro_sigma, "gyro_sigma_deg_h": gyro_sigma / DEG_H_TO_RAD_S if gyro_sigma is not None else None,
                "bias_initial_mean": None, "static_mean_contains_gravity": True,
                "sample_covariance_ddof": 1, "divide_covariance_by_run_count": False,
                "warmup_seconds_from_first_recorded_sample": args.warmup_seconds, "window_seconds": args.window_seconds,
                "window_time_origin": "first recorded IMU sample; not verified IMU power-on time",
                "processing_order": "raw -> per-sample temperature polynomial excluding c0 -> fixed calibration",
                "temperature_fit_range_c": [model["tmin"], model["tmax"]], "temperature_reference_c": model["tref"],
                "temperature_active_axes": [AXES[i] for i in np.flatnonzero(model["active"])],
                "calibration_input_sha256": model["input_sha256"],
                "manifest_snapshot_file": str(snapshot_file), "manifest_sha256": hashlib.sha256(manifest_bytes).hexdigest(),
                "qc_thresholds": {key: getattr(args, key) for key in ["block_seconds", "expected_rate_hz", "rate_tolerance", "min_coverage", "max_accel_std", "max_gyro_std", "max_gravity_error", "max_block_drift", "max_temp_slope"]},
                "experiment_assumptions_to_verify": ["rigid fixed pose", "startup timing representative of filter use", "cold/warm startup conditions representative", "calibrated sensor frame matches filter bias state"],
                "limitations": ["finite-window mean noise and residual thermal drift retained", "pose changes indistinguishable from bias without a reference", "small pilot sample is not a final uncertainty estimate"]}
        init = safe_json(init)
        (output / "filter_init.json").write_text(json.dumps(init, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
        save_plots(summary, blocks, output, mode)
        write_report(summary, init, model, args, output)
        print(f"Analysis outputs: {output}")
        print(f"Valid runs: {len(valid)}; suitable for filter initialization: {init['suitable_for_filter_initialization']}")
        return 0
    except (OSError, KeyError, TypeError, ValueError, pd.errors.ParserError) as exc:
        parser.exit(2, f"Analysis failed: {exc}\n")


if __name__ == "__main__":
    raise SystemExit(main())
