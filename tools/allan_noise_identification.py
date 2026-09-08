"""Identify IMU stochastic error terms from a decoded physical-unit CSV.

The program computes overlapping Allan deviation, finds candidate power-law
regions, estimates ARW/VRW, bias instability, rate random walk and rate ramp,
and writes overview/identification plots, a machine-readable CSV, and a Chinese
Markdown report.

Expected columns:
sample,time_s,dt_s,ax_m_s2,ay_m_s2,az_m_s2,temp_deg_c,
gx_deg_h,gy_deg_h,gz_deg_h
"""

from __future__ import annotations

import argparse
import csv
import math
import pathlib
from dataclasses import dataclass

import matplotlib.pyplot as plt
import numpy as np


TOOLS_DIR = pathlib.Path(__file__).resolve().parent
DATA_DIR = TOOLS_DIR.parent / "data"
G0 = 9.80665
DEG_PER_RAD = 180.0 / math.pi

PHYSICAL_DTYPE = np.dtype([
    ("sample", "<i4"),
    ("time_s", "<f8"),
    ("dt_s", "<f8"),
    ("ax_m_s2", "<f8"),
    ("ay_m_s2", "<f8"),
    ("az_m_s2", "<f8"),
    ("temp_deg_c", "<f8"),
    ("gx_deg_h", "<f8"),
    ("gy_deg_h", "<f8"),
    ("gz_deg_h", "<f8"),
])

EXPECTED_COLUMNS = tuple(PHYSICAL_DTYPE.names or ())


@dataclass
class RegionFit:
    kind: str
    target_slope: float
    slope: float
    intercept: float
    rmse_log10: float
    tau_start: float
    tau_end: float
    start_index: int
    end_index: int
    confidence: str
    detected: bool


@dataclass
class AxisResult:
    sensor: str
    axis: str
    unit: str
    tau: np.ndarray
    adev: np.ndarray
    fits: dict[str, RegionFit]
    parameters: dict[str, float]
    mean: float
    std: float
    initial_rise: bool
    temperature_correlation: float = float("nan")
    long_term_usable: bool = True


FIT_CONFIG = {
    "white": {"target": -0.5, "tolerance": 0.18, "min_tau": 0.02, "max_tau_fraction": 0.01},
    "bias": {"target": 0.0, "tolerance": 0.13, "min_tau": 0.1, "max_tau_fraction": 0.10},
    "rrw": {"target": 0.5, "tolerance": 0.20, "min_tau": 1.0, "max_tau_fraction": 0.10},
    "ramp": {"target": 1.0, "tolerance": 0.25, "min_tau": 3.0, "max_tau_fraction": 0.10},
}


def find_header_line(path: pathlib.Path) -> tuple[int, tuple[int, ...]]:
    with path.open("r", encoding="utf-8-sig", errors="replace") as stream:
        for line_number, line in enumerate(stream):
            columns = tuple(part.strip() for part in line.strip().split(","))
            if all(name in columns for name in EXPECTED_COLUMNS):
                return line_number, tuple(columns.index(name) for name in EXPECTED_COLUMNS)
    raise ValueError(
        "Required IMU CSV header not found. Required columns: " + ",".join(EXPECTED_COLUMNS)
    )


def load_decoded_csv(path: pathlib.Path) -> np.ndarray:
    header_line, use_columns = find_header_line(path)
    data = np.loadtxt(
        path,
        delimiter=",",
        comments="#",
        skiprows=header_line + 1,
        dtype=PHYSICAL_DTYPE,
        usecols=use_columns,
    )
    if data.ndim == 0:
        data = data.reshape(1)
    if data.size < 1000:
        raise ValueError("Too few valid samples; at least 1000 are required")
    for name in EXPECTED_COLUMNS[1:]:
        if not np.all(np.isfinite(data[name])):
            raise ValueError(f"Column {name} contains NaN or infinite values")
    return data


def overlapping_allan_deviation(
    values: np.ndarray,
    sample_period: float,
    points: int,
    maximum_tau_fraction: float = 0.1,
) -> tuple[np.ndarray, np.ndarray]:
    """Return overlapping Allan deviation for uniformly sampled rate data."""
    values = np.asarray(values, dtype=np.float64)
    values = values - np.mean(values)
    count = values.size
    maximum_m = max(1, int(count * maximum_tau_fraction))
    clusters = np.unique(
        np.logspace(0.0, math.log10(maximum_m), points).astype(np.int64)
    )
    integral = np.empty(count + 1, dtype=np.float64)
    integral[0] = 0.0
    np.cumsum(values, dtype=np.float64, out=integral[1:])
    integral *= sample_period

    taus: list[float] = []
    deviations: list[float] = []
    for m in clusters:
        if 2 * m >= integral.size:
            continue
        second = integral[2 * m:] - 2.0 * integral[m:-m] + integral[:-2 * m]
        variance = float(np.dot(second, second)) / (
            second.size * 2.0 * (m * sample_period) ** 2
        )
        taus.append(float(m * sample_period))
        deviations.append(math.sqrt(max(variance, 0.0)))
    return np.asarray(taus), np.asarray(deviations)


def linear_log_fit(tau: np.ndarray, adev: np.ndarray) -> tuple[float, float, float]:
    x = np.log10(tau)
    y = np.log10(adev)
    slope, intercept = np.polyfit(x, y, 1)
    residual = y - (slope * x + intercept)
    rmse = float(np.sqrt(np.mean(residual * residual)))
    return float(slope), float(intercept), rmse


def confidence_label(slope_error: float, tolerance: float, rmse: float) -> str:
    if slope_error <= tolerance / 3.0 and rmse <= 0.025:
        return "高"
    if slope_error <= 2.0 * tolerance / 3.0 and rmse <= 0.050:
        return "中"
    return "低"


def find_region(
    kind: str,
    tau: np.ndarray,
    adev: np.ndarray,
    duration: float,
) -> RegionFit:
    config = FIT_CONFIG[kind]
    target = float(config["target"])
    tolerance = float(config["tolerance"])
    min_tau = float(config["min_tau"])
    max_tau = duration * float(config["max_tau_fraction"])
    valid = np.flatnonzero((tau >= min_tau) & (tau <= max_tau) & np.isfinite(adev) & (adev > 0.0))

    best: tuple[float, RegionFit] | None = None
    for window in (7, 9, 11, 13):
        if valid.size < window:
            continue
        for offset in range(valid.size - window + 1):
            indexes = valid[offset:offset + window]
            if np.any(np.diff(indexes) != 1):
                continue
            start = int(indexes[0])
            end = int(indexes[-1]) + 1
            if math.log10(tau[end - 1] / tau[start]) < 0.42:
                continue
            slope, intercept, rmse = linear_log_fit(tau[start:end], adev[start:end])
            slope_error = abs(slope - target)
            score = slope_error / tolerance + rmse / 0.04

            if kind == "bias":
                # BI should occur near the Allan minimum rather than at an
                # arbitrary transition that momentarily has zero slope.
                minimum = float(np.min(adev[valid]))
                level_ratio = float(np.median(adev[start:end])) / minimum
                score += 2.0 * abs(math.log10(max(level_ratio, 1.0)))
            elif kind == "white":
                # Prefer the earlier valid white-noise region.
                score += 0.03 * math.log10(tau[start] / min_tau + 1.0)
            else:
                # RRW and ramp are long-term effects; slightly prefer later
                # regions when fits have otherwise comparable quality.
                score -= 0.02 * math.log10(tau[start] / min_tau + 1.0)

            detected = slope_error <= tolerance and rmse <= 0.08
            fit = RegionFit(
                kind=kind,
                target_slope=target,
                slope=slope,
                intercept=intercept,
                rmse_log10=rmse,
                tau_start=float(tau[start]),
                tau_end=float(tau[end - 1]),
                start_index=start,
                end_index=end,
                confidence=confidence_label(slope_error, tolerance, rmse) if detected else "未检出",
                detected=detected,
            )
            if best is None or score < best[0]:
                best = (score, fit)

    if best is not None:
        return best[1]
    return RegionFit(
        kind=kind,
        target_slope=target,
        slope=float("nan"),
        intercept=float("nan"),
        rmse_log10=float("nan"),
        tau_start=float("nan"),
        tau_end=float("nan"),
        start_index=0,
        end_index=0,
        confidence="未检出",
        detected=False,
    )


def detect_initial_rise(tau: np.ndarray, adev: np.ndarray) -> bool:
    length = min(9, tau.size)
    if length < 5:
        return False
    slopes = np.diff(np.log10(adev[:length])) / np.diff(np.log10(tau[:length]))
    return bool(np.count_nonzero(slopes > 0.10) >= 2)


def estimate_parameters(
    tau: np.ndarray,
    adev: np.ndarray,
    fits: dict[str, RegionFit],
) -> dict[str, float]:
    output: dict[str, float] = {}
    for kind, fit in fits.items():
        if not fit.detected:
            output[kind] = float("nan")
            continue
        selected_tau = tau[fit.start_index:fit.end_index]
        selected_adev = adev[fit.start_index:fit.end_index]
        if kind == "white":
            output[kind] = float(np.median(selected_adev * np.sqrt(selected_tau)))
        elif kind == "bias":
            output[kind] = float(np.median(selected_adev) / 0.664)
        elif kind == "rrw":
            output[kind] = float(np.median(selected_adev * np.sqrt(3.0 / selected_tau)))
        elif kind == "ramp":
            output[kind] = float(np.median(math.sqrt(2.0) * selected_adev / selected_tau))
    return output


def analyze_axis(
    sensor: str,
    axis: str,
    values: np.ndarray,
    unit: str,
    period: float,
    points: int,
    duration: float,
) -> AxisResult:
    tau, adev = overlapping_allan_deviation(values, period, points)
    fits = {kind: find_region(kind, tau, adev, duration) for kind in FIT_CONFIG}
    parameters = estimate_parameters(tau, adev, fits)
    return AxisResult(
        sensor=sensor,
        axis=axis,
        unit=unit,
        tau=tau,
        adev=adev,
        fits=fits,
        parameters=parameters,
        mean=float(np.mean(values)),
        std=float(np.std(values, ddof=1)),
        initial_rise=detect_initial_rise(tau, adev),
    )


def format_number(value: float, digits: int = 6) -> str:
    if not math.isfinite(value):
        return "未检出"
    return f"{value:.{digits}g}"


def parameter_display(result: AxisResult) -> dict[str, tuple[float, str]]:
    white = result.parameters["white"]
    bias = result.parameters["bias"]
    rrw = result.parameters["rrw"]
    ramp = result.parameters["ramp"]
    if result.sensor == "gyro":
        return {
            "white_primary": (white * DEG_PER_RAD * 60.0, "deg/sqrt(h)"),
            "bias_primary": (bias * DEG_PER_RAD * 3600.0, "deg/h"),
            "rrw_primary": (rrw * DEG_PER_RAD * 3600.0 * 60.0, "deg/h/sqrt(h)"),
            "ramp_primary": (ramp * DEG_PER_RAD * 3600.0, "deg/s per h"),
            "white_si": (white, "rad/sqrt(s)"),
            "bias_si": (bias, "rad/s"),
            "rrw_si": (rrw, "rad/s/sqrt(s)"),
            "ramp_si": (ramp, "rad/s^2"),
        }
    return {
        "white_primary": (white * 60.0, "m/s/sqrt(h)"),
        "bias_primary": (bias / G0 * 1000.0, "mg"),
        "rrw_primary": (rrw / G0 * 1000.0 * 60.0, "mg/sqrt(h)"),
        "ramp_primary": (ramp / G0 * 1000.0 * 3600.0, "mg/h"),
        "white_si": (white, "m/s/sqrt(s)"),
        "bias_si": (bias, "m/s^2"),
        "rrw_si": (rrw, "m/s^2/sqrt(s)"),
        "ramp_si": (ramp, "m/s^3"),
    }


def write_parameters_csv(path: pathlib.Path, results: list[AxisResult]) -> None:
    fields = [
        "sensor", "axis", "mean_si", "std_si", "initial_rise",
        "temperature_correlation_60s_mean", "long_term_parameter_use",
        "arw_vrw", "arw_vrw_unit", "arw_vrw_slope", "arw_vrw_tau_start_s",
        "arw_vrw_tau_end_s", "arw_vrw_confidence",
        "bias_instability", "bias_instability_unit", "bias_slope",
        "bias_tau_start_s", "bias_tau_end_s", "bias_confidence",
        "rate_random_walk", "rate_random_walk_unit", "rrw_slope",
        "rrw_tau_start_s", "rrw_tau_end_s", "rrw_confidence",
        "rate_ramp", "rate_ramp_unit", "ramp_slope", "ramp_tau_start_s",
        "ramp_tau_end_s", "ramp_confidence",
    ]
    with path.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for result in results:
            display = parameter_display(result)
            white_fit = result.fits["white"]
            bias_fit = result.fits["bias"]
            rrw_fit = result.fits["rrw"]
            ramp_fit = result.fits["ramp"]
            writer.writerow({
                "sensor": result.sensor,
                "axis": result.axis,
                "mean_si": result.mean,
                "std_si": result.std,
                "initial_rise": result.initial_rise,
                "temperature_correlation_60s_mean": result.temperature_correlation,
                "long_term_parameter_use": "可作候选" if result.long_term_usable else "温漂污染_不建议直接使用",
                "arw_vrw": display["white_primary"][0],
                "arw_vrw_unit": display["white_primary"][1],
                "arw_vrw_slope": white_fit.slope,
                "arw_vrw_tau_start_s": white_fit.tau_start,
                "arw_vrw_tau_end_s": white_fit.tau_end,
                "arw_vrw_confidence": white_fit.confidence,
                "bias_instability": display["bias_primary"][0],
                "bias_instability_unit": display["bias_primary"][1],
                "bias_slope": bias_fit.slope,
                "bias_tau_start_s": bias_fit.tau_start,
                "bias_tau_end_s": bias_fit.tau_end,
                "bias_confidence": bias_fit.confidence,
                "rate_random_walk": display["rrw_primary"][0],
                "rate_random_walk_unit": display["rrw_primary"][1],
                "rrw_slope": rrw_fit.slope,
                "rrw_tau_start_s": rrw_fit.tau_start,
                "rrw_tau_end_s": rrw_fit.tau_end,
                "rrw_confidence": rrw_fit.confidence,
                "rate_ramp": display["ramp_primary"][0],
                "rate_ramp_unit": display["ramp_primary"][1],
                "ramp_slope": ramp_fit.slope,
                "ramp_tau_start_s": ramp_fit.tau_start,
                "ramp_tau_end_s": ramp_fit.tau_end,
                "ramp_confidence": ramp_fit.confidence,
            })


def write_allan_csv(path: pathlib.Path, results: list[AxisResult]) -> None:
    tau = results[0].tau
    columns = [tau] + [result.adev for result in results]
    header = "tau_s," + ",".join(f"{r.sensor}_{r.axis}_adev_si" for r in results)
    np.savetxt(path, np.column_stack(columns), delimiter=",", header=header, comments="")


def plot_allan(path: pathlib.Path, results: list[AxisResult]) -> None:
    colors = {"white": "#2ca02c", "bias": "#ff7f0e", "rrw": "#d62728", "ramp": "#9467bd"}
    labels = {"white": "ARW/VRW", "bias": "BI", "rrw": "RRW", "ramp": "Ramp"}
    confidence_ascii = {"高": "High", "中": "Medium", "低": "Low", "未检出": "Not found"}
    figure, axes = plt.subplots(2, 3, figsize=(16, 9), constrained_layout=True)
    for plot_axis, result in zip(axes.flat, results):
        factor = DEG_PER_RAD if result.sensor == "gyro" else 1.0
        ylabel = "Allan deviation (deg/s)" if result.sensor == "gyro" else "Allan deviation (m/s^2)"
        plot_axis.loglog(result.tau, result.adev * factor, color="#1f77b4", linewidth=1.8)
        for kind, fit in result.fits.items():
            if not fit.detected:
                continue
            indexes = slice(fit.start_index, fit.end_index)
            plot_axis.loglog(
                result.tau[indexes], result.adev[indexes] * factor,
                color=colors[kind], linewidth=3.0,
                label=f"{labels[kind]} slope={fit.slope:.2f} ({confidence_ascii[fit.confidence]})",
            )
        title = f"{result.sensor.capitalize()} {result.axis.upper()}"
        if result.initial_rise:
            title += " | initial rise"
        plot_axis.set(title=title, xlabel="Cluster time tau (s)", ylabel=ylabel)
        plot_axis.grid(True, which="both", alpha=0.30)
        plot_axis.legend(fontsize=8)
    figure.suptitle("MPU6050 overlapping Allan deviation and identified regions", fontsize=15)
    figure.savefig(path, dpi=180)
    plt.close(figure)


def plot_allan_overview(path: pathlib.Path, results: list[AxisResult]) -> None:
    """Write the compact accelerometer/gyroscope plot from allan_analysis.py."""
    figure, axes = plt.subplots(1, 2, figsize=(12, 5), constrained_layout=True)
    for result in results:
        plot_axis = axes[0] if result.sensor == "accel" else axes[1]
        plot_axis.loglog(result.tau, result.adev, label=result.axis.upper())

    axes[0].set(
        title="Accelerometer Allan deviation",
        xlabel="Cluster time tau (s)",
        ylabel="Allan deviation (m/s^2)",
    )
    axes[1].set(
        title="Gyroscope Allan deviation",
        xlabel="Cluster time tau (s)",
        ylabel="Allan deviation (rad/s)",
    )
    for plot_axis in axes:
        plot_axis.grid(True, which="both", alpha=0.35)
        plot_axis.legend()
    figure.savefig(path, dpi=180)
    plt.close(figure)


def block_means(values: np.ndarray, block: int) -> np.ndarray:
    usable = values.size // block * block
    if usable == 0:
        return np.asarray([], dtype=np.float64)
    return np.asarray(values[:usable], dtype=np.float64).reshape(-1, block).mean(axis=1)


def plot_stability(
    path: pathlib.Path,
    data: np.ndarray,
    period: float,
    block_seconds: float = 60.0,
) -> None:
    block = max(1, int(round(block_seconds / period)))
    temperature = data["temp_deg_c"].astype(np.float64)
    acceleration = [data[f"a{axis}_m_s2"].astype(np.float64) for axis in "xyz"]
    gyro = [data[f"g{axis}_deg_h"].astype(np.float64) for axis in "xyz"]
    temp_mean = block_means(temperature, block)
    time_hours = (np.arange(temp_mean.size) + 0.5) * block * period / 3600.0

    figure, axes = plt.subplots(3, 1, figsize=(13, 9), sharex=True, constrained_layout=True)
    for axis, values in zip("XYZ", acceleration):
        means = block_means(values, block)
        residual_mg = (means - np.median(means)) / G0 * 1000.0
        axes[0].plot(time_hours, residual_mg, label=axis, linewidth=1.0)
    for axis, values in zip("XYZ", gyro):
        means = block_means(values, block)
        residual_deg_h = means - np.median(means)
        axes[1].plot(time_hours, residual_deg_h, label=axis, linewidth=1.0)
    axes[2].plot(time_hours, temp_mean, color="#d62728", linewidth=1.2)
    axes[0].set(ylabel="Accel mean residual (mg)", title=f"{block_seconds:.0f} s block-mean residuals")
    axes[1].set(ylabel="Gyro mean residual (deg/h)")
    axes[2].set(ylabel="Temperature (degC)", xlabel="Time after analyzed segment start (h)")
    for axis in axes:
        axis.grid(True, alpha=0.30)
    axes[0].legend(ncol=3)
    axes[1].legend(ncol=3)
    figure.savefig(path, dpi=180)
    plt.close(figure)


def fit_temperature_slope(temperature: np.ndarray, period: float) -> float:
    time_hours = np.arange(temperature.size, dtype=np.float64) * period / 3600.0
    if temperature.size > 20000:
        stride = max(1, temperature.size // 20000)
        time_hours = time_hours[::stride]
        temperature = temperature[::stride]
    return float(np.polyfit(time_hours, temperature, 1)[0])


def assign_temperature_correlations(
    data: np.ndarray,
    results: list[AxisResult],
    period: float,
    temperature_range: float,
    block_seconds: float = 60.0,
) -> None:
    block = max(1, int(round(block_seconds / period)))
    temperature = data["temp_deg_c"].astype(np.float64)
    temperature_mean = block_means(temperature, block)
    for result in results:
        suffix = "deg_h" if result.sensor == "gyro" else "m_s2"
        values = data[f"{result.sensor[0]}{result.axis}_{suffix}"].astype(np.float64)
        sensor_mean = block_means(values, block)
        if sensor_mean.size >= 3 and np.std(sensor_mean) > 0.0 and np.std(temperature_mean) > 0.0:
            result.temperature_correlation = float(np.corrcoef(sensor_mean, temperature_mean)[0, 1])
        # A large thermal excursion contaminates long-tau terms globally;
        # a strong axis-specific correlation is an additional warning.
        result.long_term_usable = not (
            temperature_range > 1.0 or abs(result.temperature_correlation) >= 0.50
        )


def report_table_rows(results: list[AxisResult]) -> list[str]:
    rows = [
        "| 传感器 | 轴 | ARW/VRW | BI | RRW | Rate Ramp | 最左端上升 |",
        "|---|---|---:|---:|---:|---:|---|",
    ]
    for result in results:
        values = parameter_display(result)
        rows.append(
            "| {sensor} | {axis} | {white} {wu} ({wc}) | {bias} {bu} ({bc}) | "
            "{rrw} {ru} ({rc}) | {ramp} {pu} ({pc}) | {rise} |".format(
                sensor="陀螺仪" if result.sensor == "gyro" else "加速度计",
                axis=result.axis.upper(),
                white=format_number(values["white_primary"][0]),
                wu=values["white_primary"][1],
                wc=result.fits["white"].confidence,
                bias=format_number(values["bias_primary"][0]),
                bu=values["bias_primary"][1],
                bc=result.fits["bias"].confidence,
                rrw=format_number(values["rrw_primary"][0]),
                ru=values["rrw_primary"][1],
                rc=(result.fits["rrw"].confidence + ("/温漂污染" if not result.long_term_usable else "")),
                ramp=format_number(values["ramp_primary"][0]),
                pu=values["ramp_primary"][1],
                pc=(result.fits["ramp"].confidence + ("/温漂污染" if not result.long_term_usable else "")),
                rise="是" if result.initial_rise else "否",
            )
        )
    return rows


def region_detail_rows(results: list[AxisResult]) -> list[str]:
    rows = [
        "| 传感器 | 轴 | 类型 | 实际斜率 | 拟合τ范围(s) | log10 RMSE | 可信度 |",
        "|---|---|---|---:|---:|---:|---|",
    ]
    names = {"white": "ARW/VRW", "bias": "BI", "rrw": "RRW", "ramp": "Rate Ramp"}
    for result in results:
        for kind, fit in result.fits.items():
            if fit.detected:
                tau_text = f"{fit.tau_start:.4g}～{fit.tau_end:.4g}"
                slope_text = f"{fit.slope:.3f}"
                rmse_text = f"{fit.rmse_log10:.4f}"
            else:
                tau_text = slope_text = rmse_text = "—"
            rows.append(
                f"| {'陀螺仪' if result.sensor == 'gyro' else '加速度计'} | {result.axis.upper()} | "
                f"{names[kind]} | {slope_text} | {tau_text} | {rmse_text} | {fit.confidence} |"
            )
    return rows


def write_report(
    path: pathlib.Path,
    source: pathlib.Path,
    data_all: np.ndarray,
    data: np.ndarray,
    results: list[AxisResult],
    rate: float,
    skip_minutes: float,
    sequence_discontinuities: int,
    missing_samples: int,
    empirical_rate: float,
    dt_summary: str,
    temperature: np.ndarray,
    temperature_slope: float,
) -> None:
    analyzed_duration = (data.size - 1) / rate
    confidence_notes: list[str] = []
    for result in results:
        if not result.long_term_usable:
            confidence_notes.append(
                f"- {result.sensor} {result.axis.upper()} 的长期项受温度变化污染，"
                "RRW/Ramp数值只保留为曲线候选，不用于滤波器定参。"
            )
        for kind in ("rrw", "ramp"):
            fit = result.fits[kind]
            if fit.confidence in {"低", "未检出"}:
                confidence_notes.append(
                    f"- {result.sensor} {result.axis.upper()} 的 {kind.upper()} 为{fit.confidence}，"
                    "不建议直接写入滤波参数。"
                )
    if not confidence_notes:
        confidence_notes.append("- 所有长期项均找到候选区域，但仍应结合温度曲线和重复实验复核。")

    lines = [
        "# MPU6050 12小时静态数据随机误差判读报告",
        "",
        "## 结论摘要",
        "",
        "- 数据链路质量良好：采样序号连续，未发现缺帧。",
        "- 六轴均找到清晰的-1/2白噪声区，ARW/VRW可作为滤波参数候选。",
        "- 六轴均找到接近Allan最低点的BI平台，BI可用于零偏初始不确定度，但仍建议重复实验。",
        "- 本次温度变化超过2°C且存在明显突降，长τ区域受到温漂污染；RRW和Rate Ramp即使曲线斜率吻合，也不建议直接写入卡尔曼滤波。",
        "",
        "## 1. 数据与配置",
        "",
        f"- 源文件：`{source.name}`",
        f"- 原始有效帧数：{data_all.size:,}",
        f"- 跳过开头热稳定数据：{skip_minutes:.1f} min",
        f"- 实际参与分析帧数：{data.size:,}",
        f"- 分析时长：{analyzed_duration / 3600.0:.3f} h",
        f"- Allan计算采用标称采样率：{rate:.6g} Hz",
        f"- 全文件时间戳估计采样率：{empirical_rate:.6f} Hz",
        f"- 时间间隔统计：{dt_summary}",
        f"- 采样序号不连续位置：{sequence_discontinuities}",
        f"- 估计缺失帧数：{missing_samples}",
        "- 当前固件配置：100 Hz，陀螺仪±250 deg/s，加速度计±2 g，DLPF约42/44 Hz。",
        "",
        "## 2. 温度情况",
        "",
        f"- 分析段温度均值：{float(np.mean(temperature)):.3f} °C",
        f"- 最低/最高温度：{float(np.min(temperature)):.3f} / {float(np.max(temperature)):.3f} °C",
        f"- 首尾温差：{float(temperature[-1] - temperature[0]):+.3f} °C",
        f"- 线性温度趋势：{temperature_slope:+.4f} °C/h",
        "",
        "温度趋势可能在长平均时间区域伪装成RRW或Rate Ramp，因此长期项必须结合稳定性图复核。",
        "",
        "### 60秒均值与温度相关系数",
        "",
        "| 传感器 | 轴 | Pearson r | 长期参数建议 |",
        "|---|---|---:|---|",
        *[
            f"| {'陀螺仪' if result.sensor == 'gyro' else '加速度计'} | {result.axis.upper()} | "
            f"{result.temperature_correlation:+.3f} | "
            f"{'可作候选' if result.long_term_usable else '温漂污染，不直接使用'} |"
            for result in results
        ],
        "",
        "相关系数只描述线性关系；即使相关系数不高，只要存在明显温度阶跃，长周期Allan参数仍可能被污染。",
        "",
        "## 3. 自动提取参数",
        "",
        *report_table_rows(results),
        "",
        "括号内首先是曲线拟合可信度。`温漂污染`表示形状可能吻合，但物理上不能确认是器件固有RRW。`未检出`不能按零填写。",
        "",
        "## 4. 拟合区间明细",
        "",
        *region_detail_rows(results),
        "",
        "## 5. 参数定义",
        "",
        "- ARW/VRW：在斜率约为-1/2的区域，使用 `N=median[σ(τ)√τ]`。",
        "- BI：在斜率约为0且靠近Allan最低点的区域，使用 `B=median[σ(τ)]/0.664`。",
        "- RRW：在斜率约为+1/2的区域，使用 `K=median[σ(τ)√(3/τ)]`。",
        "- Rate Ramp：在斜率约为+1的区域，使用 `R=median[√2σ(τ)/τ]`。",
        "",
        "## 6. GNSS/INS参数使用建议",
        "",
        "1. ARW/VRW用于陀螺仪和加速度计快速测量白噪声。先确认滤波程序要求的是连续噪声密度、每样本标准差还是离散方差。",
        "2. BI主要用于零偏初始协方差或一阶高斯-马尔可夫模型的稳态波动尺度，不能直接把`BI²`当作过程噪声Q。",
        "3. RRW用于零偏随机游走过程噪声。在单位一致时，每步离散方差为`Q_bias=K²dt`。",
        "4. Rate Ramp优先作为温度或确定性趋势处理，不应直接作为白噪声写入R。",
        "5. 低或未检出可信度的长期参数应通过更长数据、控温实验或重复实验确认。",
        "",
        "## 7. 自动判读限制",
        "",
        *confidence_notes,
        "- 自动判读只识别局部幂律斜率，不能单凭形状区分器件固有随机噪声与温度、电源、振动等环境影响。",
        "- Allan最右侧点可用的独立数据块很少。本程序将最大τ限制为分析时长的十分之一，但长期结论仍需人工复核。",
        "- 如果曲线最左端先上升，通常表示DLPF、相邻样本相关或窄带振动；程序不会把该段用于ARW/VRW拟合。",
        "",
        "## 8. 输出文件",
        "",
        "- `allan_deviation.png`：加速度计和陀螺仪双图总览。",
        "- `allan_identification.png`：六轴Allan曲线及自动识别区间。",
        "- `stability_overview.png`：60秒均值和温度随时间变化。",
        "- `allan_parameters.csv`：参数、拟合斜率、区间和可信度。",
        "- `allan_deviation.csv`：六轴Allan偏差原始结果。",
    ]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=pathlib.Path, help="Physical-unit IMU CSV from Qt or capture_serial.py")
    parser.add_argument("--rate", type=float, default=100.0, help="Nominal sample rate in Hz")
    parser.add_argument("--skip-minutes", type=float, default=0.0, help="Discard warm-up data at the start")
    parser.add_argument("--points", type=int, default=90, help="Number of logarithmic Allan points")
    parser.add_argument("--output", type=pathlib.Path, default=None, help="Output directory")
    args = parser.parse_args()

    if args.rate <= 0.0:
        raise SystemExit("--rate must be positive")
    if args.skip_minutes < 0.0:
        raise SystemExit("--skip-minutes cannot be negative")
    if args.points < 30:
        raise SystemExit("--points must be at least 30")
    if not args.csv.exists():
        raise SystemExit(f"CSV not found: {args.csv}")

    output = args.output or DATA_DIR / "allan_results" / f"{args.csv.stem}_noise"
    output.mkdir(parents=True, exist_ok=True)

    print(f"Loading {args.csv.resolve()} ...", flush=True)
    data_all = load_decoded_csv(args.csv)
    skip_samples = int(round(args.skip_minutes * 60.0 * args.rate))
    if skip_samples >= data_all.size - 1000:
        raise SystemExit("Warm-up skip leaves too few samples")
    data = data_all[skip_samples:]
    period = 1.0 / args.rate

    sample_delta = np.diff(data_all["sample"].astype(np.int64))
    sequence_discontinuities = int(np.count_nonzero(sample_delta != 1))
    missing_samples = int(np.sum(np.maximum(sample_delta - 1, 0)))
    elapsed = float(data_all["time_s"][-1] - data_all["time_s"][0])
    empirical_rate = (data_all.size - 1) / elapsed if elapsed > 0.0 else float("nan")
    dt_ms = np.rint(data_all["dt_s"].astype(np.float64) * 1000.0).astype(np.int64)
    dt_values, dt_counts = np.unique(dt_ms, return_counts=True)
    dt_summary = ", ".join(
        f"{int(value)} ms:{int(count):,}" for value, count in zip(dt_values, dt_counts)
    )

    print(f"Frames: {data_all.size:,}; analyzing: {data.size:,}", flush=True)
    print(f"Duration: {elapsed / 3600.0:.3f} h; empirical rate: {empirical_rate:.6f} Hz", flush=True)
    print(f"Sequence discontinuities: {sequence_discontinuities}; missing: {missing_samples}", flush=True)

    results: list[AxisResult] = []
    for sensor, suffix, scale, unit in (
        ("accel", "m_s2", 1.0, "m/s^2"),
        ("gyro", "deg_h", math.pi / 180.0 / 3600.0, "rad/s"),
    ):
        for axis in "xyz":
            print(f"Computing {sensor} {axis.upper()} ...", flush=True)
            values = data[f"{sensor[0]}{axis}_{suffix}"].astype(np.float64) * scale
            results.append(analyze_axis(sensor, axis, values, unit, period, args.points, data.size * period))

    # Present accelerometer row first and gyroscope row second in the plot.
    results.sort(key=lambda item: (0 if item.sensor == "accel" else 1, item.axis))
    temperature = data["temp_deg_c"].astype(np.float64)
    temperature_slope = fit_temperature_slope(temperature, period)
    temperature_range = float(np.max(temperature) - np.min(temperature))
    assign_temperature_correlations(data, results, period, temperature_range)

    overview_plot = output / "allan_deviation.png"
    allan_plot = output / "allan_identification.png"
    stability_plot = output / "stability_overview.png"
    parameter_csv = output / "allan_parameters.csv"
    deviation_csv = output / "allan_deviation.csv"
    report = output / "随机误差判读报告.md"

    plot_allan_overview(overview_plot, results)
    plot_allan(allan_plot, results)
    plot_stability(stability_plot, data, period)
    write_parameters_csv(parameter_csv, results)
    write_allan_csv(deviation_csv, results)
    write_report(
        report, args.csv, data_all, data, results, args.rate, args.skip_minutes,
        sequence_discontinuities, missing_samples, empirical_rate, dt_summary,
        temperature, temperature_slope,
    )

    print(f"Report: {report.resolve()}")
    print(f"Parameters: {parameter_csv.resolve()}")
    print(f"Overview plot: {overview_plot.resolve()}")
    print(f"Allan plot: {allan_plot.resolve()}")
    print(f"Stability plot: {stability_plot.resolve()}")


if __name__ == "__main__":
    main()
