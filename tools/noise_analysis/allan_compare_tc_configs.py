#!/usr/bin/env python3
r"""Compare Allan deviation before/after MPU6050 temperature compensation.

Stages are configurable: ``raw`` (original), ``tc`` (temperature compensated),
and ``calib`` (TC plus matching 24-position calibration).

Recommended command for a PPT-ready Raw/TC comparison::

python -X utf8 tools/noise_analysis/allan_compare_tc_configs.py `
  data/decoded/20260926005735/imu.csv `
  --stages raw,tc `
  --tc-csv data/decoded/20260926005735/imu_tempcomp_multiorder.csv `
  --plot-layout sensor-pair `
  --plot-size 1800x780 `
  --output data/decoded/20260926005735/allan_raw_tc


  python tools/noise_analysis/allan_compare_tc_configs.py data/decoded/<session>/imu.csv \
      --stages raw,tc \
      --tc-csv data/decoded/<session>/imu_tempcomp_multiorder.csv \
      --plot-layout sensor-pair --plot-size 1800x780 --dpi 300

With exactly raw and tc selected, one figure contains two panels (accelerometer
and gyroscope); each panel contains Raw/TC curves for X/Y/Z. ``--plot-size``
controls the PNG dimensions exactly. For a 600x260 display area in PowerPoint,
1800x780 at 300 dpi provides a sharp 3x-resolution source image.
"""

from __future__ import annotations

import argparse
import math
import pathlib
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import scipy.io as sio

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import allan_noise_identification as ani  # noqa: E402

G0 = 9.80665
DEG_H_TO_RAD_S = math.pi / 180.0 / 3600.0
AXES = ["ax", "ay", "az", "gx", "gy", "gz"]
VALUE_COLUMNS = ["ax_m_s2", "ay_m_s2", "az_m_s2", "gx_deg_h", "gy_deg_h", "gz_deg_h"]
STAGE_ALIASES = {
    "raw": "raw",
    "tc": "raw+TC",
    "raw+tc": "raw+TC",
    "calib": "raw+TC+calib",
    "raw+tc+calib": "raw+TC+calib",
}
STAGE_LABELS = {"raw": "Raw", "raw+TC": "TC", "raw+TC+calib": "TC+Calib"}
TAU_TARGETS = [30.0, 60.0, 180.0, 300.0, 600.0, 1000.0, 3600.0]
BLOCK_SEC = 600.0

plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "Noto Sans CJK SC", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False


def parse_stages(text: str) -> list[str]:
    stages: list[str] = []
    for token in (part.strip().lower() for part in text.split(",")):
        if token not in STAGE_ALIASES:
            raise argparse.ArgumentTypeError(f"unknown stage {token!r}; use raw, tc, calib")
        stage = STAGE_ALIASES[token]
        if stage not in stages:
            stages.append(stage)
    if not stages:
        raise argparse.ArgumentTypeError("at least one stage is required")
    return stages


def parse_plot_size(text: str) -> tuple[int, int]:
    try:
        width_text, height_text = text.lower().replace(",", "x").split("x", 1)
        width, height = int(width_text), int(height_text)
    except (TypeError, ValueError) as exc:
        raise argparse.ArgumentTypeError("--plot-size must look like 600x260") from exc
    if width < 320 or height < 180:
        raise argparse.ArgumentTypeError("--plot-size is too small (minimum 320x180)")
    return width, height


def adev_at_tau(values: np.ndarray, period: float, taus) -> dict[float, float]:
    """Overlapping Allan deviation evaluated exactly at requested taus."""
    v = np.asarray(values, dtype=np.float64)
    v = v - v.mean()
    n = v.size
    integral = np.empty(n + 1, dtype=np.float64)
    integral[0] = 0.0
    np.cumsum(v, out=integral[1:])
    integral *= period
    out: dict[float, float] = {}
    for tau in taus:
        m = max(1, int(round(tau / period)))
        if 2 * m >= n:
            out[tau] = float("nan")
            continue
        second = integral[2 * m:] - 2.0 * integral[m:-m] + integral[:-2 * m]
        variance = float(np.dot(second, second)) / (second.size * 2.0 * (m * period) ** 2)
        out[tau] = math.sqrt(max(variance, 0.0))
    return out


def block_mean_std(values: np.ndarray, period: float, block_sec: float = BLOCK_SEC) -> float:
    m = max(1, int(round(block_sec / period)))
    blocks = values.size // m
    if blocks < 2:
        return float("nan")
    means = values[: blocks * m].reshape(blocks, m).mean(axis=1)
    return float(np.std(means - np.median(means), ddof=1))


def load_coeffs(path: pathlib.Path) -> tuple[np.ndarray, np.ndarray, float, dict]:
    """Return coefficient matrix [c5..c0], orders, Tref, and MAT metadata."""
    frame = pd.read_csv(path)
    required = {"axis", "order", "c0", "c1", "c2", "c3", "c4", "c5"}
    missing = required.difference(frame.columns)
    if missing:
        raise SystemExit(f"--coeff: missing columns {sorted(missing)}")
    if len(frame) != 6:
        raise SystemExit("--coeff: exactly six axis rows are required")
    coefficient = frame[["c5", "c4", "c3", "c2", "c1", "c0"]].to_numpy(dtype=np.float64)
    orders = frame["order"].to_numpy(dtype=int)
    if not np.isfinite(coefficient).all() or np.any((orders < 0) | (orders > 5)):
        raise SystemExit("--coeff: invalid polynomial coefficients or orders")
    mat_path = path.with_suffix(".mat")
    if not mat_path.is_file():
        raise SystemExit(f"--coeff: companion MAT metadata is required (missing {mat_path})")
    metadata = sio.loadmat(mat_path, simplify_cells=True)
    tref = float(metadata["Tref"])
    if not np.allclose(np.asarray(metadata["coef"], dtype=float), coefficient,
                       rtol=1e-13, atol=1e-12):
        raise SystemExit("--coeff: CSV and companion MAT coefficient matrices differ")
    return coefficient, orders, tref, metadata


def calibration_field(calibration: np.ndarray, name: str) -> np.ndarray:
    if calibration.dtype.names is None or name not in calibration.dtype.names:
        raise SystemExit(f"--calib: result.{name} is required")
    return np.asarray(calibration[name]).squeeze()


def read_imu_csv(path: pathlib.Path, need_temperature: bool, label: str):
    columns = ["sample", "time_s", *VALUE_COLUMNS]
    if need_temperature:
        columns.append("temp_deg_c")
    try:
        frame = pd.read_csv(path, usecols=columns, dtype=np.float64)
    except (ValueError, FileNotFoundError) as exc:
        raise SystemExit(f"{label}: cannot read required IMU columns from {path}: {exc}") from exc
    if len(frame) < 1000:
        raise SystemExit(f"{label}: at least 1000 samples are required")
    if not np.isfinite(frame.to_numpy()).all():
        raise SystemExit(f"{label}: input contains NaN/Inf")
    samples = frame["sample"].to_numpy(dtype=np.int64, copy=True)
    times = frame["time_s"].to_numpy(dtype=np.float64, copy=True)
    values = frame[VALUE_COLUMNS].to_numpy(dtype=np.float64, copy=True)
    temperature = (frame["temp_deg_c"].to_numpy(dtype=np.float64, copy=True)
                   if need_temperature else None)
    return samples, times, values, temperature


def validate_sequence(samples: np.ndarray, times: np.ndarray, label: str) -> float:
    time_steps = np.diff(times)
    if np.any(time_steps <= 0) or np.any(np.diff(samples) != 1):
        raise SystemExit(f"{label}: time/sample sequence is not strictly contiguous")
    median_period = float(np.median(time_steps))
    if np.any(time_steps > 1.5 * median_period):
        raise SystemExit(f"{label}: input contains a time gap; split the recording first")
    return median_period


def save_sensor_pair_figure(curves, stages, output, pixel_size, dpi) -> None:
    """Save one PPT-friendly figure containing accelerometer and gyro panels."""
    width, height = pixel_size
    fig, plot_axes = plt.subplots(1, 2, figsize=(width / dpi, height / dpi), dpi=dpi)
    axis_colors = {"x": "#0072B2", "y": "#D55E00", "z": "#009E73"}
    stage_styles = {
        "raw": ("--", 1.05, 0.82),
        "raw+TC": ("-", 1.50, 1.0),
        "raw+TC+calib": (":", 1.35, 0.95),
    }
    panel_specs = [
        (plot_axes[0], AXES[:3], 1000.0 / G0, "ACCE Allan dev. (mg)"),
        (plot_axes[1], AXES[3:], 180.0 / math.pi * 3600.0,
         "GYRO Allan dev. (deg/h)"),
    ]
    for plot_axis, sensor_axes, conversion, ylabel in panel_specs:
        for stage in stages:
            linestyle, linewidth, alpha = stage_styles[stage]
            for axis in sensor_axes:
                tau, adev = curves[(stage, axis)]
                axis_letter = axis[-1]
                line, = plot_axis.loglog(
                    tau, adev * conversion,
                    color=axis_colors[axis_letter], linestyle=linestyle,
                    linewidth=linewidth, alpha=alpha,
                    label=f"{STAGE_LABELS[stage]} {axis_letter.upper()}",
                )
        plot_axis.set_xlabel(r"$\tau$ (s)", fontsize=8.0, labelpad=1)
        plot_axis.set_ylabel(ylabel, fontsize=8.0, labelpad=1)
        plot_axis.tick_params(axis="both", which="major", labelsize=7.0, length=2.5, pad=1.5)
        plot_axis.tick_params(axis="both", which="minor", length=1.5)
        plot_axis.grid(True, which="major", color="#b8b8b8", linewidth=0.45, alpha=0.65)
        plot_axis.grid(True, which="minor", color="#dddddd", linewidth=0.30, alpha=0.45)
        handles, labels = plot_axis.get_legend_handles_labels()
        # Matplotlib fills a multi-column legend column-first. Interleave the
        # stage handles by axis so the rendered rows become Raw X/Y/Z, then
        # TC X/Y/Z (and likewise for any additional selected stage).
        stage_count = len(stages)
        legend_order = [stage_index * 3 + axis_index
                        for axis_index in range(3)
                        for stage_index in range(stage_count)]
        plot_axis.legend(
            [handles[index] for index in legend_order],
            [labels[index] for index in legend_order],
            loc="upper center", ncol=3, frameon=True, framealpha=0.82,
            facecolor="white", edgecolor="#cccccc", fontsize=6.3,
            handlelength=1.8, columnspacing=0.75, handletextpad=0.35,
            borderpad=0.30, labelspacing=0.25,
        )

    fig.subplots_adjust(left=0.090, right=0.990, bottom=0.185, top=0.975, wspace=0.31)
    fig.savefig(output, dpi=dpi, facecolor="white")
    plt.close(fig)


def save_per_axis_figure(curves, stages, output, pixel_size, dpi) -> None:
    width, height = pixel_size
    colors = {"raw": "#8c8c8c", "raw+TC": "#1f77b4", "raw+TC+calib": "#d62728"}
    fig, axes = plt.subplots(2, 3, figsize=(width / dpi, height / dpi), dpi=dpi)
    for index, axis in enumerate(AXES):
        plot_axis = axes.flat[index]
        conversion = 180.0 / math.pi * 3600.0 if index >= 3 else 1000.0 / G0
        for stage in stages:
            tau, adev = curves[(stage, axis)]
            plot_axis.loglog(tau, adev * conversion, color=colors[stage], lw=1.1,
                             label=STAGE_LABELS[stage])
        plot_axis.set_title(axis.upper(), fontsize=7)
        plot_axis.set_xlabel(r"$\tau$ (s)", fontsize=6)
        plot_axis.set_ylabel("deg/h" if index >= 3 else "mg", fontsize=6)
        plot_axis.tick_params(labelsize=5.5)
        plot_axis.grid(True, which="both", alpha=0.30)
        if index == 0:
            plot_axis.legend(fontsize=5.5)
    fig.tight_layout(pad=0.5)
    fig.savefig(output, dpi=dpi, facecolor="white")
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("csv", type=pathlib.Path, help="raw physical-unit IMU CSV")
    parser.add_argument("--stages", type=parse_stages, default=parse_stages("raw,tc,calib"),
                        help="comma-separated stages: raw,tc,calib")
    parser.add_argument("--tc-csv", type=pathlib.Path,
                        help="already temperature-compensated IMU CSV; skips TC recalculation")
    parser.add_argument("--coeff", type=pathlib.Path,
                        help="temp_coeffs_raw.csv; required when TC must be calculated")
    parser.add_argument("--calib", type=pathlib.Path,
                        help="matching 24-position calibration MAT; required for calib stage")
    parser.add_argument("--enable", default="0,0,1,1,1,0",
                        help="per-axis TC enable ax,ay,az,gx,gy,gz")
    parser.add_argument("--rate", type=float, default=None,
                        help="override sample rate; default derives it from time_s")
    parser.add_argument("--points", type=int, default=90,
                        help="number of logarithmic Allan tau points")
    parser.add_argument("--tref", type=float, default=None, help="override coefficient Tref")
    parser.add_argument("--plot-layout", choices=("auto", "sensor-pair", "per-axis"), default="auto",
                        help="auto uses sensor-pair for raw+tc, otherwise per-axis")
    parser.add_argument("--plot-size", type=parse_plot_size, default=parse_plot_size("1800x780"),
                        help="source PNG pixels; use 1800x780 for a 600x260 PPT display area")
    parser.add_argument("--dpi", type=int, default=300,
                        help="rendering DPI; output pixels remain controlled by --plot-size")
    parser.add_argument("--output", type=pathlib.Path, default=None)
    args = parser.parse_args()

    if args.points < 20:
        raise SystemExit("--points must be at least 20")
    if args.dpi < 50:
        raise SystemExit("--dpi must be at least 50")
    try:
        enable_bits = np.array([int(value) for value in args.enable.split(",")], dtype=int)
    except ValueError as exc:
        raise SystemExit("--enable needs 6 comma-separated 0/1 values") from exc
    if enable_bits.size != 6 or not np.isin(enable_bits, [0, 1]).all():
        raise SystemExit("--enable needs 6 comma-separated bits")
    enable = enable_bits.astype(bool)

    selected_stages: list[str] = args.stages
    needs_tc = "raw+TC" in selected_stages or "raw+TC+calib" in selected_stages
    needs_calib = "raw+TC+calib" in selected_stages
    if needs_tc and args.tc_csv is None and args.coeff is None:
        raise SystemExit("TC stage needs --tc-csv or --coeff")
    if needs_calib and args.calib is None:
        raise SystemExit("calib stage needs --calib")
    if needs_calib and args.coeff is None:
        raise SystemExit("calib stage needs --coeff to verify the matching TC model")

    calculate_tc = needs_tc and args.tc_csv is None
    print(f"Loading raw data: {args.csv}", flush=True)
    samples, times, raw, temperature = read_imu_csv(args.csv, calculate_tc, "raw CSV")
    median_period = validate_sequence(samples, times, "raw CSV")
    sample_count = len(times)
    period = 1.0 / args.rate if args.rate else float((times[-1] - times[0]) / (sample_count - 1))
    duration_h = (sample_count - 1) * period / 3600.0
    print(f"samples: {sample_count:,}; duration: {duration_h:.2f} h; rate: {1.0 / period:.4f} Hz",
          flush=True)

    stages: dict[str, np.ndarray] = {"raw": raw}
    coefficient = orders = coefficient_metadata = None
    tref = tcpoly = None
    if args.coeff is not None:
        coefficient, orders, tref, coefficient_metadata = load_coeffs(args.coeff)
        if args.tref is not None:
            tref = args.tref
        tcpoly = np.hstack([coefficient[:, 0:5], np.zeros((6, 1))])
        active_from_model = orders > 0
        if not np.array_equal(enable, active_from_model):
            raise SystemExit(
                f"--enable {enable_bits.tolist()} differs from coefficient orders>0 "
                f"{active_from_model.astype(int).tolist()}")

    if needs_tc:
        if args.tc_csv is not None:
            print(f"Loading temperature-compensated data: {args.tc_csv}", flush=True)
            tc_samples, tc_times, stage2, _ = read_imu_csv(args.tc_csv, False, "TC CSV")
            validate_sequence(tc_samples, tc_times, "TC CSV")
            tolerance = max(1e-9, median_period * 1e-5)
            if not np.array_equal(tc_samples, samples):
                raise SystemExit("TC CSV sample numbers do not align with raw CSV")
            if not np.allclose(tc_times, times, rtol=0.0, atol=tolerance):
                raise SystemExit("TC CSV timestamps do not align with raw CSV")
        else:
            assert tcpoly is not None and tref is not None and coefficient_metadata is not None
            assert temperature is not None
            tmin = float(coefficient_metadata["Tmin"])
            tmax = float(coefficient_metadata["Tmax"])
            if temperature.min() < tmin - 1e-9 or temperature.max() > tmax + 1e-9:
                raise SystemExit(
                    f"input temperature [{temperature.min():.4f}, {temperature.max():.4f}] C exceeds "
                    f"fitted range [{tmin:.4f}, {tmax:.4f}] C")
            delta_temperature = temperature - tref
            correction = np.array([
                np.polyval(tcpoly[index], delta_temperature) if enable[index]
                else np.zeros(sample_count)
                for index in range(6)
            ]).T
            stage2 = raw - correction
        stages["raw+TC"] = stage2

    if needs_calib:
        assert args.calib is not None and tcpoly is not None and tref is not None
        calibration = sio.loadmat(args.calib)["result"][0, 0]
        ba = calibration_field(calibration, "ba").astype(np.float64).reshape(3)
        ca = calibration_field(calibration, "Ca").astype(np.float64).reshape(3, 3)
        gb = calibration_field(calibration, "gyroBiasMean_deg_h").astype(np.float64).reshape(3)
        cal_coeff = calibration_field(calibration, "tempCoeff").astype(np.float64).reshape(6, 6)
        cal_tref = float(calibration_field(calibration, "Tref"))
        cal_active = calibration_field(calibration, "tcActive").astype(bool).reshape(6)
        if not np.allclose(cal_coeff, tcpoly, rtol=1e-13, atol=1e-12):
            raise SystemExit("--calib and --coeff contain different temperature models")
        if abs(cal_tref - tref) > 1e-10:
            raise SystemExit("--calib and --coeff use different reference temperatures")
        if not np.array_equal(cal_active, enable):
            raise SystemExit("--calib active axes differ from --enable")
        stage3 = stages["raw+TC"].copy()
        stage3[:, 0:3] = (stage3[:, 0:3] - ba) @ ca.T
        stage3[:, 3:6] -= gb
        stages["raw+TC+calib"] = stage3

    print("Selected stages: " + ", ".join(STAGE_LABELS[stage] for stage in selected_stages), flush=True)
    results: dict[tuple[str, str], ani.AxisResult] = {}
    curves: dict[tuple[str, str], tuple[np.ndarray, np.ndarray]] = {}
    tau_table: list[dict] = []
    for axis_index, axis in enumerate(AXES):
        is_gyro = axis_index >= 3
        scale = DEG_H_TO_RAD_S if is_gyro else 1.0
        conversion = 180.0 / math.pi * 3600.0 if is_gyro else 1000.0 / G0
        unit = "deg/h" if is_gyro else "mg"
        row = {"axis": axis, "unit": unit}
        for stage in selected_stages:
            values = stages[stage][:, axis_index] * scale
            result = ani.analyze_axis(
                "gyro" if is_gyro else "accel", axis[-1], values,
                "rad/s" if is_gyro else "m/s^2", period, args.points,
                sample_count * period,
            )
            results[(stage, axis)] = result
            curves[(stage, axis)] = (result.tau, result.adev)
            fixed_tau = adev_at_tau(values, period, TAU_TARGETS)
            row[f"{stage}_block600_std"] = block_mean_std(values, period) * conversion
            for tau in TAU_TARGETS:
                row[f"{stage}_adev_{int(tau)}s"] = fixed_tau[tau] * conversion
        tau_table.append(row)
        summary = " / ".join(
            f"{STAGE_LABELS[stage]} {row[f'{stage}_adev_600s']:.4g}"
            for stage in selected_stages
        )
        print(f"  {axis}: adev@600s {summary} {unit}", flush=True)

    output_dir = args.output or (args.csv.resolve().parent / "allan_compare_tc_configs")
    output_dir.mkdir(parents=True, exist_ok=True)
    layout = args.plot_layout
    if layout == "auto":
        layout = "sensor-pair" if selected_stages == ["raw", "raw+TC"] else "per-axis"
    width, height = args.plot_size
    if layout == "sensor-pair":
        figure_path = output_dir / f"allan_raw_tc_{width}x{height}.png"
        save_sensor_pair_figure(curves, selected_stages, figure_path, args.plot_size, args.dpi)
    else:
        figure_path = output_dir / f"allan_per_axis_{width}x{height}.png"
        save_per_axis_figure(curves, selected_stages, figure_path, args.plot_size, args.dpi)

    tau_columns = ["axis", "unit"]
    tau_columns += [f"{stage}_block600_std" for stage in selected_stages]
    tau_columns += [f"{stage}_adev_{int(tau)}s"
                    for stage in selected_stages for tau in TAU_TARGETS]
    pd.DataFrame(tau_table)[tau_columns].to_csv(
        output_dir / "allan_adev_at_tau.csv", index=False, encoding="utf-8-sig")

    parameter_rows = []
    for (stage, axis), result in results.items():
        display = ani.parameter_display(result)
        parameter_rows.append({
            "stage": stage, "axis": axis,
            "arw_vrw": display["white_primary"][0],
            "arw_vrw_unit": display["white_primary"][1],
            "bias_instability": display["bias_primary"][0],
            "bias_instability_unit": display["bias_primary"][1],
            "rrw": display["rrw_primary"][0],
            "rrw_unit": display["rrw_primary"][1],
            "rate_ramp": display["ramp_primary"][0],
            "rate_ramp_unit": display["ramp_primary"][1],
        })
    pd.DataFrame(parameter_rows).to_csv(
        output_dir / "allan_selected_stages_params.csv", index=False, encoding="utf-8-sig")

    reference_tau = curves[(selected_stages[0], "ax")][0]
    curve_columns = [reference_tau] + [curves[(stage, axis)][1]
                                       for stage in selected_stages for axis in AXES]
    header = "tau_s," + ",".join(
        f"{stage}_{axis}_adev_rad_s_or_m_s2"
        for stage in selected_stages for axis in AXES)
    np.savetxt(output_dir / "allan_deviation.csv", np.column_stack(curve_columns),
               delimiter=",", header=header, comments="")

    report_lines = [
        "# Allan 对比摘要", "",
        f"- 原始数据：`{args.csv.name}`",
        f"- 温补数据：`{args.tc_csv.name}`" if args.tc_csv else "- 温补数据：由温补系数计算",
        f"- 样本数：{sample_count:,}；时长：{duration_h:.2f} h；采样率：{1.0 / period:.4f} Hz",
        f"- 计算阶段：{', '.join(STAGE_LABELS[stage] for stage in selected_stages)}",
        f"- 图片：`{figure_path.name}`（{width}×{height} px）", "",
        "## 600 s Allan 偏差", "",
        "| 轴 | 单位 | " + " | ".join(STAGE_LABELS[stage] for stage in selected_stages) + " |",
        "|---|---|" + "---:|" * len(selected_stages),
    ]
    for row in tau_table:
        report_lines.append(
            f"| {row['axis'].upper()} | {row['unit']} | "
            + " | ".join(f"{row[f'{stage}_adev_600s']:.4g}" for stage in selected_stages)
            + " |")
    (output_dir / "Allan对比摘要.md").write_text(
        "\n".join(report_lines) + "\n", encoding="utf-8")

    print(f"figure: {figure_path}")
    print(f"outputs: {output_dir}")
    print("DONE")


if __name__ == "__main__":
    main()
