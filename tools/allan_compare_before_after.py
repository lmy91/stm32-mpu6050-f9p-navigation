"""Compare Allan deviation & noise parameters before/after temperature-drift
compensation.

"Before" = 24-position-calibration-compensated data (temperature drift NOT
removed).  "After" = same data with the 3rd-order temperature model terms
(c1*dT + c2*dT^2 + c3*dT^3) removed.

The "after" dataset is the full-resolution imu_compensated.csv written by
tools/fit_temp_bias_poly3.m.  The "before" dataset is reconstructed from it by
adding the fitted temperature terms back (cal = tempcomp + temp_terms).

Outputs (in --output dir):
  allan_compare.png          2x3 per-axis Allan curves, before vs after
  allan_compare_overview.png accel/gyro overview, before (dashed) vs after
  allan_parameters_compare.csv  per-axis noise parameters before/after
  allan_parameters_compare.md   Markdown comparison table

Usage:
  python allan_compare_before_after.py <imu_compensated.csv> <coeffs.csv> \
      [--rate 100] [--output DIR]
"""

from __future__ import annotations

import argparse
import math
import pathlib
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# 中文字体（Windows 优先微软雅黑，回退 SimHei / Noto Sans CJK）
for font in ("Microsoft YaHei", "SimHei", "Noto Sans CJK SC"):
    try:
        from matplotlib import font_manager
        if any(font.lower() in f.name.lower() for f in font_manager.fontManager.ttflist):
            matplotlib.rcParams["font.sans-serif"] = [font, "DejaVu Sans"]
            break
    except Exception:
        pass
matplotlib.rcParams["axes.unicode_minus"] = False
import numpy as np
import numpy as np
import pandas as pd

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from allan_noise_identification import (  # noqa: E402
    DEG_PER_RAD,
    G0,
    analyze_axis,
    parameter_display,
)

COLOR_BEFORE = "#7f7f7f"
COLOR_AFTER = "#d62728"


def load_columns(path: pathlib.Path) -> dict[str, np.ndarray]:
    usecols = [
        "sample", "time_s", "dt_s",
        "ax_m_s2", "ay_m_s2", "az_m_s2", "temp_deg_c",
        "gx_deg_h", "gy_deg_h", "gz_deg_h",
    ]
    print(f"Loading {path} ...", flush=True)
    frame = pd.read_csv(path, usecols=usecols)
    frame = frame.dropna()
    data = {name: frame[name].to_numpy(dtype=np.float64) for name in usecols}
    print(f"Loaded {len(frame):,} samples "
          f"({(data['time_s'][-1] - data['time_s'][0]) / 3600.0:.2f} h)", flush=True)
    return data


def load_coeffs(path: pathlib.Path) -> np.ndarray:
    """Return 6x4 [c0 c1 c2 c3] in file row order ax ay az gx gy gz."""
    frame = pd.read_csv(path)
    return frame[["c0", "c1", "c2", "c3"]].to_numpy(dtype=np.float64)


def temp_terms(temp: np.ndarray, t0: float, coeffs: np.ndarray, k: int) -> np.ndarray:
    dT = temp - t0
    return coeffs[k, 1] * dT + coeffs[k, 2] * dT ** 2 + coeffs[k, 3] * dT ** 3


def fmt(value: float, digits: int = 4) -> str:
    return f"{value:.{digits}g}" if math.isfinite(value) else "未检出"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("compensated_csv", type=pathlib.Path)
    parser.add_argument("coeffs_csv", type=pathlib.Path)
    parser.add_argument("--rate", type=float, default=100.0)
    parser.add_argument("--points", type=int, default=90)
    parser.add_argument("--output", type=pathlib.Path, default=None)
    args = parser.parse_args()

    out_dir = args.output or args.compensated_csv.parent / "allan_compare_before_after"
    out_dir.mkdir(parents=True, exist_ok=True)
    period = 1.0 / args.rate

    data = load_columns(args.compensated_csv)
    coeffs = load_coeffs(args.coeffs_csv)
    t0 = float(data["temp_deg_c"][0])  # same reference as the MATLAB fit
    print(f"T0 (reference temperature) = {t0:.4f} C", flush=True)

    axes_cfg = [
        ("accel", "x", "ax_m_s2", 1.0, "m/s^2"),
        ("accel", "y", "ay_m_s2", 1.0, "m/s^2"),
        ("accel", "z", "az_m_s2", 1.0, "m/s^2"),
        ("gyro", "x", "gx_deg_h", math.pi / 180.0 / 3600.0, "rad/s"),
        ("gyro", "y", "gy_deg_h", math.pi / 180.0 / 3600.0, "rad/s"),
        ("gyro", "z", "gz_deg_h", math.pi / 180.0 / 3600.0, "rad/s"),
    ]

    duration = (data["time_s"][-1] - data["time_s"][0])
    results = []
    for idx, (sensor, axis, col, scale, unit) in enumerate(axes_cfg):
        print(f"Allan: {sensor} {axis.upper()} ...", flush=True)
        after = data[col] * scale
        before = after + temp_terms(data["temp_deg_c"], t0, coeffs, idx) * scale
        res_before = analyze_axis(sensor, axis, before, unit, period, args.points, duration)
        res_after = analyze_axis(sensor, axis, after, unit, period, args.points, duration)
        results.append((sensor, axis, unit, res_before, res_after))
        del before, after

    # ---- per-axis comparison plot ----
    fig, axes = plt.subplots(2, 3, figsize=(16, 9), constrained_layout=True)
    for ax, (sensor, axis, unit, rb, ra) in zip(axes.flat, results):
        factor = DEG_PER_RAD if sensor == "gyro" else 1.0
        ylabel = "Allan deviation (deg/s)" if sensor == "gyro" else "Allan deviation (m/s^2)"
        ax.loglog(rb.tau, rb.adev * factor, color=COLOR_BEFORE, linewidth=1.6,
                  linestyle="--", label="温漂补偿前 (24位标定)")
        ax.loglog(ra.tau, ra.adev * factor, color=COLOR_AFTER, linewidth=1.6,
                  label="温漂补偿后 (标定+3阶温漂)")
        ax.set(title=f"{sensor.capitalize()} {axis.upper()}",
               xlabel="Cluster time tau (s)", ylabel=ylabel)
        ax.grid(True, which="both", alpha=0.3)
        ax.legend(fontsize=8)
    fig.suptitle("温漂补偿前后 Allan 偏差对比 (21h 静态, 24位标定补偿基础上)", fontsize=14)
    fig.savefig(out_dir / "allan_compare.png", dpi=180)
    plt.close(fig)

    # ---- overview plot ----
    fig, axes = plt.subplots(1, 2, figsize=(12, 5), constrained_layout=True)
    for sensor, panel in (("accel", axes[0]), ("gyro", axes[1])):
        factor = DEG_PER_RAD if sensor == "gyro" else 1.0
        for sensor_r, axis, _, rb, ra in results:
            if sensor_r != sensor:
                continue
            panel.loglog(rb.tau, rb.adev * factor, linestyle="--", linewidth=1.3,
                         color=COLOR_BEFORE, alpha=0.85)
            panel.loglog(ra.tau, ra.adev * factor, linewidth=1.3, label=f"{axis.upper()} (after)")
    axes[0].set(title="Accelerometer", xlabel="tau (s)", ylabel="Allan deviation (m/s^2)")
    axes[1].set(title="Gyroscope", xlabel="tau (s)", ylabel="Allan deviation (deg/s)")
    h_before = plt.Line2D([], [], color=COLOR_BEFORE, linestyle="--", label="温漂补偿前")
    h_after = plt.Line2D([], [], color="#1f77b4", label="温漂补偿后")
    for panel in axes:
        panel.grid(True, which="both", alpha=0.35)
        handles, labels = panel.get_legend_handles_labels()
        panel.legend(handles=[h_before, h_after] + handles, fontsize=8)
    fig.suptitle("温漂补偿前后 Allan 偏差总览", fontsize=13)
    fig.savefig(out_dir / "allan_compare_overview.png", dpi=180)
    plt.close(fig)

    # ---- parameter comparison table ----
    kinds = ["white", "bias", "rrw", "ramp"]
    kind_names = {"white": "ARW/VRW", "bias": "BI", "rrw": "RRW", "ramp": "Rate Ramp"}
    rows_csv = []
    rows_md = [
        "| 传感器 | 轴 | 参数 | 补偿前 | 补偿后 | 变化 |",
        "|---|---|---|---:|---:|---:|",
    ]
    for sensor, axis, unit, rb, ra in results:
        db = parameter_display(rb)
        da = parameter_display(ra)
        for kind in kinds:
            vb, ub = db[f"{kind}_primary"]
            va, _ = da[f"{kind}_primary"]
            change = (va / vb - 1.0) * 100.0 if math.isfinite(vb) and vb != 0 else float("nan")
            rows_csv.append({
                "sensor": sensor, "axis": axis, "parameter": kind_names[kind],
                "before": vb, "after": va, "unit": ub,
                "change_percent": change,
            })
            rows_md.append(
                f"| {'陀螺仪' if sensor == 'gyro' else '加速度计'} | {axis.upper()} "
                f"| {kind_names[kind]} | {fmt(vb)} {ub} | {fmt(va)} {ub} | {fmt(change, 3)}% |"
            )

    csv_path = out_dir / "allan_parameters_compare.csv"
    pd.DataFrame(rows_csv).to_csv(csv_path, index=False, encoding="utf-8-sig")
    md_path = out_dir / "allan_parameters_compare.md"
    md_path.write_text(
        "# 温漂补偿前后随机误差参数对比\n\n"
        "- 补偿前：24 位标定补偿（未去温漂）\n"
        "- 补偿后：24 位标定 + 3 阶温度模型温漂补偿\n"
        f"- 数据：{args.compensated_csv.name}（{len(data['sample']):,} 样本）\n\n"
        + "\n".join(rows_md) + "\n",
        encoding="utf-8",
    )
    print(f"Parameters CSV: {csv_path}")
    print(f"Parameters MD:  {md_path}")
    print(f"Compare plot:   {out_dir / 'allan_compare.png'}")
    print(f"Overview plot:  {out_dir / 'allan_compare_overview.png'}")


if __name__ == "__main__":
    main()
