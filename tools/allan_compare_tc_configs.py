#!/usr/bin/env python3
"""Three-config Allan-variance comparison for the MPU6050 temp-compensation study.

Stages compared on the same long static recording (21 h outdoor):

  1. raw            recorded IMU data, nothing applied
  2. raw+TC         per-axis temperature compensation (no 0-order), only axes
                    whose enable bit is 1 (default az 3rd / gx 5th / gy 5th)
  3. raw+TC+calib   stage 2 plus the MATCHING 24-position calibration
                    (a_cal = Ca*(a_tc - ba), g_cal = g_tc - gb)

Rationale: with GNSS correcting at 1 Hz and outages shorter than 600 s, the
figure of merit is short/medium tau behaviour, not multi-hour Allan numbers.
This tool therefore reports Allan deviation at tau = 30/60/180/300/600/1000/3600 s
plus the identified stochastic parameters, and a 600 s block-mean drift std.

Usage:
  python tools/allan_compare_tc_configs.py data/decoded/<session>/imu.csv \
      --coeff data/calib24/temp_coeffs_raw.csv \
      --calib data/calib24/calib24_result_tempcomp_azgxgy.mat \
      --enable 0,0,1,1,1,0

Outputs (default <imu dir>/allan_compare_tc_configs/):
  allan_three_configs.png        2x3 grid, 3 stage curves per axis
  allan_adev_at_tau.csv          adev at fixed tau + 600 s block std, per stage/axis
  allan_three_configs_params.csv identified ARW/VRW, BI, RRW, Ramp per stage/axis
  allan_deviation.csv            full adev curves (tau + 18 columns)
  温补配置Allan对比报告.md        Chinese summary report
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

G0 = 9.80665                      # tool convention for mg display
DEG_H_TO_RAD_S = math.pi / 180.0 / 3600.0
AXES = ["ax", "ay", "az", "gx", "gy", "gz"]
STAGES = ["raw", "raw+TC", "raw+TC+calib"]
TAU_TARGETS = [30.0, 60.0, 180.0, 300.0, 600.0, 1000.0, 3600.0]
BLOCK_SEC = 600.0

plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "Noto Sans CJK SC", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False


def adev_at_tau(values: np.ndarray, period: float, taus) -> dict[float, float]:
    """Overlapping Allan deviation evaluated exactly at the requested taus."""
    v = np.asarray(values, dtype=np.float64)
    v = v - v.mean()
    n = v.size
    integral = np.empty(n + 1, dtype=np.float64)
    integral[0] = 0.0
    np.cumsum(v, out=integral[1:])
    integral *= period
    out: dict[float, float] = {}
    for t in taus:
        m = max(1, int(round(t / period)))
        if 2 * m >= n:
            out[t] = float("nan")
            continue
        second = integral[2 * m:] - 2.0 * integral[m:-m] + integral[:-2 * m]
        var = float(np.dot(second, second)) / (second.size * 2.0 * (m * period) ** 2)
        out[t] = math.sqrt(max(var, 0.0))
    return out


def block_mean_std(values: np.ndarray, period: float, block_sec: float = BLOCK_SEC) -> float:
    m = max(1, int(round(block_sec / period)))
    nb = values.size // m
    if nb < 2:
        return float("nan")
    bm = values[: nb * m].reshape(nb, m).mean(axis=1)
    return float(np.std(bm - np.median(bm), ddof=1))


def load_coeffs(path: pathlib.Path) -> tuple[np.ndarray, np.ndarray, float, dict]:
    """Return (coef 6x6 [c5..c0], orders 6, Tref, MAT metadata)."""
    cf = pd.read_csv(path)
    required = {"axis", "order", "c0", "c1", "c2", "c3", "c4", "c5"}
    missing = required.difference(cf.columns)
    if missing:
        raise SystemExit(f"--coeff: missing columns {sorted(missing)}")
    if len(cf) != 6:
        raise SystemExit("--coeff: exactly six axis rows are required")
    coef = cf[["c5", "c4", "c3", "c2", "c1", "c0"]].to_numpy(dtype=np.float64)
    orders = cf["order"].to_numpy(dtype=int)
    if not np.isfinite(coef).all() or np.any((orders < 0) | (orders > 5)):
        raise SystemExit("--coeff: invalid polynomial coefficients or orders")
    mat_path = path.with_suffix(".mat")
    if mat_path.is_file():
        meta = sio.loadmat(mat_path, simplify_cells=True)
        tref = float(meta["Tref"])
        if not np.allclose(np.asarray(meta["coef"], dtype=float), coef,
                           rtol=1e-13, atol=1e-12):
            raise SystemExit("--coeff: CSV and companion MAT coefficient matrices differ")
    else:
        raise SystemExit(f"--coeff: companion MAT metadata is required (missing {mat_path})")
    return coef, orders, tref, meta


def calibration_field(cal: np.ndarray, name: str) -> np.ndarray:
    if cal.dtype.names is None or name not in cal.dtype.names:
        raise SystemExit(f"--calib: result.{name} is required")
    return np.asarray(cal[name]).squeeze()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("csv", type=pathlib.Path, help="Physical-unit IMU CSV (imu.csv)")
    parser.add_argument("--coeff", type=pathlib.Path, required=True,
                        help="temp_coeffs_raw.csv (axis,order,c0..c5)")
    parser.add_argument("--calib", type=pathlib.Path, required=True,
                        help="matching 24-pos calibration mat (calib24_result_tempcomp_*.mat)")
    parser.add_argument("--enable", default="0,0,1,1,1,0",
                        help="per-axis TC enable ax,ay,az,gx,gy,gz (default 0,0,1,1,1,0)")
    parser.add_argument("--rate", type=float, default=None,
                        help="override sample rate; default derives it from time_s")
    parser.add_argument("--points", type=int, default=90)
    parser.add_argument("--tref", type=float, default=None, help="override Tref")
    parser.add_argument("--output", type=pathlib.Path, default=None)
    args = parser.parse_args()

    try:
        enable_bits = np.array([int(v) for v in args.enable.split(",")], dtype=int)
    except ValueError as exc:
        raise SystemExit("--enable needs 6 comma-separated 0/1 values") from exc
    if enable_bits.size != 6 or not np.isin(enable_bits, [0, 1]).all():
        raise SystemExit("--enable needs 6 comma-separated bits")
    enable = enable_bits.astype(bool)

    print(f"Loading {args.csv} ...", flush=True)
    cols = ["sample", "time_s", "ax_m_s2", "ay_m_s2", "az_m_s2",
            "temp_deg_c", "gx_deg_h", "gy_deg_h", "gz_deg_h"]
    df = pd.read_csv(args.csv, usecols=cols, dtype=np.float64)
    if not np.isfinite(df.to_numpy()).all():
        raise SystemExit("input contains NaN/Inf; refusing to concatenate samples silently")
    n = len(df)
    times = df["time_s"].to_numpy()
    dt = np.diff(times)
    sample_step = np.diff(df["sample"].to_numpy())
    if np.any(dt <= 0) or np.any(sample_step != 1):
        raise SystemExit("input time/sample sequence is not strictly contiguous")
    median_period = float(np.median(dt))
    if np.any(dt > 1.5 * median_period):
        raise SystemExit("input contains a time gap; split the recording before comparison")
    period = 1.0 / args.rate if args.rate else float((times[-1] - times[0]) / (n - 1))
    duration_h = (n - 1) * period / 3600.0
    print(f"samples: {n:,}  duration: {duration_h:.2f} h", flush=True)

    coef, orders, tref, coeff_meta = load_coeffs(args.coeff)
    if args.tref is not None:
        tref = args.tref
    tcpoly = np.hstack([coef[:, 0:5], np.zeros((6, 1))])   # [c5 c4 c3 c2 c1 0]

    active_from_model = orders > 0
    if not np.array_equal(enable, active_from_model):
        raise SystemExit(
            f"--enable {enable_bits.tolist()} differs from coefficient orders>0 "
            f"{active_from_model.astype(int).tolist()}")

    cal = sio.loadmat(args.calib)["result"][0, 0]
    ba = calibration_field(cal, "ba").astype(np.float64).reshape(3)
    Ca = calibration_field(cal, "Ca").astype(np.float64).reshape(3, 3)
    gb = calibration_field(cal, "gyroBiasMean_deg_h").astype(np.float64).reshape(3)
    cal_coeff = calibration_field(cal, "tempCoeff").astype(np.float64).reshape(6, 6)
    cal_tref = float(calibration_field(cal, "Tref"))
    cal_active = calibration_field(cal, "tcActive").astype(bool).reshape(6)
    if not np.allclose(cal_coeff, tcpoly, rtol=1e-13, atol=1e-12):
        raise SystemExit("--calib and --coeff contain different temperature models")
    if abs(cal_tref - tref) > 1e-10:
        raise SystemExit("--calib and --coeff use different reference temperatures")
    if not np.array_equal(cal_active, enable):
        raise SystemExit("--calib active axes differ from --enable")
    print(f"T0 = {tref:.4f} C; orders {orders.astype(int)}; enable {enable.astype(int)}")
    print(f"ba = {np.round(ba, 5)}  gb[deg/h] = {np.round(gb, 2)}")

    temp = df["temp_deg_c"].to_numpy()
    tmin = float(coeff_meta["Tmin"])
    tmax = float(coeff_meta["Tmax"])
    if temp.min() < tmin - 1e-9 or temp.max() > tmax + 1e-9:
        raise SystemExit(
            f"input temperature [{temp.min():.4f}, {temp.max():.4f}] C exceeds "
            f"fitted range [{tmin:.4f}, {tmax:.4f}] C")
    dT = temp - tref
    raw = df[["ax_m_s2", "ay_m_s2", "az_m_s2", "gx_deg_h", "gy_deg_h", "gz_deg_h"]].to_numpy()

    # stage 2: per-axis temperature compensation (no 0-order), masked
    tc = np.array([np.polyval(tcpoly[k], dT) if enable[k] else np.zeros(n)
                   for k in range(6)]).T
    stage2 = raw - tc

    # stage 3: matching 24-pos calibration on top of stage 2
    a_cal = (stage2[:, 0:3] - ba) @ Ca.T
    stage3 = stage2.copy()
    stage3[:, 0:3] = a_cal
    stage3[:, 3:6] = stage2[:, 3:6] - gb

    stages = {"raw": raw, "raw+TC": stage2, "raw+TC+calib": stage3}

    # ---- per-axis analysis ----
    results: dict[tuple[str, str], ani.AxisResult] = {}
    tau_table: list[dict] = []
    curves: dict[tuple[str, str], tuple[np.ndarray, np.ndarray]] = {}

    for k, axis in enumerate(AXES):
        is_gyro = k >= 3
        scale = DEG_H_TO_RAD_S if is_gyro else 1.0
        for stage in STAGES:
            values = stages[stage][:, k] * scale
            unit = "rad/s" if is_gyro else "m/s^2"
            res = ani.analyze_axis("gyro" if is_gyro else "accel", axis[-1],
                                   values, unit, period, args.points, n * period)
            results[(stage, axis)] = res
            curves[(stage, axis)] = (res.tau, res.adev)

        # fixed-tau table in display units (mg / deg/h)
        conv = 1.0 / G0 * 1000.0 if not is_gyro else 180.0 / math.pi * 3600.0
        unit = "mg" if not is_gyro else "deg/h"
        row = {"axis": axis, "unit": unit}
        for stage in STAGES:
            values = stages[stage][:, k] * scale    # SI: m/s^2 或 rad/s
            adevs = adev_at_tau(values, period, TAU_TARGETS)
            row[f"{stage}_block600_std"] = block_mean_std(values, period) * conv
            for t in TAU_TARGETS:
                row[f"{stage}_adev_{int(t)}s"] = adevs[t] * conv
        tau_table.append(row)
        print(f"  {axis}: adev@600s raw/TC/TC+calib = "
              f"{row['raw_adev_600s']:.4g} / {row['raw+TC_adev_600s']:.4g} / "
              f"{row['raw+TC+calib_adev_600s']:.4g} {row['unit']}", flush=True)

    out = args.output or (args.csv.resolve().parent / "allan_compare_tc_configs")
    out.mkdir(parents=True, exist_ok=True)

    # ---- figure ----
    colors = {"raw": "#8c8c8c", "raw+TC": "#1f77b4", "raw+TC+calib": "#d62728"}
    fig, axes = plt.subplots(2, 3, figsize=(16, 9), constrained_layout=True)
    for idx, axis in enumerate(AXES):
        ax = axes.flat[idx]
        for stage in STAGES:
            tau, adev = curves[(stage, axis)]
            conv = (180.0 / math.pi * 3600.0) if idx >= 3 else (1.0 / G0 * 1000.0)
            ax.loglog(tau, adev * conv, color=colors[stage], lw=1.6,
                      label=stage)
        for t in TAU_TARGETS:
            ax.axvline(t, color="#bbbbbb", lw=0.5, ls=":", zorder=0)
        ax.set(title=f"{axis.upper()}  (TC {'on' if enable[idx] else 'off'}, "
                    f"order {int(orders[idx])})",
               xlabel="tau (s)",
               ylabel="Allan deviation (deg/h)" if idx >= 3 else "Allan deviation (mg)")
        ax.grid(True, which="both", alpha=0.30)
        if idx == 0:
            ax.legend(fontsize=9)
    fig.suptitle("温补三方案 Allan 对比 (raw / raw+TC / raw+TC+calib, "
                 f"enable={''.join(str(int(e)) for e in enable)})", fontsize=14)
    fig.savefig(out / "allan_three_configs.png", dpi=160)
    plt.close(fig)

    # ---- CSVs ----
    tau_cols = ["axis", "unit"] + [f"{s}_block600_std" for s in STAGES] + \
               [f"{s}_adev_{int(t)}s" for s in STAGES for t in TAU_TARGETS]
    pd.DataFrame(tau_table)[tau_cols].to_csv(out / "allan_adev_at_tau.csv",
                                             index=False, encoding="utf-8-sig")

    param_rows = []
    for (stage, axis), res in results.items():
        disp = ani.parameter_display(res)
        param_rows.append({
            "stage": stage, "axis": axis,
            "arw_vrw": disp["white_primary"][0], "arw_vrw_unit": disp["white_primary"][1],
            "bias_instability": disp["bias_primary"][0], "bias_instability_unit": disp["bias_primary"][1],
            "rrw": disp["rrw_primary"][0], "rrw_unit": disp["rrw_primary"][1],
            "rate_ramp": disp["ramp_primary"][0], "rate_ramp_unit": disp["ramp_primary"][1],
        })
    pd.DataFrame(param_rows).to_csv(out / "allan_three_configs_params.csv",
                                    index=False, encoding="utf-8-sig")

    tau0 = curves[("raw", "ax")][0]
    curve_cols = [tau0] + [curves[(s, a)][1] for s in STAGES for a in AXES]
    header = "tau_s," + ",".join(f"{s}_{a}_adev_rad_s_or_m_s2" for s in STAGES for a in AXES)
    np.savetxt(out / "allan_deviation.csv", np.column_stack(curve_cols),
               delimiter=",", header=header, comments="")

    # ---- markdown report ----
    def fmt(v):
        return "—" if not math.isfinite(v) else f"{v:.4g}"

    lines = [
        "# 温补三方案 Allan 对比报告（raw / raw+TC / raw+TC+calib）",
        "",
        f"- 数据：`{args.csv.name}`（{duration_h:.2f} h 静态，{1/period:.4f} Hz）",
        f"- 温补系数：`{args.coeff.name}`（T0 = {tref:.4f} °C，各轴阶数 "
        f"{'/'.join(str(int(o)) for o in orders)}）",
        f"- 轴开关 enable = {''.join(str(int(e)) for e in enable)} "
        "（1=温补，0=不补；不补的轴 raw 与 raw+TC 完全相同）",
        f"- 配套标定：`{args.calib.name}`（ba / Ca / gb 与该温补配置配套，不可与其他配置混用）",
        "- 场景前提：GNSS 1 Hz 修正、失锁 ≤ 600 s；因此关注 30~1000 s 的 Allan 行为，"
        "数小时尺度仅作参考。",
        "",
        "## 1. 固定 τ 的 Allan 偏差（mg / deg/h）",
        "",
        "括号内为相对 raw 的变化率，负值=改善。",
    ]
    for row in tau_table:
        lines.append(f"\n### {row['axis'].upper()}（{row['unit']}）\n")
        lines.append("| τ (s) | raw | raw+TC | raw+TC+calib |")
        lines.append("|---:|---:|---:|---:|")
        for t in TAU_TARGETS:
            r0 = row[f"raw_adev_{int(t)}s"]
            r1 = row[f"raw+TC_adev_{int(t)}s"]
            r2 = row[f"raw+TC+calib_adev_{int(t)}s"]
            d1 = f" ({100*(r1/r0-1):+.1f}%)" if math.isfinite(r0) and r0 > 0 else ""
            d2 = f" ({100*(r2/r0-1):+.1f}%)" if math.isfinite(r0) and r0 > 0 else ""
            lines.append(f"| {int(t)} | {fmt(r0)} | {fmt(r1)}{d1} | {fmt(r2)}{d2} |")
        b0 = row["raw_block600_std"]; b1 = row["raw+TC_block600_std"]; b2 = row["raw+TC+calib_block600_std"]
        lines.append(f"\n600 s 块均值漂移 std：raw {fmt(b0)} → raw+TC {fmt(b1)}"
                     f" → raw+TC+calib {fmt(b2)} {row['unit']}")
        lines.append("")

    lines += [
        "## 2. 随机误差辨识参数（自动幂律拟合）",
        "",
        "| stage | 轴 | ARW/VRW | BI | RRW | Rate Ramp |",
        "|---|---|---:|---:|---:|---:|",
    ]
    for row in param_rows:
        lines.append("| {stage} | {axis} | {w} {wu} | {b} {bu} | {r} {ru} | {p} {pu} |".format(
            stage=row["stage"], axis=row["axis"].upper(),
            w=fmt(row["arw_vrw"]), wu=row["arw_vrw_unit"],
            b=fmt(row["bias_instability"]), bu=row["bias_instability_unit"],
            r=fmt(row["rrw"]), ru=row["rrw_unit"],
            p=fmt(row["rate_ramp"]), pu=row["rate_ramp_unit"]))
    lines += [
        "",
        "## 3. 判读要点与局限",
        "",
        "- **Allan 静态指标 ≠ 失锁位置误差**：失锁误差还取决于姿态误差、初始状态与误差传播，"
        "本报告只回答\"各温补方案在 30~1000 s 尺度的静态平滑度\"，不直接等于 600 s 失锁精度。",
        "- GNSS 1 Hz 修正下，滤波器在线估计的是**温补+标定后的剩余零偏**；温补的意义是把"
        "需要滤波器跟踪的零偏动态范围压小，因此重点看 raw+TC+calib 相对 raw 在 30~600 s 的变化。",
        "- **raw+TC 与 raw+TC+calib 曲线基本重合属预期**：Allan 偏差计算本身先扣除均值，"
        "而标定只是减常值零偏（陀螺 gb）加常数矩阵（Ca），不改变曲线形状——陀螺两方案完全重合，"
        "加计仅因 Ca 交叉耦合有 <1% 差异。标定的作用体现在均值/零偏层面（ba/gb），不体现在 Allan 曲线上。",
        "- **az 的权衡**：180~1000 s 改善 6~18%，3600 s 反而 +5~7%——3 阶模型针对的正是"
        "GNSS 失锁 ≤600 s 工况关心的短中期区间，超长周期变差不构成否决理由。",
        "- 所有数据为静态场景，无运动/振动激励，结论不外推到动态工况。",
        "- 加计温补量近常数（24 位置 ΔT 仅 0.71 °C），其收益主要在长 τ；短 τ 由噪声主导，"
        "三个方案在 30~60 s 处应基本重合，属预期。",
        "- 自动幂律拟合受温度趋势污染的长 τ 区间影响，RRW/Ramp 数值仅作曲线候选。",
        "",
        "## 4. 输出文件",
        "",
        "- `allan_three_configs.png`：六轴 Allan 曲线，三方案叠加，虚线标出 30~3600 s 关注点。",
        "- `allan_adev_at_tau.csv`：固定 τ 的 adev 与 600 s 块均值漂移 std。",
        "- `allan_three_configs_params.csv`：三方案各自的 ARW/VRW、BI、RRW、Ramp。",
        "- `allan_deviation.csv`：完整 Allan 曲线（SI 单位）。",
    ]
    (out / "温补配置Allan对比报告.md").write_text("\n".join(lines) + "\n", encoding="utf-8")

    print(f"\noutputs in: {out}")
    print("DONE")


if __name__ == "__main__":
    main()
