"""Comprehensive analysis of the latest navigation data capture - pure Python."""
from __future__ import annotations

import csv
import argparse
import math
import pathlib
import statistics
import sys
from collections import Counter

DATA_ROOT = pathlib.Path(__file__).resolve().parent.parent / "data" / "decoded"
DATA_DIR = DATA_ROOT
G0 = 9.80665


def resolve_data_dir(value=None):
    if value:
        directory = pathlib.Path(value).expanduser().resolve()
    else:
        candidates = [path for path in DATA_ROOT.iterdir()
                      if path.is_dir() and (path / "imu.csv").is_file()]
        if not candidates:
            raise FileNotFoundError(f"没有找到采集目录：{DATA_ROOT}")
        directory = max(candidates, key=lambda path: path.stat().st_mtime)
    if not directory.is_dir():
        raise FileNotFoundError(f"采集目录不存在：{directory}")
    return directory


def load_csv(path):
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def col(rows, key, cast=float):
    return [cast(r[key]) for r in rows]


def stats(arr, name="", unit="", prec=4):
    if not arr:
        return
    m = statistics.mean(arr)
    s = statistics.pstdev(arr)
    mn, mx = min(arr), max(arr)
    print(f"  {name}: mean={m:.{prec}f}{unit}, std={s:.{prec}f}{unit}, "
          f"min={mn:.{prec}f}{unit}, max={mx:.{prec}f}{unit}, range={mx-mn:.{prec}f}{unit}")
    return m, s, mn, mx


def percentile(arr, p):
    if not arr:
        return 0
    s = sorted(arr)
    k = (len(s) - 1) * p / 100
    f = math.floor(k)
    c = math.ceil(k)
    if f == c:
        return s[int(k)]
    return s[f] + (s[c] - s[f]) * (k - f)


def analyze_imu(rows):
    print("=" * 70)
    print("IMU 数据分析 (MPU6050, 100Hz目标)")
    print("=" * 70)
    n = len(rows)
    print(f"总样本数: {n:,}")

    sample = col(rows, "sample", int)
    timer_us = col(rows, "timer_us", int)
    dt_s = [float(r["dt_s"]) for r in rows]
    time_s = [float(r["time_s"]) for r in rows]
    time_valid = col(rows, "time_valid", int)

    sample_diffs = [sample[i] - sample[i-1] for i in range(1, n)]
    lost = sum(d - 1 for d in sample_diffs if d > 1)
    dup_or_back = sum(1 for d in sample_diffs if d <= 0)
    print(f"sample号范围: {sample[0]} ~ {sample[-1]} (跨度 {sample[-1]-sample[0]+1})")
    print(f"丢帧(跳过的sample号): {lost:,}")
    print(f"重复/回退sample: {dup_or_back}")

    duration = time_s[-1] - time_s[0]
    effective_rate = (n - 1) / duration if duration > 0 else 0
    dt_valid = [d for d in dt_s if d > 0]
    print(f"采集时长: {duration:.2f} s ({duration/60:.1f} min)")
    print(f"有效采样率: {effective_rate:.2f} Hz")
    dt_mean = statistics.mean(dt_valid) * 1000
    dt_std = statistics.pstdev(dt_valid) * 1000
    print(f"dt统计: mean={dt_mean:.3f}ms, std={dt_std:.3f}ms, "
          f"min={min(dt_valid)*1000:.3f}ms, max={max(dt_valid)*1000:.3f}ms")
    dt_jitter = [abs(dt_valid[i] - dt_valid[i-1]) * 1000 for i in range(1, len(dt_valid))]
    print(f"dt抖动(相邻差): mean={statistics.mean(dt_jitter):.3f}ms, "
          f"max={max(dt_jitter):.3f}ms, p99={percentile(dt_jitter, 99):.3f}ms")
    print(f"GPS时间有效比例: {sum(time_valid)/n*100:.1f}%")

    ax = col(rows, "ax_m_s2")
    ay = col(rows, "ay_m_s2")
    az = col(rows, "az_m_s2")
    temp = col(rows, "temp_deg_c")

    print(f"\n加速度计 (m/s²):")
    stats(ax, "Ax")
    stats(ay, "Ay")
    stats(az, "Az")
    accel_norm = [math.sqrt(ax[i]**2 + ay[i]**2 + az[i]**2) for i in range(n)]
    an_mean = statistics.mean(accel_norm)
    print(f"  合加速度: mean={an_mean:.4f}, std={statistics.pstdev(accel_norm):.4f}, "
          f"min={min(accel_norm):.4f}, max={max(accel_norm):.4f}")
    print(f"  与G0={G0}的偏差: {an_mean-G0:.4f} m/s² ({(an_mean/G0-1)*100:.2f}%)")

    gx = col(rows, "gx_deg_h")
    gy = col(rows, "gy_deg_h")
    gz = col(rows, "gz_deg_h")

    print(f"\n陀螺仪 (deg/h):")
    stats(gx, "Gx", prec=1)
    stats(gy, "Gy", prec=1)
    stats(gz, "Gz", prec=1)

    print(f"\n陀螺仪零偏 (deg/s):")
    for name, arr in [("Gx", gx), ("Gy", gy), ("Gz", gz)]:
        print(f"  {name}: mean={statistics.mean(arr)/3600:.4f}, std={statistics.pstdev(arr)/3600:.4f}")

    print(f"\n温度: mean={statistics.mean(temp):.2f}°C, std={statistics.pstdev(temp):.3f}°C, "
          f"min={min(temp):.2f}°C, max={max(temp):.2f}°C, 变化={max(temp)-min(temp):.3f}°C")

    print(f"\n噪声快速评估(前10000样本标准差):")
    for name, arr in [("Ax", ax), ("Ay", ay), ("Az", az)]:
        seg = arr[:10000]
        print(f"  {name} std={statistics.pstdev(seg):.4f} m/s² = {statistics.pstdev(seg)/G0*1000:.1f} mg")
    for name, arr in [("Gx", gx), ("Gy", gy), ("Gz", gz)]:
        seg = arr[:10000]
        print(f"  {name} std={statistics.pstdev(seg):.1f} deg/h = {statistics.pstdev(seg)/3600*1000:.2f} mdeg/s")

    ax_raw = col(rows, "ax_raw", int)
    ay_raw = col(rows, "ay_raw", int)
    az_raw = col(rows, "az_raw", int)
    gx_raw = col(rows, "gx_raw", int)
    gy_raw = col(rows, "gy_raw", int)
    gz_raw = col(rows, "gz_raw", int)
    print(f"\n原始值范围(±2g/±250dps量程):")
    print(f"  Accel raw: Ax[{min(ax_raw)},{max(ax_raw)}] Ay[{min(ay_raw)},{max(ay_raw)}] Az[{min(az_raw)},{max(az_raw)}]")
    print(f"  Gyro raw: Gx[{min(gx_raw)},{max(gx_raw)}] Gy[{min(gy_raw)},{max(gy_raw)}] Gz[{min(gz_raw)},{max(gz_raw)}]")
    all_accel_raw = ax_raw + ay_raw + az_raw
    all_gyro_raw = gx_raw + gy_raw + gz_raw
    accel_clip = sum(1 for v in all_accel_raw if abs(v) >= 32760)
    gyro_clip = sum(1 for v in all_gyro_raw if abs(v) >= 32760)
    print(f"  加速度计饱和点数: {accel_clip}, 陀螺仪饱和点数: {gyro_clip}")


def analyze_gnss(rows):
    print("\n" + "=" * 70)
    print("GNSS 数据分析 (u-blox F9P, 1Hz目标)")
    print("=" * 70)
    n = len(rows)
    print(f"总历元数: {n:,}")

    week = col(rows, "gps_week", int)
    tow_ms = col(rows, "gps_tow_ms", int)
    fix = col(rows, "fix", int)
    num_sv = col(rows, "num_sv", int)
    carr_soln = col(rows, "carr_soln", int)
    gnss_fix_ok = col(rows, "gnss_fix_ok", int)
    diff_soln = col(rows, "diff_soln", int)
    lat = col(rows, "lat_deg")
    lon = col(rows, "lon_deg")
    hmsl = col(rows, "hmsl_m")
    h_acc = col(rows, "h_acc_m")
    v_acc = col(rows, "v_acc_m")
    vn = col(rows, "vel_n_m_s")
    ve = col(rows, "vel_e_m_s")
    vd = col(rows, "vel_d_m_s")
    ground = col(rows, "ground_speed_m_s")
    s_acc = col(rows, "s_acc_m_s")
    pdop = col(rows, "pdop")

    tow_s = [t / 1000.0 for t in tow_ms]
    tow_diffs = [tow_s[i] - tow_s[i-1] for i in range(1, n)]
    duration = tow_s[-1] - tow_s[0]
    print(f"时间跨度: W{week[0]} {tow_s[0]:.3f}s ~ W{week[-1]} {tow_s[-1]:.3f}s")
    print(f"采集时长: {duration:.1f} s ({duration/60:.1f} min)")
    print(f"历元间隔: mean={statistics.mean(tow_diffs):.3f}s, std={statistics.pstdev(tow_diffs):.3f}s, "
          f"min={min(tow_diffs):.3f}s, max={max(tow_diffs):.3f}s")
    gaps = [d for d in tow_diffs if d > 1.5]
    print(f"间隔>1.5s的次数: {len(gaps)}" + (f", 最大间隔: {max(tow_diffs):.3f}s" if gaps else ""))

    fix_names = {0: "无定位", 1: "航位推算", 2: "2D", 3: "3D", 4: "GNSS+DR", 5: "仅时间"}
    fix_counts = Counter(fix)
    print(f"\n定位类型分布:")
    for ftype in sorted(fix_counts):
        print(f"  {fix_names.get(ftype, str(ftype))}({ftype}): {fix_counts[ftype]} ({fix_counts[ftype]/n*100:.1f}%)")

    carrier_names = {0: "无RTK", 1: "RTK浮点", 2: "RTK固定"}
    carrier_counts = Counter(carr_soln)
    print(f"\n载波解算状态:")
    for c in sorted(carrier_counts):
        print(f"  {carrier_names.get(c, str(c))}({c}): {carrier_counts[c]} ({carrier_counts[c]/n*100:.1f}%)")
    print(f"gnss_fix_ok=1比例: {sum(gnss_fix_ok)/n*100:.1f}%")
    print(f"diff_soln=1(差分)比例: {sum(diff_soln)/n*100:.1f}%")

    print(f"\n卫星数: mean={statistics.mean(num_sv):.1f}, std={statistics.pstdev(num_sv):.1f}, "
          f"min={min(num_sv)}, max={max(num_sv)}")
    print(f"PDOP: mean={statistics.mean(pdop):.2f}, std={statistics.pstdev(pdop):.2f}, "
          f"min={min(pdop):.2f}, max={max(pdop):.2f}")
    print(f"PDOP>2.5比例: {sum(1 for p in pdop if p>2.5)/n*100:.1f}%, "
          f"PDOP>4比例: {sum(1 for p in pdop if p>4)/n*100:.1f}%")

    print(f"\n水平精度hAcc: mean={statistics.mean(h_acc):.3f}m, std={statistics.pstdev(h_acc):.3f}m, "
          f"min={min(h_acc):.3f}m, max={max(h_acc):.3f}m")
    print(f"垂直精度vAcc: mean={statistics.mean(v_acc):.3f}m, std={statistics.pstdev(v_acc):.3f}m, "
          f"min={min(v_acc):.3f}m, max={max(v_acc):.3f}m")
    print(f"速度精度sAcc: mean={statistics.mean(s_acc):.3f}m/s, std={statistics.pstdev(s_acc):.3f}m/s, "
          f"min={min(s_acc):.3f}m/s, max={max(s_acc):.3f}m/s")

    print(f"\n位置统计(静止测试):")
    lat_mean = statistics.mean(lat)
    lon_mean = statistics.mean(lon)
    lat_std_m = statistics.pstdev(lat) * 111000
    lon_std_m = statistics.pstdev(lon) * 111000 * math.cos(math.radians(lat_mean))
    print(f"  纬度: {lat_mean:.9f}° ± {lat_std_m:.2f}m (std)")
    print(f"  经度: {lon_mean:.9f}° ± {lon_std_m:.2f}m (std)")
    print(f"  高程: {statistics.mean(hmsl):.3f}m ± {statistics.pstdev(hmsl):.3f}m (std)")
    lat_range = (max(lat) - min(lat)) * 111000
    lon_range = (max(lon) - min(lon)) * 111000 * math.cos(math.radians(lat_mean))
    print(f"  纬度范围: {lat_range:.2f}m, 经度范围: {lon_range:.2f}m, 高程范围: {max(hmsl)-min(hmsl):.3f}m")

    dlat = [(lat[i] - lat_mean) * 111000 for i in range(n)]
    dlon = [(lon[i] - lon_mean) * 111000 * math.cos(math.radians(lat_mean)) for i in range(n)]
    dist_2d = [math.sqrt(dlat[i]**2 + dlon[i]**2) for i in range(n)]
    rms_2d = math.sqrt(sum(d*d for d in dist_2d) / n)
    print(f"  2D位置散布: mean={statistics.mean(dist_2d):.3f}m, RMS={rms_2d:.3f}m, "
          f"max={max(dist_2d):.3f}m, p95={percentile(dist_2d, 95):.3f}m")

    print(f"\n速度统计(静止测试):")
    for name, arr in [("Vn", vn), ("Ve", ve), ("Vd", vd)]:
        print(f"  {name}: mean={statistics.mean(arr):.4f}m/s, std={statistics.pstdev(arr):.4f}m/s, "
              f"range=[{min(arr):.4f},{max(arr):.4f}]")
    print(f"  地速: mean={statistics.mean(ground):.4f}m/s, max={max(ground):.4f}m/s, "
          f">0.1m/s比例={sum(1 for g in ground if g>0.1)/n*100:.1f}%")
    speed_3d = [math.sqrt(vn[i]**2 + ve[i]**2 + vd[i]**2) for i in range(n)]
    print(f"  3D速度: mean={statistics.mean(speed_3d):.4f}m/s, "
          f"RMS={math.sqrt(sum(s*s for s in speed_3d)/n):.4f}m/s, max={max(speed_3d):.4f}m/s")


def analyze_rawx(rows):
    print("\n" + "=" * 70)
    print("RAWX 观测数据分析")
    print("=" * 70)
    n = len(rows)
    print(f"总观测数: {n:,}")

    epochs = set()
    for r in rows:
        epochs.add((int(r["gps_week"]), r["rcv_tow_s"]))
    print(f"历元数: {len(epochs)}")
    print(f"每历元平均观测数: {n/len(epochs):.1f}")

    gnss_names = {0: "GPS", 1: "SBAS", 2: "GAL", 3: "BDS", 4: "IMES", 5: "QZSS", 6: "GLO"}
    gnss_counts = Counter(int(r["gnss_id"]) for r in rows)
    print(f"\n各系统观测数:")
    for gid in sorted(gnss_counts):
        print(f"  {gnss_names.get(gid, str(gid))}: {gnss_counts[gid]} ({gnss_counts[gid]/n*100:.1f}%)")

    sig_counts = Counter(r["signal"] for r in rows)
    print(f"\n信号类型:")
    for sig, cnt in sig_counts.most_common():
        print(f"  {sig}: {cnt}")

    cno = [float(r["cno_dbhz"]) for r in rows]
    pr_valid = [int(r["pr_valid"]) for r in rows]
    cp_valid = [int(r["cp_valid"]) for r in rows]
    locktime = [float(r["locktime_ms"]) for r in rows]

    print(f"\n载噪比C/No: mean={statistics.mean(cno):.1f} dB-Hz, std={statistics.pstdev(cno):.1f}, "
          f"min={min(cno):.0f}, max={max(cno):.0f}")
    print(f"  C/No<30比例: {sum(1 for c in cno if c<30)/n*100:.1f}%, "
          f"C/No<25比例: {sum(1 for c in cno if c<25)/n*100:.1f}%")
    print(f"伪距有效比例: {sum(pr_valid)/n*100:.1f}%")
    print(f"载波相位有效比例: {sum(cp_valid)/n*100:.1f}%")
    print(f"锁定时间: mean={statistics.mean(locktime)/1000:.1f}s, "
          f"median={statistics.median(locktime)/1000:.1f}s, max={max(locktime)/1000:.1f}s")


def analyze_aim(rows):
    print("\n" + "=" * 70)
    print("AIM 自对准/融合数据分析 (10Hz输出)")
    print("=" * 70)
    n = len(rows)
    print(f"总输出行数: {n:,}")

    gps_time = [float(r["gps_time_s"]) for r in rows]
    elapsed = [float(r["elapsed_s"]) for r in rows]
    stage = [r["stage"] for r in rows]
    roll = [float(r["roll_deg"]) for r in rows]
    pitch = [float(r["pitch_deg"]) for r in rows]
    heading = [float(r["heading_deg"]) for r in rows]

    stage_counts = Counter(stage)
    print(f"阶段分布:")
    for s, cnt in stage_counts.most_common():
        print(f"  {s}: {cnt}行 ({cnt/n*100:.1f}%)")

    transitions = []
    for i in range(1, n):
        if stage[i] != stage[i-1]:
            transitions.append((i, elapsed[i], stage[i-1], stage[i]))
    print(f"\n阶段转换:")
    for idx, t, from_s, to_s in transitions:
        print(f"  行{idx} (elapsed={t:.1f}s): {from_s} -> {to_s}")

    fine_init = [r for r in rows if r["fine_initial_roll_deg"] != "nan"]
    if fine_init:
        r0 = fine_init[0]
        print(f"\n精对准初始AVP(首次出现):")
        print(f"  姿态: R={float(r0['fine_initial_roll_deg']):.4f}°, "
              f"P={float(r0['fine_initial_pitch_deg']):.4f}°, "
              f"H={float(r0['fine_initial_heading_deg']):.4f}°")
        print(f"  位置: lat={float(r0['fine_initial_lat_deg']):.9f}°, "
              f"lon={float(r0['fine_initial_lon_deg']):.9f}°, "
              f"h={float(r0['fine_initial_height_m']):.3f}m")
        print(f"  有效GNSS均值点数: {r0['fine_initial_position_samples']}")

    fine_idx = [i for i, s in enumerate(stage) if s == "精对准"]
    if fine_idx:
        fi = fine_idx[0]
        fl = fine_idx[-1]
        fine_elapsed = [elapsed[i] for i in fine_idx]
        fine_roll = [roll[i] for i in fine_idx]
        fine_pitch = [pitch[i] for i in fine_idx]
        fine_heading = [heading[i] for i in fine_idx]
        print(f"\n精对准阶段姿态 (持续 {fine_elapsed[-1]-fine_elapsed[0]:.1f}s):")
        print(f"  Roll:  start={fine_roll[0]:.4f}°, end={fine_roll[-1]:.4f}°, "
              f"mean={statistics.mean(fine_roll):.4f}°, std={statistics.pstdev(fine_roll):.4f}°, "
              f"range={max(fine_roll)-min(fine_roll):.4f}°")
        print(f"  Pitch: start={fine_pitch[0]:.4f}°, end={fine_pitch[-1]:.4f}°, "
              f"mean={statistics.mean(fine_pitch):.4f}°, std={statistics.pstdev(fine_pitch):.4f}°, "
              f"range={max(fine_pitch)-min(fine_pitch):.4f}°")
        print(f"  Heading: start={fine_heading[0]:.4f}°, end={fine_heading[-1]:.4f}°, "
              f"mean={statistics.mean(fine_heading):.4f}°, std={statistics.pstdev(fine_heading):.4f}°, "
              f"range={max(fine_heading)-min(fine_heading):.4f}°")

        pos_n = [float(r["pos_n_m"]) for r in rows]
        pos_e = [float(r["pos_e_m"]) for r in rows]
        pos_d = [float(r["pos_d_m"]) for r in rows]
        fn = [pos_n[i] for i in fine_idx]
        fe = [pos_e[i] for i in fine_idx]
        fd = [pos_d[i] for i in fine_idx]
        print(f"\n精对准阶段NED位置 (相对原点):")
        print(f"  N: start={fn[0]:.3f}m, end={fn[-1]:.3f}m, range=[{min(fn):.3f},{max(fn):.3f}]m")
        print(f"  E: start={fe[0]:.3f}m, end={fe[-1]:.3f}m, range=[{min(fe):.3f},{max(fe):.3f}]m")
        print(f"  D: start={fd[0]:.3f}m, end={fd[-1]:.3f}m, range=[{min(fd):.3f},{max(fd):.3f}]m")
        drift_2d = math.sqrt((fn[-1]-fn[0])**2 + (fe[-1]-fe[0])**2)
        print(f"  2D位置漂移(起点到终点): {drift_2d:.3f}m")

        vel_n = [float(r["vel_n_m_s"]) for r in rows]
        vel_e = [float(r["vel_e_m_s"]) for r in rows]
        vel_d = [float(r["vel_d_m_s"]) for r in rows]
        print(f"\n精对准阶段NED速度:")
        for name, arr in [("Vn", [vel_n[i] for i in fine_idx]),
                          ("Ve", [vel_e[i] for i in fine_idx]),
                          ("Vd", [vel_d[i] for i in fine_idx])]:
            print(f"  {name}: mean={statistics.mean(arr):.4f}m/s, std={statistics.pstdev(arr):.4f}m/s, "
                  f"max|v|={max(abs(v) for v in arr):.4f}m/s")

        gbx = [float(r["gyro_bias_x_deg_s"]) for r in rows]
        gby = [float(r["gyro_bias_y_deg_s"]) for r in rows]
        gbz = [float(r["gyro_bias_z_deg_s"]) for r in rows]
        print(f"\n陀螺零偏估计 (deg/s):")
        for name, arr in [("Bx", gbx), ("By", gby), ("Bz", gbz)]:
            fa = [arr[i] for i in fine_idx]
            print(f"  {name}: coarse_end={arr[fi]:.6f}, fine_end={fa[-1]:.6f}, "
                  f"fine_mean={statistics.mean(fa):.6f}, fine_std={statistics.pstdev(fa):.6f}")

        abx = [float(r["accel_bias_x_m_s2"]) for r in rows]
        aby = [float(r["accel_bias_y_m_s2"]) for r in rows]
        abz = [float(r["accel_bias_z_m_s2"]) for r in rows]
        print(f"\n加计零偏估计 (m/s²):")
        for name, arr in [("Bx", abx), ("By", aby), ("Bz", abz)]:
            fa = [arr[i] for i in fine_idx]
            print(f"  {name}: coarse_end={arr[fi]:.6f}, fine_end={fa[-1]:.6f}, "
                  f"fine_mean={statistics.mean(fa):.6f}, fine_std={statistics.pstdev(fa):.6f}")

        gnss_updates = [int(r["gnss_updates"]) for r in rows]
        gnss_rejections = [int(r["gnss_rejections"]) for r in rows]
        print(f"\nGNSS更新统计:")
        print(f"  最终累计更新: {gnss_updates[-1]}, 拒绝: {gnss_rejections[-1]}")
        fine_updates = gnss_updates[fl] - gnss_updates[fi]
        fine_rejects = gnss_rejections[fl] - gnss_rejections[fi]
        total = fine_updates + fine_rejects
        if total > 0:
            print(f"  精对准期间: 更新{fine_updates}次, 拒绝{fine_rejects}次, 接受率={fine_updates/total*100:.1f}%")

        last_reason = [r["last_gnss_reason"] for r in rows]
        reason_counts = Counter(last_reason)
        print(f"  last_gnss_reason分布:")
        for reason, cnt in reason_counts.most_common(10):
            print(f"    {reason}: {cnt}行")

        time_match = []
        for r in rows:
            v = r["time_match_s"]
            if v != "nan":
                time_match.append(float(v))
        if time_match:
            print(f"\nGNSS时间匹配差: mean={statistics.mean(time_match):.4f}s, "
                  f"std={statistics.pstdev(time_match):.4f}s, "
                  f"min={min(time_match):.4f}s, max={max(time_match):.4f}s")

        std_roll = [float(r["std_att_roll_deg"]) for r in rows]
        std_pitch = [float(r["std_att_pitch_deg"]) for r in rows]
        std_heading = [float(r["std_att_heading_deg"]) for r in rows]
        std_pos_n = [float(r["std_pos_n_m"]) for r in rows]
        std_pos_e = [float(r["std_pos_e_m"]) for r in rows]
        std_pos_d = [float(r["std_pos_d_m"]) for r in rows]
        print(f"\n协方差收敛(精对准阶段):")
        print(f"  姿态1σ: Roll {std_roll[fi]:.4f}°→{std_roll[fl]:.4f}°, "
              f"Pitch {std_pitch[fi]:.4f}°→{std_pitch[fl]:.4f}°, "
              f"Heading {std_heading[fi]:.4f}°→{std_heading[fl]:.4f}°")
        print(f"  位置1σ: N {std_pos_n[fi]:.3f}→{std_pos_n[fl]:.3f}m, "
              f"E {std_pos_e[fi]:.3f}→{std_pos_e[fl]:.3f}m, "
              f"D {std_pos_d[fi]:.3f}→{std_pos_d[fl]:.3f}m")


def analyze_event_log():
    print("\n" + "=" * 70)
    print("事件日志")
    print("=" * 70)
    log_path = DATA_DIR / "event.log"
    if not log_path.is_file():
        print("  本次未保存 event.log")
        return
    raw = log_path.read_bytes()
    for enc in ("utf-8", "gbk", "utf-8-sig"):
        try:
            content = raw.decode(enc)
            break
        except UnicodeDecodeError:
            continue
    else:
        content = raw.decode("utf-8", errors="replace")
    for line in content.strip().split("\n"):
        print(f"  {line}")


def main():
    global DATA_DIR
    parser = argparse.ArgumentParser(description="分析一组Qt采集数据；默认选择最近修改的会话")
    parser.add_argument("data_dir", nargs="?", help="会话目录，例如 data/decoded/20260911001503")
    args = parser.parse_args()
    DATA_DIR = resolve_data_dir(args.data_dir)
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    print(f"数据目录: {DATA_DIR}")
    print(f"文件大小:")
    for f in sorted(DATA_DIR.iterdir()):
        print(f"  {f.name}: {f.stat().st_size:,} bytes")

    imu_rows = load_csv(DATA_DIR / "imu.csv")
    gnss_rows = load_csv(DATA_DIR / "gnss.csv")
    rawx_rows = load_csv(DATA_DIR / "rawx.csv")
    aim_rows = load_csv(DATA_DIR / "aim.csv")

    analyze_imu(imu_rows)
    analyze_gnss(gnss_rows)
    analyze_rawx(rawx_rows)
    analyze_aim(aim_rows)
    analyze_event_log()

    print("\n" + "=" * 70)
    print("交叉验证: IMU-GNSS 时间同步")
    print("=" * 70)
    imu_first_gps = int(imu_rows[0]["gps_week"]) * 604800 + int(imu_rows[0]["gps_tow_us"]) / 1e6
    imu_last_gps = int(imu_rows[-1]["gps_week"]) * 604800 + int(imu_rows[-1]["gps_tow_us"]) / 1e6
    gnss_first_gps = int(gnss_rows[0]["gps_week"]) * 604800 + int(gnss_rows[0]["gps_tow_ms"]) / 1000
    gnss_last_gps = int(gnss_rows[-1]["gps_week"]) * 604800 + int(gnss_rows[-1]["gps_tow_ms"]) / 1000
    print(f"IMU GPS时间范围: {imu_first_gps:.3f} ~ {imu_last_gps:.3f} (跨度{imu_last_gps-imu_first_gps:.1f}s)")
    print(f"GNSS GPS时间范围: {gnss_first_gps:.3f} ~ {gnss_last_gps:.3f} (跨度{gnss_last_gps-gnss_first_gps:.1f}s)")
    print(f"IMU先于GNSS: {imu_first_gps-gnss_first_gps:.3f}s")
    print(f"IMU/GNSS样本比: {len(imu_rows)/len(gnss_rows):.1f}:1 (期望100:1)")

    valid_imu = [r for r in imu_rows if int(r["time_valid"]) == 1]
    if len(valid_imu) > 10:
        timer = [int(r["timer_us"]) for r in valid_imu]
        tow = [int(r["gps_tow_us"]) for r in valid_imu]
        timer_elapsed = [(t - timer[0]) / 1e6 for t in timer]
        gps_elapsed = [(t - tow[0]) / 1e6 for t in tow]
        drift = [timer_elapsed[i] - gps_elapsed[i] for i in range(len(valid_imu))]
        print(f"STM32定时器 vs GPS时间漂移: 起始={drift[0]:.3f}s, 结束={drift[-1]:.3f}s, "
              f"最大={max(abs(d) for d in drift):.3f}s")
        if gps_elapsed[-1] > gps_elapsed[0]:
            drift_rate = (drift[-1] - drift[0]) / (gps_elapsed[-1] - gps_elapsed[0]) * 1e6
            print(f"漂移速率: {drift_rate:.1f} ppm")


if __name__ == "__main__":
    main()
