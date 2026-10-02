"""精确分析精对准发散临界点：速度新息、姿态、零偏的演化。"""
import csv
import math
import os

DATA = r"C:\Users\12597\Desktop\lowcost\stm32-mpu6050-f9p-navigation\data\decoded\20260910142516"

def read_aim():
    rows = []
    with open(os.path.join(DATA, "aim.csv"), encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for r in reader:
            rows.append(r)
    return rows

def read_gnss():
    rows = []
    with open(os.path.join(DATA, "gnss.csv"), encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for r in reader:
            rows.append(r)
    return rows

aim = read_aim()
gnss = read_gnss()

# 精对准起始 elapsed = 30.0s
FINE_START = 30.0

# 找到每个GNSS历元对应的最近aim行（按gps_time匹配）
# aim.csv有gps_time_s列
aim_by_time = {}
for r in aim:
    try:
        t = float(r["gps_time_s"])
        aim_by_time[t] = r
    except (ValueError, KeyError):
        pass

aim_times = sorted(aim_by_time.keys())

def nearest_aim(t):
    """二分查找最近的aim时间戳"""
    lo, hi = 0, len(aim_times) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if aim_times[mid] < t:
            lo = mid + 1
        else:
            hi = mid
    if lo > 0 and abs(aim_times[lo-1] - t) < abs(aim_times[lo] - t):
        lo -= 1
    return aim_by_time[aim_times[lo]]

print("=== GNSS速度新息随精对准时间演化（每10秒采样） ===")
print(f"{'fine_elapsed':>12} {'elapsed':>8} {'Vn_filt':>8} {'Ve_filt':>8} {'Vd_filt':>8} "
      f"{'Vn_gnss':>8} {'Ve_gnss':>8} {'Vd_gnss':>8} {'innov_max':>10} {'updates':>7} {'reject':>6} {'reason':>20}")
print("-" * 130)

GPS_WEEK_SECONDS = 604800.0
last_fine_elapsed_shown = -1
for g in gnss:
    try:
        week = float(g["gps_week"]); tow_ms = float(g["gps_tow_ms"])
        gps_t = week * GPS_WEEK_SECONDS + tow_ms / 1000.0
        vn_g = float(g["vel_n_m_s"]); ve_g = float(g["vel_e_m_s"]); vd_g = float(g["vel_d_m_s"])
    except (ValueError, KeyError):
        continue
    a = nearest_aim(gps_t)
    try:
        elapsed = float(a["elapsed_s"])
        stage = a["stage"]
        if stage != "精对准":
            continue
        fine_el = elapsed - FINE_START
        vn_f = float(a["vel_n_m_s"]); ve_f = float(a["vel_e_m_s"]); vd_f = float(a["vel_d_m_s"])
        updates = int(a["gnss_updates"]); rejects = int(a["gnss_rejections"])
        reason = a["last_gnss_reason"]
    except (ValueError, KeyError):
        continue
    innov_n = vn_f - vn_g
    innov_e = ve_f - ve_g
    innov_d = vd_f - vd_g
    innov_max = max(abs(innov_n), abs(innov_e), abs(innov_d))
    # 每10秒显示一次，以及临界点附近
    if int(fine_el) % 10 == 0 and int(fine_el) != last_fine_elapsed_shown:
        last_fine_elapsed_shown = int(fine_el)
        print(f"{fine_el:12.1f} {elapsed:8.1f} {vn_f:8.3f} {ve_f:8.3f} {vd_f:8.3f} "
              f"{vn_g:8.3f} {ve_g:8.3f} {vd_g:8.3f} {innov_max:10.3f} {updates:7d} {rejects:6d} {reason:>20}")
    # 临界点附近（320-340s）全部显示
    elif 320 <= fine_el <= 345:
        print(f"{fine_el:12.1f} {elapsed:8.1f} {vn_f:8.3f} {ve_f:8.3f} {vd_f:8.3f} "
              f"{vn_g:8.3f} {ve_g:8.3f} {vd_g:8.3f} {innov_max:10.3f} {updates:7d} {rejects:6d} {reason:>20}")

print()
print("=== 发散前后姿态/零偏变化（aim.csv 10Hz输出，每秒采样） ===")
print(f"{'fine_elapsed':>12} {'elapsed':>8} {'Roll':>8} {'Pitch':>8} {'Heading':>8} "
      f"{'gb_x':>8} {'gb_y':>8} {'gb_z':>8} {'ab_x':>8} {'ab_y':>8} {'ab_z':>8} {'PosN':>8} {'PosE':>8}")
print("-" * 120)
last_sec = -1
for r in aim:
    try:
        elapsed = float(r["elapsed_s"])
        stage = r["stage"]
        if stage != "精对准":
            continue
        fine_el = elapsed - FINE_START
        if fine_el < 280 or fine_el > 360:
            continue
        sec = int(elapsed)
        if sec == last_sec:
            continue
        last_sec = sec
        roll = float(r["roll_deg"]); pitch = float(r["pitch_deg"]); heading = float(r["heading_deg"])
        gbx = float(r["gyro_bias_x_deg_s"]); gby = float(r["gyro_bias_y_deg_s"]); gbz = float(r["gyro_bias_z_deg_s"])
        abx = float(r["accel_bias_x_m_s2"]); aby = float(r["accel_bias_y_m_s2"]); abz = float(r["accel_bias_z_m_s2"])
        posn = float(r["pos_n_m"]); pose = float(r["pos_e_m"])
        print(f"{fine_el:12.1f} {elapsed:8.1f} {roll:8.3f} {pitch:8.3f} {heading:8.3f} "
              f"{gbx:8.3f} {gby:8.3f} {gbz:8.3f} {abx:8.4f} {aby:8.4f} {abz:8.4f} {posn:8.2f} {pose:8.2f}")
    except (ValueError, KeyError):
        continue

print()
print("=== 协方差标准差演化（每秒采样） ===")
print(f"{'fine_elapsed':>12} {'std_roll':>8} {'std_pitch':>8} {'std_yaw':>8} "
      f"{'std_vn':>8} {'std_ve':>8} {'std_vd':>8} {'std_gbx':>8} {'std_gby':>8} {'std_gbz':>8}")
print("-" * 100)
last_sec = -1
for r in aim:
    try:
        elapsed = float(r["elapsed_s"])
        stage = r["stage"]
        if stage != "精对准":
            continue
        fine_el = elapsed - FINE_START
        if fine_el < 280 or fine_el > 360:
            continue
        sec = int(elapsed)
        if sec == last_sec:
            continue
        last_sec = sec
        sr = float(r["std_att_roll_deg"]); sp = float(r["std_att_pitch_deg"]); sy = float(r["std_att_heading_deg"])
        svn = float(r["std_vel_n_m_s"]); sve = float(r["std_vel_e_m_s"]); svd = float(r["std_vel_d_m_s"])
        sgbx = float(r["std_gyro_bias_x_deg_s"]); sgby = float(r["std_gyro_bias_y_deg_s"]); sgbz = float(r["std_gyro_bias_z_deg_s"])
        print(f"{fine_el:12.1f} {sr:8.4f} {sp:8.4f} {sy:8.4f} {svn:8.4f} {sve:8.4f} {svd:8.4f} {sgbx:8.5f} {sgby:8.5f} {sgbz:8.5f}")
    except (ValueError, KeyError):
        continue
