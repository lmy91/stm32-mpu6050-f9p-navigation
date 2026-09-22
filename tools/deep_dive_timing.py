"""Deep-dive: AIM time_match and GNSS rejection trends over time."""
import csv
import pathlib
from collections import Counter

DATA_DIR = pathlib.Path(r"C:\Users\12597\Desktop\lowcost\stm32-mpu6050-f9p-navigation\data\decoded\20260910142516")

def load_csv(path):
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))

aim = load_csv(DATA_DIR / "aim.csv")

print("=== AIM 行中 time_match_s 和 last_gnss_reason 随时间变化 ===")
print(f"{'elapsed_s':>10} {'stage':>8} {'time_match':>12} {'updates':>8} {'rejects':>8}  reason")
print("-" * 90)

# Print every ~50 rows plus all rows where reason changes
last_reason = None
for i, r in enumerate(aim):
    elapsed = float(r["elapsed_s"])
    stage = r["stage"]
    tm = r["time_match_s"]
    upd = r["gnss_updates"]
    rej = r["gnss_rejections"]
    reason = r["last_gnss_reason"]
    # Print at key intervals or when reason changes or every 100 rows
    if (i % 100 == 0 or reason != last_reason or 
        elapsed > 300 or  # focus on late stage
        (stage == "精对准" and abs(elapsed - 30) < 0.5)):
        print(f"{elapsed:10.1f} {stage:>8} {tm:>12} {upd:>8} {rej:>8}  {reason}")
    last_reason = reason

print("\n=== 精对准阶段按30秒分段统计 ===")
fine_rows = [r for r in aim if r["stage"] == "精对准"]
fine_elapsed0 = float(fine_rows[0]["elapsed_s"])
print(f"精对准起始 elapsed = {fine_elapsed0:.1f}s")

bins = {}
for r in fine_rows:
    e = float(r["elapsed_s"]) - fine_elapsed0
    b = int(e // 30) * 30
    if b not in bins:
        bins[b] = {"reasons": Counter(), "tm_vals": [], "upd_start": None, "upd_end": None,
                    "rej_start": None, "rej_end": None}
    bins[b]["reasons"][r["last_gnss_reason"]] += 1
    if r["time_match_s"] != "nan":
        bins[b]["tm_vals"].append(float(r["time_match_s"]))

# Get update/reject deltas per bin
for b in sorted(bins):
    rows_in_bin = [r for r in fine_rows if int((float(r["elapsed_s"]) - fine_elapsed0) // 30) * 30 == b]
    if rows_in_bin:
        bins[b]["upd_start"] = int(rows_in_bin[0]["gnss_updates"])
        bins[b]["upd_end"] = int(rows_in_bin[-1]["gnss_updates"])
        bins[b]["rej_start"] = int(rows_in_bin[0]["gnss_rejections"])
        bins[b]["rej_end"] = int(rows_in_bin[-1]["gnss_rejections"])

for b in sorted(bins):
    d = bins[b]
    upd_delta = d["upd_end"] - d["upd_start"]
    rej_delta = d["rej_end"] - d["rej_start"]
    tm_mean = sum(d["tm_vals"]) / len(d["tm_vals"]) if d["tm_vals"] else float("nan")
    tm_max = max(d["tm_vals"]) if d["tm_vals"] else float("nan")
    top_reason = d["reasons"].most_common(1)[0]
    print(f"  精对准+{b:>3}~{b+30:<3}s: GNSS更新{upd_delta:>3} 拒绝{rej_delta:>3} "
          f"time_match mean={tm_mean:.4f}s max={tm_max:.4f}s | top原因: {top_reason[0]} ({top_reason[1]}行)")

print("\n=== 检查 IMU gps_tow_us 递增率 ===")
imu = load_csv(DATA_DIR / "imu.csv")
# Check every 5000th row
for i in range(0, len(imu), 5000):
    r = imu[i]
    print(f"  sample={r['sample']:>7}  elapsed={float(r['time_s']):>8.2f}s  "
          f"gps_tow_us={r['gps_tow_us']}  timer_us={r['timer_us']}")

# Compute gps_tow_us drift vs timer_us
print("\n=== gps_tow_us vs timer_us 漂移（每10000帧）===")
for i in range(0, len(imu), 10000):
    r = imu[i]
    tow = int(r["gps_tow_us"])
    timer = int(r["timer_us"])
    if i == 0:
        tow0, timer0 = tow, timer
    drift = (tow - tow0) - (timer - timer0)
    print(f"  sample={r['sample']:>7}  elapsed={float(r['time_s']):>8.2f}s  "
          f"tow增量={tow-tow0:>12}us  timer增量={timer-timer0:>12}us  漂移={drift:>8}us ({drift/1000:.3f}ms)")

print("\n=== GNSS gps_tow_ms vs IMU gps_tow_us 在对应时刻的差 ===")
gnss = load_csv(DATA_DIR / "gnss.csv")
# For each GNSS epoch, find the closest IMU sample and compare GPS times
imu_tows = [(int(r["gps_week"]) * 604800 + int(r["gps_tow_us"]) / 1e6, float(r["time_s"])) for r in imu]
gnss_times = [(int(r["gps_week"]) * 604800 + int(r["gps_tow_ms"]) / 1000.0, i) for i, r in enumerate(gnss)]

# Sample every 60 GNSS epochs (~1 min)
import bisect
imu_gps_only = [t[0] for t in imu_tows]
for idx in range(0, len(gnss), 60):
    gtime, gi = gnss_times[idx]
    # Find closest IMU
    pos = bisect.bisect_left(imu_gps_only, gtime)
    if pos > 0 and pos < len(imu_gps_only):
        best = pos if abs(imu_gps_only[pos] - gtime) < abs(imu_gps_only[pos-1] - gtime) else pos-1
        diff = imu_gps_only[best] - gtime
        print(f"  GNSS[{gi:>3}] gps_time={gtime:.3f}  最近IMU gps_time={imu_gps_only[best]:.3f}  "
              f"差(IMU-GNSS)={diff:+.4f}s  IMU_elapsed={imu_tows[best][1]:.1f}s")
