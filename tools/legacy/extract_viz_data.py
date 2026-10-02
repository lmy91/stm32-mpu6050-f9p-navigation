"""Extract downsampled time series for ECharts visualization."""
import csv
import json
import pathlib

DATA_DIR = pathlib.Path(r"C:\Users\12597\Desktop\lowcost\stm32-mpu6050-f9p-navigation\data\decoded\20260909203315")

def load_csv(path):
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))

def downsample(rows, max_points=300):
    if len(rows) <= max_points:
        return rows
    step = len(rows) // max_points
    return rows[::step]

# AIM data - full resolution is fine (3793 rows, downsample to ~400)
aim_rows = load_csv(DATA_DIR / "aim.csv")
aim_ds = downsample(aim_rows, 400)

aim_time = [float(r["elapsed_s"]) for r in aim_ds]
aim_roll = [float(r["roll_deg"]) for r in aim_ds]
aim_pitch = [float(r["pitch_deg"]) for r in aim_ds]
aim_heading = [float(r["heading_deg"]) for r in aim_ds]
aim_pos_n = [float(r["pos_n_m"]) for r in aim_ds]
aim_pos_e = [float(r["pos_e_m"]) for r in aim_ds]
aim_pos_d = [float(r["pos_d_m"]) for r in aim_ds]
aim_vel_n = [float(r["vel_n_m_s"]) for r in aim_ds]
aim_vel_e = [float(r["vel_e_m_s"]) for r in aim_ds]
aim_vel_d = [float(r["vel_d_m_s"]) for r in aim_ds]
aim_std_roll = [float(r["std_att_roll_deg"]) for r in aim_ds]
aim_std_pitch = [float(r["std_att_pitch_deg"]) for r in aim_ds]
aim_std_heading = [float(r["std_att_heading_deg"]) for r in aim_ds]
aim_std_pos_n = [float(r["std_pos_n_m"]) for r in aim_ds]
aim_std_pos_e = [float(r["std_pos_e_m"]) for r in aim_ds]
aim_std_pos_d = [float(r["std_pos_d_m"]) for r in aim_ds]

# GNSS data - 589 rows, use all
gnss_rows = load_csv(DATA_DIR / "gnss.csv")
gnss_time = [(int(r["gps_tow_ms"]) - int(gnss_rows[0]["gps_tow_ms"])) / 1000.0 for r in gnss_rows]
gnss_hacc = [float(r["h_acc_m"]) for r in gnss_rows]
gnss_vacc = [float(r["v_acc_m"]) for r in gnss_rows]
gnss_pdop = [float(r["pdop"]) for r in gnss_rows]
gnss_sv = [int(r["num_sv"]) for r in gnss_rows]
gnss_ground_speed = [float(r["ground_speed_m_s"]) for r in gnss_rows]

# IMU data - downsample heavily (58873 rows -> 500)
imu_rows = load_csv(DATA_DIR / "imu.csv")
imu_ds = downsample(imu_rows, 500)
imu_time = [float(r["time_s"]) for r in imu_ds]
imu_ax = [float(r["ax_m_s2"]) for r in imu_ds]
imu_ay = [float(r["ay_m_s2"]) for r in imu_ds]
imu_az = [float(r["az_m_s2"]) for r in imu_ds]
imu_gx = [float(r["gx_deg_h"]) / 3600 for r in imu_ds]  # deg/s
imu_gy = [float(r["gy_deg_h"]) / 3600 for r in imu_ds]
imu_gz = [float(r["gz_deg_h"]) / 3600 for r in imu_ds]

data = {
    "aim": {
        "time": aim_time, "roll": aim_roll, "pitch": aim_pitch, "heading": aim_heading,
        "pos_n": aim_pos_n, "pos_e": aim_pos_e, "pos_d": aim_pos_d,
        "vel_n": aim_vel_n, "vel_e": aim_vel_e, "vel_d": aim_vel_d,
        "std_roll": aim_std_roll, "std_pitch": aim_std_pitch, "std_heading": aim_std_heading,
        "std_pos_n": aim_std_pos_n, "std_pos_e": aim_std_pos_e, "std_pos_d": aim_std_pos_d,
    },
    "gnss": {
        "time": gnss_time, "hacc": gnss_hacc, "vacc": gnss_vacc,
        "pdop": gnss_pdop, "sv": gnss_sv, "ground_speed": gnss_ground_speed,
    },
    "imu": {
        "time": imu_time, "ax": imu_ax, "ay": imu_ay, "az": imu_az,
        "gx": imu_gx, "gy": imu_gy, "gz": imu_gz,
    }
}

out = pathlib.Path(r"C:\Users\12597\Desktop\lowcost\stm32-mpu6050-f9p-navigation\tools\viz_data.json")
out.write_text(json.dumps(data), encoding="utf-8")
print(f"Written {out} ({out.stat().st_size} bytes)")
print(f"AIM points: {len(aim_time)}, GNSS points: {len(gnss_time)}, IMU points: {len(imu_time)}")
