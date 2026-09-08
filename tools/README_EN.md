# MPU6050/F9P Capture, Decode, and Allan Tools

[Project home](../README_EN.md) | [中文](README.md) | English

These tools match STM32 serial protocol v2. By default they receive the PA9/USART1 USB-TTL stream at 460800 bit/s and save GPS-timestamped IMU data separately from the 1 Hz GNSS navigation solution. Generated experiment data lives under `data/` and is excluded from Git.

## Tools

| File | Purpose | Default output |
| --- | --- | --- |
| `capture_serial.py` | Headless capture of the current complete serial stream | Separate IMU and GNSS CSV files in `data/decoded/` |
| `decode_imu_data.py` | Decode current or legacy IMU files and plot seven channels | `data/decoded/` |
| `allan_noise_identification.py` | Read the canonical IMU CSV and identify Allan noise terms | `data/allan_results/` |

## Install

Run from the repository root:

    D:\anaconda\envs\allan-toolkit\python.exe -m pip install -r tools\requirements.txt

Any Python 3.10+ interpreter may be used. The Qt monitor, command-line capture, and serial terminals cannot own the same COM port simultaneously.

## 1. Command-line capture

The tested USB-TTL port on the current computer is COM7:

    D:\anaconda\envs\allan-toolkit\python.exe tools\capture_serial.py COM7 --hours 12

The default baud rate is 460800. `--hours 0` runs until Ctrl+C and closes the files safely. Default outputs are:

    data\decoded\imu_gnss_time_YYYYMMDD_HHMMSS.csv
    data\decoded\gnss_nav_YYYYMMDD_HHMMSS.csv

Optionally retain the complete STM32 stream:

    D:\anaconda\envs\allan-toolkit\python.exe tools\capture_serial.py COM7 --hours 1 --raw-output data\raw\serial_1h.txt

Custom output paths:

    D:\anaconda\envs\allan-toolkit\python.exe tools\capture_serial.py COM7 --imu-output data\decoded\imu.csv --gnss-output data\decoded\gnss.csv

`--output` remains as a compatibility alias for `--imu-output`. The capture tool reports lost IMU frames, invalid lines, and satellite records. `SAT`/`SAT_END` records are retained only in the optional raw stream rather than duplicated into the GNSS navigation table.

## 2. Decode an IMU file

The decoder accepts:

- current typed `IMU,...` raw serial logs;
- current 21-column canonical IMU CSV files;
- legacy 10-column MPU6050 CSV files.

Run:

    D:\anaconda\envs\allan-toolkit\python.exe tools\decode_imu_data.py data\raw\serial_1h.txt

Custom outputs:

    D:\anaconda\envs\allan-toolkit\python.exe tools\decode_imu_data.py data\raw\serial_1h.txt --output-csv data\decoded\imu.csv --plot data\decoded\imu.png --rate 100

Output always uses the canonical 21-column IMU schema. The decoder streams long files and plots block means to avoid exhausting memory.

## 3. Allan noise identification

Canonical IMU CSV files produced by the capture tool or Qt monitor can be used directly:

    D:\anaconda\envs\allan-toolkit\python.exe tools\allan_noise_identification.py data\decoded\imu.csv --rate 100 --skip-minutes 30 --points 90

`--rate` is the nominal sampling rate, currently 100 Hz; `--skip-minutes` discards warm-up; `--points` must be at least 30. Results include Allan and stability plots, parameter CSV files, and a Chinese interpretation report.

## Current serial protocol

    IMU,sample,gps_week,gps_tow_us,time_valid,timer_us,ax_raw,ay_raw,az_raw,temp_raw,gx_raw,gy_raw,gz_raw
    GNSS,gps_week,gps_tow_ms,time_valid,fix,num_sv,lat_e7,lon_e7,hmsl_mm,vel_n_mms,vel_e_mms,vel_d_mms,g_speed_mms,pdop_x100
    SAT,gps_week,gps_tow_ms,time_valid,gnss_id,sv_id,cno_dbhz,elev_deg,azim_deg,used
    SAT_END,gps_week,gps_tow_ms,time_valid,num_svs

Canonical IMU CSV:

    sample,gps_week,gps_tow_us,time_valid,timer_us,time_s,dt_s,ax_raw,ay_raw,az_raw,temp_raw,gx_raw,gy_raw,gz_raw,ax_m_s2,ay_m_s2,az_m_s2,temp_deg_c,gx_deg_h,gy_deg_h,gz_deg_h

Canonical GNSS CSV:

    gps_week,gps_tow_ms,time_valid,fix,num_sv,lat_deg,lon_deg,hmsl_m,vel_n_m_s,vel_e_m_s,vel_d_m_s,ground_speed_m_s,pdop

`time_valid=1` means GPS time is valid. Coordinates are WGS-84. Angular rates are stored in deg/h and converted to rad/s internally by the Allan tool.

## Recommendations

- After power-up, wait for an F9P fix and verify `time_valid=1`.
- For a static Allan test, rigidly mount and warm up the IMU for about 30 minutes; avoid temperature changes and cable motion.
- Run a 5–10 minute pilot first and confirm `lost=0`, `invalid=0`, and one GNSS row per second.
- Analyze a copied snapshot of a file that is still being recorded; never use two writers on one file.

## Troubleshooting

- Port busy: close the Qt monitor, u-center, and every other serial program.
- Garbled or invalid lines: select the STM32 USB-TTL port at 460800, not the C099 USB port.
- No GNSS rows: check crossed PA2/PA3 wiring, common ground, and the C099 J4 ARD route.
- Invalid GPS time: move the antenna to an open-sky location and wait for valid F9P time.
- Too few Allan samples: reduce `--skip-minutes` or record a longer static data set.
