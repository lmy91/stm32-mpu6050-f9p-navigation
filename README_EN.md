# STM32 MPU6050/F9P Integrated Navigation Testbed

[中文](README.md) | [English](README_EN.md)

This project is a low-cost GNSS/INS testbed. An STM32F103 captures MPU6050 DATA_RDY and ZED-F9P 1PPS in the same hardware-timer domain, assigns a GPS week/time-of-week timestamp to every IMU sample, and outputs 1 Hz position, velocity, PDOP, and sky-view data. The Qt application provides live plots, track display, and separate IMU/GNSS recording.

The current release implements the synchronized acquisition and visualization foundation. It does not yet publish a loosely coupled EKF or tightly coupled pseudorange/Doppler solution. Validate timing, sensor noise, installation angles, and lever arms first, then add the fusion layer.

## Features

- 100 Hz MPU6050 acceleration, angular rate, and temperature
- F9P 10 Hz internal navigation with 1 Hz NAV-PVT, NAV-SAT, and TIM-TP output
- Hardware capture of PPS on PA0/TIM2_CH1 and IMU DATA_RDY on PA1/TIM2_CH2
- GPS week and microsecond TOW on every IMU sample
- WGS-84 position, altitude, NED/ground speed, fix, satellite count, and PDOP
- Qt IMU/speed plots, local/AMap track, and multi-constellation sky plot
- Separate IMU/GNSS CSV logging from both Qt and the command-line capture tool
- Decoder and Allan tools compatible with the canonical 21-column IMU v2 file

## Wiring

| Device | STM32F103C8T6 |
| --- | --- |
| MPU6050 VCC/GND | 3.3V/GND |
| MPU6050 SCL/SDA | PB6/PB7 |
| MPU6050 INT | PA1/TIM2_CH2 |
| C099 TP | PA0/TIM2_CH1 |
| C099 TX_ZED | PA3/USART2_RX |
| C099 RX_ZED | PA2/USART2_TX |
| C099 GND | GND |
| USB-TTL RX/GND | PA9/GND |

All devices must share ground. The PC logger uses PA9 at 460800 bit/s; the F9P UART remains at 115200 bit/s.

## Quick start

```powershell
D:\anaconda\envs\allan-toolkit\python.exe -m pip install -r host\requirements.txt
D:\anaconda\envs\allan-toolkit\python.exe -m pip install -r tools\requirements.txt
cmake --preset Release -S firmware
cmake --build firmware\build\Release --clean-first
```

Flash `firmware/build/Release/mpu6050_f9p_navigation.elf`, then start:

```powershell
D:\anaconda\envs\allan-toolkit\python.exe host\imu_serial_qt.py
```

Choose the PA9 USB-TTL port and 460800 baud. Logging creates `imu_gnss_time_*.csv` and `gnss_nav_*.csv`. AMap requires a Web JS API Key plus `securityJsCode`; secrets are local settings and must not be committed.

Command-line acquisition and analysis:

```powershell
D:\anaconda\envs\allan-toolkit\python.exe tools\capture_serial.py COM7 --hours 0
D:\anaconda\envs\allan-toolkit\python.exe tools\decode_imu_data.py data\raw\record.log
D:\anaconda\envs\allan-toolkit\python.exe tools\allan_noise_identification.py data\decoded\imu_gnss_time_xxx.csv --rate 100 --skip-minutes 30
```

See [firmware](firmware/README_EN.md), [desktop application](host/README_EN.md), [tools](tools/README_EN.md), and the [fusion roadmap](fusion/README_EN.md) for details.

## Protocol v2

```text
IMU,sample,gps_week,gps_tow_us,time_valid,timer_us,ax_raw,ay_raw,az_raw,temp_raw,gx_raw,gy_raw,gz_raw
GNSS,gps_week,gps_tow_ms,time_valid,fix,num_sv,lat_e7,lon_e7,hmsl_mm,vel_n_mms,vel_e_mms,vel_d_mms,g_speed_mms,pdop_x100
SAT,gps_week,gps_tow_ms,time_valid,gnss_id,sv_id,cno_dbhz,elev_deg,azim_deg,used
SAT_END,gps_week,gps_tow_ms,time_valid,num_svs
```

GPS timestamps are usable only when `time_valid=1`. Saved positions remain WGS-84; GCJ-02 conversion is display-only for AMap.

## Roadmap

1. Long static runs, Allan identification, installation-angle and lever-arm calibration.
2. INS mechanization, stationary detection, and zero-velocity updates.
3. Loosely coupled F9P position/velocity plus MPU6050 error-state EKF.
4. RTCM/NTRIP and RTK status, followed by tightly coupled raw-measurement fusion.

## License and origin

MIT License. This project evolved from [lmy91/stm32-mpu6050-allan-toolkit](https://github.com/lmy91/stm32-mpu6050-allan-toolkit); the original copyright and license are retained.
