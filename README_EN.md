# STM32 MPU6050/F9P Integrated Navigation Testbed

[中文](README.md) | [English](README_EN.md)

This project is a low-cost GNSS/INS testbed. An STM32F103 timestamps MPU6050 data from ZED-F9P 1PPS and outputs 1 Hz navigation, sky-view, and RXM-RAWX observations. Qt records separate IMU, navigation, and raw-observation files.

The current release implements the synchronized acquisition and visualization foundation. It does not yet publish a loosely coupled EKF or tightly coupled pseudorange/Doppler solution. Validate timing, sensor noise, installation angles, and lever arms first, then add the fusion layer.

## Features

- 100 Hz MPU6050 acceleration, angular rate, and temperature
- F9P 10 Hz internal navigation with 1 Hz NAV-PVT, NAV-SAT, RXM-RAWX, and TIM-TP output
- Hardware capture of PPS on PA0/TIM2_CH1 and IMU DATA_RDY on PA1/TIM2_CH2
- GPS week and microsecond TOW on every IMU sample
- WGS-84 position, altitude, NED/ground speed, fix, satellite count, and PDOP
- Qt IMU/speed plots, local/AMap track, and multi-constellation sky plot
- Exact pseudorange, carrier phase, Doppler, C/N0, quality flags, and signal/frequency IDs
- Independently selectable IMU/navigation/RAWX CSV logging in Qt, plus separate command-line recording
- Allan analysis directly reads the canonical 21-column IMU v3 files produced during capture
- Direct NTRIP v2 reception in Qt, credit-controlled RTCM forwarding through STM32, and receiver-reported RTK status

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
| USB-TTL TX (3.3 V TTL) | PA10/USART1_RX |

All devices must share ground. The PC link uses PA9/PA10 at 460800 bit/s; the F9P UART remains at 115200 bit/s. Select only C099 J4 `ARD`, not `UART1`/`UART3` simultaneously. PA10 is required for RTCM injection.

## WUH2 RTK

Qt includes F9P MSM compatibility handling: if unsupported NavIC terminates an MSM group, it waits for the complete group, omits NavIC and moves the end-of-group flag to the last retained MSM, recalculating CRC. All other observation bits remain unchanged; normally terminated groups are untouched. This does not change the CSV schema or require reflashing STM32/F9P.

Connect COM7 and wait for the RTCM bridge to become ready. Open the base settings, enter `ntrip.gnsswhu.cn:2101`, mountpoint `WUH200CHN0` and credentials, or import a private BNC file. Connect the base. BNC is not used in this path. Qt bypasses system HTTP proxies using direct TCP sockets; VPN TUN/global routing still requires a direct-routing exception.

Qt de-chunks HTTP, validates RTCM CRC24Q and limits unacknowledged UART bytes to 1024. STM32 forwards PA10 to PA2 using interrupts. Queues are bounded; a frame waiting more than two seconds warns and keeps draining, while missing STM32 status/ACK progress or UART write failure stops correction injection without interrupting acquisition. UBX-RXM-RTCM counters distinguish network delivery from receiver reception/use. The displayed arrival interval is not measurement correction age. Existing v3 records and the three CSV files are unchanged: `carr_soln=1/2` means RTK float/fixed; `fix=3` alone does not identify RTK.

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

Choose the PA9 USB-TTL port and 460800 baud. Qt creates one `YYYYMMDDHHMMSS` session folder per acquisition and writes the selected `imu.csv`, `gnss.csv`, and `rawx.csv` files inside it.

Command-line acquisition and analysis:

```powershell
D:\anaconda\envs\allan-toolkit\python.exe tools\capture_serial.py COM7 --hours 0
D:\anaconda\envs\allan-toolkit\python.exe tools\allan_noise_identification.py data\decoded\20260908180500\imu.csv --rate 100 --skip-minutes 30
```

See [firmware](firmware/README_EN.md), [desktop application](host/README_EN.md), [tools](tools/README_EN.md), the [fusion roadmap](fusion/README_EN.md), and [known issues and hardening](docs/KNOWN_ISSUES.md) for details.

## Protocol v3

```text
IMU,sample,gps_week,gps_tow_us,time_valid,timer_us,ax_raw,ay_raw,az_raw,temp_raw,gx_raw,gy_raw,gz_raw
GNSS,gps_week,gps_tow_ms,time_valid,rx_timer_us,fix,num_sv,flags,flags2,carr_soln,lat_e7,lon_e7,hmsl_mm,h_acc_mm,v_acc_mm,vel_n_mms,vel_e_mms,vel_d_mms,g_speed_mms,s_acc_mms,pdop_x100
SAT,gps_week,gps_tow_ms,time_valid,gnss_id,sv_id,cno_dbhz,elev_deg,azim_deg,used
SAT_END,gps_week,gps_tow_ms,time_valid,num_svs
RAWX,gps_week,rcv_tow_f64hex,leap_s,rec_stat,num_meas,total_meas,rx_timer_us
RAWX_MEAS,gnss_id,sv_id,sig_id,freq_id,pr_f64hex,cp_f64hex,do_f32hex,lock_ms,cno,pr_std,cp_std,do_std,trk_stat
RAWX_END,num_meas
```

RAWX floating-point fields are transported losslessly as IEEE-754 bit-pattern hex. The desktop capture software uses `gnss_id/sig_id/freq_id` to record the received constellation, signal, and frequency. One record is emitted per IMU epoch to keep acquisition responsive.

## Roadmap

1. Long static runs, Allan identification, installation-angle and lever-arm calibration.
2. INS mechanization, stationary detection, and zero-velocity updates.
3. Loosely coupled F9P position/velocity plus MPU6050 error-state EKF.
4. RTCM/NTRIP and RTK status, followed by tightly coupled raw-measurement fusion.

## License and origin

MIT License. This project evolved from [lmy91/stm32-mpu6050-allan-toolkit](https://github.com/lmy91/stm32-mpu6050-allan-toolkit); the original copyright and license are retained.
