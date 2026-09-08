# MPU6050/F9P Navigation Qt Monitor

[Project home](../README_EN.md) | [中文](README.md) | English

This directory contains the GNSS/IMU desktop monitor. It displays IMU channels, WGS-84 position/track, NED and ground speed, fix state, satellite count, PDOP, and a satellite sky plot. GPS-timestamped IMU and GNSS navigation data are saved to separate CSV files.

## Install dependencies

Run from the repository root:

    D:\anaconda\envs\allan-toolkit\python.exe -m pip install -r host\requirements.txt

Any Python 3.10+ interpreter may be used. PyQtWebEngine provides the optional online AMap view; the local WGS-84 metric track works without an API key.

## Start

Run from the repository root:

    D:\anaconda\envs\allan-toolkit\python.exe host\imu_serial_qt.py

Alternatively, double-click host/run_imu_serial_qt.bat.

## Operation

1. Verify STM32 PA9→USB-TTL RX and connect the grounds.
2. Plug USB-TTL into the computer and close any serial terminal using the port.
3. Click Refresh and select the corresponding COM port.
4. Select 460800 baud and click Connect.
5. Use the Navigation tab for position, speed, DOP, satellites and sky view; use the IMU tab for sensor curves.
6. Enable separate IMU/GNSS CSV logging. Choose a directory when connecting and two timestamped files are created automatically.
7. Click Disconnect before unplugging USB-TTL.

Pause plots stops UI refresh only; reception and enabled recording continue. Clear plots clears the display buffer without deleting saved CSV files.

## Input and output

The input consists of typed records:

    IMU,sample,gps_week,gps_tow_us,time_valid,timer_us,...
    GNSS,gps_week,gps_tow_ms,time_valid,rx_timer_us,fix,num_sv,flags,flags2,carr_soln,position,accuracy,velocity,speed_accuracy,pdop
    SAT,gps_week,gps_tow_ms,time_valid,gnss_id,sv_id,cno,elevation,azimuth,used

Logging creates two files:

- `imu_gnss_time_*.csv`: GPS time, local capture time, raw IMU and physical units.
- `gnss_nav_*.csv`: GPS time and receive time, WGS-84 position/velocity and accuracy, PDOP, fix/RTK quality, and satellite count.

The monitor accepts complete protocol-v3 records only.

## Package a Windows EXE

The project uses PyInstaller onedir mode. It loads files directly from the output directory instead of extracting a large onefile archive at every launch.

Create an isolated packaging environment from the repository root:

    D:\anaconda\envs\allan-toolkit\python.exe -m venv .venv-package
    .\.venv-package\Scripts\python.exe -m pip install --upgrade pip
    .\.venv-package\Scripts\python.exe -m pip install -r host\requirements.txt pyinstaller

Build:

    .\.venv-package\Scripts\python.exe -m PyInstaller --noconfirm --clean host\MPU6050_F9P_Navigation.spec

Output:

    dist\MPU6050_F9P_Navigation\MPU6050_F9P_Navigation.exe

Zip and distribute the complete MPU6050_F9P_Navigation directory, not the EXE alone. build/, dist/, and .venv-package/ are reproducible and excluded from Git.

## Troubleshooting

- Connected but no data: verify the COM port, 460800 baud, PA9→RX, and common ground.
- Port cannot be opened: close other serial programs, reconnect USB-TTL, and refresh the list.
- Garbled text or increasing invalid lines: confirm the firmware format and baud rate.
- Batch file closes immediately: run the Python command in PowerShell to see the error.
- Slow EXE startup: use the current onedir spec and launch from the complete output directory.

For command-line capture, decoding, and Allan analysis, see the [tools guide](../tools/README_EN.md).
