# MPU6050/F9P Navigation Qt Monitor

[Project home](../README_EN.md) | [中文](README.md) | English

This monitor displays IMU, navigation status, and sky-view data. It saves separate GPS-timestamped IMU, navigation, and raw-observation CSV files. Signal and frequency details are kept out of the live UI but remain in the RAWX file for later tightly coupled processing.

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
6. Select any combination of IMU, GNSS, and RAWX logging, or click Select All. After choosing a parent directory, the monitor creates a session folder such as `20260908180500`; only selected files are created. Selecting none keeps live display without logging.
7. Click Disconnect before unplugging USB-TTL.

Pause plots stops UI refresh only; reception and enabled recording continue. Clear plots clears the display buffer without deleting saved CSV files.

Each connection starts a fresh local track. Independent point rendering avoids horizontal path artifacts seen with continuous curves on some Windows/PyQtGraph combinations. The map applies no position-jump filter and displays every valid navigation point saved in the session's `gnss.csv`.

The local east/north axes use the same metric scale. After panning or zooming, click Best View to fit all collected positions while preserving that scale.

## Input and output

The input consists of typed records:

    IMU,sample,gps_week,gps_tow_us,time_valid,timer_us,...
    GNSS,gps_week,gps_tow_ms,time_valid,rx_timer_us,fix,num_sv,flags,flags2,carr_soln,position,accuracy,velocity,speed_accuracy,pdop
    SAT,gps_week,gps_tow_ms,time_valid,gnss_id,sv_id,cno,elevation,azimuth,used
    RAWX,gps_week,rcv_tow_f64hex,leap_s,rec_stat,num_meas,total_meas,rx_timer_us
    RAWX_MEAS,gnss_id,sv_id,sig_id,freq_id,pr_f64hex,cp_f64hex,do_f32hex,lock_ms,cno,pr_std,cp_std,do_std,trk_stat
    RAWX_END,num_meas

Three short-name files are available inside each session folder; only those selected before connecting are created:

- `imu.csv`: GPS time, local capture time, raw IMU and physical units.
- `gnss.csv`: GPS time and receive time, WGS-84 position/velocity and accuracy, PDOP, fix/RTK quality, and satellite count.
- `rawx.csv`: per-signal pseudorange, carrier phase, Doppler, frequency, C/N0, lock time, and quality flags.

If acquisition starts twice within the same second, the next folder is suffixed with `_01` so existing data is never overwritten.

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
- A connection can begin in the middle of a RAWX epoch. Measurements before the first epoch header are discarded without increasing the invalid-line count; malformed fields and headerless measurements after synchronization are still counted.
- Batch file closes immediately: run the Python command in PowerShell to see the error.
- Slow EXE startup: use the current onedir spec and launch from the complete output directory.

For command-line capture, decoding, and Allan analysis, see the [tools guide](../tools/README_EN.md).
