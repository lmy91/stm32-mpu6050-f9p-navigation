@echo off
cd /d "%~dp0"
if exist "D:\anaconda\envs\allan-toolkit\python.exe" (
    "D:\anaconda\envs\allan-toolkit\python.exe" imu_serial_qt.py
) else if exist "D:\Anaconda3\python.exe" (
    "D:\Anaconda3\python.exe" imu_serial_qt.py
) else (
    python imu_serial_qt.py
)
if errorlevel 1 pause
