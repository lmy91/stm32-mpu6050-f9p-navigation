@echo off
rem Called from launchers. Set PYTHON_EXE only, never change authentication.
if defined STATION_DOWNLOADER_PYTHON (
    set "PYTHON_EXE=%STATION_DOWNLOADER_PYTHON%"
) else if exist "%~dp0.venv\Scripts\python.exe" (
    set "PYTHON_EXE=%~dp0.venv\Scripts\python.exe"
) else if exist "%~dp0..\..\.venv\Scripts\python.exe" (
    set "PYTHON_EXE=%~dp0..\..\.venv\Scripts\python.exe"
) else if exist "D:\Anaconda3\python.exe" (
    set "PYTHON_EXE=D:\Anaconda3\python.exe"
) else (
    set "PYTHON_EXE=python"
)
exit /b 0
