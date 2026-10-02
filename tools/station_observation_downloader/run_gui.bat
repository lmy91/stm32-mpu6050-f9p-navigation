@echo off
setlocal
call "%~dp0choose_python.bat"
"%PYTHON_EXE%" -X utf8 "%~dp0station_downloader_qt.py" %*
if errorlevel 1 (
    echo Launch failed. Install dependencies with:
    echo "%PYTHON_EXE%" -m pip install -r "%~dp0requirements.txt"
    pause
)
endlocal
