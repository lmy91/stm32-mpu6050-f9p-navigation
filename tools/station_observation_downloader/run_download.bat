@echo off
setlocal
call "%~dp0choose_python.bat"
"%PYTHON_EXE%" -u -X utf8 "%~dp0one_click_download.py" %*
if errorlevel 1 echo Download incomplete. See the message above; valid files will be reused on retry.
pause
endlocal
