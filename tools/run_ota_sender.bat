@echo off
REM ===========================================================================
REM  run_ota_sender.bat - OTA firmware sender launcher (double-click to run)
REM
REM  Why this exists:
REM    In CMD, the bare command "python" may resolve to the Microsoft Store
REM    placeholder (AppData\Local\Microsoft\WindowsApps\python.exe), which
REM    prints NOTHING and exits immediately.
REM    This launcher calls the real Python by its full path instead.
REM
REM  Usage:
REM    double-click                        -> sends Doc\app_v1.bin
REM    drag a .bin onto this .bat          -> sends that file
REM    run in CMD: run_ota_sender.bat X.bin
REM ===========================================================================

setlocal
chcp 65001 >nul
cd /d "%~dp0.."

set "PY=C:\Users\Administrator\.workbuddy\binaries\python\versions\3.13.12\python.exe"

if not exist "%PY%" (
    echo [ERROR] Python not found at:
    echo         %PY%
    echo.
    pause
    exit /b 1
)

if "%~1"=="" (set "BIN=Doc\app_v1.bin") else (set "BIN=%~1")

echo ============================================================
echo  OTA sender
echo  firmware : %BIN%
echo  port     : 9000
echo  python   : %PY%
echo ============================================================
echo.
echo  Waiting for the device to connect...
echo  Now publish  {"cmd":"ota_recv"}  to topic  rs485gw/cmd
echo.

"%PY%" -u "tools\ota_sender.py" "%BIN%"

echo.
echo ============================================================
echo  sender exited. press any key to close this window.
echo ============================================================
pause >nul
endlocal
