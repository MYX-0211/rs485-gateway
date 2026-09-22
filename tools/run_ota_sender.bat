@echo off
REM ===========================================================================
REM  run_ota_sender.bat - OTA firmware sender launcher (double-click to run)
REM
REM  Why this exists:
REM    In CMD, the bare command "python" may resolve to the Microsoft Store
REM    placeholder (AppData\Local\Microsoft\WindowsApps\python.exe), which
REM    prints NOTHING and exits immediately.
REM    This launcher probes for a real interpreter (py -3, then python on
REM    PATH) and skips the Store stub, so no machine-specific path is needed.
REM
REM  Usage:
REM    double-click                        -> sends Doc\app_v1.bin
REM    drag a .bin onto this .bat          -> sends that file
REM    run in CMD: run_ota_sender.bat X.bin
REM ===========================================================================

setlocal EnableDelayedExpansion
chcp 65001 >nul
cd /d "%~dp0.."

REM ---- Locate a real Python interpreter (no hard-coded machine path) ----
REM   1) "py -3" launcher, shipped by python.org installers
REM   2) "python" on PATH, rejecting the Microsoft Store stub whose path
REM      lives under ...\Microsoft\WindowsApps\ (it is a 0-byte reparse
REM      point that opens the Store and exits without printing anything)
REM   3) per-user python.org install under %LOCALAPPDATA%\Programs\Python
REM   4) a Python bundled with a local toolchain, when one happens to exist
set "PY="
for /f "delims=" %%I in ('py -3 -c "import sys;print(sys.executable)" 2^>nul') do set "PY=%%I"
if not defined PY for /f "delims=" %%I in ('python -c "import sys;print(sys.executable)" 2^>nul') do set "PY=%%I"
if defined PY if not "!PY:WindowsApps=!"=="!PY!" set "PY="
if not defined PY for /d %%D in ("%LOCALAPPDATA%\Programs\Python\Python3*") do if exist "%%~fD\python.exe" set "PY=%%~fD\python.exe"
if not defined PY for /d %%D in ("%USERPROFILE%\.workbuddy\binaries\python\versions\3*") do if exist "%%~fD\python.exe" set "PY=%%~fD\python.exe"

if not defined PY (
    echo [ERROR] No usable Python interpreter found.
    echo         Install Python 3 from https://www.python.org/downloads/
    echo         and tick "Add python.exe to PATH", then run this again.
    echo.
    pause
    exit /b 1
)

if not exist "!PY!" (
    echo [ERROR] Python not found at:
    echo         !PY!
    echo.
    pause
    exit /b 1
)

if "%~1"=="" (set "BIN=Doc\app_v1.bin") else (set "BIN=%~1")

echo ============================================================
echo  OTA sender
echo  firmware : %BIN%
echo  port     : 9000
echo  python   : !PY!
echo ============================================================
echo.
echo  Waiting for the device to connect...
echo  Now publish  {"cmd":"ota_recv"}  to topic  rs485gw/cmd
echo.

"!PY!" -u "tools\ota_sender.py" "%BIN%"

echo.
echo ============================================================
echo  sender exited. press any key to close this window.
echo ============================================================
pause >nul
endlocal
