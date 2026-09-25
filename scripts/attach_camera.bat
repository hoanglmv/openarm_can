@echo off
title OpenArm RealSense Camera WSL2 Attacher
color 0b
echo ======================================================================
echo           OpenArm RealSense Camera USB Attacher for WSL2
echo ======================================================================
echo.

:: Check for Administrator privileges
net session >nul 2>&1
if %errorlevel% neq 0 (
    echo [INFO] Requesting Administrator privileges to bind RealSense USB...
    powershell -NoProfile -ExecutionPolicy Bypass -Command "Start-Process cmd -ArgumentList '/c %~s0' -Verb RunAs"
    exit /b
)

echo [1/3] Scanning for connected Intel RealSense camera...
set BUSID=
for /f "tokens=1" %%i in ('usbipd list ^| findstr /r /c:"^[0-9][0-9]*-.*8086:0b3a" /c:"^[0-9][0-9]*-.*RealSense"') do set BUSID=%%i

if "%BUSID%"=="" (
    echo [ERROR] RealSense camera is NOT connected to any USB port right now!
    echo Please plug your Intel RealSense D435i USB cable into a USB 3.0 port and try again.
    echo.
    usbipd list
    pause
    exit /b 1
)

echo [OK] Found RealSense on Bus ID: %BUSID%
echo.
echo [2/3] Sharing/Binding RealSense USB device...
usbipd bind --busid %BUSID% 2>nul

echo.
echo [3/3] Attaching RealSense to WSL2 with auto-attach...
usbipd attach --wsl --busid %BUSID% --auto-attach

echo.
echo ======================================================================
echo [SUCCESS] RealSense Camera attached to WSL2!
echo You can now run OpenArm Dashboard: python sim/server.py
echo ======================================================================
echo.
pause
