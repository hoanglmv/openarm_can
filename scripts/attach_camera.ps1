# OpenArm RealSense Camera USB Attacher for WSL2
# Run in PowerShell (Administrator)

Write-Host "======================================================================" -ForegroundColor Cyan
Write-Host "          OpenArm RealSense Camera USB Attacher for WSL2" -ForegroundColor Cyan
Write-Host "======================================================================" -ForegroundColor Cyan

# Check elevation
$isAdmin = ([Security.Principal.WindowsPrincipal][Security.Principal.WindowsIdentity]::GetCurrent()).IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)
if (-not $isAdmin) {
    Write-Host "[INFO] Elevating to Administrator..." -ForegroundColor Yellow
    Start-Process powershell -Verb RunAs -ArgumentList "-NoProfile -ExecutionPolicy Bypass -File `"$PSCommandPath`""
    Exit
}

# Scan devices
Write-Host "`n[1/3] Scanning for Intel RealSense camera..." -ForegroundColor Yellow
$deviceLine = (usbipd list) | Where-Object { $_ -match "8086:0b3a|RealSense" } | Select-Object -First 1

if (-not $deviceLine) {
    Write-Host "[ERROR] RealSense camera not found on Windows host!" -ForegroundColor Red
    Write-Host "Please plug in your Intel RealSense D435i USB cable and try again." -ForegroundColor Yellow
    usbipd list
    Read-Host "Press Enter to exit..."
    Exit
}

$busid = ($deviceLine -split '\s+')[0]
Write-Host "[OK] Found RealSense on Bus ID: $busid" -ForegroundColor Green

Write-Host "`n[2/3] Sharing/Binding RealSense USB device..." -ForegroundColor Yellow
usbipd bind --busid $busid 2>$null

Write-Host "`n[3/3] Attaching RealSense to WSL2..." -ForegroundColor Yellow
usbipd attach --wsl --busid $busid --auto-attach

Write-Host "`n======================================================================" -ForegroundColor Green
Write-Host "[SUCCESS] RealSense Camera successfully attached to WSL2!" -ForegroundColor Green
Write-Host "You can now run or refresh OpenArm Dashboard at http://localhost:8888" -ForegroundColor Green
Write-Host "======================================================================" -ForegroundColor Green
Read-Host "Press Enter to exit..."
