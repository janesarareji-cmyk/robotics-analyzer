@echo off
rem Double-click wrapper for install.ps1. Arguments are passed on (e.g. -DryRun, -Yes).
chcp 65001 >nul 2>&1
cd /d "%~dp0"
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0install.ps1" %*
set RC=%ERRORLEVEL%
echo.
echo (exit code %RC%) Press any key to close this window.
pause >nul
exit /b %RC%
