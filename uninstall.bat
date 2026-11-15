@echo off
rem Double-click wrapper for uninstall.ps1. Runs a temp copy so the install folder can be deleted.
setlocal
set "HERE=%~dp0"
set "TMPPS=%TEMP%\ra_uninstall_%RANDOM%.ps1"
copy /y "%HERE%uninstall.ps1" "%TMPPS%" >nul
cd /d "%TEMP%"
powershell -NoProfile -ExecutionPolicy Bypass -File "%TMPPS%" -InstallDir "%HERE%." %* & del "%TMPPS%" >nul 2>&1 & echo. & echo Press any key to close this window. & pause >nul
