@echo off
rem Double-click wrapper for run.ps1. Arguments are passed on.
cd /d "%~dp0"
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0run.ps1" %*
set RC=%ERRORLEVEL%
if not "%RC%"=="0" pause
exit /b %RC%
