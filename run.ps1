<#
  Start Robotics Analyzer. Extra arguments go to the app, e.g.  .\run.ps1 --smoke
  If PowerShell says running scripts is disabled, use run.bat or:
    powershell -ExecutionPolicy Bypass -File run.ps1
#>
$Root = $PSScriptRoot
Set-Location $Root
$VenvPy = Join-Path $Root ".venv\Scripts\python.exe"

if (-not (Test-Path $VenvPy)) {
    Write-Host "The virtual environment (.venv) is missing. Run setup.ps1 first:" -ForegroundColor Red
    Write-Host "  powershell -ExecutionPolicy Bypass -File setup.ps1"
    exit 1
}
if (-not (Get-ChildItem -Path $Root -Filter "core_bridge.*.pyd" -ErrorAction SilentlyContinue)) {
    Write-Host "The compiled module core_bridge is missing. Run setup.ps1 first:" -ForegroundColor Red
    Write-Host "  powershell -ExecutionPolicy Bypass -File setup.ps1"
    exit 1
}

& $VenvPy -m app.main @args
exit $LASTEXITCODE
