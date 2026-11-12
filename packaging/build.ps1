# Build the one-folder distribution and the zip. Run from anywhere.
# Needs: python with pyinstaller (pip install pyinstaller). Nothing is installed system-wide.
$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
Set-Location $root
$sw = [Diagnostics.Stopwatch]::StartNew()
python -m PyInstaller --noconfirm --clean --distpath "$root\dist" --workpath "$root\build\pyi" "$root\packaging\analyzer.spec"
if ($LASTEXITCODE -ne 0) { throw "PyInstaller failed" }
Copy-Item "$root\packaging\install.ps1" "$root\dist\RoboticsAnalyzer\install.ps1" -Force
$zip = "$root\dist\RoboticsAnalyzer-win64.zip"
if (Test-Path $zip) { Remove-Item $zip }
Compress-Archive -Path "$root\dist\RoboticsAnalyzer" -DestinationPath $zip
$size = (Get-ChildItem "$root\dist\RoboticsAnalyzer" -Recurse | Measure-Object Length -Sum).Sum / 1MB
"build time {0:N0} s, folder {1:N0} MiB, zip {2:N0} MiB" -f $sw.Elapsed.TotalSeconds, $size, ((Get-Item $zip).Length / 1MB)
# Inno Setup (optional, manual): ISCC.exe packaging\installer.iss
