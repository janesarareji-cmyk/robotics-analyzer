# Per-user install of Robotics Analyzer from the unzipped folder. No admin rights, no PATH change.
# Usage: powershell -ExecutionPolicy Bypass -File install.ps1 [-Target "$env:LOCALAPPDATA\RoboticsAnalyzer"]
param([string]$Target = "$env:LOCALAPPDATA\RoboticsAnalyzer")
$src = $PSScriptRoot
New-Item -ItemType Directory -Force $Target | Out-Null
Copy-Item "$src\*" $Target -Recurse -Force
$lnkDir = [Environment]::GetFolderPath("Programs")
$ws = New-Object -ComObject WScript.Shell
$lnk = $ws.CreateShortcut("$lnkDir\Robotics Analyzer.lnk")
$lnk.TargetPath = "$Target\RoboticsAnalyzer.exe"
$lnk.WorkingDirectory = $Target
$lnk.Save()
"Installed to $Target. Start menu entry: Robotics Analyzer. Uninstall: delete the folder and the shortcut."
