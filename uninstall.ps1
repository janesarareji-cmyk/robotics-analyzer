<#
.SYNOPSIS
  Remove a Robotics Analyzer install made by install.ps1.

.DESCRIPTION
  Usage:  powershell -ExecutionPolicy Bypass -File uninstall.ps1 [-InstallDir <path>] [-Yes] [-DryRun]
  (or double-click uninstall.bat)

  Safety rules:
  - Only a folder that holds the marker file written by install.ps1 is touched.
  - A folder installed by the wizard (mode "fresh") is deleted as a whole.
  - A clone that you made yourself (mode "clone") is NOT deleted: only .venv, build/, the built .pyd
    files, install.log and the marker are removed.
  - Shortcuts are removed only when they point into the install folder.
  - Nothing else is ever deleted. It asks before it deletes.
#>
param(
    [string]$InstallDir = "",
    [switch]$Yes,
    [switch]$DryRun
)

$ErrorActionPreference = "Continue"
$MarkerName = ".robotics-analyzer-install"
$MarkerHead = "RoboticsAnalyzer-install-marker"

function Say([string]$t, [string]$c = "") {
    $col = $true
    if ($env:NO_COLOR) { $col = $false }
    if ($c -and $col) { Write-Host $t -ForegroundColor $c } else { Write-Host $t }
}

function Read-Answer {
    $r = $null
    try { $r = Read-Host } catch { $r = $null }
    if ($null -eq $r) { return "" }
    return ([string]$r).Trim()
}

function Ask-YesNo([string]$Q, [bool]$Default) {
    $hint = "[y/N]"
    if ($Default) { $hint = "[Y/n]" }
    if ($Yes) { Say "  ? $Q $hint -> yes (auto, -Yes)" "DarkCyan"; return $true }
    Write-Host ("  ? $Q $hint ") -NoNewline
    $r = Read-Answer
    if ($r -eq "") { return $Default }
    if ($r -match '^(q|quit|exit)$') { return $false }
    return ($r -match '^(y|yes)$')
}

function Read-Marker([string]$dir) {
    $h = @{}
    $m = Join-Path $dir $MarkerName
    if (Test-Path -LiteralPath $m -PathType Leaf) {
        $lines = @(Get-Content -LiteralPath $m -ErrorAction SilentlyContinue)
        if ($lines.Count -gt 0 -and $lines[0].Trim() -eq $MarkerHead) {
            foreach ($l in $lines[1..($lines.Count - 1)]) {
                $ix = $l.IndexOf("=")
                if ($ix -gt 0) {
                    $k = $l.Substring(0, $ix).Trim()
                    if ($h.ContainsKey($k)) { $h[$k] = $h[$k] + "|" + $l.Substring($ix + 1) } else { $h[$k] = $l.Substring($ix + 1) }
                }
            }
            $h["_valid"] = "1"
        }
    }
    return $h
}

function Get-FolderMB([string]$dir) {
    try {
        $sum = (Get-ChildItem -LiteralPath $dir -Recurse -Force -File -ErrorAction SilentlyContinue | Measure-Object -Property Length -Sum).Sum
        if ($sum) { return [int]($sum / 1MB) }
    } catch { }
    return 0
}

function Test-Reparse([string]$path) {
    try {
        $a = [System.IO.File]::GetAttributes($path)
        return (($a -band [System.IO.FileAttributes]::ReparsePoint) -ne 0)
    } catch { return $false }
}

# S3: delete junctions / symlinks inside the tree first (the link itself, never its target),
# then delete the tree with .NET. No cmd.exe, so no %VAR% expansion.
function Remove-Links([string]$path) {
    $subs = @()
    try { $subs = @([System.IO.Directory]::GetDirectories($path)) } catch { return }
    foreach ($d in $subs) {
        if (Test-Reparse $d) {
            try { [System.IO.Directory]::Delete($d, $false) } catch { }
        } else {
            Remove-Links $d
        }
    }
}

function Remove-Tree([string]$path) {
    if (-not (Test-Path -LiteralPath $path)) { return }
    if ($DryRun) { Say "   [dry-run] would delete $path" "DarkYellow"; return }
    try {
        if (Test-Reparse $path) {
            [System.IO.Directory]::Delete($path, $false)
        } else {
            Remove-Links $path
            [System.IO.Directory]::Delete($path, $true)
        }
    } catch { }
    if (Test-Path -LiteralPath $path) { Say "   ! could not delete everything in $path (is something still running from there?)" "Yellow" }
    else { Say "   removed $path" "Green" }
}

# S4: the marker must be complete and belong to this folder, else nothing is deleted
function Test-MarkerStrict($mk, [string]$dir) {
    if (-not $mk.ContainsKey("created_by") -or $mk["created_by"].Trim() -ne "install.ps1") { return "no created_by=install.ps1 line" }
    if (-not $mk.ContainsKey("dir")) { return "no dir= line" }
    if ($mk["dir"].Trim().TrimEnd('\') -ine $dir) { return "dir= says '$($mk['dir'])', not this folder" }
    if (-not $mk.ContainsKey("mode") -or @("fresh", "clone") -notcontains $mk["mode"].Trim()) { return "mode= missing or unknown" }
    if (-not $mk.ContainsKey("version") -or $mk["version"].Trim() -notmatch '^[0-9]+$') { return "version= missing or not a number" }
    return ""
}

Write-Host ""
Say "Robotics Analyzer - uninstall" "Cyan"
if ($DryRun) { Say "  *** DRY RUN: nothing will be deleted ***" "Yellow" }

# ---- find the folder
$cands = @()
if ($InstallDir) { $cands += $InstallDir }
else {
    $cands += $PSScriptRoot
    if ($env:LOCALAPPDATA) { $cands += (Join-Path $env:LOCALAPPDATA "RoboticsAnalyzer") }
}
$Dir = $null
$mk = @{}
foreach ($c in $cands) {
    try { $full = [System.IO.Path]::GetFullPath($c.Trim('"').Trim()) } catch { continue }
    if ($full.Length -gt 3) { $full = $full.TrimEnd('\') }
    if (Test-Path -LiteralPath $full -PathType Container) {
        $m = Read-Marker $full
        if ($m.ContainsKey("_valid")) { $Dir = $full; $mk = $m; break }
    }
}
if (-not $Dir) {
    $shown = ($cands -join ", ")
    Say "No Robotics Analyzer install found (looked in: $shown)." "Red"
    Say "  I only remove folders that hold the file $MarkerName written by install.ps1."
    Say "  Nothing was deleted. Use -InstallDir <path> if it is somewhere else."
    exit 1
}
# the marker must belong to this very folder (a copied marker does not count)
$why = Test-MarkerStrict $mk $Dir
if ($why) {
    Say "The marker in $Dir is not valid: $why." "Red"
    Say "  Refusing to delete. Nothing was deleted. Remove the folder by hand if you are sure."
    exit 1
}
if ($Dir -match '[%!^&]') {
    Say "Refusing: the path $Dir has one of the characters % ! ^ &. Nothing was deleted." "Red"
    exit 1
}
if (Test-Reparse $Dir) {
    Say "Refusing: $Dir is a junction or symbolic link. Nothing was deleted." "Red"
    exit 1
}
$root = [System.IO.Path]::GetPathRoot($Dir)
$prot = @($env:USERPROFILE, $env:SystemRoot, $env:ProgramFiles, ${env:ProgramFiles(x86)}, $env:ProgramData, $env:LOCALAPPDATA, $env:APPDATA, $env:SystemDrive)
$isProt = ($Dir -ieq $root.TrimEnd('\'))
foreach ($b in $prot) {
    if (-not $b) { continue }
    $bb = $b.TrimEnd('\')
    # the folder itself, or a folder that contains it (the profile inside it would be deleted too)
    if ($Dir -ieq $bb -or $bb.StartsWith($Dir + "\", [StringComparison]::OrdinalIgnoreCase)) { $isProt = $true }
}
if ($isProt) {
    Say "Refusing: $Dir is a protected location. Nothing was deleted." "Red"
    exit 1
}
$mode = $mk["mode"].Trim()

Say "  Found install: $Dir" "White"
Say ("  Installed by : install.ps1 on " + $mk["date"] + "  (mode: $mode)")
$sizeMB = Get-FolderMB $Dir
Say "  Size         : about $sizeMB MB"

# ---- shortcuts
$links = @()
if ($mk.ContainsKey("shortcut")) { $links = @($mk["shortcut"] -split "\|" | Where-Object { $_ }) }
foreach ($d in @([Environment]::GetFolderPath("Desktop"), [Environment]::GetFolderPath("Programs"))) {
    if ($d) { $l = Join-Path $d "Robotics Analyzer.lnk"; if ((Test-Path -LiteralPath $l) -and ($links -notcontains $l)) { $links += $l } }
}
$ours = @()
foreach ($l in $links) {
    if (-not (Test-Path -LiteralPath $l -PathType Leaf)) { continue }
    try {
        $sh = New-Object -ComObject WScript.Shell
        $t = [string]($sh.CreateShortcut($l)).TargetPath
        if ($t.StartsWith($Dir + "\", [StringComparison]::OrdinalIgnoreCase)) { $ours += $l }
        else { Say "  Skipping shortcut $l (it points elsewhere)" "Yellow" }
    } catch { }
}
foreach ($l in $ours) { Say "  Shortcut     : $l" }

Write-Host ""
if ($mode -eq "clone") {
    Say "  This folder is a clone that was already there. I will NOT delete the folder." "Yellow"
    Say "  I will remove only: .venv, build, the built .pyd files, install.log, the marker, shortcuts."
} else {
    Say "  I will delete the whole folder and the shortcuts above."
}
if (-not (Ask-YesNo "Really uninstall?" $false)) {
    Say "Cancelled. Nothing was deleted." "Yellow"
    exit 0
}

# leave the folder before deleting it
try { Set-Location -LiteralPath $env:TEMP } catch { }

foreach ($l in $ours) {
    if ($DryRun) { Say "   [dry-run] would delete shortcut $l" "DarkYellow" }
    else { Remove-Item -LiteralPath $l -Force -ErrorAction SilentlyContinue; Say "   removed $l" "Green" }
}

if ($mode -eq "clone") {
    foreach ($n in @(".venv", "build")) { Remove-Tree (Join-Path $Dir $n) }
    foreach ($f in @(Get-ChildItem -LiteralPath $Dir -Filter "*_bridge.*.pyd" -File -ErrorAction SilentlyContinue)) {
        if ($DryRun) { Say "   [dry-run] would delete $($f.FullName)" "DarkYellow" }
        else { Remove-Item -LiteralPath $f.FullName -Force; Say "   removed $($f.FullName)" "Green" }
    }
    foreach ($n in @("install.log", $MarkerName)) {
        $p = Join-Path $Dir $n
        if (Test-Path -LiteralPath $p) {
            if ($DryRun) { Say "   [dry-run] would delete $p" "DarkYellow" }
            else { Remove-Item -LiteralPath $p -Force; Say "   removed $p" "Green" }
        }
    }
} else {
    Remove-Tree $Dir
}

Write-Host ""
if ($DryRun) { Say "DRY RUN finished. Nothing was deleted." "Yellow" }
else { Say "Uninstalled. Bye, robot." "Green" }
exit 0
