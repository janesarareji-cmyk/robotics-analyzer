<#
.SYNOPSIS
  Friendly install wizard for Robotics Analyzer (Windows, PowerShell 5.1, no extra tools).

.DESCRIPTION
  Easiest start: double-click install.bat. Or:
    powershell -ExecutionPolicy Bypass -File install.ps1 [options]

  -InstallDir <path>  where to install (default: %LOCALAPPDATA%\RoboticsAnalyzer, or this folder if
                      the wizard runs inside a clone)
  -Yes                accept every default, ask nothing (for automation)
  -CpuOnly            do not build the GPU module
  -NoShortcuts        do not create Desktop / Start menu shortcuts
  -DryRun             only print what would be done; installs nothing, runs no winget/git/cmake
  -SkipLaunch         do not offer to start the app at the end
  -RepoUrl <url>      (testing) where to clone from; default is the GitHub repository

  The wizard reuses setup.ps1 for the real work. Press Enter for the default answer, Q to quit.
  Safe to run again. Log: install.log in the install folder.
#>
param(
    [string]$InstallDir = "",
    [switch]$Yes,
    [switch]$CpuOnly,
    [switch]$NoShortcuts,
    [switch]$DryRun,
    [switch]$SkipLaunch,
    [string]$RepoUrl = "https://github.com/janesarareji-cmyk/robotics-analyzer.git"
)

# Native tools write warnings to stderr; exit codes are checked by hand.
$ErrorActionPreference = "Continue"
$ProgressPreference = "SilentlyContinue"

$Script:Total = 8
$Script:StepNo = 0
$Script:Done = $false
$Script:EofHits = 0
$Script:LogFile = $null
$Script:LogBuf = New-Object System.Collections.ArrayList
$Script:LastOutput = ""
$Script:Notes = New-Object System.Collections.ArrayList
$MarkerName = ".robotics-analyzer-install"
$MarkerHead = "RoboticsAnalyzer-install-marker"
$ShortcutName = "Robotics Analyzer.lnk"

# ------------------------------------------------------------------ console helpers
$Script:Color = $true
if ($env:NO_COLOR) { $Script:Color = $false }
$Script:Spin = $false
try { if (-not [Console]::IsOutputRedirected) { $Script:Spin = $true } } catch { }
$Script:Utf = $false
try { if ([Console]::OutputEncoding.CodePage -eq 65001) { $Script:Utf = $true } } catch { }
if ($Script:Utf) {
    $Script:Tick = [string][char]0x2713
    $Script:Cross = [string][char]0x2717
} else {
    $Script:Tick = "+"
    $Script:Cross = "x"
}

function Log([string]$t) {
    $t = $t.Replace($Script:Tick, "[ok]").Replace($Script:Cross, "[x]") -replace '[^\x09\x0A\x0D\x20-\x7E]', '?'
    $line = "{0}  {1}" -f (Get-Date -Format "HH:mm:ss"), $t
    if ($Script:LogFile) {
        try { [System.IO.File]::AppendAllText($Script:LogFile, $line + "`r`n") } catch { }
    } else {
        [void]$Script:LogBuf.Add($line)
    }
}

function Say([string]$t, [string]$c = "") {
    Log $t
    if ($c -and $Script:Color) { Write-Host $t -ForegroundColor $c } else { Write-Host $t }
}
function Good([string]$t)   { Say ("   " + $Script:Tick + " " + $t) "Green" }
function Bad([string]$t)    { Say ("   " + $Script:Cross + " " + $t) "Red" }
function Info([string]$t)   { Say ("   " + $t) "Gray" }
function Warn([string]$t)   { Say ("   ! " + $t) "Yellow" }
function Dry([string]$t)    { Say ("   [dry-run] " + $t) "DarkYellow" }

function Prompt([string]$t) {
    Log ("PROMPT " + $t)
    if ($Script:Color) { Write-Host $t -ForegroundColor Cyan -NoNewline } else { Write-Host $t -NoNewline }
}

function Show-Banner {
    $art = @'

         ___________
        /  _     _  \          ROBOTICS ANALYZER
       |  (o)   (o)  |         ------------------
   ____|      ___     |____    install wizard
  |____|     \___/    |____|
       \_____________/         kinematics - dynamics - planning
          |   |   |
        __|___|___|__
'@
    if ($Script:Color) { Write-Host $art -ForegroundColor Cyan } else { Write-Host $art }
    Write-Host ""
}

function Next-Step([string]$title) {
    $Script:StepNo++
    Write-Host ""
    $head = "Step {0}/{1}  {2}" -f $Script:StepNo, $Script:Total, $title
    Log ("=== " + $head)
    if ($Script:Color) {
        Write-Host $head -ForegroundColor White
        Write-Host ("-" * ([Math]::Min(60, [Math]::Max(20, $head.Length)))) -ForegroundColor DarkCyan
    } else {
        Write-Host $head
        Write-Host ("-" * ([Math]::Min(60, [Math]::Max(20, $head.Length))))
    }
}

function Quit-Wizard {
    $Script:Done = $true
    Write-Host ""
    Say "Okay, stopping here. Nothing is half-broken: run the wizard again any time." "Yellow"
    exit 0
}

function Read-Line {
    $r = $null
    try { $r = Read-Host } catch { $r = $null }
    if ($null -eq $r) { $Script:EofHits++; return "" }
    $r = ([string]$r).Trim()
    Log ("ANSWER " + $r)
    if ($r -match '^(q|quit|exit)$') { Quit-Wizard }
    return $r
}

function Ask-YesNo([string]$Q, [bool]$Default = $true) {
    $hint = "[y/N]"
    $dtxt = "no"
    if ($Default) { $hint = "[Y/n]"; $dtxt = "yes" }
    if ($Yes) { Say ("  ? $Q $hint -> $dtxt (auto, -Yes)") "DarkCyan"; return $Default }
    for ($i = 0; $i -lt 3; $i++) {
        Prompt ("  ? $Q $hint ")
        $r = Read-Line
        if ($r -eq "") { return $Default }
        if ($r -match '^(y|yes)$') { return $true }
        if ($r -match '^(n|no)$') { return $false }
        Warn "Please answer y or n (or Q to quit)."
    }
    return $Default
}

function Ask-Text([string]$Q, [string]$Default) {
    if ($Yes) { Say ("  ? $Q -> $Default (auto, -Yes)") "DarkCyan"; return $Default }
    Prompt ("  ? $Q [$Default] ")
    $r = Read-Line
    if ($r -eq "") { return $Default }
    return $r
}

function Ask-Choice([string]$Q, [string[]]$Options, [int]$Default = 1) {
    Say ("  ? $Q") "Cyan"
    for ($i = 0; $i -lt $Options.Count; $i++) { Say ("      {0}) {1}" -f ($i + 1), $Options[$i]) }
    if ($Yes) { Say ("    -> $Default (auto, -Yes)") "DarkCyan"; return $Default }
    for ($t = 0; $t -lt 3; $t++) {
        Prompt ("    Your choice [$Default] ")
        $r = Read-Line
        if ($r -eq "") { return $Default }
        $n = 0
        if ([int]::TryParse($r, [ref]$n) -and $n -ge 1 -and $n -le $Options.Count) { return $n }
        Warn ("Type a number from 1 to {0}." -f $Options.Count)
    }
    return $Default
}

function Fail-Wizard([string]$Msg, [string[]]$Hints) {
    $Script:Done = $true
    Write-Host ""
    Bad $Msg
    if ($Script:LogFile -and (Test-Path -LiteralPath $Script:LogFile)) {
        Write-Host ""
        Say "   Last lines of the log:" "Yellow"
        $tail = @(Get-Content -LiteralPath $Script:LogFile -Tail 25 -ErrorAction SilentlyContinue)
        foreach ($l in $tail) { Write-Host ("   | " + $l) }
    }
    elseif ($Script:LogBuf.Count -gt 0) {
        Write-Host ""
        Say "   Last lines of the log:" "Yellow"
        $from = [Math]::Max(0, $Script:LogBuf.Count - 8)
        for ($k = $from; $k -lt $Script:LogBuf.Count; $k++) { foreach ($l in ([string]$Script:LogBuf[$k] -split "[\r\n]+")) { if ($l) { Write-Host ("   | " + $l) } } }
    }
    if ($Hints) {
        Write-Host ""
        Say "   What to try:" "Yellow"
        foreach ($h in $Hints) { Say ("    - " + $h) }
    }
    if ($Script:LogFile) { Write-Host ""; Say ("   Full log: " + $Script:LogFile) }
    Write-Host ""
    Say "   Fix that, then run the wizard again. It picks up where it stopped." "Yellow"
    exit 1
}

# ------------------------------------------------------------------ running things with a spinner
function Update-PathFromRegistry {
    try {
        $m = [Environment]::GetEnvironmentVariable("Path", "Machine")
        $u = [Environment]::GetEnvironmentVariable("Path", "User")
        $env:Path = (@($m, $u, $env:Path) | Where-Object { $_ }) -join ";"
    } catch { }
}

function Add-ToPath([string]$dir) {
    if ($dir -and (Test-Path -LiteralPath $dir) -and (($env:Path -split ";") -notcontains $dir)) {
        $env:Path = $dir + ";" + $env:Path
    }
}

# Runs a program with output going to temp files; shows a spinner and the latest output line.
# Returns the exit code. Output is appended to the log. -DryRun: prints and returns 0.
function Invoke-Logged([string]$Exe, [string]$ArgString, [string]$Label, [string]$WorkDir) {
    if ($DryRun) {
        Dry ("would run: $Exe $ArgString")
        return 0
    }
    if (-not $WorkDir -or -not (Test-Path -LiteralPath $WorkDir)) { $WorkDir = $env:TEMP }
    $tag = [Guid]::NewGuid().ToString("N").Substring(0, 8)
    $out = Join-Path $env:TEMP ("ra_inst_{0}.out" -f $tag)
    $err = Join-Path $env:TEMP ("ra_inst_{0}.err" -f $tag)
    Log ("RUN $Exe $ArgString (in $WorkDir)")
    $p = $null
    try {
        $p = Start-Process -FilePath $Exe -ArgumentList $ArgString -WorkingDirectory $WorkDir -NoNewWindow -PassThru `
            -RedirectStandardOutput $out -RedirectStandardError $err
    } catch {
        Log ("START FAILED: " + $_.Exception.Message)
        $Script:LastOutput = $_.Exception.Message
        return 9009
    }
    $null = $p.Handle   # keeps the handle so ExitCode is readable
    $frames = @("|", "/", "-", "\")
    $sw = [System.Diagnostics.Stopwatch]::StartNew()
    $pos = 0L
    $phase = ""
    $last = ""
    $i = 0
    $lastPoll = -1000
    $lastDot = 0
    $width = 80
    try { $width = [Console]::WindowWidth } catch { }
    if ($width -lt 40) { $width = 80 }
    try {
        while (-not $p.HasExited) {
            if ($sw.ElapsedMilliseconds - $lastPoll -ge 700) {
                $lastPoll = $sw.ElapsedMilliseconds
                try {
                    $fs = New-Object System.IO.FileStream($out, [System.IO.FileMode]::Open, [System.IO.FileAccess]::Read, [System.IO.FileShare]::ReadWrite)
                    try {
                        if ($fs.Length -gt $pos) {
                            [void]$fs.Seek($pos, [System.IO.SeekOrigin]::Begin)
                            $buf = New-Object byte[] ([int][Math]::Min(262144, $fs.Length - $pos))
                            $n = $fs.Read($buf, 0, $buf.Length)
                            $pos += $n
                            $txt = [System.Text.Encoding]::UTF8.GetString($buf, 0, $n)
                            foreach ($ln in ($txt -split "[\r\n]+")) {
                                $c = ($ln -replace '[^\x20-\x7E]', '').Trim()
                                if ($c) {
                                    $last = $c
                                    if ($c -match '^==\s*(.+)$') { $phase = $Matches[1] }
                                }
                            }
                        }
                    } finally { $fs.Dispose() }
                } catch { }
            }
            $secs = [int]$sw.Elapsed.TotalSeconds
            $clock = "{0:00}:{1:00}" -f [Math]::Floor($secs / 60), ($secs % 60)
            if ($Script:Spin) {
                $head = "   " + $frames[$i % 4] + " " + $Label + "  " + $clock
                if ($phase) { $head += "  [" + $phase + "]" }
                $line = $head
                $room = $width - 2 - $head.Length
                if ($last -and $room -gt 12) {
                    $l2 = $last
                    if ($l2.Length -gt ($room - 3)) { $l2 = $l2.Substring(0, $room - 3) }
                    $line = $head + "  " + $l2
                }
                if ($line.Length -gt ($width - 1)) { $line = $line.Substring(0, $width - 1) }
                [Console]::Write("`r" + $line.PadRight($width - 1))
            } elseif (($secs - $lastDot) -ge 20) {
                $lastDot = $secs
                Write-Host ("   ... $Label  $clock  " + $phase)
            }
            $i++
            Start-Sleep -Milliseconds 120
        }
        $p.WaitForExit()
    } finally {
        if ($p -and -not $p.HasExited) {
            try { & taskkill.exe /T /F /PID $p.Id 2>$null | Out-Null } catch { }
        }
    }
    if ($Script:Spin) { [Console]::Write("`r" + (" " * ($width - 1)) + "`r") }
    $rc = $p.ExitCode
    if ($null -eq $rc) { $rc = 0 }
    # append captured output to the log
    $all = ""
    foreach ($f in @($out, $err)) {
        if (Test-Path -LiteralPath $f) {
            try {
                $t = [System.IO.File]::ReadAllText($f)
                if ($t -and ($f -eq $out)) {
                    foreach ($ln in ($t -split "[\r\n]+")) {
                        $c = ($ln -replace '[^\x20-\x7E]', '').Trim()
                        if ($c) { $last = $c }
                    }
                }
                if ($t) { $all += $t; if (-not $t.EndsWith("`n")) { $all += "`r`n" } }
            } catch { }
            Remove-Item -LiteralPath $f -Force -ErrorAction SilentlyContinue
        }
    }
    if ($all) {
        if ($Script:LogFile) {
            try { [System.IO.File]::AppendAllText($Script:LogFile, $all) } catch { }
        } else {
            [void]$Script:LogBuf.Add($all)
        }
    }
    Log ("EXIT $rc ($Exe)")
    $Script:LastOutput = $last
    return $rc
}

# ------------------------------------------------------------------ detection
function Test-IsClone([string]$dir) {
    if (-not $dir) { return $false }
    return ((Test-Path -LiteralPath (Join-Path $dir "setup.ps1")) -and
            (Test-Path -LiteralPath (Join-Path $dir "CMakeLists.txt")) -and
            (Test-Path -LiteralPath (Join-Path $dir "app\main.py")))
}

function Get-PythonInfo {
    $r = New-Object PSObject -Property @{ Path = $null; Lines = @(); Other = @() }
    $py = Get-Command py -ErrorAction SilentlyContinue
    if ($py) {
        try {
            $list = @(& py -0p 2>$null)
            foreach ($l in $list) { $t = ([string]$l).Trim(); if ($t) { $r.Lines += $t } }
        } catch { }
        try {
            $c = (& py -3.13 -c "import sys; print(sys.executable)" 2>$null) | Select-Object -First 1
            if ($LASTEXITCODE -eq 0 -and $c -and (Test-Path -LiteralPath $c)) { $r.Path = [string]$c }
        } catch { }
    }
    $pc = Get-Command python -ErrorAction SilentlyContinue
    if ($pc -and $pc.Source -notmatch 'WindowsApps') {
        try {
            $v = (& python -c "import sys; print('%d.%d.%d' % sys.version_info[:3])" 2>$null) | Select-Object -First 1
            if ($v) { $r.Other += ("python on PATH: {0} (version {1})" -f $pc.Source, $v) }
            if (-not $r.Path -and $v -and ([string]$v).StartsWith("3.13.")) { $r.Path = $pc.Source }
        } catch { }
    } elseif ($pc) {
        $r.Other += "python on PATH is only the Microsoft Store stub (does not count)"
    }
    if (-not $r.Path) {
        foreach ($c in @((Join-Path $env:LOCALAPPDATA "Programs\Python\Python313\python.exe"),
                         (Join-Path $env:ProgramFiles "Python313\python.exe"))) {
            if (Test-Path -LiteralPath $c) { $r.Path = $c; Add-ToPath (Split-Path $c -Parent); break }
        }
    }
    return $r
}

function Get-VsWhere {
    $pf = ${env:ProgramFiles(x86)}
    if (-not $pf) { return $null }
    $v = Join-Path $pf "Microsoft Visual Studio\Installer\vswhere.exe"
    if (Test-Path -LiteralPath $v) { return $v }
    return $null
}

function Get-CMakeInfo {
    $r = New-Object PSObject -Property @{ Path = $null; Version = $null }
    $c = Get-Command cmake -ErrorAction SilentlyContinue
    if ($c) { $r.Path = $c.Source }
    if (-not $r.Path) {
        $vw = Get-VsWhere
        if ($vw) {
            $vs = (& $vw -latest -products * -property installationPath 2>$null) | Select-Object -First 1
            if ($vs) {
                $x = Join-Path $vs "Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe"
                if (Test-Path -LiteralPath $x) { $r.Path = $x }
            }
        }
    }
    if (-not $r.Path) {
        $x = Join-Path $env:ProgramFiles "CMake\bin\cmake.exe"
        if (Test-Path -LiteralPath $x) { $r.Path = $x; Add-ToPath (Split-Path $x -Parent) }
    }
    if ($r.Path) {
        $line = (& $r.Path --version 2>$null) | Select-Object -First 1
        if ($line -and ([string]$line) -match '(\d+)\.(\d+)\.(\d+)') {
            $r.Version = [version]("{0}.{1}.{2}" -f $Matches[1], $Matches[2], $Matches[3])
        }
    }
    return $r
}

function Get-VsCppPath {
    $vw = Get-VsWhere
    if (-not $vw) { return $null }
    $p = (& $vw -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath 2>$null) | Select-Object -First 1
    if ($p) { return [string]$p }
    return $null
}

function Get-GpuInfo {
    $r = New-Object PSObject -Property @{ Name = $null; Cap = $null; Nvcc = $null }
    try {
        $l = (& nvidia-smi --query-gpu=name,compute_cap --format=csv,noheader 2>$null) | Select-Object -First 1
        if ($l -and ([string]$l) -match '^(.+),\s*(\d+\.\d+)\s*$') { $r.Name = $Matches[1].Trim(); $r.Cap = $Matches[2] }
    } catch { }
    $n = Get-Command nvcc -ErrorAction SilentlyContinue
    if ($n) { $r.Nvcc = $n.Source }
    if (-not $r.Nvcc -and $env:CUDA_PATH) {
        $c = Join-Path $env:CUDA_PATH "bin\nvcc.exe"
        if (Test-Path -LiteralPath $c) { $r.Nvcc = $c }
    }
    return $r
}

function Has-Winget { return [bool](Get-Command winget -ErrorAction SilentlyContinue) }

function Read-Marker([string]$dir) {
    $m = Join-Path $dir $MarkerName
    $h = @{}
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

function Write-Marker([string]$dir, [string]$mode, [string[]]$shortcuts) {
    if ($DryRun) { Dry "would write marker file $MarkerName"; return }
    $lines = @($MarkerHead, "created_by=install.ps1", "version=1", "mode=$mode", "dir=$dir", "date=$(Get-Date -Format 'yyyy-MM-dd HH:mm')")
    foreach ($s in $shortcuts) { if ($s) { $lines += "shortcut=$s" } }
    [System.IO.File]::WriteAllText((Join-Path $dir $MarkerName), (($lines -join "`r`n") + "`r`n"))
}

# ================================================================== STEP 1: welcome
Show-Banner
if ($DryRun) { Say "  *** DRY RUN: I only talk. Nothing will be installed, created or downloaded. ***" "Yellow"; Write-Host "" }
Next-Step "Welcome"
Say "  Hi! I will set up Robotics Analyzer on this PC. About what happens:"
Info "1. you pick a folder, I fetch the source code (git or zip)"
Info "2. I check Python 3.13, CMake and the Visual Studio C++ tools (and offer to install them)"
Info "3. you choose GPU or CPU, then I build everything with setup.ps1 (a few minutes)"
Info "4. optional shortcuts, a quick self test, and a launch"
Info "Enter = default answer.  Q = quit.  Safe to run again."
if (-not $Yes) {
    Write-Host ""
    Prompt "  Press Enter to begin (or Q to quit) "
    [void](Read-Line)
}

# ================================================================== STEP 2: folder
Next-Step "Install folder"
$HereClone = Test-IsClone $PSScriptRoot
$DefaultDir = Join-Path $env:LOCALAPPDATA "RoboticsAnalyzer"
$ShortDir = "C:\RoboticsAnalyzer"
$Script:Mode = "fresh"
$Script:SourcePresent = $false
$Dir = ""

function Test-DirChoice([string]$cand) {
    # returns hashtable: Ok, Msg, Full
    $res = @{ Ok = $false; Msg = ""; Full = "" }
    if (-not $cand) { $res.Msg = "The folder is empty."; return $res }
    try { $full = [System.IO.Path]::GetFullPath($cand.Trim('"').Trim()) } catch { $res.Msg = "That is not a valid path."; return $res }
    if ($full.Length -gt 3) { $full = $full.TrimEnd('\') }
    $res.Full = $full
    if ($full -match '[%!^&]') { $res.Msg = "The path must not contain the characters % ! ^ &."; return $res }
    $root = [System.IO.Path]::GetPathRoot($full)
    if ($full.TrimEnd('\') -eq $root.TrimEnd('\')) { $res.Msg = "Do not install into a drive root. Pick a folder."; return $res }
    $bad = @($env:USERPROFILE, $env:SystemRoot, $env:ProgramFiles, ${env:ProgramFiles(x86)}, $env:LOCALAPPDATA, $env:APPDATA)
    foreach ($b in $bad) { if ($b -and ($full -ieq $b.TrimEnd('\'))) { $res.Msg = "That is a system folder. Pick a sub-folder of it."; return $res } }
    if (Test-Path -LiteralPath $full -PathType Leaf) { $res.Msg = "That path is a file, not a folder."; return $res }
    if (Test-Path -LiteralPath $full -PathType Container) {
        $items = @(Get-ChildItem -LiteralPath $full -Force -ErrorAction SilentlyContinue)
        if ($items.Count -gt 0) {
            $mk = Read-Marker $full
            if (-not $mk.ContainsKey("_valid") -and -not (Test-IsClone $full)) {
                $res.Msg = "The folder is not empty and does not look like a Robotics Analyzer install. I will not touch it."
                return $res
            }
        }
    } else {
        $anc = $full
        while ($anc -and -not (Test-Path -LiteralPath $anc)) { $anc = Split-Path $anc -Parent }
        if (-not $anc -or -not (Test-Path -LiteralPath $anc -PathType Container)) { $res.Msg = "That drive or parent folder does not exist."; return $res }
        if (-not $DryRun) {
            $probe = Join-Path $anc (".ra_probe_" + [Guid]::NewGuid().ToString("N").Substring(0, 6))
            try { Set-Content -LiteralPath $probe -Value "x" -ErrorAction Stop; Remove-Item -LiteralPath $probe -Force }
            catch { $res.Msg = "I cannot write into $anc (no permission?)."; return $res }
        }
    }
    $res.Ok = $true
    return $res
}

$chosen = $InstallDir
$explicit = [bool]$InstallDir
for ($try = 0; $try -lt 4; $try++) {
    if (-not $chosen) {
        if ($HereClone) {
            Say "  This wizard is running from a Robotics Analyzer folder:" "White"
            Info $PSScriptRoot
            if (Ask-YesNo "Use this folder (no copy, no download)?" $true) { $chosen = $PSScriptRoot }
        }
        if (-not $chosen) {
            if (-not $Yes) { Info "Short paths are best: Windows has a 260 character limit and the build is deep." }
            $chosen = Ask-Text "Install folder" $DefaultDir
        }
    }
    $chk = Test-DirChoice $chosen
    if ($chk.Ok) {
        if ($chk.Full.Length -gt 70) {
            Warn ("The path is {0} characters long (limit trouble starts around 70+)." -f $chk.Full.Length)
            if (-not $explicit -and -not $Yes -and ($chk.Full -ine $ShortDir)) {
                if (Ask-YesNo "Use the short folder $ShortDir instead?" $true) {
                    $chosen = $ShortDir
                    $chk = Test-DirChoice $chosen
                    if (-not $chk.Ok) { Bad $chk.Msg; $chosen = ""; continue }
                }
            } else {
                Info "Continuing with the long path. If the build fails, rerun with a short one, e.g. $ShortDir."
            }
        }
        $Dir = $chk.Full
        break
    }
    Bad $chk.Msg
    if ($explicit -or $Yes) { Fail-Wizard "Cannot use the install folder." @("Pick another folder with -InstallDir, e.g. $ShortDir.") }
    $chosen = ""
}
if (-not $Dir) { Fail-Wizard "No usable install folder after 4 tries." @("Run the wizard again and give an empty folder or a new path.") }

$mk = Read-Marker $Dir
if (Test-IsClone $Dir) {
    $Script:SourcePresent = $true
    # S4: a folder that already holds source is a clone. A marker found there is never trusted
    # for "mode": this run writes its own marker with mode=clone.
    $Script:Mode = "clone"
} elseif ($mk.ContainsKey("_valid")) {
    Info "Found an earlier unfinished install here. I will continue it."
}
Good ("Install folder: " + $Dir)
if ($Script:SourcePresent) { Info "Source code is already there." }

function Open-Log {
    # the log lives in the install folder; it is opened only when the folder may hold files
    # (git clone needs an empty folder, so a fresh install opens it after the clone)
    if ($DryRun -or $Script:LogFile) { return }
    if (-not (Test-Path -LiteralPath $Dir)) { New-Item -ItemType Directory -Path $Dir -Force | Out-Null }
    $Script:LogFile = Join-Path $Dir "install.log"
    try {
        [System.IO.File]::AppendAllText($Script:LogFile, ("`r`n==== Robotics Analyzer install " + (Get-Date -Format "yyyy-MM-dd HH:mm") + " ====`r`n"))
        foreach ($b in $Script:LogBuf) { [System.IO.File]::AppendAllText($Script:LogFile, $b + "`r`n") }
    } catch { $Script:LogFile = $null }
    $Script:LogBuf.Clear()
}
if ($DryRun) { Dry ("would create the folder " + $Dir + " and the log install.log in it") }
elseif ($Script:SourcePresent) { Open-Log }

# ================================================================== STEP 3: source
Next-Step "Get the source code"
if ($Script:SourcePresent) {
    Good "Source is already in place, nothing to download."
} else {
    $git = Get-Command git -ErrorAction SilentlyContinue
    if ($git) {
        Good ("Git found: " + $git.Source)
    } else {
        Warn "Git is not installed."
        $installGit = $false
        if (Has-Winget) {
            Info "winget can install Git for you (about 60 MB, a minute or two)."
            $installGit = Ask-YesNo "Install Git with winget now? (No = download a zip instead)" $true
        } else {
            Info "winget is not available, so I will use the zip download instead."
        }
        if ($installGit) {
            $rc = Invoke-Logged "winget" "install --id Git.Git -e --accept-package-agreements --accept-source-agreements" "Installing Git" $env:TEMP
            Update-PathFromRegistry
            $git = Get-Command git -ErrorAction SilentlyContinue
            if (-not $git) {
                $gx = Join-Path $env:ProgramFiles "Git\cmd\git.exe"
                if (Test-Path -LiteralPath $gx) { Add-ToPath (Split-Path $gx -Parent); $git = Get-Command git -ErrorAction SilentlyContinue }
            }
            if ($DryRun) { Good "Git would be installed." }
            elseif ($git) { Good "Git installed." }
            else { Warn "Git still not found (winget exit code $rc). Falling back to the zip download." }
        }
    }

    $viaGit = [bool]$git -or ($DryRun -and (Has-Winget) -and -not $git)
    if ($viaGit) {
        $rc = Invoke-Logged "git" ("clone `"$RepoUrl`" `"$Dir`"") "Cloning the repository" $env:TEMP
        if ($rc -ne 0) {
            Fail-Wizard "git clone failed (exit code $rc)." @("Check your internet connection and that $RepoUrl is reachable.", "Make sure the folder is empty, or pick another one.", "Or install without git: rerun and say No to the Git question to get the zip.")
        }
    } else {
        $zipUrl = $null
        if ($RepoUrl -match '^https://github\.com/([^/]+)/([^/]+?)(\.git)?/?$') {
            $zipUrl = "https://github.com/{0}/{1}/archive/refs/heads/main.zip" -f $Matches[1], $Matches[2]
        }
        if (-not $zipUrl) {
            Fail-Wizard "No git and no zip address for this repository." @("Install Git from https://git-scm.com/ and rerun.")
        }
        if ($DryRun) {
            Dry "would download $zipUrl and unpack it into $Dir"
        } else {
            $work = Join-Path $env:TEMP ("ra_zip_" + [Guid]::NewGuid().ToString("N").Substring(0, 8))
            New-Item -ItemType Directory -Path $work -Force | Out-Null
            $zip = Join-Path $work "src.zip"
            Say "   Downloading the zip archive (about 100 MB, depends on your line)..."
            try {
                Invoke-WebRequest -Uri $zipUrl -OutFile $zip -UseBasicParsing
                Expand-Archive -LiteralPath $zip -DestinationPath (Join-Path $work "x") -Force
                $top = @(Get-ChildItem -LiteralPath (Join-Path $work "x") | Select-Object -First 1)
                if (-not $top) { throw "the archive was empty" }
                Get-ChildItem -LiteralPath $top[0].FullName -Force | Move-Item -Destination $Dir -Force
            } catch {
                Remove-Item -LiteralPath $work -Recurse -Force -ErrorAction SilentlyContinue
                Fail-Wizard ("Zip download failed: " + $_.Exception.Message) @("Check your internet connection and try again.", "Or install Git (https://git-scm.com/) and rerun.")
            }
            Remove-Item -LiteralPath $work -Recurse -Force -ErrorAction SilentlyContinue
        }
    }
    if (-not $DryRun) {
        if (-not (Test-IsClone $Dir)) {
            Fail-Wizard "The source code arrived, but setup.ps1 / CMakeLists.txt / app\main.py are missing." @("Is the repository address right? ($RepoUrl)", "Delete the folder $Dir and rerun.")
        }
        $Script:SourcePresent = $true
    }
    Good "Source code is ready."
}
Open-Log
Write-Marker $Dir $Script:Mode @()

# ================================================================== STEP 4: Python
Next-Step "Python 3.13"
Say "  Why 3.13 exactly? The compiled modules are built for it. Other versions will not do." "Gray"
$pyInfo = Get-PythonInfo
foreach ($l in $pyInfo.Lines) { Info ("py launcher: " + $l) }
if ($pyInfo.Lines.Count -eq 0 -and -not (Get-Command py -ErrorAction SilentlyContinue)) { Info "py launcher: not installed" }
foreach ($l in $pyInfo.Other) { Info $l }
if ($pyInfo.Path) {
    Good ("Python 3.13 found: " + $pyInfo.Path)
} else {
    Warn "Python 3.13 (64-bit) was not found."
    $manual = @("Install it from https://www.python.org/downloads/windows/ (tick 'Add python.exe to PATH'), then rerun.")
    if (-not (Has-Winget)) {
        Fail-Wizard "Python 3.13 is missing and winget is not available." $manual
    }
    Info "winget can install it: Python.Python.3.13, about 30 MB, about a minute."
    if (-not (Ask-YesNo "Install Python 3.13 with winget now?" $true)) {
        Fail-Wizard "Python 3.13 is required." $manual
    }
    $rc = Invoke-Logged "winget" "install --id Python.Python.3.13 -e --accept-package-agreements --accept-source-agreements" "Installing Python 3.13" $env:TEMP
    if ($DryRun) {
        Good "Python 3.13 would be installed."
        [void]$Script:Notes.Add("Python 3.13 (would be installed)")
    } else {
        Update-PathFromRegistry
        $pyInfo = Get-PythonInfo
        if ($pyInfo.Path) { Good ("Python 3.13 installed: " + $pyInfo.Path) }
        else { Fail-Wizard "Python 3.13 still not found after winget (exit code $rc)." (@("Open a NEW terminal and rerun the wizard (the PATH may need a refresh).") + $manual) }
    }
}

# ================================================================== STEP 5: CMake + VS
Next-Step "Build tools (CMake + Visual Studio C++)"
$cm = Get-CMakeInfo
$cmOk = ($cm.Path -and $cm.Version -and $cm.Version -ge [version]"3.25.0")
if ($cmOk) {
    Good ("CMake {0}: {1}" -f $cm.Version, $cm.Path)
} else {
    if ($cm.Path) { Warn ("CMake {0} is too old (3.25 or newer is needed)." -f $cm.Version) } else { Warn "CMake was not found." }
    $manual = @("Install CMake from https://cmake.org/download/ ('Add CMake to the PATH'), then rerun.")
    if (-not (Has-Winget)) { Fail-Wizard "CMake is missing and winget is not available." $manual }
    Info "winget can install it: Kitware.CMake, about 50 MB, about a minute."
    if (-not (Ask-YesNo "Install CMake with winget now?" $true)) { Fail-Wizard "CMake 3.25+ is required." $manual }
    $rc = Invoke-Logged "winget" "install --id Kitware.CMake -e --accept-package-agreements --accept-source-agreements" "Installing CMake" $env:TEMP
    if ($DryRun) {
        Good "CMake would be installed."
        [void]$Script:Notes.Add("CMake (would be installed)")
    } else {
        Update-PathFromRegistry
        $cm = Get-CMakeInfo
        if ($cm.Path -and $cm.Version -and $cm.Version -ge [version]"3.25.0") { Good ("CMake {0} installed." -f $cm.Version) }
        else { Fail-Wizard "CMake still not usable after winget (exit code $rc)." (@("Open a NEW terminal and rerun the wizard.") + $manual) }
    }
}

$vc = Get-VsCppPath
if ($vc) {
    Good ("C++ tools found: " + $vc)
} else {
    $vw = Get-VsWhere
    if ($vw) { Warn "Visual Studio is there, but the C++ workload is missing." } else { Warn "Visual Studio C++ Build Tools were not found." }
    $manual = @("Install 'Build Tools for Visual Studio' with the workload 'Desktop development with C++':", "https://visualstudio.microsoft.com/downloads/ (section 'Tools for Visual Studio'), then rerun.")
    if (-not (Has-Winget)) { Fail-Wizard "The C++ tools are missing and winget is not available." $manual }
    Say "  Heads up, this one is big (honest numbers):" "Yellow"
    Info "download + install: 3 to 7 GB on disk, roughly 15 to 40 minutes"
    Info "Windows will ask for administrator permission (a UAC prompt) - say yes there"
    Info "grab a coffee; I will keep the spinner going"
    if (-not (Ask-YesNo "Install the Visual Studio 2022 Build Tools (C++ workload) with winget now?" $true)) {
        Fail-Wizard "The C++ tools are required to build the core." $manual
    }
    $over = "--override `"--passive --wait --norestart --add Microsoft.VisualStudio.Workload.VCTools --includeRecommended`""
    $rc = Invoke-Logged "winget" ("install --id Microsoft.VisualStudio.2022.BuildTools -e --accept-package-agreements --accept-source-agreements " + $over) "Installing C++ Build Tools" $env:TEMP
    if ($DryRun) {
        Good "The C++ Build Tools would be installed."
        [void]$Script:Notes.Add("Visual Studio C++ Build Tools (would be installed)")
    } else {
        $vc = Get-VsCppPath
        if ($vc) { Good ("C++ tools installed: " + $vc) }
        else { Fail-Wizard "The C++ tools are still not detected after winget (exit code $rc)." (@("If a reboot was requested, reboot and rerun the wizard.") + $manual) }
    }
}

# ================================================================== STEP 6: GPU
Next-Step "GPU or CPU?"
$gpu = Get-GpuInfo
if ($gpu.Name) { Good ("NVIDIA GPU: {0} (compute capability {1})" -f $gpu.Name, $gpu.Cap) } else { Info "NVIDIA GPU: none detected (nvidia-smi gave nothing)" }
if ($gpu.Nvcc) { Good ("CUDA Toolkit (nvcc): " + $gpu.Nvcc) } else { Info "CUDA Toolkit (nvcc): not found" }
$gpuReady = [bool]($gpu.Name -and $gpu.Nvcc)
$useCpu = $false
if ($CpuOnly) {
    $useCpu = $true
    Info "-CpuOnly given: building without the GPU module."
} else {
    $auto = "Auto (recommended): GPU module if GPU + CUDA Toolkit are found, else CPU"
    $choice = Ask-Choice "How should I build it?" @($auto, "CPU only (works everywhere, big workspace runs are slower)", "Force GPU (needs NVIDIA GPU + CUDA Toolkit)") 1
    if ($choice -eq 2) { $useCpu = $true }
    if ($choice -eq 3 -and -not $gpuReady) {
        Warn "You asked for the GPU, but I do not see both a GPU and the CUDA Toolkit."
        Info "CUDA Toolkit: https://developer.nvidia.com/cuda-downloads (with its Visual Studio integration)"
        if (-not (Ask-YesNo "Continue anyway? (setup then falls back to CPU by itself)" $false)) {
            Say "  Install the CUDA Toolkit and run the wizard again." "Yellow"
            Quit-Wizard
        }
    }
    if ($choice -eq 1 -and -not $gpuReady) { Info "No complete GPU setup found, so Auto means CPU here. The app works the same, just slower on big runs." }
}
if ($useCpu) { Good "Mode: CPU only" } elseif ($gpuReady) { Good "Mode: GPU module will be built" } else { Good "Mode: CPU (no usable GPU toolchain)" }

# ================================================================== STEP 7: run setup.ps1
Next-Step "Build and install (setup.ps1)"
Say "  This is the long step: Python packages (PySide6, VTK, ...) then the C++ build." "Gray"
Say "  First run: roughly 5 to 15 minutes. Go stretch." "Gray"
$setupArgs = "-NoProfile -ExecutionPolicy Bypass -File `"" + (Join-Path $Dir "setup.ps1") + "`""
if ($useCpu) { $setupArgs += " -CpuOnly" }
$rc = Invoke-Logged "powershell.exe" $setupArgs "Building" $Dir
if ($DryRun) {
    Good "setup.ps1 would run here."
} elseif ($rc -ne 0) {
    Fail-Wizard "setup.ps1 failed (exit code $rc)." @(
        "Read the lines above; the first error is usually the real cause.",
        "No internet during CMake configure? Eigen comes from gitlab.com, the rest from github.com.",
        "Very long path? Try a short folder like $ShortDir.",
        "Retry from zero: powershell -ExecutionPolicy Bypass -File `"$Dir\setup.ps1`" -Clean")
} else {
    Good "Packages installed and the C++ core is built."
}

# ================================================================== STEP 8: shortcuts, smoke, launch
Next-Step "Finishing touches"
$made = @()
$doShort = -not $NoShortcuts
if ($doShort) { $doShort = Ask-YesNo "Create Desktop and Start-menu shortcuts?" $true }
if ($doShort) {
    $targets = @()
    try { $targets += (Join-Path ([Environment]::GetFolderPath("Desktop")) $ShortcutName) } catch { }
    try { $targets += (Join-Path ([Environment]::GetFolderPath("Programs")) $ShortcutName) } catch { }
    $runBat = Join-Path $Dir "run.bat"
    foreach ($lnk in $targets) {
        if (-not $lnk -or -not (Test-Path -LiteralPath (Split-Path $lnk -Parent))) { Warn ("Folder for shortcut not found: " + $lnk); continue }
        if ($DryRun) { Dry ("would create shortcut " + $lnk + " -> " + $runBat); continue }
        try {
            $sh = New-Object -ComObject WScript.Shell
            if (Test-Path -LiteralPath $lnk) {
                $old = $sh.CreateShortcut($lnk)
                if (-not ([string]$old.TargetPath).StartsWith($Dir.TrimEnd([char]92) + [string][char]92, [StringComparison]::OrdinalIgnoreCase)) {
                    Warn ("A shortcut with that name already exists and belongs to something else, leaving it: " + $lnk)
                    continue
                }
            }
            $s = $sh.CreateShortcut($lnk)
            $s.TargetPath = $runBat
            $s.WorkingDirectory = $Dir
            $s.Description = "Robotics Analyzer"
            $s.WindowStyle = 7
            $s.Save()
            $made += $lnk
            Good ("Shortcut: " + $lnk)
        } catch { Warn ("Could not create " + $lnk + ": " + $_.Exception.Message) }
    }
} else {
    Info "No shortcuts. Start the app with run.bat in the install folder."
}
Write-Marker $Dir $Script:Mode $made

# smoke test
$smokeOk = $true
if ($DryRun) {
    Dry "would run the smoke test: python -m app.main --smoke"
} else {
    $vpy = Join-Path $Dir ".venv\Scripts\python.exe"
    $env:QT_QPA_PLATFORM = "offscreen"
    $rc = Invoke-Logged $vpy "-m app.main --smoke" "Self test" $Dir
    Remove-Item Env:\QT_QPA_PLATFORM -ErrorAction SilentlyContinue
    if ($rc -eq 0) { Good ("Self test passed. " + $Script:LastOutput) }
    else { $smokeOk = $false; Fail-Wizard "The self test (--smoke) failed (exit code $rc)." @("Retry from zero: powershell -ExecutionPolicy Bypass -File `"$Dir\setup.ps1`" -Clean") }
}

# ------------------------------------------------------------------ summary
Write-Host ""
if ($DryRun) {
    Say "DRY RUN finished. Nothing was installed, downloaded or created." "Yellow"
    if ($Script:Notes.Count -gt 0) { Say ("  Would have installed: " + ($Script:Notes -join ", ")) }
    Say "  Run again without -DryRun to do it for real."
    $Script:Done = $true
    exit 0
}
Say "ALL DONE. The robot is awake." "Green"
Info ("Installed in : " + $Dir)
Info ("Start it     : " + (Join-Path $Dir "run.bat") + "  (or the shortcut)")
Info ("Uninstall    : " + (Join-Path $Dir "uninstall.bat"))
Info ("Log          : " + $Script:LogFile)
$Script:Done = $true
if (-not $SkipLaunch) {
    if (Ask-YesNo "Launch Robotics Analyzer now?" $true) {
        Start-Process -FilePath (Join-Path $Dir "run.bat") -WorkingDirectory $Dir | Out-Null
        Say "  Launched. Have fun!" "Green"
    }
}
exit 0
