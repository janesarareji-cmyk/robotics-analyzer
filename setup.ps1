<#
.SYNOPSIS
  One-step install for Robotics Analyzer: Python venv, packages, C++/CUDA build.

.DESCRIPTION
  Usage:  powershell -ExecutionPolicy Bypass -File setup.ps1 [-CpuOnly] [-SkipBuild] [-Dev] [-Clean]

  -CpuOnly    do not build the CUDA module gpu_bridge (the app then runs on the CPU)
  -SkipBuild  only create the venv and install packages
  -Dev        also install requirements-dev.txt (pytest, pytest-qt, mujoco)
  -Clean      delete .venv, build/ and the built .pyd files first

  Safe to run again. Needs no administrator rights.
#>
param(
    [switch]$CpuOnly,
    [switch]$SkipBuild,
    [switch]$Dev,
    [switch]$Clean
)

# Native tools write warnings to stderr; exit codes are checked by hand.
$ErrorActionPreference = "Continue"
$Root = $PSScriptRoot
Set-Location $Root
$VenvDir = Join-Path $Root ".venv"
$BuildDir = Join-Path $Root "build"
$VenvPy = Join-Path $VenvDir "Scripts\python.exe"

function Step($t) { Write-Host ""; Write-Host "== $t" -ForegroundColor Cyan }
function Ok($t)   { Write-Host "   OK: $t" -ForegroundColor Green }
function Warn($t) { Write-Host "   WARNING: $t" -ForegroundColor Yellow }
function Fail($t) {
    Write-Host ""
    Write-Host "SETUP FAILED: $t" -ForegroundColor Red
    exit 1
}

Write-Host "Robotics Analyzer setup" -ForegroundColor Cyan

# ---------------------------------------------------------------- Clean
if ($Clean) {
    Step "Clean"
    foreach ($p in @($VenvDir, $BuildDir)) {
        if (Test-Path $p) { Remove-Item -Recurse -Force $p; Write-Host "   removed $p" }
    }
    $old = @(Get-ChildItem -Path $Root -Filter "*_bridge.*.pyd" -ErrorAction SilentlyContinue) +
           @(Get-ChildItem -Path (Join-Path $Root "bindings") -Filter "*_bridge.*.pyd" -ErrorAction SilentlyContinue)
    foreach ($p in $old) { Remove-Item -Force $p.FullName; Write-Host "   removed $($p.FullName)" }
}

# ---------------------------------------------------------------- Python 3.13
Step "Python 3.13"
$PyExe = $null
try {
    $cand = (& py -3.13 -c "import sys; print(sys.executable)") | Select-Object -First 1
    if ($LASTEXITCODE -eq 0 -and $cand -and (Test-Path $cand)) { $PyExe = $cand }
} catch { }
if (-not $PyExe) {
    try {
        $v = (& python -c "import sys; print('%d.%d' % sys.version_info[:2])") | Select-Object -First 1
        if ($LASTEXITCODE -eq 0 -and $v -eq "3.13") {
            $PyExe = (& python -c "import sys; print(sys.executable)") | Select-Object -First 1
        }
    } catch { }
}
if (-not $PyExe) {
    Fail ("Python 3.13 (64-bit) was not found.`n" +
          "   Install it from https://www.python.org/downloads/windows/ (tick 'Add python.exe to PATH'),`n" +
          "   then open a new terminal and run setup.ps1 again.`n" +
          "   Other Python versions are not supported: the compiled modules are built for 3.13.")
}
$bits = (& $PyExe -c "import struct; print(struct.calcsize('P')*8)") | Select-Object -First 1
if ($bits -ne "64") { Fail "Python at $PyExe is 32-bit. Install the 64-bit Python 3.13." }
Ok $PyExe

# ---------------------------------------------------------------- CMake + compiler
$CMake = $null
if (-not $SkipBuild) {
    Step "CMake (3.25 or newer)"
    $VsWhere = Join-Path ${env:ProgramFiles(x86)} "Microsoft Visual Studio\Installer\vswhere.exe"
    $cmCmd = Get-Command cmake -ErrorAction SilentlyContinue
    if ($cmCmd) { $CMake = $cmCmd.Source }
    if (-not $CMake -and (Test-Path $VsWhere)) {
        $vsPath = (& $VsWhere -latest -products * -property installationPath) | Select-Object -First 1
        if ($vsPath) {
            $c = Join-Path $vsPath "Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe"
            if (Test-Path $c) { $CMake = $c }
        }
    }
    if (-not $CMake) {
        Fail ("CMake was not found.`n" +
              "   Install CMake 3.25 or newer from https://cmake.org/download/ (choose 'Add CMake to the PATH'),`n" +
              "   or run:  winget install Kitware.CMake`n" +
              "   Then open a new terminal and run setup.ps1 again.")
    }
    $verLine = (& $CMake --version) | Select-Object -First 1
    if ($verLine -notmatch '(\d+)\.(\d+)\.(\d+)') { Fail "Cannot read the CMake version from: $verLine" }
    $cmVer = [version]("{0}.{1}.{2}" -f $Matches[1], $Matches[2], $Matches[3])
    if ($cmVer -lt [version]"3.25.0") {
        Fail "CMake $cmVer is too old. Version 3.25 or newer is needed (https://cmake.org/download/)."
    }
    Ok "CMake $cmVer ($CMake)"

    Step "C++ compiler (Visual Studio Build Tools)"
    if (-not (Test-Path $VsWhere)) {
        Fail ("Visual Studio Build Tools were not found (vswhere.exe is missing).`n" +
              "   Install 'Build Tools for Visual Studio' with the workload 'Desktop development with C++':`n" +
              "   https://visualstudio.microsoft.com/downloads/  (section 'Tools for Visual Studio')`n" +
              "   or run:  winget install Microsoft.VisualStudio.2022.BuildTools --override `"--add Microsoft.VisualStudio.Workload.VCTools --includeRecommended`"`n" +
              "   Then run setup.ps1 again.")
    }
    $vc = (& $VsWhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath) | Select-Object -First 1
    if (-not $vc) {
        Fail ("Visual Studio is installed but the C++ tools are missing.`n" +
              "   Open the Visual Studio Installer, choose Modify, tick 'Desktop development with C++', install,`n" +
              "   then run setup.ps1 again.")
    }
    Ok "MSVC tools in $vc"
}

# ---------------------------------------------------------------- venv + packages
Step "Virtual environment (.venv)"
if (Test-Path $VenvPy) {
    $vv = (& $VenvPy -c "import sys; print('%d.%d' % sys.version_info[:2])") | Select-Object -First 1
    if ($vv -ne "3.13") {
        Warn ".venv uses Python $vv, recreating it with 3.13"
        Remove-Item -Recurse -Force $VenvDir
    }
}
if (-not (Test-Path $VenvPy)) {
    & $PyExe -m venv $VenvDir
    if ($LASTEXITCODE -ne 0 -or -not (Test-Path $VenvPy)) { Fail "Could not create the virtual environment in $VenvDir." }
    Ok "created"
} else {
    Ok "exists"
}

Step "Python packages"
& $VenvPy -m pip install --upgrade pip
if ($LASTEXITCODE -ne 0) { Warn "pip upgrade failed (continuing)" }
$reqFile = "requirements.txt"
if ($Dev) { $reqFile = "requirements-dev.txt" }
& $VenvPy -m pip install -r (Join-Path $Root $reqFile)
if ($LASTEXITCODE -ne 0) {
    Fail "pip could not install $reqFile. Check the internet connection and the messages above, then run setup.ps1 again."
}
Ok "installed $reqFile"

# ---------------------------------------------------------------- build
$GpuBuilt = $false
if ($SkipBuild) {
    Step "Build"
    Write-Host "   skipped (-SkipBuild)"
} else {
    Step "CUDA detection"
    $useCuda = $false
    $arch = $null
    $nvcc = $null
    if ($CpuOnly) {
        Write-Host "   skipped (-CpuOnly): the app will run on the CPU"
    } else {
        $nv = Get-Command nvcc -ErrorAction SilentlyContinue
        if ($nv) { $nvcc = $nv.Source }
        if (-not $nvcc -and $env:CUDA_PATH) {
            $c = Join-Path $env:CUDA_PATH "bin\nvcc.exe"
            if (Test-Path $c) { $nvcc = $c }
        }
        if (-not $nvcc) {
            Warn ("CUDA Toolkit (nvcc) not found. Building CPU-only.`n" +
                  "   For the GPU module install the CUDA Toolkit from https://developer.nvidia.com/cuda-downloads`n" +
                  "   and run setup.ps1 again.")
        } else {
            $cc = $null
            try {
                $cc = (& nvidia-smi --query-gpu=compute_cap --format=csv,noheader) | Select-Object -First 1
            } catch { }
            if ($cc) { $cc = "$cc".Trim() }
            if ($cc -and $cc -match '^(\d+)\.(\d+)$') {
                $arch = "{0}{1}-real" -f $Matches[1], $Matches[2]
                $useCuda = $true
                Ok "nvcc $nvcc ; GPU compute capability $cc -> architecture $arch"
            } else {
                Warn "CUDA Toolkit found but no NVIDIA GPU or driver was detected (nvidia-smi). Building CPU-only."
            }
        }
    }

    if ($Root.Length -gt 70) {
        Warn ("The folder path is long ($($Root.Length) characters). Windows limits paths to 260 characters and the")
        Write-Host "   downloaded libraries have long file names; if configure fails, move the repo to a short path like C:a."
    }
    $pyFwd = $VenvPy.Replace("\", "/")
    function Invoke-Configure([bool]$cuda) {
        $a = @("-S", $Root, "-B", $BuildDir, "-DPython_EXECUTABLE=$pyFwd", "-DPYTHON_EXECUTABLE=$pyFwd")
        if ($cuda) {
            $a += "-DRA_ENABLE_CUDA=ON"
            $a += "-DRA_CUDA_ARCH=$arch"
        } else {
            $a += "-DRA_ENABLE_CUDA=OFF"
        }
        & $CMake @a | Out-Host
        return ($LASTEXITCODE -eq 0)
    }

    Step "Configure (CMake; the first run downloads Eigen, json, pybind11)"
    $okCfg = Invoke-Configure $useCuda
    if (-not $okCfg -and $useCuda) {
        Warn ("Configure with CUDA failed (often: CUDA Toolkit without its Visual Studio integration).`n" +
              "   Retrying CPU-only.")
        if (Test-Path $BuildDir) { Remove-Item -Recurse -Force $BuildDir }
        $useCuda = $false
        $okCfg = Invoke-Configure $false
    }
    if (-not $okCfg) {
        Fail ("CMake configure failed. Read the messages above.`n" +
              "   Common causes: no internet (Eigen comes from gitlab.com), missing C++ tools.`n" +
              "   After fixing run:  setup.ps1 -Clean")
    }

    Step "Build (Release, a few minutes)"
    & $CMake --build $BuildDir --config Release --parallel | Out-Host
    if ($LASTEXITCODE -ne 0) {
        Fail "Build failed. Read the compiler messages above. To retry from zero run:  setup.ps1 -Clean"
    }

    # a stale GPU module from an earlier GPU build would be picked up by the app
    if (-not $useCuda) {
        foreach ($p in @(Get-ChildItem -Path $Root -Filter "gpu_bridge.*.pyd" -ErrorAction SilentlyContinue)) {
            Remove-Item -Force $p.FullName
        }
    }
    $core = Get-ChildItem -Path $Root -Filter "core_bridge.*.pyd" -ErrorAction SilentlyContinue | Select-Object -First 1
    if (-not $core) { Fail "Build finished but core_bridge.*.pyd is not in the repo root." }
    Ok "core_bridge: $($core.Name)"
    if ($useCuda) {
        $g = Get-ChildItem -Path $Root -Filter "gpu_bridge.*.pyd" -ErrorAction SilentlyContinue | Select-Object -First 1
        if (-not $g) { Fail "Build finished but gpu_bridge.*.pyd is not in the repo root." }
        Ok "gpu_bridge: $($g.Name)"
        $GpuBuilt = $true
    } else {
        Ok "CPU-only build (no gpu_bridge)"
    }
}

# ---------------------------------------------------------------- smoke
Step "Smoke test (python -m app.main --smoke)"
$env:QT_QPA_PLATFORM = "offscreen"
& $VenvPy -m app.main --smoke
$smokeRc = $LASTEXITCODE
Remove-Item Env:\QT_QPA_PLATFORM -ErrorAction SilentlyContinue
if ($smokeRc -ne 0) {
    Fail ("The smoke test failed (exit code $smokeRc). See the line above.`n" +
          "   If you used -SkipBuild, run setup.ps1 without it. Otherwise try:  setup.ps1 -Clean")
}

Write-Host ""
Write-Host "SETUP DONE." -ForegroundColor Green
if ($SkipBuild) { Write-Host "   (C++ build skipped: run setup.ps1 without -SkipBuild before starting the app.)" }
elseif ($GpuBuilt) { Write-Host "   GPU module built: workspace analysis can use the NVIDIA GPU." }
else { Write-Host "   CPU-only: everything works, large workspace runs are slower." }
Write-Host "Next step:  .\run.ps1   (or double-click run.bat)"
if ($Dev) { Write-Host "Tests:      .\run_tests.ps1" }
exit 0
