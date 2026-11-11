# run_tests.ps1
# Robotics Analyzer — combined test runner
# Runs CTest (C++ tests) and pytest (Python tests) and reports combined result.
# Usage: .\run_tests.ps1

$ErrorActionPreference = "Continue"
$ProjectRoot = $PSScriptRoot
$BuildDir = Join-Path $ProjectRoot "build"

Write-Host ""
Write-Host "============================================" -ForegroundColor Cyan
Write-Host "  Robotics Analyzer — Test Suite" -ForegroundColor Cyan
Write-Host "============================================" -ForegroundColor Cyan
Write-Host ""

# --- CTest (C++ tests) ---
Write-Host "--- C++ Tests (CTest) ---" -ForegroundColor Yellow

if (-not (Test-Path $BuildDir)) {
    Write-Host "ERROR: Build directory not found at $BuildDir" -ForegroundColor Red
    Write-Host "Run 'cmake --build build --config Release' first." -ForegroundColor Red
    exit 1
}

$ctestArgs = @(
    "--test-dir", $BuildDir,
    "--build-config", "Release",
    "--output-on-failure"
)

$ctestResult = & ctest @ctestArgs
$ctestExit = $LASTEXITCODE

Write-Host ""

# --- pytest (Python tests) ---
Write-Host "--- Python Tests (pytest) ---" -ForegroundColor Yellow

Set-Location $ProjectRoot
$pytestResult = & python -m pytest tests/python/ -v
$pytestExit = $LASTEXITCODE

Write-Host ""

# --- Summary ---
Write-Host "============================================" -ForegroundColor Cyan
Write-Host "  Test Summary" -ForegroundColor Cyan
Write-Host "============================================" -ForegroundColor Cyan

$allPassed = $true

if ($ctestExit -eq 0) {
    Write-Host "  C++ Tests (CTest):  PASS" -ForegroundColor Green
} else {
    Write-Host "  C++ Tests (CTest):  FAIL (exit code $ctestExit)" -ForegroundColor Red
    $allPassed = $false
}

if ($pytestExit -eq 0) {
    Write-Host "  Python Tests (pytest): PASS" -ForegroundColor Green
} else {
    Write-Host "  Python Tests (pytest): FAIL (exit code $pytestExit)" -ForegroundColor Red
    $allPassed = $false
}

Write-Host ""

if ($allPassed) {
    Write-Host "  ALL TESTS PASSED" -ForegroundColor Green
    Write-Host ""
    exit 0
} else {
    Write-Host "  SOME TESTS FAILED" -ForegroundColor Red
    Write-Host ""
    exit 1
}
