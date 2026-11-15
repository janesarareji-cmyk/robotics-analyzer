"""Install / run scripts: PowerShell syntax, README pointers, requirements split."""
from __future__ import annotations

import re
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]


def _powershell():
    return shutil.which("powershell") or shutil.which("pwsh")


@pytest.mark.parametrize("name", ["setup.ps1", "run.ps1"])
def test_powershell_syntax(name):
    ps = _powershell()
    if ps is None:
        pytest.skip("PowerShell not available")
    path = str(ROOT / name)
    cmd = (
        "$t=$null;$e=$null;"
        f"[System.Management.Automation.Language.Parser]::ParseFile('{path}',[ref]$t,[ref]$e)|Out-Null;"
        "if($e.Count -gt 0){$e|ForEach-Object{Write-Output $_.Message};exit 1}"
    )
    r = subprocess.run([ps, "-NoProfile", "-Command", cmd], capture_output=True, text=True, timeout=60)
    assert r.returncode == 0, r.stdout + r.stderr


def test_scripts_exist_and_are_ascii():
    for name in ("setup.ps1", "run.ps1", "run.bat"):
        data = (ROOT / name).read_bytes()
        data.decode("ascii")  # PowerShell 5.1 reads BOM-less files as ANSI


def test_setup_has_flags():
    text = (ROOT / "setup.ps1").read_text(encoding="ascii")
    for flag in ("CpuOnly", "SkipBuild", "Dev", "Clean"):
        assert re.search(r"\[switch\]\$" + flag + r"\b", text), flag


def test_readme_mentions_scripts():
    text = (ROOT / "README.md").read_text(encoding="utf-8")
    assert "Install and run" in text
    assert "setup.ps1" in text
    assert "run.ps1" in text
    assert "run.bat" in text


def test_requirements_split():
    runtime = (ROOT / "requirements.txt").read_text(encoding="utf-8").lower()
    assert "pytest" not in runtime
    for pkg in ("numpy", "pyside6", "pyvista", "pillow", "matplotlib", "python-fcl"):
        assert pkg in runtime, pkg
    dev = (ROOT / "requirements-dev.txt").read_text(encoding="utf-8").lower()
    assert "pytest" in dev and "mujoco" in dev


def test_cmake_has_cuda_option():
    text = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
    assert "RA_ENABLE_CUDA" in text
    assert "LANGUAGES CXX CUDA" not in text  # CUDA is enabled conditionally
