"""Install / run scripts: PowerShell syntax, README pointers, requirements split."""
from __future__ import annotations

import re
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
NL = chr(10)


def _powershell():
    return shutil.which("powershell") or shutil.which("pwsh")


@pytest.mark.parametrize("name", ["setup.ps1", "run.ps1", "install.ps1", "uninstall.ps1"])
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
    for name in ("setup.ps1", "run.ps1", "run.bat", "install.ps1", "install.bat", "uninstall.ps1", "uninstall.bat"):
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


def test_readme_mentions_wizard():
    text = (ROOT / "README.md").read_text(encoding="utf-8")
    assert "install.bat" in text
    assert "Easy install (wizard)" in text
    assert "uninstall.bat" in text


def test_install_has_options():
    text = (ROOT / "install.ps1").read_text(encoding="ascii")
    for flag in ("Yes", "CpuOnly", "NoShortcuts", "DryRun", "SkipLaunch"):
        assert re.search(r"\[switch\]\$" + flag + r"(?![A-Za-z])", text), flag
    assert "$InstallDir" in text and "$RepoUrl" in text
    bat = (ROOT / "install.bat").read_text(encoding="ascii")
    assert "-ExecutionPolicy Bypass" in bat and "pause" in bat


def _run_install(tmp_path, *extra, stdin=None):
    ps = _powershell()
    if ps is None:
        pytest.skip("PowerShell not available")
    target = tmp_path / "inst"
    r = subprocess.run(
        [ps, "-NoProfile", "-ExecutionPolicy", "Bypass", "-File", str(ROOT / "install.ps1"),
         "-DryRun", "-InstallDir", str(target), *extra],
        capture_output=True, text=True, timeout=180, input=stdin)
    return r, target


def test_install_dry_run_yes(tmp_path):
    r, target = _run_install(tmp_path, "-Yes", "-CpuOnly", "-SkipLaunch", "-NoShortcuts")
    out = r.stdout
    assert r.returncode == 0, out + r.stderr
    assert "DRY RUN" in out
    for n in range(1, 9):
        assert f"Step {n}/8" in out
    assert "would run: powershell.exe" in out and "-CpuOnly" in out
    assert "Nothing was installed" in out
    assert not target.exists()  # a dry run creates nothing


def test_install_dry_run_interactive(tmp_path):
    # Enter to begin, folder question answered with Enter is not possible with -InstallDir,
    # so answer the GPU menu (2 = CPU only) and the shortcuts question (n)
    r, target = _run_install(tmp_path, "-SkipLaunch", stdin=NL.join(["", "2", "n", ""]))
    assert r.returncode == 0, r.stdout + r.stderr
    assert "Mode: CPU only" in r.stdout
    assert "No shortcuts" in r.stdout
    assert not target.exists()


def test_install_refuses_unrelated_nonempty_folder(tmp_path):
    ps = _powershell()
    if ps is None:
        pytest.skip("PowerShell not available")
    other = tmp_path / "other"
    other.mkdir()
    (other / "mine.txt").write_text("keep me")
    r = subprocess.run(
        [ps, "-NoProfile", "-ExecutionPolicy", "Bypass", "-File", str(ROOT / "install.ps1"),
         "-DryRun", "-Yes", "-InstallDir", str(other)],
        capture_output=True, text=True, timeout=120)
    assert r.returncode != 0
    assert "not empty" in r.stdout
    assert (other / "mine.txt").read_text() == "keep me"


def test_uninstall_refuses_folder_without_marker(tmp_path):
    ps = _powershell()
    if ps is None:
        pytest.skip("PowerShell not available")
    d = tmp_path / "plain"
    d.mkdir()
    (d / "file.txt").write_text("x")
    r = subprocess.run(
        [ps, "-NoProfile", "-ExecutionPolicy", "Bypass", "-File", str(ROOT / "uninstall.ps1"),
         "-Yes", "-InstallDir", str(d)],
        capture_output=True, text=True, timeout=120)
    assert r.returncode != 0
    assert "Nothing was deleted" in r.stdout
    assert (d / "file.txt").exists()


def test_uninstall_dry_run_with_marker(tmp_path):
    ps = _powershell()
    if ps is None:
        pytest.skip("PowerShell not available")
    d = tmp_path / "inst"
    d.mkdir()
    (d / "a.txt").write_text("x")
    (d / ".robotics-analyzer-install").write_text(
        NL.join(["RoboticsAnalyzer-install-marker", "created_by=install.ps1", "version=1", "mode=fresh", f"dir={d}", "date=now", ""]))
    r = subprocess.run(
        [ps, "-NoProfile", "-ExecutionPolicy", "Bypass", "-File", str(ROOT / "uninstall.ps1"),
         "-Yes", "-DryRun", "-InstallDir", str(d)],
        capture_output=True, text=True, timeout=120)
    assert r.returncode == 0, r.stdout + r.stderr
    assert "would delete" in r.stdout
    assert (d / "a.txt").exists()


# ---------------------------------------------------------------- S3 / S4 (security audit)
def _marker(d, **over):
    fields = {"created_by": "install.ps1", "version": "1", "mode": "fresh", "dir": str(d), "date": "now"}
    for k, v in over.items():
        if v is None:
            fields.pop(k, None)
        else:
            fields[k] = v
    lines = ["RoboticsAnalyzer-install-marker"] + [f"{k}={v}" for k, v in fields.items()] + [""]
    (d / ".robotics-analyzer-install").write_text(NL.join(lines), encoding="ascii")


def _uninstall(d, *extra, dry=True):
    ps = _powershell()
    if ps is None:
        pytest.skip("PowerShell not available")
    return subprocess.run(
        [ps, "-NoProfile", "-ExecutionPolicy", "Bypass", "-File", str(ROOT / "uninstall.ps1"),
         "-Yes", *(["-DryRun"] if dry else []), "-InstallDir", str(d), *extra],
        capture_output=True, text=True, timeout=120)


def _folder(tmp_path, name="inst"):
    d = tmp_path / name
    d.mkdir()
    (d / "keep.txt").write_text("x")
    return d


@pytest.mark.parametrize("over", [
    {"dir": None}, {"mode": None}, {"mode": "weird"}, {"version": None}, {"version": "x"},
    {"created_by": None}, {"created_by": "evil"}, {"dir": "C:\\somewhere\\else"},
])
def test_uninstall_refuses_incomplete_or_wrong_marker(tmp_path, over):
    d = _folder(tmp_path)
    _marker(d, **over)
    r = _uninstall(d)
    assert r.returncode != 0, r.stdout
    assert "Refusing" in r.stdout
    assert "would delete" not in r.stdout
    assert (d / "keep.txt").exists()


def test_uninstall_header_only_marker_refuses(tmp_path):
    d = _folder(tmp_path)
    (d / ".robotics-analyzer-install").write_text("RoboticsAnalyzer-install-marker" + NL)
    r = _uninstall(d)
    assert r.returncode != 0 and "would delete" not in r.stdout
    assert (d / "keep.txt").exists()


def test_uninstall_refuses_percent_in_path(tmp_path):
    d = _folder(tmp_path, "%USERPROFILE%")
    _marker(d)
    r = _uninstall(d)
    assert r.returncode != 0
    assert "Refusing" in r.stdout and "would delete" not in r.stdout
    assert (d / "keep.txt").exists()


def test_uninstall_refuses_junction(tmp_path):
    if not hasattr(subprocess, "run") or not _powershell():
        pytest.skip("PowerShell not available")
    target = _folder(tmp_path, "target")
    _marker(target)
    link = tmp_path / "link"
    r0 = subprocess.run(["cmd", "/c", "mklink", "/J", str(link), str(target)], capture_output=True, text=True)
    if r0.returncode != 0:
        pytest.skip("cannot create a junction: " + r0.stdout + r0.stderr)
    _marker(target, dir=str(link))
    r = _uninstall(link)
    assert r.returncode != 0
    assert "junction" in r.stdout and "would delete" not in r.stdout
    assert (target / "keep.txt").exists()


def test_uninstall_refuses_drive_root_and_profile(tmp_path):
    import os
    root = os.path.splitdrive(str(tmp_path))[0] + "\\"
    for p in (root, os.environ.get("USERPROFILE", root), os.environ.get("SystemRoot", root)):
        r = _uninstall(p)
        assert r.returncode != 0, p
        assert "would delete" not in r.stdout, p


def test_uninstall_refuses_parent_of_profile(tmp_path):
    import os
    prof = os.environ.get("USERPROFILE")
    if not prof:
        pytest.skip("no USERPROFILE")
    r = _uninstall(os.path.dirname(prof))
    assert r.returncode != 0 and "would delete" not in r.stdout


def test_uninstall_valid_fresh_marker_dry_run(tmp_path):
    d = _folder(tmp_path)
    _marker(d)
    r = _uninstall(d)
    assert r.returncode == 0, r.stdout + r.stderr
    assert "would delete" in r.stdout
    assert (d / "keep.txt").exists()


def test_scripts_have_no_cmd_or_expression_eval():
    for name in ("uninstall.ps1", "install.ps1", "setup.ps1", "run.ps1"):
        text = (ROOT / name).read_text(encoding="ascii")
        code = NL.join(ln for ln in text.splitlines() if not ln.strip().startswith("#"))
        assert not re.search(r"cmd(\.exe)?\s+/c", code, re.I), name
        assert not re.search(r"Invoke-Expression|iex", code, re.I), name
        assert "rmdir" not in code.lower(), name


def test_install_never_adopts_mode_from_marker():
    text = (ROOT / "install.ps1").read_text(encoding="ascii")
    assert '$Script:Mode = $mk["mode"]' not in text
    assert '"version=1"' in text


def test_uninstall_real_delete_does_not_follow_inner_junction(tmp_path):
    d = _folder(tmp_path, "inst")
    _marker(d)
    outside = _folder(tmp_path, "outside")
    (outside / "precious.txt").write_text("keep")
    r0 = subprocess.run(["cmd", "/c", "mklink", "/J", str(d / "robots"), str(outside)],
                        capture_output=True, text=True)
    if r0.returncode != 0:
        pytest.skip("cannot create a junction: " + r0.stdout + r0.stderr)
    r = _uninstall(d, dry=False)
    assert r.returncode == 0, r.stdout + r.stderr
    assert not d.exists()
    assert (outside / "precious.txt").read_text() == "keep"
