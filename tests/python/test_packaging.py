"""R8.5 packaging tests: spec parses, hidden imports cover the third-party imports, --smoke works."""
import ast
import os
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
SPEC = ROOT / "packaging" / "analyzer.spec"
DIST = ROOT / "dist" / "RoboticsAnalyzer"


def _spec_tree():
    return ast.parse(SPEC.read_text(encoding="utf-8"))


def _spec_literal(name):
    for node in _spec_tree().body:
        if isinstance(node, ast.Assign) and getattr(node.targets[0], "id", "") == name:
            return ast.literal_eval(node.value)
    raise AssertionError(f"{name} not in spec")


def test_spec_parses_and_has_parts():
    src = SPEC.read_text(encoding="utf-8")
    _spec_tree()
    for part in ("Analysis(", "EXE(", "COLLECT(", "core_bridge", "gpu_bridge", "examples"):
        assert part in src
    for f in ("build.ps1", "install.ps1"):
        assert (ROOT / "packaging" / f).is_file()


def test_hidden_imports_cover_third_party():
    listed = {h.split(".")[0] for h in _spec_literal("HIDDEN_IMPORTS")}
    stdlib = set(sys.stdlib_module_names)
    found = set()
    for pkg in ("app", "robotics_api"):
        for f in (ROOT / pkg).rglob("*.py"):
            for node in ast.walk(ast.parse(f.read_text(encoding="utf-8", errors="replace"))):
                if isinstance(node, ast.Import):
                    found.update(a.name.split(".")[0] for a in node.names)
                elif isinstance(node, ast.ImportFrom) and node.level == 0 and node.module:
                    found.add(node.module.split(".")[0])
    third = {m for m in found if m not in stdlib and m not in ("app", "robotics_api", "__future__")}
    # PySide6 is found by PyInstaller's own hook.
    missing = sorted(m for m in third - listed if m != "PySide6")
    assert not missing, f"add to HIDDEN_IMPORTS: {missing}"


def test_dist_ignored_by_git():
    assert "dist/" in (ROOT / ".gitignore").read_text()


def test_smoke_flag_in_source_tree():
    env = dict(os.environ, QT_QPA_PLATFORM="offscreen", ANALYZER_FORCE_CPU="1")
    r = subprocess.run([sys.executable, "-m", "app.main", "--smoke"], cwd=str(ROOT), env=env,
                       capture_output=True, text=True, timeout=180)
    assert r.returncode == 0, r.stdout + r.stderr
    assert "SMOKE OK" in r.stdout and "backend=cpu" in r.stdout and "fk_tcp=[2.0" in r.stdout
    assert len(r.stdout.strip().splitlines()) == 1


@pytest.mark.skipif(os.environ.get("ANALYZER_TEST_DIST") != "1" or not (DIST / "RoboticsAnalyzer.exe").exists(),
                    reason="opt-in: set ANALYZER_TEST_DIST=1 and build dist/ with packaging/build.ps1")
@pytest.mark.parametrize("cpu", [False, True])
def test_built_exe_smoke_scrubbed_env(cpu):
    sys.path.insert(0, str(ROOT / "packaging"))
    from scrub_smoke import run_smoke
    r = run_smoke(DIST, cpu)
    assert r.returncode == 0, r.stdout + r.stderr
    assert "SMOKE OK" in r.stdout
    if cpu:
        assert "backend=cpu" in r.stdout
