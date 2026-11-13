"""A1.8: the process must exit cleanly (code 0) after Qt tests and after closing a MainWindow."""
import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
ENV = dict(os.environ, QT_QPA_PLATFORM="offscreen")


def _run(args, timeout=600):
    return subprocess.run([sys.executable, "-X", "faulthandler", *args], cwd=ROOT, env=ENV,
                          capture_output=True, text=True, timeout=timeout)


def test_ui_ghosts_subprocess_exits_zero_three_times():
    # test_ui_ghosts.py never imports this file, so there is no recursion.
    for i in range(3):
        r = _run(["-m", "pytest", "tests/python/test_ui_ghosts.py", "-q", "-p", "no:cacheprovider"])
        assert r.returncode == 0, f"run {i}: exit {r.returncode}\n{r.stdout[-1500:]}\n{r.stderr[-1500:]}"


def test_main_window_close_script_exits_zero():
    r = _run(["tests/python/_exit_script.py"], timeout=120)
    assert r.returncode == 0, f"exit {r.returncode}\n{r.stdout[-1500:]}\n{r.stderr[-1500:]}"
