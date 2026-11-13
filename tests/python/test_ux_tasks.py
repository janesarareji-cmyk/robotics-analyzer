"""U5.1: the usability script runs offscreen and its counts are stable."""
import os
import sys
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from app.main_window import MainWindow          # noqa: E402
from app.settings import AppSettings            # noqa: E402
from tools import ux_tasks                      # noqa: E402


@pytest.fixture
def win(qapp, monkeypatch):
    monkeypatch.setattr(AppSettings, "_ORGANIZATION", "RoboticsAnalyzer_test_ux")
    monkeypatch.setattr(AppSettings, "_APPLICATION", "RoboticsAnalyzer_test_ux")
    AppSettings()._settings.clear()          # no saved layout from an earlier run
    w = MainWindow()
    w.settings._settings.clear()
    w.resize(1600, 950)
    w.show()
    ux_tasks.pump(0.5)
    yield w
    w.project_controller._is_modified = False
    w.sim_dock.hide()
    w.close()


def test_tasks_run_and_counts_are_stable(win):
    res = ux_tasks.run_tasks(win, "test")
    names = list(res["tasks"])
    assert names == [n for n, _ in ux_tasks.TASKS] and len(names) == 6
    assert all(t["actions"] >= 1 for t in res["tasks"].values())
    assert res["tasks"]["2_jog_joint"]["actions"] == 1
    assert res["task1_plus_3"] <= 8
    assert win.compare_panel.results, "compare produced a table"
