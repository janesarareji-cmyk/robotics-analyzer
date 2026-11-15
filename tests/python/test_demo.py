"""D1.2 / D1.3: sample targets loaded with the sample robot, demo tour."""
import os
import time

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
pytest.importorskip("fcl")

from PySide6.QtCore import QCoreApplication
from PySide6.QtTest import QTest

from app.main_window import MainWindow
from app.sample_targets import sample_kinds
from app.settings import AppSettings
from tests.python.test_live_blocking import SAMPLES, load as _load


def load(win, name):
    win._import_sample = True        # as if opened from the start screen / sample menu
    return _load(win, name)


@pytest.fixture
def win(qapp, monkeypatch):
    monkeypatch.setattr(AppSettings, "_ORGANIZATION", "RoboticsAnalyzer_test_d1")
    monkeypatch.setattr(AppSettings, "_APPLICATION", "RoboticsAnalyzer_test_d1")
    w = MainWindow()
    w.settings._settings.clear()
    w.collision_action.setChecked(True)
    w.show()
    yield w
    w.project_controller._is_modified = False
    w.sim_dock.hide()
    w.close()


def pump_until(fn, timeout=60.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        QCoreApplication.processEvents()
        if fn():
            return True
        time.sleep(0.005)
    return False


def test_sample_targets_loaded_with_robot(win):
    load(win, "ur5")
    tp = win.target_panel if hasattr(win, "target_panel") else win._target_panel
    assert tp.store.names() == ["Pick A", "Too far", "Place B", "Home-side C", "Too close"]
    assert win.project_controller.current_project.targets is tp.store
    assert win._toast.isVisible() and "Run demo" in win._toast.action_button.text()
    assert pump_until(lambda: len(tp.last_classification) == 5 and tp.markers.status_of("Too far") == "bad")
    kinds = sample_kinds(SAMPLES["ur5"][1])
    for r in tp.last_classification:
        assert (r["status"] != "bad") == (kinds[r["name"]] == "reachable"), r
    assert tp.markers.status_of("Pick A") in ("ok", "warn")


def test_plain_import_has_no_sample_targets(win):
    from tests.python.test_live_blocking import load as plain
    plain(win, "ur3")
    assert len(win.target_panel.store) == 0


def test_sample_card_loads_targets(win):
    from PySide6.QtCore import Qt
    QTest.mouseClick(win.start_screen.card("ur3"), Qt.MouseButton.LeftButton)
    assert win.task_runner.wait_all(30000)
    QCoreApplication.processEvents()
    assert len(win.target_panel.store) == 5
    win.project_controller._is_modified = False
    win.project_controller.close_project(win)
    QCoreApplication.processEvents()
    assert len(win.target_panel.store) == 0
