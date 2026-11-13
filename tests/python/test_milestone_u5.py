"""U5 milestone: modes, start screen flow, jog, undo, version, usability numbers."""
import json
import os
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import numpy as np
import pytest
from PySide6.QtCore import QCoreApplication, Qt
from PySide6.QtTest import QTest

import app
import core_bridge as cb
from app.kin_utils import current_q
from app.main_window import MainWindow
from app.modes import MODE_NAMES, MODES
from app.settings import AppSettings
from tools import ux_tasks

ROOT = Path(__file__).resolve().parents[2]


@pytest.fixture
def win(qapp, monkeypatch):
    monkeypatch.setattr(AppSettings, "_ORGANIZATION", "RoboticsAnalyzer_test_ms_u5")
    monkeypatch.setattr(AppSettings, "_APPLICATION", "RoboticsAnalyzer_test_ms_u5")
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


def _managed(win):
    names = {n for _m, (_t, ds) in MODES.items() for n in ds}
    out = {}
    for n in names:
        action = getattr(win, "_action_view_" + n)
        out[n] = next(d for a, d in win._view_action_docks() if a is action)
    return out


def test_u5_milestone_flow(win):
    # version in one place
    assert app.__version__ == "1.1.0" and app.__version__ in win.windowTitle()

    # start screen -> one click loads a robot, the view buttons appear, the screen goes away
    assert win.start_screen.isVisible() and len(win.start_screen.card_stems) == 9
    assert not win.view_buttons.isVisible()
    QTest.mouseClick(win.start_screen.card("ur5"), Qt.MouseButton.LeftButton)
    assert win.task_runner.wait_all(30000)
    QCoreApplication.processEvents()
    model = win.project_controller.robot
    assert model is not None and not win.start_screen.isVisible() and win.view_buttons.isVisible()
    win.collision_action.setChecked(False)

    # every mode shows exactly its docks
    docks = _managed(win)
    for mode in MODE_NAMES:
        QTest.mouseClick(win.mode_bar.button(mode), Qt.MouseButton.LeftButton)
        QCoreApplication.processEvents()
        shown = {n for n, d in docks.items() if not d.isHidden()}
        assert shown == set(MODES[mode][1]), mode
        assert win.mode == mode
    win.set_mode("Move")

    # Cartesian jog: tool moves 10 mm along base x, orientation kept
    T0 = np.asarray(cb.fk(model)["tcp"], float)
    q0 = current_q(model)
    win.undo_manager.reset()
    win.jog_panel.set_steps(lin_mm=10, frame="base")
    QTest.mouseClick(win.jog_panel.buttons[("x", 1)], Qt.MouseButton.LeftButton)
    T1 = np.asarray(cb.fk(model)["tcp"], float)
    assert np.linalg.norm((T1[:3, 3] - T0[:3, 3]) - [0.010, 0, 0]) < 2e-4
    assert np.linalg.norm(T1[:3, :3] - T0[:3, :3]) < 5e-3

    # undo brings the joints back, redo repeats
    win._action_undo.trigger()
    assert np.allclose(current_q(model), q0, atol=1e-9)
    win._action_redo.trigger()
    assert np.allclose(np.asarray(cb.fk(model)["tcp"])[:3, 3], T1[:3, 3], atol=1e-9)

    # poses and limit bar
    assert win.joint_panel.pose_names()[:2] == ["Home", "Ready"]
    assert all(0.0 <= r.limit_bar.fraction <= 1.0 for r in win.joint_panel.rows)

    # help
    win.set_mode("Plan")
    win._action_help.trigger()
    assert "The Plan mode" in win._help_dialog.text
    win._help_dialog.close()

    # closing the project brings the start screen back
    win.project_controller._is_modified = False
    win.project_controller.close_project(win)
    QCoreApplication.processEvents()
    assert win.start_screen.isVisible() and not win.view_buttons.isVisible()


def test_usability_numbers_not_worse_than_baseline(win):
    base = json.loads((ROOT / "docs" / "ux" / "baseline.json").read_text(encoding="utf-8"))
    res = ux_tasks.run_tasks(win, "milestone")
    for name, t in res["tasks"].items():
        assert t["actions"] <= base["tasks"][name]["actions"], (name, t["actions"])
    assert res["task1_plus_3"] <= 4, res["task1_plus_3"]
    assert res["tasks"]["1_load_robot"]["actions"] == 1
