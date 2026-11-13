"""U5.9: undo / redo (Ctrl+Z, Ctrl+Y) for joint values, targets and objects."""
import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import numpy as np
import pytest

import app.undo as undo_mod
from app.kin_utils import current_q
from app.main_window import MainWindow
from app.settings import AppSettings
from tests.python.test_live_blocking import load, put_q


@pytest.fixture
def win(qapp, monkeypatch):
    monkeypatch.setattr(AppSettings, "_ORGANIZATION", "RoboticsAnalyzer_test_undo")
    monkeypatch.setattr(AppSettings, "_APPLICATION", "RoboticsAnalyzer_test_undo")
    monkeypatch.setattr(undo_mod, "MERGE_WINDOW_S", 0.0)         # every edit is its own step
    w = MainWindow()
    w.settings._settings.clear()
    w.show()
    yield w
    w.project_controller._is_modified = False
    w.sim_dock.hide()
    w.close()


def test_menu_and_shortcuts(win):
    assert win._action_undo.shortcut().toString() == "Ctrl+Z"
    assert "Ctrl+Y" in [s.toString() for s in win._action_redo.shortcuts()]
    assert not win._action_undo.isEnabled() and not win._action_redo.isEnabled()
    assert "&Edit" in [a.text() for a in win.menuBar().actions()]


def test_undo_redo_joint_values(win):
    model = load(win, "ur5")
    win.collision_action.setChecked(False)
    q0 = current_q(model)
    row = win.joint_panel.rows[1]
    row.spinbox.setValue(row.spinbox.value() + 20.0)
    q1 = current_q(model)
    row.spinbox.setValue(row.spinbox.value() + 20.0)
    q2 = current_q(model)
    assert not np.allclose(q0, q1) and not np.allclose(q1, q2)
    assert win._action_undo.isEnabled() and "Move joints" in win._action_undo.text()
    win._action_undo.trigger()
    assert np.allclose(current_q(model), q1, atol=1e-9)
    assert abs(np.radians(row.spinbox.value()) - model.joints[row.joint_index].q) < 1e-2    # rows follow
    win._action_undo.trigger()
    assert np.allclose(current_q(model), q0, atol=1e-9)
    assert not win._action_undo.isEnabled() and win._action_redo.isEnabled()
    win._action_redo.trigger()
    win._action_redo.trigger()
    assert np.allclose(current_q(model), q2, atol=1e-9)


def test_slider_drag_merges_into_one_step(win, monkeypatch):
    monkeypatch.setattr(undo_mod, "MERGE_WINDOW_S", 5.0)
    model = load(win, "ur5")
    win.collision_action.setChecked(False)
    q0 = current_q(model)
    row = win.joint_panel.rows[0]
    for d in (5, 10, 15, 20):
        row.spinbox.setValue(row.spinbox.value() + d)
    assert win.undo_manager.stack.count() == 1
    win._action_undo.trigger()
    assert np.allclose(current_q(model), q0, atol=1e-9)


def test_jog_and_pose_are_undoable(win):
    model = load(win, "ur5")
    win.collision_action.setChecked(False)
    put_q(win, [0.3, -1.2, 1.0, -1.0, -1.5, 0.2])
    win.undo_manager.reset()
    q0 = current_q(model)
    assert win.jog_panel.step("x", +1)
    q1 = current_q(model)
    assert not np.allclose(q0, q1)
    win._action_undo.trigger()
    assert np.allclose(current_q(model), q0, atol=1e-9)
    win._action_redo.trigger()
    assert np.allclose(current_q(model), q1, atol=1e-9)
    win.joint_panel.go_to_pose("Ready")
    assert win._action_undo.isEnabled()
    win._action_undo.trigger()
    assert np.allclose(current_q(model), q1, atol=1e-9)


def test_undo_target_edits(win):
    load(win, "ur5")
    tp = win.target_panel
    assert tp.add_from_tcp()
    T0 = tp.current_pose().copy()
    tp.auto_solve = False
    tp.move_to(T0[:3, 3] + [0.05, 0.0, 0.0])
    assert abs(tp.current_pose()[0, 3] - (T0[0, 3] + 0.05)) < 1e-9
    assert "Edit target" in win._action_undo.text()
    win._action_undo.trigger()
    assert np.allclose(tp.current_pose(), T0, atol=1e-9)
    win._action_redo.trigger()
    assert abs(tp.current_pose()[0, 3] - (T0[0, 3] + 0.05)) < 1e-9
    win._action_undo.trigger()
    win._action_undo.trigger()                      # undo the add
    assert len(tp.store) == 0


def test_undo_object_edits(win):
    load(win, "ur5")
    cp = win.collision_panel
    assert len(cp.config.environment) == 0
    assert win.add_scene_object("box")
    assert len(cp.config.environment) == 1
    assert "Edit object" in win._action_undo.text()
    win._action_undo.trigger()
    assert len(cp.config.environment) == 0
    win._action_redo.trigger()
    assert len(cp.config.environment) == 1 and cp.config.environment[0].kind == "box"


def test_history_is_cleared_on_new_robot(win):
    load(win, "ur5")
    win.collision_action.setChecked(False)
    row = win.joint_panel.rows[0]
    row.spinbox.setValue(row.spinbox.value() + 10)
    assert win._action_undo.isEnabled()
    win.project_controller._is_modified = False
    load(win, "ur3")
    assert not win._action_undo.isEnabled()
