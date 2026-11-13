"""U5.6: limit bar per joint row, Home / Ready / saved poses."""
import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import numpy as np
import pytest
from PySide6.QtCore import QCoreApplication

from app.kin_utils import current_q, dof_limits
from app.main_window import MainWindow
from app.settings import AppSettings
from tests.python.test_live_blocking import load, put_q


@pytest.fixture
def win(qapp, monkeypatch):
    monkeypatch.setattr(AppSettings, "_ORGANIZATION", "RoboticsAnalyzer_test_poses")
    monkeypatch.setattr(AppSettings, "_APPLICATION", "RoboticsAnalyzer_test_poses")
    w = MainWindow()
    w.settings._settings.clear()
    w.show()
    yield w
    w.project_controller._is_modified = False
    w.sim_dock.hide()
    w.close()


def test_limit_bar_position_and_red_near_limit(win):
    model = load(win, "ur5")
    row = win.joint_panel.rows[0]
    lo, hi = row.lower_model, row.upper_model
    row.spinbox.setValue(row.lower_disp + 0.5 * (row.upper_disp - row.lower_disp))
    assert abs(row.limit_bar.fraction - 0.5) < 0.01 and not row.limit_bar.near_limit
    row.spinbox.setValue(row.upper_disp - 0.01 * (row.upper_disp - row.lower_disp))
    assert row.limit_bar.fraction > 0.95 and row.limit_bar.near_limit
    row.spinbox.setValue(row.lower_disp)
    assert row.limit_bar.fraction < 1e-9 and row.limit_bar.near_limit
    assert "Range" in row.limit_bar.toolTip()
    # moved from outside (set_value) updates too
    row.set_value(0.5 * (lo + hi))
    assert not row.limit_bar.near_limit


def test_home_ready_and_saved_poses(win):
    model = load(win, "ur5")
    win.collision_action.setChecked(False)
    jp = win.joint_panel
    start = current_q(model)
    assert jp.pose_names()[:2] == ["Home", "Ready"]
    assert jp.pose_combo.itemText(0) == "Go to pose..."
    assert jp.go_to_pose("Ready")
    lo, hi = dof_limits(model)
    assert np.allclose(current_q(model), 0.5 * (np.array(lo) + np.array(hi)), atol=1e-9)
    # home = the start pose of the project
    assert jp.go_to_pose("Home")
    assert np.allclose(current_q(model), start, atol=1e-9)
    # save and delete
    put_q(win, np.array(start) + 0.1)
    assert jp.save_pose("mine") == "mine"
    assert "mine" in jp.pose_names() and "mine" in win.ghost_panel.store
    put_q(win, start)
    assert jp.go_to_pose("mine")
    assert np.allclose(current_q(model), np.array(start) + 0.1, atol=1e-9)
    assert jp.save_pose("Home") is None                      # built-in names are protected
    jp.delete_pose("Home")
    assert "Home" in jp.pose_names()
    jp.delete_pose("mine")
    assert "mine" not in jp.pose_names() and "mine" not in win.ghost_panel.store


def test_pose_combo_applies_and_resets(win):
    model = load(win, "ur3")
    win.collision_action.setChecked(False)
    jp = win.joint_panel
    idx = jp.pose_combo.findText("Ready")
    jp._on_pose_activated(idx)
    assert jp.pose_combo.currentIndex() == 0
    lo, hi = dof_limits(model)
    assert np.allclose(current_q(model), 0.5 * (np.array(lo) + np.array(hi)), atol=1e-9)
    assert win.project_controller.current_project is not None
