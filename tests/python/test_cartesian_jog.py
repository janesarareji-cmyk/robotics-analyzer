"""U5.7: Cartesian jog (X/Y/Z, Rx/Ry/Rz), base / tool frame, steps, collision blocking."""
import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import numpy as np
import pytest
from PySide6.QtCore import Qt
from PySide6.QtTest import QTest

import core_bridge as cb
from app.cartesian_jog import jogged_pose, rot_axis
from app.kin_utils import current_q
from app.main_window import MainWindow
from app.settings import AppSettings
from tests.python.test_live_blocking import load, put_q
from tests.python.test_curve_editor import UR5_Q


@pytest.fixture
def win(qapp, monkeypatch):
    monkeypatch.setattr(AppSettings, "_ORGANIZATION", "RoboticsAnalyzer_test_jog")
    monkeypatch.setattr(AppSettings, "_APPLICATION", "RoboticsAnalyzer_test_jog")
    w = MainWindow()
    w.settings._settings.clear()
    w.show()
    yield w
    w.project_controller._is_modified = False
    w.sim_dock.hide()
    w.close()


def tcp(model):
    return np.asarray(cb.fk(model)["tcp"], float)


def test_jogged_pose_math():
    T = np.eye(4)
    T[:3, :3] = rot_axis(2, np.pi / 2)             # tool x points along base y
    T[:3, 3] = [0.1, 0.2, 0.3]
    b = jogged_pose(T, "x", +1, 0.01, 0.1, "base")
    assert np.allclose(b[:3, 3] - T[:3, 3], [0.01, 0, 0])
    t = jogged_pose(T, "x", +1, 0.01, 0.1, "tool")
    assert np.allclose(t[:3, 3] - T[:3, 3], [0, 0.01, 0], atol=1e-12)
    r = jogged_pose(T, "rz", -1, 0.01, 0.1, "base")
    assert np.allclose(r[:3, :3], rot_axis(2, -0.1) @ T[:3, :3])
    r = jogged_pose(T, "rx", +1, 0.01, 0.1, "tool")
    assert np.allclose(r[:3, :3], T[:3, :3] @ rot_axis(0, 0.1))


@pytest.mark.parametrize("axis,sign,frame", [("x", 1, "base"), ("y", -1, "base"), ("z", 1, "base"),
                                              ("z", -1, "tool"), ("x", 1, "tool")])
def test_linear_jog_moves_tool_by_the_step(win, axis, sign, frame):
    model = load(win, "ur5")
    win.collision_action.setChecked(False)
    put_q(win, UR5_Q)
    jp = win.jog_panel
    jp.set_steps(lin_mm=10, ang_deg=5, frame=frame)
    T0 = tcp(model)
    assert jp.step(axis, sign)
    T1 = tcp(model)
    k = "xyz".index(axis)
    d = np.zeros(3)
    d[k] = sign * 0.010
    want = T0[:3, 3] + (T0[:3, :3] @ d if frame == "tool" else d)
    assert np.linalg.norm(T1[:3, 3] - want) < 2e-4
    assert np.linalg.norm(T1[:3, :3] - T0[:3, :3]) < 5e-3           # orientation kept


@pytest.mark.parametrize("axis,frame", [("rz", "base"), ("rx", "tool"), ("ry", "base")])
def test_rotation_jog_turns_tool(win, axis, frame):
    model = load(win, "ur5")
    win.collision_action.setChecked(False)
    put_q(win, UR5_Q)
    jp = win.jog_panel
    jp.set_steps(ang_deg=5, frame=frame)
    T0 = tcp(model)
    assert jp.step(axis, +1)
    T1 = tcp(model)
    want = jogged_pose(T0, axis, +1, 0.01, np.radians(5), frame)
    assert np.linalg.norm(T1[:3, :3] - want[:3, :3]) < 5e-3
    assert np.linalg.norm(T1[:3, 3] - T0[:3, 3]) < 2e-4             # position kept


def test_buttons_update_joint_rows_and_auto_repeat(win):
    model = load(win, "ur5")
    win.collision_action.setChecked(False)
    put_q(win, UR5_Q)
    jp = win.jog_panel
    b = jp.buttons[("x", 1)]
    assert b.autoRepeat() and b.autoRepeatDelay() > 0 and b.minimumHeight() >= 28
    q0 = current_q(model)
    x0 = tcp(model)[0, 3]
    QTest.mouseClick(b, Qt.MouseButton.LeftButton)
    assert tcp(model)[0, 3] > x0 + 0.009
    row = win.joint_panel.rows[0]
    assert abs(np.radians(row.spinbox.value()) - model.joints[row.joint_index].q) < 1e-2   # rows follow
    assert current_q(model) != q0


def test_jog_blocked_by_object_and_unreachable(win):
    model = load(win, "ur5")
    put_q(win, UR5_Q)
    win.collision_action.setChecked(True)
    p = tcp(model)[:3, 3]
    win.collision_panel.add_shape("box", {"size": [0.2, 0.2, 0.2]}, list(p + [0.14, 0, 0]), name="wall")
    win._sync_env_objects()
    win.collision_service.invalidate()
    jp = win.jog_panel
    jp.set_steps(lin_mm=50)
    moved = 0
    for _ in range(12):
        if jp.step("x", +1):
            moved += 1
    assert tcp(model)[0, 3] < p[0] + 0.14            # never into the box
    assert jp.message or win.jog_panel.jog.last_status
    # far out of reach: fails with a plain message
    win.collision_action.setChecked(False)
    win.collision_panel.remove_shape(0)
    win._sync_env_objects()
    n = 0
    while jp.step("z", +1) and n < 60:
        n += 1
    assert n < 60 and "cannot go further" in jp.message
