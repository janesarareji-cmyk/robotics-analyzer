"""U2.6 drag the TCP in 3D: IK on the GUI thread, throttled."""

import os

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtCore import QCoreApplication

import core_bridge as cb
from app.gripper import gripper_dofs
from app.importers.urdf import import_urdf
from app.kin_utils import current_q
from app.main_window import MainWindow
from app.sample_robots import list_sample_robots
from app.settings import AppSettings

SAMPLES = {p.parent.name: (t, p) for t, p in list_sample_robots()}


@pytest.fixture
def win(qapp, monkeypatch):
    monkeypatch.setattr(AppSettings, "_ORGANIZATION", "RoboticsAnalyzer_test_u2d")
    monkeypatch.setattr(AppSettings, "_APPLICATION", "RoboticsAnalyzer_test_u2d")
    w = MainWindow()
    w.settings._settings.clear()
    w.collision_service.enabled = False         # these tests move through poses; U3 has its own
    w.show()
    yield w
    w.project_controller._is_modified = False
    w.close()


def _load(win, name):
    path = SAMPLES[name][1]
    res = import_urdf(str(path))
    win._import_path = str(path)
    win._on_urdf_import_result(res)
    QCoreApplication.processEvents()
    return win.project_controller.robot


def _tcp(model):
    return np.asarray(cb.fk(model)["tcp"])


def _fake_clock():
    t = [0.0]

    def clock():
        return t[0]
    clock.t = t
    return clock


@pytest.mark.parametrize("name", ["ur5", "xarm5"])
def test_drag_reaches_target(win, name):
    model = _load(win, name)
    drag = win.tcp_drag
    clk = _fake_clock()
    drag._clock = clk
    win.drag_tcp_action.setChecked(True)
    assert drag.enabled and drag.widget is not None
    start = _tcp(model)
    target = start[:3, 3] + np.array([0.05, 0.0, 0.0])
    q_before = current_q(model)
    modified_before = win.project_controller.is_modified
    assert drag.on_tcp_drag(target) is True
    assert drag.reachable
    assert np.linalg.norm(_tcp(model)[:3, 3] - target) < 1e-4
    assert current_q(model) != q_before
    if name == "ur5":      # 6 DOF: orientation kept
        assert np.allclose(_tcp(model)[:3, :3], start[:3, :3], atol=1e-3)
    print(f"\n{name}: IK per drag event {drag.last_ik_ms:.2f} ms")
    assert drag.last_ik_ms < 50.0
    drag.on_release()
    assert win.project_controller.is_modified or modified_before
    assert win.project_controller.is_modified
    # handle sits on the TCP again, sliders show the new q
    assert np.allclose(drag.widget.GetCenter(), _tcp(model)[:3, 3], atol=1e-9)


def test_unreachable_keeps_q_and_flags(win):
    model = _load(win, "ur5")
    drag = win.tcp_drag
    drag._clock = _fake_clock()
    win.drag_tcp_action.setChecked(True)
    q_before = current_q(model)
    flags = []
    drag.reachable_changed.connect(flags.append)
    win.project_controller._is_modified = False
    assert drag.on_tcp_drag(np.array([5.0, 0.0, 0.0])) is True
    assert not drag.reachable
    assert flags == [False]
    assert current_q(model) == q_before
    assert "not reachable" in win._status_label.text().lower()
    drag.on_release()
    assert current_q(model) == q_before
    assert not win.project_controller.is_modified      # nothing written: nothing to commit


def test_throttle_drops_events_and_release_solves_last(win):
    model = _load(win, "ur5")
    drag = win.tcp_drag
    clk = _fake_clock()
    drag._clock = clk
    win.drag_tcp_action.setChecked(True)
    p0 = _tcp(model)[:3, 3]
    assert drag.on_tcp_drag(p0 + [0.01, 0, 0]) is True
    clk.t[0] += 0.005
    assert drag.on_tcp_drag(p0 + [0.02, 0, 0]) is False
    clk.t[0] += 0.005
    last = p0 + [0.03, 0, 0]
    assert drag.on_tcp_drag(last) is False
    assert drag.drop_count == 2 and drag.solve_count == 1
    clk.t[0] += 0.05
    assert drag.on_tcp_drag(p0 + [0.04, 0, 0]) is True
    clk.t[0] += 0.001
    last = p0 + [0.06, 0, 0]
    assert drag.on_tcp_drag(last) is False
    drag.on_release()                       # the dropped last target is solved on release
    assert np.linalg.norm(_tcp(model)[:3, 3] - last) < 1e-4


def test_gripper_dofs_unchanged(win):
    model = _load(win, "panda")
    drag = win.tcp_drag
    drag._clock = _fake_clock()
    win.drag_tcp_action.setChecked(True)
    g = gripper_dofs(model)[0]
    q0 = current_q(model)
    target = _tcp(model)[:3, 3] + np.array([0.0, 0.04, 0.0])
    assert drag.on_tcp_drag(target)
    q1 = current_q(model)
    assert q1[g.dof_index] == q0[g.dof_index]
    assert np.linalg.norm(_tcp(model)[:3, 3] - target) < 1e-4
    drag.on_release()


def test_widget_lifecycle(win):
    model = _load(win, "ur5")
    plotter = win.viewport.plotter
    assert not plotter.widgets.sphere_widgets
    win.drag_tcp_action.setChecked(True)
    assert len(plotter.widgets.sphere_widgets) == 1
    # slider move: handle follows the TCP
    q = current_q(model)
    q[0] += 0.4
    from app.kin_utils import write_q
    write_q(model, q)
    win._update_robot_view()
    assert np.allclose(win.tcp_drag.widget.GetCenter(), _tcp(model)[:3, 3], atol=1e-9)
    # robot change: widget recreated, exactly one
    old = win.tcp_drag.widget
    _load(win, "xarm5")
    assert len(plotter.widgets.sphere_widgets) == 1
    assert win.tcp_drag.widget is not old
    assert np.allclose(win.tcp_drag.widget.GetCenter(),
                       _tcp(win.project_controller.robot)[:3, 3], atol=1e-9)
    # off: removed; on again: back
    win.drag_tcp_action.setChecked(False)
    assert not plotter.widgets.sphere_widgets
    win.drag_tcp_action.setChecked(True)
    assert len(plotter.widgets.sphere_widgets) == 1
    # project close: removed (action stays on)
    win.project_controller._is_modified = False
    win.project_controller.close_project(win)
    QCoreApplication.processEvents()
    assert not plotter.widgets.sphere_widgets
    assert win.drag_tcp_action.isChecked()


def test_vtk_event_calls_handler(win):
    """The VTK interaction event reaches on_tcp_drag (what a real mouse drag fires)."""
    model = _load(win, "ur5")
    drag = win.tcp_drag
    drag._clock = _fake_clock()
    win.drag_tcp_action.setChecked(True)
    w = drag.widget
    target = _tcp(model)[:3, 3] + np.array([0.0, 0.0, 0.05])
    w.SetCenter(*target)
    w.InvokeEvent("InteractionEvent")
    assert drag.solve_count == 1
    assert np.linalg.norm(_tcp(model)[:3, 3] - target) < 1e-4
    w.InvokeEvent("EndInteractionEvent")
    assert not drag.dragging
    assert win.project_controller.is_modified


def test_toggle_action_shortcut_and_toolbar(win):
    assert win.drag_tcp_action.isCheckable()
    assert win.drag_tcp_action.shortcut().toString() == "T"
    from PySide6.QtWidgets import QToolBar
    assert any(win.drag_tcp_action in t.actions() for t in win.findChildren(QToolBar))
