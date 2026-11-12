"""R6.6 Graphs dock (offscreen Qt): draw, cursor link, timer, clear."""
import os
import time

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication

from app.graphs.series import KINDS
from app.panels.graph_panel import GraphPanel
from app.panels.sim_panel import SimPanel
from app.project.project_data import SimulationSettings
from app.sim.engine import RUNNING
from app.sim.trajectory import Trajectory, Waypoint

from .robots import random_inertias, random_robot


def _pump(ms):
    end = time.perf_counter() + ms / 1000.0
    while time.perf_counter() < end:
        QApplication.processEvents()
        time.sleep(0.002)


@pytest.fixture
def rig(qapp):
    model = random_robot(3, 2)
    random_inertias(model, np.random.default_rng(3))
    sp = SimPanel()
    sp.show()
    sp.set_model(model, SimulationSettings(physics_timestep=0.002))
    tr = Trajectory([Waypoint(0.0, [0.0, 0.0]), Waypoint(0.5, [0.4, -0.2]),
                     Waypoint(1.0, [0.8, 0.3])], "cubic")
    sp.set_trajectory(tr)
    gp = GraphPanel()
    gp.set_source(lambda: sp.recorder, lambda: sp.dof_names)
    sp.model_replaced.connect(gp.model_changed)
    sp.position_changed.connect(gp.set_cursor)
    gp.seek_requested.connect(sp.seek_trajectory)
    gp.show()
    gp.resize(600, 400)
    yield sp, gp, tr
    sp.shutdown()
    gp.hide()
    sp.hide()


def _run(sp, ms=1200):
    sp.timeline._speed.setValue(4.0)
    sp.play_trajectory()
    _pump(ms)


def test_all_kinds_draw_after_a_run(rig):
    sp, gp, _ = rig
    _run(sp)
    assert len(sp.recorder) > 5
    paths = []
    gp.path_3d_changed.connect(paths.append)
    for kind in KINDS:
        gp.set_kind(kind)
        assert gp.last_error is None, (kind, gp.last_error)
        assert gp.plot.grab_ok()
        if kind == "tcp_path_3d":
            assert gp.plot.curve_count == 0
            assert paths[-1] is not None and paths[-1].shape[1] == 3
        else:
            assert gp.plot.curve_count > 0
    gp.set_kind("joint_pos")
    assert gp.plot.legend_labels() == [f"{n} [rad|m]" for n in sp.dof_names]
    assert max(gp.plot.point_counts()) <= 5000


def test_joint_checklist_filters_curves(rig):
    sp, gp, _ = rig
    _run(sp, 600)
    gp.set_kind("joint_pos")
    assert gp.plot.curve_count == 2
    gp.set_joint_checked(1, False)
    assert gp.selected_joints() == [0] and gp.plot.curve_count == 1
    gp.set_kind("power")                                  # selected joint + sum
    assert gp.plot.curve_count == 2
    gp.set_kind("tcp_xyz")
    assert gp.plot.curve_count == 3


def test_data_survives_stop_and_graph_still_drawn(rig):
    sp, gp, _ = rig
    _run(sp, 800)
    n = len(sp.recorder)
    sp.stop_trajectory()
    assert len(sp.recorder) == n and n > 0
    gp.set_kind("joint_vel")
    assert gp.plot.curve_count == 2 and gp.plot.point_counts()[0] == n


def test_cursor_to_timeline_and_back(rig):
    sp, gp, tr = rig
    gp.set_kind("joint_pos")
    sp.seek_trajectory(0.0)
    gp.plot.cursor.setValue(0.5)                          # user drags the cursor
    assert np.allclose(sp.engine.state.q, tr.q(0.5), atol=1e-12)
    assert sp.timeline._slider.value() == 500
    sp.seek_trajectory(0.25)                              # timeline seeks
    assert gp.plot.cursor_time() == pytest.approx(0.25)
    assert np.allclose(sp.engine.state.q, tr.q(0.25), atol=1e-12)   # no feedback loop


def test_timer_runs_only_while_running_and_visible(rig):
    sp, gp, _ = rig
    assert not gp.timer.isActive()
    sp.timeline._speed.setValue(0.1)
    sp.play_trajectory()
    assert sp.engine.clock.state == RUNNING and gp.timer.isActive()
    n0 = gp.refresh_count
    _pump(350)
    assert gp.refresh_count > n0
    gp.on_visibility_changed(False)
    assert not gp.timer.isActive()
    gp.on_visibility_changed(True)
    assert gp.timer.isActive()
    gp.hide()
    assert not gp.timer.isActive()
    gp.show()
    sp.pause_trajectory()
    assert not gp.timer.isActive()


def test_pause_freezes_view(rig):
    sp, gp, _ = rig
    gp.set_kind("joint_pos")
    sp.timeline._speed.setValue(0.1)
    sp.play_trajectory()
    _pump(250)
    gp._pause.setChecked(True)
    c = gp.refresh_count
    _pump(300)
    assert gp.refresh_count == c
    gp._pause.setChecked(False)
    assert gp.refresh_count > c
    sp.stop_trajectory()


def test_clear_empties_plot_and_recorder(rig):
    sp, gp, _ = rig
    _run(sp, 600)
    gp.set_kind("joint_pos")
    assert gp.plot.curve_count > 0 and len(sp.recorder) > 0
    gp._clear_btn.click()
    assert gp.plot.curve_count == 0 and len(sp.recorder) == 0
    gp.refresh(force=True)
    assert gp.plot.point_counts() == [0, 0]


def test_no_robot_is_safe(qapp):
    gp = GraphPanel()
    gp.set_source(lambda: None, lambda: [])
    gp.show()
    for k in KINDS:
        gp.set_kind(k)
    assert gp.plot.curve_count == 0


def test_main_window_graphs_dock_hidden_and_wired(qapp):
    from app.main_window import MainWindow
    mw = MainWindow()
    assert mw.graph_dock.isHidden() and not mw.graph_panel.timer.isActive()
    mw._action_view_graphs.setChecked(True)
    assert not mw.graph_dock.isHidden()
    mw._action_view_graphs.setChecked(False)
    assert mw.graph_dock.isHidden()
    mw.close()


def test_path_3d_goes_to_viewport_polyline(qapp):
    from app.main_window import MainWindow
    mw = MainWindow()
    rv = mw._viewport.robot_view
    pts = np.column_stack([np.linspace(0, 1, 20), np.zeros(20), np.zeros(20)])
    mw.graph_panel.path_3d_changed.emit(pts)
    assert rv.tcp_path_actor is not None and np.allclose(rv.tcp_path_points, pts)
    mw.graph_panel.path_3d_changed.emit(None)
    assert rv.tcp_path_actor is None
    mw.close()
