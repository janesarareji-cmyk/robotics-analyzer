"""R6.4 trajectory UI: waypoint table, playback in the Simulation dock, project wiring (offscreen Qt)."""
import os
import time
import unittest.mock as mock
from pathlib import Path

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtCore import QSettings
from PySide6.QtWidgets import QApplication

import core_bridge as cb
from app.kin_utils import current_q, write_q
from app.main_window import MainWindow
from app.panels.sim_panel import SimPanel
from app.panels.timeline_panel import TimelinePanel
from app.project.project_data import SimulationSettings
from app.project.serializer import save
from app.settings import AppSettings
from app.sim import DYNAMIC, KINEMATIC
from app.sim.engine import PAUSED, RUNNING, STOPPED
from app.sim.trajectory import Trajectory, Waypoint

from .robots import random_inertias, random_robot

FX = Path(__file__).parent / "fixtures" / "urdf"
TEST_ORG = "RoboticsAnalyzer_test_r64ui"


def _model(seed=3, n=2):
    m = random_robot(seed, n)
    random_inertias(m, np.random.default_rng(seed))
    return m


def _pump(ms):
    end = time.perf_counter() + ms / 1000.0
    while time.perf_counter() < end:
        QApplication.processEvents()
        time.sleep(0.002)


def _set(tl, r, c, text):
    tl.table.item(r, c).setText(text)


# ------------------------------------------------------------- timeline widget
@pytest.fixture
def tl(qapp):
    w = TimelinePanel()
    w.show()
    w.set_model(2, ["a", "b"], ([-3.0, -3.0], [3.0, 3.0]))
    return w


def test_table_add_from_current_q_and_trajectory_signal(tl):
    q = [0.5, -0.25]
    tl.set_q_provider(lambda: q)
    got = []
    tl.trajectory_changed.connect(got.append)
    tl.add_waypoint()                      # 1 row: not yet a trajectory
    assert tl.table.rowCount() == 1 and tl.trajectory is None and not got
    q[:] = [1.0, 0.0]
    tl.add_waypoint()
    assert tl.table.rowCount() == 2
    assert tl.playable is not None and len(got) == 1
    tr = got[0]
    assert [w.t for w in tr.waypoints] == [0.0, 1.0]
    assert np.allclose(tr.q(0.0), [0.5, -0.25]) and np.allclose(tr.q(1.0), [1.0, 0.0])
    assert tl.table.horizontalHeaderItem(1).text() == "a"


def test_invalid_rows_are_marked_not_dropped(tl):
    tl.set_q_provider(lambda: [0.0, 0.0])
    for _ in range(3):
        tl.add_waypoint()
    ok = tl.trajectory
    assert ok is not None and tl.playable is not None
    _set(tl, 2, 0, "0.5")                        # t not increasing
    assert tl.table.rowCount() == 3
    assert tl.playable is None and "strictly increasing" in tl.info_text
    assert tl.table.item(2, 0).background().color().red() > 150
    assert "strictly" in tl.table.item(2, 0).toolTip()
    assert tl.trajectory == ok                   # last valid kept for saving
    _set(tl, 1, 1, "abc")                        # not a number
    assert tl.table.item(1, 1).text() == "abc" and 1 in {int(s.split(":")[0]) for s in
                                                          tl.info_text.split("Invalid rows: ")[1].split("; ")}
    _set(tl, 1, 1, "0.0")
    _set(tl, 2, 0, "2.0")
    assert tl.playable is not None and tl.info_text == ""


def test_limit_violation_is_warning_only(tl):
    tl.set_q_provider(lambda: [0.0, 0.0])
    tl.add_waypoint()
    tl.add_waypoint()
    _set(tl, 1, 2, "3.5")                        # limit 3.0
    assert tl.playable is not None
    assert "limit warning" in tl.info_text
    assert tl.table.item(1, 2).background().color().red() > 200


def test_delete_move_capture(tl):
    qs = [[0.1, 0.1], [0.2, 0.2], [0.3, 0.3]]
    it = iter(qs)
    tl.set_q_provider(lambda: next(it))
    for _ in range(3):
        tl.add_waypoint()
    tl.table.selectRow(0)
    tl.move_selected(1)                          # row 0 -> 1, whole row
    assert [tl.table.item(r, 1).text() for r in range(3)] == ["0.2", "0.1", "0.3"]
    assert tl.playable is None                   # t order broken -> marked, not dropped
    tl.move_selected(-1)
    assert [tl.table.item(r, 1).text() for r in range(3)] == ["0.1", "0.2", "0.3"]
    assert tl.playable is not None
    tl.set_q_provider(lambda: [9.0, 8.0])
    tl.table.selectRow(1)
    tl.capture_current_pose()
    assert tl.table.item(1, 0).text() == "1.0" and tl.table.item(1, 1).text() == "9.0"
    tl.delete_selected()
    assert tl.table.rowCount() == 2 and tl.playable is not None


def test_deleting_all_rows_clears_trajectory(tl):
    tl.set_q_provider(lambda: [0.0, 0.0])
    tl.add_waypoint()
    tl.add_waypoint()
    got = []
    tl.trajectory_changed.connect(got.append)
    tl.table.selectAll()
    tl.delete_selected()
    assert tl.trajectory is None and got == [None]


def test_set_trajectory_silent_and_dof_mismatch_dropped(tl):
    got, logs = [], []
    tl.trajectory_changed.connect(got.append)
    tl.log_requested.connect(logs.append)
    tr = Trajectory([Waypoint(0.0, [0.0, 0.1]), Waypoint(1.5, [0.2, 0.3])], "cubic", False)
    assert tl.set_trajectory(tr) and not got
    assert tl.table.rowCount() == 2 and tl.trajectory == tr
    assert tl._interp.currentText() == "cubic" and not tl._clamped.isChecked()
    bad = Trajectory([Waypoint(0.0, [0.0]), Waypoint(1.0, [1.0])])
    assert tl.set_trajectory(bad) is False and any("dropped" in m for m in logs)


def test_model_n_dof_change_drops_trajectory(tl):
    tl.set_trajectory(Trajectory([Waypoint(0.0, [0, 0]), Waypoint(1.0, [1, 1])]))
    got, logs = [], []
    tl.trajectory_changed.connect(got.append)
    tl.log_requested.connect(logs.append)
    tl.set_model(2, ["a", "b"], ([-3, -3], [3, 3]))      # same DOF: kept
    assert tl.trajectory is not None and not got
    tl.set_model(3, None, ([-3] * 3, [3] * 3))
    assert tl.trajectory is None and got == [None] and tl.table.rowCount() == 0
    assert any("WARNING" in m for m in logs) and tl.table.columnCount() == 4


# --------------------------------------------------------- playback in SimPanel
@pytest.fixture
def panel(qapp):
    p = SimPanel()
    p.show()
    yield p
    p.shutdown()
    p.hide()


def _loaded(panel, dt=0.002):
    model = _model()
    panel.set_model(model, SimulationSettings(physics_timestep=dt))
    tr = Trajectory([Waypoint(0.0, [0.0, 0.0]), Waypoint(0.5, [0.4, -0.2]),
                     Waypoint(1.0, [0.8, 0.3])], "cubic")
    assert panel.set_trajectory(tr)
    return model, tr


def test_kinematic_playback_via_timer_ends_on_last_waypoint(panel):
    model, tr = _loaded(panel)
    assert panel.mode == KINEMATIC
    assert panel.timeline.transport_enabled() == dict(play=True, pause=False, stop=False)
    assert not any(panel.button_enabled().values())          # run buttons stay off
    states = []
    panel.state_updated.connect(lambda s: states.append(s.q.copy()))
    panel.timeline.speed_changed.emit(4.0)
    panel.timeline._speed.setValue(4.0)
    panel.play_trajectory()
    assert panel.engine.clock.state == RUNNING and panel.timer.isActive()
    _pump(1200)
    assert panel.engine.clock.state == PAUSED
    assert np.allclose(panel.engine.state.q, [0.8, 0.3], atol=1e-12)
    assert len(states) > 3
    assert panel.timeline._slider.value() == 1000
    assert "traj t=" in panel.status_text
    panel.stop_trajectory()
    assert panel.engine.clock.state == STOPPED and panel.player is None


def test_pause_resume_seek_kinematic(panel):
    model, tr = _loaded(panel)
    panel.play_trajectory()
    _pump(50)
    panel.pause_trajectory()
    assert panel.engine.clock.state == PAUSED and not panel.timer.isActive()
    assert panel.timeline.transport_enabled()["play"]
    panel.timeline._slider.setValue(500)                      # user seek to t = 0.5
    assert np.allclose(panel.engine.state.q, tr.q(0.5), atol=1e-12)
    panel.play_trajectory()                                    # resume
    assert panel.engine.clock.state == RUNNING
    panel.stop_trajectory()


def test_seek_while_stopped_shows_exact_q(panel):
    model, tr = _loaded(panel)
    panel.seek_trajectory(0.25)
    assert panel.engine.clock.state == STOPPED
    assert np.allclose(panel.engine.state.q, tr.q(0.25), atol=1e-14)


def test_dynamic_playback_shows_tracking_error(panel):
    model, tr = _loaded(panel)
    panel.set_mode(DYNAMIC)
    panel.play_trajectory()
    _pump(300)
    assert "track err" in panel.status_text
    assert panel.engine.torque_fn is not None
    panel.stop_trajectory()
    assert panel.engine.torque_fn is None and panel.engine.state.t == 0.0


def test_editing_table_during_playback_discards_player(panel):
    model, tr = _loaded(panel)
    panel.play_trajectory()
    assert panel.player is not None
    got = []
    panel.trajectory_changed.connect(got.append)
    _set(panel.timeline, 2, 1, "0.9")
    assert panel.player is None and got and got[-1].q(1.0)[0] == pytest.approx(0.9)


def test_model_dof_change_drops_trajectory_in_panel(panel):
    _loaded(panel)
    got, logs = [], []
    panel.trajectory_changed.connect(got.append)
    panel.log_requested.connect(logs.append)
    panel.set_model(_model(4, 3), SimulationSettings())
    assert panel.trajectory is None and got == [None]
    assert any("trajectory dropped" in m for m in logs)


# ------------------------------------------------------------ MainWindow wiring
@pytest.fixture
def win(qapp, monkeypatch):
    monkeypatch.setattr(AppSettings, "_ORGANIZATION", TEST_ORG)
    monkeypatch.setattr(AppSettings, "_APPLICATION", TEST_ORG)
    s = QSettings(TEST_ORG, TEST_ORG)
    s.clear()
    s.sync()
    w = MainWindow()
    w.show()
    yield w
    w.project_controller._is_modified = False
    w.sim_dock.hide()
    w.close()
    s.clear()
    s.sync()


def _load(win, name="planar_2r.urdf"):
    with mock.patch("app.main_window.QMessageBox.warning"):
        win.import_urdf_file(str(FX / name))
        assert win.task_runner.wait_all(20000)
        QApplication.processEvents()
    win.project_controller._is_modified = False
    return win.project_controller.robot


def test_trajectory_saved_in_project_and_reloaded(win, tmp_path):
    model = _load(win)
    pc = win.project_controller
    n = model.dof()
    tr = Trajectory([Waypoint(0.0, [0.0] * n), Waypoint(1.0, [0.2] * n),
                     Waypoint(2.5, [-0.1] * n)], "cubic", True)
    assert win.sim_panel.set_trajectory(tr)
    # edit through the table -> project follows and is marked modified
    _set(win.sim_panel.timeline, 1, 1, "0.25")
    assert pc.current_project.trajectory is not None and pc.is_modified
    expected = win.sim_panel.trajectory
    assert expected.waypoints[1].q[0] == 0.25
    out = tmp_path / "t.robotproj"
    save(pc.current_project, out)
    pc._is_modified = False
    pc.close_project(None)
    assert win.sim_panel.trajectory is None
    assert pc.open_project(None, str(out))
    assert pc.current_project.trajectory == expected
    assert win.sim_panel.trajectory == expected
    assert win.sim_panel.timeline.table.rowCount() == 3


def test_robot_with_other_dof_drops_trajectory_in_project(win):
    model = _load(win)
    pc = win.project_controller
    n = model.dof()
    win.sim_panel.set_trajectory(Trajectory([Waypoint(0.0, [0.0] * n), Waypoint(1.0, [0.1] * n)]))
    pc.current_project.trajectory = win.sim_panel.trajectory
    other = _model(7, n + 1)
    pc._is_modified = False
    pc.set_robot(other)
    assert pc.current_project.trajectory is None
    assert win.sim_panel.trajectory is None
    assert pc.is_modified


def test_open_project_with_mismatched_trajectory_drops_it(win, tmp_path):
    model = _load(win)
    pc = win.project_controller
    n = model.dof()
    pc.current_project.trajectory = Trajectory([Waypoint(0.0, [0.0] * (n + 1)),
                                                Waypoint(1.0, [0.1] * (n + 1))])
    out = tmp_path / "m.robotproj"
    save(pc.current_project, out)
    pc._is_modified = False
    pc.close_project(None)
    assert pc.open_project(None, str(out))
    assert pc.current_project.trajectory is None and win.sim_panel.trajectory is None
