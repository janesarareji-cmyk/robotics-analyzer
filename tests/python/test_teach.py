"""U2.5 teach waypoints."""

import os
import time

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtCore import QCoreApplication
from PySide6.QtWidgets import QApplication

from app.importers.urdf import import_urdf
from app.kin_utils import current_q, write_q
from app.main_window import MainWindow
from app.project.serializer import save
from app.sample_robots import list_sample_robots
from app.settings import AppSettings
from app.sim.engine import PAUSED, RUNNING
from app.panels.timeline_panel import TEACH_MIN_SEGMENT, TEACH_SPEED

SAMPLES = {p.parent.name: (t, p) for t, p in list_sample_robots()}


@pytest.fixture
def win(qapp, monkeypatch):
    monkeypatch.setattr(AppSettings, "_ORGANIZATION", "RoboticsAnalyzer_test_u2t")
    monkeypatch.setattr(AppSettings, "_APPLICATION", "RoboticsAnalyzer_test_u2t")
    w = MainWindow()
    w.settings._settings.clear()
    w.show()
    yield w
    w.project_controller._is_modified = False
    w.sim_dock.hide()
    w.close()


def _load(win, name="ur5"):
    path = SAMPLES[name][1]
    res = import_urdf(str(path))
    win._import_path = str(path)
    win._on_urdf_import_result(res)
    QCoreApplication.processEvents()
    return win.project_controller.robot


def _pose(win, model, q):
    write_q(model, q)
    win.joint_panel.set_model(model, save_baseline=False)
    win.sim_panel.notify_q_edited()


def _pump(ms):
    end = time.perf_counter() + ms / 1000.0
    while time.perf_counter() < end:
        QApplication.processEvents()
        time.sleep(0.002)


def test_teach_add_undo_clear_and_times(win):
    model = _load(win)
    tl = win.sim_panel.timeline
    n = model.dof()
    q0 = np.zeros(n)
    q1 = q0.copy(); q1[0] = 0.1          # short move: min segment
    q2 = q1.copy(); q2[1] = 2.0          # long move: distance / speed
    for q in (q0, q1, q2):
        _pose(win, model, q)
        tl.teach_add()
    assert tl.table.rowCount() == 3 and "3" in tl._teach_count.text()
    ts = [float(tl.table.item(r, 0).text()) for r in range(3)]
    assert ts[0] == 0.0
    assert ts[1] == pytest.approx(TEACH_MIN_SEGMENT)
    assert ts[2] - ts[1] == pytest.approx(2.0 / TEACH_SPEED)
    assert tl.trajectory is not None and tl.trajectory.interpolation == "cubic"
    tl.teach_undo()
    assert tl.table.rowCount() == 2 and "2" in tl._teach_count.text()
    tl.teach_clear()
    assert tl.table.rowCount() == 0 and tl.trajectory is None
    assert win.project_controller.current_project.trajectory is None


def test_teach_play_ends_on_last_and_roundtrip(win, tmp_path):
    model = _load(win)
    tl = win.sim_panel.timeline
    n = model.dof()
    poses = []
    for k in range(3):
        q = np.zeros(n)
        q[0] = 0.2 * k
        q[1] = -0.1 * k
        poses.append(q)
        _pose(win, model, q)
        tl.teach_add()
    pc = win.project_controller
    assert pc.current_project.trajectory is not None
    win.sim_dock.show()
    QApplication.processEvents()
    win.sim_panel.timeline._speed.setValue(8.0)
    tl.teach_play()
    assert win.sim_panel.engine.clock.state == RUNNING
    _pump(1500)
    assert win.sim_panel.engine.clock.state == PAUSED
    assert np.allclose(win.sim_panel.engine.state.q, poses[-1], atol=1e-9)
    win.sim_panel.stop_trajectory()
    expected = win.sim_panel.trajectory
    out = tmp_path / "teach.robotproj"
    save(pc.current_project, out)
    pc._is_modified = False
    pc.close_project(None)
    assert pc.open_project(None, str(out))
    assert pc.current_project.trajectory == expected
    assert win.sim_panel.timeline.table.rowCount() == 3
    assert np.allclose(win.sim_panel.trajectory.waypoints[-1].q, poses[-1])
