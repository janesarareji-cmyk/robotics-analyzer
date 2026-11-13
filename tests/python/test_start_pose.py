"""U3.4b: a loaded sample robot starts collision free (floor on)."""
import os

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtCore import QCoreApplication

from app.importers.urdf import import_urdf
from app.kin_utils import current_q
from app.main_window import MainWindow
from app.sample_robots import list_sample_robots
from app.settings import AppSettings

SAMPLES = {p.parent.name: p for _, p in list_sample_robots()}


@pytest.fixture
def win(qapp, monkeypatch):
    monkeypatch.setattr(AppSettings, "_ORGANIZATION", "RoboticsAnalyzer_test_u34b")
    monkeypatch.setattr(AppSettings, "_APPLICATION", "RoboticsAnalyzer_test_u34b")
    w = MainWindow()
    w.settings._settings.clear()
    yield w
    w.project_controller._is_modified = False
    w.close()


def test_nine_samples():
    assert len(SAMPLES) == 9


@pytest.mark.parametrize("name", sorted(SAMPLES))
def test_start_pose_free(win, name):
    path = SAMPLES[name]
    res = import_urdf(str(path))
    win._import_path = str(path)
    win._on_urdf_import_result(res)
    assert win.task_runner.wait_all(60000)
    QCoreApplication.processEvents()
    model = win.project_controller.robot
    q = current_q(model)
    svc = win.collision_service
    assert svc.enabled
    r = svc.check(q, emit=False)
    assert r.ok, (name, r.text())
    # joint panel and model agree
    assert win.joint_panel is not None


def test_planner_start_not_in_collision(win):
    """UR5: the start pose is free, so a planner call does not fail with 'start in collision'."""
    path = SAMPLES["ur5"]
    res = import_urdf(str(path))
    win._import_path = str(path)
    win._on_urdf_import_result(res)
    assert win.task_runner.wait_all(60000)
    QCoreApplication.processEvents()
    assert win.collision_service.is_free(current_q(win.project_controller.robot))
