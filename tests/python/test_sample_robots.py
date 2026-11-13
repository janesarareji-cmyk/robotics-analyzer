"""
tests/python/test_sample_robots.py
U1: bundled sample robots (robots/<name>/<name>.urdf) and the File menu entries.
"""

import os

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import core_bridge as cb
from PySide6.QtWidgets import QMenu

from app.importers.urdf import import_urdf
from app.kin_utils import dof_limits
from app.main_window import MainWindow
from app.sample_robots import list_sample_robots, sample_robots_dir
from app.settings import AppSettings

ROBOTS = list_sample_robots()


@pytest.fixture
def win(qapp, monkeypatch):
    monkeypatch.setattr(AppSettings, "_ORGANIZATION", "RoboticsAnalyzer_test_samples")
    monkeypatch.setattr(AppSettings, "_APPLICATION", "RoboticsAnalyzer_test_samples")
    w = MainWindow()
    w.settings._settings.clear()
    yield w
    w.project_controller._is_modified = False
    w.close()


def test_list_sample_robots_finds_nine():
    assert sample_robots_dir().is_dir()
    assert len(ROBOTS) == 9
    names = {p.parent.name for _, p in ROBOTS}
    assert "xarm5_gripper" in names
    titles = [t for t, _ in ROBOTS]
    assert titles == sorted(titles, key=str.lower)
    assert "UFACTORY xArm 5 + xArm Gripper" in titles


def test_list_sample_robots_missing_folder(tmp_path):
    assert list_sample_robots(tmp_path / "nope") == []


def test_list_sample_robots_title_fallback(tmp_path):
    (tmp_path / "abc").mkdir()
    (tmp_path / "abc" / "abc.urdf").write_text("<robot name='a'/>")
    assert list_sample_robots(tmp_path) == [("abc", tmp_path / "abc" / "abc.urdf")]


@pytest.mark.parametrize("title,path", ROBOTS, ids=[p.parent.name for _, p in ROBOTS])
def test_sample_robot_imports_and_fk_finite(title, path):
    res = import_urdf(str(path))
    assert not res.report.has_errors(), res.report.summary()
    assert res.model is not None
    assert not any(v.fallback for v in res.visuals)
    cm = cb.compile(res.model)
    n = res.model.dof()
    assert n >= 5
    lo, hi = dof_limits(res.model)
    rng = np.random.default_rng(1)
    qs = [np.zeros(n), lo + rng.random(n) * (np.asarray(hi) - np.asarray(lo))]
    for q in qs:
        out = cb.fk_compiled(cm, [float(x) for x in q])
        for f in out["frames"]:
            assert np.all(np.isfinite(np.asarray(f)))


def test_xarm5_gripper_structure():
    path = next(p for _, p in ROBOTS if p.parent.name == "xarm5_gripper")
    model = import_urdf(str(path)).model
    assert model.dof() == 6
    names = [j.name for j in model.joints]
    assert names.count("drive_joint") == 1
    assert all(f"joint{i}" in names for i in range(1, 6))
    mimic = [j.name for j in model.joints if j.mimic is not None]
    assert len(mimic) == 5
    assert "drive_joint" not in mimic
    for j in model.joints:
        if j.mimic is not None:
            assert j.mimic.joint == "drive_joint"


def _sample_menu(win):
    for a in win.menuBar().actions():
        if a.text() == "&File":
            for b in a.menu().actions():
                if b.menu() is not None and "Sample Robot" in b.text():
                    return b.menu()
    raise AssertionError("no sample robot menu")


def test_file_menu_has_nine_enabled_entries(win):
    menu = _sample_menu(win)
    acts = menu.actions()
    assert len(acts) == 9
    assert all(a.isEnabled() for a in acts)
    assert win._sample_button.menu() is menu


def test_trigger_loads_project(win):
    menu = _sample_menu(win)
    act = next(a for a in menu.actions() if a.text() == "UFACTORY xArm 5 + xArm Gripper")
    from PySide6.QtWidgets import QApplication
    import time
    act.trigger()
    t0 = time.time()
    while win.project_controller.robot is None and time.time() - t0 < 60:
        QApplication.processEvents()
        time.sleep(0.01)
    robot = win.project_controller.robot
    assert robot is not None
    assert robot.dof() == 6
