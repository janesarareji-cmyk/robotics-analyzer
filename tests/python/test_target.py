"""U4a.1: target poses, store, project save/load, 3D target controller."""
import os

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtCore import QCoreApplication

import core_bridge as cb
from app.importers.urdf import import_urdf
from app.main_window import MainWindow
from app.project.project_data import ProjectData
from app.project import serializer
from app.sample_robots import list_sample_robots
from app.settings import AppSettings
from app.target import (
    Target, TargetStore, pose_from_xyz_rpy, pose_on_surface, rotate_pose, xyz_rpy_from_pose,
)
from app.viewport.target_widget import TargetController

SAMPLES = {p.parent.name: (t, p) for t, p in list_sample_robots()}


@pytest.fixture
def win(qapp, monkeypatch):
    monkeypatch.setattr(AppSettings, "_ORGANIZATION", "RoboticsAnalyzer_test_u4a1")
    monkeypatch.setattr(AppSettings, "_APPLICATION", "RoboticsAnalyzer_test_u4a1")
    w = MainWindow()
    w.settings._settings.clear()
    w.show()
    yield w
    w.project_controller._is_modified = False
    w.close()


def _load(win, name):
    path = SAMPLES[name][1]
    win._import_path = str(path)
    win._on_urdf_import_result(import_urdf(str(path)))
    win.task_runner.wait_all(30000)
    QCoreApplication.processEvents()
    return win.project_controller.robot


def test_rpy_round_trip():
    rng = np.random.default_rng(1)
    for _ in range(50):
        xyz = rng.uniform(-1, 1, 3)
        rpy = rng.uniform([-3, -1.5, -3], [3, 1.5, 3])
        T = pose_from_xyz_rpy(xyz, rpy)
        x2, r2 = xyz_rpy_from_pose(T)
        assert np.allclose(x2, xyz)
        assert np.allclose(pose_from_xyz_rpy(x2, r2), T, atol=1e-12)


def test_rotate_pose_local_and_world():
    T = pose_from_xyz_rpy([0.1, 0.2, 0.3], [0, 0, 0])
    T2 = rotate_pose(T, "z", 90)
    assert np.allclose(T2[:3, 0], [0, 1, 0], atol=1e-12)
    assert np.allclose(T2[:3, 3], T[:3, 3])
    T3 = rotate_pose(rotate_pose(T, "x", 30), "x", -30)
    assert np.allclose(T3, T, atol=1e-12)


def test_pose_on_surface_z_is_minus_normal():
    for n in ([0, 0, 1], [1, 0, 0], [0.3, -0.5, 0.8], [0, 0, -1]):
        T = pose_on_surface([0.4, 0.1, 0.2], n)
        nn = np.array(n, float) / np.linalg.norm(n)
        assert np.allclose(T[:3, 2], -nn, atol=1e-12)
        assert np.allclose(T[:3, :3] @ T[:3, :3].T, np.eye(3), atol=1e-12)
        assert np.isclose(np.linalg.det(T[:3, :3]), 1.0)
        assert np.allclose(T[:3, 3], [0.4, 0.1, 0.2])
    with pytest.raises(ValueError):
        pose_on_surface([0, 0, 0], [0, 0, 0])


def test_store_names_and_dict_round_trip():
    s = TargetStore()
    a = s.add(np.eye(4))
    b = s.add(pose_from_xyz_rpy([1, 2, 3], [0.1, 0.2, 0.3]), "pick")
    assert a.name == "Target 1" and b.name == "pick"
    c = s.add(np.eye(4), "pick")                      # duplicate name made unique
    assert c.name != "pick" and len(set(s.names())) == 3
    assert not s.rename(0, "pick") and s.rename(0, "home")
    s2 = TargetStore.from_dict(s.to_dict())
    assert s2 == s
    bad = {"targets": [{"name": "x", "pose": [[1, 2]]}, {"name": "ok", "pose": np.eye(4).tolist()}]}
    assert TargetStore.from_dict(bad).names() == ["ok"]
    with pytest.raises(ValueError):
        Target("n", np.full((4, 4), np.nan))


def test_project_save_load(tmp_path):
    p = ProjectData.new("t")
    p.targets.add(pose_from_xyz_rpy([0.3, 0.1, 0.5], [0.2, -0.4, 1.0]), "A")
    p.targets.add(np.eye(4), "B")
    f = tmp_path / "t.robotproj"
    serializer.save(p, f)
    q = serializer.load(f)
    assert q.targets == p.targets
    # no targets: member absent, old behaviour
    p2 = ProjectData.new("t2")
    f2 = tmp_path / "t2.robotproj"
    serializer.save(p2, f2)
    import zipfile
    assert "targets.json" not in zipfile.ZipFile(f2).namelist()
    assert len(serializer.load(f2).targets) == 0


def test_controller_move_rotate_status(win):
    _load(win, "ur5")
    tc = TargetController(win.viewport)
    seen = []
    tc.pose_changed.connect(lambda p: seen.append(p))
    tc.set_pose(pose_from_xyz_rpy([0.4, 0.0, 0.3], [0, 0, 0]), emit=False)
    tc.show(True)
    tc.move_to([0.5, 0.1, 0.2])
    assert np.allclose(tc.pose[:3, 3], [0.5, 0.1, 0.2])
    tc.rotate("y", 90)
    assert np.allclose(tc.pose[:3, 2], [1, 0, 0], atol=1e-12)
    assert len(seen) == 2
    for st in ("ok", "bad", "warn", "neutral"):
        tc.set_status(st)
        assert tc.status == st
    with pytest.raises(ValueError):
        tc.set_status("nope")
    assert tc.widget is not None
    assert len(tc._actors) == 2
    tc.show(False)
    assert tc.widget is None and not tc._actors
    tc.dispose()


def test_controller_place_at_and_new_default(win):
    model = _load(win, "ur5")
    tc = TargetController(win.viewport, get_ref_rot=lambda: np.eye(3))
    tc.show(True)
    got = []
    tc.committed.connect(lambda p: got.append(p))
    tcp = np.asarray(cb.fk(model)["tcp"])
    tc.set_pose(tcp)                                 # default = current TCP pose
    assert np.allclose(tc.pose, tcp)
    p = tc.place_at([0.4, 0.0, 0.1], [0, 0, 1])
    assert np.allclose(p[:3, 2], [0, 0, -1]) and np.allclose(p[:3, 3], [0.4, 0, 0.1])
    assert len(got) == 1
    tc.dispose()


def test_controller_pick_box_top(win):
    """Real picker: project a point on a box top to the screen, pick, get the top normal."""
    _load(win, "ur5")
    win.collision_panel.add_shape("box", {"size": [0.2, 0.2, 0.1]}, [0.5, 0.0, 0.05], name="box")
    QCoreApplication.processEvents()
    win._sync_env_objects()
    tc = TargetController(win.viewport)
    pl = win.viewport.plotter
    pl.camera.focal_point = (0.5, 0.0, 0.1)
    pl.camera.position = (0.9, -0.5, 0.9)
    pl.render()
    import vtk
    coord = vtk.vtkCoordinate()
    coord.SetCoordinateSystemToWorld()
    coord.SetValue(0.5, 0.0, 0.1)
    x, y = coord.GetComputedDoubleDisplayValue(pl.renderer)
    hit = tc.pick_at_display(x, y)
    assert hit is not None, "picker found nothing"
    pt, n = hit
    assert abs(pt[2] - 0.1) < 2e-3 and np.linalg.norm(pt[:2] - [0.5, 0.0]) < 0.02
    assert np.allclose(n, [0, 0, 1], atol=1e-6)
    tc.dispose()
