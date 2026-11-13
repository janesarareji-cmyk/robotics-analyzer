"""U3.7: grasp (close on an object attaches it, open releases it)."""
import os
from unittest import mock

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtCore import QCoreApplication

import core_bridge as cb
from app.grasp import grasp_link, gripper_links, iso_to_xyz_rpy
from app.gripper import gripper_dofs
from app.importers.urdf import import_urdf
from app.kin_utils import current_q, dof_limits, write_q
from app.main_window import MainWindow
from app.planning.collision import pose_matrix
from app.project.serializer import save
from app.sample_robots import list_sample_robots
from app.settings import AppSettings

SAMPLES = {p.parent.name: p for _, p in list_sample_robots()}


@pytest.fixture
def win(qapp, monkeypatch):
    monkeypatch.setattr(AppSettings, "_ORGANIZATION", "RoboticsAnalyzer_test_u37")
    monkeypatch.setattr(AppSettings, "_APPLICATION", "RoboticsAnalyzer_test_u37")
    w = MainWindow()
    w.settings._settings.clear()
    yield w
    w.project_controller._is_modified = False
    w.close()


def load(win, name):
    path = SAMPLES[name]
    res = import_urdf(str(path))
    win._import_path = str(path)
    win._on_urdf_import_result(res)
    assert win.task_runner.wait_all(60000)
    QCoreApplication.processEvents()
    return win.project_controller.robot


def put_q(win, q):
    model = win.project_controller.robot
    write_q(model, q)
    win.joint_panel.set_model(model, save_baseline=False)
    win._update_robot_view()


def cfg(win):
    return win.project_controller.current_project.collision


def free_poses(win, n=80):
    model = win.project_controller.robot
    svc = win.collision_service
    lo, hi = (np.asarray(v) for v in dof_limits(model))
    rng = np.random.default_rng(3)
    yield np.asarray(current_q(model))
    for _ in range(n):
        q = lo + rng.random(len(lo)) * (hi - lo)
        for g in gripper_dofs(model):
            q[g.dof_index] = g.open_value
        if svc.is_free(q):
            yield q


def place_cube(win, size=0.02):
    """Open gripper at a free pose, cube between the fingers and still collision free."""
    model = win.project_controller.robot
    svc = win.collision_service
    fingers, _ = gripper_links(model)
    gdof = gripper_dofs(model)
    for q in free_poses(win):
        q = np.array(q)
        for g in gdof:
            q[g.dof_index] = g.open_value
        write_q(model, q)
        fr = np.asarray(cb.fk(model)["frames"])
        pts = np.array([fr[k][:3, 3] for k in sorted(fingers)])
        mid = pts.mean(axis=0)
        axes = [fr[k][:3, 2] for k in sorted(fingers)]
        for off in (0.0, 0.02, 0.03, 0.04, 0.05, 0.06, -0.02, -0.04):
            c = mid + off * axes[0]
            if c[2] < 0.1:
                continue
            put_q(win, q)
            win.collision_panel.add_shape("box", {"size": [size] * 3}, c.tolist(), name="cube")
            if svc.is_free(q):
                return q, c
            win.collision_panel.remove_shape(0)
    raise AssertionError("no free pose with a cube between the fingers")


def cube_world(win, link_name):
    """World pose of the attached cube from the link frame."""
    model = win.project_controller.robot
    li = [l.name for l in model.links].index(link_name)
    fr = np.asarray(cb.fk(model)["frames"])
    e = cfg(win).attached[0]
    return fr[li] @ pose_matrix(e.xyz, e.rpy)


def test_iso_round_trip():
    rng = np.random.default_rng(1)
    for _ in range(50):
        xyz = rng.uniform(-1, 1, 3)
        rpy = rng.uniform(-1.4, 1.4, 3)
        T = pose_matrix(xyz, rpy)
        x2, r2 = iso_to_xyz_rpy(T)
        assert np.allclose(pose_matrix(x2, r2), T, atol=1e-9)


@pytest.mark.parametrize("name", ["xarm5_gripper", "panda"])
def test_grasp_move_release(win, name):
    model = load(win, name)
    svc = win.collision_service
    place_cube(win)
    jp = win.joint_panel
    jp.set_gripper(1.0)                                       # close on the cube
    assert [e.name for e in cfg(win).attached] == ["cube"]
    assert cfg(win).environment == []
    link = cfg(win).attached[0].attached_to
    assert link == model.links[grasp_link(model)].name
    assert win._status_label.text() == "Grasped cube"
    assert win.project_controller.is_modified
    win._sync_env_objects()
    assert len(win.robot_view.attached_actors) == 1 and win.robot_view.env_actors == []
    world0 = cube_world(win, link)
    # frozen relative to the link: move the arm, the cube follows the link
    q1 = np.array(current_q(model))
    q1[0] += 0.4
    q1[1] += 0.2
    put_q(win, q1)
    assert svc.is_free(q1)
    world1 = cube_world(win, link)
    assert not np.allclose(world0[:3, 3], world1[:3, 3])
    assert np.allclose(np.asarray(win.robot_view.attached_actors[0].user_matrix), world1, atol=1e-9)
    # collides with the environment: a box at the cube position
    win.collision_panel.add_shape("box", {"size": [0.05] * 3}, world1[:3, 3].tolist(), name="wall")
    assert not svc.is_free(q1)
    assert svc.check(q1, emit=False).env_ids == {0}
    win.collision_panel.remove_shape(0)
    assert svc.is_free(q1)
    # open: released at the current world pose
    jp.set_gripper(0.0)
    assert cfg(win).attached == []
    (e,) = cfg(win).environment
    assert e.name == "cube" and e.attached_to == ""
    assert np.allclose(pose_matrix(e.xyz, e.rpy), world1, atol=1e-9)
    assert win._status_label.text() == "Released cube"
    win._sync_env_objects()
    assert len(win.robot_view.env_actors) == 1 and win.robot_view.attached_actors == []


def test_attached_not_blocked_by_gripper_links(win):
    model = load(win, "panda")
    svc = win.collision_service
    place_cube(win)
    win.joint_panel.set_gripper(1.0)
    assert cfg(win).attached
    q = np.asarray(current_q(model))
    assert svc.is_free(q)
    w = svc.world
    assert any(i in w._ignore for i in range(len(w.robot_shapes)))
    win.joint_panel.set_gripper(1.0)          # closing further: no collision with the grasped cube
    assert svc.is_free(np.asarray(current_q(model)))


def test_attached_hits_arm_link(win):
    """A big grasped cube collides with a non-gripper link of the arm."""
    model = load(win, "panda")
    svc = win.collision_service
    place_cube(win)
    win.joint_panel.set_gripper(1.0)
    link = cfg(win).attached[0].attached_to
    li = [l.name for l in model.links].index(link)
    assert svc.world.robot_shapes[-1].link == li
    hit = False
    for size in (0.1, 0.2, 0.3, 0.4, 0.6):
        cfg(win).attached[0].params = {"size": [size] * 3}
        svc.invalidate()
        r = svc.check(np.asarray(current_q(model)), emit=False)
        if not r.ok:
            hit = True
            break
    assert hit


def test_close_in_air_does_nothing(win):
    load(win, "panda")
    jp = win.joint_panel
    jp.set_gripper(1.0)
    assert cfg(win) is None or cfg(win).attached == []
    jp.set_gripper(0.0)
    assert cfg(win) is None or (cfg(win).attached == [] and cfg(win).environment == [])


def test_save_reopen_attached(win, tmp_path):
    load(win, "panda")
    place_cube(win)
    win.joint_panel.set_gripper(1.0)
    before = [e.to_dict() for e in cfg(win).attached]
    assert before and before[0]["attached_to"]
    path = tmp_path / "grasp.robotproj"
    pc = win.project_controller
    save(pc.current_project, path)
    pc._is_modified = False
    pc.close_project(win)
    with mock.patch("app.main_window.QMessageBox.warning"):
        assert pc.open_project(win, str(path))
    assert [e.to_dict() for e in cfg(win).attached] == before
    assert len(win.robot_view.attached_actors) == 1
    win.joint_panel.set_gripper(0.0)
    assert cfg(win).attached == [] and len(cfg(win).environment) == 1
