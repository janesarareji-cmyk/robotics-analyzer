"""U3.6: objects in the scene (add, move, edit, save/reopen, collide with the robot)."""
import os
from unittest import mock

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtCore import QCoreApplication

from app.importers.urdf import import_urdf
from app.kin_utils import current_q, write_q
from app.main_window import MainWindow
from app.project.serializer import save
from app.sample_robots import list_sample_robots
from app.scene_objects import default_object
from app.settings import AppSettings

SAMPLES = {p.parent.name: p for _, p in list_sample_robots()}


@pytest.fixture
def win(qapp, monkeypatch):
    monkeypatch.setattr(AppSettings, "_ORGANIZATION", "RoboticsAnalyzer_test_u36")
    monkeypatch.setattr(AppSettings, "_APPLICATION", "RoboticsAnalyzer_test_u36")
    w = MainWindow()
    w.settings._settings.clear()
    yield w
    w.project_controller._is_modified = False
    w.close()


def load(win, name="ur5"):
    path = SAMPLES[name]
    res = import_urdf(str(path))
    win._import_path = str(path)
    win._on_urdf_import_result(res)
    assert win.task_runner.wait_all(60000)
    QCoreApplication.processEvents()
    return win.project_controller.robot


def env(win):
    return win.project_controller.current_project.collision.environment


@pytest.mark.parametrize("kind", ["box", "cylinder", "sphere"])
def test_add_object_defaults(win, kind):
    model = load(win)
    assert win.add_scene_object(kind)
    (o,) = env(win)
    assert o.kind == kind
    # on the floor (bottom at base z - 1 mm), 0.4 m in front of the base along +x
    params, xyz = default_object(kind, model)
    assert o.xyz == pytest.approx(xyz)
    assert o.xyz[0] == pytest.approx(0.4, abs=1e-6) and o.xyz[1] == pytest.approx(0.0, abs=1e-6)
    h = {"box": 0.05, "cylinder": 0.05, "sphere": 0.05}[kind]
    assert o.xyz[2] == pytest.approx(-0.001 + h)
    # 0.1 m size
    if kind == "box":
        assert o.params["size"] == [0.1, 0.1, 0.1]
    win._sync_env_objects()
    assert len(win.robot_view.env_actors) == 1
    assert win.project_controller.is_modified
    assert win.collision_panel.selected_shape == 0


def test_debounced_redraw(win):
    load(win)
    win.collision_panel.add_shape("box", {"size": [0.1, 0.1, 0.1]}, [0.4, 0, 0.05])
    assert win._env_timer.isActive() and win._env_timer.interval() == 100
    assert len(win.robot_view.env_actors) == 0
    win._env_timer.timeout.emit()
    assert len(win.robot_view.env_actors) == 1


def test_drag_widget_callback_moves_object(win):
    load(win)
    win.add_scene_object("box")
    win._sync_env_objects()
    import pyvista as pv
    drag = win._object_drag
    assert drag.index == 0
    drag.on_box(pv.Box(bounds=(0.5, 0.7, -0.1, 0.1, 0.0, 0.2)))        # centre (0.6, 0, 0.1)
    assert env(win)[0].xyz == pytest.approx([0.6, 0.0, 0.1])
    win._sync_env_objects()
    m = win.robot_view.env_actors[0].user_matrix
    assert np.asarray(m)[:3, 3] == pytest.approx([0.6, 0.0, 0.1])


def test_edit_selected_object(win):
    load(win)
    win.add_scene_object("cylinder")
    cp = win.collision_panel
    assert cp.update_shape(0, name="can", params={"radius": 0.03, "length": 0.2},
                           xyz=[0.3, 0.1, 0.1], rpy=[0.0, 0.5, 0.0])
    o = env(win)[0]
    assert (o.name, o.params, o.xyz, o.rpy) == ("can", {"radius": 0.03, "length": 0.2},
                                               [0.3, 0.1, 0.1], [0.0, 0.5, 0.0])
    assert cp.selected_shape == 0
    assert not cp.update_shape(0, params={"radius": -1.0, "length": 0.2})     # bad size refused
    assert env(win)[0].params["radius"] == 0.03
    # form -> object
    cp._name.setText("tin")
    cp._size[0].setValue(0.04)
    assert cp._update_selected_from_form()
    assert env(win)[0].name == "tin" and env(win)[0].params["radius"] == pytest.approx(0.04)
    assert cp.remove_shape(0)
    assert env(win) == []
    win._sync_env_objects()
    assert win.robot_view.env_actors == []


def test_save_reopen_equal(win, tmp_path):
    load(win)
    win.add_scene_object("box")
    win.add_scene_object("sphere")
    win.collision_panel.move_shape(1, [0.2, 0.3, 0.05])
    before = [o.to_dict() for o in env(win)]
    path = tmp_path / "scene.robotproj"
    pc = win.project_controller
    save(pc.current_project, path)
    pc._is_modified = False
    pc.close_project(win)
    with mock.patch("app.main_window.QMessageBox.warning"):
        assert pc.open_project(win, str(path))
    assert [o.to_dict() for o in env(win)] == before
    assert len(win.robot_view.env_actors) == 2


def test_object_collides_with_robot_and_turns_red(win):
    model = load(win)
    win.add_scene_object("box")
    win._sync_env_objects()
    svc = win.collision_service
    # put a big box on the TCP of the current pose: robot touches it
    import core_bridge as cb
    tcp = np.asarray(cb.fk(model)["tcp"])
    win.collision_panel.update_shape(0, params={"size": [0.3, 0.3, 0.3]}, xyz=tcp[:3, 3].tolist())
    win._sync_env_objects()
    res = svc.check(current_q(model))
    assert not res.ok and res.env_ids == {0}
    assert win.robot_view.env_color(0) == pytest.approx((1.0, 0.1882, 0.1882), abs=0.005)
    # move the box far away: free again, normal colour
    win.collision_panel.move_shape(0, [3.0, 3.0, 0.05])
    res = svc.check(current_q(model))
    assert res.ok
    assert win.robot_view.env_color(0) != pytest.approx((1.0, 0.1882, 0.1882), abs=0.005)


def test_motion_blocked_by_object(win):
    model = load(win)
    win.add_scene_object("box")
    svc = win.collision_service
    import core_bridge as cb
    q0 = np.array(current_q(model))
    q1 = q0.copy()
    q1[1] += 0.6
    write_q(model, q1)
    tcp = np.asarray(cb.fk(model)["tcp"])
    write_q(model, q0)
    win.collision_panel.update_shape(0, params={"size": [0.1, 0.1, 0.1]}, xyz=tcp[:3, 3].tolist())
    assert svc.is_free(q0) and not svc.is_free(q1)
    q = np.array(svc.first_free(q0, q1))
    assert svc.is_free(q) and 0 < np.linalg.norm(q - q0) < np.linalg.norm(q1 - q0)
    assert 0 in svc.last_block.env_ids
