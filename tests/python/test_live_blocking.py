"""U3.4: live collision blocking in every motion input."""
import os
import time

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
pytest.importorskip("fcl")

from PySide6.QtCore import QCoreApplication
from PySide6.QtWidgets import QApplication

import core_bridge as cb
from app.gripper import gripper_dofs
from app.importers.urdf import import_urdf
from app.kin_utils import current_q, dof_limits, write_q
from app.main_window import MainWindow
from app.planning.collision import FLOOR_ID
from app.sample_robots import list_sample_robots
from app.settings import AppSettings
from app.sim.trajectory import Trajectory

SAMPLES = {p.parent.name: (t, p) for t, p in list_sample_robots()}


@pytest.fixture
def win(qapp, monkeypatch):
    monkeypatch.setattr(AppSettings, "_ORGANIZATION", "RoboticsAnalyzer_test_u3b")
    monkeypatch.setattr(AppSettings, "_APPLICATION", "RoboticsAnalyzer_test_u3b")
    w = MainWindow()
    w.settings._settings.clear()
    w.collision_action.setChecked(True)
    w.show()
    yield w
    w.project_controller._is_modified = False
    w.sim_dock.hide()
    w.close()


def load(win, name):
    path = SAMPLES[name][1]
    res = import_urdf(str(path))
    win._import_path = str(path)
    win._on_urdf_import_result(res)
    assert win.task_runner.wait_all(30000)
    QCoreApplication.processEvents()
    return win.project_controller.robot


def put_q(win, q):
    model = win.project_controller.robot
    write_q(model, q)
    win.joint_panel.set_model(model, save_baseline=False)
    win._update_robot_view()
    win.sim_panel.notify_q_edited()


def random_free_q(win, seed=0, n=2000):
    svc = win.collision_service
    model = win.project_controller.robot
    lo, hi = (np.asarray(v) for v in dof_limits(model))
    rng = np.random.default_rng(seed)
    for _ in range(n):
        q = lo + rng.random(len(lo)) * (hi - lo)
        for g in gripper_dofs(model):
            q[g.dof_index] = g.open_value
        if svc.is_free(q):
            return q
    raise AssertionError("no free pose found")


def find_blocked_move(win, a):
    """(dof, target value): moving dof alone from ``a`` ends in collision."""
    svc = win.collision_service
    model = win.project_controller.robot
    lo, hi = dof_limits(model)
    for dof in range(len(a)):
        if dof in {g.dof_index for g in gripper_dofs(model)}:
            continue
        for v in np.linspace(lo[dof], hi[dof], 41):
            b = a.copy()
            b[dof] = v
            if abs(v - a[dof]) > 0.3 and not svc.is_free(b):
                return dof, float(v)
    raise AssertionError("no blocked move found")


def row_of_dof(win, dof):
    model = win.project_controller.robot
    for row in win.joint_panel.rows:
        if model.q_index_map()[row.joint_index] == dof:
            return row
    raise AssertionError("no row")


@pytest.mark.parametrize("name", ["ur5", "panda"])
def test_slider_move_into_collision_stops_near_contact(win, name):
    model = load(win, name)
    svc = win.collision_service
    a = random_free_q(win)
    put_q(win, a)
    dof, v = find_blocked_move(win, a)
    row = row_of_dof(win, dof)
    row.spinbox.setValue(v * row.to_display)               # jump: collides at the target
    q = np.asarray(current_q(model))
    assert svc.is_free(q), "robot must end collision-free"
    assert abs(q[dof] - v) > 1e-3 and abs(q[dof] - a[dof]) < abs(v - a[dof])
    assert np.allclose(np.delete(q, dof), np.delete(a, dof))
    # contact is right in front of the stop: 2e-3 rad further along the move collides
    step = 2e-3 * np.sign(v - a[dof])
    ahead = q.copy()
    ahead[dof] += step
    assert not svc.is_free(ahead), "stop must be within 2e-3 of the contact"
    assert win._status_label.text().startswith("Blocked:")
    # the slider went back to the free value
    assert row.spinbox.value() == pytest.approx(q[dof] * row.to_display, abs=0.02)


def test_free_slider_move_is_untouched(win):
    model = load(win, "panda")
    a = random_free_q(win, seed=3)
    put_q(win, a)
    row = win.joint_panel.rows[0]
    target = a[0] + 0.01
    row.spinbox.setValue(target * row.to_display)
    assert current_q(model)[0] == pytest.approx(target, abs=1e-4)


def test_collision_toggle_is_pass_through_and_saved(win):
    model = load(win, "panda")
    a = random_free_q(win)
    put_q(win, a)
    dof, v = find_blocked_move(win, a)
    win.collision_action.setChecked(False)
    assert not win.collision_service.enabled and not win.settings.collision_enabled()
    row = row_of_dof(win, dof)
    row.spinbox.setValue(v * row.to_display)
    assert current_q(model)[dof] == pytest.approx(v, abs=1e-3)
    win.collision_action.setChecked(True)
    assert win.collision_service.enabled and win.settings.collision_enabled()


def test_drag_tcp_into_floor_stops_above_floor(win):
    model = load(win, "panda")
    svc = win.collision_service
    drag = win.tcp_drag
    t = [0.0]
    drag._clock = lambda: t[0]
    win.drag_tcp_action.setChecked(True)
    hit = False
    for seed in range(30):                  # find a start pose whose straight-down drag is reachable
        a = random_free_q(win, seed=seed)
        put_q(win, a)
        tcp = np.asarray(cb.fk(model)["tcp"])[:3, 3]
        if tcp[2] < 0.25:
            continue
        for z in np.arange(0.2, -0.3, -0.05):          # ... and goes below the floor plane
            t[0] += 1.0
            write_q(model, a)
            target = np.array([tcp[0], tcp[1], z])
            drag.on_tcp_drag(target)
            if drag.reachable and drag.blocked:
                hit = True
                break
        if hit:
            break
    assert hit, "a reachable drag target at or under the floor must be blocked"
    q = np.asarray(current_q(model))
    assert svc.is_free(q)
    assert win._status_label.text().startswith("Blocked")
    assert np.asarray(cb.fk(model)["tcp"])[2, 3] > -0.001, "TCP must stay above the floor"


def test_ik_apply_refuses_colliding_solution(win):
    model = load(win, "ur5")
    a = random_free_q(win, seed=2)
    put_q(win, a)
    dof, v = find_blocked_move(win, a)
    bad = a.copy()
    bad[dof] = v
    ik = win.ik_panel
    ik._on_result([{"q": list(bad), "distance": 0.0, "pos_error": 0.0, "rot_error": 0.0}],
                  ik._generation)
    ik.table.selectRow(0)
    QApplication.processEvents()
    assert ik.apply_button.isEnabled()
    ik.apply_button.click()
    assert current_q(model) == pytest.approx(list(a))          # refused: q back to before
    assert not ik.is_previewing
    assert win._status_label.text().startswith("Blocked:")
    # a free solution is applied
    good = a.copy()
    good[dof] = a[dof] + 0.01
    ik._on_result([{"q": list(good), "distance": 0.0, "pos_error": 0.0, "rot_error": 0.0}],
                  ik._generation)
    ik.table.selectRow(0)
    QApplication.processEvents()
    ik.apply_button.click()
    assert current_q(model) == pytest.approx(list(good))


def _tick(sp, n, dt=0.02):
    for _ in range(n):
        sp._last_tick -= dt
        sp._on_tick()


def test_playback_stops_before_obstacle(win):
    model = load(win, "ur5")
    svc = win.collision_service
    a = random_free_q(win, seed=4)
    # b: another free pose; then a box at the TCP of the pose halfway
    b = random_free_q(win, seed=9)
    while not (np.allclose(a, b) is False and all(
            svc.is_free(a + s * (b - a)) for s in np.linspace(0, 1, 60))):
        b = random_free_q(win, seed=int(np.abs(b).sum() * 1e6) % 10_000)
    put_q(win, a)
    mid = 0.5 * (a + b)
    cm = cb.compile(model)
    tcp_mid = np.asarray(cb.fk_compiled(cm, list(mid))["tcp"])[:3, 3]
    assert win.collision_panel.add_shape("box", {"size": [0.12, 0.12, 0.12]}, list(tcp_mid), name="blk")
    assert not svc.is_free(mid), "the obstacle sits in the path"
    traj = Trajectory([(0.0, a), (2.0, b)], "linear")
    assert win.sim_panel.load_trajectory(traj)
    win._action_view_simulation.setChecked(True)
    QApplication.processEvents()
    sp = win.sim_panel
    sp.play_trajectory()
    t0 = time.perf_counter()
    stops = []
    sp.collision_stopped.connect(lambda t, txt: stops.append((t, txt)))
    for _ in range(150):
        _tick(sp, 1)
        if stops:
            break
    assert stops, "playback must stop at the obstacle"
    t_stop, text = stops[0]
    assert 0.0 < t_stop < 1.0 and "blk" in text
    q = np.asarray(current_q(model))
    assert svc.is_free(q) and not np.allclose(q, b)
    assert sp.engine.clock.state == "PAUSED"
    assert win._status_label.text().startswith("Playback stopped: collision at t =")
    # the stop is at the contact: a little later on the path collides
    s_stop = t_stop / 2.0
    later = a + (s_stop + 4e-3) * (b - a)
    assert not svc.is_free(later)


def test_teach_add_refuses_colliding_pose(win):
    model = load(win, "panda")
    svc = win.collision_service
    a = random_free_q(win, seed=7)
    put_q(win, a)
    tl = win.sim_panel._timeline
    tl.teach_add()
    assert tl.table.rowCount() == 1
    # a colliding pose written straight into the model (not through the guard)
    lo, hi = (np.asarray(v) for v in dof_limits(model))
    rng = np.random.default_rng(11)
    for _ in range(500):
        q = lo + rng.random(len(lo)) * (hi - lo)
        res = svc.check(q, emit=False)
        if not res.ok and any(p[1] != FLOOR_ID for p in res.pairs):
            break
    else:
        pytest.skip("no self-colliding pose found")
    put_q(win, q)
    tl.teach_add()
    assert tl.table.rowCount() == 1                     # refused


def test_gripper_close_stops_at_object(win):
    model = load(win, "panda")
    svc = win.collision_service
    names = [l.name for l in model.links]
    gl = [i for i, n in enumerate(names) if n.endswith("leftfinger")][0]
    gr = [i for i, n in enumerate(names) if n.endswith("rightfinger")][0]
    g = gripper_dofs(model)[0]
    a = centre = None
    for seed in range(60):
        q = random_free_q(win, seed=seed)
        cm = cb.compile(model)
        fr = np.asarray(cb.fk_compiled(cm, list(q))["frames"])
        mid = 0.5 * (fr[gl][:3, 3] + fr[gr][:3, 3])
        axis = fr[gl][:3, 2]                                    # finger frame z = along the fingers
        for off in (0.03, 0.04, 0.05, 0.02, 0.06):
            c = mid + off * axis
            if c[2] > 0.1:
                a, centre = q, c
                break
        if a is not None:
            put_q(win, a)
            win.collision_panel.add_shape("box", {"size": [0.02, 0.02, 0.02]}, list(centre),
                                          name="cube")
            if svc.is_free(a):
                break
            win.collision_panel.remove_shape(0)
            a = None
    assert a is not None, "need a free pose with a cube between the open fingers"
    win.joint_panel.set_gripper(1.0)                        # close
    q = np.asarray(current_q(model))
    assert svc.is_free(q)
    # stopped by the cube: fingers neither fully closed (0.0) nor still open (0.04)
    assert g.closed_value + 0.003 < q[g.dof_index] < g.open_value - 0.003
    assert win.joint_panel._g_slider.value() < 100


def test_pass_through_playback_not_stopped(win):
    model = load(win, "ur5")
    svc = win.collision_service
    a = random_free_q(win, seed=4)
    b = random_free_q(win, seed=8)
    put_q(win, a)
    svc.enabled = False
    traj = Trajectory([(0.0, a), (1.0, b)], "linear")
    assert win.sim_panel.load_trajectory(traj)
    win._action_view_simulation.setChecked(True)
    QApplication.processEvents()
    sp = win.sim_panel
    stops = []
    sp.collision_stopped.connect(lambda t, txt: stops.append(t))
    sp.play_trajectory()
    _tick(sp, 80)
    assert stops == []
    assert np.allclose(current_q(model), b, atol=1e-6)


# ---------------------------------------------------------------- U3.5 red highlight
def test_highlight_follows_collision_service(win):
    load(win, "ur5")
    a = random_free_q(win)
    put_q(win, a)
    dof, v = find_blocked_move(win, a)
    b = a.copy()
    b[dof] = v
    svc = win.collision_service
    rv = win.viewport.robot_view
    res = svc.check(b)                                  # emits collision_changed
    assert not res.ok
    assert rv.collision_links == res.links and rv.collision_links
    assert win._status_label.text().startswith("Collision:")
    red = rv._visual_links
    for i, li in enumerate(red):
        if li in res.links:
            assert rv.visual_color(i) == pytest.approx((1.0, 0.1882, 0.1882), abs=0.01)
            assert rv._visual_actors[i].prop.opacity == 1.0
        else:
            assert rv.visual_color(i) != pytest.approx((1.0, 0.1882, 0.1882), abs=0.01)
    svc.check(a)                                        # free again: back to normal
    assert rv.collision_links == set()
    for i in range(len(rv._visual_actors)):
        assert rv.visual_color(i) == pytest.approx(tuple(rv._visual_rgba[i][:3]), abs=0.005)


def test_highlight_marker_without_meshes(win):
    load(win, "ur5")
    rv = win.viewport.robot_view
    rv.set_show_visuals(False)
    rv.set_collision_highlight({2})
    assert rv.highlight_actor is not None
    rv.set_collision_highlight(set())
    assert rv.highlight_actor is None
    rv.set_show_visuals(True)
    rv.set_collision_highlight({2})
    assert rv.highlight_actor is None            # link 2 has a mesh: the mesh turns red


def test_highlight_cleared_on_collision_off(win):
    load(win, "ur5")
    a = random_free_q(win)
    dof, v = find_blocked_move(win, a)
    b = a.copy()
    b[dof] = v
    win.collision_service.check(b)
    assert win.viewport.robot_view.collision_links
    win.collision_action.setChecked(False)
    assert win.viewport.robot_view.collision_links == set()
