"""U3.9 stage milestone: real collision end to end through an offscreen MainWindow.

ur5 -> automatic ACM -> box in the scene -> jog into the box is blocked -> Drag TCP into the
floor is blocked -> plan around the box -> play -> dense recheck (a fresh CollisionWorld, 1 cm
joint steps) finds no collision -> save/reopen equal -> xarm5_gripper grasps a cube, moves,
releases -> save/reopen equal.
"""
import os
import time
from unittest import mock

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
pytest.importorskip("fcl")

from PySide6.QtCore import QCoreApplication
from PySide6.QtWidgets import QApplication

import core_bridge as cb
from app.kin_utils import current_q, dof_limits, write_q
from app.main_window import MainWindow
from app.planning.collision import CollisionWorld, shapes_from_project
from app.project.serializer import save
from app.settings import AppSettings

from .test_grasp import cfg, cube_world, place_cube
from .test_live_blocking import load, put_q, random_free_q, row_of_dof

ORG = "RoboticsAnalyzer_test_u3milestone"
RESULTS: dict = {}


@pytest.fixture
def win(qapp, monkeypatch):
    monkeypatch.setattr(AppSettings, "_ORGANIZATION", ORG)
    monkeypatch.setattr(AppSettings, "_APPLICATION", ORG)
    w = MainWindow()
    w.settings._settings.clear()
    w.collision_action.setChecked(True)
    w.show()
    yield w
    w.project_controller._is_modified = False
    w.sim_dock.hide()
    w.close()


def pump_until(cond, timeout=90.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        QCoreApplication.processEvents()
        if cond():
            return True
        time.sleep(0.01)
    return cond()


def snapshot(win):
    p = win.project_controller.current_project
    q = [float(v) for v in current_q(win.project_controller.robot)]
    return {"collision": p.collision.to_dict(), "q": q}


def reopen(win, tmp_path, name):
    pc = win.project_controller
    path = tmp_path / f"{name}.robotproj"
    save(pc.current_project, path)
    pc._is_modified = False
    pc.close_project(win)
    with mock.patch("app.main_window.QMessageBox.warning"):
        assert pc.open_project(win, str(path))
    QCoreApplication.processEvents()


def env_hits(svc, q):
    return svc.check(np.asarray(q, float), emit=False).env_ids


def test_u3_milestone(win, tmp_path):
    # ---- 1. ur5 + automatic ACM
    model = load(win, "ur5")
    proj = win.project_controller.current_project
    svc = win.collision_service
    assert proj.collision.auto_info.get("n_samples", 0) > 0, "auto ACM must have run on import"
    assert proj.collision.auto, "auto ACM must hold pairs"
    RESULTS["acm_pairs"] = len(proj.collision.auto)

    # ---- 2. start / goal and a box between them on the straight joint-space line
    a = b = boxc = None
    for sa in range(60):
        qa = random_free_q(win, seed=100 + sa)
        qb = random_free_q(win, seed=200 + sa)
        if np.linalg.norm(qa - qb) < 1.5:
            continue
        if not all(svc.is_free(q) for q in np.linspace(qa, qb, 80)):
            continue
        mid = 0.5 * (qa + qb)
        tcp = np.asarray(cb.fk_compiled(cb.compile(model), list(mid))["tcp"])[:3, 3]
        if tcp[2] < 0.15:
            continue
        a, b, boxc = qa, qb, tcp
        break
    assert a is not None, "no start/goal pair with a free straight line"
    put_q(win, a)
    assert win.collision_panel.add_shape("box", {"size": [0.12] * 3}, boxc.tolist(), name="blk")
    assert len(proj.collision.environment) == 1
    assert svc.is_free(a) and svc.is_free(b)
    line_hits = sum(1 for q in np.linspace(a, b, 200) if not svc.is_free(q))
    assert line_hits > 0, "box must block the straight line"
    RESULTS["line_hits_of_200"] = line_hits

    # ---- 3. jog into the box: slider stops, robot stays free, status says Blocked
    jog = None
    lo, hi = dof_limits(model)
    for dof in range(len(a)):
        for v in np.linspace(lo[dof], hi[dof], 61):
            q = a.copy()
            q[dof] = v
            if abs(v - a[dof]) > 0.3 and env_hits(svc, q):
                jog = (dof, float(v))
                break
        if jog:
            break
    assert jog is not None, "no single-joint jog reaches the box"
    dof, v = jog
    row = row_of_dof(win, dof)
    row.spinbox.setValue(v * row.to_display)
    q = np.asarray(current_q(model))
    assert svc.is_free(q), "jog must end collision free"
    assert abs(q[dof] - v) > 1e-3
    assert win._status_label.text().startswith("Blocked:")
    RESULTS["jog_status"] = win._status_label.text()

    # ---- 4. drag TCP into the floor: blocked, TCP stays above the floor
    drag = win.tcp_drag
    t = [0.0]
    drag._clock = lambda: t[0]
    win.drag_tcp_action.setChecked(True)
    hit = False
    for seed in range(40):
        s0 = random_free_q(win, seed=seed)
        put_q(win, s0)
        tcp0 = np.asarray(cb.fk(model)["tcp"])[:3, 3]
        if tcp0[2] < 0.25:
            continue
        for z in np.arange(0.2, -0.3, -0.05):
            t[0] += 1.0
            write_q(model, s0)
            drag.on_tcp_drag(np.array([tcp0[0], tcp0[1], z]))
            if drag.reachable and drag.blocked:
                hit = True
                break
        if hit:
            break
    assert hit, "a drag target under the floor must be blocked"
    assert svc.is_free(np.asarray(current_q(model)))
    assert win._status_label.text().startswith("Blocked")
    assert np.asarray(cb.fk(model)["tcp"])[2, 3] > -0.001
    win.drag_tcp_action.setChecked(False)

    # ---- 5. plan around the box with the existing planner on the hull shapes
    put_q(win, a)
    pp = win.planning_panel
    pp.set_vector("start", [float(x) for x in a])
    pp.set_vector("goal", [float(x) for x in b])
    pp._planner.setCurrentIndex(0)
    pp._seed.setValue(5)
    pp._time_limit.setValue(60.0)
    pp.plan()
    assert pump_until(lambda: not pp.is_running), "planner timed out"
    assert pp.result is not None and pp.result.success, pp.status_text
    path = np.asarray(pp.result.path)
    assert len(path) >= 3, "straight line is blocked: path needs a via point"
    RESULTS.update(path_wps=len(path), plan_ms=pp.result.time * 1000)
    pp.send_to_trajectory()

    # ---- 6. play it: no collision stop, ends at the goal
    sp = win.sim_panel
    win._action_view_simulation.setChecked(True)
    QApplication.processEvents()
    stops = []
    sp.collision_stopped.connect(lambda tt, txt: stops.append(txt))
    sp.set_mode("kinematic")
    sp.play_trajectory()
    pl = sp.player
    assert pl is not None
    guard = 0
    while not pl.finished and guard < 100000:
        pl.advance(0.02)
        guard += 1
    assert pl.finished and not stops, stops
    sp._emit_state()
    assert np.allclose(current_q(model), b, atol=1e-6)

    # ---- 7. dense recheck with a fresh world (not the service's)
    fresh = CollisionWorld(model, [s for s in shapes_from_project(proj) if s.link >= 0],
                           proj.collision)
    tr = pp.timed.trajectory
    ts = np.append(np.arange(0.0, tr.duration, 0.01), tr.duration)
    qs = np.array([tr.q(x) for x in ts])
    bad = np.asarray(fresh.check_batch(qs))
    assert not bad.any(), f"trajectory collides at {ts[int(np.argmax(bad))]:.2f} s"
    edge = []
    for p0, p1 in zip(path[:-1], path[1:]):
        n = max(2, int(np.ceil(np.linalg.norm(p1 - p0) / 0.01)) + 1)
        edge.extend(np.linspace(p0, p1, n))
    assert not np.asarray(fresh.check_batch(np.array(edge))).any()
    RESULTS["dense_samples"] = len(qs) + len(edge)

    # ---- 8. save / reopen equal (ur5 scene)
    snap_ur5 = snapshot(win)
    reopen(win, tmp_path, "ur5")
    assert snapshot(win) == snap_ur5
    assert len(win.project_controller.current_project.collision.environment) == 1

    # ---- 9. grasp with xarm5_gripper: close attaches, move, save/reopen, release
    model = load(win, "xarm5_gripper")
    svc = win.collision_service
    place_cube(win)
    win.joint_panel.set_gripper(1.0)
    c = cfg(win)
    assert [e.name for e in c.attached] == ["cube"] and c.environment == []
    assert win._status_label.text() == "Grasped cube"
    link = c.attached[0].attached_to
    w0 = cube_world(win, link)
    q1 = np.array(current_q(model))
    q1[0] += 0.4
    q1[1] += 0.2
    put_q(win, q1)
    assert svc.is_free(q1)
    w1 = cube_world(win, link)
    assert not np.allclose(w0[:3, 3], w1[:3, 3])
    snap_grasp = snapshot(win)
    reopen(win, tmp_path, "grasp")
    assert snapshot(win) == snap_grasp, "attached cube must survive save/reopen"
    assert len(cfg(win).attached) == 1
    win.joint_panel.set_gripper(0.0)
    assert cfg(win).attached == [] and len(cfg(win).environment) == 1
    assert win._status_label.text() == "Released cube"
    assert win.collision_service.is_free(np.asarray(current_q(win.project_controller.robot)))


def test_zz_print_numbers():
    print("U3 milestone numbers:", RESULTS)
