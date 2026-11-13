"""U4a.3 / U4a.4: ghost preview, target panel, play to target."""
import os

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
pytest.importorskip("fcl")

from PySide6.QtCore import QCoreApplication

import core_bridge as cb
from app.analysis.ik_rank import RankedSolution, rank_solutions
from app.importers.urdf import import_urdf
from app.kin_utils import SINGULAR_WARN, current_q
from app.main_window import MainWindow
from app.sample_robots import list_sample_robots
from app.settings import AppSettings
from app.target_preview import GHOST_BAD, GHOST_OK, TargetPreview, target_status

SAMPLES = {p.parent.name: (t, p) for t, p in list_sample_robots()}


@pytest.fixture
def win(qapp, monkeypatch):
    monkeypatch.setattr(AppSettings, "_ORGANIZATION", "RoboticsAnalyzer_test_u4a4")
    monkeypatch.setattr(AppSettings, "_APPLICATION", "RoboticsAnalyzer_test_u4a4")
    AppSettings()._settings.clear()           # a previous test may have saved the dock layout
    w = MainWindow()
    w.settings._settings.clear()
    w.collision_action.setChecked(True)
    w.show()
    yield w
    w.project_controller._is_modified = False
    w.sim_dock.hide()
    w.close()


def _load(win, name):
    path = SAMPLES[name][1]
    win._import_path = str(path)
    win._on_urdf_import_result(import_urdf(str(path)))
    win.task_runner.wait_all(30000)
    QCoreApplication.processEvents()
    return win.project_controller.robot


def _sol(q, valid=True, sigma=0.5, reason=""):
    return RankedSolution(list(q), 0.5, {}, valid, reason, {"sigma_min": sigma})


# ---------------------------------------------------------------- U4a.3 preview

def test_target_status_colours():
    assert target_status([]) == "bad"
    assert target_status([_sol([0], valid=False)]) == "bad"
    assert target_status([_sol([0], sigma=0.5)]) == "ok"
    assert target_status([_sol([0], sigma=SINGULAR_WARN / 2)]) == "warn"
    # invalid first, then a good valid one: status follows the best VALID one
    assert target_status([_sol([0], valid=False), _sol([1], sigma=0.5)]) == "ok"


def test_preview_ghost_colours_and_show_all(win):
    model = _load(win, "ur5")
    rv = win.viewport.robot_view
    pv_ = TargetPreview(lambda: rv, lambda: win.project_controller.robot)
    q0 = current_q(model)
    sols = [_sol(q0), _sol(np.add(q0, 0.3), valid=False, reason="collision")] + \
           [_sol(np.add(q0, 0.1 * k)) for k in range(2, 11)]
    pv_.show(sols, selected=1, show_all=False)
    assert rv.ghost_keys == [("ik", 1)]
    actor = rv.ghost_actors(("ik", 1))[1]
    col = tuple(round(c, 2) for c in actor.GetProperty().GetColor())
    from pyvista import Color
    assert col == tuple(round(c, 2) for c in Color(GHOST_BAD).float_rgb)
    pv_.show(sols, selected=0)
    col = tuple(round(c, 2) for c in rv.ghost_actors(("ik", 0))[1].GetProperty().GetColor())
    assert col == tuple(round(c, 2) for c in Color(GHOST_OK).float_rgb)
    pv_.show(sols, selected=9, show_all=True)           # more than 8: selected one is kept
    assert len(rv.ghost_keys) == 8 and ("ik", 9) in rv.ghost_keys
    sel_op = rv.ghost_actors(("ik", 9))[1].GetProperty().GetOpacity()
    oth_op = rv.ghost_actors(("ik", 0))[1].GetProperty().GetOpacity()
    assert sel_op > oth_op
    # the Ghosts-dock ghosts are not touched by clear()
    rv.add_ghost("named", model, cb.fk(model))
    pv_.clear()
    assert rv.ghost_keys == ["named"]
    rv.clear_ghosts()


def test_preview_ghost_at_solution_pose(win):
    model = _load(win, "ur5")
    rv = win.viewport.robot_view
    cm = cb.compile(model)
    q = np.array([0.3, -1.2, 1.4, -0.9, 1.1, 0.4])
    T = np.asarray(cb.fk_compiled(cm, list(q))["tcp"])
    res = rank_solutions(model, T, current_q(model), win.collision_service, cm=cm)
    pv_ = TargetPreview(lambda: rv, lambda: model)
    pv_.show(res.solutions, selected=0)
    tcp = rv.ghost_tcp(("ik", 0))
    assert np.allclose(tcp[:3, 3], T[:3, 3], atol=1e-4)
    pv_.clear()
    assert rv.ghost_count == 0


# ---------------------------------------------------------------- U4a.4 panel + play

import time                                                             # noqa: E402

from app.kin_utils import write_q                                       # noqa: E402
from app.planning.cartesian import rot_log                              # noqa: E402


def _pump(cond, timeout=60.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        QCoreApplication.processEvents()
        if cond():
            return True
        time.sleep(0.005)
    return cond()


def _put_q(win, q):
    model = win.project_controller.robot
    write_q(model, q)
    win.joint_panel.set_model(model, save_baseline=False)
    win._update_robot_view()
    win.sim_panel.notify_q_edited()


def _open_target(win):
    win._action_view_target.setChecked(True)
    QCoreApplication.processEvents()
    return win.target_panel


Q_TARGET = np.array([0.3, -1.2, 1.4, -0.9, 1.1, 0.4])


def _ur5_with_target(win):
    model = _load(win, "ur5")
    cm = cb.compile(model)
    T = np.asarray(cb.fk_compiled(cm, list(Q_TARGET))["tcp"])
    _put_q(win, np.array([0.0, -1.0, 1.0, -1.0, 1.0, 0.0]))
    tp = _open_target(win)
    tp.auto_solve = False
    assert tp.add_from_tcp(pose=T)
    return model, cm, T, tp


def _solve(tp):
    tp.solve()
    assert tp.is_solving
    assert _pump(lambda: not tp.is_solving), "IK timed out"


def test_panel_dock_registered_and_empty_state(win):
    assert (win._action_view_target, win.target_dock) in win._view_action_docks()
    assert win.target_dock.isHidden()
    tp = _open_target(win)
    assert not win.target_dock.isHidden() and win._action_view_target.isChecked()
    be = tp.button_enabled()
    assert be["solve"] is False and be["new"] is False                   # no robot yet
    _load(win, "ur5")
    be = tp.button_enabled()
    assert be["new"] and not be["play"] and not be["solve"]


def test_add_edit_rename_delete_and_save(win, tmp_path):
    model = _load(win, "ur5")
    tp = _open_target(win)
    tp.auto_solve = False
    assert tp.add_from_tcp()
    tcp = np.asarray(cb.fk(model)["tcp"])
    assert np.allclose(tp.current_pose(), tcp)                      # default = current TCP pose
    tp.set_pose(xyz_mm=[400, 100, 300])
    assert np.allclose(tp.current_pose()[:3, 3], [0.4, 0.1, 0.3])
    tp.rotate("x", 5)
    assert win.project_controller.is_modified
    tp._name.setText("pick")
    tp._name.editingFinished.emit()
    assert tp.store.names() == ["pick"]
    tp.add_from_tcp()
    assert len(tp.store) == 2 and tp._combo.count() == 2
    proj = win.project_controller.current_project
    from app.project import serializer
    f = tmp_path / "t.robotproj"
    serializer.save(proj, f)
    q = serializer.load(f)
    assert q.targets == proj.targets
    tp.delete_target()
    assert len(tp.store) == 1


def test_solve_ranks_and_previews(win):
    model, cm, T, tp = _ur5_with_target(win)
    rv = win.viewport.robot_view
    assert tp.controller.visible and len(tp.controller._actors) == 2
    _solve(tp)
    sols = tp.solutions
    assert len(sols) >= 2 and sols[0].valid
    assert tp.table.rowCount() == len(sols)
    assert tp.controller.status in ("ok", "warn")
    assert tp.selected_row() == 0 and rv.ghost_keys == [("ik", 0)]
    tp.select_row(1)
    assert rv.ghost_keys == [("ik", 1)]
    tp._chk_all.setChecked(True)
    assert len(rv.ghost_keys) == min(8, len(sols))
    assert tp.button_enabled()["play"] == sols[1].valid
    # hiding the dock removes the ghosts and the 3D target
    win._action_view_target.setChecked(False)
    QCoreApplication.processEvents()
    assert rv.ghost_keys == [] and not tp.controller.visible
    win._action_view_target.setChecked(True)
    QCoreApplication.processEvents()
    assert rv.ghost_keys and tp.controller.visible


def test_unreachable_target_is_red(win):
    model, cm, T, tp = _ur5_with_target(win)
    tp.set_pose(xyz_mm=[3000, 0, 500])
    _solve(tp)
    assert not any(s.valid for s in tp.solutions)
    assert tp.controller.status == "bad"
    assert "No valid" in tp.status_text
    assert not tp.button_enabled()["play"]


def test_weights_change_order(win):
    model, cm, T, tp = _ur5_with_target(win)
    _solve(tp)
    tp.set_weights((0, 0, 1, 0))
    assert tp.weights == (0.0, 0.0, 1.0, 0.0)
    assert win.settings.ik_rank_weights() == (0.0, 0.0, 1.0, 0.0)
    v = [s for s in tp.solutions if s.valid]
    assert v[0].raw["sigma_min"] == pytest.approx(max(s.raw["sigma_min"] for s in v))
    tp.set_weights((1, 0, 0, 0))
    v = [s for s in tp.solutions if s.valid]
    assert v[0].raw["distance"] == pytest.approx(min(s.raw["distance"] for s in v))


def _play_through(win, tp, timeout=60.0):
    got = []
    tp.trajectory_ready.connect(lambda traj, info: got.append(traj))
    tp.play_to_target()
    assert tp.is_planning
    assert _pump(lambda: not tp.is_planning, timeout)
    assert got, tp.motion_status_text
    sp = win.sim_panel
    pl = sp.player
    assert pl is not None
    stops = []
    sp.collision_stopped.connect(lambda t, txt: stops.append(txt))
    guard = 0
    while not pl.finished and guard < 100000:
        pl.advance(0.02)
        guard += 1
    sp._emit_state()
    return got[0], stops


def test_play_to_target_end_q_and_tcp(win):
    model, cm, T, tp = _ur5_with_target(win)
    _solve(tp)
    tp.select_row(0)
    sol = tp.solutions[0]
    traj, stops = _play_through(win, tp)
    assert not stops
    last = np.array(traj.waypoints[-1].q)
    assert np.allclose(last, sol.q, atol=1e-9)
    assert np.allclose(current_q(model), sol.q, atol=1e-9)
    tcp = np.asarray(cb.fk(model)["tcp"])
    assert np.linalg.norm(tcp[:3, 3] - T[:3, 3]) < 1e-4
    assert np.linalg.norm(rot_log(tcp[:3, :3].T @ T[:3, :3])) < 1e-3


def test_play_with_approach_line_is_straight(win):
    model, cm, T, tp = _ur5_with_target(win)
    _solve(tp)
    tp.select_row(0)
    tp.set_approach(True, 100, retreat=True)
    traj, stops = _play_through(win, tp)
    assert not stops
    res = tp.last_motion
    assert set(res.segments) == {"move", "approach", "retreat"}
    a0, a1 = res.segments["approach"]
    r0, r1 = res.segments["retreat"]
    pre = T[:3, 3] - 0.1 * T[:3, 2]
    ts = np.linspace(a0, a1, 200)
    pts = np.array([np.asarray(cb.fk_compiled(cm, list(traj.q(t)))["tcp"])[:3, 3] for t in ts])
    d = pts - pre
    along = d @ T[:3, 2]
    perp = np.linalg.norm(d - np.outer(along, T[:3, 2]), axis=1)
    assert perp.max() < 1e-4, perp.max()
    assert np.allclose(pts[0], pre, atol=1e-4) and np.allclose(pts[-1], T[:3, 3], atol=1e-4)
    assert np.all(np.diff(along) >= -1e-6)                    # monotone toward the target
    assert np.allclose(traj.q(a1), tp.solutions[0].q, atol=1e-9)
    assert np.allclose(np.asarray(cb.fk_compiled(cm, list(traj.q(r1)))["tcp"])[:3, 3], pre, atol=1e-4)


def test_play_blocked_by_object_is_planned_around(win):
    model, cm, T, tp = _ur5_with_target(win)
    _solve(tp)
    tp.select_row(0)
    start = np.array(current_q(model))
    goal = np.array(tp.solutions[0].q)
    svc = win.collision_service
    mids = [start + t * (goal - start) for t in (0.4, 0.5, 0.6)]
    placed = False
    for k in (6, 7, 8, 5):                     # a frame where the box blocks the middle only
        spot = np.asarray(cb.fk_compiled(cm, list(mids[1]))["frames"][k])[:3, 3]
        win.collision_panel.add_shape("box", {"size": [0.06, 0.06, 0.06]}, list(spot), name="obstacle")
        QCoreApplication.processEvents()
        win._sync_env_objects()
        if svc.is_free(start) and svc.is_free(goal) and not svc.is_free(mids[1]):
            placed = True
            break
        win.collision_panel.remove_shape(0)
    assert placed, "no blocking box position found"
    traj, stops = _play_through(win, tp)
    assert not stops
    assert len(traj.waypoints) > 3
    from app.planning.collision import CollisionWorld, shapes_from_project
    proj = win.project_controller.current_project
    fresh = CollisionWorld(model, [s for s in shapes_from_project(proj) if s.link >= 0], proj.collision)
    ts = np.append(np.arange(0.0, traj.duration, 0.02), traj.duration)
    assert not np.asarray(fresh.check_batch(np.array([traj.q(t) for t in ts]))).any()
    assert np.allclose(current_q(model), goal, atol=1e-6)


def test_play_refused_for_invalid_and_without_selection(win):
    model, cm, T, tp = _ur5_with_target(win)
    tp.play_to_target()
    assert not tp.is_planning
    _solve(tp)
    inv = [k for k, s in enumerate(tp.solutions) if not s.valid]
    assert inv
    tp.select_row(inv[0])
    assert not tp.button_enabled()["play"]
    tp.play_to_target()
    assert not tp.is_planning


def test_place_on_surface_then_fields(win):
    _load(win, "ur5")
    tp = _open_target(win)
    tp.auto_solve = False
    tp.place_on_surface([0.4, 0.1, 0.2], [0, 0, 1])
    T = tp.current_pose()
    assert np.allclose(T[:3, 2], [0, 0, -1]) and np.allclose(T[:3, 3], [0.4, 0.1, 0.2])
    assert np.allclose([b.value() for b in tp._xyz], [400, 100, 200], atol=0.06)


def test_model_change_clears_results(win):
    model, cm, T, tp = _ur5_with_target(win)
    _solve(tp)
    assert tp.table.rowCount() > 0
    _load(win, "panda")
    assert tp.table.rowCount() == 0 and win.viewport.robot_view.ghost_keys == []
