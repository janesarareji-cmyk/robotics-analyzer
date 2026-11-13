"""U4a stage milestone: 3D target, ranked IK, ghost preview, play to target (offscreen MainWindow).

panda + box in the scene -> target placed on the box top by a click (real VTK picker) -> ranked IK
-> pick the 2nd valid solution (ghost preview) -> play with approach and retreat -> dense recheck
with a fresh collision world, TCP error < 1e-4 m, straight approach line -> save / reopen: targets
and the target pose equal.
"""
import os
import time
from unittest import mock

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
pytest.importorskip("fcl")

from PySide6.QtCore import QCoreApplication

import core_bridge as cb
from app.kin_utils import current_q
from app.main_window import MainWindow
from app.planning.collision import CollisionWorld, shapes_from_project
from app.project.serializer import save
from app.settings import AppSettings

from .test_live_blocking import load, put_q

ORG = "RoboticsAnalyzer_test_u4amilestone"
RESULTS: dict = {}
BOX = (0.5, 0.0, 0.15)          # centre; size 0.2 x 0.2 x 0.3 -> top at z = 0.30


@pytest.fixture
def win(qapp, monkeypatch):
    monkeypatch.setattr(AppSettings, "_ORGANIZATION", ORG)
    monkeypatch.setattr(AppSettings, "_APPLICATION", ORG)
    AppSettings()._settings.clear()
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
        time.sleep(0.005)
    return cond()


def box_top_pixel(win):
    """Display pixel of the box top centre (camera looks at it from the front, above)."""
    import vtk
    pl = win.viewport.plotter
    QCoreApplication.processEvents()
    pl.render_window.SetSize(800, 600)            # offscreen: the window has no size yet
    pl.camera.focal_point = (BOX[0], BOX[1], 0.3)
    pl.camera.position = (1.1, -0.7, 1.0)
    pl.render()
    c = vtk.vtkCoordinate()
    c.SetCoordinateSystemToWorld()
    c.SetValue(BOX[0], BOX[1], 0.3)
    return c.GetComputedDoubleDisplayValue(pl.renderer)


def test_u4a_milestone(win, tmp_path):
    model = load(win, "panda")
    proj = win.project_controller.current_project
    svc = win.collision_service
    assert win.collision_panel.add_shape("box", {"size": [0.2, 0.2, 0.3]}, list(BOX), name="block")
    QCoreApplication.processEvents()
    win._sync_env_objects()

    # ---- 1. target placed on the box top by a click
    win._action_view_target.setChecked(True)
    QCoreApplication.processEvents()
    tp = win.target_panel
    tp.auto_solve = False
    x, y = box_top_pixel(win)
    tp._btn_click.setChecked(True)
    assert tp.controller.pick_mode
    tp.controller._on_click((x, y))                  # what a left click without drag does
    T = tp.current_pose()
    assert T is not None and len(tp.store) == 1
    assert np.allclose(T[:3, 2], [0, 0, -1], atol=1e-6), "tool z must point into the box top"
    assert abs(T[2, 3] - (0.30 + 0.010)) < 1e-3, T[2, 3]                  # top + 10 mm lift
    assert np.linalg.norm(T[:2, 3] - np.array(BOX[:2])) < 0.02
    tp._btn_click.setChecked(False)

    # ---- 2. ranked IK: all valid solutions free of the box, scores sorted
    tp.solve()
    assert pump_until(lambda: not tp.is_solving)
    sols = tp.solutions
    valid = [s for s in sols if s.valid]
    assert len(valid) >= 3, tp.status_text
    assert [s.total for s in valid] == sorted([s.total for s in valid], reverse=True)
    for s in valid:
        assert svc.is_free(np.asarray(s.q))
        tcp = np.asarray(cb.fk_compiled(cb.compile(model), s.q)["tcp"])
        assert np.linalg.norm(tcp[:3, 3] - T[:3, 3]) < 1e-4
    RESULTS.update(n_solutions=len(sols), n_valid=len(valid), rank_ms=round(tp.last_rank_ms, 1),
                   status=tp.status_text)
    assert tp.controller.status in ("ok", "warn")

    # ---- 3. second solution, ghost preview
    tp.select_row(1)
    rv = win.viewport.robot_view
    assert rv.ghost_keys == [("ik", 1)]
    assert np.allclose(rv.ghost_tcp(("ik", 1))[:3, 3], T[:3, 3], atol=1e-4)
    chosen = np.array(tp.solutions[1].q)

    # ---- 4. play with approach + retreat
    tp.set_approach(True, 100, retreat=True)
    stops = []
    win.sim_panel.collision_stopped.connect(lambda t, txt: stops.append(txt))
    tp.play_to_target()
    assert pump_until(lambda: not tp.is_planning), tp.motion_status_text
    res = tp.last_motion
    assert res is not None and res.ok, tp.motion_status_text
    sp = win.sim_panel
    pl = sp.player
    assert pl is not None
    guard = 0
    while not pl.finished and guard < 100000:
        pl.advance(0.02)
        guard += 1
    sp._emit_state()
    assert pl.finished and not stops, stops

    # ---- 5. dense recheck (fresh world, 1 cm joint steps and 20 ms samples)
    traj = res.trajectory
    fresh = CollisionWorld(model, [s for s in shapes_from_project(proj) if s.link >= 0], proj.collision)
    ts = np.append(np.arange(0.0, traj.duration, 0.01), traj.duration)
    qs = np.array([traj.q(t) for t in ts])
    bad = np.asarray(fresh.check_batch(qs))
    assert not bad.any(), f"collision at t = {ts[int(np.argmax(bad))]:.2f} s"
    wp = np.array([w.q for w in traj.waypoints])
    edge = []
    for p0, p1 in zip(wp[:-1], wp[1:]):
        n = max(2, int(np.ceil(np.abs(p1 - p0).max() / 0.01)) + 1)
        edge.extend(np.linspace(p0, p1, n))
    assert not np.asarray(fresh.check_batch(np.array(edge))).any()

    # ---- 6. TCP error and straight approach
    a0, a1 = res.segments["approach"]
    q_at_target = np.array(traj.q(a1))
    assert np.allclose(q_at_target, chosen, atol=1e-9)
    cm = cb.compile(model)
    tcp = np.asarray(cb.fk_compiled(cm, list(q_at_target))["tcp"])
    err = float(np.linalg.norm(tcp[:3, 3] - T[:3, 3]))
    assert err < 1e-4
    pre = T[:3, 3] - 0.1 * T[:3, 2]
    pts = np.array([np.asarray(cb.fk_compiled(cm, list(traj.q(t)))["tcp"])[:3, 3]
                    for t in np.linspace(a0, a1, 200)])
    d = pts - pre
    along = d @ T[:3, 2]
    perp = float(np.linalg.norm(d - np.outer(along, T[:3, 2]), axis=1).max())
    assert perp < 1e-4
    r1 = res.segments["retreat"][1]
    q_end = np.array(traj.q(r1))
    assert np.linalg.norm(np.asarray(cb.fk_compiled(cm, list(q_end))["tcp"])[:3, 3] - pre) < 1e-4
    RESULTS.update(tcp_err_m=err, line_dev_m=perp, dense_samples=len(qs) + len(edge),
                   duration_s=round(traj.duration, 2), segments={k: round(b - a, 2)
                                                                 for k, (a, b) in res.segments.items()})

    # ---- 7. save / reopen: targets equal
    before = proj.targets.to_dict()
    pc = win.project_controller
    path = tmp_path / "u4a.robotproj"
    save(pc.current_project, path)
    pc._is_modified = False
    pc.close_project(win)
    with mock.patch("app.main_window.QMessageBox.warning"):
        assert pc.open_project(win, str(path))
    QCoreApplication.processEvents()
    assert pc.current_project.targets.to_dict() == before
    assert win.target_panel.store.names() == ["Target 1"]
    assert np.allclose(win.target_panel.current_pose(), T)


def test_zz_print_numbers():
    print("U4a milestone numbers:", RESULTS)
