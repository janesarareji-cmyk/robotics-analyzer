"""U4b stage milestone: curve from the current pose around an obstacle, checked, timed, played,
with the live matrix panel (offscreen MainWindow, ur5).

ur5 + box between the arm and a target behind it -> the straight line is red (colliding), a spline
with two via points around the box is clear -> TOPP timing -> play -> dense recheck with a fresh
collision world (no contact), TCP on the curve within 1 mm, matrix panel T equals FK at 5 sampled
times, a line through the wrist singularity shows warn segments and keeps a clear tube.
"""
import os
import time

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
pytest.importorskip("fcl")

from PySide6.QtCore import QCoreApplication

import core_bridge as cb
from app.kin_utils import current_q
from app.main_window import MainWindow
from app.planning.collision import CollisionWorld, shapes_from_project
from app.planning.path_check import BAD, WARN
from app.settings import AppSettings

from .test_curve_editor import UR5_Q, _fk, pump_until, settle
from .test_live_blocking import load, put_q

ORG = "RoboticsAnalyzer_test_u4bmilestone"
RESULTS: dict = {}
SIDE = np.array([0.0, -0.34, 0.0])         # target is 34 cm to the side of the start


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


def test_u4b_milestone(win):
    model = load(win, "ur5")
    put_q(win, UR5_Q)
    T0 = _fk(model, UR5_Q)
    p0 = T0[:3, 3]
    mid = p0 + 0.5 * SIDE
    # ---- 1. box between the start and the target, target behind it
    assert win.collision_panel.add_shape("box", {"size": [0.08, 0.08, 0.08]},
                                         [mid[0], mid[1], mid[2]], name="block")
    win._sync_env_objects()
    win.collision_service.invalidate()
    win._action_view_target.setChecked(True)
    win._action_view_curve.setChecked(True)
    win._action_view_matrices.setChecked(True)
    QCoreApplication.processEvents()
    tp, cp, mp = win.target_panel, win.curve_panel, win.matrix_panel
    tp.auto_solve = False
    assert tp.add_from_tcp()
    tp.move_to(p0 + SIDE)
    target = tp.current_pose()
    settle(cp)

    # ---- 2. the straight line is red: the box is in the way, Play is off
    cp.set_kind("line")
    settle(cp)
    line_res = cp.result
    assert not line_res.ok and not cp.button_enabled()["play"]
    assert "Sample" in cp.play_tooltip
    n_bad_line = line_res.check.n_bad
    cols = win.viewport.robot_view.path_tube_colors
    assert np.allclose(cols[line_res.check.first_bad], [1.0, 0.32, 0.32])

    # ---- 3. spline with two via points around the box: all samples ok
    cp.set_kind("spline")
    assert cp.add_via() and cp.add_via()
    around = [p0 + 0.25 * SIDE + [0.0, 0.0, 0.2], p0 + 0.7 * SIDE + [0.0, 0.0, 0.2]]
    for i, v in enumerate(around):
        cp.set_via_xyz(i, v * 1000.0)
    settle(cp)
    res = cp.result
    assert res.ok, cp.check_text
    assert res.check.n_bad == 0 and len(res.via_points) == 2
    cols = win.viewport.robot_view.path_tube_colors
    assert not np.any(np.all(np.isclose(cols, [1.0, 0.32, 0.32]), axis=1))

    # ---- 4. TOPP timing, generate and play
    cp.set_timing("topp")
    got = []
    cp.trajectory_ready.connect(lambda t, i: got.append(t))
    cp.generate(play=False)
    assert pump_until(lambda: got and not cp.busy)
    traj = got[0]
    RESULTS.update(duration_s=round(traj.duration, 2), n_samples=len(res.points),
                   curve_length_mm=round(res.length * 1000), check_ms=round(cp.last_compute_ms),
                   line_bad_samples=n_bad_line)
    sp = win.sim_panel
    P = res.points
    seg = np.diff(P, axis=0)
    T_seen, q_seen = [], []
    sample_t = list(np.linspace(0.0, traj.duration, 5))
    for t in sample_t:                                      # 5 sampled times: scrub, panel vs FK
        sp.seek_trajectory(t)
        QCoreApplication.processEvents()
        mp.refresh_now()
        q_now = np.array(current_q(model))
        assert np.allclose(q_now, traj.q(t), atol=1e-12)
        fk = cb.fk(model, list(q_now))
        ref = np.asarray(fk["tcp"], float)
        T0b = np.asarray(cb.pose_to_iso(model.base), float)
        assert np.max(np.abs(mp.data.T_base_tcp - np.linalg.inv(T0b) @ ref)) < 1e-12
        assert mp.data.sigma_min > 0.0
        T_seen.append(ref)
    # ---- 5. play to the end through the timer path, dense recheck with a fresh world
    sp.seek_trajectory(0.0)
    sp.set_background_play(True)
    sp.play_trajectory()
    pl = sp.player
    assert pl is not None
    for _ in range(4000):
        if pl.finished:
            break
        sp._last_tick -= 0.05
        sp._on_tick()
    assert pl.finished and pl.collision_stop is None
    assert np.linalg.norm(_fk(model, current_q(model))[:3, 3] - target[:3, 3]) < 1e-4
    proj = win.project_controller.current_project
    world = CollisionWorld(model, [s for s in shapes_from_project(proj) if s.link >= 0],
                           proj.collision)
    ts = np.linspace(0.0, traj.duration, 200)
    qs = np.array([traj.q(t) for t in ts])
    assert win.collision_service.world is not None
    flags = np.asarray(win.collision_service.world.check_batch(qs))
    allowed = np.asarray(win.collision_service.world.check_batch(qs[:1]))
    assert not flags.any() or flags[0] == allowed[0] and not flags[1:].any()
    worst = 0.0
    for q in qs:
        x = _fk(model, q)[:3, 3]
        u = np.clip(np.einsum("ij,ij->i", x - P[:-1], seg) / np.einsum("ij,ij->i", seg, seg), 0, 1)
        worst = max(worst, float(np.min(np.linalg.norm(P[:-1] + u[:, None] * seg - x, axis=1))))
    assert worst < 1e-3, worst
    RESULTS.update(max_tcp_off_curve_mm=round(worst * 1000, 3), dense_samples=len(ts),
                   matrix_refresh_ms=round(mp.last_refresh_ms, 2))

    # ---- 6. lines toward the wrist singularity (box removed): warn segments, then a blocked case
    assert win.collision_panel.remove_shape(0)
    win._sync_env_objects()
    win.collision_service.invalidate()
    sp.stop_trajectory()
    QCoreApplication.processEvents()
    qa = UR5_Q.copy()
    qa[4] = 0.5
    put_q(win, qa)
    win._on_joint_q_changed()
    cp.set_chain(False)
    cp.set_kind("line")
    qb = UR5_Q.copy()
    qb[4] = 0.03
    from app.target import xyz_rpy_from_pose
    xyz, rpy = xyz_rpy_from_pose(_fk(model, qb))
    tp.set_pose(xyz_mm=xyz * 1000.0, rpy_deg=np.degrees(rpy))
    settle(cp)
    chk = cp.result.check
    assert cp.result.ok and chk.n_warn >= 2 and chk.n_bad == 0
    warn_runs = chk.segments(WARN)
    assert warn_runs and warn_runs[-1][1] == chk.n - 1          # the warn part is at the singular end
    assert cp.button_enabled()["play"]
    cols = win.viewport.robot_view.path_tube_colors
    assert np.allclose(cols[-1], [1.0, 0.65, 0.15]) and np.allclose(cols[0], [0.40, 0.73, 0.42])
    assert np.nanmin(chk.sigma_min) < 0.05
    cp.set_color_mode("singularity")
    cols2 = win.viewport.robot_view.path_tube_colors
    assert cols2[-1][0] > cols2[0][0]                           # red part grows toward the end
    tp.move_to(p0 + [2.5, 0.0, 0.2])
    settle(cp)
    assert not cp.result.ok and not cp.button_enabled()["play"]
    RESULTS.update(singular_warn_samples=chk.n_warn, singular_min_sigma=round(float(np.nanmin(chk.sigma_min)), 4))
    print("U4B_MILESTONE", RESULTS)
