"""U4b.3: curve build (legs, chain, timing) and the Curve panel."""
import numpy as np
import pytest

pytest.importorskip("fcl")

import core_bridge as cb
from app.planning.curve import (
    build_curve, default_arc_via, longest_segment_midpoint, tcp_at_time, timed_trajectory,
    trajectory_speeds,
)

from .test_collision_service import service

UR5_Q = np.array([0.0, -1.4, 1.6, -1.8, -1.5, 0.0])


def _fk(model, q):
    return np.asarray(cb.fk_compiled(cb.compile(model), list(q))["tcp"], float)


def _setup():
    svc, proj = service("ur5")
    model = proj.robot_model
    T0 = _fk(model, UR5_Q)
    return svc, model, T0


def _shift(T, d):
    T = T.copy()
    T[:3, 3] += d
    return T


def test_line_arc_spline_all_reach_the_target():
    svc, model, T0 = _setup()
    T1 = _shift(T0, [0.0, 0.10, 0.05])
    mid = longest_segment_midpoint(T0[:3, 3], [], T1[:3, 3])
    via = mid + np.array([0.0, 0.0, 0.04])
    for kind in ("line", "arc", "spline"):
        res = build_curve(model, UR5_Q, [(T1, [via])], kind, 0.1, svc)
        assert res.ok, (kind, res.problem_text())
        assert np.allclose(res.q[0], UR5_Q)
        tcp = _fk(model, res.q[-1])
        assert np.linalg.norm(tcp[:3, 3] - T1[:3, 3]) < 1e-4
        assert res.duration == pytest.approx(res.length / 0.1, rel=0.1)
        assert len(res.points) == len(res.q) == len(res.check.status)
    arc_res = build_curve(model, UR5_Q, [(T1, [via])], "arc", 0.1, svc)
    P = arc_res.points                                    # the arc passes the via point (between samples)
    seg = np.diff(P, axis=0)
    u = np.clip(np.einsum("ij,ij->i", via - P[:-1], seg) / np.einsum("ij,ij->i", seg, seg), 0, 1)
    assert np.min(np.linalg.norm(P[:-1] + u[:, None] * seg - via, axis=1)) < 1e-3
    assert arc_res.via_points and np.allclose(arc_res.via_points[0], via)


def test_arc_without_via_gets_default_and_line_ignores_via():
    svc, model, T0 = _setup()
    T1 = _shift(T0, [0.0, 0.10, 0.0])
    v = default_arc_via(T0, T1)
    assert np.linalg.norm(v - 0.5 * (T0[:3, 3] + T1[:3, 3])) > 0.01
    res = build_curve(model, UR5_Q, [(T1, [])], "arc", 0.1, svc)
    assert res.ok and len(res.via_points) == 1
    res = build_curve(model, UR5_Q, [(T1, [v])], "line", 0.1, svc)
    assert res.via_points == []
    assert np.max(np.abs(res.points[:, 2] - T0[2, 3])) < 1e-9


def test_chain_joins_legs_and_adds_cycle_time():
    svc, model, T0 = _setup()
    B = _shift(T0, [0.0, 0.08, 0.0])
    C = _shift(T0, [0.0, 0.08, 0.06])
    res = build_curve(model, UR5_Q, [(B, []), (C, [])], "line", 0.1, svc)
    assert res.ok and len(res.legs) == 2
    assert np.linalg.norm(res.legs[1].T0[:3, 3] - B[:3, 3]) < 1e-12
    (a0, b0), (a1, b1) = res.leg_ranges
    assert b0 == a1 and b1 == len(res.points) - 1
    assert np.all(np.diff(res.t) > 0)
    assert res.duration == pytest.approx(0.14 / 0.1, rel=0.08)
    one = build_curve(model, UR5_Q, [(B, [])], "line", 0.1, svc)
    assert res.duration > one.duration
    # the leg 2 start is the leg 1 end: q continuous
    assert np.max(np.abs(np.diff(res.q, axis=0))) < 0.5


def test_unreachable_stop_is_reported_not_raised():
    svc, model, T0 = _setup()
    far = _shift(T0, [3.0, 0.0, 0.0])
    res = build_curve(model, UR5_Q, [(_shift(T0, [0.0, 0.05, 0.0]), []), (far, [])], "line", 0.1, svc)
    assert not res.ok and res.message.startswith("Leg 2")
    with pytest.raises(ValueError):
        timed_trajectory(model, res)
    res = build_curve(model, UR5_Q, [(far, []), (T0, [])], "line", 0.1, svc)
    assert "not computed" in res.message


def test_timing_speed_and_topp_reach_the_same_end():
    svc, model, T0 = _setup()
    T1 = _shift(T0, [0.0, 0.12, 0.05])
    res = build_curve(model, UR5_Q, [(T1, [])], "line", 0.1, svc)
    tr = timed_trajectory(model, res, "speed")
    assert tr.duration == pytest.approx(res.duration)
    assert np.allclose(tr.q(0.0), UR5_Q) and np.allclose(tr.q(tr.duration), res.q[-1])
    tr2 = timed_trajectory(model, res, "speed", speed=0.2)
    assert tr2.duration == pytest.approx(res.duration / 2)
    sess = svc.session()
    tt = timed_trajectory(model, res, "topp", sess.world)
    assert np.allclose(tt.q(tt.duration), res.q[-1], atol=1e-6)
    assert tt.duration < tr.duration                       # 0.13 m at 0.1 m/s is slow for a UR5
    sp = trajectory_speeds(model, tt, 20)
    assert sp[0] < 0.05 and np.max(sp) > 0.1
    p = tcp_at_time(model, tt, 0.5 * tt.duration)
    assert p.shape == (3,)
    # the timed trajectory stays on the curve (TCP within 1 mm of the line)
    d = T1[:3, 3] - T0[:3, 3]
    for t in np.linspace(0, tt.duration, 40):
        x = tcp_at_time(model, tt, t) - T0[:3, 3]
        perp = x - (x @ d) / (d @ d) * d
        assert np.linalg.norm(perp) < 1e-3


# ----------------------------------------------------------------------------- Curve panel (GUI)
import os
import time

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtCore import QCoreApplication

from app.main_window import MainWindow
from app.settings import AppSettings

from .test_live_blocking import load, put_q

ORG = "RoboticsAnalyzer_test_u4b3"


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


def pump_until(cond, timeout=60.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        QCoreApplication.processEvents()
        if cond():
            return True
        time.sleep(0.005)
    return cond()


def settle(cp):
    """Wait for the debounce timer and the worker."""
    assert pump_until(lambda: not cp.busy)


def curve_setup(win, delta=(0.0, 0.10, 0.05), robot="ur5"):
    model = load(win, robot)
    put_q(win, UR5_Q)
    tp = win.target_panel
    tp.auto_solve = False
    win._action_view_target.setChecked(True)
    win._action_view_curve.setChecked(True)
    QCoreApplication.processEvents()
    assert tp.add_from_tcp()
    tp.move_to(_fk(model, UR5_Q)[:3, 3] + np.array(delta))
    cp = win.curve_panel
    settle(cp)
    return model, tp, cp


def test_panel_follows_target_and_draws_tube(win):
    model, tp, cp = curve_setup(win)
    res = cp.result
    assert res is not None and res.ok, cp.check_text
    assert "clear" in cp.check_text.lower()
    rv = win.viewport.robot_view
    assert rv.path_tube_actor is not None
    assert np.allclose(rv.path_tube_points[-1], tp.current_pose()[:3, 3], atol=1e-6)
    assert cp.button_enabled()["play"] and cp.button_enabled()["generate"]
    assert "Cycle time" in cp.cycle_text
    cp.set_color_mode("speed")
    assert rv.path_tube_colors is not None
    cp.set_color_mode("singularity")
    cp.set_color_mode("status")
    win._action_view_curve.setChecked(False)
    QCoreApplication.processEvents()
    assert rv.path_tube_actor is None


def test_unreachable_target_turns_tube_red_and_blocks_play(win):
    model, tp, cp = curve_setup(win)
    tp.move_to([2.5, 0.0, 0.4])
    settle(cp)
    res = cp.result
    assert res is not None and not res.ok
    assert not cp.button_enabled()["play"] and not cp.button_enabled()["generate"]
    assert "Sample" in cp.play_tooltip and "t =" in cp.play_tooltip
    assert cp.check_text.startswith("Problem")
    cols = win.viewport.robot_view.path_tube_colors
    assert cols is not None and np.allclose(cols[-1], [1.0, 0.32, 0.32])


def test_spline_via_points_add_type_delete_drag(win):
    model, tp, cp = curve_setup(win)
    cp.set_kind("spline")
    assert cp.button_enabled()["add"]
    assert cp.add_via() and len(cp.via_points()) == 1
    v0 = cp.via_points()[0]
    start = _fk(model, UR5_Q)[:3, 3]
    assert np.allclose(v0, 0.5 * (start + tp.current_pose()[:3, 3]))      # midpoint of the longest
    assert cp.add_via() and len(cp.via_points()) == 2
    assert cp.handles is not None and len(cp.handles.widgets) == 2
    cp.set_via_xyz(0, (v0 + np.array([0.0, 0.0, 0.05])) * 1000.0)
    settle(cp)
    assert cp.result.ok and len(cp.result.via_points) == 2
    assert np.allclose(cp.via_points()[0], v0 + [0, 0, 0.05])
    # a drag of the first handle moves the via point and re-runs the check
    cp.handles.move_point(0, v0 + np.array([0.0, 0.02, 0.06]))
    settle(cp)
    assert np.allclose(cp.via_points()[0], v0 + [0, 0.02, 0.06])
    assert cp.result.ok and len(win.viewport.robot_view.path_via_actors) == 4
    cp.via_list.setCurrentRow(1)
    assert cp.delete_via() and len(cp.via_points()) == 1
    settle(cp)
    assert len(cp.result.via_points) == 1
    cp.set_kind("line")
    assert not cp.button_enabled()["add"]
    settle(cp)
    assert cp.result.via_points == []


def test_arc_gets_default_via_and_draws_through_it(win):
    model, tp, cp = curve_setup(win)
    cp.set_kind("arc")
    settle(cp)
    assert len(cp.via_points()) == 1 and cp.result.ok
    assert not cp.button_enabled()["add"]


def test_chain_two_targets_cycle_time(win):
    model, tp, cp = curve_setup(win)
    p = tp.current_pose()
    assert tp.add_from_tcp(pose=np.vstack([np.column_stack([p[:3, :3], p[:3, 3] + [0.0, -0.05, 0.05]]),
                                           [0, 0, 0, 1]]), name="C")
    tp.select_target(0)
    settle(cp)
    one = cp.result.duration
    cp.set_chain(True)
    settle(cp)
    assert len(cp.result.legs) == 2 and cp.result.ok
    assert cp.result.duration > one
    assert "2 legs" in cp.cycle_text
    cp.set_chain(True, names=["Target 1"])
    settle(cp)
    assert len(cp.result.legs) == 1


def test_generate_sends_trajectory_to_timeline(win):
    model, tp, cp = curve_setup(win)
    got = []
    cp.trajectory_ready.connect(lambda t, info: got.append((t, info)))
    cp.set_timing("topp")
    cp.generate(play=False)
    assert pump_until(lambda: not cp.busy and got)
    traj, info = got[0]
    assert info["play"] is False and abs(info["duration"] - traj.duration) < 1e-12
    assert win.sim_panel.trajectory is not None
    assert win.sim_panel.trajectory.duration == pytest.approx(traj.duration)
    assert f"{traj.duration:.2f}" in cp.cycle_text
    # editing the curve makes the generated trajectory stale
    cp.set_kind("spline")
    assert cp.trajectory is None


def test_manual_move_restarts_curve_from_new_pose(win):
    model, tp, cp = curve_setup(win)
    q2 = UR5_Q.copy()
    q2[0] = 0.15
    put_q(win, q2)
    win._on_joint_q_changed()
    settle(cp)
    assert cp.result.ok
    assert np.allclose(cp.result.q[0], q2)
