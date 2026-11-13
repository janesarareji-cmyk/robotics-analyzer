"""U4b.4: follow the generated curve: playback, scrub, moving marker, end ghost, status text."""
import numpy as np
import pytest

pytest.importorskip("fcl")

from PySide6.QtCore import QCoreApplication

from app.kin_utils import current_q

from .test_curve_editor import UR5_Q, _fk, curve_setup, pump_until, settle, win  # noqa: F401


def play_to_end(win, step=0.05):
    sp = win.sim_panel
    pl = sp.player
    assert pl is not None
    for _ in range(4000):                     # what the dock timer does, with a fixed wall step
        if pl.finished:
            break
        sp._last_tick -= step
        sp._on_tick()
    QCoreApplication.processEvents()
    return pl


def generate(cp, play, timing="speed"):
    got = []
    cp.trajectory_ready.connect(lambda t, i: got.append(t))
    cp.set_timing(timing)
    cp.generate(play=play)
    assert pump_until(lambda: got and not cp.busy)
    return got[0]


def test_play_follows_curve_and_ends_on_target(win):
    model, tp, cp = curve_setup(win, delta=(0.0, 0.10, 0.05))
    target = tp.current_pose()
    cp.set_kind("spline")
    cp.add_via()
    settle(cp)
    assert cp.result.ok
    traj = generate(cp, play=True, timing="topp")
    sp = win.sim_panel
    assert sp.trajectory is not None and sp.player is not None
    assert "Playing the curve" in win._status_label.text()
    pl = play_to_end(win)
    assert pl.finished, (win._status_label.text(), win.sim_panel.status_text, pl.t, pl.duration)
    assert pl.collision_stop is None
    tcp = _fk(model, current_q(model))
    assert np.linalg.norm(tcp[:3, 3] - target[:3, 3]) < 1e-4
    assert win._status_label.text() == "Motion finished"
    # the played TCP stays on the drawn curve (dense recheck against the tube points)
    P = cp.result.points
    seg = np.diff(P, axis=0)
    for t in np.linspace(0.0, traj.duration, 30):
        x = _fk(model, traj.q(t))[:3, 3]
        u = np.clip(np.einsum("ij,ij->i", x - P[:-1], seg) / np.einsum("ij,ij->i", seg, seg), 0, 1)
        d = np.min(np.linalg.norm(P[:-1] + u[:, None] * seg - x, axis=1))
        assert d < 1e-3, (t, d)


def test_scrub_gives_trajectory_q_and_marker(win):
    model, tp, cp = curve_setup(win)
    traj = generate(cp, play=False)
    sp = win.sim_panel
    assert sp.trajectory is not None
    for f in (0.25, 0.6, 1.0):
        t = f * traj.duration
        sp.seek_trajectory(t)
        QCoreApplication.processEvents()
        assert np.allclose(current_q(model), traj.q(t), atol=1e-12)
        marker = win.viewport.robot_view.path_marker_actor
        assert marker is not None
        assert np.allclose(np.asarray(marker.user_matrix)[:3, 3], _fk(model, traj.q(t))[:3, 3],
                           atol=1e-9)
    sp.stop_trajectory()
    QCoreApplication.processEvents()
    assert np.allclose(np.asarray(win.viewport.robot_view.path_marker_actor.user_matrix)[:3, 3],
                       _fk(model, traj.q(0.0))[:3, 3], atol=1e-9)


def test_end_ghost_stays_while_playing_and_tube_stays_when_dock_hidden(win):
    model, tp, cp = curve_setup(win)
    traj = generate(cp, play=True)
    rv = win.viewport.robot_view
    assert ("curve", "end") in rv.ghost_keys
    assert np.allclose(rv.ghost_tcp(("curve", "end"))[:3, 3], tp.current_pose()[:3, 3], atol=1e-4)
    win.sim_panel.pause_trajectory()
    win._action_view_simulation.setChecked(True)            # the Curve tab is hidden by that
    QCoreApplication.processEvents()
    assert rv.path_tube_actor is not None and ("curve", "end") in rv.ghost_keys
    assert win.sim_panel.timeline.speed == 1.0
    win.sim_panel.timeline._speed.setValue(0.5)
    assert win.sim_panel.timeline.speed == 0.5
    # editing the curve afterwards drops the generated state
    win._action_view_curve.setChecked(True)
    QCoreApplication.processEvents()
    cp.set_kind("spline")
    assert ("curve", "end") not in rv.ghost_keys
    assert rv.path_marker_actor is None


def test_curve_play_is_blocked_by_collision_guard(win):
    """A curve through an object cannot be generated (red tube), so Play never starts."""
    model, tp, cp = curve_setup(win, delta=(0.0, 0.10, 0.0))
    p = tp.current_pose()
    mid = 0.5 * (_fk(model, UR5_Q)[:3, 3] + p[:3, 3])
    assert win.collision_panel.add_shape("box", {"size": [0.05, 0.05, 0.05]}, list(mid), name="blk")
    win._sync_env_objects()
    cp.notify_q_changed()
    settle(cp)
    win.collision_service.invalidate()
    cp.refresh_now()
    settle(cp)
    assert cp.result is not None and not cp.result.ok
    assert not cp.button_enabled()["play"]
    cp.generate(play=True)
    QCoreApplication.processEvents()
    assert win.sim_panel.player is None


def test_play_runs_on_the_timer_with_the_simulation_dock_hidden(win):
    """Regression (U4a leftover): background play kept the clock running only by accident."""
    import time
    model, tp, cp = curve_setup(win)
    assert not win.sim_dock.isVisible()
    generate(cp, play=True)
    sp = win.sim_panel
    assert sp.timer.isActive()
    t0 = time.time()
    while time.time() - t0 < 0.5 and sp.player is not None and sp.player.t <= 0.0:
        QCoreApplication.processEvents()
        time.sleep(0.01)
    assert sp.player is not None and sp.player.t > 0.0
