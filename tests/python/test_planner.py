"""R7.4: path planners (RRT-Connect, RRT*, PRM), shortcut, time parametrisation, panel.

Oracles: a separate densified re-check (1e-3 step) with the collision world and an own numpy
centreline-vs-rectangle test of the planar 2R robot (link centreline inside a link box, so a free
path can never put the centreline into the obstacle).
"""
import math
import os
import threading
import time

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
pytest.importorskip("fcl")

from PySide6.QtWidgets import QApplication

import core_bridge as cb
from app.kin_utils import dof_limits
from app.panels.planning_panel import PlanningPanel
from app.panels.timeline_panel import TimelinePanel
from app.planning.checks import check_trajectory
from app.planning.collision import CollisionConfig, CollisionWorld, EnvShape
from app.planning.planner import PLANNERS, plan
from app.planning.shortcut import path_length, shortcut, time_parametrize
from app.task_runner import TaskRunner

from .test_cartesian_traj import planar_2r
from .test_collision import planar_shapes

BOX_C, BOX_SIZE = (0.0, 1.2), (0.3, 0.3)
START, GOAL = [0.0, 0.0], [2.2, 0.3]


def env_config():
    return CollisionConfig(environment=[EnvShape("box", {"size": [0.3, 0.3, 0.4]},
                                                xyz=[BOX_C[0], BOX_C[1], 0.0])],
                           floor=False)


def env_world(model=None):
    model = model or planar_2r()
    return CollisionWorld(model, planar_shapes(-5.0), env_config())


def densify(path, step=1e-3):
    pts = [np.asarray(path[0], float)]
    for a, b in zip(path[:-1], path[1:]):
        a, b = np.asarray(a, float), np.asarray(b, float)
        n = max(1, int(math.ceil(np.linalg.norm(b - a) / step)))
        pts.extend(a + (b - a) * (k / n) for k in range(1, n + 1))
    return np.array(pts)


def centreline_hits_box(qs):
    """Own oracle: does the centreline of link1/link2 enter the obstacle rectangle?"""
    qs = np.asarray(qs, float)
    q1, q12 = qs[:, 0], qs[:, 0] + qs[:, 1]
    p1 = np.stack([np.cos(q1), np.sin(q1)], 1)
    p2 = p1 + 0.5 * np.stack([np.cos(q12), np.sin(q12)], 1)
    t = np.linspace(0.0, 1.0, 101)[None, :, None]
    seg = np.concatenate([t * p1[:, None, :], p1[:, None, :] + t * (p2 - p1)[:, None, :]], 1)
    hx, hy = BOX_SIZE[0] / 2, BOX_SIZE[1] / 2
    inside = (np.abs(seg[..., 0] - BOX_C[0]) < hx) & (np.abs(seg[..., 1] - BOX_C[1]) < hy)
    return inside.any(axis=1)


def assert_valid(path, world, start, goal):
    assert np.allclose(path[0], start) and np.allclose(path[-1], goal)
    dense = densify(path, 1e-3)
    assert not world.check_batch(dense).any()
    assert not centreline_hits_box(dense).any()
    lo, hi = dof_limits(planar_2r())
    assert (dense >= np.array(lo) - 1e-12).all() and (dense <= np.array(hi) + 1e-12).all()


class Island:
    """Fake collision: inside a ball only the goal point itself is free (unreachable goal)."""

    def __init__(self, goal, r=0.6):
        self.goal, self.r = np.asarray(goal, float), r

    def check_batch(self, qs):
        d = np.linalg.norm(np.asarray(qs, float) - self.goal, axis=1)
        return (d < self.r) & (d > 1e-9)


# ----------------------------------------------------------------------------- planners
@pytest.mark.parametrize("planner", PLANNERS)
def test_free_space_path_valid(planner):
    m = planar_2r()
    w = CollisionWorld(m, planar_shapes(-5.0))
    res = plan(m, START, GOAL, w, planner=planner, time_limit=10, seed=3)
    assert res.success, res.reason
    assert not w.check_batch(densify(res.path, 1e-3)).any()
    assert np.allclose(res.path[0], START) and np.allclose(res.path[-1], GOAL)


@pytest.mark.parametrize("planner", PLANNERS)
def test_around_environment_obstacle(planner):
    m = planar_2r()
    w = env_world(m)
    # the straight joint-space line is blocked (oracle, not the planner)
    assert centreline_hits_box(densify([START, GOAL], 1e-3)).any()
    res = plan(m, START, GOAL, w, planner=planner, time_limit=20, seed=1)
    assert res.success, res.reason
    assert len(res.path) > 2
    assert_valid(res.path, w, START, GOAL)
    print(f"R7.4 {planner}: {res.time * 1000:.0f} ms, {res.iterations} it, "
          f"{len(res.path)} wp, length {res.length:.3f} (raw {res.raw_length:.3f})")


def test_self_collision_goal_reported():
    m = planar_2r()
    w = CollisionWorld(m, planar_shapes(0.75))          # link 0 sphere at x = 0.75
    w.set_q([0.0, math.pi])                              # link 2 folds back into the sphere
    assert w.in_collision()
    res = plan(m, [0.0, 0.0], [0.0, math.pi], w, time_limit=2)
    assert not res.success and "collision" in res.reason


def test_self_collision_avoided_in_path():
    m = planar_2r()
    w = CollisionWorld(m, planar_shapes(0.75))
    res = plan(m, [0.5, 1.0], [0.5, -1.0], w, time_limit=5, seed=2)
    assert res.success
    assert not w.check_batch(densify(res.path, 1e-3)).any()


@pytest.mark.parametrize("planner", PLANNERS)
def test_same_seed_same_path(planner):
    m = planar_2r()
    w = env_world(m)
    a = plan(m, START, GOAL, w, planner=planner, time_limit=30, seed=7)
    b = plan(m, START, GOAL, w, planner=planner, time_limit=30, seed=7)
    assert a.success and b.success
    assert a.path.shape == b.path.shape and np.array_equal(a.path, b.path)
    assert a.iterations == b.iterations


def test_different_seed_may_differ():
    m = planar_2r()
    w = env_world(m)
    a = plan(m, START, GOAL, w, time_limit=10, seed=1)
    b = plan(m, START, GOAL, w, time_limit=10, seed=2)
    assert a.success and b.success
    assert a.path.shape != b.path.shape or not np.array_equal(a.path, b.path)


def test_unreachable_goal_inside_obstacle_fails_cleanly():
    m = planar_2r()
    w = env_world(m)
    inside = [math.pi / 2, 0.0]          # link 1 up to (0, 1), link 2 to (0, 1.5): in the box
    w.set_q(inside)
    assert w.in_collision() and centreline_hits_box(np.array([inside])).all()
    res = plan(m, START, inside, w, time_limit=1)
    assert not res.success and res.path is None and "collision" in res.reason


def test_enclosed_goal_times_out_without_crash():
    m = planar_2r()
    res = plan(m, [0.0, 0.0], [1.0, 1.0], Island([1.0, 1.0]), time_limit=0.4, seed=0)
    assert not res.success and res.reason == "time limit reached"
    assert 0.3 < res.time < 1.5


@pytest.mark.parametrize("planner", PLANNERS)
def test_cancel_returns_quickly(planner):
    m = planar_2r()
    evt = threading.Event()
    threading.Timer(0.1, evt.set).start()
    t0 = time.perf_counter()
    res = plan(m, [0.0, 0.0], [1.0, 1.0], Island([1.0, 1.0]), planner=planner,
               time_limit=30, seed=0, cancel=evt)
    assert not res.success and res.reason == "cancelled"
    assert time.perf_counter() - t0 < 1.5


def test_cancel_callable_and_progress():
    m = planar_2r()
    calls = {"n": 0, "prog": 0}

    def cancel():
        calls["n"] += 1
        return calls["n"] > 200

    def progress(it, name):
        calls["prog"] += 1

    res = plan(m, [0.0, 0.0], [1.0, 1.0], Island([1.0, 1.0]), time_limit=30, cancel=cancel,
               progress=progress)
    assert res.reason == "cancelled" and calls["prog"] >= 1


def test_no_collision_joint_limits_only():
    m = planar_2r()
    res = plan(m, START, GOAL, None)
    assert res.success and len(res.path) == 2
    bad = plan(m, START, [4.0, 0.0], None)
    assert not bad.success and "limits" in bad.reason
    bad = plan(m, [4.0, 0.0], GOAL, None)
    assert not bad.success and "limits" in bad.reason


def test_goal_as_tcp_pose_uses_ik():
    m = planar_2r()
    w = env_world(m)
    cm = cb.compile(m)
    pose = np.asarray(cb.fk_compiled(cm, list(GOAL))["tcp"], float)
    res = plan(m, START, pose, w, time_limit=20, seed=1)
    assert res.success, res.reason
    q1, q12 = res.path[-1][0], res.path[-1][0] + res.path[-1][1]    # independent planar FK
    xy = np.array([math.cos(q1) + 0.5 * math.cos(q12), math.sin(q1) + 0.5 * math.sin(q12)])
    assert np.allclose(xy, pose[:2, 3], atol=1e-5)
    dense = densify(res.path, 1e-3)
    assert not w.check_batch(dense).any()
    assert not centreline_hits_box(dense).any()


def test_unreachable_pose_reports_failure():
    m = planar_2r()
    pose = np.eye(4)
    pose[:3, 3] = [5.0, 0.0, 0.0]
    res = plan(m, START, pose, None, time_limit=2)
    assert not res.success and res.path is None


def test_bad_arguments():
    m = planar_2r()
    with pytest.raises(ValueError):
        plan(m, START, GOAL, None, planner="nope")
    with pytest.raises(ValueError):
        plan(m, [0.0], GOAL, None)


# ----------------------------------------------------------------------------- shortcut
def _circle_free(q):
    return np.linalg.norm(q) > 0.8


def _circle_edge_free(a, b):
    pts = a + (b - a) * np.linspace(0, 1, 200)[:, None]
    return all(_circle_free(p) for p in pts)


def test_shortcut_never_longer_or_invalid():
    path = np.array([[-1.5, 0.0], [-1.0, 1.2], [-0.2, 1.6], [0.8, 1.3], [1.5, 0.0],
                     [1.4, -0.9], [0.0, -1.5], [-1.0, -1.0], [-1.5, -0.1]])
    assert all(_circle_edge_free(a, b) for a, b in zip(path[:-1], path[1:]))
    for seed in range(5):
        out = shortcut(path, _circle_edge_free, np.random.default_rng(seed), iterations=200)
        assert path_length(out) <= path_length(path) + 1e-12
        assert np.allclose(out[0], path[0]) and np.allclose(out[-1], path[-1])
        assert all(_circle_edge_free(a, b) for a, b in zip(out[:-1], out[1:]))
    out = shortcut(path, _circle_edge_free, np.random.default_rng(1), iterations=500)
    assert len(out) < len(path)
    assert path_length(out) < path_length(path)


def test_shortcut_on_planned_path_keeps_validity():
    m = planar_2r()
    w = env_world(m)
    raw = plan(m, START, GOAL, w, time_limit=20, seed=5, do_shortcut=False)
    short = plan(m, START, GOAL, w, time_limit=20, seed=5, do_shortcut=True)
    assert raw.success and short.success
    assert short.length <= raw.length + 1e-12
    assert_valid(short.path, w, START, GOAL)


# ----------------------------------------------------------------------------- time parametrisation
def test_time_parametrize_passes_waypoints_and_respects_limits():
    m = planar_2r()
    w = env_world(m)
    res = plan(m, START, GOAL, w, time_limit=20, seed=1)
    tp = time_parametrize(res.path, m, vmax=1.0, amax=2.0, collision=w)
    traj = tp.trajectory
    assert traj.duration > 0
    for wp, q in zip(traj.waypoints, res.path):
        assert np.allclose(traj.q(wp.t), q, atol=1e-9)
    rep = check_trajectory(traj, m, {"vel": 1.0, "accel": 2.0}, check_torque=False)
    assert not rep.of_kind("velocity") and not rep.of_kind("position")
    ts = np.linspace(0, traj.duration, 2000)
    qs = np.array([traj.q(t) for t in ts])
    assert not w.check_batch(qs).any()
    assert not centreline_hits_box(qs).any()
    assert np.abs(np.array([traj.qd(t) for t in ts])).max() <= 1.0 + 1e-6


def test_time_parametrize_linear():
    m = planar_2r()
    tp = time_parametrize([[0, 0], [1, 0.5], [2, 0.0]], m, vmax=0.5, amax=1.0,
                          interpolation="linear")
    assert tp.interpolation == "linear" and tp.trajectory.duration > 0
    assert not tp.report.of_kind("velocity")


# ----------------------------------------------------------------------------- panel
def _pump_until(cond, timeout=30.0):
    end = time.perf_counter() + timeout
    while time.perf_counter() < end:
        QApplication.processEvents()
        if cond():
            return True
        time.sleep(0.005)
    return False


@pytest.fixture
def panel(qapp):
    runner = TaskRunner()
    p = PlanningPanel(runner)
    p.set_model(planar_2r())
    p.show()
    p.set_collision_source(lambda: (planar_shapes(-5.0), env_config()))
    yield p
    p.shutdown()
    runner.shutdown()


def test_panel_plan_and_send(panel, qapp):
    tl = TimelinePanel()
    tl.set_model(2, ["a", "b"], ([-3.2, -3.2], [3.2, 3.2]))
    panel.trajectory_ready.connect(tl.set_trajectory)
    assert panel.button_enabled() == {"plan": True, "cancel": False, "send": False}
    panel.set_vector("start", START)
    panel.set_vector("goal", GOAL)
    panel._time_limit.setValue(20.0)
    panel._seed.setValue(1)
    panel.plan()
    assert panel.is_running and panel.button_enabled()["cancel"]
    assert _pump_until(lambda: not panel.is_running)
    res = panel.result
    assert res is not None and res.success, panel.status_text
    assert "waypoints" in panel.summary_text
    assert panel.button_enabled()["send"]
    panel.send_to_trajectory()
    traj = tl.trajectory
    assert traj is not None and traj.n_dof == 2
    assert np.allclose(traj.waypoints[0].q, START) and np.allclose(traj.waypoints[-1].q, GOAL)


def test_panel_failure_and_cancel(panel, qapp):
    panel.set_vector("start", [0.0, 0.0])
    panel.set_vector("goal", [4.0, 0.0])                 # outside the joint limits
    panel.plan()
    assert _pump_until(lambda: not panel.is_running)
    assert not panel.result.success and "limits" in panel.status_text
    assert not panel.button_enabled()["send"]
    # goal in the obstacle
    panel.set_vector("goal", [math.pi / 2, 0.0])
    panel.plan()
    assert _pump_until(lambda: not panel.is_running)
    assert not panel.result.success and "collision" in panel.status_text
    # cancel a running search (blocked goal island is not reachable through the panel, so use a
    # long time limit on a hard-ish problem and cancel at once)
    panel.set_vector("goal", GOAL)
    panel._time_limit.setValue(60.0)
    panel._planner.setCurrentIndex(1)                    # RRT*: keeps optimising after the first path
    panel.plan()
    panel.cancel()
    t0 = time.perf_counter()
    assert _pump_until(lambda: not panel.is_running)
    assert time.perf_counter() - t0 < 5.0
    assert panel.result is not None and not panel.result.success
    assert panel.result.reason == "cancelled"


def test_panel_capture_named_and_model_change(panel):
    panel.set_q_provider(lambda: [0.3, -0.4])
    panel.capture_current("start")
    assert panel.vector("start") == [0.3, -0.4]
    panel.set_named_configs({"home": [0.1, 0.2], "bad": [1.0]})   # wrong DOF ignored
    assert panel._named_goal.count() == 2
    panel._named_goal.setCurrentText("home")
    panel._apply_named("goal")
    assert panel.vector("goal") == [0.1, 0.2]
    panel._table.item(0, 1).setText("abc")
    assert panel.vector("goal") is None
    panel.plan()
    assert not panel.is_running and "finite" in panel.status_text
    panel.set_model(None)
    assert panel._table.rowCount() == 0 and not panel.button_enabled()["plan"]
