"""Motion to a chosen IK solution, with optional approach / retreat (U4a.4). No Qt dependency.

``plan_target_motion`` builds ONE trajectory (linear interpolation of dense samples):

1. joint-space path ``q_start -> q_pre`` (``q_pre = q_goal`` without approach): a straight joint
   line when it is free, else the planner (RRT-Connect, own planners of R7.4), time-parametrised
   with TOPP (``time_parametrize``). Collision world = the U3 shapes (hulls, floor, objects).
2. approach: straight Cartesian line ``T_pre -> T_target`` along the tool z axis (``T_pre`` is
   ``approach`` metres back from the target), IK sample per step seeded from the chosen solution
   (``cartesian.line``), every sample collision checked. Retreat = the same line reversed.

The last waypoint of the approach is exactly ``q_goal``. Gripper DOFs stay at their start value.
"""
from __future__ import annotations

import threading
from dataclasses import dataclass, field
from typing import Any, Sequence

import numpy as np

import core_bridge as cb
from app.gripper import gripper_dofs
from app.kin_utils import dof_limits
from app.planning.cartesian import line
from app.planning.planner import plan
from app.planning.shortcut import time_parametrize
from app.sim.trajectory import Trajectory, Waypoint

DT_SAMPLE = 0.04               # s: spacing of the dense samples taken from the TOPP trajectory
APPROACH_SPEED = 0.10          # m/s along the approach line
MIN_LINE_TIME = 1.0            # s
LINE_STEP = 0.005              # m between Cartesian samples


@dataclass
class MotionResult:
    ok: bool
    trajectory: Trajectory | None = None
    message: str = ""
    q_pre: list | None = None
    segments: dict = field(default_factory=dict)      # name -> (t0, t1)
    planner_ms: float = 0.0
    approach_samples: Any = None                      # (k, dof) approach line, pre -> target
    approach_targets: Any = None                      # (k, 4, 4) wanted TCP poses on the line


def pre_pose(T_target: Any, distance: float) -> np.ndarray:
    """Pose ``distance`` metres back from ``T_target`` along the tool -z axis."""
    T = np.array(T_target, float)
    T[:3, 3] = T[:3, 3] - float(distance) * T[:3, 2]
    return T


def _locked_limits(model, q_ref):
    lo, hi = (np.array(v, float) for v in dof_limits(model))
    for g in gripper_dofs(model):
        lo[g.dof_index] = hi[g.dof_index] = float(q_ref[g.dof_index])
    return lo, hi


def _dense(traj: Trajectory, t_offset: float, dt: float = DT_SAMPLE):
    ts = np.append(np.arange(0.0, traj.duration, dt), traj.duration)
    return [(t_offset + float(t), np.asarray(traj.q(float(t)), float)) for t in ts]


def plan_target_motion(model: Any, q_start: Sequence[float], q_goal: Sequence[float],
                       T_target: Any, world: Any = None, approach: float = 0.0,
                       retreat: bool = False, planner: str = "rrt_connect", time_limit: float = 5.0,
                       seed: int = 0, method: str = "topp", cancel: Any = None,
                       cm: Any = None) -> MotionResult:
    """Plan the whole move. Never raises for planning failures: ``ok=False`` + ``message``."""
    q0 = np.asarray(q_start, float)
    qg = np.asarray(q_goal, float)
    n = model.dof()
    if q0.shape != (n,) or qg.shape != (n,):
        raise ValueError(f"q_start and q_goal need {n} values")
    cm = cm if cm is not None else cb.compile(model)
    lo, hi = _locked_limits(model, q0)
    cancel = cancel if cancel is not None else threading.Event()
    wps: list[tuple[float, np.ndarray]] = []
    segments: dict = {}
    res = MotionResult(False)

    q_pre = qg
    line_q = None
    if approach > 0.0:
        T_pre = pre_pose(T_target, approach)
        length = float(approach)
        k = max(10, int(np.ceil(length / LINE_STEP)) + 1)
        dur = max(MIN_LINE_TIME, length / APPROACH_SPEED)
        # solved from the target backwards: sample 0 starts at the chosen solution
        path = line(model, T_target, T_pre, qg, n=k, duration=dur, profile="quintic", cm=cm,
                    ik_options={"max_iter": 60, "position_only": False})
        if not path.ok:
            res.message = f"approach line not reachable ({path.message})"
            return res
        qs = np.array(path.q)
        qs[0] = qg                                    # exactly the chosen solution
        if world is not None:
            bad = np.asarray(world.check_batch(qs))
            if bad.any():
                res.message = (f"approach line collides at {100 * int(np.argmax(bad)) / (k - 1):.0f} % "
                               "of its length")
                return res
        line_q = qs[::-1]                              # pre -> target
        q_pre = line_q[0].copy()
        res.approach_samples = line_q.copy()
        res.approach_targets = np.asarray(path.targets)[::-1].copy()

    # ---- joint segment q_start -> q_pre
    t_end = 0.0
    wps.append((0.0, q0))
    if float(np.max(np.abs(q_pre - q0))) > 1e-9:
        pr = plan(model, q0, q_pre, world, (lo, hi), planner=planner, time_limit=time_limit,
                  seed=seed, cancel=cancel)
        res.planner_ms = pr.time * 1000.0
        if not pr.success:
            res.message = f"no collision-free path: {pr.reason}"
            return res
        timed = time_parametrize(pr.path, model, collision=world, method=method)
        dense = _dense(timed.trajectory, 0.0)
        wps = dense
        wps[0] = (0.0, q0)
        wps[-1] = (wps[-1][0], q_pre.copy())
        t_end = wps[-1][0]
        segments["move"] = (0.0, t_end)

    # ---- approach (and retreat) along the line
    if line_q is not None:
        k = len(line_q)
        dur = max(MIN_LINE_TIME, approach / APPROACH_SPEED)
        times = np.linspace(0.0, 1.0, k) * dur
        if t_end == 0.0 and len(wps) == 1:
            wps = [(0.0, line_q[0].copy())]           # start already at the pre-pose
        t0 = t_end
        for tk, qk in zip(times[1:], line_q[1:]):
            wps.append((t0 + float(tk), qk.copy()))
        t_end = wps[-1][0]
        segments["approach"] = (t0, t_end)
        if retreat:
            t0 = t_end
            for tk, qk in zip(times[1:], line_q[::-1][1:]):
                wps.append((t0 + float(tk), qk.copy()))
            t_end = wps[-1][0]
            segments["retreat"] = (t0, t_end)

    if len(wps) < 2:
        res.message = "already at the target"
        return res
    res.trajectory = Trajectory([Waypoint(float(t), tuple(float(v) for v in q)) for t, q in wps],
                                "linear")
    res.ok = True
    res.q_pre = [float(v) for v in q_pre]
    res.segments = segments
    return res
