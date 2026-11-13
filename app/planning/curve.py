"""Curve from the current pose to one or more targets (U4b.3). No Qt dependency.

``build_curve`` solves one or more legs (line / arc / spline) from the current pose, each leg
starting where the previous one ended (chain A -> B -> C), checks every sample (``path_check``)
and returns a ``CurveResult``: tube points, per-sample check, nominal times. ``timed_trajectory``
turns a good result into a ``Trajectory`` (constant TCP speed or time-optimal TOPP).

Gripper DOFs stay where they are (locked in the IK limits), as in the Target dock.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Sequence

import numpy as np

import core_bridge as cb
from app.gripper import gripper_dofs
from app.kin_utils import dof_limits
from app.planning import cartesian
from app.planning.path_check import (
    BAD, PathCheck, check_path, tcp_speed,
)
from app.sim.trajectory import Trajectory, Waypoint

KINDS = ("line", "arc", "spline")
TIMINGS = ("speed", "topp")
SAMPLE_STEP = 0.01             # m between samples
DEFAULT_SPEED = 0.10           # m/s
MIN_LEG_TIME = 0.2             # s
IK_OPTIONS = {"max_iter": 60}
DENSE_DT = 0.04               # s: sample spacing of a time-optimal trajectory


@dataclass
class CurveLeg:
    kind: str
    T0: np.ndarray
    T1: np.ndarray
    via: list                       # via positions (m), arc: exactly one
    cpath: Any = None
    check: PathCheck | None = None
    note: str = ""


@dataclass
class CurveResult:
    legs: list = field(default_factory=list)
    points: np.ndarray = field(default_factory=lambda: np.zeros((0, 3)))   # tube points (N, 3)
    q: np.ndarray = field(default_factory=lambda: np.zeros((0, 0)))        # solved samples (k, dof)
    t: np.ndarray = field(default_factory=lambda: np.zeros(0))             # nominal time per point
    check: PathCheck | None = None
    leg_ranges: list = field(default_factory=list)      # (first, last) point index per leg
    via_points: list = field(default_factory=list)      # world positions of every via point
    length: float = 0.0
    message: str = ""
    q_start: Any = None
    speed: float = DEFAULT_SPEED

    @property
    def ok(self) -> bool:
        """Every leg solved and no bad sample: Play is allowed."""
        return bool(self.legs) and self.check is not None and self.check.ok and not self.message

    @property
    def duration(self) -> float:
        return float(self.t[-1]) if len(self.t) else 0.0

    def problem_text(self) -> str:
        """First problem with sample index and time ("" when there is none)."""
        text = self.check.problem_text() if self.check is not None else ""
        if self.message:
            lead = self.message.split(":")[0] if self.message.startswith("Leg ") else ""
            tail = self.message[self.message.index("Legs "):] if "Legs " in self.message else ""
            text = " ".join(x for x in (lead + ":" if lead and text else "", text or self.message,
                                        tail if text else "") if x)
        return text


def default_arc_via(T0: Any, T1: Any) -> np.ndarray:
    """A via point that makes a visible arc: the midpoint pushed sideways (up when possible)."""
    p0, p1 = np.asarray(T0, float)[:3, 3], np.asarray(T1, float)[:3, 3]
    d = p1 - p0
    L = float(np.linalg.norm(d))
    if L < 1e-9:
        return p0.copy()
    dh = d / L
    n = np.array([0.0, 0.0, 1.0]) - dh[2] * dh
    if np.linalg.norm(n) < 1e-3:
        n = np.array([1.0, 0.0, 0.0]) - dh[0] * dh
    return 0.5 * (p0 + p1) + 0.25 * L * n / np.linalg.norm(n)


def longest_segment(p_start: Any, via: Sequence, p_end: Any) -> tuple[int, np.ndarray]:
    """``(index, midpoint)`` of the longest segment of start -> via... -> end. ``index`` is where
    a new via point goes in the via list (0 = before the first via point)."""
    pts = [np.asarray(p_start, float), *[np.asarray(v, float) for v in via], np.asarray(p_end, float)]
    seg = [float(np.linalg.norm(b - a)) for a, b in zip(pts[:-1], pts[1:])]
    i = int(np.argmax(seg))
    return i, 0.5 * (pts[i] + pts[i + 1])


def longest_segment_midpoint(p_start: Any, via: Sequence, p_end: Any) -> np.ndarray:
    """Midpoint of the longest segment (where "Add via point" puts the new point)."""
    return longest_segment(p_start, via, p_end)[1]


def _leg_length(kind: str, T0, T1, via) -> float:
    p0, p1 = T0[:3, 3], T1[:3, 3]
    if kind == "line" or not len(via):
        return float(np.linalg.norm(p1 - p0))
    if kind == "arc":
        try:
            _, r, _, _, sweep = cartesian.arc_geometry(p0, via[0], p1)
            return float(r * sweep)
        except ValueError:
            return float(np.linalg.norm(p1 - p0))
    pts = [p0, *via, p1]
    return float(sum(np.linalg.norm(b - a) for a, b in zip(pts[:-1], pts[1:])))


def solve_leg(model: Any, leg: CurveLeg, q_seed: Sequence[float], speed: float, cm: Any,
              limits: Any) -> cartesian.CartesianPath:
    """IK path of one leg (nominal constant-speed times)."""
    length = _leg_length(leg.kind, leg.T0, leg.T1, leg.via)
    dur = max(MIN_LEG_TIME, length / max(speed, 1e-6))
    kw = dict(ik_options=dict(IK_OPTIONS), max_joint_step=0.5, cm=cm, limits=limits)
    n = int(np.clip(math.ceil(length / SAMPLE_STEP) + 1, 12, 150))
    kind = leg.kind
    if kind == "arc":
        try:
            cartesian.arc_geometry(leg.T0[:3, 3], leg.via[0], leg.T1[:3, 3])
        except (ValueError, IndexError):
            kind = "line"
            leg.note = "arc points are collinear: straight line"
    if kind == "line" or (kind == "spline" and not leg.via):
        return cartesian.line(model, leg.T0, leg.T1, q_seed, n=n, duration=dur, **kw)
    if kind == "arc":
        return cartesian.arc(model, leg.T0, leg.via[0], leg.T1, q_seed, n=n, duration=dur, **kw)
    poses = [leg.T0, *cartesian.via_poses(leg.T0, leg.T1, leg.via), leg.T1]
    per = int(np.clip(math.ceil(length / max(len(poses) - 1, 1) / SAMPLE_STEP), 8, 60))
    return cartesian.spline(model, poses, q_seed, n_per_seg=per, duration=dur, **kw)


def merge_checks(checks: list, t_offsets: list) -> PathCheck:
    """One ``PathCheck`` for the legs joined end to start (the shared junction sample once)."""
    parts = {k: [] for k in ("t", "status", "reachable", "colliding", "joint_jump", "sigma_min",
                             "clearance")}
    reasons: list = []
    for i, (c, off) in enumerate(zip(checks, t_offsets)):
        sl = slice(1 if i > 0 else 0, None)
        parts["t"].append(c.t[sl] + off)
        for k in ("status", "reachable", "colliding", "joint_jump", "sigma_min", "clearance"):
            parts[k].append(getattr(c, k)[sl])
        reasons += c.reasons[sl]
    msg = next((c.message for c in checks if c.message), "")
    return PathCheck(*(np.concatenate(parts[k]) for k in ("t", "status", "reachable", "colliding",
                                                         "joint_jump", "sigma_min", "clearance")),
                     reasons=reasons, message=msg)


def build_curve(model: Any, q_start: Sequence[float], stops: Sequence, kind: str = "line",
                speed: float = DEFAULT_SPEED, collision: Any = None, cm: Any = None
                ) -> CurveResult:
    """Curve from the pose at ``q_start`` through ``stops`` = ``[(T_target, via_positions), ...]``.

    One leg per stop, each from the end of the previous leg. Never raises for IK / check
    failures: they are in ``check`` / ``message``.
    """
    if kind not in KINDS:
        raise ValueError(f"kind must be one of {KINDS}")
    cm = cm if cm is not None else cb.compile(model)
    q0 = np.asarray(q_start, float)
    lo, hi = (list(v) for v in dof_limits(model))
    for g in gripper_dofs(model):
        lo[g.dof_index] = hi[g.dof_index] = float(q0[g.dof_index])
    limits = (lo, hi)
    res = CurveResult(q_start=q0.copy(), speed=float(speed))
    T_cur = np.asarray(cb.fk_compiled(cm, list(q0))["tcp"], float)
    q_cur = q0.copy()
    pts, qs, ts, checks, offsets = [], [], [], [], []
    t_off = 0.0
    for i, (T1, via) in enumerate(stops):
        via = [np.asarray(v, float).reshape(3) for v in via]
        if kind == "line":
            via = []
        elif kind == "arc":
            via = via[:1] if via else [default_arc_via(T_cur, T1)]
        leg = CurveLeg(kind, T_cur.copy(), np.asarray(T1, float).copy(), via)
        path = solve_leg(model, leg, q_cur, speed, cm, limits)
        leg.cpath = path
        leg.check = check_path(model, path, collision, cm=cm)
        res.legs.append(leg)
        res.via_points += [v.copy() for v in via]
        sl = slice(1 if i > 0 else 0, None)
        p = np.asarray(path.targets)[:, :3, 3]
        pts.append(p[sl])
        ts.append(np.asarray(path.t)[sl] + t_off)
        checks.append(leg.check)
        offsets.append(t_off)
        qs.append(np.asarray(path.q)[sl] if len(path.q) else np.zeros((0, model.dof())))
        a = sum(len(x) for x in pts[:-1])
        res.leg_ranges.append((a if i == 0 else a - 1, a + len(pts[-1]) - 1))
        if not path.ok:
            res.message = f"Leg {i + 1}: {path.message}" if len(stops) > 1 else path.message
            if i < len(stops) - 1:
                res.message += f" Legs {i + 2}-{len(stops)} not computed."
            break
        t_off += float(path.t[-1])
        q_cur = np.asarray(path.q[-1], float)
        T_cur = np.asarray(leg.T1, float)
    res.points = np.vstack(pts) if pts else np.zeros((0, 3))
    res.t = np.concatenate(ts) if ts else np.zeros(0)
    if res.legs:
        res.check = merge_checks(checks, offsets)
        res.q = np.vstack([q for q in qs if len(q)]) if any(len(q) for q in qs) else \
            np.zeros((0, model.dof()))
        if len(res.q) and res.legs[0].cpath is not None and len(res.legs[0].cpath.q):
            res.q[0] = q0                             # exactly the pose the arm is in now
    res.length = float(np.sum(np.linalg.norm(np.diff(res.points, axis=0), axis=1))) \
        if len(res.points) > 1 else 0.0
    return res


def timed_trajectory(model: Any, res: CurveResult, timing: str = "speed", collision: Any = None,
                     speed: float | None = None) -> Trajectory:
    """``Trajectory`` along a good curve. ``timing="speed"``: constant TCP speed (the nominal
    times); ``"topp"``: time-optimal with velocity, acceleration and torque limits (F1.3).
    Raises ``ValueError`` when the curve is not ok, ``PlanningError`` when TOPP fails."""
    if timing not in TIMINGS:
        raise ValueError(f"timing must be one of {TIMINGS}")
    if not res.ok:
        raise ValueError("the curve has a problem: " + (res.problem_text() or "not solved"))
    q = np.asarray(res.q, float)
    if timing == "topp":
        from app.planning.shortcut import time_parametrize
        timed = time_parametrize(q, model, collision=collision, method="topp")
        tr = timed.trajectory
        # dense linear samples: the timeline keeps no Hermite velocities, so what it plays is
        # exactly what is returned here (chord error far below 0.1 mm at 0.04 s)
        ts = np.append(np.arange(0.0, tr.duration, DENSE_DT), tr.duration)
        wps = [Waypoint(float(t), tuple(float(v) for v in tr.q(float(t)))) for t in ts]
        wps[0] = Waypoint(0.0, tuple(float(v) for v in q[0]))
        wps[-1] = Waypoint(float(tr.duration), tuple(float(v) for v in q[-1]))
        return Trajectory(wps, "linear")
    t = np.asarray(res.t, float)
    if speed is not None and abs(speed - res.speed) > 1e-12:
        t = t * (res.speed / speed)
    return Trajectory([Waypoint(float(a), tuple(float(v) for v in b)) for a, b in zip(t, q)],
                      "linear")


def trajectory_speeds(model: Any, traj: Trajectory, n: int, cm: Any = None) -> np.ndarray:
    """TCP speed at ``n`` evenly spaced times of ``traj`` (m/s), for the tube colour "speed"."""
    cm = cm if cm is not None else cb.compile(model)
    ts = np.linspace(0.0, traj.duration, n)
    T = np.zeros((n, 4, 4))
    for k, t in enumerate(ts):
        T[k] = np.asarray(cb.fk_compiled(cm, list(traj.q(float(t))))["tcp"], float)
    return tcp_speed(T, ts)


def tcp_at_time(model: Any, traj: Trajectory | None, t: float, cm: Any = None) -> np.ndarray | None:
    """TCP position (world) at playback time ``t`` of ``traj``: the moving marker on the tube."""
    if traj is None:
        return None
    cm = cm if cm is not None else cb.compile(model)
    q = traj.q(min(max(float(t), 0.0), traj.duration))
    return np.asarray(cb.fk_compiled(cm, list(q))["tcp"], float)[:3, 3]
