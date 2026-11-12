"""Cartesian TCP paths solved by IK (R7.1). No Qt.

``line`` and ``arc`` sample a TCP path (position + orientation slerp), solve each sample
with ``cb.solve_ik`` seeded from the previous solution, and enforce branch continuity:
a sample whose solution jumps more than ``max_joint_step`` (inf-norm) counts as failed.
Failure is a result (``failed_at`` index), never an exception. ``q`` holds the samples
solved before the failure.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

import core_bridge as cb

from app.kin_utils import dof_limits
from app.sim.trajectory import Trajectory, Waypoint


def _skew(w):
    return np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]], float)


def rot_exp(w):
    """Rodrigues: rotation matrix of the axis-angle vector ``w``."""
    th = float(np.linalg.norm(w))
    if th < 1e-12:
        return np.eye(3) + _skew(w)
    K = _skew(np.asarray(w, float) / th)
    return np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * (K @ K)


def rot_log(R):
    """Axis-angle vector of ``R`` (angle in [0, pi])."""
    c = min(1.0, max(-1.0, (np.trace(R) - 1.0) / 2.0))
    th = math.acos(c)
    if th < 1e-9:
        return np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / 2
    if math.pi - th < 1e-6:                      # near pi: use the symmetric part
        B = (R + np.eye(3)) / 2
        i = int(np.argmax(np.diag(B)))
        ax = B[:, i] / math.sqrt(max(B[i, i], 1e-300))
        return th * ax / np.linalg.norm(ax)
    w = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    return th * w / (2 * math.sin(th))


@dataclass
class CartesianPath:
    """Result of ``line`` / ``arc``. ``q`` is ``(k, dof)``; ``k == n`` when ``ok``."""
    t: np.ndarray                    # (n,) sample times
    targets: np.ndarray              # (n, 4, 4) wanted TCP poses
    q: np.ndarray                    # (k, dof) solved samples
    failed_at: int | None = None
    message: str = ""
    extra: dict = field(default_factory=dict)

    @property
    def ok(self) -> bool:
        return self.failed_at is None

    def to_trajectory(self, interpolation: str = "linear") -> Trajectory:
        if not self.ok:
            raise ValueError(f"path failed at sample {self.failed_at}: {self.message}")
        return Trajectory([Waypoint(float(t), tuple(q)) for t, q in zip(self.t, self.q)],
                          interpolation)


def _s_values(n, profile):
    u = np.linspace(0.0, 1.0, n)
    if profile == "linear":
        return u, u
    if profile == "quintic":
        return u, 10 * u ** 3 - 15 * u ** 4 + 6 * u ** 5
    raise ValueError("profile must be 'linear' or 'quintic'")


def _solve(model, cm, targets, ts, q_seed, ik_options, max_joint_step):
    lo, hi = dof_limits(model)
    cm = cm if cm is not None else cb.compile(model)
    prev = np.asarray(q_seed, float)
    if prev.size != model.dof():
        raise ValueError("q_seed needs one entry per DOF")
    out, failed, msg = [], None, ""
    for k, T in enumerate(targets):
        r = cb.solve_ik(cm, lo, hi, T, list(prev), ik_options)
        q = np.asarray(r["q"], float)
        if not r["converged"]:
            failed, msg = k, f"IK did not converge ({r['message']})"
            break
        step = float(np.max(np.abs(q - prev))) if q.size else 0.0
        if step > max_joint_step:
            failed, msg = k, f"joint step {step:.3g} > {max_joint_step:.3g} (branch jump)"
            break
        out.append(q)
        prev = q
    qs = np.array(out).reshape(len(out), model.dof())
    return CartesianPath(np.asarray(ts, float), np.asarray(targets), qs, failed, msg)


def line(model, T0, T1, q_seed, n: int = 50, duration: float = 1.0, profile: str = "linear",
         ik_options: dict | None = None, max_joint_step: float = 0.5, cm=None) -> CartesianPath:
    """Straight TCP line from pose ``T0`` to ``T1`` (4x4): linear position, slerp orientation.

    ``n`` samples (>= 2) at uniform times over ``duration``; ``profile`` shapes the path
    parameter (``"linear"`` or ``"quintic"`` rest-to-rest). ``q_seed`` seeds sample 0.
    """
    if n < 2:
        raise ValueError("n must be >= 2")
    T0, T1 = np.asarray(T0, float), np.asarray(T1, float)
    u, s = _s_values(n, profile)
    w = rot_log(T0[:3, :3].T @ T1[:3, :3])
    targets = np.tile(np.eye(4), (n, 1, 1))
    for k, sk in enumerate(s):
        targets[k, :3, :3] = T0[:3, :3] @ rot_exp(sk * w)
        targets[k, :3, 3] = T0[:3, 3] + sk * (T1[:3, 3] - T0[:3, 3])
    return _solve(model, cm, targets, u * duration, q_seed, ik_options, max_joint_step)


def arc_geometry(p0, pv, p1):
    """Circle through three points: ``(center, radius, e1, e2, sweep)``.

    Points move counter-clockwise about ``e1 x e2``; ``sweep`` in (0, 2 pi) is the
    angle from ``p0`` through ``pv`` to ``p1``.
    """
    p0, pv, p1 = (np.asarray(p, float) for p in (p0, pv, p1))
    a, b = p0 - p1, pv - p1
    n = np.cross(a, b)
    nn = float(n @ n)
    if nn < 1e-18:
        raise ValueError("arc points are collinear")
    center = p1 + np.cross(float(a @ a) * b - float(b @ b) * a, n) / (2 * nn)
    r = float(np.linalg.norm(p0 - center))
    e1 = (p0 - center) / r
    e3 = np.cross(pv - p0, p1 - p0)
    e3 /= np.linalg.norm(e3)
    e2 = np.cross(e3, e1)
    sweep = math.atan2((p1 - center) @ e2, (p1 - center) @ e1) % (2 * math.pi)
    return center, r, e1, e2, sweep


def arc(model, T0, via, T1, q_seed, n: int = 50, duration: float = 1.0, profile: str = "linear",
        ik_options: dict | None = None, max_joint_step: float = 0.5, cm=None) -> CartesianPath:
    """Circular TCP arc from ``T0`` through the point ``via`` to ``T1`` (positions of the
    three points define the circle; orientation slerps from ``T0`` to ``T1`` with the arc
    fraction). Collinear points raise ``ValueError``.
    """
    if n < 2:
        raise ValueError("n must be >= 2")
    T0, T1 = np.asarray(T0, float), np.asarray(T1, float)
    c, r, e1, e2, sweep = arc_geometry(T0[:3, 3], via, T1[:3, 3])
    u, s = _s_values(n, profile)
    w = rot_log(T0[:3, :3].T @ T1[:3, :3])
    targets = np.tile(np.eye(4), (n, 1, 1))
    for k, sk in enumerate(s):
        th = sk * sweep
        targets[k, :3, :3] = T0[:3, :3] @ rot_exp(sk * w)
        targets[k, :3, 3] = c + r * (math.cos(th) * e1 + math.sin(th) * e2)
    res = _solve(model, cm, targets, u * duration, q_seed, ik_options, max_joint_step)
    res.extra.update(center=c, radius=r, sweep=sweep)
    return res
