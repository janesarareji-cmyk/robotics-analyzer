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
        if k > 0 and step > max_joint_step:     # sample 0 may start far from the seed
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


# ---------------------------------------------------------------------------- spline (U4b.1)

def _cr_segment(P, T, u):
    """Barry-Goldman point of the centripetal Catmull-Rom piece P1..P2 (knots ``T``) at ``u`` in
    [0, 1]. ``P`` is a (4, 3) array, ``T`` the 4 knots."""
    t = T[1] + (T[2] - T[1]) * u
    A1 = (T[1] - t) / (T[1] - T[0]) * P[0] + (t - T[0]) / (T[1] - T[0]) * P[1]
    A2 = (T[2] - t) / (T[2] - T[1]) * P[1] + (t - T[1]) / (T[2] - T[1]) * P[2]
    A3 = (T[3] - t) / (T[3] - T[2]) * P[2] + (t - T[2]) / (T[3] - T[2]) * P[3]
    B1 = (T[2] - t) / (T[2] - T[0]) * A1 + (t - T[0]) / (T[2] - T[0]) * A2
    B2 = (T[3] - t) / (T[3] - T[1]) * A2 + (t - T[1]) / (T[3] - T[1]) * A3
    return (T[2] - t) / (T[2] - T[1]) * B1 + (t - T[1]) / (T[2] - T[1]) * B2


def spline_positions(points, n_per_seg: int = 30):
    """Centripetal Catmull-Rom curve through ``points`` (m >= 2, positions).

    Returns ``(pos (K, 3), key_idx)``: every point is an exact sample (``pos[key_idx[i]]``),
    ``n_per_seg`` samples per segment (end points included once). Two points give a straight line;
    collinear points stay on their line. Position is C1 across the via points.
    """
    P = np.asarray(points, float).reshape(-1, 3)
    m = len(P)
    if m < 2:
        raise ValueError("a spline needs at least 2 points")
    if n_per_seg < 2:
        raise ValueError("n_per_seg must be >= 2")
    ext = np.vstack([2 * P[0] - P[1], P, 2 * P[-1] - P[-2]])
    d = np.linalg.norm(np.diff(ext, axis=0), axis=1)
    d = np.sqrt(np.maximum(d, 1e-9))                       # centripetal: knot step = |dP| ** 0.5
    knots = np.concatenate([[0.0], np.cumsum(d)])
    out, key = [], [0]
    for i in range(m - 1):
        Pq, Tq = ext[i:i + 4], knots[i:i + 4]
        us = np.linspace(0.0, 1.0, n_per_seg)
        seg = np.array([_cr_segment(Pq, Tq, u) for u in us])
        seg[0], seg[-1] = P[i], P[i + 1]                  # exact at the key points
        out.append(seg if i == 0 else seg[1:])
        key.append(sum(len(s) for s in out) - 1)
    return np.vstack(out), key


def via_poses(T0, T1, positions):
    """Poses for via ``positions``: orientation = slerp from ``T0`` to ``T1`` by arc fraction of the
    polyline T0 -> via points -> T1 (the "orientation of the interpolation")."""
    T0, T1 = np.asarray(T0, float), np.asarray(T1, float)
    pts = np.vstack([T0[:3, 3], np.asarray(positions, float).reshape(-1, 3), T1[:3, 3]])
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    frac = np.cumsum(seg)[:-1] / max(float(seg.sum()), 1e-12)
    w = rot_log(T0[:3, :3].T @ T1[:3, :3])
    out = []
    for p, f in zip(pts[1:-1], frac):
        T = np.eye(4)
        T[:3, :3] = T0[:3, :3] @ rot_exp(f * w)
        T[:3, 3] = p
        out.append(T)
    return out


def spline(model, poses, q_seed, n_per_seg: int = 30, duration: float = 1.0,
           profile: str = "linear", ik_options: dict | None = None, max_joint_step: float = 0.5,
           cm=None) -> CartesianPath:
    """TCP path through ``poses`` (list of 4x4: start, via ..., end).

    Position: centripetal Catmull-Rom through all poses (``spline_positions``). Orientation: slerp
    between consecutive key poses by the fraction of the segment. Times follow the arc length
    (``profile="quintic"`` shapes them rest-to-rest). ``extra`` has ``key_idx`` (sample index of
    each pose) and ``length`` (m).
    """
    Ts = [np.asarray(T, float) for T in poses]
    if len(Ts) < 2:
        raise ValueError("a spline needs at least 2 poses")
    pos, key = spline_positions([T[:3, 3] for T in Ts], n_per_seg)
    n = len(pos)
    targets = np.tile(np.eye(4), (n, 1, 1))
    for i in range(len(Ts) - 1):
        a, b = key[i], key[i + 1]
        w = rot_log(Ts[i][:3, :3].T @ Ts[i + 1][:3, :3])
        for k in range(a, b + 1):
            f = (k - a) / (b - a)
            targets[k, :3, :3] = Ts[i][:3, :3] @ rot_exp(f * w)
    targets[:, :3, 3] = pos
    arc_len = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(pos, axis=0), axis=1))])
    length = float(arc_len[-1])
    a = arc_len / length if length > 1e-12 else np.linspace(0.0, 1.0, n)
    if profile == "linear":
        ts = a * duration
    elif profile == "quintic":
        g = np.linspace(0.0, 1.0, 2001)
        s = 10 * g ** 3 - 15 * g ** 4 + 6 * g ** 5
        ts = np.interp(a, s, g) * duration
    else:
        raise ValueError("profile must be 'linear' or 'quintic'")
    res = _solve(model, cm, targets, ts, q_seed, ik_options, max_joint_step)
    res.extra.update(key_idx=key, length=length)
    return res
