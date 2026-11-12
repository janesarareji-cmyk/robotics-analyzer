"""Synchronised joint-space point-to-point profiles (R7.1). No Qt.

Synchronisation choice: every DOF follows ``q_i(t) = q0_i + d_i * s(t)`` with
``d_i = q1_i - q0_i`` and ONE rest-to-rest scalar profile ``s: [0,T] -> [0,1]``.
The limits of ``s`` are the tightest ``limit_i / |d_i|`` over the moving DOF, so the
slowest DOF runs at its limit and the others are scaled copies (same shape, all finish
at ``T``). A DOF with ``d_i == 0`` stays put (no division by zero). If no DOF moves,
``duration == 0`` and ``to_trajectory`` raises ``ValueError``.

* ``trapezoid``: accel ``+-a``, cruise ``v``. Closed form: ``T = 1/v + v/a`` if
  ``v^2/a < 1`` else triangular ``T = 2 sqrt(1/a)`` (unit distance).
* ``scurve``: 7 segments of constant jerk ``+-j`` (jerk-limited), limits v, a, j.
* ``quintic``: ``s = 10u^3 - 15u^4 + 6u^5``, ``u = t/T``, zero v and a at both ends.
"""
from __future__ import annotations

import math

import numpy as np

from app.sim.trajectory import Trajectory, Waypoint


class _Scalar:
    """Piecewise constant-jerk rest-to-rest profile ``s(t)`` from 0 to 1."""

    def __init__(self, segs):
        # segs: list of (duration, a_start, jerk)
        self.segs = [(float(d), float(a), float(j)) for d, a, j in segs if d > 0.0]
        self.T = sum(d for d, _, _ in self.segs)
        self._start = []                       # (t, s, v) at each segment start
        t = s = v = 0.0
        for d, a, j in self.segs:
            self._start.append((t, s, v))
            s += v * d + a * d * d / 2 + j * d ** 3 / 6
            v += a * d + j * d * d / 2
            t += d
        self._ends = np.array([st[0] for st in self._start[1:]] + [self.T])

    def __call__(self, t):
        if t <= 0.0:
            return 0.0, 0.0, 0.0
        if t >= self.T:
            return 1.0, 0.0, 0.0
        i = min(int(np.searchsorted(self._ends, t, side="right")), len(self.segs) - 1)
        d, a0, j = self.segs[i]
        t0, s0, v0 = self._start[i]
        tau = t - t0
        return (s0 + v0 * tau + a0 * tau * tau / 2 + j * tau ** 3 / 6,
                v0 + a0 * tau + j * tau * tau / 2,
                a0 + j * tau)


class _Quintic:
    def __init__(self, T):
        self.T = float(T)

    def __call__(self, t):
        if t <= 0.0:
            return 0.0, 0.0, 0.0
        if t >= self.T:
            return 1.0, 0.0, 0.0
        u, T = t / self.T, self.T
        return (10 * u ** 3 - 15 * u ** 4 + 6 * u ** 5,
                (30 * u ** 2 - 60 * u ** 3 + 30 * u ** 4) / T,
                (60 * u - 180 * u ** 2 + 120 * u ** 3) / T ** 2)


class JointProfile:
    """Callable synchronised profile. ``q/qd/qdd(t)`` return ``(n_dof,)`` arrays.

    ``t`` is clamped to ``[0, duration]`` (rest at both ends).
    """

    def __init__(self, q0, q1, scalar, kind: str):
        self.q0 = np.asarray(q0, float).copy()
        self.q1 = np.asarray(q1, float).copy()
        if self.q0.shape != self.q1.shape or self.q0.ndim != 1:
            raise ValueError("q0 and q1 need the same 1-D shape")
        self.d = self.q1 - self.q0
        self._s = scalar
        self.kind = kind
        self.duration = float(scalar.T)

    @property
    def n_dof(self) -> int:
        return self.q0.size

    def q(self, t):
        s, _, _ = self._s(t)
        if t >= self.duration:
            return self.q1.copy()
        return self.q0 + self.d * s

    def qd(self, t):
        return self.d * self._s(t)[1]

    def qdd(self, t):
        return self.d * self._s(t)[2]

    __call__ = q

    def to_trajectory(self, dt: float, interpolation: str = "linear") -> Trajectory:
        """Sample at ``dt`` (last knot exactly at ``duration``) into a ``Trajectory``."""
        if not dt > 0.0:
            raise ValueError("dt must be > 0")
        if not self.duration > 0.0:
            raise ValueError("zero-duration profile: nothing to sample")
        n = max(1, int(math.ceil(self.duration / dt - 1e-9)))
        ts = np.linspace(0.0, self.duration, n + 1)
        return Trajectory([Waypoint(float(t), tuple(self.q(float(t)))) for t in ts],
                          interpolation)


def _limit(name, x, d):
    """Scalar limit on ``s`` from per-DOF limits ``x`` and moves ``d`` (>0 required)."""
    x = np.broadcast_to(np.asarray(x, float), d.shape)
    if not np.all(x > 0.0) or not np.all(np.isfinite(x)):
        raise ValueError(f"{name} must be finite and > 0")
    mv = np.abs(d) > 0.0
    if not mv.any():
        return math.inf
    return float(np.min(x[mv] / np.abs(d[mv])))


def trapezoid(q0, q1, vmax, amax) -> JointProfile:
    """Synchronised trapezoidal velocity profile (see module docstring)."""
    q0, q1 = np.asarray(q0, float), np.asarray(q1, float)
    d = q1 - q0
    v, a = _limit("vmax", vmax, d), _limit("amax", amax, d)
    if math.isinf(v):
        return JointProfile(q0, q1, _Scalar([]), "trapezoid")
    if v * v / a >= 1.0:                         # triangular
        ta, tv = math.sqrt(1.0 / a), 0.0
    else:
        ta, tv = v / a, 1.0 / v - v / a
    return JointProfile(q0, q1, _Scalar([(ta, a, 0.0), (tv, 0.0, 0.0), (ta, -a, 0.0)]),
                        "trapezoid")


def quintic(q0, q1, T) -> JointProfile:
    """Quintic polynomial profile of duration ``T`` (zero v, a at both ends)."""
    if not (T > 0.0 and math.isfinite(T)):
        raise ValueError("T must be finite and > 0")
    return JointProfile(q0, q1, _Quintic(T), "quintic")


def scurve(q0, q1, vmax, amax, jmax) -> JointProfile:
    """Synchronised jerk-limited 7-segment S-curve."""
    q0, q1 = np.asarray(q0, float), np.asarray(q1, float)
    d = q1 - q0
    v, a, j = _limit("vmax", vmax, d), _limit("amax", amax, d), _limit("jmax", jmax, d)
    if math.isinf(v):
        return JointProfile(q0, q1, _Scalar([]), "scurve")
    # accel phase for peak velocity v
    if v * j >= a * a:
        tj, ta = a / j, v / a - a / j
    else:
        tj, ta = math.sqrt(v / j), 0.0
    t_acc = 2 * tj + ta
    if v * t_acc <= 1.0:                         # cruise at v
        tv = (1.0 - v * t_acc) / v
    else:                                        # v not reached: peak from distance
        tv = 0.0
        tj = (1.0 / (2 * j)) ** (1.0 / 3.0)
        ta = 0.0
        if j * tj > a:                           # accel limit reached
            tj = a / j
            ta = (-3 * tj + math.sqrt(tj * tj + 4.0 / a)) / 2
    segs = [(tj, 0.0, j), (ta, j * tj, 0.0), (tj, j * tj, -j), (tv, 0.0, 0.0),
            (tj, 0.0, -j), (ta, -j * tj, 0.0), (tj, -j * tj, j)]
    return JointProfile(q0, q1, _Scalar(segs), "scurve")
