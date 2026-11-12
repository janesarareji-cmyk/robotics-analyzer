"""Timed waypoint trajectory (R6.4). No Qt dependency.

``Trajectory(waypoints, interpolation)``: ``q(t)``, ``qd(t)``, ``qdd(t)`` with ``t``
clamped to ``[0, duration]``. The first waypoint must have ``t == 0``.

* "linear": piecewise linear q, qd piecewise constant (right segment at a knot,
  left segment at the last knot), qdd = 0.
* "cubic": per-DOF cubic spline, C2 inside. ``clamped=True`` gives zero velocity
  at both ends; ``clamped=False`` gives the natural spline (zero qdd at the ends).
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

import numpy as np

INTERPOLATIONS = ("linear", "cubic")
DEFAULT_KP = 100.0
DEFAULT_KD = 20.0


@dataclass
class Waypoint:
    t: float
    q: tuple

    def __post_init__(self) -> None:
        self.t = float(self.t)
        self.q = tuple(float(x) for x in self.q)


def waypoint_problems(ts, qs, n_dof=None) -> dict[int, str]:
    """Row index -> first problem text for an invalid waypoint list.

    Checks: finite numbers, q length, first t == 0, strictly increasing t.
    The "at least 2 waypoints" rule is not per row (see ``Trajectory``).
    """
    out: dict[int, str] = {}
    n = n_dof if n_dof is not None else (len(qs[0]) if len(qs) else None)
    for i, (t, q) in enumerate(zip(ts, qs)):
        if t is None or not math.isfinite(t):
            out[i] = "t is not a number"
        elif any(x is None or not math.isfinite(x) for x in q):
            out[i] = "q has a non-numeric value"
        elif n is not None and len(q) != n:
            out[i] = f"q needs {n} values"
        elif i == 0 and t != 0.0:
            out[i] = "first t must be 0"
        elif i > 0 and not (ts[i - 1] is not None and math.isfinite(ts[i - 1]) and t > ts[i - 1]):
            out[i] = "t must be strictly increasing"
    return out


class Trajectory:
    def __init__(self, waypoints, interpolation: str = "linear", clamped: bool = True):
        wps = [w if isinstance(w, Waypoint) else Waypoint(*w) for w in waypoints]
        if interpolation not in INTERPOLATIONS:
            raise ValueError(f"unknown interpolation {interpolation!r}")
        if len(wps) < 2:
            raise ValueError("a trajectory needs at least 2 waypoints")
        probs = waypoint_problems([w.t for w in wps], [w.q for w in wps])
        if probs:
            i = min(probs)
            raise ValueError(f"waypoint {i}: {probs[i]}")
        self.waypoints = wps
        self.interpolation = interpolation
        self.clamped = bool(clamped)
        self._t = np.array([w.t for w in wps])
        self._y = np.array([w.q for w in wps])          # (n_wp, n_dof)
        self._M = self._second_derivs() if interpolation == "cubic" else None

    # ---------------------------------------------------------------- facts
    @property
    def n_dof(self) -> int:
        return self._y.shape[1]

    @property
    def duration(self) -> float:
        return float(self._t[-1])

    def limit_warnings(self, lo, hi) -> list[str]:
        """Waypoints outside ``[lo, hi]`` per DOF (warnings, not errors)."""
        out = []
        for i, w in enumerate(self.waypoints):
            for j, x in enumerate(w.q):
                if j < len(lo) and (x < lo[j] - 1e-12 or x > hi[j] + 1e-12):
                    out.append(f"waypoint {i} dof {j}: {x:.4g} outside [{lo[j]:.4g}, {hi[j]:.4g}]")
        return out

    # -------------------------------------------------------------- spline
    def _second_derivs(self) -> np.ndarray:
        t, y = self._t, self._y
        n = len(t)
        h = np.diff(t)
        s = np.diff(y, axis=0) / h[:, None]               # slopes (n-1, dof)
        A = np.zeros((n, n))
        r = np.zeros_like(y)
        for i in range(1, n - 1):
            A[i, i - 1], A[i, i], A[i, i + 1] = h[i - 1], 2 * (h[i - 1] + h[i]), h[i]
            r[i] = 6 * (s[i] - s[i - 1])
        if self.clamped:
            A[0, 0], A[0, 1], r[0] = 2 * h[0], h[0], 6 * s[0]
            A[-1, -2], A[-1, -1], r[-1] = h[-1], 2 * h[-1], -6 * s[-1]
        else:
            A[0, 0] = A[-1, -1] = 1.0
        return np.linalg.solve(A, r)

    def _locate(self, t: float):
        t = min(max(float(t), 0.0), self.duration)
        i = int(np.searchsorted(self._t, t, side="right")) - 1
        return t, min(max(i, 0), len(self._t) - 2)

    # ------------------------------------------------------------ sampling
    def q(self, t: float) -> np.ndarray:
        t, i = self._locate(t)
        for k in (i, i + 1):                              # exact at knots
            if t == self._t[k]:
                return self._y[k].copy()
        t0, t1 = self._t[i], self._t[i + 1]
        y0, y1, h = self._y[i], self._y[i + 1], t1 - t0
        if self._M is None:
            return y0 + (y1 - y0) * ((t - t0) / h)
        m0, m1 = self._M[i], self._M[i + 1]
        return (m0 * (t1 - t) ** 3 + m1 * (t - t0) ** 3) / (6 * h) \
            + (y0 / h - m0 * h / 6) * (t1 - t) + (y1 / h - m1 * h / 6) * (t - t0)

    def qd(self, t: float) -> np.ndarray:
        t, i = self._locate(t)
        t0, t1 = self._t[i], self._t[i + 1]
        y0, y1, h = self._y[i], self._y[i + 1], t1 - t0
        if self._M is None:
            return (y1 - y0) / h
        m0, m1 = self._M[i], self._M[i + 1]
        return (-m0 * (t1 - t) ** 2 + m1 * (t - t0) ** 2) / (2 * h) \
            - (y0 / h - m0 * h / 6) + (y1 / h - m1 * h / 6)

    def qdd(self, t: float) -> np.ndarray:
        t, i = self._locate(t)
        if self._M is None:
            return np.zeros(self.n_dof)
        t0, t1 = self._t[i], self._t[i + 1]
        h = t1 - t0
        return (self._M[i] * (t1 - t) + self._M[i + 1] * (t - t0)) / h

    # ------------------------------------------------------- serialization
    def to_dict(self) -> dict[str, Any]:
        return {
            "interpolation": self.interpolation,
            "clamped": self.clamped,
            "waypoints": [{"t": w.t, "q": list(w.q)} for w in self.waypoints],
        }

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> Trajectory:
        return cls([Waypoint(w["t"], w["q"]) for w in d["waypoints"]],
                   d.get("interpolation", "linear"), d.get("clamped", True))

    def __eq__(self, other) -> bool:
        return isinstance(other, Trajectory) and self.to_dict() == other.to_dict()

    __hash__ = None  # type: ignore[assignment]
