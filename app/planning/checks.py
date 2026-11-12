"""Constraint checks for trajectories (R7.1). No Qt.

``check_trajectory`` samples a trajectory and reports position, velocity, acceleration,
torque (RNEA with gravity) and optional TCP-speed violations. One entry per contiguous
violating run per (kind, dof); ``t`` and ``value`` are those of the worst sample.

Limits default to the URM joint limits: position ``[lower, upper]`` (``lower >= upper``
= unlimited), ``vel`` and ``effort`` (``<= 0`` = unlimited). The URM has no acceleration
limit, so ``accel`` is unlimited unless given in ``limits``. A missing limit is an info
line, never an error. ``limits`` overrides (keys ``pos_lo, pos_hi, vel, accel, effort,
tcp_speed``; arrays per DOF, ``None``/``inf`` = unlimited; ``tcp_speed`` scalar).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

import core_bridge as cb

from app.kin_utils import dof_limits

GRAVITY = (0.0, 0.0, -9.81)
KINDS = ("position", "velocity", "acceleration", "torque", "tcp_speed")


@dataclass
class Violation:
    kind: str
    dof: int            # -1 for tcp_speed
    t: float
    value: float
    limit: float


@dataclass
class ViolationReport:
    violations: list = field(default_factory=list)
    info: list = field(default_factory=list)
    n_samples: int = 0

    @property
    def ok(self) -> bool:
        return not self.violations

    def of_kind(self, kind: str) -> list:
        return [v for v in self.violations if v.kind == kind]

    def summary(self) -> str:
        if self.ok:
            head = f"OK: {self.n_samples} samples, no violations"
        else:
            head = f"{len(self.violations)} violation(s) in {self.n_samples} samples"
        lines = [head]
        for v in self.violations:
            where = "tcp" if v.dof < 0 else f"dof {v.dof}"
            lines.append(f"  {v.kind} {where} t={v.t:.4g}: {v.value:.4g} > limit {v.limit:.4g}")
        lines += [f"  info: {m}" for m in self.info]
        return "\n".join(lines)


def _model_limits(model) -> dict:
    n = model.dof()
    lo, hi = dof_limits(model)
    lo, hi = np.array(lo, float), np.array(hi, float)
    nolim = ~(lo < hi)
    lo[nolim], hi[nolim] = -math.inf, math.inf
    vel = np.full(n, math.inf)
    eff = np.full(n, math.inf)
    for j_idx, q_idx in enumerate(model.q_index_map()):
        if q_idx >= 0:
            lim = model.joints[j_idx].limits
            if lim.vel > 0:
                vel[q_idx] = lim.vel
            if lim.effort > 0:
                eff[q_idx] = lim.effort
    return {"pos_lo": lo, "pos_hi": hi, "vel": vel, "accel": np.full(n, math.inf),
            "effort": eff, "tcp_speed": math.inf}


def _arr(x, n):
    if x is None:
        return np.full(n, math.inf)
    return np.broadcast_to(np.asarray(x, float), (n,)).copy()


def _samples(traj, dt):
    """Return ``(t, q, qd, qdd)`` arrays from a trajectory-like object or a tuple."""
    if isinstance(traj, dict):
        traj = (traj["t"], traj["q"], traj["qd"], traj["qdd"])
    if isinstance(traj, (tuple, list)) and len(traj) == 4:
        return tuple(np.asarray(x, float) for x in traj)
    T = float(traj.duration)
    n = max(2, int(math.ceil(T / dt)) + 1)
    t = np.linspace(0.0, T, n)
    knots = getattr(traj, "waypoints", None)
    if knots:                                   # always check the knots themselves
        t = np.unique(np.concatenate([t, [w.t for w in knots]]))
    q = np.array([traj.q(x) for x in t])
    qd = np.array([traj.qd(x) for x in t])
    qdd = np.array([traj.qdd(x) for x in t])
    return t, q, qd, qdd


def _runs(report, kind, dof, t, val, lim, tol):
    """Add one Violation per contiguous run of ``val > lim`` (worst sample)."""
    bad = val > lim + tol
    i = 0
    while i < len(bad):
        if bad[i]:
            j = i
            while j + 1 < len(bad) and bad[j + 1]:
                j += 1
            k = i + int(np.argmax(val[i:j + 1]))
            report.violations.append(Violation(kind, dof, float(t[k]), float(val[k]), float(lim)))
            i = j + 1
        else:
            i += 1


def check_trajectory(traj, model, limits: dict | None = None, dt: float = 0.01,
                     gravity=GRAVITY, check_torque: bool = True, cm=None, dm=None,
                     tol: float = 1e-9) -> ViolationReport:
    """Check ``traj`` against limits. ``traj``: object with ``duration`` and ``q/qd/qdd(t)``
    (``Trajectory``, ``JointProfile``) or samples ``(t, q, qd, qdd)`` / dict of arrays.
    """
    n = model.dof()
    t, q, qd, qdd = _samples(traj, dt)
    if q.ndim != 2 or q.shape[1] != n:
        raise ValueError("trajectory needs one entry per DOF")
    lim = _model_limits(model)
    for k, v in (limits or {}).items():
        if k not in lim:
            raise ValueError(f"unknown limit key {k!r}")
        if k == "tcp_speed":
            lim[k] = math.inf if v is None else float(v)
        else:
            lim[k] = _arr(v, n)
    rep = ViolationReport(n_samples=len(t))
    for k, name in (("vel", "velocity"), ("accel", "acceleration"), ("effort", "torque")):
        miss = [i for i in range(n) if not math.isfinite(lim[k][i])]
        if miss:
            rep.info.append(f"no {name} limit for dof {miss}: unlimited")
    for i in range(n):
        if not (math.isfinite(lim["pos_lo"][i]) or math.isfinite(lim["pos_hi"][i])):
            rep.info.append(f"no position limit for dof {i}: unlimited")
    for i in range(n):
        _runs(rep, "position", i, t, lim["pos_lo"][i] - q[:, i], 0.0, tol)
        _runs(rep, "position", i, t, q[:, i] - lim["pos_hi"][i], 0.0, tol)
        _runs(rep, "velocity", i, t, np.abs(qd[:, i]), lim["vel"][i], tol)
        _runs(rep, "acceleration", i, t, np.abs(qdd[:, i]), lim["accel"][i], tol)
    # position runs store the overshoot, not q: report the actual q and the bound
    for v in rep.violations:
        if v.kind == "position":
            qi = float(q[int(np.argmin(np.abs(t - v.t))), v.dof])
            v.value = qi
            v.limit = float(lim["pos_hi"][v.dof] if qi > lim["pos_hi"][v.dof]
                            else lim["pos_lo"][v.dof])
    if check_torque and np.any(np.isfinite(lim["effort"])):
        cm = cm if cm is not None else cb.compile(model)
        dm = dm if dm is not None else cb.build_dyn_model(model, cm)
        tau = np.array([cb.inverse_dynamics(dm, list(q[k]), list(qd[k]), list(qdd[k]),
                                            tuple(gravity)) for k in range(len(t))])
        for i in range(n):
            _runs(rep, "torque", i, t, np.abs(tau[:, i]), lim["effort"][i], tol)
    if math.isfinite(lim["tcp_speed"]):
        cm = cm if cm is not None else cb.compile(model)
        sp = np.array([np.linalg.norm((np.asarray(cb.jacobian_tcp(cm, list(q[k]))) @ qd[k])[:3])
                       for k in range(len(t))])
        _runs(rep, "tcp_speed", -1, t, sp, lim["tcp_speed"], tol)
    else:
        rep.info.append("no TCP speed limit: not checked")
    return rep
