"""R7.4: path shortcutting and time parametrisation of planned paths.

``shortcut`` removes waypoints by random pair shortcuts (never longer, never invalid).
``time_parametrize`` turns a waypoint path into a timed ``Trajectory`` (cubic through the
waypoints, linear fallback), scaled in time until ``check_trajectory`` has no velocity or
acceleration violation. Revolute wrap is not used: paths stay inside the joint limits.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Callable

import numpy as np

from app.planning.checks import GRAVITY, ViolationReport, _model_limits, check_trajectory
from app.sim.trajectory import Trajectory, Waypoint

DEFAULT_VMAX = 1.0      # rad/s (or m/s) when the joint has no velocity limit
DEFAULT_AMAX = 2.0


def path_length(path) -> float:
    p = np.asarray(path, float)
    if len(p) < 2:
        return 0.0
    return float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum())


def shortcut(path, edge_free: Callable[[np.ndarray, np.ndarray], bool],
             rng: np.random.Generator | None = None, iterations: int = 100,
             seed: int = 0, cancel: Callable[[], bool] | None = None) -> np.ndarray:
    """Random pair shortcut. ``edge_free(a, b)`` must be the validity test of the planner.

    Picks i < j - 1; when the straight edge ``path[i] -> path[j]`` is free the waypoints
    between are removed. By the triangle inequality the result is never longer; endpoints
    are kept.
    """
    p = [np.asarray(x, float) for x in path]
    rng = rng if rng is not None else np.random.default_rng(seed)
    for _ in range(int(iterations)):
        if len(p) < 3:
            break
        if cancel is not None and cancel():
            break
        i, j = sorted(int(x) for x in rng.choice(len(p), size=2, replace=False))
        if j - i < 2:
            continue
        if edge_free(p[i], p[j]):
            p = p[:i + 1] + p[j:]
    return np.array(p)


@dataclass
class TimedPath:
    trajectory: Trajectory
    report: ViolationReport
    interpolation: str
    time_scale: float = 1.0
    notes: list = field(default_factory=list)


def _segment_times(path: np.ndarray, vmax: np.ndarray, amax: np.ndarray) -> np.ndarray:
    """Per-segment duration: slowest DOF of a rest-to-rest trapezoid (all DOF in sync)."""
    out = []
    for a, b in zip(path[:-1], path[1:]):
        d = np.abs(b - a)
        t = 0.0
        for di, v, ac in zip(d, vmax, amax):
            if di <= 1e-12:
                continue
            if di >= v * v / ac:
                ti = di / v + v / ac
            else:
                ti = 2.0 * math.sqrt(di / ac)
            t = max(t, ti)
        out.append(max(t, 1e-3))
    return np.array(out)


def time_parametrize(path, model, limits: dict | None = None, vmax=None, amax=None,
                     interpolation: str = "cubic", collision=None, dt: float = 0.01,
                     check_torque: bool = False, max_scale_steps: int = 14,
                     method: str = "scale", n_grid: int = 100, gravity=GRAVITY) -> TimedPath:
    """Timed ``Trajectory`` through the waypoints of ``path`` (one segment per edge).

    Segment times come from ``vmax`` / ``amax`` (default: joint velocity limits, else
    ``DEFAULT_VMAX``; ``DEFAULT_AMAX``). The cubic spline can overshoot, so it is accepted only
    when a dense sample stays inside the position limits and (when ``collision`` is given)
    collision free; otherwise linear interpolation (no overshoot) is used. Times are scaled up by
    1.25 until ``check_trajectory`` shows no velocity/acceleration violation.

    ``method="topp"`` (F1.3): time-optimal parametrisation with velocity, acceleration AND torque
    limits (``app/planning/topp.py``, ``n_grid`` path samples, friction left out of the solve,
    ``gravity`` world vector). Torque limits come from ``limits["effort"]`` or the URM joint
    efforts. Raises ``PlanningError`` when the torque limits cannot be met rest-to-rest.
    """
    if method not in ("scale", "topp"):
        raise ValueError("method must be 'scale' or 'topp'")
    p = np.asarray(path, float)
    if p.ndim != 2 or len(p) < 2:
        raise ValueError("path needs at least 2 waypoints")
    n = p.shape[1]
    lim = _model_limits(model)
    v = np.broadcast_to(np.asarray(vmax if vmax is not None else DEFAULT_VMAX, float), (n,)).copy()
    if vmax is None:
        fin = np.isfinite(lim["vel"])
        v[fin] = lim["vel"][fin]
    a = np.broadcast_to(np.asarray(amax if amax is not None else DEFAULT_AMAX, float), (n,)).copy()
    seg = _segment_times(p, v, a)
    check_lim = dict(limits or {})
    check_lim.setdefault("vel", v)
    check_lim.setdefault("accel", a)
    notes: list[str] = []
    if method == "topp":
        return _time_parametrize_topp(p, model, lim, v, a, check_lim, interpolation, collision,
                                      dt, n_grid, gravity)

    def build(kind: str, scale: float) -> Trajectory:
        t = np.concatenate([[0.0], np.cumsum(seg * scale)])
        return Trajectory([Waypoint(float(ti), tuple(q)) for ti, q in zip(t, p)],
                          interpolation=kind, clamped=True)

    def acceptable(traj: Trajectory) -> bool:
        T = traj.duration
        ts = np.linspace(0.0, T, max(2, int(math.ceil(T / min(dt, 0.01))) + 1))
        qs = np.array([traj.q(x) for x in ts])
        if np.any(qs < lim["pos_lo"] - 1e-9) or np.any(qs > lim["pos_hi"] + 1e-9):
            return False
        if collision is not None and np.any(collision.check_batch(qs)):
            return False
        return True

    kind = interpolation
    scale = 1.0
    traj = None
    if kind == "cubic" and len(p) >= 2:
        traj = build("cubic", 1.0)
        if not acceptable(traj):
            notes.append("cubic spline left the free space: linear used")
            kind, traj = "linear", None
    if traj is None:
        kind = "linear" if kind != "cubic" else kind
        traj = build(kind, 1.0)
    for _ in range(max_scale_steps):
        rep = check_trajectory(traj, model, check_lim, dt=dt, check_torque=False)
        bad = [x for x in rep.violations if x.kind in ("velocity", "acceleration")]
        if not bad:
            break
        scale *= 1.25
        traj = build(kind, scale)
    rep = check_trajectory(traj, model, check_lim, dt=dt, check_torque=check_torque)
    return TimedPath(traj, rep, kind, scale, notes)


TOPP_MAX_RESCALE = 20


def _time_parametrize_topp(p, model, lim, v, a, check_lim, interpolation, collision, dt,
                           n_grid, gravity) -> TimedPath:
    """F1.3 helper: TOPP-RA solve, then rescale the time until ``check_trajectory`` is clean.

    The grid solve enforces the limits only at grid points and the cubic spline through
    ``(q(s_i), t_i)`` rings at acceleration jumps, so the checked trajectory can exceed a limit
    by a few percent. Time is then scaled up by ``max(1.02, 1.001 * sqrt(worst ratio))``
    (acceleration and dynamic torque scale with 1/scale^2, velocity with 1/scale), at most
    ``TOPP_MAX_RESCALE`` times.
    """
    from app.planning.topp import solution_trajectory, solve_topp
    eff = np.asarray(check_lim.get("effort", lim["effort"]), float)
    eff = np.broadcast_to(eff, (p.shape[1],)).copy()
    check_lim = dict(check_lim)
    check_lim["effort"] = eff
    sol, kind, notes = solve_topp(model, p, v, a, eff, gravity=gravity, n_grid=n_grid,
                                  interpolation=interpolation, collision=collision,
                                  pos_lo=lim["pos_lo"], pos_hi=lim["pos_hi"])
    scale = 1.0
    traj = solution_trajectory(sol, scale)
    rep = check_trajectory(traj, model, check_lim, dt=dt, check_torque=True, gravity=gravity)
    for _ in range(TOPP_MAX_RESCALE):
        bad = [x for x in rep.violations
               if x.kind in ("velocity", "acceleration", "torque") and x.limit > 0]
        if not bad:
            break
        worst_v = max([x.value / x.limit for x in bad if x.kind == "velocity"] or [1.0])
        worst_d = max([x.value / x.limit for x in bad if x.kind != "velocity"] or [1.0])
        scale *= max(1.02, 1.001 * math.sqrt(worst_d), 1.001 * worst_v)
        traj = solution_trajectory(sol, scale)
        rep = check_trajectory(traj, model, check_lim, dt=dt, check_torque=True, gravity=gravity)
    notes.append(f"topp: {kind} path geometry, {len(sol.q) - 1} stages, solver duration {sol.duration:.4g} s, "
                 f"final time scale {scale:.4g}")
    return TimedPath(traj, rep, "cubic", scale, notes)
