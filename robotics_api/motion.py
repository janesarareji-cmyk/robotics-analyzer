"""Trajectories, simulation and path planning (R8.1).

Units: rad (or m), rad/s, s. ``simulate``, ``plan`` and ``workspace`` call ``check_cancel()``
so the script runner can stop them (``ScriptCancelled``).
"""
from __future__ import annotations

import math
from typing import Any, Callable

import numpy as np

from ._cancel import check_cancel, current_token
from ._util import gravity3, guard, shape_from_dict, vec
from .errors import (InvalidArgument, PlanningError, RoboticsApiError, ScriptCancelled,
                     SimulationError)
from .robot import Robot

PLANNERS = ("rrt_connect", "rrt_star", "prm")
INTEGRATORS = ("euler", "rk4", "rk45")
MAX_SAMPLES = 5_000_000


class Trajectory:
    """A timed joint trajectory through waypoints.

    ``times``: strictly increasing seconds starting at 0. ``waypoints``: array ``(n, n_dof)``.
    ``interpolation``: ``"linear"`` or ``"cubic"`` (clamped spline, zero speed at both ends).
    ``q(t)``, ``qd(t)``, ``qdd(t)`` return arrays of n_dof; ``t`` is clamped to the duration.
    After ``time_parametrize``: ``ok`` is True when the limit check found no violation and
    ``report`` is its text.
    """

    def __init__(self, times: Any, waypoints: Any, interpolation: str = "cubic"):
        from app.sim.trajectory import INTERPOLATIONS, Trajectory as _T, Waypoint
        t = np.asarray(times, dtype=float)
        w = np.asarray(waypoints, dtype=float)
        if t.ndim != 1 or w.ndim != 2 or len(t) != len(w):
            raise InvalidArgument("times needs one entry per waypoint row, waypoints is 2-D")
        if interpolation not in INTERPOLATIONS:
            raise InvalidArgument(f"interpolation must be one of {INTERPOLATIONS}")
        try:
            self._impl = _T([Waypoint(a, b) for a, b in zip(t, w)], interpolation)
        except ValueError as exc:
            raise InvalidArgument(str(exc)) from exc
        self.ok = True
        self.report = ""

    @classmethod
    def _wrap(cls, impl, ok: bool = True, report: str = "") -> "Trajectory":
        obj = cls.__new__(cls)
        obj._impl, obj.ok, obj.report = impl, ok, report
        return obj

    @property
    def n_dof(self) -> int:
        """Number of joints (columns of each waypoint)."""
        return int(self._impl.n_dof)

    @property
    def duration(self) -> float:
        """Duration in seconds."""
        return float(self._impl.duration)

    @property
    def interpolation(self) -> str:
        """Interpolation name: 'linear' or 'cubic' (as given to the constructor)."""
        return self._impl.interpolation

    @property
    def times(self) -> np.ndarray:
        """Waypoint times in seconds, shape (n,)."""
        return np.array([w.t for w in self._impl.waypoints])

    @property
    def waypoints(self) -> np.ndarray:
        """Waypoint joint vectors, shape (n, n_dof)."""
        return np.array([w.q for w in self._impl.waypoints])

    def q(self, t: float) -> np.ndarray:
        """Joint vector at time ``t`` (s); clamped to the trajectory ends."""
        return self._impl.q(float(t))

    def qd(self, t: float) -> np.ndarray:
        """Joint velocity at time ``t`` (rad/s or m/s)."""
        return self._impl.qd(float(t))

    def qdd(self, t: float) -> np.ndarray:
        """Joint acceleration at time ``t`` (rad/s^2 or m/s^2)."""
        return self._impl.qdd(float(t))

    def sample(self, dt: float) -> dict:
        """Samples every ``dt`` seconds (last sample at the duration): dict ``t, q, qd, qdd``."""
        if not (float(dt) > 0):
            raise InvalidArgument("dt must be > 0")
        n = int(math.ceil(self.duration / float(dt) - 1e-12)) + 1
        t = np.minimum(np.arange(n) * float(dt), self.duration)
        return {"t": t, "q": np.array([self.q(x) for x in t]),
                "qd": np.array([self.qd(x) for x in t]),
                "qdd": np.array([self.qdd(x) for x in t])}

    def __repr__(self) -> str:
        return f"Trajectory(n_dof={self.n_dof}, duration={self.duration:.4g} s)"


def _cancel_or_raise_if(reason: str) -> None:
    if reason == "cancelled":
        check_cancel()
        raise ScriptCancelled("cancelled")


# ----------------------------------------------------------------------------- simulate
@guard(SimulationError)
def simulate(robot: Robot, control: Any = None, duration: float | None = None, dt: float = 0.001,
             integrator: str = "rk4", q0: Any = None, qd0: Any = None, gravity: Any = None,
             record_every: int = 1, kp: Any = None, kd: Any = None) -> dict:
    """Simulate the robot dynamics and return the recorded signals as arrays.

    ``control``: ``None`` (zero torque), a constant torque vector (N.m), a callable
    ``tau = f(t, q, qd)`` (numpy arrays in, n_dof torques out), or a ``Trajectory`` (computed
    torque tracking with gains ``kp``, ``kd``, default 100 and 20). ``duration`` in s (default
    the trajectory duration). ``dt`` in s. ``integrator``: euler, rk4 or rk45. ``q0``/``qd0``:
    initial state (default zeros, or the first trajectory point). ``gravity``: world vector
    (default [0, 0, -9.81]). ``record_every``: keep every n-th step.

    Returns a dict of numpy arrays: ``t`` (N,) and ``q, qd, qdd, tau`` (N, n_dof) plus
    ``tcp_pos`` (m), ``tcp_quat`` (w,x,y,z), ``tcp_vel`` (m/s), ``ke, pe, total`` (J),
    ``power`` (W per DOF), ``power_sum``, ``sigma_min``, ``manipulability``: the same signals
    as the Recorder of the app (see docs/simulation.md). Sample 0 is the initial state.
    """
    from app.project.project_data import SimulationSettings
    from app.sim import DYNAMIC, Recorder, SimEngine
    from app.sim.playback import TrajectoryPlayer
    from app.sim.recorder import MEM_MB_MAX, capacity_for, signal_layout

    n = robot.n_dof
    if integrator not in INTEGRATORS:
        raise InvalidArgument(f"integrator must be one of {INTEGRATORS}")
    traj = control if isinstance(control, Trajectory) else None
    if traj is not None and traj.n_dof != n:
        raise InvalidArgument(f"trajectory has {traj.n_dof} DOF, the robot has n_dof = {n}")
    if duration is None:
        if traj is None:
            raise InvalidArgument("duration is required unless control is a Trajectory")
        duration = traj.duration
    duration, dt = float(duration), float(dt)
    if not (duration > 0 and math.isfinite(duration)) or not (dt > 0 and math.isfinite(dt)):
        raise InvalidArgument("duration and dt must be positive numbers")
    if int(record_every) < 1:
        raise InvalidArgument("record_every must be >= 1")
    n_steps = int(math.ceil(duration / dt - 1e-9))
    if n_steps > MAX_SAMPLES:
        raise InvalidArgument(f"{n_steps} steps: too many, raise dt")
    rows = n_steps // int(record_every) + 2
    _layout, width = signal_layout(n)
    mem_mb = max(1.0, rows * 8.0 * (width + 1) / (1024 * 1024) * 1.01)
    if mem_mb > MEM_MB_MAX or capacity_for(mem_mb, width) < rows:
        raise InvalidArgument(f"{rows} samples need {mem_mb:.0f} MiB: raise record_every or dt")

    q_init = (traj.q(0.0) if traj is not None else np.zeros(n)) if q0 is None \
        else vec(q0, n, "q0")
    qd_init = np.zeros(n) if qd0 is None else vec(qd0, n, "qd0")
    settings = SimulationSettings(integrator=integrator, physics_timestep=dt,
                                  gravity=list(gravity3(gravity)))
    torque_fn: Callable | None = None
    const_tau = None
    if traj is not None:
        pass
    elif callable(control):
        torque_fn = control
    elif control is not None:
        const_tau = vec(control, n, "control (torque)")

    engine = SimEngine(robot._model, settings)
    rec = Recorder(engine, memory_mb=min(float(MEM_MB_MAX), mem_mb),
                   record_every=int(record_every), overwrite=False)
    player = None
    try:
        if traj is not None:
            player = TrajectoryPlayer(engine, traj._impl, DYNAMIC, kp, kd)
        elif torque_fn is not None:
            def tau_of(t, q, qd, _f=torque_fn):
                out = np.asarray(_f(float(t), np.array(q), np.array(qd)), dtype=float)
                if out.shape != (n,) or not np.all(np.isfinite(out)):
                    raise InvalidArgument(f"control must return {n} finite torques")
                return out
            engine.torque_fn = tau_of
        elif const_tau is not None:
            engine.set_tau(const_tau)
        engine.reset(q_init, qd_init)           # recompute the initial state with the torque
        engine.start()
        rec.sample(engine.state)
        adaptive = hasattr(engine.integrator, "next_dt")
        while engine.state.t < duration - 1e-12:
            check_cancel()
            h = dt
            if adaptive:
                h = min(h, engine.integrator.next_dt or dt)
            h = min(h, duration - engine.state.t)
            engine.step(h, full=False)
        if rec.stopped_full:
            raise SimulationError("recording buffer full: raise record_every or dt")
        arrays = rec.export_arrays()
        out = {"t": arrays["q"][0]}
        for name, (_t, data) in arrays.items():
            out[name] = data
        return out
    except RoboticsApiError:
        raise
    except RuntimeError as exc:
        raise SimulationError(f"simulation failed at t = {engine.state.t:.6g} s: {exc}") from exc
    finally:
        rec.detach()
        if player is not None:
            player.detach()
        engine.torque_fn = None


# ----------------------------------------------------------------------- time parametrize
@guard(PlanningError)
def time_parametrize(path: Any, robot: Robot, vmax: Any = None, amax: Any = None,
                     interpolation: str = "cubic") -> Trajectory:
    """Timed trajectory through the waypoints of ``path`` (array ``(n, n_dof)``, rad or m).

    ``vmax`` (rad/s or m/s) and ``amax`` (rad/s^2) per DOF or scalar; default the joint velocity
    limits (1 where none) and 2. Segment times are scaled up until no velocity or acceleration
    limit is violated. Cubic interpolation falls back to linear when the spline would leave the
    joint limits. Returns a ``Trajectory`` (``ok``, ``report``).
    """
    from app.planning.shortcut import time_parametrize as _tp
    p = np.asarray(path, dtype=float)
    if p.ndim != 2 or p.shape[1] != robot.n_dof or len(p) < 2:
        raise InvalidArgument(f"path must have shape (n >= 2, {robot.n_dof})")
    if not np.all(np.isfinite(p)):
        raise InvalidArgument("path has a non-finite value")
    if interpolation not in ("linear", "cubic"):
        raise InvalidArgument("interpolation must be 'linear' or 'cubic'")
    for name, v in (("vmax", vmax), ("amax", amax)):
        if v is not None:
            a = np.asarray(v, dtype=float)
            if a.ndim > 1 or (a.ndim == 1 and a.shape[0] != robot.n_dof) or np.any(a <= 0):
                raise InvalidArgument(f"{name} must be positive: scalar or n_dof values")
    timed = _tp(p, robot._model, vmax=vmax, amax=amax, interpolation=interpolation)
    rep = timed.report
    return Trajectory._wrap(timed.trajectory, bool(rep.ok), rep.summary())


# ------------------------------------------------------------------------------- plan
def _obstacle(d: dict):
    from app.planning.collision import EnvShape
    _shape, xyz, rpy, params = shape_from_dict(d, ("box", "sphere", "cylinder"))
    kind = d.get("type", d.get("kind"))
    return EnvShape(kind, params, xyz=xyz, rpy=rpy, name=str(d.get("name", "")))


@guard(PlanningError)
def plan(robot: Robot, start: Any, goal: Any, obstacles: list | None = None,
         planner: str = "rrt_connect", seed: int = 0, time_limit: float = 5.0,
         resolution: float = 0.02, shortcut: bool = True) -> np.ndarray:
    """Collision free joint space path from ``start`` to ``goal``.

    ``start``: n_dof joint values (rad or m). ``goal``: n_dof joint values or a 4x4 TCP pose
    (inverse kinematics solutions become the goals). ``obstacles``: list of dicts
    ``{"type": "box" | "sphere" | "cylinder", "pose": ..., "size": ...}`` in the world frame
    (pose: 4x4, ``{"xyz","rpy"}`` or 3 / 6 numbers in m and rad; size: box [x, y, z] m, sphere
    radius m, cylinder [radius, length] m). The robot collision geometry comes from the imported
    URDF (or ``Robot.set_collision_shapes``). ``planner``: rrt_connect, rrt_star or prm.
    ``seed``: same seed, same path. ``time_limit`` in s, ``resolution`` = edge check spacing in
    rad. Returns an array ``(n_points, n_dof)``, start first, goal last. Raises ``PlanningError``
    when no path is found.
    """
    from app.planning.collision import CollisionConfig, CollisionWorld
    from app.planning.planner import plan as _plan

    if planner not in PLANNERS:
        raise InvalidArgument(f"planner must be one of {PLANNERS}")
    n = robot.n_dof
    s = vec(start, n, "start")
    g = np.asarray(goal, dtype=float)
    if g.shape == (4, 4):
        from ._util import pose4
        g = pose4(g, "goal")
    else:
        g = vec(goal, n, "goal")
    envs = [_obstacle(d) for d in (obstacles or [])]
    shapes = robot._link_shapes()
    world = None
    if envs and not shapes:
        raise InvalidArgument("the robot has no collision geometry: import a URDF with "
                              "geometry or call Robot.set_collision_shapes")
    if shapes:
        world = CollisionWorld(robot._model, shapes, CollisionConfig(environment=envs))
    cancel = current_token()
    check_cancel()
    res = _plan(robot._model, s, g, world, planner=planner, time_limit=float(time_limit),
                seed=int(seed), resolution=float(resolution), cancel=cancel,
                do_shortcut=bool(shortcut))
    if not res.success:
        _cancel_or_raise_if(res.reason)
        raise PlanningError(f"no path: {res.reason}")
    return np.asarray(res.path, dtype=float)
