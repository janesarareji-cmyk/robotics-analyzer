"""UI-free simulation engine and clock state machine (R6.1).

The step integrator is pluggable: ``integrator(accel, t, q, qd, dt)`` returns
``(q_new, qd_new, dt_accepted)``. ``rk4_step`` is the fixed-step placeholder;
R6.2 supplies the real integrator module.
"""
from __future__ import annotations

import math

import numpy as np

import core_bridge as cb

from .state import SimState

STOPPED, RUNNING, PAUSED = "STOPPED", "RUNNING", "PAUSED"
MAX_TIME_SCALE = 100.0


def rk4_step(accel, t, q, qd, dt):
    """Fixed-step RK4 placeholder. ``accel(t, q, qd)`` -> qdd. Returns (q, qd, dt)."""
    k1q = qd
    k1v = accel(t, q, qd)
    k2q = qd + 0.5 * dt * k1v
    k2v = accel(t + 0.5 * dt, q + 0.5 * dt * k1q, k2q)
    k3q = qd + 0.5 * dt * k2v
    k3v = accel(t + 0.5 * dt, q + 0.5 * dt * k2q, k3q)
    k4q = qd + dt * k3v
    k4v = accel(t + dt, q + dt * k3q, k4q)
    q_new = q + dt / 6.0 * (k1q + 2 * k2q + 2 * k3q + k4q)
    qd_new = qd + dt / 6.0 * (k1v + 2 * k2v + 2 * k3v + k4v)
    return q_new, qd_new, dt


_INTEGRATORS = {"rk4": rk4_step}


class SimClock:
    """STOPPED / RUNNING / PAUSED state machine with callbacks (no Qt)."""

    def __init__(self):
        self._state = STOPPED
        self.on_state_changed = []  # callables(new_state, old_state)

    @property
    def state(self):
        return self._state

    def _go(self, allowed_from, new):
        if self._state not in allowed_from:
            raise ValueError(f"illegal transition {self._state} -> {new}")
        old, self._state = self._state, new
        for fn in list(self.on_state_changed):
            fn(new, old)

    def start(self):
        self._go((STOPPED,), RUNNING)

    def pause(self):
        self._go((RUNNING,), PAUSED)

    def resume(self):
        self._go((PAUSED,), RUNNING)

    def stop(self):
        self._go((RUNNING, PAUSED), STOPPED)


class SimEngine:
    """Holds the simulation state and steps it (forward dynamics)."""

    def __init__(self, model, settings, integrator=None, friction=None, armature=None,
                 max_steps_per_advance=200):
        self.model = model
        self.settings = settings
        self.cm = cb.compile(model)
        self.dm = cb.build_dyn_model(model, self.cm)
        self.n_dof = int(self.dm.n_dof)
        self.gravity = tuple(float(v) for v in settings.gravity)
        self.dt = float(settings.physics_timestep)
        self.friction = friction if friction is not None else self._friction_from(settings)
        a = armature if armature is not None else getattr(settings, "armature", None)
        self.armature = [float(v) for v in a] if a is not None else None
        if integrator is None:
            integrator = getattr(settings, "integrator", "rk4")
        if isinstance(integrator, str):
            if integrator not in _INTEGRATORS:
                raise ValueError(f"unknown integrator {integrator!r}")
            integrator = _INTEGRATORS[integrator]
        self.integrator = integrator
        self.max_steps_per_advance = int(max_steps_per_advance)
        self.clock = SimClock()
        self.lag = 0.0
        self._time_scale = 1.0
        self._tau = np.zeros(self.n_dof)
        self.reset(np.zeros(self.n_dof))

    # ------------------------------------------------------------ settings
    def _friction_from(self, settings):
        v = getattr(settings, "friction_viscous", None)
        c = getattr(settings, "friction_coulomb", None)
        if v is None and c is None:
            return None
        n = self.n_dof
        f = cb.Friction()
        f.viscous = [float(x) for x in (v if v is not None else [0.0] * n)]
        f.coulomb = [float(x) for x in (c if c is not None else [0.0] * n)]
        e = getattr(settings, "friction_eps", None)
        f.eps = [float(x) for x in (e if e is not None else [1e-3] * n)]
        return f

    @property
    def time_scale(self):
        return self._time_scale

    @time_scale.setter
    def time_scale(self, v):
        v = float(v)
        if not (0.0 < v <= MAX_TIME_SCALE):
            raise ValueError(f"time_scale must be in (0, {MAX_TIME_SCALE}]")
        self._time_scale = v

    @property
    def state(self):
        return self._state

    @property
    def tau(self):
        return self._tau.copy()

    def set_tau(self, tau):
        tau = np.asarray(tau, float)
        if tau.shape != (self.n_dof,):
            raise ValueError("tau needs one entry per DOF")
        self._tau = tau.copy()

    # --------------------------------------------------------------- state
    def _fill(self, s):
        q, qd = list(s.q), list(s.qd)
        s.tcp = np.asarray(cb.fk(self.model, q)["tcp"], float)
        ke = float(cb.kinetic_energy(self.dm, q, qd))
        pe = float(cb.potential_energy(self.dm, q, self.gravity))
        s.energy = {"ke": ke, "pe": pe, "total": ke + pe}

    def _accel(self, tau):
        def f(t, q, qd):
            return np.asarray(cb.forward_dynamics(
                self.dm, list(q), list(qd), list(tau), self.gravity,
                self.friction, self.armature), float)
        return f

    def _finish(self, s):
        """Fill qdd, tau_inv, tcp, energy of a state whose t, q, qd are set."""
        s.tau = self._tau.copy()
        s.qdd = self._accel(s.tau)(s.t, s.q, s.qd)
        s.tau_inv = np.asarray(cb.inverse_dynamics(
            self.dm, list(s.q), list(s.qd), list(s.qdd), self.gravity, self.friction), float)
        self._fill(s)
        return s

    def reset(self, q0, qd0=None):
        n = self.n_dof
        q0 = np.asarray(q0, float)
        qd0 = np.zeros(n) if qd0 is None else np.asarray(qd0, float)
        if q0.shape != (n,) or qd0.shape != (n,):
            raise ValueError("q0 and qd0 need n_dof entries")
        s = SimState.zeros(n)
        s.q, s.qd = q0.copy(), qd0.copy()
        self._finish(s)
        self._initial = s.copy()
        self._state = s
        self.lag = 0.0
        if self.clock.state != STOPPED:
            self.clock.stop()

    def step(self, dt=None):
        """One physics step of ``dt`` (default engine dt). Returns the new state."""
        dt = self.dt if dt is None else float(dt)
        if dt <= 0.0:
            raise ValueError("dt must be > 0")
        s = self._state
        q, qd, dt_used = self.integrator(self._accel(self._tau), s.t, s.q, s.qd, dt)
        new = SimState.zeros(self.n_dof)
        new.t = s.t + dt_used
        new.q, new.qd = np.asarray(q, float), np.asarray(qd, float)
        self._finish(new)
        self._state = new
        return new

    # --------------------------------------------------------------- clock
    def start(self):
        self.clock.start()

    def pause(self):
        self.clock.pause()

    def resume(self):
        self.clock.resume()

    def stop(self):
        """Stop and restore the initial state."""
        self.clock.stop()
        self._state = self._initial.copy()
        self.lag = 0.0

    def advance(self, wall_dt):
        """Advance by ``wall_dt * time_scale`` sim seconds (only when RUNNING).

        At most ``max_steps_per_advance`` steps; the rest is dropped and
        reported in ``lag``. Returns the sim time advanced.
        """
        if self.clock.state != RUNNING:
            return 0.0
        target = float(wall_dt) * self._time_scale
        if target <= 0.0:
            return 0.0
        n = max(1, math.ceil(target / self.dt - 1e-9))
        n_run = min(n, self.max_steps_per_advance)
        done = 0.0
        for _ in range(n_run):
            h = min(self.dt, target - done)
            if h <= 1e-12:
                break
            before = self._state.t
            self.step(h)
            done += self._state.t - before
        self.lag = max(0.0, target - done) if n > n_run else 0.0
        return done
