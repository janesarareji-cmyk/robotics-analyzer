"""Trajectory playback on the simulation engine (R6.4). No Qt dependency.

KINEMATIC: the engine clock runs (STOPPED -> RUNNING), ``advance`` sets the state to
the trajectory sample (q, qd, qdd, RNEA torque); no dynamics step.
DYNAMIC ("track with computed torque"): ``engine.torque_fn`` = RNEA(q_d, qd_d, qdd_d)
+ Kp (q_d - q) + Kd (qd_d - qd), the engine integrates the real state.
Stop restores the engine initial state (the pose before playback).
"""
from __future__ import annotations

import numpy as np

import core_bridge as cb

from .engine import PAUSED, RUNNING, STOPPED
from .mode import DYNAMIC, KINEMATIC, check_mode
from .trajectory import DEFAULT_KD, DEFAULT_KP, Trajectory

MIN_SPEED, MAX_SPEED = 0.1, 4.0


def resolve_gains(settings, n_dof):
    """(kp, kd) arrays from ``settings.track_kp / track_kd`` (None -> defaults)."""
    out = []
    for attr, default in (("track_kp", DEFAULT_KP), ("track_kd", DEFAULT_KD)):
        v = getattr(settings, attr, None) if settings is not None else None
        if v is None:
            arr = np.full(n_dof, default)
        else:
            arr = np.broadcast_to(np.asarray(v, float), (n_dof,)).copy()
        out.append(arr)
    return out[0], out[1]


class TrajectoryPlayer:
    def __init__(self, engine, traj: Trajectory, mode: str, kp=None, kd=None,
                 speed: float = 1.0, loop: bool = False):
        check_mode(mode)
        if traj.n_dof != engine.n_dof:
            raise ValueError("trajectory n_dof differs from the robot DOF")
        self.engine, self.traj, self.mode = engine, traj, mode
        d_kp, d_kd = resolve_gains(None, engine.n_dof)
        self.kp = d_kp if kp is None else np.broadcast_to(np.asarray(kp, float), d_kp.shape).copy()
        self.kd = d_kd if kd is None else np.broadcast_to(np.asarray(kd, float), d_kd.shape).copy()
        self.speed = speed
        self.loop = bool(loop)
        self.finished = False
        self._t = 0.0
        if mode == DYNAMIC:
            engine.torque_fn = self._torque

    # ----------------------------------------------------------- properties
    @property
    def speed(self) -> float:
        return self._speed

    @speed.setter
    def speed(self, v: float) -> None:
        v = float(v)
        if not (MIN_SPEED - 1e-12 <= v <= MAX_SPEED + 1e-12):
            raise ValueError(f"speed must be in [{MIN_SPEED}, {MAX_SPEED}]")
        self._speed = v

    @property
    def t(self) -> float:
        """Playback time in [0, duration]."""
        return float(self.engine.state.t) if self.mode == DYNAMIC else self._t

    @property
    def duration(self) -> float:
        return self.traj.duration

    # ------------------------------------------------------------- control
    def _torque(self, t, q, qd):
        e, tr = self.engine, self.traj
        qdes, vdes, ades = tr.q(t), tr.qd(t), tr.qdd(t)
        ff = np.asarray(cb.inverse_dynamics(
            e.dm, list(qdes), list(vdes), list(ades), e.gravity, e.friction), float)
        if e.armature is not None:
            ff = ff + np.asarray(e.armature, float) * ades
        return ff + self.kp * (qdes - q) + self.kd * (vdes - qd)

    def tracking_error(self) -> dict:
        """RMS and max |q_d(t) - q| over the DOFs at the current state."""
        s = self.engine.state
        e = self.traj.q(s.t) - s.q
        return {"rms": float(np.sqrt(np.mean(e * e))), "max": float(np.max(np.abs(e)))}

    def _apply(self, t: float) -> None:
        tr, e = self.traj, self.engine
        if self.mode == KINEMATIC:
            self._t = t
            e.set_state(t, tr.q(t), tr.qd(t), tr.qdd(t))
        else:
            e.set_state(t, tr.q(t), tr.qd(t))

    def seek(self, t: float):
        """Jump to ``t`` (clamped); the clock state is not changed."""
        t = min(max(float(t), 0.0), self.duration)
        self._apply(t)
        self.finished = t >= self.duration
        return self.engine.state

    def play(self) -> None:
        """STOPPED: start from the current position (restart at the end);
        PAUSED: resume; RUNNING: nothing."""
        e = self.engine
        if e.clock.state == RUNNING:
            return
        if self.finished or self.t >= self.duration:
            self.seek(0.0)
        if e.clock.state == STOPPED:
            self._apply(self.t)
            e.start()
        else:
            e.error = None
            e.resume()

    def pause(self) -> None:
        if self.engine.clock.state == RUNNING:
            self.engine.pause()

    def stop(self) -> None:
        e = self.engine
        if e.clock.state in (RUNNING, PAUSED):
            e.stop()
        self._t = 0.0
        self.finished = False

    def detach(self) -> None:
        """Remove the controller from the engine."""
        if self.engine.torque_fn == self._torque:
            self.engine.torque_fn = None

    def advance(self, wall_dt: float) -> bool:
        """Advance by ``wall_dt * speed`` s (only when RUNNING). True when the
        end was reached this call (loop restarts instead of stopping)."""
        e = self.engine
        if e.clock.state != RUNNING:
            return False
        dt = float(wall_dt) * self._speed
        T = self.duration
        if self.mode == KINEMATIC:
            t = self._t + dt
            reached = t >= T
            if reached and self.loop:
                t = t % T
                reached = False
            self._apply(min(t, T))
        else:
            left = T - self.t
            # sim time advanced = min(dt, left): speed only, the engine's time_scale multiplier
            # is divided out so kinematic and dynamic play at the same rate (audit LOW)
            e.advance(min(dt, left) / e.time_scale)
            reached = self.t >= T - 1e-9
            if reached and self.loop:
                self._apply(0.0)
                reached = False
        if reached:
            self.finished = True
            e.pause()
        return reached
