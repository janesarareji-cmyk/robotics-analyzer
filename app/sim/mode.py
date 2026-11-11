"""Simulation modes and their rules (R6.3). No Qt dependency.

KINEMATIC: the user sets q (sliders); qd = qdd = 0; the clock never runs.
DYNAMIC: state evolves by forward dynamics from the applied torque.
"""
from __future__ import annotations

import numpy as np

import core_bridge as cb

from .engine import PAUSED, RUNNING, STOPPED

KINEMATIC = "kinematic"
DYNAMIC = "dynamic"
MODES = (KINEMATIC, DYNAMIC)


def check_mode(mode):
    if mode not in MODES:
        raise ValueError(f"unknown mode {mode!r}")
    return mode


def switch_mode(engine, new_mode):
    """Mode switch: q is kept, qd = 0 (both directions), clock stopped.

    Switching to DYNAMIC sets qd = 0; switching to KINEMATIC drops qd.
    """
    check_mode(new_mode)
    q = engine.state.q.copy()
    engine.reset(q)  # reset() stops a running clock and zeroes qd
    if new_mode == KINEMATIC:
        _make_static(engine)
    return engine.state


def set_kinematic_q(engine, q):
    """Kinematic edit: state q = q, qd = qdd = 0 (tcp, tau_inv, energy refreshed)."""
    engine.reset(np.asarray(q, float))
    _make_static(engine)
    return engine.state


def _make_static(engine):
    """qdd = 0 and tau_inv = RNEA(q, 0, 0) (static holding torque)."""
    s = engine.state
    s.qd[:] = 0.0
    s.qdd[:] = 0.0
    s.tau_inv = np.asarray(cb.inverse_dynamics(
        engine.dm, list(s.q), list(s.qd), list(s.qdd), engine.gravity, engine.friction), float)


def button_states(mode, clock_state, has_engine=True):
    """Which run buttons are enabled. Kinematic mode has no run clock."""
    off = dict(start=False, pause=False, resume=False, stop=False)
    if not has_engine or mode != DYNAMIC:
        return off
    return dict(
        start=clock_state == STOPPED,
        pause=clock_state == RUNNING,
        resume=clock_state == PAUSED,
        stop=clock_state in (RUNNING, PAUSED),
    )


def finite_difference_torque(engine, samples):
    """Estimate tau_inv from the last three (t, q) samples (kinematic mode).

    qd and qdd come from backward finite differences; returns the RNEA
    torque, or None if fewer than 3 samples or a time gap is too small.
    """
    if len(samples) < 3:
        return None
    (t0, q0), (t1, q1), (t2, q2) = samples[-3:]
    h1, h2 = t1 - t0, t2 - t1
    if h1 < 1e-6 or h2 < 1e-6:
        return None
    q0, q1, q2 = (np.asarray(x, float) for x in (q0, q1, q2))
    v1, v2 = (q1 - q0) / h1, (q2 - q1) / h2
    qdd = 2.0 * (v2 - v1) / (h1 + h2)
    return np.asarray(cb.inverse_dynamics(
        engine.dm, list(q2), list(v2), list(qdd), engine.gravity, engine.friction), float)
