"""Dynamics helpers for UI and tests (R4.8).

``check_torque_limits`` compares tau with the effort limit of each DOF.
``integrate_rk4_forward_dynamics`` is a STUB integrator for tests and the
energy check. It is NOT a simulation engine. The real one is R6.
"""
import math

import numpy as np

import core_bridge as cb


def check_torque_limits(model, tau):
    """Compare actuator torque with ``joint.limits.effort`` per DOF.

    Limit = effort of the DOF's master joint (``model.q_index_map()``).
    Mimic slaves are not listed. A limit <= 0 gives ``no_limit`` True,
    ``exceeded`` False and ``ratio`` inf (tau != 0) or 0 (tau == 0).
    Returns a list of dicts with keys ``dof, joint_name, tau, limit,
    ratio, exceeded, no_limit``.
    """
    q_map = model.q_index_map()
    n = model.dof()
    if len(tau) != n:
        raise ValueError("tau needs one entry per DOF")
    master = [-1] * n
    for j_idx, q_idx in enumerate(q_map):
        if q_idx >= 0:
            master[q_idx] = j_idx
    out = []
    for dof in range(n):
        j_idx = master[dof]
        joint = model.joints[j_idx]
        t = float(tau[dof])
        limit = float(joint.limits.effort)
        if limit <= 0.0:
            ratio = math.inf if t != 0.0 else 0.0
            exceeded = False
            no_limit = True
        else:
            ratio = abs(t) / limit
            exceeded = ratio > 1.0
            no_limit = False
        out.append({
            "dof": dof,
            "joint_name": joint.name,
            "tau": t,
            "limit": limit,
            "ratio": ratio,
            "exceeded": exceeded,
            "no_limit": no_limit,
        })
    return out


def integrate_rk4_forward_dynamics(dm, q0, qd0, tau_fn, dt, steps, gravity=(0.0, 0.0, -9.81)):
    """STUB fixed-step RK4 on ``cb.forward_dynamics``. Not a simulation engine.

    ``tau_fn(t, q, qd)`` returns the applied torque per DOF (list). Viscous
    friction can be added there as ``-b * qd``. Returns ``(q_hist, qd_hist)``,
    numpy arrays of shape ``(steps + 1, n_dof)``. No friction argument is used.
    """
    n = dm.n_dof
    q = np.asarray(q0, float).copy()
    qd = np.asarray(qd0, float).copy()
    if q.size != n or qd.size != n:
        raise ValueError("q0 and qd0 need n_dof entries")
    g = tuple(float(v) for v in gravity)

    def accel(t, qq, qd_state):
        tau = list(map(float, tau_fn(t, qq, qd_state)))
        return np.asarray(cb.forward_dynamics(dm, list(qq), list(qd_state), tau, g), float)

    q_hist = np.zeros((steps + 1, n))
    qd_hist = np.zeros((steps + 1, n))
    q_hist[0], qd_hist[0] = q, qd
    t = 0.0
    for s in range(steps):
        k1q = qd
        k1v = accel(t, q, qd)
        k2q = qd + 0.5 * dt * k1v
        k2v = accel(t + 0.5 * dt, q + 0.5 * dt * k1q, k2q)
        k3q = qd + 0.5 * dt * k2v
        k3v = accel(t + 0.5 * dt, q + 0.5 * dt * k2q, k3q)
        k4q = qd + dt * k3v
        k4v = accel(t + dt, q + dt * k3q, k4q)
        q = q + dt / 6.0 * (k1q + 2 * k2q + 2 * k3q + k4q)
        qd = qd + dt / 6.0 * (k1v + 2 * k2v + 2 * k3v + k4v)
        t += dt
        q_hist[s + 1], qd_hist[s + 1] = q, qd
    return q_hist, qd_hist
