"""Step integrators for the simulation engine (R6.2).

Common call: ``integrator(accel, t, q, qd, dt) -> (q_new, qd_new, dt_accepted)``
with ``accel(t, q, qd) -> qdd`` (numpy). Fixed-step methods return ``dt``.
``RK45`` is adaptive: it may accept a smaller step than asked and exposes the
next suggested step as ``next_dt`` (``None`` until the first step).
"""
from __future__ import annotations

import numpy as np

INTEGRATOR_NAMES = ("euler", "rk4", "rk45")


def euler_step(accel, t, q, qd, dt):
    """Semi-implicit Euler: qd += dt*qdd, then q += dt*qd."""
    qd_new = qd + dt * accel(t, q, qd)
    q_new = q + dt * qd_new
    return q_new, qd_new, dt


def rk4_step(accel, t, q, qd, dt):
    """Classic RK4 on y = [q, qd]."""
    h = 0.5 * dt
    k1v = accel(t, q, qd)
    k2q = qd + h * k1v
    k2v = accel(t + h, q + h * qd, k2q)
    k3q = qd + h * k2v
    k3v = accel(t + h, q + h * k2q, k3q)
    k4q = qd + dt * k3v
    k4v = accel(t + dt, q + dt * k3q, k4q)
    q_new = q + dt / 6.0 * (qd + 2 * k2q + 2 * k3q + k4q)
    qd_new = qd + dt / 6.0 * (k1v + 2 * k2v + 2 * k3v + k4v)
    return q_new, qd_new, dt


# Dormand-Prince 5(4) tableau
_C = (0.0, 1 / 5, 3 / 10, 4 / 5, 8 / 9, 1.0, 1.0)
_A = (
    (),
    (1 / 5,),
    (3 / 40, 9 / 40),
    (44 / 45, -56 / 15, 32 / 9),
    (19372 / 6561, -25360 / 2187, 64448 / 6561, -212 / 729),
    (9017 / 3168, -355 / 33, 46732 / 5247, 49 / 176, -5103 / 18656),
    (35 / 384, 0.0, 500 / 1113, 125 / 192, -2187 / 6784, 11 / 84),
)
_B5 = _A[6]
_E = (71 / 57600, 0.0, -71 / 16695, 71 / 1920, -17253 / 339200, 22 / 525, -1 / 40)


class RK45:
    """Dormand-Prince adaptive integrator with step-size control."""

    def __init__(self, atol=1e-9, rtol=1e-7, dt_min=1e-7, dt_max=0.01,
                 safety=0.9, min_factor=0.2, max_factor=5.0):
        self.atol, self.rtol = float(atol), float(rtol)
        self.dt_min, self.dt_max = float(dt_min), float(dt_max)
        self.safety, self.min_factor, self.max_factor = safety, min_factor, max_factor
        self.next_dt = None
        self.n_rejected = 0

    def __call__(self, accel, t, q, qd, dt):
        dt = min(float(dt), self.dt_max)
        n = len(q)
        y = np.concatenate([q, qd])
        while True:
            if dt < self.dt_min:
                raise RuntimeError("step size underflow")
            ks = []
            for i in range(7):
                yi = y
                if i:
                    yi = y + dt * sum(a * k for a, k in zip(_A[i], ks))
                qdi = yi[n:]
                ks.append(np.concatenate([qdi, accel(t + _C[i] * dt, yi[:n], qdi)]))
            y_new = y + dt * sum(b * k for b, k in zip(_B5, ks))
            err_v = dt * sum(e * k for e, k in zip(_E, ks))
            sc = self.atol + self.rtol * np.maximum(np.abs(y), np.abs(y_new))
            err = float(np.sqrt(np.mean((err_v / sc) ** 2)))
            if err != err:
                err = float("inf")
            factor = self.max_factor if err == 0.0 else self.safety * err ** -0.2
            factor = min(self.max_factor, max(self.min_factor, factor))
            if err <= 1.0:
                self.next_dt = min(dt * factor, self.dt_max)
                return y_new[:n], y_new[n:], dt
            self.n_rejected += 1
            dt *= min(factor, 1.0)


def make_integrator(name, **rk45_kw):
    """Fresh integrator callable by name. Unknown name -> ValueError."""
    if name == "euler":
        return euler_step
    if name == "rk4":
        return rk4_step
    if name == "rk45":
        return RK45(**rk45_kw)
    raise ValueError(f"unknown integrator {name!r}")
