"""Simulation state (R6.1)."""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


def _z(n):
    return np.zeros(n)


@dataclass
class SimState:
    """State of the simulated robot at time ``t``.

    q, qd, qdd, tau, tau_inv: numpy arrays, one entry per DOF.
    tcp: 4x4 pose. energy: dict with ke, pe, total.
    """

    t: float = 0.0
    q: np.ndarray = field(default_factory=lambda: _z(0))
    qd: np.ndarray = field(default_factory=lambda: _z(0))
    qdd: np.ndarray = field(default_factory=lambda: _z(0))
    tau: np.ndarray = field(default_factory=lambda: _z(0))
    tau_inv: np.ndarray = field(default_factory=lambda: _z(0))
    tcp: np.ndarray = field(default_factory=lambda: np.eye(4))
    energy: dict = field(default_factory=lambda: {"ke": 0.0, "pe": 0.0, "total": 0.0})

    @classmethod
    def zeros(cls, n_dof):
        return cls(q=_z(n_dof), qd=_z(n_dof), qdd=_z(n_dof), tau=_z(n_dof), tau_inv=_z(n_dof))

    def copy(self):
        """Deep copy."""
        return SimState(
            t=float(self.t),
            q=np.array(self.q, float, copy=True),
            qd=np.array(self.qd, float, copy=True),
            qdd=np.array(self.qdd, float, copy=True),
            tau=np.array(self.tau, float, copy=True),
            tau_inv=np.array(self.tau_inv, float, copy=True),
            tcp=np.array(self.tcp, float, copy=True),
            energy=dict(self.energy),
        )
