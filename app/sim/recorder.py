"""Signal recording with bounded memory (R6.5). No Qt dependency.

``RingBuffer``: preallocated (capacity, width) data + time arrays.
``Recorder``: hooks the engine step, computes the SIGNALS row of each (decimated)
sample and stores it in one ring buffer. Memory cap = ``record_memory_mb``.
"""
from __future__ import annotations

import numpy as np

import core_bridge as cb

from .engine import RUNNING, STOPPED

BYTES_PER_VALUE = 8
MEM_MB_MIN, MEM_MB_MAX, MEM_MB_DEFAULT = 1, 2048, 64

# name, width ("n" = one per DOF), unit, source (how it is computed)
SIGNALS = (
    ("q", "n", "rad|m", "state.q"),
    ("qd", "n", "rad/s|m/s", "state.qd"),
    ("qdd", "n", "rad/s^2|m/s^2", "state.qdd"),
    ("tau", "n", "N.m|N", "state.tau (applied) or RNEA(q, qd, qdd) if estimate_torque"),
    ("tcp_pos", 3, "m", "cb.fk(model, q)['tcp'] translation"),
    ("tcp_quat", 4, "1 (w,x,y,z)", "rotation of the TCP pose, sign-continuous"),
    ("tcp_vel", 3, "m/s", "(cb.jacobian_tcp(cm, q) @ qd)[:3]"),
    ("ke", 1, "J", "cb.kinetic_energy"),
    ("pe", 1, "J", "cb.potential_energy"),
    ("total", 1, "J", "ke + pe"),
    ("power", "n", "W", "cb.joint_power(tau, qd)"),
    ("power_sum", 1, "W", "sum of power"),
    ("sigma_min", 1, "mixed", "cb.analyze_singularity(J)['sigma_min']"),
    ("manipulability", 1, "mixed", "cb.analyze_singularity(J)['manipulability']"),
)


def signal_layout(n_dof):
    """``([(name, width, unit, source, col_start)], total_width)``."""
    out, col = [], 0
    for name, w, unit, src in SIGNALS:
        w = n_dof if w == "n" else int(w)
        out.append((name, w, unit, src, col))
        col += w
    return out, col


def capacity_for(memory_mb, width):
    """Rows that fit in ``memory_mb`` MiB: one row = ``width`` values + one time value."""
    return int(float(memory_mb) * 1024 * 1024) // (BYTES_PER_VALUE * (int(width) + 1))


class RingBuffer:
    def __init__(self, capacity, width):
        capacity, width = int(capacity), int(width)
        if capacity < 1 or width < 1:
            raise ValueError("capacity and width must be >= 1")
        self.capacity, self.width = capacity, width
        self._t = np.zeros(capacity)
        self._d = np.zeros((capacity, width))
        self._head = 0       # next write index
        self._n = 0
        self.dropped = 0

    def __len__(self):
        return self._n

    @property
    def nbytes(self):
        return self._t.nbytes + self._d.nbytes

    def push(self, t, row):
        row = np.asarray(row, float)
        if row.shape != (self.width,):
            raise ValueError(f"row needs width {self.width}")
        i = self._head
        self._t[i] = t
        self._d[i] = row
        self._head = (i + 1) % self.capacity
        if self._n < self.capacity:
            self._n += 1
        else:
            self.dropped += 1

    def view(self):
        """``(t, data)`` in time order. A view when not wrapped, a copy when wrapped."""
        if self._n < self.capacity:
            return self._t[:self._n], self._d[:self._n]
        h = self._head
        if h == 0:
            return self._t, self._d
        return (np.concatenate((self._t[h:], self._t[:h])),
                np.concatenate((self._d[h:], self._d[:h])))

    def clear(self):
        self._head = 0
        self._n = 0
        self.dropped = 0


def _quat_wxyz(R):
    """Unit quaternion (w, x, y, z) of a rotation matrix (Shepperd)."""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = 2.0 * np.sqrt(tr + 1.0)
        q = [0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s]
    elif R[0, 0] >= R[1, 1] and R[0, 0] >= R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        q = [(R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s]
    elif R[1, 1] >= R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        q = [(R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s]
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        q = [(R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s]
    q = np.asarray(q, float)
    return q / np.linalg.norm(q)


class Recorder:
    """Records SIGNALS while the engine clock is RUNNING (kinematic playback included).

    Settings (``SimulationSettings`` ``record_memory_mb``, ``record_every``,
    ``record_overwrite``) are read from ``engine.settings`` unless overridden.
    Cleared on engine reset / stop. ``overwrite=False``: recording stops when full
    (``stopped_full``).
    """

    def __init__(self, engine, model=None, memory_mb=None, record_every=None,
                 overwrite=None, estimate_torque=False):
        st = getattr(engine, "settings", None)
        self.engine = engine
        self.model = model if model is not None else engine.model
        mb = memory_mb if memory_mb is not None else getattr(st, "record_memory_mb", MEM_MB_DEFAULT)
        ev = record_every if record_every is not None else getattr(st, "record_every", 1)
        ow = overwrite if overwrite is not None else getattr(st, "record_overwrite", True)
        if not (MEM_MB_MIN <= float(mb) <= MEM_MB_MAX):
            raise ValueError(f"record_memory_mb must be in [{MEM_MB_MIN}, {MEM_MB_MAX}]")
        if int(ev) < 1:
            raise ValueError("record_every must be >= 1")
        self.memory_mb, self.record_every = mb, int(ev)
        self.overwrite = bool(ow)
        self.estimate_torque = bool(estimate_torque)
        self.n_dof = int(engine.n_dof)
        self.layout, self.width = signal_layout(self.n_dof)
        cap = capacity_for(self.memory_mb, self.width)
        if cap < 1:
            raise ValueError("memory cap too small for one sample")
        self.buffer = RingBuffer(cap, self.width)
        self.stopped_full = False
        self.enabled = True
        self._count = 0
        self._prev_quat = None
        self.attach()

    # ------------------------------------------------------------ wiring
    def attach(self):
        e = self.engine
        if self._on_step not in e.step_callbacks:
            e.step_callbacks.append(self._on_step)
            e.reset_callbacks.append(self.clear)
            e.clock.on_state_changed.append(self._on_clock)

    def detach(self):
        e = self.engine
        for lst, fn in ((e.step_callbacks, self._on_step), (e.reset_callbacks, self.clear),
                        (e.clock.on_state_changed, self._on_clock)):
            if fn in lst:
                lst.remove(fn)

    def _on_clock(self, new, _old):
        if new == STOPPED:
            self.clear()

    def _on_step(self, state):
        if self.enabled and self.engine.clock.state == RUNNING:
            self.sample(state)

    # --------------------------------------------------------- recording
    @property
    def capacity(self):
        return self.buffer.capacity

    @property
    def dropped(self):
        return self.buffer.dropped

    def __len__(self):
        return len(self.buffer)

    def clear(self):
        self.buffer.clear()
        self.stopped_full = False
        self._count = 0
        self._prev_quat = None

    def sample(self, s):
        """Count one step; record it when it falls on the decimation grid."""
        k = self._count
        self._count += 1
        if k % self.record_every:
            return False
        if self.stopped_full or (not self.overwrite and len(self.buffer) >= self.buffer.capacity):
            self.stopped_full = True
            return False
        self.buffer.push(s.t, self._row(s))
        return True

    def _row(self, s):
        e = self.engine
        q, qd, qdd = (np.asarray(v, float) for v in (s.q, s.qd, s.qdd))
        if self.estimate_torque:
            tau = np.asarray(cb.inverse_dynamics(e.dm, list(q), list(qd), list(qdd),
                                                 e.gravity, e.friction), float)
        else:
            tau = np.asarray(s.tau, float)
        T = np.asarray(cb.fk(self.model, list(q))["tcp"], float)
        quat = _quat_wxyz(T[:3, :3])
        if self._prev_quat is not None and float(quat @ self._prev_quat) < 0.0:
            quat = -quat
        self._prev_quat = quat
        J = np.asarray(cb.jacobian_tcp(e.cm, list(q)), float)
        sing = cb.analyze_singularity(J)
        ke = float(cb.kinetic_energy(e.dm, list(q), list(qd)))
        pe = float(cb.potential_energy(e.dm, list(q), e.gravity))
        power = np.asarray(cb.joint_power(list(tau), list(qd)), float)
        return np.concatenate((q, qd, qdd, tau, T[:3, 3], quat, (J @ qd)[:3],
                               [ke, pe, ke + pe], power, [power.sum()],
                               [float(sing["sigma_min"]), float(sing["manipulability"])]))

    # ------------------------------------------------------------ output
    def export_arrays(self):
        """``{name: (t, data)}``: ``t`` shape (N,), ``data`` shape (N, width), time order."""
        t, d = self.buffer.view()
        return {name: (t.copy(), d[:, c:c + w].copy()) for name, w, _u, _s, c in self.layout}

    def status(self):
        txt = f"rec {len(self)}/{self.capacity}"
        if self.stopped_full:
            txt += " FULL (recording stopped)"
        elif self.dropped:
            txt += f" (dropped {self.dropped})"
        return txt
