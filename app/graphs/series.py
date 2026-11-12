"""Pure graph data from recorder arrays (R6.6). No Qt dependency.

``graph_data(arrays, kind, joints=None, names=None, max_points=5000)`` ->
``list[Series]``. ``arrays`` is ``Recorder.export_arrays()`` (``{name: (t, data)}``).
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

MAX_POINTS = 5000

KINDS = ("joint_pos", "joint_vel", "joint_acc", "joint_torque", "tcp_xyz",
         "tcp_orientation", "tcp_path_3d", "energy", "manipulability", "power")
JOINT_KINDS = ("joint_pos", "joint_vel", "joint_acc", "joint_torque", "power")
KIND_LABELS = {
    "joint_pos": "Joint position", "joint_vel": "Joint velocity",
    "joint_acc": "Joint acceleration", "joint_torque": "Joint torque",
    "tcp_xyz": "TCP position", "tcp_orientation": "TCP orientation",
    "tcp_path_3d": "TCP path 3D", "energy": "Energy",
    "manipulability": "Manipulability", "power": "Power",
}
_JOINT_SIGNAL = {"joint_pos": "q", "joint_vel": "qd", "joint_acc": "qdd",
                 "joint_torque": "tau", "power": "power"}
_JOINT_UNITS = {  # (revolute, prismatic) share one unit string in SIGNALS: split on "|"
    "q": "rad|m", "qd": "rad/s|m/s", "qdd": "rad/s^2|m/s^2", "tau": "N.m|N", "power": "W",
}


@dataclass
class Series:
    name: str
    unit: str
    t: np.ndarray
    y: np.ndarray


def decimate_minmax(t, y, max_points=MAX_POINTS):
    """Min/max bucket decimation: at most ``max_points`` points, peaks kept.

    Each bucket keeps its argmin and argmax (time order). Global min and max survive.
    """
    t, y = np.asarray(t), np.asarray(y)
    n = len(y)
    if n <= max_points:
        return t, y
    nb = max(1, max_points // 2)
    edges = np.linspace(0, n, nb + 1).astype(int)
    idx = []
    for a, b in zip(edges[:-1], edges[1:]):
        if b <= a:
            continue
        seg = y[a:b]
        i, j = a + int(np.argmin(seg)), a + int(np.argmax(seg))
        idx.extend((i, j) if i < j else (j, i) if j < i else (i,))
    idx = np.asarray(idx, int)
    return t[idx], y[idx]


def _stride(t, cols, max_points=MAX_POINTS):
    """Same indices for several columns (3D path): uniform stride, last point kept."""
    n = len(t)
    if n <= max_points:
        return t, cols
    idx = np.unique(np.append(np.linspace(0, n - 1, max_points - 1).astype(int), n - 1))
    return t[idx], [c[idx] for c in cols]


def _rpy_from_quat(q):
    """Roll-pitch-yaw (R = Rz(yaw) Ry(pitch) Rx(roll)) of unit quaternions (N,4) wxyz."""
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1.0, 1.0))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def _joint_unit(sig, joint_type=None):
    """Unit of a per-joint signal. ``joint_type`` (str starting with "p" = prismatic) picks the
    revolute or prismatic half; None keeps the combined "rad|m" form (old call signature)."""
    u = _JOINT_UNITS[sig]
    if joint_type is None or "|" not in u:
        return u
    a, b = u.split("|", 1)
    return b if str(joint_type).lower().startswith("p") else a


def graph_data(arrays, kind, joints=None, names=None, max_points=MAX_POINTS, joint_types=None):
    """Series for one graph ``kind``.

    ``joints``: DOF indices for per-joint kinds (None = all). ``names``: DOF names.
    ``joint_types``: per DOF "revolute" / "prismatic" (None = unknown: combined "rad|m" units);
    prismatic columns then show m, m/s, m/s^2, N. ``power`` adds a "sum" series. Empty recording
    -> empty series (length 0).
    """
    if kind not in KINDS:
        raise ValueError(f"unknown graph kind {kind!r}")

    def ser(name, unit, t, y, dec=True):
        if dec:
            t, y = decimate_minmax(t, y, max_points)
        return Series(name, unit, np.asarray(t), np.asarray(y))

    if kind in _JOINT_SIGNAL:
        sig = _JOINT_SIGNAL[kind]
        t, d = arrays[sig]
        n = d.shape[1]
        sel = list(range(n)) if joints is None else [int(j) for j in joints]
        out = []
        for j in sel:
            if not 0 <= j < n:
                raise ValueError(f"joint {j} out of range")
            nm = names[j] if names is not None and j < len(names) else f"dof {j}"
            jt = joint_types[j] if joint_types is not None and j < len(joint_types) else None
            out.append(ser(nm, _joint_unit(sig, jt), t, d[:, j]))
        if kind == "power":
            ts, ps = arrays["power_sum"]
            out.append(ser("sum", "W", ts, ps[:, 0]))
        return out
    if kind == "tcp_xyz":
        t, d = arrays["tcp_pos"]
        return [ser(n, "m", t, d[:, i]) for i, n in enumerate("xyz")]
    if kind == "tcp_orientation":
        t, d = arrays["tcp_quat"]
        out = [ser("q" + n, "1", t, d[:, i]) for i, n in enumerate("wxyz")]
        r, p, y = _rpy_from_quat(d) if len(d) else (d[:, 0],) * 3
        out += [ser(n, "rad", t, v) for n, v in (("roll", r), ("pitch", p), ("yaw", y))]
        return out
    if kind == "tcp_path_3d":
        t, d = arrays["tcp_pos"]
        ts, cols = _stride(t, [d[:, 0], d[:, 1], d[:, 2]], max_points)
        return [Series(n, "m", ts, c) for n, c in zip("xyz", cols)]
    if kind == "energy":
        return [ser(n, "J", *arrays[n][:1], arrays[n][1][:, 0]) for n in ("ke", "pe", "total")]
    if kind == "manipulability":
        return [ser("sigma_min", "mixed", arrays["sigma_min"][0], arrays["sigma_min"][1][:, 0]),
                ser("manipulability", "mixed", arrays["manipulability"][0],
                    arrays["manipulability"][1][:, 0])]
    raise AssertionError(kind)  # pragma: no cover
