"""Closed-form planar IK (TEST ONLY, pure numpy, no core_bridge). R4.4."""
import numpy as np


def wrap(a):
    """Wrap angle(s) to (-pi, pi]."""
    r = np.pi - np.mod(np.pi - np.asarray(a, dtype=float), 2 * np.pi)
    return r


def ik_2r(x, y, l1=1.0, l2=1.0):
    """Planar 2R analytic solutions for tip (x, y): list of (q1, q2).

    Elbow-down (q2 >= 0) and elbow-up (q2 <= 0); one entry when q2 == 0
    (full stretch / fold); empty when unreachable.
    """
    r2 = x * x + y * y
    c2 = (r2 - l1 * l1 - l2 * l2) / (2 * l1 * l2)
    if c2 > 1 + 1e-12 or c2 < -1 - 1e-12:
        return []
    c2 = min(1.0, max(-1.0, c2))
    s2 = np.sqrt(max(0.0, 1 - c2 * c2))
    out = []
    for sgn in ((1.0,) if s2 < 1e-9 else (1.0, -1.0)):
        q2 = np.arctan2(sgn * s2, c2)
        q1 = np.arctan2(y, x) - np.arctan2(l2 * np.sin(q2), l1 + l2 * np.cos(q2))
        out.append((float(wrap(q1)), float(wrap(q2))))
    return out


def ik_3r(x, y, phi, l1=1.0, l2=1.0, l3=1.0):
    """Planar 3R analytic solutions for tip (x, y) and orientation phi."""
    wx = x - l3 * np.cos(phi)
    wy = y - l3 * np.sin(phi)
    out = []
    for q1, q2 in ik_2r(wx, wy, l1, l2):
        out.append((q1, q2, float(wrap(phi - q1 - q2))))
    return out


def fk_2r(q, l1=1.0, l2=1.0):
    return (l1 * np.cos(q[0]) + l2 * np.cos(q[0] + q[1]),
            l1 * np.sin(q[0]) + l2 * np.sin(q[0] + q[1]))


def fk_3r(q, l1=1.0, l2=1.0, l3=1.0):
    x, y = fk_2r(q, l1, l2)
    s = q[0] + q[1] + q[2]
    return x + l3 * np.cos(s), y + l3 * np.sin(s), s
