"""R7.1: synchronised joint profiles vs closed forms."""
import math

import numpy as np
import pytest

from app.planning import quintic, scurve, trapezoid
from app.sim.trajectory import Trajectory

Q0 = np.array([0.0, 1.0, -0.5, 0.3])
Q1 = np.array([1.2, 1.0, 0.7, -0.8])          # dof 1 does not move
VMAX = np.array([1.0, 1.0, 0.5, 2.0])
AMAX = np.array([2.0, 2.0, 1.0, 4.0])
JMAX = np.array([10.0, 10.0, 5.0, 30.0])


def _scan(p, n=4001):
    ts = np.linspace(0.0, p.duration, n)
    q = np.array([p.q(t) for t in ts])
    qd = np.array([p.qd(t) for t in ts])
    qdd = np.array([p.qdd(t) for t in ts])
    return ts, q, qd, qdd


def _bounds(p, vmax=None, amax=None, jmax=None):
    ts, q, qd, qdd = _scan(p)
    if vmax is not None:
        assert np.all(np.abs(qd) <= vmax + 1e-9)
    if amax is not None:
        assert np.all(np.abs(qdd) <= amax + 1e-9)
    if jmax is not None:
        jerk = np.diff(qdd, axis=0) / np.diff(ts)[:, None]
        assert np.all(np.abs(jerk) <= jmax + 1e-6)
    return ts, q, qd, qdd


def _ends(p):
    assert np.allclose(p.q(0.0), Q0, atol=1e-12)
    assert np.allclose(p.q(p.duration), Q1, atol=1e-12)
    for t in (0.0, p.duration, p.duration + 1.0, -1.0):
        assert np.allclose(p.qd(t), 0.0, atol=1e-12)
        assert np.allclose(p.qdd(t), 0.0, atol=1e-12)


def _trap_T(d, v, a):
    return d / v + v / a if d >= v * v / a else 2 * math.sqrt(d / a)


def test_trapezoid_boundaries_duration_limits():
    p = trapezoid(Q0, Q1, VMAX, AMAX)
    _ends(p)
    d = np.abs(Q1 - Q0)
    T_ref = max(_trap_T(d[i], VMAX[i], AMAX[i]) for i in (0, 2, 3))
    assert abs(p.duration - T_ref) < 1e-12
    ts, q, qd, qdd = _bounds(p, VMAX, AMAX)
    # slowest dof (here dof 2: d=1.2, v=.5) reaches its limit
    assert abs(np.max(np.abs(qd[:, 2])) - VMAX[2]) < 1e-3
    # all dof end together: still at rest and q1 exactly at T; unmoved dof constant
    assert np.allclose(q[:, 1], Q0[1], atol=1e-14)
    # monotonic path along each dof (rest-to-rest)
    assert np.all(np.diff(q[:, 0]) >= -1e-12)


def test_trapezoid_triangular_and_cruise_closed_form():
    for dist, v, a in ((1.0, 10.0, 2.0), (4.0, 1.0, 2.0)):
        p = trapezoid([0.0], [dist], [v], [a])
        assert abs(p.duration - _trap_T(dist, v, a)) < 1e-12
        # midpoint closed form: q(T/2) = dist/2 (symmetric)
        assert abs(p.q(p.duration / 2)[0] - dist / 2) < 1e-12
        vp = min(v, math.sqrt(dist * a))
        assert abs(p.qd(p.duration / 2)[0] - vp) < 1e-12


def test_trapezoid_binding_dof_is_slowest():
    p = trapezoid([0, 0], [1.0, 3.0], [1.0, 1.0], [10.0, 10.0])
    # dof 1 (d=3) binds: its own closed-form T, dof 0 scaled
    assert abs(p.duration - _trap_T(3.0, 1.0, 10.0)) < 1e-12
    ts, q, qd, qdd = _scan(p)
    assert abs(np.max(np.abs(qd[:, 1])) - 1.0) < 1e-9
    assert abs(np.max(np.abs(qd[:, 0])) - 1.0 / 3.0) < 1e-9


def test_quintic_closed_form():
    T = 2.5
    p = quintic(Q0, Q1, T)
    _ends(p)
    assert p.duration == T
    d = Q1 - Q0
    for t in np.linspace(0, T, 37):
        u = t / T
        s = 10 * u**3 - 15 * u**4 + 6 * u**5
        sd = (30 * u**2 - 60 * u**3 + 30 * u**4) / T
        sdd = (60 * u - 180 * u**2 + 120 * u**3) / T**2
        assert np.allclose(p.q(t), Q0 + d * s, atol=1e-12)
        assert np.allclose(p.qd(t), d * sd, atol=1e-12)
        assert np.allclose(p.qdd(t), d * sdd, atol=1e-12)
    # peak velocity 15/8 d/T at the middle
    assert np.allclose(p.qd(T / 2), 15 / 8 * d / T, atol=1e-12)


def test_scurve_boundaries_limits_and_phases():
    p = scurve(Q0, Q1, VMAX, AMAX, JMAX)
    _ends(p)
    ts, q, qd, qdd = _bounds(p, VMAX, AMAX, JMAX)
    assert np.allclose(q[-1], Q1, atol=1e-12)
    # velocity integrates to the distance (independent trapezoid-rule check)
    dist = np.trapz(qd, ts, axis=0) if hasattr(np, "trapz") else np.trapezoid(qd, ts, axis=0)
    assert np.allclose(dist, Q1 - Q0, atol=1e-5)


@pytest.mark.parametrize("dist,v,a,j", [
    (10.0, 1.0, 2.0, 10.0),      # cruise, accel limit reached
    (10.0, 1.0, 100.0, 10.0),    # cruise, accel limit not reached
    (0.5, 1.0, 2.0, 10.0),       # no cruise, accel limit reached
    (0.05, 5.0, 20.0, 10.0),     # no cruise, accel limit not reached
])
def test_scurve_closed_form_duration(dist, v, a, j):
    p = scurve([0.0], [dist], [v], [a], [j])
    # independent closed form
    if v * j >= a * a:
        tj, ta = a / j, v / a - a / j
    else:
        tj, ta = math.sqrt(v / j), 0.0
    if dist >= v * (2 * tj + ta):
        T = dist / v + 2 * tj + ta
    else:
        tj0 = (dist / (2 * j)) ** (1 / 3)
        if j * tj0 <= a:
            T = 4 * tj0
        else:
            tj = a / j
            ta = (-3 * tj + math.sqrt(tj * tj + 4 * dist / a)) / 2
            T = 4 * tj + 2 * ta
    assert abs(p.duration - T) < 1e-9, (p.duration, T)
    _ends_1d(p, dist)
    _bounds(p, v, a, j)


def _ends_1d(p, dist):
    assert abs(p.q(p.duration)[0] - dist) < 1e-12
    assert abs(p.qd(p.duration)[0]) < 1e-12 and abs(p.qdd(p.duration)[0]) < 1e-12
    # end-of-integration state (not just clamping): sample just before T
    assert abs(p.qd(p.duration * (1 - 1e-9))[0]) < 1e-6


def test_zero_motion_and_errors():
    p = trapezoid(Q0, Q0, VMAX, AMAX)
    assert p.duration == 0.0 and np.allclose(p.q(0.3), Q0)
    with pytest.raises(ValueError):
        p.to_trajectory(0.1)
    with pytest.raises(ValueError):
        trapezoid(Q0, Q1, 0.0, AMAX)
    with pytest.raises(ValueError):
        scurve(Q0, Q1, VMAX, AMAX, -1.0)
    with pytest.raises(ValueError):
        quintic(Q0, Q1, 0.0)


@pytest.mark.parametrize("interp", ["linear", "cubic"])
def test_to_trajectory(interp):
    p = scurve(Q0, Q1, VMAX, AMAX, JMAX)
    tr = p.to_trajectory(0.01, interp)
    assert isinstance(tr, Trajectory)
    assert tr.waypoints[0].t == 0.0
    assert abs(tr.duration - p.duration) < 1e-12
    assert np.allclose(tr.q(0.0), Q0) and np.allclose(tr.q(tr.duration), Q1, atol=1e-12)
    mid = p.duration * 0.37
    tol = 5e-3 if interp == "linear" else 5e-4
    assert np.allclose(tr.q(mid), p.q(mid), atol=tol)
