"""R4.4: multi-start IK, validated against closed form and numpy FK."""
import numpy as np

import core_bridge as cb

from app.kin_utils import dof_limits

from .closed_form import fk_3r, ik_2r, ik_3r, wrap
from .numpy_ref import fk_numpy, so3_log, spec_from_model
from .robots import pose, random_q, random_robot

N_STARTS = 60  # test value (spec default 30); see report


def planar(n, tool_x=1.0):
    m = cb.RobotModel()
    for i in range(n + 1):
        lk = cb.Link()
        lk.name = f"l{i}"
        m.links.append(lk)
    for i in range(n):
        j = cb.Joint()
        j.name = f"j{i}"
        j.parent_link, j.child_link = i, i + 1
        j.type = cb.JointType.Revolute
        j.kinematics = cb.JointKinematics.OriginAxis
        j.origin = pose([0.0 if i == 0 else 1.0, 0, 0], [0, 0, 0])
        j.axis = [0.0, 0.0, 1.0]
        lim = cb.JointLimits()
        lim.lower, lim.upper = -np.pi, np.pi
        j.limits = lim
        m.joints.append(j)
    m.tool = pose([tool_x, 0, 0], [0, 0, 0])
    m.tcp_link = n
    return m


def target_xy(x, y, phi=0.0):
    T = np.eye(4)
    c, s = np.cos(phi), np.sin(phi)
    T[:2, :2] = [[c, -s], [s, c]]
    T[0, 3], T[1, 3] = x, y
    return T


def run(model, target, q_cur, opts=None, **kw):
    cm = cb.compile(model)
    lo, hi = dof_limits(model)
    return cb.solve_ik_multi(cm, lo, hi, target, list(q_cur), opts, **kw)


def same_set(found, expected, tol):
    """Every expected is matched by one found (wrapped) and counts equal."""
    assert len(found) == len(expected), (found, expected)
    for e in expected:
        assert any(np.max(np.abs(wrap(np.array(f) - np.array(e)))) < tol for f in found), (e, found)


def test_2r_two_solutions():
    m = planar(2)
    sols = run(m, target_xy(1.2, 0.8), [0.3, 0.3], {"position_only": True}, n_starts=N_STARTS)
    expected = ik_2r(1.2, 0.8)
    assert len(expected) == 2
    same_set([s["q"] for s in sols], expected, 1e-5)


def test_2r_full_stretch_one_solution():
    m = planar(2)
    # At full stretch pos error ~ l*q2^2/2, so tol_pos 1e-6 allows |q2| ~ 1.4e-3:
    # the two mirror solutions sit within a looser dedupe tol (test value 1e-2).
    sols = run(m, target_xy(2.0, 0.0), [0.3, 0.3], {"position_only": True}, n_starts=N_STARTS,
               dedupe_tol=1e-2)
    assert len(sols) == 1
    assert np.allclose(sols[0]["q"], [0, 0], atol=1e-2)


def test_2r_unreachable_empty():
    m = planar(2)
    sols = run(m, target_xy(3.0, 0.0), [0.3, 0.3], {"position_only": True}, n_starts=N_STARTS)
    assert sols == []


def test_3r_orientation_vs_closed_form_and_numpy_fk():
    m = planar(3)
    spec = spec_from_model(m)
    rng = np.random.default_rng(5)
    for _ in range(5):
        q_true = rng.uniform(-2.0, 2.0, 3)
        x, y, phi = fk_3r(q_true)
        target = target_xy(x, y, phi)
        sols = run(m, target, q_true + 0.1, n_starts=N_STARTS)
        assert len(sols) >= 1
        cf = ik_3r(x, y, phi)
        for s in sols:
            T = fk_numpy(spec, s["q"])[1]
            assert np.linalg.norm(T[:3, 3] - target[:3, 3]) < 1e-6
            assert np.linalg.norm(so3_log(target[:3, :3] @ T[:3, :3].T)) < 1e-6
            assert any(np.max(np.abs(wrap(np.array(s["q"]) - np.array(c)))) < 1e-5 for c in cf), (s, cf)
        # the closed-form set is complete for this target
        for c in cf:
            if all(abs(v) < np.pi - 1e-3 for v in c):
                assert any(np.max(np.abs(wrap(np.array(s["q"]) - np.array(c)))) < 1e-5 for s in sols), (c, sols)


def test_6r_random_valid_sorted_deterministic():
    model = random_robot(21, 6, kinds="origin_axis")
    spec = spec_from_model(model)
    lo, hi = dof_limits(model)
    rng = np.random.default_rng(1)
    q_true = np.array(random_q(model, rng))
    target = fk_numpy(spec, q_true)[1]
    q_cur = list(np.clip(q_true + 0.3, lo, hi))
    a = run(model, target, q_cur, n_starts=30, seed=7)
    b = run(model, target, q_cur, n_starts=30, seed=7)
    assert a == b
    assert len(a) >= 1
    for s in a:
        q = np.array(s["q"])
        assert np.all(q >= np.array(lo) - 1e-9) and np.all(q <= np.array(hi) + 1e-9)
        T = fk_numpy(spec, q)[1]
        assert np.linalg.norm(T[:3, 3] - target[:3, 3]) < 1e-6
        assert np.linalg.norm(so3_log(target[:3, :3] @ T[:3, :3].T)) < 1e-6
        assert abs(s["distance"] - np.max(np.abs(q - np.array(q_cur)))) < 1e-12
    d = [s["distance"] for s in a]
    assert d == sorted(d)
    # other seed: all valid
    for s in run(model, target, q_cur, n_starts=30, seed=8):
        T = fk_numpy(spec, s["q"])[1]
        assert np.linalg.norm(T[:3, 3] - target[:3, 3]) < 1e-6


def test_one_start_equals_solve_ik():
    model = random_robot(22, 6, kinds="origin_axis")
    spec = spec_from_model(model)
    cm = cb.compile(model)
    lo, hi = dof_limits(model)
    rng = np.random.default_rng(2)
    q_true = np.array(random_q(model, rng))
    target = fk_numpy(spec, q_true)[1]
    q0 = list(np.clip(q_true + 0.2, lo, hi))
    r = cb.solve_ik(cm, lo, hi, target, q0, None)
    sols = cb.solve_ik_multi(cm, lo, hi, target, q0, None, n_starts=1)
    if r["converged"]:
        assert len(sols) == 1
        assert sols[0]["q"] == r["q"]
    else:
        assert sols == []


def test_bad_sizes_throw():
    m = planar(2)
    cm = cb.compile(m)
    lo, hi = dof_limits(m)
    import pytest
    with pytest.raises(ValueError):
        cb.solve_ik_multi(cm, lo, hi, np.eye(4), [0.0], None)
    with pytest.raises(ValueError):
        cb.solve_ik_multi(cm, lo, hi, np.eye(4), [0.0, 0.0], None, n_starts=0)
