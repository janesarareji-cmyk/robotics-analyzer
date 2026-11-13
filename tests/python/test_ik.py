"""R4.3: damped least squares IK (checked with the independent numpy FK)."""
import numpy as np
import pytest

import core_bridge as cb

from app.kin_utils import dof_limits

from .numpy_ref import fk_numpy, so3_log, spec_from_model
from .robots import pose, random_q, random_robot

N_CASES = 100
TOL = 1e-6


def planar_3r():
    m = cb.RobotModel()
    for i in range(4):
        lk = cb.Link()
        lk.name = f"l{i}"
        m.links.append(lk)
    for i in range(3):
        j = cb.Joint()
        j.name = f"j{i}"
        j.parent_link, j.child_link = i, i + 1
        j.type = cb.JointType.Revolute
        j.kinematics = cb.JointKinematics.OriginAxis
        j.origin = pose([0.0 if i == 0 else 0.4, 0, 0], [0, 0, 0])
        j.axis = [0.0, 0.0, 1.0]
        lim = cb.JointLimits()
        lim.lower, lim.upper = -3.0, 3.0
        j.limits = lim
        m.joints.append(j)
    m.tool = pose([0.3, 0, 0], [0, 0, 0])
    m.tcp_link = 3
    return m


ROBOTS = {
    "dh6": lambda: random_robot(11, 6, kinds="dh"),
    "oa6": lambda: random_robot(12, 6, kinds="origin_axis"),
    "planar3": planar_3r,
    "branched": lambda: random_robot(13, 6, kinds="origin_axis", branch=True),
}


def _err(spec, q, target):
    T = fk_numpy(spec, q)[1]
    ep = np.linalg.norm(target[:3, 3] - T[:3, 3])
    er = np.linalg.norm(so3_log(target[:3, :3] @ T[:3, :3].T))
    return ep, er


def _target(model, spec, q):
    return fk_numpy(spec, q)[1]


def _run(name, seed=0):
    model = ROBOTS[name]()
    assert model.validate() == []
    spec = spec_from_model(model)
    cm = cb.compile(model)
    lo, hi = dof_limits(model)
    rng = np.random.default_rng(seed)
    n = model.dof()
    n_conv = 0
    for _ in range(N_CASES):
        q_true = np.array(random_q(model, rng))
        target = _target(model, spec, q_true)
        q0 = np.clip(q_true + rng.uniform(-0.5, 0.5, n) / np.sqrt(n), lo, hi)
        r = cb.solve_ik(cm, lo, hi, target, list(q0), None)
        if r["converged"]:
            n_conv += 1
            ep, er = _err(spec, r["q"], target)
            assert ep < TOL and er < TOL, (name, ep, er)
        assert np.all(np.array(r["q"]) >= np.array(lo) - 1e-12)
        assert np.all(np.array(r["q"]) <= np.array(hi) + 1e-12)
    return n_conv / N_CASES, model, spec, cm, lo, hi, rng


@pytest.mark.parametrize("name", list(ROBOTS))
def test_reachable_perturbed_start(name):
    rate, *_ = _run(name)
    assert rate >= 0.9, f"{name}: converged rate {rate:.2f} < 0.90"


@pytest.mark.parametrize("name", list(ROBOTS))
def test_reachable_zero_start(name):
    model = ROBOTS[name]()
    spec = spec_from_model(model)
    cm = cb.compile(model)
    lo, hi = dof_limits(model)
    rng = np.random.default_rng(5)
    for _ in range(20):
        q_true = random_q(model, rng)
        target = _target(model, spec, q_true)
        r = cb.solve_ik(cm, lo, hi, target, [0.0] * model.dof(), None)
        if r["converged"]:
            ep, er = _err(spec, r["q"], target)
            assert ep < TOL and er < TOL
        assert set(r) == {"q", "converged", "pos_error", "rot_error", "iterations", "message"}


def test_reported_error_matches_numpy():
    model = ROBOTS["oa6"]()
    spec = spec_from_model(model)
    cm = cb.compile(model)
    lo, hi = dof_limits(model)
    rng = np.random.default_rng(2)
    q_true = random_q(model, rng)
    target = _target(model, spec, q_true)
    r = cb.solve_ik(cm, lo, hi, target, [0.0] * 6, {"max_iter": 3})
    ep, er = _err(spec, r["q"], target)
    assert abs(ep - r["pos_error"]) < 1e-9
    assert abs(er - r["rot_error"]) < 1e-9


def test_position_only_planar():
    model = planar_3r()
    spec = spec_from_model(model)
    cm = cb.compile(model)
    lo, hi = dof_limits(model)
    rng = np.random.default_rng(3)
    n_ok = 0
    for _ in range(50):
        q_true = np.array(random_q(model, rng))
        target = _target(model, spec, q_true)
        q0 = np.clip(q_true + rng.uniform(-0.3, 0.3, 3), lo, hi)
        r = cb.solve_ik(cm, lo, hi, target, list(q0), {"position_only": True})
        if r["converged"]:
            n_ok += 1
            ep, _ = _err(spec, r["q"], target)
            assert ep < TOL
    assert n_ok >= 45, f"position-only converged {n_ok}/50"


def test_unreachable_target():
    model = ROBOTS["oa6"]()
    cm = cb.compile(model)
    lo, hi = dof_limits(model)
    target = np.eye(4)
    target[:3, 3] = [10.0, 0, 0]
    r = cb.solve_ik(cm, lo, hi, target, [0.0] * 6, None)
    assert r["converged"] is False
    assert r["message"] in ("max iterations", "stalled")
    assert np.isfinite(r["pos_error"]) and r["pos_error"] > 5.0


def test_limits_enforced_and_best_iterate():
    model = planar_3r()
    cm = cb.compile(model)
    lo, hi = [-0.2] * 3, [0.2] * 3
    target = np.eye(4)
    target[:3, 3] = [0.5, 0.5, 0.0]
    r = cb.solve_ik(cm, lo, hi, target, [0.0] * 3, None)
    assert all(-0.2 - 1e-12 <= v <= 0.2 + 1e-12 for v in r["q"])
    r0 = cb.solve_ik(cm, lo, hi, target, [0.0] * 3, {"max_iter": 1})
    assert r["pos_error"] <= r0["pos_error"] + 1e-12


def test_null_gain_runs_in_limits():
    model = ROBOTS["oa6"]()
    spec = spec_from_model(model)
    cm = cb.compile(model)
    lo, hi = dof_limits(model)
    rng = np.random.default_rng(4)
    q_true = random_q(model, rng)
    target = _target(model, spec, q_true)
    r = cb.solve_ik(cm, lo, hi, target, [0.0] * 6, {"null_gain": 0.5})
    assert all(l - 1e-12 <= v <= h + 1e-12 for v, l, h in zip(r["q"], lo, hi))


def test_zero_weight_ignores_rz():
    model = planar_3r()
    spec = spec_from_model(model)
    cm = cb.compile(model)
    lo, hi = dof_limits(model)
    q_true = np.array([0.3, 0.4, -0.2])
    T = _target(model, spec, q_true)
    # Rotate the target about world z: a planar arm cannot match it with
    # x, y free; with rz weight 0 it still reaches the position.
    Rz = np.eye(4)
    c, s = np.cos(0.5), np.sin(0.5)
    Rz[:2, :2] = [[c, -s], [s, c]]
    target = T.copy()
    target[:3, :3] = Rz[:3, :3] @ T[:3, :3]
    opts = {"weights": [1, 1, 1, 1, 1, 0], "max_iter": 200}
    r = cb.solve_ik(cm, lo, hi, target, list(q_true + 0.1), opts)
    ep, er = _err(spec, r["q"], target)
    assert ep < 1e-6
    # The ignored rotation is still reported (unweighted)
    assert abs(er - r["rot_error"]) < 1e-9


def test_wrong_sizes_throw():
    model = ROBOTS["oa6"]()
    cm = cb.compile(model)
    lo, hi = dof_limits(model)
    T = np.eye(4)
    with pytest.raises(ValueError):
        cb.solve_ik(cm, lo, hi, T, [0.0] * 5, None)
    with pytest.raises(ValueError):
        cb.solve_ik(cm, lo[:5], hi, T, [0.0] * 6, None)
    with pytest.raises(ValueError):
        cb.solve_ik(cm, lo, hi[:5], T, [0.0] * 6, None)


def test_unknown_option_key():
    model = ROBOTS["oa6"]()
    cm = cb.compile(model)
    lo, hi = dof_limits(model)
    with pytest.raises(ValueError):
        cb.solve_ik(cm, lo, hi, np.eye(4), [0.0] * 6, {"bogus": 1})


def test_deterministic():
    model = ROBOTS["dh6"]()
    spec = spec_from_model(model)
    cm = cb.compile(model)
    lo, hi = dof_limits(model)
    rng = np.random.default_rng(6)
    target = _target(model, spec, random_q(model, rng))
    a = cb.solve_ik(cm, lo, hi, target, [0.1] * 6, None)
    b = cb.solve_ik(cm, lo, hi, target, [0.1] * 6, None)
    assert a == b


def test_dof_limits_mimic_and_branch():
    model = random_robot(21, 4, kinds="origin_axis", branch=True, mimic=True)
    lo, hi = dof_limits(model)
    assert len(lo) == len(hi) == model.dof()
    assert all(l < h for l, h in zip(lo, hi))


def test_a14_bad_options_raise():
    """A1.4 (audit LOW): IK options that break the solver -> ValueError."""
    model = planar_3r()
    cm = cb.compile(model)
    lo, hi = [-3.0] * 3, [3.0] * 3
    T = np.eye(4)
    T[:3, 3] = [0.5, 0.5, 0.0]
    for bad in ({"damping": 0.0}, {"damping": -1.0}, {"max_iter": 0},
                {"max_iter": 10**7}, {"max_step": 0.0}, {"tol_pos": 0.0},
                {"tol_rot": -1e-6}, {"damping": float("nan")}):
        with pytest.raises(ValueError):
            cb.solve_ik(cm, lo, hi, T, [0.0] * 3, bad)
    cb.solve_ik(cm, lo, hi, T, [0.0] * 3, {"damping": 0.05, "max_iter": 50})


def test_a14_ik_multi_limits_must_be_finite():
    """A1.4 (audit LOW ik_multi): non-finite limits -> ValueError; hi <= lo is fine."""
    model = planar_3r()
    cm = cb.compile(model)
    T = np.eye(4)
    T[:3, 3] = [0.5, 0.5, 0.0]
    with pytest.raises(ValueError):
        cb.solve_ik_multi(cm, [-3.0, -3.0, float("nan")], [3.0] * 3, T, [0.0] * 3)
    cb.solve_ik_multi(cm, [-3.0, 0.5, -3.0], [3.0, 0.5, 3.0], T, [0.0] * 3)
