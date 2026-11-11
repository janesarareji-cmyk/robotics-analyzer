"""R4.2: geometric Jacobian vs independent numpy central differences."""
import math

import numpy as np
import pytest

import core_bridge as cb

from .numpy_ref import numeric_jacobian, numeric_jacobian_tcp, spec_from_model
from .robots import pose, random_q, random_robot

TOL = 1e-6


def _check(model, rng, n_poses, link=None, point=None):
    """Analytic vs numeric Jacobian on random poses (TCP or given link/point)."""
    spec = spec_from_model(model)
    cm = cb.compile(model)
    for _ in range(n_poses):
        q = random_q(model, rng)
        if link is None:
            Ja = cb.jacobian_tcp(cm, q)
            Jn = numeric_jacobian_tcp(spec, q)
        else:
            Ja = cb.jacobian(cm, q, link, point)
            Jn = numeric_jacobian(spec, q, link, point)
        assert Ja.shape == (6, spec["n_dof"])
        assert np.allclose(Ja[:3], Jn[:3], atol=TOL), "linear rows"
        assert np.allclose(Ja[3:], Jn[3:], atol=TOL), "angular rows"


@pytest.mark.parametrize("modified", [False, True])
def test_random_dh_robots_tcp_and_interior(modified):
    rng = np.random.default_rng(1)
    for seed in range(4):
        m = random_robot(seed, 5, kinds="dh", prismatic=0.3, modified=modified)
        assert m.validate() == []
        _check(m, rng, 25)
        _check(m, rng, 25, link=3, point=[0.1, -0.2, 0.3])


def test_random_origin_axis_robots_tcp_and_interior():
    rng = np.random.default_rng(2)
    for seed in range(4):
        m = random_robot(100 + seed, 6, kinds="origin_axis", prismatic=0.3)
        assert m.validate() == []
        _check(m, rng, 25)
        _check(m, rng, 25, link=4, point=[-0.3, 0.2, 0.15])


def test_mixed_dh_and_origin_axis():
    rng = np.random.default_rng(3)
    m = random_robot(7, 6, kinds=("dh", "origin_axis"), prismatic=0.2)
    assert m.validate() == []
    _check(m, rng, 50)
    _check(m, rng, 50, link=2, point=[0.2, 0.1, -0.1])


def test_planar_2r_closed_form():
    l1, l2 = 0.7, 0.4
    m = cb.RobotModel()
    for i in range(3):
        lk = cb.Link()
        lk.name = f"l{i}"
        m.links.append(lk)
    for i, l in enumerate((l1, l2)):
        j = cb.Joint()
        j.name = f"j{i}"
        j.parent_link, j.child_link = i, i + 1
        j.dh.a = l
        m.joints.append(j)
    m.tcp_link = 2
    cm = cb.compile(m)
    rng = np.random.default_rng(4)
    for _ in range(20):
        q1, q2 = rng.uniform(-3, 3, 2)
        s1, c1 = math.sin(q1), math.cos(q1)
        s12, c12 = math.sin(q1 + q2), math.cos(q1 + q2)
        J = cb.jacobian_tcp(cm, [q1, q2])
        ref = np.zeros((6, 2))
        ref[0] = [-l1 * s1 - l2 * s12, -l2 * s12]
        ref[1] = [l1 * c1 + l2 * c12, l2 * c12]
        ref[5] = [1, 1]
        assert np.allclose(J, ref, atol=1e-12)


def test_prismatic_cart_pendulum():
    L = 0.8
    m = cb.RobotModel()
    for i in range(3):
        lk = cb.Link()
        lk.name = f"l{i}"
        m.links.append(lk)
    cart = cb.Joint()
    cart.name = "cart"
    cart.parent_link, cart.child_link = 0, 1
    cart.type = cb.JointType.Prismatic
    cart.kinematics = cb.JointKinematics.OriginAxis
    cart.axis = [1, 0, 0]
    pend = cb.Joint()
    pend.name = "pend"
    pend.parent_link, pend.child_link = 1, 2
    pend.kinematics = cb.JointKinematics.OriginAxis
    pend.axis = [0, 1, 0]
    m.joints.append(cart)
    m.joints.append(pend)
    m.tool = pose([0, 0, -L], [0, 0, 0])
    m.tcp_link = 2
    cm = cb.compile(m)
    for x, th in [(0.0, 0.0), (0.3, 0.7), (-0.5, -1.2)]:
        J = cb.jacobian_tcp(cm, [x, th])
        ref = np.zeros((6, 2))
        ref[:3, 0] = [1, 0, 0]
        ref[:3, 1] = [-L * math.cos(th), 0, L * math.sin(th)]
        ref[4, 1] = 1
        assert np.allclose(J, ref, atol=1e-12)
        Jn = numeric_jacobian_tcp(spec_from_model(m), [x, th])
        assert np.allclose(J, Jn, atol=TOL)


def test_mimic_column_merge():
    rng = np.random.default_rng(5)
    for seed in range(6):
        m = random_robot(200 + seed, 4, kinds="origin_axis", mimic=True)
        assert m.validate() == []
        n = m.dof()
        mult = m.joints[-1].mimic.multiplier
        off = m.joints[-1].mimic.offset
        # same robot, mimic slave as an independent DOF (last DOF)
        ind = random_robot(200 + seed, 4, kinds="origin_axis", mimic=True)
        ind.joints[-1].mimic = None
        assert ind.dof() == n + 1
        cm_dep, cm_ind = cb.compile(m), cb.compile(ind)
        link = len(m.links) - 1  # child of the mimic slave
        for _ in range(10):
            q = random_q(m, rng)
            q_ind = q + [mult * q[0] + off]
            Jd = cb.jacobian(cm_dep, q, link, [0.1, 0.2, -0.1])
            Ji = cb.jacobian(cm_ind, q_ind, link, [0.1, 0.2, -0.1])
            expect = Ji[:, :n].copy()
            expect[:, 0] += mult * Ji[:, n]
            assert np.allclose(Jd, expect, atol=1e-12)
        # and vs numeric on the true (mimic) model
        _check(m, rng, 10, link=link, point=[0.1, 0.2, -0.1])
        _check(m, rng, 10)


def test_fixed_joint_adds_no_column():
    rng = np.random.default_rng(6)
    m = random_robot(11, 5, kinds="origin_axis")
    m.joints[2].type = cb.JointType.Fixed
    assert m.validate() == []
    assert m.dof() == 4
    cm = cb.compile(m)
    q = random_q(m, rng)
    J = cb.jacobian_tcp(cm, q)
    assert J.shape == (6, 4)
    _check(m, rng, 30)


def test_branch_dofs_off_path_are_zero():
    rng = np.random.default_rng(7)
    for seed in range(6):
        m = random_robot(300 + seed, 5, kinds="origin_axis", branch=True)
        assert m.validate() == []
        n = 5
        cm = cb.compile(m)
        q = random_q(m, rng)
        J = cb.jacobian_tcp(cm, q)  # tcp = end of chain
        assert J.shape == (6, n + 1)
        assert np.all(J[:, n] == 0.0)  # branch joint dof
        assert np.any(J[:, :n] != 0.0)
        # at the branch tip: chain joints below the branch parent are zero
        parent = m.joints[n].parent_link
        Jb = cb.jacobian(cm, q, n + 1)
        assert np.all(Jb[:, parent:n] == 0.0)
        assert np.any(Jb[:, n] != 0.0)
        _check(m, rng, 10)
        _check(m, rng, 10, link=n + 1, point=[0.05, 0.0, 0.1])


def test_base_link_jacobian_is_zero():
    m = random_robot(1, 4, kinds=("dh", "origin_axis"))
    cm = cb.compile(m)
    J = cb.jacobian(cm, random_q(m, np.random.default_rng(0)), 0, [0.3, 0.1, 0.2])
    assert J.shape == (6, 4)
    assert np.all(J == 0.0)


def test_errors_value_error():
    m = random_robot(2, 3)
    cm = cb.compile(m)
    with pytest.raises(ValueError):
        cb.jacobian(cm, [0.0, 0.0], 3)
    with pytest.raises(ValueError):
        cb.jacobian_tcp(cm, [0.0] * 4)
    with pytest.raises(ValueError):
        cb.jacobian(cm, [0.0] * 3, 4)
    with pytest.raises(ValueError):
        cb.jacobian(cm, [0.0] * 3, -1)
