"""R4.6: inverse dynamics (RNEA) vs the independent Lagrangian oracle."""
import numpy as np
import pytest

import core_bridge as cb

from .numpy_ref import (inertias_from_model, numpy_dynamics, pose_matrix,
                        spec_from_model)
from .robots import pose, random_inertias, random_q, random_robot

G = np.array([0.0, 0.0, -9.81])


def _setup(model):
    cm = cb.compile(model)
    return cm, cb.build_dyn_model(model, cm)


def _rel(a, b):
    a, b = np.asarray(a), np.asarray(b)
    return np.linalg.norm(a - b) / max(1.0, np.linalg.norm(b))


def _oracle_tau(model, q, qd, qdd, g=G):
    spec = spec_from_model(model)
    M, c, grav = numpy_dynamics(spec, inertias_from_model(model), q, qd, g)
    return M @ np.asarray(qdd) + c + grav, M


# ------------------------------------------------- random spatial 3R
@pytest.mark.parametrize("seed", [0, 1])
def test_3r_spatial_vs_numpy(seed):
    rng = np.random.default_rng(100 + seed)
    model = random_robot(seed, 3)
    random_inertias(model, rng)
    cm, dm = _setup(model)
    worst = 0.0
    for _ in range(25):
        q = random_q(model, rng)
        for _ in range(2):
            qd, qdd = rng.normal(size=3), rng.normal(size=3)
            ref, _ = _oracle_tau(model, q, qd, qdd)
            tau = cb.inverse_dynamics(dm, q, list(qd), list(qdd), tuple(G))
            worst = max(worst, _rel(tau, ref))
    print("worst 3R relative error", worst)
    assert worst < 1e-8, worst


def test_3r_mixed_dh_prismatic_vs_numpy():
    rng = np.random.default_rng(7)
    model = random_robot(3, 3, kinds=("dh", "origin_axis"), prismatic=0.4)
    random_inertias(model, rng)
    cm, dm = _setup(model)
    for _ in range(10):
        q = random_q(model, rng)
        qd, qdd = rng.normal(size=3), rng.normal(size=3)
        ref, _ = _oracle_tau(model, q, qd, qdd)
        tau = cb.inverse_dynamics(dm, q, list(qd), list(qdd), tuple(G))
        assert _rel(tau, ref) < 1e-8


def test_split_terms_consistent():
    rng = np.random.default_rng(11)
    model = random_robot(5, 3)
    random_inertias(model, rng)
    cm, dm = _setup(model)
    q = random_q(model, rng)
    qd = list(rng.normal(size=3))
    g = cb.gravity_torque(dm, q, tuple(G))
    b = cb.bias_torque(dm, q, qd, tuple(G))
    c = cb.coriolis_torque(dm, q, qd, tuple(G))
    z = [0, 0, 0]
    assert np.allclose(g, cb.inverse_dynamics(dm, q, z, z, tuple(G)), atol=1e-14)
    assert np.allclose(b, cb.inverse_dynamics(dm, q, qd, z, tuple(G)), atol=1e-14)
    assert np.allclose(c, np.array(b) - np.array(g), atol=1e-14)
    spec = spec_from_model(model)
    _, c_ref, g_ref = numpy_dynamics(spec, inertias_from_model(model), q, qd, G)
    assert _rel(g, g_ref) < 1e-8
    assert _rel(c, c_ref) < 1e-8


# ------------------------------------------------------------- mimic
@pytest.mark.parametrize("seed", [0, 4, 9])
def test_mimic_projection(seed):
    rng = np.random.default_rng(200 + seed)
    model = random_robot(seed, 3, mimic=True)
    random_inertias(model, np.random.default_rng(300 + seed))
    free = random_robot(seed, 3, mimic=True)
    random_inertias(free, np.random.default_rng(300 + seed))   # same inertias
    free.joints[-1].mimic = None                               # slave becomes DOF 3
    mult = model.joints[-1].mimic.multiplier
    offset = model.joints[-1].mimic.offset
    cm, dm = _setup(model)
    cmf, dmf = _setup(free)
    assert model.dof() == 3 and free.dof() == 4
    for _ in range(10):
        q = random_q(model, rng)
        qd, qdd = list(rng.normal(size=3)), list(rng.normal(size=3))
        qf = q + [mult * q[0] + offset]
        qdf = qd + [mult * qd[0]]
        qddf = qdd + [mult * qdd[0]]
        tf = np.array(cb.inverse_dynamics(dmf, qf, qdf, qddf, tuple(G)))
        expect = tf[:3].copy()
        expect[0] += mult * tf[3]
        tau = cb.inverse_dynamics(dm, q, qd, qdd, tuple(G))
        assert np.allclose(tau, expect, rtol=1e-10, atol=1e-10)
        ref, _ = _oracle_tau(model, q, qd, qdd)   # and against the Lagrangian oracle
        assert _rel(tau, ref) < 1e-8


# ------------------------------------------------- fixed joint / branch
def _rpy_of(R):
    return [np.arctan2(R[2, 1], R[2, 2]), -np.arcsin(R[2, 0]), np.arctan2(R[1, 0], R[0, 0])]


def _link(m, c, I):
    lk = cb.Link()
    lk.name = "x"
    i = cb.Inertia()
    i.mass = float(m)
    i.com = [float(v) for v in c]
    i.inertia_tensor = [float(I[0, 0]), float(I[0, 1]), float(I[0, 2]),
                        float(I[1, 1]), float(I[1, 2]), float(I[2, 2])]
    lk.inertia = i
    return lk


def _oa(name, p, c, xyz, rpy, axis, fixed=False):
    j = cb.Joint()
    j.name = name
    j.parent_link, j.child_link = p, c
    j.type = cb.JointType.Fixed if fixed else cb.JointType.Revolute
    j.kinematics = cb.JointKinematics.OriginAxis
    j.origin = pose(xyz, rpy)
    j.axis = [float(v) / np.linalg.norm(axis) for v in axis]
    return j


def _rand_inertia(rng):
    A = rng.normal(size=(3, 3))
    Q, _ = np.linalg.qr(A)
    return float(rng.uniform(0.5, 2)), rng.uniform(-0.3, 0.3, 3), Q @ np.diag(rng.uniform(0.02, 0.2, 3)) @ Q.T


def _about(m, c, I, new_c):
    """Tensor about a new point: I + m (|d|^2 1 - d d^T)."""
    d = c - new_c
    return I + m * ((d @ d) * np.eye(3) - np.outer(d, d))


def test_fixed_joint_in_chain_merges_into_parent():
    """Fixed link 2 rigidly follows link 1: same tau as one merged link."""
    rng = np.random.default_rng(31)
    m1, c1, I1 = _rand_inertia(rng)
    m2, c2, I2 = _rand_inertia(rng)
    m3, c3, I3 = _rand_inertia(rng)
    ax0, ax1 = rng.normal(size=3), rng.normal(size=3)
    o0 = (rng.uniform(-.3, .3, 3), rng.uniform(-1, 1, 3))
    of = (rng.uniform(-.3, .3, 3), rng.uniform(-1, 1, 3))
    o1 = (rng.uniform(-.3, .3, 3), rng.uniform(-1, 1, 3))

    a = cb.RobotModel()
    a.links.append(cb.Link())
    for lk in (_link(m1, c1, I1), _link(m2, c2, I2), _link(m3, c3, I3)):
        a.links.append(lk)
    a.joints.append(_oa("j0", 0, 1, *o0, ax0))
    a.joints.append(_oa("jf", 1, 2, *of, [0, 0, 1], fixed=True))
    a.joints.append(_oa("j1", 2, 3, *o1, ax1))
    a.tcp_link = 3

    Tf = pose_matrix(*of)
    Tm = Tf @ pose_matrix(*o1)
    R = Tf[:3, :3]
    m_tot = m1 + m2
    c2_in_1 = R @ c2 + Tf[:3, 3]
    c_tot = (m1 * c1 + m2 * c2_in_1) / m_tot
    I_tot = _about(m1, c1, I1, c_tot) + _about(m2, c2_in_1, R @ I2 @ R.T, c_tot)
    b = cb.RobotModel()
    b.links.append(cb.Link())
    b.links.append(_link(m_tot, c_tot, I_tot))
    b.links.append(_link(m3, c3, I3))
    b.joints.append(_oa("j0", 0, 1, *o0, ax0))
    b.joints.append(_oa("j1", 1, 2, Tm[:3, 3], _rpy_of(Tm[:3, :3]), ax1))
    b.tcp_link = 2
    _, dma = _setup(a)
    _, dmb = _setup(b)
    for _ in range(20):
        q = list(rng.uniform(-3, 3, 2))
        qd, qdd = list(rng.normal(size=2)), list(rng.normal(size=2))
        ta = cb.inverse_dynamics(dma, q, qd, qdd, tuple(G))
        tb = cb.inverse_dynamics(dmb, q, qd, qdd, tuple(G))
        assert np.allclose(ta, tb, rtol=1e-10, atol=1e-10)


def test_branch_is_sum_of_single_fingers():
    rng = np.random.default_rng(41)
    base_inertia = _rand_inertia(rng)
    fa, fb = _rand_inertia(rng), _rand_inertia(rng)

    def rpose():
        return (rng.uniform(-.3, .3, 3), rng.uniform(-1, 1, 3))
    o0, oa_, ob_ = rpose(), rpose(), rpose()
    ax = [rng.normal(size=3) for _ in range(3)]
    zero = (0.0, np.zeros(3), np.zeros((3, 3)))

    def robot(l1, fingers):
        m = cb.RobotModel()
        m.links.append(cb.Link())
        m.links.append(_link(*l1))
        m.joints.append(_oa("j0", 0, 1, *o0, ax[0]))
        for k, (inert, o, a) in enumerate(fingers):
            m.links.append(_link(*inert))
            m.joints.append(_oa(f"f{k}", 1, 2 + k, *o, a))
        m.tcp_link = 1
        return m

    full = robot(base_inertia, [(fa, oa_, ax[1]), (fb, ob_, ax[2])])
    only_a = robot(base_inertia, [(fa, oa_, ax[1])])
    only_b = robot(zero, [(fb, ob_, ax[2])])
    dms = [_setup(m)[1] for m in (full, only_a, only_b)]
    for _ in range(20):
        q = list(rng.uniform(-3, 3, 3))
        qd, qdd = list(rng.normal(size=3)), list(rng.normal(size=3))
        t = cb.inverse_dynamics(dms[0], q, qd, qdd, tuple(G))
        ta = cb.inverse_dynamics(dms[1], [q[0], q[1]], [qd[0], qd[1]], [qdd[0], qdd[1]], tuple(G))
        tb = cb.inverse_dynamics(dms[2], [q[0], q[2]], [qd[0], qd[2]], [qdd[0], qdd[2]], tuple(G))
        assert np.allclose(t, [ta[0] + tb[0], ta[1], tb[1]], rtol=1e-10, atol=1e-10)


# ------------------------------------------------------------- hooks
def test_kinetic_energy_hook_matches_oracle():
    rng = np.random.default_rng(51)
    model = random_robot(2, 3, kinds=("dh", "origin_axis"), prismatic=0.3, branch=True, mimic=True)
    random_inertias(model, rng)
    cm, dm = _setup(model)
    spec = spec_from_model(model)
    inert = inertias_from_model(model)
    for _ in range(3):
        q = random_q(model, rng)
        qd = rng.normal(size=model.dof())
        M, _, _ = numpy_dynamics(spec, inert, q, qd, G)
        ke = cb.dyn_kinetic_energy_spatial(dm, q, list(qd))
        assert abs(ke - 0.5 * qd @ M @ qd) <= 1e-8 * max(1.0, abs(ke))


@pytest.mark.parametrize("kwargs", [dict(), dict(branch=True, mimic=True),
                                    dict(kinds=("dh",), modified=True)])
def test_dyn_frames_equal_fk(kwargs):
    rng = np.random.default_rng(61)
    model = random_robot(8, 4, **kwargs)
    cm, dm = _setup(model)
    for _ in range(5):
        q = random_q(model, rng)
        fr = cb.dyn_frames(dm, q)
        ref = cb.fk(model, q)["frames"]
        assert len(fr) == len(ref)
        for a, b in zip(fr, ref):
            assert np.max(np.abs(np.asarray(a) - np.asarray(b))) < 1e-12


def test_wrong_sizes_throw():
    model = random_robot(1, 3)
    cm, dm = _setup(model)
    ok = [0.0] * 3
    for args in (([0.0] * 2, ok, ok), (ok, [0.0] * 4, ok), (ok, ok, [])):
        with pytest.raises(ValueError):
            cb.inverse_dynamics(dm, *args)
    with pytest.raises(ValueError):
        cb.gravity_torque(dm, [0.0])
    with pytest.raises(ValueError):
        cb.dyn_frames(dm, [0.0])
    with pytest.raises(ValueError):
        cb.dyn_kinetic_energy_spatial(dm, ok, [0.0])


def test_zero_gravity_rest_gives_zero():
    rng = np.random.default_rng(71)
    model = random_robot(6, 3, branch=True, mimic=True)
    random_inertias(model, rng)
    cm, dm = _setup(model)
    z = [0.0] * model.dof()
    q = random_q(model, rng)
    tau = cb.inverse_dynamics(dm, q, z, z, (0.0, 0.0, 0.0))
    assert np.max(np.abs(tau)) < 1e-15
    assert np.max(np.abs(cb.inverse_dynamics(dm, q, z, z))) > 1e-3   # default gravity -z


def test_a14_forward_dynamics_nonfinite_input():
    """A1.4 (audit LOW): NaN q/qd/tau in forward dynamics -> error, never NaN qdd."""
    model = random_robot(1, 3)
    cm, dm = _setup(model)
    ok = [0.0] * 3
    for args in (([float("nan"), 0, 0], ok, ok), (ok, [0, float("inf"), 0], ok),
                 (ok, ok, [0, 0, float("nan")])):
        with pytest.raises(ValueError):
            cb.forward_dynamics(dm, *args)
