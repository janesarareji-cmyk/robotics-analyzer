"""R4.7: mass matrix (CRBA), forward dynamics, friction."""
import numpy as np
import pytest

import core_bridge as cb

from .numpy_ref import inertias_from_model, numpy_dynamics, spec_from_model
from .robots import random_inertias, random_q, random_robot
from .test_dynamics import _link, _oa, _rand_inertia, _rel, _setup

G = np.array([0.0, 0.0, -9.81])

# 24 robots: DH + origin_axis, prismatic, branched, mimic
CONFIGS = ([dict(kinds=("origin_axis",))] * 6 + [dict(kinds=("dh",))] * 4 +
           [dict(kinds=("dh", "origin_axis"), prismatic=0.4)] * 4 +
           [dict(kinds=("dh",), modified=True)] * 2 +
           [dict(branch=True)] * 4 + [dict(mimic=True)] * 4)


def _robot(i):
    model = random_robot(i, 3 + (i % 2), **CONFIGS[i])
    random_inertias(model, np.random.default_rng(1000 + i))
    return model


WORST = {}


def _track(name, v):
    WORST[name] = max(WORST.get(name, 0.0), float(v))


@pytest.mark.parametrize("i", range(len(CONFIGS)))
def test_identities(i):
    rng = np.random.default_rng(500 + i)
    model = _robot(i)
    cm, dm = _setup(model)
    n = model.dof()
    spec, inert = spec_from_model(model), inertias_from_model(model)
    for _ in range(3):
        q = random_q(model, rng)
        qd = rng.normal(size=n)
        M = np.asarray(cb.mass_matrix(dm, q))
        assert M.shape == (n, n)
        _track("symmetry", np.linalg.norm(M - M.T) / np.linalg.norm(M))
        assert np.linalg.norm(M - M.T) <= 1e-12 * np.linalg.norm(M)
        assert np.linalg.eigvalsh(0.5 * (M + M.T)).min() > 0
        # RNEA column trick
        gq = np.array(cb.gravity_torque(dm, q, tuple(G)))
        cols = np.array([np.array(cb.inverse_dynamics(dm, q, [0.0] * n, list(np.eye(n)[k]), tuple(G))) - gq
                         for k in range(n)]).T
        _track("column", _rel(cols, M))
        assert _rel(cols, M) < 1e-10
        # oracle
        Mo, co, go = numpy_dynamics(spec, inert, q, qd, G)
        _track("oracle", _rel(M, Mo))
        assert _rel(M, Mo) < 1e-8
        _track("gravity", _rel(gq, go))
        assert np.linalg.norm(gq - go) <= 1e-7 * max(1.0, np.linalg.norm(go))
        # FD / RNEA round trip
        tau = rng.normal(size=n)
        qdd = cb.forward_dynamics(dm, q, list(qd), list(tau), tuple(G))
        back = cb.inverse_dynamics(dm, q, list(qd), qdd, tuple(G))
        _track("roundtrip", _rel(back, tau))
        assert _rel(back, tau) < 1e-10
        # skew symmetry: qd^T Mdot qd = 2 qd^T C qd
        h = 1e-6
        Md = (np.asarray(cb.mass_matrix(dm, list(np.array(q) + h * qd))) -
              np.asarray(cb.mass_matrix(dm, list(np.array(q) - h * qd)))) / (2 * h)
        c = np.array(cb.coriolis_torque(dm, q, list(qd), tuple(G)))
        lhs, scale = qd @ Md @ qd - 2 * qd @ c, max(1.0, abs(qd @ Md @ qd))
        _track("skew", abs(lhs) / scale)
        assert abs(lhs) / scale < 1e-6
        # kinetic energy
        ke = cb.dyn_kinetic_energy_spatial(dm, q, list(qd))
        _track("ke", abs(ke - 0.5 * qd @ M @ qd) / max(1.0, abs(ke)))
        assert abs(ke - 0.5 * qd @ M @ qd) <= 1e-12 * max(1.0, abs(ke))


def test_report_worst():
    print("worst relative errors", WORST)


@pytest.mark.parametrize("seed", [0, 4, 9])
def test_mimic_mass_matrix_is_projection(seed):
    rng = np.random.default_rng(700 + seed)
    model = random_robot(seed, 3, mimic=True)
    random_inertias(model, np.random.default_rng(300 + seed))
    free = random_robot(seed, 3, mimic=True)
    random_inertias(free, np.random.default_rng(300 + seed))
    free.joints[-1].mimic = None
    mult = model.joints[-1].mimic.multiplier
    _, dm = _setup(model)
    _, dmf = _setup(free)
    Jm = np.zeros((4, 3))
    Jm[:3, :3] = np.eye(3)
    Jm[3, 0] = mult
    for _ in range(5):
        q = random_q(model, rng)
        qf = q + [mult * q[0] + model.joints[-1].mimic.offset]
        M = np.asarray(cb.mass_matrix(dm, q))
        Mf = np.asarray(cb.mass_matrix(dmf, qf))
        assert np.allclose(M, Jm.T @ Mf @ Jm, rtol=1e-10, atol=1e-10)


def test_branch_siblings_decouple():
    rng = np.random.default_rng(41)
    base, fa, fb = _rand_inertia(rng), _rand_inertia(rng), _rand_inertia(rng)
    m = cb.RobotModel()
    m.links.append(cb.Link())
    m.links.append(_link(*base))
    m.links.append(_link(*fa))
    m.links.append(_link(*fb))

    def pose():
        return (rng.uniform(-.3, .3, 3), rng.uniform(-1, 1, 3))
    m.joints.append(_oa("j0", 0, 1, *pose(), rng.normal(size=3)))
    m.joints.append(_oa("f1", 1, 2, *pose(), rng.normal(size=3)))
    m.joints.append(_oa("f2", 1, 3, *pose(), rng.normal(size=3)))
    m.tcp_link = 1
    _, dm = _setup(m)
    M = np.asarray(cb.mass_matrix(dm, list(rng.uniform(-3, 3, 3))))
    assert M[1, 2] == 0.0 and M[2, 1] == 0.0
    assert abs(M[0, 1]) > 1e-6 and abs(M[0, 2]) > 1e-6


# ------------------------------------------------------------- friction
def _fric(v, c, eps=1e-3):
    f = cb.Friction()
    f.viscous, f.coulomb, f.eps = v, c, eps
    return f


def test_friction_terms():
    rng = np.random.default_rng(81)
    model = random_robot(3, 3)
    random_inertias(model, rng)
    _, dm = _setup(model)
    q = random_q(model, rng)
    qd = list(rng.normal(size=3))
    qdd = list(rng.normal(size=3))
    zero = [0.0] * 3
    fv = _fric([0.5, 0.2, 0.1], zero)
    base = np.array(cb.inverse_dynamics(dm, q, qd, qdd, tuple(G)))
    withf = np.array(cb.inverse_dynamics(dm, q, qd, qdd, tuple(G), fv))
    assert np.allclose(withf - base, np.array([0.5, 0.2, 0.1]) * qd, atol=1e-14)
    fc = _fric(zero, [0.4, 0.3, 0.2], eps=0.05)
    t = cb.friction_torque(dm, fc, [1.0, 1.0, 1.0])
    assert np.allclose(t, np.array([0.4, 0.3, 0.2]) * np.tanh(1 / 0.05), atol=1e-14)
    # friction does not enter g or C (no friction argument on those calls)
    c = np.array(cb.coriolis_torque(dm, q, qd, tuple(G)))
    ref = (np.array(cb.inverse_dynamics(dm, q, qd, zero, tuple(G))) -
           np.array(cb.inverse_dynamics(dm, q, zero, zero, tuple(G))))
    assert np.allclose(c, ref, atol=1e-14)
    # FD round trip with friction
    f = _fric([0.5, 0.2, 0.1], [0.4, 0.3, 0.2])
    tau = rng.normal(size=3)
    qdd2 = cb.forward_dynamics(dm, q, qd, list(tau), tuple(G), f)
    back = cb.inverse_dynamics(dm, q, qd, qdd2, tuple(G), f)
    assert _rel(back, tau) < 1e-10
    assert abs(f.eps - 1e-3) < 1e-15


def test_friction_per_dof_with_mimic():
    model = random_robot(0, 3, mimic=True)
    random_inertias(model, np.random.default_rng(1))
    _, dm = _setup(model)
    assert model.dof() == 3
    f = _fric([1.0, 2.0, 3.0], [0.0] * 3)
    t = cb.friction_torque(dm, f, [1.0, 1.0, 1.0])
    assert np.allclose(t, [1.0, 2.0, 3.0])


# ------------------------------------------------------------- errors
def test_massless_leaf_and_armature():
    model = random_robot(2, 3)
    random_inertias(model, np.random.default_rng(3), skip=(3,))
    _, dm = _setup(model)
    q, z = [0.1, 0.2, 0.3], [0.0] * 3
    with pytest.raises(RuntimeError):
        cb.forward_dynamics(dm, q, z, z)
    qdd = cb.forward_dynamics(dm, q, z, z, armature=[0.0, 0.0, 0.5])
    # tau = 0 = RNEA(qdd) + armature*qdd
    back = np.array(cb.inverse_dynamics(dm, q, z, qdd, tuple(G)))
    assert np.allclose(back + np.array([0, 0, 0.5]) * np.array(qdd), 0.0, atol=1e-10)


def test_wrong_sizes_throw():
    model = random_robot(1, 3)
    random_inertias(model, np.random.default_rng(4))
    _, dm = _setup(model)
    ok = [0.0] * 3
    with pytest.raises(ValueError):
        cb.mass_matrix(dm, [0.0])
    with pytest.raises(ValueError):
        cb.forward_dynamics(dm, ok, ok, [0.0])
    with pytest.raises(ValueError):
        cb.forward_dynamics(dm, [0.0], ok, ok)
    with pytest.raises(ValueError):
        cb.forward_dynamics(dm, ok, ok, ok, armature=[1.0])
    with pytest.raises(ValueError):
        cb.inverse_dynamics(dm, ok, ok, ok, friction=_fric([1.0], [0.0] * 3))
    with pytest.raises(ValueError):
        cb.friction_torque(dm, _fric([0.0] * 3, [0.0] * 3), [0.0])
