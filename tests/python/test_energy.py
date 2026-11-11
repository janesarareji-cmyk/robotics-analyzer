"""R4.8: kinetic / potential / total energy, joint power, torque limits, RK4 stub."""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

import core_bridge as cb

from .numpy_ref import fk_numpy, inertias_from_model, spec_from_model
from .robots import pose, random_inertias, random_q, random_robot
from .test_dynamics import _link, _oa, _setup

APP = Path(__file__).resolve().parents[2] / "app"
if str(APP) not in sys.path:
    sys.path.insert(0, str(APP))
import dyn_utils  # noqa: E402

G = np.array([0.0, 0.0, -9.81])


def _pe_numpy(model, q, g):
    """Independent oracle: V = -sum m_k g . p_com,k from fk_numpy frames."""
    frames, _ = fk_numpy(spec_from_model(model), q)
    V = 0.0
    for k, (m, com, _) in enumerate(inertias_from_model(model)):
        if m == 0.0:
            continue
        p = (frames[k] @ np.append(com, 1.0))[:3]
        V -= m * float(np.dot(g, p))
    return V


def _rel(a, b):
    return abs(a - b) / max(1.0, abs(b))


# ------------------------------------------------------------ kinetic energy
@pytest.mark.parametrize("seed", [0, 1])
def test_kinetic_energy_matches_spatial_hook(seed):
    rng = np.random.default_rng(800 + seed)
    model = random_robot(seed, 3)
    random_inertias(model, rng)
    cm, dm = _setup(model)
    for _ in range(5):
        q = random_q(model, rng)
        qd = rng.normal(size=3)
        M = np.asarray(cb.mass_matrix(dm, q))
        ke = cb.kinetic_energy(dm, q, list(qd))
        assert ke == pytest.approx(0.5 * qd @ M @ qd, rel=1e-12, abs=1e-14)
        assert ke == pytest.approx(cb.dyn_kinetic_energy_spatial(dm, q, list(qd)),
                                   rel=1e-10, abs=1e-12)


# ---------------------------------------------------------- potential energy
@pytest.mark.parametrize("theta", [0.0, 0.4, 1.2, -2.0, 3.0])
def test_pendulum_potential_closed_form(theta):
    m_kg, r, g0 = 2.0, 0.7, 9.81
    model = cb.RobotModel()
    model.base = pose((0, 0, 0), (0, 0, 0))
    model.tool = pose((0, 0, 0), (0, 0, 0))
    model.links.append(cb.Link())
    model.links.append(_link(m_kg, (r, 0, 0), 0.1 * np.eye(3)))
    model.joints.append(_oa("j0", 0, 1, (0, 0, 0), (0, 0, 0), (0, 0, 1)))
    model.tcp_link = 1
    _, dm = _setup(model)
    gvec = (0.0, -g0, 0.0)
    V = cb.potential_energy(dm, [theta], gvec)
    assert V == pytest.approx(m_kg * g0 * r * math.sin(theta), abs=1e-12)


@pytest.mark.parametrize("seed", [2, 5])
def test_potential_energy_matches_numpy(seed):
    rng = np.random.default_rng(900 + seed)
    model = random_robot(seed, 3, kinds=("dh", "origin_axis"), prismatic=0.3)
    random_inertias(model, rng)
    cm, dm = _setup(model)
    for _ in range(5):
        q = random_q(model, rng)
        V = cb.potential_energy(dm, q, tuple(G))
        assert V == pytest.approx(_pe_numpy(model, q, G), rel=1e-10, abs=1e-10)


def test_potential_energy_free_mass_height():
    # one link, joint about x at the origin, COM 1.5 m up in z (zero pose)
    model = cb.RobotModel()
    model.base = pose((0, 0, 0), (0, 0, 0))
    model.tool = pose((0, 0, 0), (0, 0, 0))
    model.links.append(cb.Link())
    model.links.append(_link(3.0, (0, 0, 1.5), 0.1 * np.eye(3)))
    model.joints.append(_oa("j0", 0, 1, (0, 0, 0), (0, 0, 0), (1, 0, 0)))
    model.tcp_link = 1
    _, dm = _setup(model)
    # zero-angle pose: COM at z = 1.5, V = m * 9.81 * z
    assert cb.potential_energy(dm, [0.0], (0.0, 0.0, -9.81)) == pytest.approx(3.0 * 9.81 * 1.5, rel=1e-12)


def test_total_energy_is_sum():
    rng = np.random.default_rng(11)
    model = random_robot(4, 3)
    random_inertias(model, rng)
    _, dm = _setup(model)
    q = random_q(model, rng)
    qd = list(rng.normal(size=3))
    tot = cb.total_energy(dm, q, qd, tuple(G))
    assert tot == pytest.approx(cb.kinetic_energy(dm, q, qd) + cb.potential_energy(dm, q, tuple(G)),
                                rel=1e-12, abs=1e-14)


# -------------------------------------------------------------- joint power
def test_joint_power_equals_tau_times_qd():
    tau = [1.5, -2.0, 0.25]
    qd = [0.4, 3.0, -8.0]
    p = cb.joint_power(tau, qd)
    assert np.allclose(p, np.array(tau) * np.array(qd), rtol=0, atol=1e-15)


def test_joint_power_size_mismatch_raises():
    with pytest.raises(ValueError):
        cb.joint_power([1.0, 2.0], [1.0])


# -------------------------------------------------------- torque limit check
def _limit_robot():
    model = random_robot(7, 3, mimic=True)
    for j, eff in zip(model.joints, [2.0, 0.0, 5.0, 1.0]):
        lim = j.limits
        lim.effort = eff
        j.limits = lim
    return model


def test_torque_limit_below_at_above():
    model = _limit_robot()
    res = dyn_utils.check_torque_limits(model, [1.0, 0.0, 6.0])
    assert [r["dof"] for r in res] == [0, 1, 2]
    below, _, above = res
    assert below["ratio"] == pytest.approx(0.5) and not below["exceeded"]
    assert above["ratio"] == pytest.approx(1.2) and above["exceeded"]
    res_at = dyn_utils.check_torque_limits(model, [2.0, 0.0, 5.0])
    assert res_at[0]["ratio"] == pytest.approx(1.0)
    assert not res_at[0]["exceeded"]
    assert not res_at[2]["exceeded"]


def test_torque_limit_names_and_no_limit():
    model = _limit_robot()
    res = dyn_utils.check_torque_limits(model, [0.0, 0.0, 0.0])
    assert [r["joint_name"] for r in res] == ["j0", "j1", "j2"]
    nl = res[1]
    assert nl["no_limit"] is True and nl["exceeded"] is False and nl["ratio"] == 0.0
    res2 = dyn_utils.check_torque_limits(model, [0.0, 3.0, 0.0])
    assert res2[1]["no_limit"] is True and res2[1]["exceeded"] is False
    assert math.isinf(res2[1]["ratio"])


def test_torque_limit_mimic_slave_not_listed():
    model = _limit_robot()
    assert model.dof() == 3
    assert len(dyn_utils.check_torque_limits(model, [0.0, 0.0, 0.0])) == 3


# ------------------------------------------------------ RK4 stub integrator
def test_rk4_energy_conservation_no_friction():
    rng = np.random.default_rng(21)
    model = random_robot(0, 3)
    random_inertias(model, rng)
    _, dm = _setup(model)
    q0 = random_q(model, rng)
    qd0 = rng.normal(size=3)
    E0 = cb.total_energy(dm, q0, list(qd0), tuple(G))
    assert abs(E0) > 1e-3
    qh, qdh = dyn_utils.integrate_rk4_forward_dynamics(
        dm, q0, qd0, lambda t, q, qd: [0.0, 0.0, 0.0], dt=1e-4, steps=10000, gravity=tuple(G))
    assert qh.shape == (10001, 3) and qdh.shape == (10001, 3)
    worst = 0.0
    for k in range(0, 10001, 10):
        E = cb.total_energy(dm, list(qh[k]), list(qdh[k]), tuple(G))
        worst = max(worst, abs(E - E0) / abs(E0))
    print("RK4 3R energy drift max |dE|/E0 =", worst)
    assert worst < 1e-8, worst


def test_rk4_viscous_friction_energy_decreases():
    rng = np.random.default_rng(33)
    model = random_robot(1, 3)
    random_inertias(model, rng)
    _, dm = _setup(model)
    q0 = random_q(model, rng)
    qd0 = rng.normal(size=3)
    b = np.array([0.5, 0.3, 0.2])
    qh, qdh = dyn_utils.integrate_rk4_forward_dynamics(
        dm, q0, qd0, lambda t, q, qd: list(-b * np.asarray(qd)), dt=1e-4, steps=10000, gravity=tuple(G))
    E = np.array([cb.total_energy(dm, list(qh[k]), list(qdh[k]), tuple(G)) for k in range(10001)])
    scale = max(1.0, abs(E[0]))
    assert np.all(np.diff(E) <= 1e-13 * scale), np.max(np.diff(E))
    assert E[-1] < E[0]
