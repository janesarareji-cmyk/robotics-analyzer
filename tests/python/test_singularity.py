"""R4.5: singularity indicators (analyze_singularity) vs independent numpy checks."""
import math

import numpy as np
import pytest

import core_bridge as cb

from app.kin_utils import singularity_at
from .robots import pose, random_q, random_robot

SV_TOL = 1e-10


def _planar_2r(l1, l2):
    """Planar 2R (DH, a = link length), tcp = link 2. Same build as test_jacobian."""
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
    return m


def _wrist_robot():
    """6-DOF arm with a spherical wrist (origin_axis joints).

    j0 z (base), j1 y, j2 y (shoulder/elbow), wrist centre W at the origins of
    j3 (z), j4 (y), j5 (z). q[4] = 0 makes the axes of j3 and j5 collinear.
    """
    m = cb.RobotModel()
    for i in range(7):
        lk = cb.Link()
        lk.name = f"l{i}"
        m.links.append(lk)
    spec = [
        ("j0", (0, 0, 0.0), (0, 0, 1)),
        ("j1", (0, 0, 0.3), (0, 1, 0)),
        ("j2", (0, 0, 0.4), (0, 1, 0)),
        ("j3", (0.1, 0.0, 0.1), (0, 0, 1)),
        ("j4", (0.0, 0.0, 0.0), (0, 1, 0)),
        ("j5", (0.0, 0.0, 0.0), (0, 0, 1)),
    ]
    for i, (name, xyz, ax) in enumerate(spec):
        j = cb.Joint()
        j.name = name
        j.parent_link, j.child_link = i, i + 1
        j.kinematics = cb.JointKinematics.OriginAxis
        j.origin = pose(xyz, [0, 0, 0])
        j.axis = [float(v) for v in ax]
        m.joints.append(j)
    m.tool = pose([0, 0, 0.15], [0, 0, 0])
    m.tcp_link = 6
    assert m.validate() == []
    return m


def _rand_jac(rng, rows, cols):
    return rng.normal(size=(rows, cols))


def test_2r_position_manipulability_and_sigma():
    l1, l2 = 0.7, 0.4
    cm = cb.compile(_planar_2r(l1, l2))
    rng = np.random.default_rng(11)
    for _ in range(20):
        q = [float(v) for v in rng.uniform(-3, 3, 2)]
        J = cb.jacobian_tcp(cm, q)
        info = cb.analyze_singularity(J, mode="position")
        assert info["manipulability"] == pytest.approx(abs(l1 * l2 * math.sin(q[1])), abs=1e-12)
        A = J[0:3, :]  # position block (2 nonzero columns)
        eig = np.linalg.eigvalsh(A.T @ A)
        ref = np.sqrt(np.sort(np.clip(eig, 0.0, None))[::-1])
        assert np.allclose(info["singular_values"], ref, atol=1e-12, rtol=0)
        assert info["sigma_max"] == pytest.approx(ref[0], abs=1e-12)
        assert info["sigma_min"] == pytest.approx(ref[1], abs=1e-12)


def test_2r_near_singular_at_straight_and_folded():
    cm = cb.compile(_planar_2r(0.7, 0.4))
    for q2 in (0.0, math.pi):
        info = cb.analyze_singularity(cb.jacobian_tcp(cm, [0.3, q2]), mode="position")
        assert info["near_singular"] is True
    info = cb.analyze_singularity(cb.jacobian_tcp(cm, [0.3, math.pi / 2]), mode="position")
    assert info["near_singular"] is False


def test_wrist_aligned_full_mode_singular():
    m = _wrist_robot()
    cm = cb.compile(m)
    q = [0.3, -0.4, 0.5, 0.7, 0.0, -0.2]  # q[4] = 0: axes 4 and 6 collinear
    info = cb.analyze_singularity(cb.jacobian_tcp(cm, q), mode="full")
    assert info["sigma_min"] < 1e-9
    assert info["near_singular"] is True
    assert info["manipulability"] < 1e-9
    assert len(info["singular_values"]) == 6


def test_wrist_generic_pose_not_singular():
    m = _wrist_robot()
    cm = cb.compile(m)
    q = [0.3, -0.4, 0.5, 0.7, 0.6, -0.2]
    info = cb.analyze_singularity(cb.jacobian_tcp(cm, q), mode="full")
    assert info["near_singular"] is False
    assert info["sigma_min"] > 1e-3
    assert info["condition"] == pytest.approx(info["sigma_max"] / info["sigma_min"], rel=1e-12)


def test_threshold_changes_flag():
    m = _wrist_robot()
    cm = cb.compile(m)
    J = cb.jacobian_tcp(cm, [0.3, -0.4, 0.5, 0.7, 0.6, -0.2])
    s = cb.analyze_singularity(J)["sigma_min"]
    assert cb.analyze_singularity(J, threshold=2 * s)["near_singular"] is True
    assert cb.analyze_singularity(J, threshold=0.5 * s)["near_singular"] is False
    assert cb.analyze_singularity(J)["near_singular"] is False  # default 1e-3


def test_unknown_mode_raises():
    J = np.eye(6)
    with pytest.raises(ValueError):
        cb.analyze_singularity(J, mode="linear")
    with pytest.raises(ValueError):
        cb.analyze_singularity(J, mode="")


def test_zero_columns_dropped_branch():
    """Branch DOF is off the active chain: its Jacobian column is exactly zero."""
    rng = np.random.default_rng(12)
    for seed in range(4):
        m = random_robot(300 + seed, 4, kinds="origin_axis", branch=True)
        assert m.validate() == []
        cm = cb.compile(m)
        names = [j.name for j in m.joints]
        qi = m.q_index_map()[names.index("jb")]
        q = random_q(m, rng)
        J = cb.jacobian_tcp(cm, q)
        assert np.all(J[:, qi] == 0.0)
        full = cb.analyze_singularity(J)
        chain = cb.analyze_singularity(np.delete(J, qi, axis=1))
        assert len(full["singular_values"]) == min(6, 4)
        assert np.allclose(full["singular_values"], chain["singular_values"], atol=SV_TOL, rtol=0)
        assert full["manipulability"] == pytest.approx(chain["manipulability"], abs=SV_TOL)
        assert full["near_singular"] == chain["near_singular"]


def test_all_zero_jacobian():
    info = cb.analyze_singularity(np.zeros((6, 3)))
    assert info["near_singular"] is True
    assert info["manipulability"] == 0.0
    assert info["sigma_min"] == 0.0 and info["sigma_max"] == 0.0
    assert info["condition"] == math.inf


def test_random_matrices_vs_numpy_svd():
    rng = np.random.default_rng(13)
    modes = {"full": slice(0, 6), "position": slice(0, 3), "angular": slice(3, 6)}
    for cols in (3, 5, 7):
        for _ in range(5):
            J = _rand_jac(rng, 6, cols)
            for mode, rows in modes.items():
                info = cb.analyze_singularity(J, mode=mode)
                ref = np.linalg.svd(J[rows], compute_uv=False)
                assert np.allclose(info["singular_values"], ref, atol=SV_TOL, rtol=0), mode
                assert info["sigma_min"] == pytest.approx(ref[-1], abs=SV_TOL)
                assert info["manipulability"] == pytest.approx(np.prod(ref), abs=SV_TOL)


def test_random_matrix_zero_column_dropped_matches_numpy():
    rng = np.random.default_rng(14)
    J = _rand_jac(rng, 6, 5)
    J_z = np.insert(J, 2, 0.0, axis=1)  # exact zero column in the middle
    info = cb.analyze_singularity(J_z)
    ref = np.linalg.svd(J, compute_uv=False)
    assert np.allclose(info["singular_values"], ref, atol=SV_TOL, rtol=0)


def test_singularity_at_helper_matches_binding():
    m = _wrist_robot()
    cm = cb.compile(m)
    q = [0.3, -0.4, 0.5, 0.7, 0.6, -0.2]
    a = singularity_at(cm, q, mode="position", threshold=1e-2)
    b = cb.analyze_singularity(cb.jacobian_tcp(cm, q), mode="position", threshold=1e-2)
    assert a == b
