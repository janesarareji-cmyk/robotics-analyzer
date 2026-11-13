"""U4b.1: spline Cartesian path (centripetal Catmull-Rom through via points)."""
import numpy as np

from app.planning import spline
from app.planning.cartesian import rot_exp, spline_positions, via_poses

from .numpy_ref import fk_numpy
from .test_cartesian_traj import _pose_at, _six_dof


def _poses(spec, qa, qb):
    T0, T1 = _pose_at(spec, qa), _pose_at(spec, qb)
    return T0, T1


def test_positions_pass_through_via_points_exactly():
    pts = np.array([[0, 0, 0], [0.1, 0.05, 0], [0.2, -0.05, 0.03], [0.35, 0.0, 0.0]])
    pos, key = spline_positions(pts, 25)
    assert len(key) == 4 and key[0] == 0 and key[-1] == len(pos) - 1
    for k, p in zip(key, pts):
        assert np.linalg.norm(pos[k] - p) < 1e-9
    assert len(pos) == 3 * 24 + 1


def test_position_is_c1_across_via_points():
    pts = np.array([[0, 0, 0], [0.1, 0.1, 0], [0.25, 0.0, 0.1], [0.3, -0.1, 0.0]])
    pos, key = spline_positions(pts, 1500)
    for k in key[1:-1]:
        a = pos[k] - pos[k - 1]
        b = pos[k + 1] - pos[k]
        cosang = a @ b / (np.linalg.norm(a) * np.linalg.norm(b))
        assert cosang > 0.9999, cosang          # no corner at a via point


def test_collinear_via_points_give_a_straight_line():
    pts = np.array([[0, 0, 0], [0.1, 0.0, 0.0], [0.3, 0.0, 0.0], [0.4, 0.0, 0.0]])
    pos, _ = spline_positions(pts, 20)
    assert np.max(np.abs(pos[:, 1:])) < 1e-12
    assert np.all(np.diff(pos[:, 0]) > 0)


def test_two_points_straight():
    pos, key = spline_positions(np.array([[0, 0, 0], [1.0, 2.0, 3.0]]), 11)
    d = np.array([1.0, 2.0, 3.0])
    for k, p in enumerate(pos):
        assert np.linalg.norm(p - k / 10 * d) < 1e-12


def test_spline_ik_passes_via_and_orientation():
    model, spec, qa, qb = _six_dof()
    T0, T1 = _poses(spec, qa, qb)
    mid = 0.5 * (T0[:3, 3] + T1[:3, 3]) + np.array([0.0, 0.0, 0.02])
    vias = via_poses(T0, T1, [mid])
    assert np.allclose(vias[0][:3, 3], mid)
    res = spline(model, [T0, vias[0], T1], qa, n_per_seg=12, duration=2.0, max_joint_step=1.0)
    assert res.ok, res.message
    assert res.q.shape == (23, 6)
    assert res.t[0] == 0.0 and abs(res.t[-1] - 2.0) < 1e-12 and np.all(np.diff(res.t) > 0)
    k = res.extra["key_idx"]
    for i, Tv in zip(k, (T0, vias[0], T1)):
        Tfk = fk_numpy(spec, res.q[i])[1]
        assert np.linalg.norm(Tfk[:3, 3] - Tv[:3, 3]) < 1e-6
    assert res.extra["length"] > np.linalg.norm(T1[:3, 3] - T0[:3, 3])
    tr = res.to_trajectory()
    assert tr.n_dof == 6


def test_quintic_profile_rest_to_rest_times():
    model, spec, qa, qb = _six_dof()
    T0, T1 = _poses(spec, qa, qb)
    res = spline(model, [T0, T1], qa, n_per_seg=21, duration=1.0, profile="quintic",
                 max_joint_step=1.0)
    assert res.ok
    assert res.t[1] - res.t[0] > 1e-4 and np.all(np.diff(res.t) > 0)
    # slow at the ends, fast in the middle
    assert (res.t[1] - res.t[0]) > (res.t[11] - res.t[10])
