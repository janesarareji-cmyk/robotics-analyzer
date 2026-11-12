"""R7.1: Cartesian line/arc by IK, checked with the independent numpy FK."""
import math

import numpy as np
import pytest

import core_bridge as cb

from app.planning import arc, line
from app.planning.cartesian import arc_geometry

from .numpy_ref import fk_numpy, so3_log, spec_from_model
from .robots import pose, random_q, random_robot


def planar_2r(l1=1.0, l2=0.5):
    m = cb.RobotModel()
    for i in range(3):
        lk = cb.Link()
        lk.name = f"l{i}"
        m.links.append(lk)
    for i, x in enumerate([0.0, l1]):
        j = cb.Joint()
        j.name = f"j{i}"
        j.parent_link, j.child_link = i, i + 1
        j.type = cb.JointType.Revolute
        j.kinematics = cb.JointKinematics.OriginAxis
        j.origin = pose([x, 0, 0], [0, 0, 0])
        j.axis = [0.0, 0.0, 1.0]
        lim = cb.JointLimits()
        lim.lower, lim.upper = -math.pi, math.pi
        j.limits = lim
        m.joints.append(j)
    m.tool = pose([l2, 0, 0], [0, 0, 0])
    m.tcp_link = 2
    return m


def _pose_at(spec, q):
    return fk_numpy(spec, q)[1]


def _six_dof():
    model = random_robot(21, 6, kinds="origin_axis")
    spec = spec_from_model(model)
    rng = np.random.default_rng(3)
    qa = np.array(random_q(model, rng)) * 0.5
    qb = qa + rng.uniform(-0.06, 0.06, 6)
    return model, spec, qa, qb


def test_line_6dof_geometry_and_slerp():
    model, spec, qa, qb = _six_dof()
    T0, T1 = _pose_at(spec, qa), _pose_at(spec, qb)
    res = line(model, T0, T1, qa, n=21, duration=2.0, max_joint_step=1.0)
    assert res.ok, res.message
    assert res.q.shape == (21, 6) and res.t[0] == 0.0 and res.t[-1] == 2.0
    d = T1[:3, 3] - T0[:3, 3]
    L = np.linalg.norm(d)
    total = np.linalg.norm(so3_log(T0[:3, :3].T @ T1[:3, :3]))
    for k, q in enumerate(res.q):
        T = _pose_at(spec, q)
        s = k / 20
        assert np.linalg.norm(T[:3, 3] - (T0[:3, 3] + s * d)) < 1e-6, k
        ang = np.linalg.norm(so3_log(T0[:3, :3].T @ T[:3, :3]))
        assert abs(ang - s * total) < 1e-5, k
        assert np.linalg.norm(so3_log(res.targets[k][:3, :3] @ T[:3, :3].T)) < 1e-5
    # samples lie on the ideal segment (distance to the infinite line)
    pts = np.array([_pose_at(spec, q)[:3, 3] for q in res.q]) - T0[:3, 3]
    perp = pts - np.outer(pts @ d / (L * L), d)
    assert np.max(np.linalg.norm(perp, axis=1)) < 1e-6
    tr = res.to_trajectory()
    assert tr.waypoints[0].t == 0.0 and tr.n_dof == 6


def test_line_quintic_time_law():
    model, spec, qa, qb = _six_dof()
    T0, T1 = _pose_at(spec, qa), _pose_at(spec, qb)
    res = line(model, T0, T1, qa, n=11, profile="quintic", max_joint_step=1.0)
    assert res.ok
    u = np.linspace(0, 1, 11)
    s = 10 * u**3 - 15 * u**4 + 6 * u**5
    for k, q in enumerate(res.q):
        p = _pose_at(spec, q)[:3, 3]
        assert np.linalg.norm(p - (T0[:3, 3] + s[k] * (T1[:3, 3] - T0[:3, 3]))) < 1e-6


def test_line_unreachable_endpoint_failed_at():
    m = planar_2r()                      # reach 1.5
    spec = spec_from_model(m)
    qa = [0.3, 0.5]
    T0 = _pose_at(spec, qa)
    T1 = np.eye(4)
    T1[:3, 3] = [2.45, 0.0, 0.0]         # outside
    n = 26
    res = line(m, T0, T1, qa, n=n, ik_options={"position_only": True}, max_joint_step=2.0)
    p0 = T0[:3, 3]
    r = [np.linalg.norm(p0 + k / (n - 1) * (T1[:3, 3] - p0)) for k in range(n)]
    first_out = next(k for k, x in enumerate(r) if x > 1.5)
    assert min(abs(x - 1.5) for x in r) > 0.01          # no knife-edge sample
    assert not res.ok and res.failed_at == first_out
    assert res.q.shape[0] == first_out
    with pytest.raises(ValueError):
        res.to_trajectory()


def test_line_branch_jump_flagged():
    m = planar_2r()
    spec = spec_from_model(m)
    T0 = _pose_at(spec, [0.3, 0.8])
    T1 = np.eye(4)
    T1[:3, 3] = [0.9, 0.2, 0.0]
    res = line(m, T0, T1, [0.3, 0.8], n=5, ik_options={"position_only": True},
               max_joint_step=1e-6)
    assert not res.ok and res.failed_at == 1 and "step" in res.message


def test_arc_constant_radius_and_geometry():
    m = planar_2r()
    spec = spec_from_model(m)
    c, R = np.array([0.8, 0.2, 0.0]), 0.4
    ang = [0.2, 1.4, 2.6]
    pts = [c + R * np.array([math.cos(a), math.sin(a), 0.0]) for a in ang]
    T0, T1 = np.eye(4), np.eye(4)
    T0[:3, 3], T1[:3, 3] = pts[0], pts[2]
    q0 = np.array(cb.solve_ik(cb.compile(m), [-3.2] * 2, [3.2] * 2, T0, [0.3, 0.3],
                              {"position_only": True})["q"])
    res = arc(m, T0, pts[1], T1, q0, n=41, ik_options={"position_only": True},
              max_joint_step=1.0)
    assert res.ok, res.message
    assert abs(res.extra["radius"] - R) < 1e-12
    assert abs(res.extra["sweep"] - (ang[2] - ang[0])) < 1e-12
    for k, q in enumerate(res.q):
        p = _pose_at(spec, q)[:3, 3]
        assert abs(np.linalg.norm(p - c) - R) < 1e-6, k
        want = c + R * np.array([math.cos(ang[0] + k / 40 * 2.4), math.sin(ang[0] + k / 40 * 2.4), 0])
        assert np.linalg.norm(p - want) < 1e-6
    # via point is on the path
    assert np.min(np.linalg.norm(res.targets[:, :3, 3] - pts[1], axis=1)) < 0.05


def test_arc_geometry_3d_and_collinear():
    rng = np.random.default_rng(5)
    Q, _ = np.linalg.qr(rng.normal(size=(3, 3)))
    c, R = rng.normal(size=3), 0.7
    ang = [0.1, 2.0, 4.0]                  # sweep > pi
    p = [c + R * (Q[:, 0] * math.cos(a) + Q[:, 1] * math.sin(a)) for a in ang]
    cc, r, e1, e2, sw = arc_geometry(*p)
    assert np.allclose(cc, c, atol=1e-12) and abs(r - R) < 1e-12
    assert abs(sw - (ang[2] - ang[0])) < 1e-12
    with pytest.raises(ValueError):
        arc_geometry([0, 0, 0], [1, 0, 0], [2, 0, 0])
