"""U4b.2: per-sample path check (reach, collision, singularity, branch jump) and tube colours."""
import numpy as np
import pytest

pytest.importorskip("fcl")

import core_bridge as cb
from app.kin_utils import current_q, dof_limits
from app.planning import line
from app.planning.path_check import BAD, OK, WARN, check_path, tcp_speed, tube_colors

from .test_collision_service import service


def _fk(model, q):
    return np.asarray(cb.fk_compiled(cb.compile(model), list(q))["tcp"], float)


UR5_Q = np.array([0.0, -1.4, 1.6, -1.8, -1.5, 0.0])


def _ur5():
    svc, proj = service("ur5")
    return svc, proj.robot_model


def test_free_line_all_ok():
    svc, model = _ur5()
    q0 = UR5_Q.copy()
    T0 = _fk(model, q0)
    T1 = T0.copy()
    T1[:3, 3] += [0.0, 0.0, 0.08]
    path = line(model, T0, T1, q0, n=20, max_joint_step=1.0)
    assert path.ok
    chk = check_path(model, path, svc)
    assert chk.ok and chk.n == 20 and chk.problem_text() == ""
    assert chk.reachable.all() and not chk.colliding.any()
    assert np.all(np.isfinite(chk.sigma_min))
    assert chk.summary().startswith("Path OK")


def test_line_through_base_is_found_bad():
    svc, model = _ur5()
    q0 = UR5_Q.copy()
    T0 = _fk(model, q0)
    T1 = T0.copy()
    T1[:3, 3] = [-T0[0, 3], -T0[1, 3], T0[2, 3]]          # the far side: the line crosses the base
    path = line(model, T0, T1, q0, n=40, max_joint_step=1.0)
    chk = check_path(model, path, svc)
    assert not chk.ok and chk.n_bad > 0 and chk.first_bad is not None
    assert chk.status[0] != BAD
    assert chk.problem_text().startswith("Sample ")
    bad = chk.segments(BAD)
    assert bad and bad[0][0] == chk.first_bad
    assert (chk.colliding.any() or (~chk.reachable).any() or chk.joint_jump.any())


def test_unreachable_samples_after_failure_are_bad():
    svc, model = _ur5()
    q0 = UR5_Q.copy()
    T0 = _fk(model, q0)
    T1 = T0.copy()
    T1[:3, 3] = [3.0, 0.0, 0.5]                           # far outside the reach
    path = line(model, T0, T1, q0, n=15, max_joint_step=1.0)
    assert not path.ok
    chk = check_path(model, path, svc)
    k = path.failed_at
    assert np.all(chk.status[k:] == BAD) and not chk.reachable[k:].any()
    assert "not reachable" in chk.reasons[-1]


def test_wrist_singularity_gives_warn_segment():
    svc, model = _ur5()
    qa = UR5_Q.copy()
    qb = UR5_Q.copy()
    qa[4], qb[4] = 0.5, 0.01                              # wrist 2 -> 0: wrist singularity
    T0, T1 = _fk(model, qa), _fk(model, qb)
    path = line(model, T0, T1, qa, n=40, max_joint_step=1.0)
    chk = check_path(model, path, svc)
    assert chk.n_warn > 0, (chk.sigma_min, path.message)
    w = chk.segments(WARN)
    assert w and w[-1][1] >= chk.n - 3 or not chk.ok
    assert chk.sigma_min[-1] < chk.sigma_min[0]


def test_start_contact_is_allowed():
    svc, proj = service("ur5")
    model = proj.robot_model
    q0 = np.zeros(model.dof())                            # zero pose: wrist under the base plane
    T0 = _fk(model, q0)
    assert not svc.check(q0, emit=False).ok
    T1 = T0.copy()
    T1[:3, 3] += [0.0, 0.0, 0.01]
    path = line(model, T0, T1, q0, n=5, max_joint_step=1.0)
    chk = check_path(model, path, svc)
    assert not chk.colliding[0]


def test_tube_colours_and_speed():
    svc, model = _ur5()
    q0 = UR5_Q.copy()
    T0 = _fk(model, q0)
    T1 = T0.copy()
    T1[:3, 3] += [0.0, 0.0, 0.06]
    path = line(model, T0, T1, q0, n=10, profile="quintic", max_joint_step=1.0)
    chk = check_path(model, path, svc)
    for mode in ("status", "clearance", "singularity"):
        c = tube_colors(chk, mode)
        assert c.shape == (10, 3) and np.all((c >= 0) & (c <= 1))
    spd = tcp_speed(path.targets, path.t)
    assert spd[5] > spd[0] and spd[5] > spd[-1]
    c = tube_colors(None, "speed", spd)
    assert c.shape == (10, 3)
    assert np.allclose(tube_colors(chk, "status")[0], tube_colors(chk, "status")[1])
    with pytest.raises(ValueError):
        tube_colors(chk, "nope")


def test_robot_view_tube():
    import pyvista as pv
    from app.viewport.robot_view import RobotView
    pl = pv.Plotter(off_screen=True)
    try:
        rv = RobotView(pl)
        pts = np.column_stack([np.linspace(0, 0.5, 20), np.zeros(20), np.full(20, 0.3)])
        cols = np.tile([0.4, 0.7, 0.4], (20, 1))
        cols[10:14] = [1.0, 0.3, 0.3]
        rv.set_path_tube(pts, cols, via_points=[pts[10]])
        assert rv.path_tube_actor is not None
        assert len(rv.path_via_actors) == 3                    # start, end, one via
        assert np.allclose(rv.path_tube_colors[11], [1.0, 0.3, 0.3])
        assert rv.path_tube_radius > 0
        mesh = rv.path_tube_mesh
        assert mesh.n_points > 20 and "rgb" in mesh.point_data
        rv.set_path_marker(pts[5])
        assert rv.path_marker_actor is not None
        assert np.allclose(np.asarray(rv.path_marker_actor.user_matrix)[:3, 3], pts[5])
        rv.set_path_marker(None)
        assert rv.path_marker_actor is None
        rv.set_path_tube(None)
        assert rv.path_tube_actor is None and rv.path_via_actors == []
    finally:
        pl.close()
