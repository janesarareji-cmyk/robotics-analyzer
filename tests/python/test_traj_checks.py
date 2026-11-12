"""R7.1: trajectory constraint checks."""
import numpy as np
import pytest

import core_bridge as cb

from app.planning import check_trajectory, quintic, trapezoid
from app.sim.trajectory import Trajectory, Waypoint

from .numpy_ref import inertias_from_model, numpy_dynamics, spec_from_model
from .test_cartesian_traj import planar_2r


def _heavy_2r(effort=10.0, vel=1.0):
    m = planar_2r()
    for k in (1, 2):
        i = cb.Inertia()
        i.mass = 5.0
        i.com = [0.5, 0.0, 0.0]
        i.inertia_tensor = [0.1, 0.0, 0.0, 0.1, 0.0, 0.1]
        m.links[k].inertia = i
    for j in m.joints:
        lim = j.limits
        lim.effort, lim.vel = effort, vel
        j.limits = lim
    return m


GY = (0.0, -9.81, 0.0)


def test_feasible_profile_passes():
    m = _heavy_2r(effort=1e4, vel=2.0)
    p = trapezoid([0.0, 0.0], [0.5, -0.4], [1.0, 1.0], [2.0, 2.0])
    rep = check_trajectory(p, m, limits={"accel": [2.5, 2.5]}, gravity=GY)
    assert rep.ok, rep.summary()
    assert rep.summary().startswith("OK")
    assert rep.n_samples > 10


def test_accel_too_high_flagged_with_dof_and_time():
    m = _heavy_2r(effort=1e4, vel=10.0)
    # dof 1 moves 3x farther: its accel is 3x dof 0's
    p = quintic([0.0, 0.0], [0.2, 0.6], 1.0)
    amax = 2.5
    rep = check_trajectory(p, m, limits={"accel": [amax, amax]}, gravity=GY, dt=0.001)
    v = rep.of_kind("acceleration")
    # oracle: |qdd_i(t)| = d_i |60u - 180u^2 + 120u^3|; peak at u = 0.5 -+ sqrt(3)/6
    peak = 10 / np.sqrt(3)
    exp_t = [0.5 - np.sqrt(3) / 6, 0.5 + np.sqrt(3) / 6]
    assert {x.dof for x in v} == {1}                    # dof 0: 0.2*5.77 = 1.15 < 2.5
    assert len(v) == 2                                   # two separate peaks
    for x in v:
        assert abs(x.value - 0.6 * peak) < 1e-3 and x.limit == amax
        assert min(abs(x.t - e) for e in exp_t) < 2e-3
    assert not rep.ok and "acceleration" in rep.summary()


def test_velocity_and_position_violations():
    m = _heavy_2r(effort=1e4, vel=1.0)
    tr = Trajectory([Waypoint(0.0, (0.0, 0.0)), Waypoint(1.0, (3.5, 0.5))])  # dof0 beyond pi
    rep = check_trajectory(tr, m, gravity=GY, dt=0.1)
    pos = rep.of_kind("position")
    assert len(pos) == 1 and pos[0].dof == 0 and pos[0].limit == pytest.approx(np.pi)
    assert pos[0].value == pytest.approx(3.5) and pos[0].t == pytest.approx(1.0)
    vel = rep.of_kind("velocity")
    assert [x.dof for x in vel] == [0] and vel[0].value == pytest.approx(3.5)


def test_torque_limit_vs_numpy_oracle():
    m = _heavy_2r(effort=10.0)
    q = np.array([0.0, 0.0])                   # arm stretched horizontal: max gravity load
    z = np.zeros((3, 2))
    t = np.array([0.0, 0.5, 1.0])
    rep = check_trajectory((t, np.tile(q, (3, 1)), z, z), m, gravity=GY)
    spec = spec_from_model(m)
    _, _, g = numpy_dynamics(spec, inertias_from_model(m), q, np.zeros(2), np.array(GY))
    tq = rep.of_kind("torque")
    assert {x.dof for x in tq} == {0, 1}
    for x in tq:
        assert x.value == pytest.approx(abs(g[x.dof]), rel=1e-8) and x.limit == 10.0
    assert abs(g[0]) > 10 and abs(g[1]) > 10
    # hanging straight down: no load -> pass
    rep2 = check_trajectory((t, np.tile([-np.pi / 2, 0.0], (3, 1)), z, z), m,
                            gravity=(0.0, -9.81, 0.0))
    assert not rep2.of_kind("torque"), rep2.summary()


def test_missing_limits_are_info_not_errors():
    m = planar_2r()                              # default URM effort 100, vel 1
    for j in m.joints:
        lim = j.limits
        lim.vel, lim.effort = 0.0, 0.0
        j.limits = lim
    p = quintic([0.0, 0.0], [1.0, 1.0], 0.5)     # fast, but no limit set
    rep = check_trajectory(p, m)
    assert rep.ok
    txt = " ".join(rep.info)
    assert "velocity" in txt and "acceleration" in txt and "torque" in txt


def test_tcp_speed():
    m = planar_2r()
    for j in m.joints:
        lim = j.limits
        lim.vel, lim.effort = 0.0, 0.0
        j.limits = lim
    # dof0 sweeps at constant 1 rad/s from q=(0,0): tcp radius 1.5 -> 1.5 m/s
    t = np.linspace(0, 1, 11)
    q = np.stack([t * 1.0, np.zeros_like(t)], axis=1)
    qd = np.tile([1.0, 0.0], (11, 1))
    qdd = np.zeros_like(qd)
    rep = check_trajectory((t, q, qd, qdd), m, limits={"tcp_speed": 1.0})
    v = rep.of_kind("tcp_speed")
    assert len(v) == 1 and v[0].dof == -1
    # exact: |v| = |qd0| * dist(tcp, base) with tcp radius from numpy
    assert v[0].value == pytest.approx(1.5, rel=1e-9)
    assert check_trajectory((t, q, qd, qdd), m, limits={"tcp_speed": 2.0}).ok


def test_bad_inputs():
    m = planar_2r()
    with pytest.raises(ValueError):
        check_trajectory(quintic([0.0], [1.0], 1.0), m)
    with pytest.raises(ValueError):
        check_trajectory(quintic([0.0, 0.0], [1.0, 1.0], 1.0), m, limits={"bogus": 1})
