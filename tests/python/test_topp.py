"""F1.3: torque-aware time parametrisation (TOPP-RA, ``app/planning/topp.py``).

Oracles: exact vertex enumeration with ``itertools`` for the LP, the analytic trapezoid /
triangle time for the double integrator, a fine grid (N = 1000) reference run, and the
independent Lagrangian dynamics of ``numpy_ref`` (never ``core_bridge``) for the torque.
"""
import itertools
import math
import time
from pathlib import Path

import numpy as np
import pytest

import core_bridge as cb

from app.importers import import_urdf
from app.planning.shortcut import time_parametrize
from app.planning.topp import _lp2d, solve_topp, solution_trajectory
from robotics_api.errors import PlanningError

from .numpy_ref import inertias_from_model, numpy_dynamics, spec_from_model
from .robots import pose
from .test_traj_checks import GY, _heavy_2r

FX = Path(__file__).parent / "fixtures" / "urdf"
G6 = (0.0, 0.0, -9.81)
START6 = [-1.0, -1.5, 0.36, 0.0, -0.25, 0.0, 0.0]
MID6 = [0.0, -1.0, 0.8, 0.3, -0.25, 0.0, 0.0]
GOAL6 = [1.0, -1.5, 0.36, 0.0, -0.25, 0.0, 0.0]


def six_dof():
    return import_urdf(str(FX / "six_dof_with_gripper.urdf")).model


def one_dof():
    m = cb.RobotModel()
    for i in range(2):
        lk = cb.Link()
        lk.name = f"l{i}"
        m.links.append(lk)
    j = cb.Joint()
    j.name = "j0"
    j.parent_link, j.child_link = 0, 1
    j.type = cb.JointType.Revolute
    j.kinematics = cb.JointKinematics.OriginAxis
    j.origin = pose([0, 0, 0], [0, 0, 0])
    j.axis = [0.0, 0.0, 1.0]
    lim = cb.JointLimits()
    lim.lower, lim.upper = -10.0, 10.0
    j.limits = lim
    m.joints.append(j)
    m.tcp_link = 1
    return m


def numpy_tau(model, traj, g, n=15):
    spec, inert = spec_from_model(model), inertias_from_model(model)
    worst = np.zeros(model.dof())
    for t in np.linspace(0.0, traj.duration, n):
        M, c, grav = numpy_dynamics(spec, inert, traj.q(t), traj.qd(t), g)
        worst = np.maximum(worst, np.abs(M @ traj.qdd(t) + c + grav))
    return worst


def peak_torque(model, traj, g, n=300):
    dm = cb.build_dyn_model(model, cb.compile(model))
    return np.max([np.abs(cb.inverse_dynamics(dm, list(traj.q(t)), list(traj.qd(t)),
                                              list(traj.qdd(t)), g))
                   for t in np.linspace(0.0, traj.duration, n)], axis=0)


# ---------------------------------------------------------------------------------- LP
def _brute(c, A, b):
    best = None
    for i, j in itertools.combinations(range(len(b)), 2):
        M = np.array([A[i], A[j]])
        if abs(np.linalg.det(M)) < 1e-12:
            continue
        z = np.linalg.solve(M, [b[i], b[j]])
        if np.all(A @ z <= b + 1e-9):
            v = float(c @ z)
            best = v if best is None or v > best else best
    return best


def test_lp2d_matches_brute_force_on_200_random_lps():
    rng = np.random.default_rng(1)
    n_empty = 0
    for _ in range(200):
        m = int(rng.integers(3, 9))
        A = rng.normal(size=(m, 2))
        b = rng.uniform(-0.3, 2.0, size=m)          # some LPs are empty
        A = np.vstack([A, [[1, 0], [-1, 0], [0, 1], [0, -1]]])
        b = np.concatenate([b, [5.0] * 4])
        c = rng.normal(size=2)
        ref = _brute(c, A, b)
        got = _lp2d(c, A, b)
        if ref is None:
            n_empty += 1
            assert got is None
            continue
        assert got is not None
        z, val = got
        assert abs(val - ref) < 1e-9
        assert np.all(A @ z <= b + 1e-8)
        # coarse grid search can never beat the optimum
        g = np.linspace(-5, 5, 41)
        zz = np.array([[x, y] for x in g for y in g])
        ok = np.all(zz @ A.T <= b[None, :] + 1e-12, axis=1)
        if np.any(ok):
            assert (zz[ok] @ c).max() <= val + 1e-9
    assert n_empty < 150


# -------------------------------------------------------------------- double integrator
@pytest.mark.parametrize("D,v,a", [(1.0, 1.0, 2.0), (0.2, 1.0, 2.0), (3.0, 0.5, 4.0)])
def test_double_integrator_matches_trapezoid(D, v, a):
    m = one_dof()
    sol, kind, _ = solve_topp(m, [[0.0], [D]], [v], [a], [math.inf], gravity=(0, 0, 0), n_grid=200)
    ref = D / v + v / a if D >= v * v / a else 2.0 * math.sqrt(D / a)
    assert kind == "linear"
    assert abs(sol.duration - ref) / ref < 0.01
    tp = time_parametrize([[0.0], [D]], m, vmax=v, amax=a, method="topp")
    assert tp.report.ok, tp.report.summary()
    # U2.9: per-waypoint velocity (Hermite) removes the ringing at the acceleration jumps
    assert ref * 0.99 < tp.trajectory.duration < ref * 1.03


# --------------------------------------------------------------------- planar 2R, torque
def test_planar_2r_torque_limited_time_optimal():
    m = _heavy_2r(effort=1e4, vel=10.0)
    path = np.array([[0.0, 0.0], [1.0, 0.5], [1.5, -0.8]])
    eff = [200.0, 200.0]
    lim = {"effort": eff, "vel": [10.0, 10.0], "accel": [50.0, 50.0]}
    # the old method ignores torque: its peak is far above the limit
    old = time_parametrize(path, m, limits=lim, vmax=10.0, amax=50.0, gravity=GY, check_torque=True)
    assert old.report.of_kind("torque")
    assert peak_torque(m, old.trajectory, GY).max() > 2 * 200.0
    tp = time_parametrize(path, m, limits=lim, vmax=10.0, amax=50.0, gravity=GY, method="topp")
    assert tp.report.ok, tp.report.summary()
    assert not tp.report.of_kind("torque")
    pk = peak_torque(m, tp.trajectory, GY)
    assert np.all(pk <= 200.0 + 1e-6) and pk[0] > 150.0          # the limit is really active
    # time optimality: some constraint is binding at every stage; same time as a fine grid
    sol, _, _ = solve_topp(m, path, [10, 10], [50, 50], eff, gravity=GY, n_grid=100)
    ref, _, _ = solve_topp(m, path, [10, 10], [50, 50], eff, gravity=GY, n_grid=1000)
    assert all(len(a) > 0 for a in sol.active)
    assert any(n.startswith("tau") for a in sol.active for n in a)
    assert abs(sol.duration - ref.duration) / ref.duration < 0.02
    # independent torque (numpy Lagrangian) of the final trajectory
    assert np.all(numpy_tau(m, tp.trajectory, GY) <= 200.0 + 1e-6)


# ------------------------------------------------------------------------- six_dof
def test_six_dof_40_newton_meters():
    m = six_dof()
    path = np.array([START6, MID6, GOAL6])
    lim = {"effort": 40.0}
    tp = time_parametrize(path, m, limits=lim, method="topp")
    assert tp.report.ok, tp.report.summary()
    assert not tp.report.of_kind("torque")
    sc = time_parametrize(path, m, limits=lim, check_torque=True)
    assert tp.trajectory.duration <= sc.trajectory.duration
    pk = peak_torque(m, tp.trajectory, G6, n=120)
    assert np.all(pk <= 40.0 + 1e-6)
    ind = numpy_tau(m, tp.trajectory, G6, n=6)                   # independent oracle
    assert np.all(ind <= 40.0 + 1e-6), ind
    assert any("topp" in n for n in tp.notes)


def test_effort_below_gravity_raises_without_hang():
    m = six_dof()                                               # effort 10 < 25.9 gravity at q = 0
    path = np.array([[0.0] * 7, [0.3, 0.0, 0.2, 0, 0, 0, 0], [0.6, 0.0, 0.4, 0, 0, 0, 0]])
    t0 = time.perf_counter()
    with pytest.raises(PlanningError, match="torque limits cannot be met"):
        time_parametrize(path, m, method="topp")
    assert time.perf_counter() - t0 < 5.0


def test_speed_7dof_100_grid_under_1s():
    m = six_dof()
    dm = cb.build_dyn_model(m, cb.compile(m))
    t0 = time.perf_counter()
    sol, kind, _ = solve_topp(m, np.array([START6, MID6, GOAL6]), [1.0] * 7, [2.0] * 7, [40.0] * 7,
                              gravity=G6, n_grid=100, dm=dm)
    dt = time.perf_counter() - t0
    assert dt < 1.0, dt
    assert len(sol.q) == 101 and sol.duration > 0


# ------------------------------------------------------------------------- misc
def test_default_method_unchanged_and_bad_method():
    m = _heavy_2r(effort=1e4, vel=10.0)
    path = np.array([[0.0, 0.0], [1.0, 0.5]])
    a = time_parametrize(path, m)
    b = time_parametrize(path, m, method="scale")
    assert a.trajectory == b.trajectory and a.time_scale == b.time_scale
    with pytest.raises(ValueError):
        time_parametrize(path, m, method="nope")


def test_polyline_fallback_rests_at_waypoints():
    m = _heavy_2r(effort=1e4, vel=10.0)
    path = np.array([[0.0, 0.0], [1.0, 0.5], [1.5, -0.8]])
    sol, kind, _ = solve_topp(m, path, [10, 10], [50, 50], [300.0, 300.0], gravity=GY,
                              interpolation="linear", n_grid=100)
    assert kind == "linear"
    tr = solution_trajectory(sol)
    assert tr.duration == pytest.approx(sol.duration)
    # rest at the middle waypoint (junction speed zero)
    k = int(np.argmin(np.abs(sol.q[:, 0] - 1.0) + np.abs(sol.q[:, 1] - 0.5)))
    assert sol.x[k] == pytest.approx(0.0, abs=1e-12)


def test_planning_panel_has_time_optimal_entry(qapp):
    import os
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    from app.panels.planning_panel import PlanningPanel
    from app.task_runner import TaskRunner
    panel = PlanningPanel(TaskRunner())
    items = [panel._time_method.itemText(i) for i in range(panel._time_method.count())]
    assert "Time-optimal (torque limits)" in items
    assert panel.time_method() == "scale"
    panel.set_time_method("topp")
    assert panel.time_method() == "topp"
    assert "time_method" not in panel.options()
    panel.shutdown()


# ------------------------------------------------------------------------- U2.9 Hermite
def test_hermite_trajectory_exact_and_roundtrip():
    from app.sim.trajectory import Trajectory, Waypoint
    # constant acceleration 2: q = t^2, qd = 2t -> cubic Hermite is exact
    tr = Trajectory([Waypoint(t, (t * t,), (2 * t,)) for t in (0.0, 0.5, 1.5)], "cubic")
    for t in (0.0, 0.2, 0.5, 1.0, 1.5):
        assert tr.q(t)[0] == pytest.approx(t * t, abs=1e-12)
        assert tr.qd(t)[0] == pytest.approx(2 * t, abs=1e-12)
        assert tr.qdd(t)[0] == pytest.approx(2.0, abs=1e-9)
    d = tr.to_dict()
    assert d["waypoints"][1]["qd"] == [1.0]
    tr2 = Trajectory.from_dict(d)
    assert tr2 == tr and tr2.q(0.7) == pytest.approx(tr.q(0.7))
    # without velocities: old file format, no "qd" key, spline unchanged
    plain = Trajectory([Waypoint(0.0, (0.0,)), Waypoint(1.0, (1.0,))], "cubic")
    assert "qd" not in plain.to_dict()["waypoints"][0]
    # old reader style (ignores qd): the dict still has t and q
    assert set(d["waypoints"][0]) == {"t", "q", "qd"}
    with pytest.raises(ValueError):
        Trajectory([Waypoint(0.0, (0.0,), (0.0, 1.0)), Waypoint(1.0, (1.0,), (0.0, 1.0))], "cubic")


def test_hermite_solution_matches_solver_and_no_dense_violations():
    from app.planning.checks import check_trajectory
    m = _heavy_2r(effort=1e4, vel=10.0)
    path = np.array([[0.0, 0.0], [1.0, 0.5], [1.5, -0.8]])
    lim = {"effort": [200.0, 200.0], "vel": [10.0, 10.0], "accel": [50.0, 50.0]}
    sol, _, _ = solve_topp(m, path, [10, 10], [50, 50], [200.0, 200.0], gravity=GY, n_grid=100)
    tp = time_parametrize(path, m, limits=lim, vmax=10.0, amax=50.0, gravity=GY, method="topp",
                          dt=1e-3)
    assert tp.trajectory.duration <= sol.duration * 1.03
    assert tp.trajectory.waypoints[1].qd is not None
    rep = check_trajectory(tp.trajectory, m, {**lim, "effort": np.array([200.0, 200.0])}, dt=1e-3,
                           check_torque=True, gravity=GY)
    assert not rep.violations, rep.summary()
    # old spline form (no velocities) is still available and slower after the rescale loop
    old = solution_trajectory(sol, hermite=False)
    assert old.waypoints[1].qd is None


def test_six_dof_duration_close_to_solver():
    m = six_dof()
    path = np.array([START6, MID6, GOAL6])
    tp = time_parametrize(path, m, limits={"effort": 40.0}, method="topp", dt=1e-3)
    assert tp.report.ok, tp.report.summary()
    note = [n for n in tp.notes if "solver duration" in n][0]
    solver = float(note.split("solver duration ")[1].split(" s")[0])
    assert tp.trajectory.duration <= solver * 1.03
