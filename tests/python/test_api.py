"""R8.1: robotics_api vs independent oracles (numpy_ref, closed form, own numpy collision check).

Every API function is compared with a result computed WITHOUT the code under test: numpy FK /
numeric Jacobian (``numpy_ref``), closed-form planar 2R kinematics and dynamics, an own RK4 on the
closed-form 2R equations, an own segment-vs-rectangle check of a planned path, an own re-check of
the path with a fresh ``CollisionWorld`` on a dense grid.
"""
import math
import re
import runpy
from pathlib import Path

import numpy as np
import pytest

import core_bridge as cb

import robotics_api as api
from robotics_api import (ComputationError, ExportError, ImportFailed, InvalidArgument,
                          PlanningError, ProjectError, Robot, RoboticsApiError, SimulationError)

from .closed_form import fk_2r, ik_2r, wrap
from .numpy_ref import fk_numpy, numeric_jacobian_tcp, spec_from_model
from .robots import pose, random_q, random_robot
from .test_cartesian_traj import planar_2r

FX = Path(__file__).parent / "fixtures" / "urdf"
EXAMPLES = Path(__file__).parent.parent.parent / "examples"
G_Y = (0.0, -9.81, 0.0)          # gravity inside the plane of the planar robots
L1, L2 = 1.0, 0.5                # planar_2r(): link lengths
M1, R1 = 1.3, 0.6                # point mass 1 at (R1, 0, 0) of link 1
M2, R2 = 0.7, 0.3                # point mass 2 at (R2, 0, 0) of link 2


def mass_robot():
    """Planar 2R (z axes) with two POINT masses: closed-form dynamics exist."""
    m = planar_2r(L1, L2)
    for idx, mass, r in ((1, M1, R1), (2, M2, R2)):
        i = cb.Inertia()
        i.mass = mass
        i.com = [r, 0.0, 0.0]
        i.inertia_tensor = [0.0] * 6
        m.links[idx].inertia = i
    return Robot(m, "mass2r")


def cf_dynamics(q, qd, g=9.81):
    """Closed-form M, C(q, qd) and G(q) of the two point-mass planar arm (gravity -y)."""
    c2, s2 = math.cos(q[1]), math.sin(q[1])
    M = np.array([[M1 * R1**2 + M2 * (L1**2 + R2**2 + 2 * L1 * R2 * c2), M2 * (R2**2 + L1 * R2 * c2)],
                  [M2 * (R2**2 + L1 * R2 * c2), M2 * R2**2]])
    h = M2 * L1 * R2 * s2
    C = np.array([-h * (2 * qd[0] * qd[1] + qd[1] ** 2), h * qd[0] ** 2])
    G = np.array([(M1 * R1 + M2 * L1) * g * math.cos(q[0]) + M2 * R2 * g * math.cos(q[0] + q[1]),
                  M2 * R2 * g * math.cos(q[0] + q[1])])
    return M, C, G


def cf_energy(q, qd, g=9.81):
    M, _, _ = cf_dynamics(q, qd)
    p1y = R1 * math.sin(q[0])
    p2y = L1 * math.sin(q[0]) + R2 * math.sin(q[0] + q[1])
    return 0.5 * qd @ M @ qd, g * (M1 * p1y + M2 * p2y)


def ref_rk4(q0, qd0, dur, dt, g=9.81, tau=(0.0, 0.0)):
    """Own RK4 on the closed-form equations (independent of the engine)."""
    tau = np.asarray(tau, float)

    def acc(q, qd):
        M, C, G = cf_dynamics(q, qd, g)
        return np.linalg.solve(M, tau - C - G)

    q, qd = np.array(q0, float), np.array(qd0, float)
    for _ in range(int(round(dur / dt))):
        k1v = acc(q, qd)
        k2q = qd + 0.5 * dt * k1v
        k2v = acc(q + 0.5 * dt * qd, k2q)
        k3q = qd + 0.5 * dt * k2v
        k3v = acc(q + 0.5 * dt * k2q, k3q)
        k4q = qd + dt * k3v
        k4v = acc(q + dt * k3q, k4q)
        q = q + dt / 6 * (qd + 2 * k2q + 2 * k3q + k4q)
        qd = qd + dt / 6 * (k1v + 2 * k2v + 2 * k3v + k4v)
    return q, qd


def collision_robot():
    r = Robot(planar_2r(), "planar")
    r.set_collision_shapes([
        {"type": "sphere", "link": 0, "pose": [0.75, 0, 0], "size": 0.1},
        {"type": "box", "link": 1, "pose": [0.5, 0, 0], "size": [1.0, 0.1, 0.1]},
        {"type": "box", "link": 2, "pose": [0.25, 0, 0], "size": [0.5, 0.1, 0.1]},
    ])
    return r


OBST = [{"type": "box", "pose": {"xyz": [0.0, 1.2, 0.0]}, "size": [0.3, 0.3, 0.4]}]


def seg_hits_rect(p, q, lo, hi):
    """Slab test: does the segment p-q touch the axis-aligned rectangle [lo, hi]?"""
    t0, t1 = 0.0, 1.0
    d = q - p
    for a in range(2):
        if abs(d[a]) < 1e-14:
            if p[a] < lo[a] or p[a] > hi[a]:
                return False
        else:
            ta, tb = (lo[a] - p[a]) / d[a], (hi[a] - p[a]) / d[a]
            t0, t1 = max(t0, min(ta, tb)), min(t1, max(ta, tb))
            if t0 > t1:
                return False
    return True


# ------------------------------------------------------------------------------ version
def test_api_version_string():
    assert api.__api_version__ == "1.2"
    assert re.fullmatch(r"\d+\.\d+", api.__api_version__)
    assert "Versioning" in api.__doc__ and "MINOR" in api.__doc__


# ------------------------------------------------------------------------- kinematics
@pytest.mark.parametrize("seed", range(4))
def test_fk_jacobian_vs_numpy_ref(seed):
    model = random_robot(seed, 5, kinds=("dh", "origin_axis"), prismatic=0.2)
    robot = Robot(model)
    spec = spec_from_model(model)
    rng = np.random.default_rng(seed)
    assert robot.n_dof == spec["n_dof"] == 5
    for _ in range(10):
        q = random_q(model, rng)
        assert np.allclose(robot.fk(q), fk_numpy(spec, q)[1], atol=1e-10)
        assert np.allclose(robot.jacobian(q), numeric_jacobian_tcp(spec, q), atol=1e-6)


def test_facts_names_limits_types():
    model = random_robot(3, 4, prismatic=0.5)
    r = Robot(model)
    assert r.joint_names == [f"j{i}" for i in range(4)]
    assert r.limits.shape == (4, 2)
    assert np.all(r.limits[:, 0] < r.limits[:, 1])
    assert set(r.joint_types) <= {"revolute", "prismatic"}
    for t, lim in zip(r.joint_types, r.limits):
        assert (t == "prismatic") == (abs(lim[1]) == 0.5)


def test_ik_vs_closed_form_both_elbows():
    r = Robot(planar_2r(L1, L2))      # tool at x = L2 on link 2: tcp = 2R chain with lengths L1, L2
    target = fk_2r([0.7, -0.9], L1, L2)
    T = np.eye(4)
    T[0, 3], T[1, 3] = target
    sols = r.ik(T, seed=[0.5, 0.5], all_solutions=False, position_only=True)
    allsol = r.ik(T, seed=[0.5, 0.5], all_solutions=True, position_only=True)
    ref = ik_2r(target[0], target[1], L1, L2)
    assert len(allsol) == len(ref) == 2
    for s in allsol:                                    # each closed-form solution is returned
        assert min(np.linalg.norm(wrap(s - np.array(c))) for c in ref) < 1e-4
    for s in allsol:                                    # and reproduces the pose (numpy FK)
        assert np.allclose(fk_2r(s, L1, L2), target, atol=1e-6)
    d = [np.linalg.norm(s - [0.5, 0.5]) for s in allsol]
    assert np.allclose(sols, allsol[int(np.argmin(d))])          # nearest to the seed first


def test_ik_full_pose_and_unreachable():
    r = Robot(planar_2r(L1, L2))
    q = np.array([0.4, 0.6])
    sol = r.ik(r.fk(q), seed=[0.3, 0.5])
    assert np.allclose(fk_numpy(spec_from_model(r._model), sol)[1], r.fk(q), atol=1e-6)
    far = np.eye(4)
    far[0, 3] = 5.0
    assert r.ik(far) is None
    assert r.ik(far, all_solutions=True) == []


def test_manipulability_vs_numpy_svd():
    model = random_robot(5, 4)
    r = Robot(model)
    spec = spec_from_model(model)
    rng = np.random.default_rng(1)
    for _ in range(5):
        q = random_q(model, rng)
        J = numeric_jacobian_tcp(spec, q)
        ref = float(np.prod(np.linalg.svd(J, compute_uv=False)))      # Yoshikawa, 4 columns
        assert r.manipulability(q) == pytest.approx(ref, rel=1e-5, abs=1e-9)


# --------------------------------------------------------------------------- dynamics
def test_2r_inverse_forward_mass_energy_closed_form():
    r = mass_robot()
    rng = np.random.default_rng(3)
    for _ in range(8):
        q, qd, qdd = rng.uniform(-2, 2, 2), rng.uniform(-2, 2, 2), rng.uniform(-3, 3, 2)
        M, C, G = cf_dynamics(q, qd)
        assert np.allclose(r.mass_matrix(q), M, atol=1e-10)
        tau = r.inverse_dynamics(q, qd, qdd, gravity=G_Y)
        assert np.allclose(tau, M @ qdd + C + G, atol=1e-9)
        assert np.allclose(r.forward_dynamics(q, qd, tau, gravity=G_Y), qdd, atol=1e-9)
        ke, pe = cf_energy(q, qd)
        e = r.energy(q, qd, gravity=G_Y)
        assert e["ke"] == pytest.approx(ke, rel=1e-10, abs=1e-12)
        assert e["pe"] == pytest.approx(pe, rel=1e-10, abs=1e-12)
        assert e["total"] == pytest.approx(ke + pe, rel=1e-10)


def test_default_gravity_is_minus_z():
    r = mass_robot()
    z = [0.0, 0.0]
    # planar arm with z axes: gravity along -z gives no joint torque
    assert np.allclose(r.inverse_dynamics([0.3, 0.4], z, z), 0.0, atol=1e-12)
    assert np.linalg.norm(r.inverse_dynamics([0.3, 0.4], z, z, gravity=G_Y)) > 1.0


# ---------------------------------------------------------------------------- simulate
def test_simulate_vs_own_rk4_on_closed_form():
    r = mass_robot()
    q0, qd0 = [0.5, -0.3], [0.2, 0.4]
    res = api.simulate(r, None, 0.5, 1e-3, "rk4", q0=q0, qd0=qd0, gravity=G_Y)
    assert res["t"].shape == (501,) and res["t"][0] == 0.0 and res["t"][-1] == pytest.approx(0.5)
    q_ref, qd_ref = ref_rk4(q0, qd0, 0.5, 1e-3)
    assert np.allclose(res["q"][-1], q_ref, atol=1e-9)
    assert np.allclose(res["qd"][-1], qd_ref, atol=1e-9)
    assert np.allclose(res["q"][0], q0) and np.allclose(res["qd"][0], qd0)
    for k in ("q", "qd", "qdd", "tau", "tcp_pos", "tcp_quat", "ke", "pe", "total", "power_sum"):
        assert len(res[k]) == 501
    ke, pe = zip(*(cf_energy(q, v) for q, v in zip(res["q"][::100], res["qd"][::100])))
    assert np.allclose(res["ke"][::100, 0], ke, atol=1e-9)
    assert np.allclose(res["total"][:, 0], res["total"][0, 0], atol=1e-6)      # energy kept
    # tcp from numpy FK of the recorded q
    assert np.allclose(res["tcp_pos"][-1][:2], fk_2r(res["q"][-1], L1, L2), atol=1e-9)


def test_simulate_constant_and_callable_torque():
    r = mass_robot()
    tau = np.array([0.8, -0.2])
    a = api.simulate(r, tau, 0.3, 1e-3, gravity=G_Y)
    b = api.simulate(r, lambda t, q, qd: tau, 0.3, 1e-3, gravity=G_Y)
    q_ref, _ = ref_rk4([0, 0], [0, 0], 0.3, 1e-3, tau=tau)
    assert np.allclose(a["q"][-1], q_ref, atol=1e-9)
    assert np.allclose(a["q"], b["q"], atol=1e-12)
    assert np.allclose(a["tau"][5], tau)


@pytest.mark.parametrize("integ", ["euler", "rk45"])
def test_simulate_other_integrators_close_to_rk4(integ):
    r = mass_robot()
    ref, _ = ref_rk4([0.5, -0.3], [0, 0], 0.3, 1e-4)
    res = api.simulate(r, None, 0.3, 1e-3 if integ == "rk45" else 1e-4, integ, q0=[0.5, -0.3],
                       gravity=G_Y)
    assert res["t"][-1] == pytest.approx(0.3)
    assert np.allclose(res["q"][-1], ref, atol=1e-2 if integ == "euler" else 1e-6)


def test_simulate_record_every_and_trajectory_tracking():
    r = mass_robot()
    traj = api.Trajectory([0.0, 1.0, 2.0], [[0, 0], [0.5, -0.4], [0.2, 0.1]], "cubic")
    res = api.simulate(r, traj, dt=1e-3, gravity=G_Y, record_every=10)
    assert res["t"][-1] == pytest.approx(2.0) and len(res["t"]) == 201
    ref = np.array([traj.q(t) for t in res["t"]])
    assert np.abs(res["q"] - ref).max() < 1e-4                # computed torque, exact model


def test_trajectory_matches_waypoints_and_sampling():
    w = np.array([[0, 0], [1, -1], [0.5, 0.5]])
    for kind in ("linear", "cubic"):
        t = api.Trajectory([0, 1, 3], w, kind)
        assert t.duration == 3.0 and t.n_dof == 2 and t.interpolation == kind
        assert np.allclose([t.q(x) for x in (0, 1, 3)], w)
        assert np.allclose(t.waypoints, w) and np.allclose(t.times, [0, 1, 3])
        s = t.sample(0.5)
        assert s["t"][-1] == 3.0 and s["q"].shape == (7, 2)
    assert np.allclose(api.Trajectory([0, 1], [[0, 0], [2, 4]], "linear").q(0.25), [0.5, 1.0])


def test_time_parametrize_respects_limits():
    r = mass_robot()
    path = np.array([[0, 0], [1.0, -0.5], [0.4, 0.8]])
    tr = api.time_parametrize(path, r, vmax=1.0, amax=2.0, interpolation="linear")
    assert tr.ok and "OK" in tr.report
    assert np.allclose(tr.waypoints, path)
    s = tr.sample(0.01)
    assert np.abs(s["qd"]).max() <= 1.0 + 1e-6          # own check of the velocity limit
    assert tr.duration > 1.0 / 1.0


# ---------------------------------------------------------------------------------- plan
def test_plan_path_is_valid_and_avoids_box():
    from app.planning.collision import CollisionConfig, CollisionWorld, EnvShape
    r = collision_robot()
    start, goal = [0.0, 0.0], [2.2, 0.3]
    path = api.plan(r, start, goal, obstacles=OBST, seed=0)
    assert path.ndim == 2 and path.shape[1] == 2
    assert np.allclose(path[0], start) and np.allclose(path[-1], goal)
    lim = r.limits
    assert np.all(path >= lim[:, 0] - 1e-9) and np.all(path <= lim[:, 1] + 1e-9)
    # (a) own numpy centreline vs rectangle test along a dense interpolation
    lo, hi = np.array([-0.15, 1.05]), np.array([0.15, 1.35])
    for a, b in zip(path[:-1], path[1:]):
        for s in np.linspace(0, 1, int(np.linalg.norm(b - a) / 1e-3) + 2):
            q = a + s * (b - a)
            p1 = np.array([math.cos(q[0]), math.sin(q[0])]) * L1
            p2 = p1 + L2 * np.array([math.cos(q[0] + q[1]), math.sin(q[0] + q[1])])
            assert not seg_hits_rect(np.zeros(2), p1, lo, hi)
            assert not seg_hits_rect(p1, p2, lo, hi)
    # (b) fresh collision world on a dense grid
    world = CollisionWorld(r._model, [__import__("app.planning.collision", fromlist=["x"]).Shape(
        d["type"], {"radius": d["radius"]} if d["type"] == "sphere" else {"size": d["size"]},
        d["link"], d["pose"]) for d in r.collision_shapes],
        CollisionConfig(environment=[EnvShape("box", {"size": [0.3, 0.3, 0.4]},
                                               xyz=[0.0, 1.2, 0.0])]))
    dense = np.vstack([a + np.linspace(0, 1, int(np.linalg.norm(b - a) / 1e-3) + 2)[:, None] * (b - a)
                       for a, b in zip(path[:-1], path[1:])])
    assert not world.check_batch(dense).any()
    # a straight line start-goal would hit the obstacle: the path is not the straight line
    assert len(path) > 2


def test_plan_same_seed_same_path_and_pose_goal():
    r = collision_robot()
    p1 = api.plan(r, [0, 0], [2.2, 0.3], obstacles=OBST, seed=3, planner="rrt_connect")
    p2 = api.plan(r, [0, 0], [2.2, 0.3], obstacles=OBST, seed=3, planner="rrt_connect")
    assert np.array_equal(p1, p2)
    goal_q = np.array([1.0, 0.5])
    p3 = api.plan(r, [0, 0], r.fk(goal_q), obstacles=None, seed=0)
    assert np.allclose(fk_numpy(spec_from_model(r._model), p3[-1])[1], r.fk(goal_q), atol=1e-5)


@pytest.mark.parametrize("planner", ["rrt_star", "prm"])
def test_plan_other_planners(planner):
    r = collision_robot()
    path = api.plan(r, [0, 0], [2.2, 0.3], obstacles=OBST, planner=planner, seed=1, time_limit=20)
    assert np.allclose(path[-1], [2.2, 0.3]) and len(path) >= 2


def test_plan_failure_is_planning_error():
    r = collision_robot()
    with pytest.raises(PlanningError):                      # goal inside the obstacle
        api.plan(r, [0, 0], [math.pi / 2 - 0.1, 0.0],
                 obstacles=[{"type": "box", "pose": [0, 1.0, 0], "size": [0.6, 0.6, 0.4]}])


# --------------------------------------------------------------------------- workspace
def test_workspace_annulus_volume_and_cancel():
    r = Robot(planar_2r(L1, L2))
    h = 0.05
    res = api.workspace(r, n_samples=200_000, voxel=h, seed=1)
    area = math.pi * ((L1 + L2) ** 2 - (L1 - L2) ** 2)            # reachable annulus
    assert res["voxel_size"] == h
    assert res["volume"] == pytest.approx(area * h, rel=0.15)     # one voxel layer thick
    rad = np.hypot(res["centers"][:, 0], res["centers"][:, 1])
    assert rad.min() >= (L1 - L2) - h and rad.max() <= (L1 + L2) + h
    assert res["centers"].shape == (res["filled"], 3)
    tok = api.CancelToken()
    tok.cancel()
    with api.cancel_scope(tok), pytest.raises(api.ScriptCancelled):
        api.workspace(r, n_samples=10**9)


# ------------------------------------------------------------------------ cancel token
def test_check_cancel_noop_outside_runner_and_raises_inside_scope():
    api.check_cancel()
    tok = api.CancelToken(time_limit=0.05)
    import time as _t
    with api.cancel_scope(tok):
        api.check_cancel()
        _t.sleep(0.08)
        with pytest.raises(api.ScriptTimeLimit):
            api.check_cancel()
    api.check_cancel()                                  # scope restored
    tok2 = api.CancelToken()
    with api.cancel_scope(tok2):
        tok2.cancel()
        with pytest.raises(api.ScriptCancelled):
            api.simulate(mass_robot(), None, 1.0, 1e-3)


# ------------------------------------------------------------------------ project / URDF
def test_project_round_trip(tmp_path):
    robot = api.import_urdf(str(FX / "planar_2r.urdf"))
    proj = api.new_project("api test")
    assert proj.path is None and proj.name == "api test"
    proj.robot = robot
    f = api.save_project(proj, str(tmp_path / "p.robotproj"))
    assert proj.path == f
    back = api.open_project(f)
    assert back.name == "api test" and back.robot.n_dof == 2
    q = [0.3, -0.7]
    assert np.allclose(back.robot.fk(q), robot.fk(q), atol=1e-12)
    assert np.allclose(robot.fk(q)[:2, 3], fk_2r(q, 1.0, 1.0), atol=1e-6)    # URDF 1 m + 1 m tip


def test_import_urdf_report_and_robot_copy():
    robot = api.import_urdf(str(FX / "serial_6dof.urdf"))
    assert robot.n_dof == 6 and isinstance(robot.import_report, str)
    c = robot.copy()
    q = np.linspace(-0.5, 0.5, 6)
    assert np.allclose(c.fk(q), robot.fk(q))
    assert Robot.from_json(robot.to_json()).n_dof == 6


# --------------------------------------------------------------------------------- export
def test_export_csv_npz_round_trip(tmp_path):
    from app.sim.export import load_csv, load_npz
    r = mass_robot()
    res = api.simulate(r, None, 0.1, 1e-3, q0=[0.3, 0.2], gravity=G_Y)
    n = api.export_csv(res, str(tmp_path / "a.csv"), names=["q", "tcp_pos"], meta={"robot": "x"})
    assert n == 101
    arrays, meta = load_csv(str(tmp_path / "a.csv"))
    assert set(arrays) == {"q", "tcp_pos"} and meta["robot"] == "x"
    assert np.array_equal(arrays["q"][1], res["q"]) and np.array_equal(arrays["q"][0], res["t"])
    api.export_npz(res, str(tmp_path / "a.npz"))
    arrays, meta = load_npz(str(tmp_path / "a.npz"))
    assert np.array_equal(arrays["qdd"][1], res["qdd"]) and "app version" in meta
    assert open(tmp_path / "a.csv").read().count("\n") >= 104


# ------------------------------------------------------------------------ error mapping
def test_error_hierarchy():
    for cls in (InvalidArgument, ProjectError, ImportFailed, ComputationError, SimulationError,
                PlanningError, ExportError, api.AnalysisError, api.ScriptCancelled):
        assert issubclass(cls, RoboticsApiError)
    assert issubclass(api.ScriptTimeLimit, api.ScriptCancelled)
    assert issubclass(InvalidArgument, ValueError)


def test_bad_file_errors(tmp_path):
    with pytest.raises(ProjectError):
        api.open_project(str(tmp_path / "missing.robotproj"))
    bad = tmp_path / "bad.robotproj"
    bad.write_bytes(b"not a zip")
    with pytest.raises(ProjectError):
        api.open_project(str(bad))
    with pytest.raises(ImportFailed) as e:
        api.import_urdf(str(tmp_path / "missing.urdf"))
    assert "error" in e.value.report.lower()
    with pytest.raises(ImportFailed):
        api.import_urdf(str(FX / "bad_xml.urdf"))
    proj = api.new_project("x")
    with pytest.raises(ProjectError):
        api.save_project(proj, str(tmp_path / "wrong.ext"))
    with pytest.raises(ProjectError):
        api.save_project(proj, str(tmp_path / "no_dir" / "p.robotproj"))


def test_wrong_n_dof_and_bad_values_raise_api_errors():
    r = mass_robot()
    calls = [
        lambda: r.fk([0.0]), lambda: r.fk([0.0, 0.0, 0.0]), lambda: r.fk([0.0, float("nan")]),
        lambda: r.fk("abc"), lambda: r.jacobian([1.0]),
        lambda: r.inverse_dynamics([0, 0], [0, 0], [0]),
        lambda: r.forward_dynamics([0, 0], [0], [0, 0]),
        lambda: r.mass_matrix([0] * 3), lambda: r.energy([0, 0], [0, 0, 0]),
        lambda: r.manipulability([]), lambda: r.ik(np.eye(3)), lambda: r.ik(np.eye(4), seed=[1]),
        lambda: r.ik(np.eye(4), n_starts=0), lambda: r.inverse_dynamics([0, 0], [0, 0], [0, 0], gravity=[1]),
        lambda: api.simulate(r, None, 1.0, 0.0), lambda: api.simulate(r, None),
        lambda: api.simulate(r, None, 1.0, 1e-3, integrator="bogus"),
        lambda: api.simulate(r, [1.0], 1.0), lambda: api.simulate(r, None, 1.0, q0=[0]),
        lambda: api.simulate(r, lambda t, q, qd: [1.0], 0.1),
        lambda: api.simulate(r, api.Trajectory([0, 1], [[0, 0, 0], [1, 1, 1]]), 0.1),
        lambda: api.plan(r, [0], [0, 0]), lambda: api.plan(r, [0, 0], [0, 0, 0]),
        lambda: api.plan(r, [0, 0], [0, 0], planner="bogus"),
        lambda: api.plan(r, [0, 0], [1, 1], obstacles=[{"type": "box", "size": [1, 1, 1]}]),
        lambda: api.plan(collision_robot(), [0, 0], [1, 1], obstacles=[{"type": "torus"}]),
        lambda: api.plan(collision_robot(), [0, 0], [1, 1], obstacles=[{"type": "box"}]),
        lambda: api.time_parametrize([[0, 0]], r), lambda: api.time_parametrize([[0], [1]], r),
        lambda: api.time_parametrize([[0, 0], [1, 1]], r, vmax=-1),
        lambda: api.Trajectory([0, 0], [[0, 0], [1, 1]]),
        lambda: api.Trajectory([1, 2], [[0, 0], [1, 1]]),
        lambda: api.Trajectory([0, 1], [[0, 0], [1, 1]], "spline"),
        lambda: api.workspace(r, n_samples=0), lambda: api.workspace(r, voxel=-1.0),
        lambda: api.export_csv({"q": 1}, "x.csv"), lambda: api.export_csv({"t": [0.0]}, "x.csv"),
        lambda: api.export_npz({"t": [0.0], "q": [[1.0], [2.0]]}, "x.npz"),
        lambda: r.set_collision_shapes([{"type": "box", "link": 99, "size": [1, 1, 1]}]),
        lambda: Robot.from_json("not json"),
        lambda: api.save_project("not a project", "x.robotproj"),
    ]
    for i, call in enumerate(calls):
        with pytest.raises(RoboticsApiError) as e:
            call()
        assert type(e.value).__module__.startswith("robotics_api"), (i, type(e.value))


def test_no_raw_bridge_exception_from_empty_robot(tmp_path):
    r = Robot(cb.RobotModel(), "empty")
    assert r.n_dof == 0
    try:                       # a 0-DOF robot may or may not compute; if not, only API errors
        r.jacobian([])
        api.simulate(r, None, 0.1)
    except Exception as exc:  # noqa: BLE001
        assert isinstance(exc, RoboticsApiError), type(exc)
    with pytest.raises(RoboticsApiError):
        api.export_csv({"t": np.zeros(2), "q": np.zeros((2, 1))}, str(tmp_path / "no" / "x.csv"))


def test_singular_dynamics_maps_to_simulation_error():
    m = planar_2r()                       # no inertia: massless links
    with pytest.raises(RoboticsApiError):
        api.simulate(Robot(m), None, 0.1, 1e-3)


# ---------------------------------------------------------------------------- examples
@pytest.mark.parametrize("name", ["fk_sweep", "plan_and_simulate", "export_data"])
def test_examples_run_headless(name, capsys):
    ns = runpy.run_path(str(EXAMPLES / f"{name}.py"), run_name="__main__")
    out = capsys.readouterr().out
    assert out.strip()
    assert "robot" in ns
    if name == "plan_and_simulate":
        assert "max tracking error" in out
