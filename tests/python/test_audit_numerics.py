"""A1.6: numerics, planning and import fixes from the 2026-11-13 audit (M6, M7, LOW numerics)."""
import math
import threading
import time

import numpy as np
import pytest

import core_bridge as cb

from app.analysis.workspace_analysis import analyze_workspace
from app.compute.backend import estimate_reach
from app.importers import import_urdf
from app.kin_utils import dof_limits
from app.importers import import_report as ir
from app.planning import check_trajectory
from app.planning.cartesian import line
from app.planning.planner import PLANNERS, _limits_arrays, plan
from app.project.project_data import SimulationSettings
from app.sim import DYNAMIC, KINEMATIC, SimEngine
from app.sim import mode as sim_mode
from app.sim.playback import TrajectoryPlayer
from app.sim.trajectory import Trajectory, Waypoint

from .numpy_ref import fk_numpy, spec_from_model
from .robots import pose, random_inertias, random_robot
from .test_cartesian_traj import planar_2r


# ------------------------------------------------------------------ helpers
def gantry(stroke=1.0):
    """One prismatic joint along x, stroke ``stroke`` m, no other geometry."""
    m = cb.RobotModel()
    for i in range(2):
        lk = cb.Link()
        lk.name = f"l{i}"
        m.links.append(lk)
    j = cb.Joint()
    j.name = "slide"
    j.parent_link, j.child_link = 0, 1
    j.type = cb.JointType.Prismatic
    j.kinematics = cb.JointKinematics.OriginAxis
    j.origin = pose([0, 0, 0], [0, 0, 0])
    j.axis = [1.0, 0.0, 0.0]
    lim = cb.JointLimits()
    lim.lower, lim.upper = 0.0, stroke
    j.limits = lim
    m.joints.append(j)
    m.tool = pose([0, 0, 0], [0, 0, 0])
    m.tcp_link = 1
    return m


def _engine(integrator=None, dt=0.002):
    model = random_robot(3, 2)
    random_inertias(model, np.random.default_rng(5))
    return SimEngine(model, SimulationSettings(physics_timestep=dt, gravity=[0.0, 0.0, -9.81]),
                     integrator=integrator)


def _write_urdf(tmp_path, joint_xml):
    p = tmp_path / "r.urdf"
    p.write_text(
        '<robot name="r"><link name="a"/><link name="b"/>'
        f"{joint_xml}</robot>")
    return p


# ------------------------------------------------------------------ M6
def test_m6_estimate_reach_adds_prismatic_stroke():
    """Audit M6: estimate_reach includes max(|lo|,|hi|) of prismatic joints."""
    m = gantry(1.0)
    assert estimate_reach(m) == pytest.approx(1.0)
    assert estimate_reach(cb.compile_arrays(m), limits=dof_limits(m)) == pytest.approx(1.0)
    # asymmetric stroke: the larger end counts
    m = gantry(1.0)
    lim = m.joints[0].limits
    lim.lower, lim.upper = -2.0, 0.5
    m.joints[0].limits = lim
    assert estimate_reach(m) == pytest.approx(2.0)


def test_m6_gantry_workspace_nothing_outside():
    """Audit M6: gantry workspace is not truncated (n_outside == 0), volume within bound."""
    m = gantry(1.0)
    res = analyze_workspace(m, {"n_samples": 20000, "seed": 1, "want_metrics": False},
                            cb.CancelToken(), None)
    assert res["n_outside"] == 0
    h = float(res["voxel_size"]) if "voxel_size" in res else estimate_reach(m) / 100.0
    # a 1 m line is covered by ~1/h voxels along x
    assert res["filled"] >= 0.9 / h
    assert abs(res["volume"] - res["filled"] * h ** 3) <= res["volume_err_bound"] + 1e-12


# ------------------------------------------------------------------ M7
class _Blocked:
    """Fake collision: everything is in collision except the start and the goal."""

    def __init__(self, *free):
        self.free = [np.asarray(f, float) for f in free]

    def check_batch(self, qs):
        qs = np.asarray(qs, float)
        out = np.ones(len(qs), dtype=bool)
        for f in self.free:
            out &= np.linalg.norm(qs - f, axis=1) > 1e-9
        return out


def test_m7_prm_cancel_in_blocked_space():
    """Audit M7: PRM sample loop honours cancel within 0.5 s in a fully blocked space."""
    m = planar_2r()
    evt = threading.Event()
    threading.Timer(0.1, evt.set).start()
    t0 = time.perf_counter()
    res = plan(m, [0.0, 0.0], [1.0, 1.0], _Blocked([0.0, 0.0], [1.0, 1.0]), planner="prm",
               time_limit=30, seed=0, cancel=evt)
    assert time.perf_counter() - t0 < 0.5
    assert not res.success and res.reason == "cancelled"


def test_m7_prm_time_limit_in_blocked_space():
    """Audit M7: PRM sample loop honours the time limit."""
    m = planar_2r()
    t0 = time.perf_counter()
    res = plan(m, [0.0, 0.0], [1.0, 1.0], _Blocked([0.0, 0.0], [1.0, 1.0]), planner="prm",
               time_limit=0.2, seed=0)
    assert time.perf_counter() - t0 < 0.7
    assert not res.success and res.reason == "time limit reached"


# ------------------------------------------------------------------ LOW: limits rule
def _locked(value=0.5):
    m = planar_2r()
    lim = m.joints[1].limits
    lim.lower = lim.upper = value
    m.joints[1].limits = lim
    return m


def test_low_locked_joint_is_not_unlimited_in_planner():
    """Audit LOW (lo == hi): a locked revolute joint stays at its value, not +-pi."""
    m = _locked(0.5)
    lo, hi = _limits_arrays(m, None)
    assert lo[1] == hi[1] == 0.5
    assert (lo[0], hi[0]) == (-math.pi, math.pi)
    rng = np.random.default_rng(0)
    # the planner samples the locked DOF at its value
    res = plan(m, [0.0, 0.5], [1.0, 0.5], None)
    assert res.success and np.all(np.abs(res.path[:, 1] - 0.5) < 1e-9)
    # start / goal must equal the locked value (tolerance 1e-9)
    bad = plan(m, [0.0, 0.3], [1.0, 0.5], None)
    assert not bad.success and "start outside" in bad.reason
    bad = plan(m, [0.0, 0.5], [1.0, 0.3], None)
    assert not bad.success and "goal outside" in bad.reason
    ok = plan(m, [0.0, 0.5 + 5e-10], [1.0, 0.5], None)
    assert ok.success
    del rng


def test_low_locked_joint_position_check():
    """Audit LOW (lo == hi): check_trajectory flags a locked joint that moves."""
    m = _locked(0.5)
    t = np.linspace(0, 1, 11)
    q = np.stack([t * 0.1, np.full_like(t, 0.5)], 1)
    z = np.zeros_like(q)
    assert check_trajectory((t, q, z, z), m, check_torque=False).ok
    q2 = q.copy()
    q2[5:, 1] = 0.6
    rep = check_trajectory((t, q2, z, z), m, check_torque=False)
    assert rep.of_kind("position") and rep.of_kind("position")[0].dof == 1


def test_low_urdf_limit_without_bounds_warns(tmp_path):
    """Audit LOW: revolute <limit> without lower/upper -> URDF_LIMIT_MISSING warning."""
    p = _write_urdf(tmp_path,
                    '<joint name="j" type="revolute"><parent link="a"/><child link="b"/>'
                    '<limit effort="1" velocity="1"/></joint>')
    res = import_urdf(str(p))
    codes = [i.code for i in res.report.issues]
    assert ir.URDF_LIMIT_MISSING in codes
    w = [i for i in res.report.issues if i.code == ir.URDF_LIMIT_MISSING][0]
    assert w.severity == ir.SEV_WARNING and w.element == "j"
    # with both bounds: no warning
    p2 = _write_urdf(tmp_path,
                     '<joint name="j" type="revolute"><parent link="a"/><child link="b"/>'
                     '<limit lower="-1" upper="1" effort="1" velocity="1"/></joint>')
    assert ir.URDF_LIMIT_MISSING not in [i.code for i in import_urdf(str(p2)).report.issues]


# ------------------------------------------------------------------ LOW: URDF non finite
@pytest.mark.parametrize("joint", [
    '<joint name="j" type="revolute"><parent link="a"/><child link="b"/>'
    '<axis xyz="nan 0 0"/></joint>',
    '<joint name="j" type="revolute"><parent link="a"/><child link="b"/>'
    '<axis xyz="inf 0 0"/></joint>',
    '<joint name="j" type="revolute"><parent link="a"/><child link="b"/>'
    '<origin xyz="0 nan 0"/></joint>',
    '<joint name="j" type="revolute"><parent link="a"/><child link="b"/>'
    '<limit lower="-inf" upper="1" effort="1" velocity="1"/></joint>',
])
def test_low_urdf_non_finite_is_error(tmp_path, joint):
    """Audit LOW: nan/inf in the URDF -> error, model not built, no crash."""
    res = import_urdf(str(_write_urdf(tmp_path, joint)))
    assert res.model is None
    errs = [i for i in res.report.issues if i.severity == ir.SEV_ERROR]
    assert errs and errs[0].code in (ir.URDF_NON_FINITE, ir.URDF_ZERO_AXIS)
    assert errs[0].code == ir.URDF_NON_FINITE
    assert errs[0].element == "j"


# ------------------------------------------------------------------ LOW: checks
def test_low_check_trajectory_flags_non_finite():
    """Audit LOW: NaN in q/qd/qdd gives a non_finite violation with the sample range."""
    m = planar_2r()
    t = np.linspace(0, 1, 21)
    q = np.stack([0.1 * t, 0.2 * t], 1)
    qd = np.full_like(q, 0.1)
    qdd = np.zeros_like(q)
    assert check_trajectory((t, q, qd, qdd), m).ok
    q2 = q.copy()
    q2[5:8, 0] = np.nan
    qdd2 = qdd.copy()
    qdd2[15, 1] = np.inf
    rep = check_trajectory((t, q2, qd, qdd2), m)
    bad = rep.of_kind("non_finite")
    assert not rep.ok and len(bad) == 2
    assert (bad[0].i0, bad[0].i1) == (5, 7) and (bad[1].i0, bad[1].i1) == (15, 15)
    assert bad[0].t == pytest.approx(t[5])
    assert "non_finite" in rep.summary()


# ------------------------------------------------------------------ LOW: cancel adapter
def test_low_cancel_object_with_method_style_token():
    """Audit LOW: is_cancelled() as a method (robotics_api.CancelToken style) is called."""
    class Token:
        calls = 0

        def is_cancelled(self):
            Token.calls += 1
            return False

    m = planar_2r()
    for name in PLANNERS:
        res = plan(m, [0.0, 0.0], [1.0, 1.0], None, planner=name, cancel=Token())
        assert res.success, (name, res.reason)
    tok = cb.CancelToken()
    assert plan(m, [0.0, 0.0], [1.0, 1.0], None, cancel=tok).success


# ------------------------------------------------------------------ LOW: FD torque
def _quintic(t, T=1.0, a=0.2, b=1.1):
    u = t / T
    s = 10 * u ** 3 - 15 * u ** 4 + 6 * u ** 5
    ds = (30 * u ** 2 - 60 * u ** 3 + 30 * u ** 4) / T
    dds = (60 * u - 180 * u ** 2 + 120 * u ** 3) / T ** 2
    return a + (b - a) * s, (b - a) * ds, (b - a) * dds


def test_low_fd_torque_is_central_second_order():
    """Audit LOW: finite_difference_torque uses the central 3-point formula (error O(h^2))."""
    eng = _engine()
    t2 = 0.45

    def err(h):
        ts = [t2 - h, t2, t2 + h]
        samples = []
        for t in ts:
            q1, _, _ = _quintic(t)
            q2, _, _ = _quintic(t, b=-0.6)
            samples.append((t, [q1, q2]))
        tau = sim_mode.finite_difference_torque(eng, samples)
        q1, d1, a1 = _quintic(t2)
        q2, d2, a2 = _quintic(t2, b=-0.6)
        ref = np.asarray(cb.inverse_dynamics(eng.dm, [q1, q2], [d1, d2], [a1, a2],
                                             eng.gravity, eng.friction))
        return float(np.max(np.abs(tau - ref)))

    e1, e2 = err(0.04), err(0.02)
    assert e1 < 0.05
    assert 3.5 < e1 / e2 < 4.5


def test_low_fd_torque_non_uniform_samples():
    """Audit LOW: non-uniform 3-point formula is exact for a quadratic q(t)."""
    eng = _engine()
    ts = [0.30, 0.37, 0.50]

    def q(t):
        return [0.3 + 0.8 * t + 0.5 * t * t, -0.2 + 0.4 * t - 0.7 * t * t]

    tau = sim_mode.finite_difference_torque(eng, [(t, q(t)) for t in ts])
    t = ts[1]
    ref = cb.inverse_dynamics(eng.dm, q(t), [0.8 + t, 0.4 - 1.4 * t], [1.0, -1.4],
                              eng.gravity, eng.friction)
    assert np.allclose(tau, ref, atol=1e-9)


# ------------------------------------------------------------------ LOW: playback speed
@pytest.mark.parametrize("scale", [1.0, 2.0, 0.5])
def test_low_playback_same_wall_time_any_time_scale(scale):
    """Audit LOW: kinematic and dynamic playback take the same wall time for any time_scale."""
    tr = Trajectory([Waypoint(0.0, [0.0, 0.0]), Waypoint(1.0, [0.3, 0.3])], "cubic")
    ek, ed = _engine(), _engine()
    ek.time_scale = ed.time_scale = scale
    pk = TrajectoryPlayer(ek, tr, KINEMATIC, speed=1.0)
    pd = TrajectoryPlayer(ed, tr, DYNAMIC, speed=1.0)
    for p in (pk, pd):
        p.seek(0.0)
        p.play()
    for _ in range(5):
        pk.advance(0.1)
        pd.advance(0.1)
    assert pk.t == pytest.approx(0.5, abs=1e-9)
    assert pd.t == pytest.approx(0.5, abs=1e-6)
    nk = nd = 5
    while not pk.advance(0.1) and nk < 100:
        nk += 1
    nk += 1
    while not pd.advance(0.1) and nd < 100:
        nd += 1
    nd += 1
    assert pk.finished and pd.finished
    assert abs(nk - nd) <= 1 and 10 <= nd <= 11   # 1.0 s of wall time (kinematic float sum may need one more tick)


# ------------------------------------------------------------------ LOW: engine / cartesian
def test_low_engine_stop_resets_rk45_step():
    """Audit LOW: stop() clears the RK45 suggested step like reset()."""
    def fresh():
        e = _engine(integrator="rk45")
        e.set_tau([0.5, -0.3])
        return e

    e = fresh()
    e.start()
    e.advance(0.2)
    assert e.integrator.next_dt is not None
    e.stop()
    assert e.integrator.next_dt is None
    e.start()
    e.advance(0.05)
    f = fresh()
    f.start()
    f.advance(0.05)
    assert np.array_equal(e.state.q, f.state.q) and e.state.t == f.state.t


def test_low_cartesian_line_far_seed_succeeds():
    """Audit LOW: the branch-jump check does not apply to sample 0 (seed may be far)."""
    m = planar_2r()
    spec = spec_from_model(m)
    qa, qb = np.array([0.3, 0.9]), np.array([0.5, 0.8])
    T0, T1 = fk_numpy(spec, qa)[1], fk_numpy(spec, qb)[1]
    seed = [qa[0] - 1.0, -qa[1]]           # other elbow branch, far from the solution
    res = line(m, T0, T1, seed, n=10, max_joint_step=0.5,
               ik_options={"position_only": True})
    assert res.ok, res.message


# ------------------------------------------------------------------ U2.2: PRM cancel latency
class _SlowCollision:
    """Wall at q0 in (0.4, 0.6); every batch costs tens of ms (big matmul)."""

    def __init__(self, n=900):
        self.a = np.random.default_rng(0).random((n, n))

    def check_batch(self, qs):
        qs = np.asarray(qs, float)
        for _ in range(3):
            self.a @ self.a
        return (qs[:, 0] > 0.4) & (qs[:, 0] < 0.6)


class _CpuLoad:
    """4 threads spinning numpy (context manager)."""

    def __enter__(self):
        self._stop = threading.Event()

        def spin():
            a = np.random.default_rng(1).random((200, 200))
            while not self._stop.is_set():
                a @ a

        self._ths = [threading.Thread(target=spin, daemon=True) for _ in range(4)]
        for t in self._ths:
            t.start()
        return self

    def __exit__(self, *exc):
        self._stop.set()
        for t in self._ths:
            t.join()


def _prm_cancel_latency(collision, delay=0.3):
    """Seconds from setting the cancel event to ``plan`` returning."""
    m = planar_2r()
    evt = threading.Event()
    fired = {}

    def fire():
        fired["t"] = time.perf_counter()
        evt.set()

    threading.Timer(delay, fire).start()
    res = plan(m, [0.0, 0.0], [1.0, 1.0], collision, planner="prm", time_limit=30, seed=0,
               cancel=evt)
    lat = time.perf_counter() - fired["t"]
    assert not res.success and res.reason == "cancelled"
    return lat


@pytest.mark.parametrize("kind", ["blocked", "slow_batches"])
def test_u2_prm_cancel_latency_under_load(kind):
    """U2.2: cancel is checked between edge checks / roadmap steps: latency < 0.3 s under load."""
    lats = []
    with _CpuLoad():
        for _ in range(4):
            col = (_Blocked([0.0, 0.0], [1.0, 1.0]) if kind == "blocked" else _SlowCollision())
            lats.append(_prm_cancel_latency(col))
    print(f"PRM cancel latency ({kind}, 4-thread load): max {max(lats) * 1000:.1f} ms, "
          f"all {[round(x * 1000, 1) for x in lats]}")
    assert max(lats) < 0.3
