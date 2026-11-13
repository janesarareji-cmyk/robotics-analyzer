"""R6.4 trajectory model, playback (no Qt) and project storage."""
import zipfile

import numpy as np
import pytest

from app.project.project_data import ProjectData, SimulationSettings
from app.project.serializer import load, save
from app.sim import DYNAMIC, KINEMATIC, SimEngine
from app.sim.engine import PAUSED, RUNNING, STOPPED
from app.sim.playback import TrajectoryPlayer, resolve_gains
from app.sim.trajectory import Trajectory, Waypoint, waypoint_problems

from .robots import random_inertias, random_robot

RNG = np.random.default_rng(11)
TS = [0.0, 0.7, 1.5, 2.4, 3.0]
QS = RNG.uniform(-1.0, 1.0, (5, 3))


def _traj(kind="cubic", clamped=True, ts=TS, qs=QS):
    return Trajectory([Waypoint(t, q) for t, q in zip(ts, qs)], kind, clamped)


def _engine(gravity=(0.0, 0.0, -9.81), dt=0.002):
    model = random_robot(3, 2)
    random_inertias(model, np.random.default_rng(5))
    return SimEngine(model, SimulationSettings(physics_timestep=dt, gravity=list(gravity)))


# ----------------------------------------------------------- model / sampling
@pytest.mark.parametrize("kind,clamped", [("linear", True), ("cubic", True), ("cubic", False)])
def test_q_equals_waypoints(kind, clamped):
    tr = _traj(kind, clamped)
    for t, q in zip(TS, QS):
        assert np.max(np.abs(tr.q(t) - q)) < 1e-12
    assert tr.duration == TS[-1] and tr.n_dof == 3


def test_linear_midpoint_and_velocity_closed_form():
    tr = _traj("linear")
    for k in range(4):
        tm = 0.5 * (TS[k] + TS[k + 1])
        assert np.allclose(tr.q(tm), 0.5 * (QS[k] + QS[k + 1]), atol=1e-14)
        slope = (QS[k + 1] - QS[k]) / (TS[k + 1] - TS[k])
        assert np.allclose(tr.qd(tm), slope, atol=1e-13)
        assert np.all(tr.qdd(tm) == 0.0)


def test_t_is_clamped():
    tr = _traj("cubic")
    assert np.allclose(tr.q(-5.0), QS[0]) and np.allclose(tr.q(99.0), QS[-1])
    assert np.allclose(tr.qd(-5.0), tr.qd(0.0)) and np.allclose(tr.qd(99.0), tr.qd(3.0))


def test_cubic_c1_c2_continuity_at_knots():
    tr = _traj("cubic")
    for tk in TS[1:-1]:
        assert np.max(np.abs(tr.qd(tk) - tr.qd(tk - 1e-13))) < 1e-9
        assert np.max(np.abs(tr.qdd(tk) - tr.qdd(tk - 1e-13))) < 1e-9
        assert np.max(np.abs(tr.q(tk + 1e-13) - tr.q(tk - 1e-13))) < 1e-9


def test_clamped_and_natural_ends():
    c = _traj("cubic", True)
    assert np.max(np.abs(c.qd(0.0))) < 1e-12 and np.max(np.abs(c.qd(c.duration))) < 1e-12
    n = _traj("cubic", False)
    assert np.max(np.abs(n.qdd(0.0))) < 1e-12 and np.max(np.abs(n.qdd(n.duration))) < 1e-12
    assert np.max(np.abs(n.qd(0.0))) > 1e-3          # natural ends move


@pytest.mark.parametrize("kind", ["linear", "cubic"])
def test_derivatives_match_finite_differences(kind):
    tr = _traj(kind)
    h = 1e-6
    for t in (0.31, 1.1, 1.93, 2.7):            # away from knots
        assert np.max(np.abs((tr.q(t + h) - tr.q(t - h)) / (2 * h) - tr.qd(t))) < 1e-5
        assert np.max(np.abs((tr.qd(t + h) - tr.qd(t - h)) / (2 * h) - tr.qdd(t))) < 1e-5


@pytest.mark.parametrize("clamped", [True, False])
def test_cubic_matches_independent_polynomial_solve(clamped):
    """Oracle: unknown cubic coefficients per segment, 4(n-1) linear equations."""
    ts, y = np.array(TS[:4]), QS[:4, 0]
    n = len(ts) - 1
    A, b = np.zeros((4 * n, 4 * n)), np.zeros(4 * n)
    row = 0
    for i in range(n):                          # p_i(t_i)=y_i, p_i(t_i+1)=y_i+1
        for tt, yy in ((ts[i], y[i]), (ts[i + 1], y[i + 1])):
            A[row, 4 * i:4 * i + 4] = [1, tt, tt ** 2, tt ** 3]
            b[row] = yy
            row += 1
    for i in range(n - 1):                      # C1, C2 at inner knots
        t = ts[i + 1]
        A[row, 4 * i:4 * i + 4] = [0, 1, 2 * t, 3 * t * t]
        A[row, 4 * i + 4:4 * i + 8] = [0, -1, -2 * t, -3 * t * t]
        row += 1
        A[row, 4 * i:4 * i + 4] = [0, 0, 2, 6 * t]
        A[row, 4 * i + 4:4 * i + 8] = [0, 0, -2, -6 * t]
        row += 1
    for i, tt in ((0, ts[0]), (n - 1, ts[-1])):
        A[row, 4 * i:4 * i + 4] = [0, 1, 2 * tt, 3 * tt * tt] if clamped else [0, 0, 2, 6 * tt]
        row += 1
    coef = np.linalg.solve(A, b).reshape(n, 4)
    tr = Trajectory([Waypoint(t, [v]) for t, v in zip(ts, y)], "cubic", clamped)
    for t in np.linspace(0, ts[-1], 37):
        i = min(int(np.searchsorted(ts, t, side="right")) - 1, n - 1)
        ref = coef[i] @ [1, t, t * t, t ** 3]
        assert abs(tr.q(t)[0] - ref) < 1e-10


def test_two_waypoints_cubic_clamped_is_smoothstep():
    tr = Trajectory([Waypoint(0.0, [0.0]), Waypoint(2.0, [1.0])], "cubic", True)
    s = 0.5
    assert abs(tr.q(1.0)[0] - (3 * s * s - 2 * s ** 3)) < 1e-14


# ------------------------------------------------------------------ validation
def test_validation_errors():
    with pytest.raises(ValueError):
        Trajectory([Waypoint(0.0, [0.0])])
    with pytest.raises(ValueError):
        Trajectory([Waypoint(0.0, [0.0]), Waypoint(0.0, [1.0])])
    with pytest.raises(ValueError):
        Trajectory([Waypoint(0.0, [0.0]), Waypoint(1.0, [1.0, 2.0])])
    with pytest.raises(ValueError):
        Trajectory([Waypoint(0.5, [0.0]), Waypoint(1.0, [1.0])])
    with pytest.raises(ValueError):
        Trajectory([Waypoint(0.0, [0.0]), Waypoint(1.0, [1.0])], "quintic")
    assert waypoint_problems([0.0, 1.0, 0.5], [[0], [1], [2]]) == {2: "t must be strictly increasing"}
    assert 1 in waypoint_problems([0.0, float("nan")], [[0], [1]])


def test_limit_violation_is_warning_not_error():
    tr = Trajectory([Waypoint(0.0, [0.0, 0.0]), Waypoint(1.0, [2.0, 0.1])])
    w = tr.limit_warnings([-1.0, -1.0], [1.0, 1.0])
    assert len(w) == 1 and "waypoint 1 dof 0" in w[0]
    assert tr.limit_warnings([-3, -3], [3, 3]) == []


# ------------------------------------------------------------------- playback
def test_seek_returns_exact_interpolated_q():
    eng = _engine()
    tr = Trajectory([Waypoint(0.0, [0.1, -0.2]), Waypoint(2.0, [0.9, 0.4])], "linear")
    pl = TrajectoryPlayer(eng, tr, KINEMATIC)
    s = pl.seek(0.5)
    assert np.allclose(s.q, [0.1 + 0.8 * 0.25, -0.2 + 0.6 * 0.25], atol=1e-14)
    assert np.allclose(s.qd, [0.4, 0.3], atol=1e-14)
    assert eng.clock.state == STOPPED            # seek does not start the clock
    pl.seek(99.0)
    assert np.allclose(eng.state.q, [0.9, 0.4]) and pl.finished
    assert np.all(np.isfinite(eng.state.tau_inv))


def test_kinematic_playback_ends_on_last_waypoint():
    eng = _engine()
    tr = Trajectory([Waypoint(0.0, [0.0, 0.0]), Waypoint(1.0, [0.5, -0.3]),
                     Waypoint(2.0, [-0.4, 0.8])], "cubic")
    q_before = eng.state.q.copy()
    pl = TrajectoryPlayer(eng, tr, KINEMATIC, speed=2.0)
    pl.play()
    assert eng.clock.state == RUNNING
    reached = False
    for _ in range(200):
        reached = pl.advance(0.02)
        if reached:
            break
    assert reached and pl.finished
    assert np.allclose(eng.state.q, [-0.4, 0.8], atol=1e-12)
    assert eng.clock.state == PAUSED
    assert abs(pl.t - 2.0) < 1e-12
    pl.stop()
    assert eng.clock.state == STOPPED and np.allclose(eng.state.q, q_before)


def test_playback_loop_and_restart():
    eng = _engine()
    tr = Trajectory([Waypoint(0.0, [0.0, 0.0]), Waypoint(1.0, [1.0, 1.0])], "linear")
    pl = TrajectoryPlayer(eng, tr, KINEMATIC, loop=True)
    pl.play()
    for _ in range(30):
        assert not pl.advance(0.1)               # 3 s of playback, never finishes
    assert eng.clock.state == RUNNING and 0.0 <= pl.t <= 1.0
    pl.loop = False
    for _ in range(20):
        if pl.advance(0.1):
            break
    assert pl.finished
    pl.play()                                    # restart from 0 after the end
    assert pl.t == 0.0 or pl.t < 0.2 and eng.clock.state == RUNNING


def test_speed_bounds():
    eng = _engine()
    tr = Trajectory([Waypoint(0.0, [0.0, 0.0]), Waypoint(1.0, [1.0, 1.0])])
    pl = TrajectoryPlayer(eng, tr, KINEMATIC)
    for bad in (0.05, 4.5):
        with pytest.raises(ValueError):
            pl.speed = bad
    pl.speed = 0.1
    pl.speed = 4.0
    with pytest.raises(ValueError):
        TrajectoryPlayer(eng, _traj("linear"), KINEMATIC)   # 3-DOF trajectory, 2-DOF robot


def test_gain_defaults_and_settings():
    kp, kd = resolve_gains(None, 3)
    assert np.all(kp == 100.0) and np.all(kd == 20.0)
    s = SimulationSettings(track_kp=[1, 2], track_kd=[3, 4])
    kp, kd = resolve_gains(s, 2)
    assert list(kp) == [1, 2] and list(kd) == [3, 4]


def test_dynamic_tracking_2r_gravity_on():
    eng = _engine(dt=0.002)
    tr = Trajectory([Waypoint(0.0, [0.2, -0.3]), Waypoint(1.0, [0.8, 0.5]),
                     Waypoint(2.0, [0.1, 0.9])], "cubic", True)
    pl = TrajectoryPlayer(eng, tr, DYNAMIC)
    pl.seek(0.0)
    pl.play()
    sq, n = 0.0, 0
    for _ in range(400):
        done = pl.advance(0.02)
        e = pl.tracking_error()
        sq += e["rms"] ** 2
        n += 1
        if done:
            break
    assert pl.finished and abs(eng.state.t - 2.0) < 1e-9
    assert np.sqrt(sq / n) < 1e-3
    assert np.allclose(eng.state.q, [0.1, 0.9], atol=1e-3)
    assert np.all(np.isfinite(eng.state.tau)) and np.max(np.abs(eng.state.tau)) > 0
    pl.stop()
    pl.detach()
    assert eng.torque_fn is None


def test_dynamic_feedback_removes_initial_offset():
    eng = _engine(dt=0.002)
    tr = Trajectory([Waypoint(0.0, [0.2, -0.3]), Waypoint(2.0, [0.8, 0.5])], "cubic")
    pl = TrajectoryPlayer(eng, tr, DYNAMIC)
    pl.seek(0.0)
    eng.set_state(0.0, np.array([0.25, -0.25]), np.zeros(2))   # 0.05 rad off the path
    e0 = pl.tracking_error()["max"]
    pl.play()
    pl.advance(1.0)
    assert e0 > 0.04 and pl.tracking_error()["max"] < 0.1 * e0


def test_dynamic_speed_scales_playback_time():
    eng = _engine(dt=0.002)
    tr = Trajectory([Waypoint(0.0, [0.0, 0.0]), Waypoint(2.0, [0.3, 0.3])], "cubic")
    pl = TrajectoryPlayer(eng, tr, DYNAMIC, speed=2.0)
    pl.seek(0.0)
    pl.play()
    pl.advance(0.1)
    assert abs(eng.state.t - 0.2) < 1e-9


# --------------------------------------------------------------------- project
def _project(traj):
    p = ProjectData.new("traj")
    p.trajectory = traj
    return p


def test_project_roundtrip_equal(tmp_path):
    tr = _traj("cubic", False)
    out = tmp_path / "a.robotproj"
    save(_project(tr), out)
    q = load(out)
    assert q.metadata.format_version == "1.4.0"
    assert q.trajectory == tr and q.trajectory.to_dict() == tr.to_dict()
    assert np.array_equal(q.trajectory.q(1.234), tr.q(1.234))


def test_project_without_trajectory_has_no_file(tmp_path):
    out = tmp_path / "b.robotproj"
    save(_project(None), out)
    assert "trajectory.json" not in zipfile.ZipFile(out).namelist()
    assert load(out).trajectory is None


def test_old_1_1_0_project_loads_without_trajectory(tmp_path):
    out = tmp_path / "c.robotproj"
    save(_project(_traj()), out)
    old = tmp_path / "old.robotproj"
    with zipfile.ZipFile(out) as zin, zipfile.ZipFile(old, "w") as zout:
        for n in zin.namelist():
            if n == "trajectory.json":
                continue
            data = zin.read(n)
            if n == "project_metadata.json":
                data = data.replace(b'"1.4.0"', b'"1.1.0"')
            zout.writestr(n, data)
    q = load(old)
    assert q.trajectory is None and q.metadata.format_version == "1.1.0"


def test_corrupt_trajectory_is_ignored(tmp_path, caplog):
    out = tmp_path / "d.robotproj"
    save(_project(_traj()), out)
    bad = tmp_path / "bad.robotproj"
    with zipfile.ZipFile(out) as zin, zipfile.ZipFile(bad, "w") as zout:
        for n in zin.namelist():
            zout.writestr(n, b'{"waypoints": [{"t": 0, "q": [1]}]}' if n == "trajectory.json"
                          else zin.read(n))
    assert load(bad).trajectory is None


def test_settings_gains_roundtrip():
    s = SimulationSettings(track_kp=[10.0, 20.0], track_kd=[1.0, 2.0])
    s2 = SimulationSettings.from_dict(s.to_dict())
    assert s2.track_kp == [10.0, 20.0] and s2.track_kd == [1.0, 2.0]
    assert SimulationSettings.from_dict({}).track_kp is None
