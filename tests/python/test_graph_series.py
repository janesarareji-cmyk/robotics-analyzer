"""R6.6 graph data: kinds, names, units, values vs recorder arrays and numpy, decimation."""
import numpy as np
import pytest

from app.graphs.series import KINDS, MAX_POINTS, decimate_minmax, graph_data
from app.project.project_data import SimulationSettings
from app.sim import Recorder, SimEngine

from .robots import random_inertias, random_robot

N = 3


@pytest.fixture(scope="module")
def run():
    rng = np.random.default_rng(5)
    model = random_robot(5, N)
    random_inertias(model, rng)
    e = SimEngine(model, SimulationSettings(physics_timestep=0.001))
    rec = Recorder(e, model)
    e.reset(rng.uniform(-1, 1, N), rng.uniform(-1, 1, N))
    e.set_tau([0.3, -0.2, 0.1])
    e.start()
    for _ in range(60):
        e.step(0.001)
    return rec.export_arrays()


NAMES = ["a", "b", "c"]


def test_joint_kinds(run):
    for kind, sig, unit in (("joint_pos", "q", "rad|m"), ("joint_vel", "qd", "rad/s|m/s"),
                            ("joint_acc", "qdd", "rad/s^2|m/s^2"),
                            ("joint_torque", "tau", "N.m|N")):
        ser = graph_data(run, kind, names=NAMES)
        assert [s.name for s in ser] == NAMES and all(s.unit == unit for s in ser)
        t, d = run[sig]
        for j, s in enumerate(ser):
            assert np.array_equal(s.t, t) and np.array_equal(s.y, d[:, j])
    sel = graph_data(run, "joint_pos", joints=[2, 0], names=NAMES)
    assert [s.name for s in sel] == ["c", "a"]
    assert [s.name for s in graph_data(run, "joint_pos")] == ["dof 0", "dof 1", "dof 2"]
    with pytest.raises(ValueError):
        graph_data(run, "joint_pos", joints=[3])


def test_tcp_xyz_and_orientation(run):
    t, p = run["tcp_pos"]
    ser = graph_data(run, "tcp_xyz")
    assert [s.name for s in ser] == ["x", "y", "z"] and all(s.unit == "m" for s in ser)
    assert all(np.array_equal(s.y, p[:, i]) for i, s in enumerate(ser))
    o = graph_data(run, "tcp_orientation")
    assert [s.name for s in o] == ["qw", "qx", "qy", "qz", "roll", "pitch", "yaw"]
    _, q = run["tcp_quat"]
    assert np.array_equal(o[1].y, q[:, 1])
    # independent: rebuild R from the quaternion, then ZYX euler from matrix entries
    w, x, y, z = q.T
    R = np.stack([
        np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)], -1),
        np.stack([2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)], -1),
        np.stack([2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)], -1)], -2)
    pitch = -np.arcsin(R[:, 2, 0])
    roll = np.arctan2(R[:, 2, 1], R[:, 2, 2])
    yaw = np.arctan2(R[:, 1, 0], R[:, 0, 0])
    assert o[4].y == pytest.approx(roll, abs=1e-9)
    assert o[5].y == pytest.approx(pitch, abs=1e-9)
    assert o[6].y == pytest.approx(yaw, abs=1e-9)
    assert o[4].unit == "rad" and o[0].unit == "1"


def test_path_energy_manip_power(run):
    p = graph_data(run, "tcp_path_3d")
    assert [s.name for s in p] == ["x", "y", "z"]
    assert np.array_equal(p[2].y, run["tcp_pos"][1][:, 2])
    en = graph_data(run, "energy")
    assert [s.name for s in en] == ["ke", "pe", "total"] and en[0].unit == "J"
    assert en[2].y == pytest.approx(en[0].y + en[1].y)
    m = graph_data(run, "manipulability")
    assert [s.name for s in m] == ["sigma_min", "manipulability"]
    assert np.array_equal(m[0].y, run["sigma_min"][1][:, 0])
    pw = graph_data(run, "power", names=NAMES)
    assert [s.name for s in pw] == NAMES + ["sum"] and all(s.unit == "W" for s in pw)
    _, tau = run["tau"]
    _, qd = run["qd"]
    assert pw[-1].y == pytest.approx((tau * qd).sum(axis=1))
    assert sum(s.y for s in pw[:-1]) == pytest.approx(pw[-1].y)


def test_every_kind_returns_series_and_unknown_raises(run):
    for k in KINDS:
        assert graph_data(run, k)
    with pytest.raises(ValueError):
        graph_data(run, "nope")


def test_empty_recording_gives_empty_series():
    e = {n: (np.zeros(0), np.zeros((0, w))) for n, w in
         (("q", 2), ("qd", 2), ("qdd", 2), ("tau", 2), ("tcp_pos", 3), ("tcp_quat", 4),
          ("ke", 1), ("pe", 1), ("total", 1), ("power", 2), ("power_sum", 1),
          ("sigma_min", 1), ("manipulability", 1))}
    for k in KINDS:
        assert all(len(s.t) == 0 for s in graph_data(e, k))


def test_decimation_keeps_extremes_and_cap():
    rng = np.random.default_rng(1)
    n = 200_001
    t = np.linspace(0, 100, n)
    y = rng.normal(size=n)
    y[12345], y[170001] = 50.0, -50.0                   # lone spikes
    td, yd = decimate_minmax(t, y)
    assert len(yd) <= MAX_POINTS and len(td) == len(yd)
    assert yd.max() == 50.0 and yd.min() == -50.0
    assert np.all(np.diff(td) > 0)
    t2, y2 = decimate_minmax(t[:100], y[:100])          # short: untouched
    assert len(y2) == 100
    for cap in (10, 101, 4999):
        assert len(decimate_minmax(t, y, cap)[0]) <= cap
    s = graph_data({"q": (t, np.column_stack([y]))}, "joint_pos")[0]
    assert len(s.y) <= MAX_POINTS and s.y.max() == 50.0 and s.y.min() == -50.0


def test_path_uses_common_indices_and_cap():
    n = 50_000
    t = np.linspace(0, 1, n)
    d = np.column_stack([np.sin(t), np.cos(t), t])
    s = graph_data({"tcp_pos": (t, d)}, "tcp_path_3d")
    assert all(len(x.y) <= MAX_POINTS for x in s)
    assert len(s[0].y) == len(s[1].y) == len(s[2].y)
    assert s[2].y[-1] == d[-1, 2] and s[0].y[-1] == d[-1, 0]
    assert np.allclose(s[0].y, np.sin(s[2].y))           # same indices
