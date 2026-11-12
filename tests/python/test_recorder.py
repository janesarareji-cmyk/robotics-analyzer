"""R6.5 state tracking: ring buffer, memory cap, signals vs independent oracles."""
import tracemalloc

import numpy as np
import pytest

from app.project.project_data import SimulationSettings
from app.sim import Recorder, RingBuffer, SIGNALS, SimEngine
from app.sim.recorder import capacity_for, signal_layout

from .numpy_ref import fk_numpy, inertias_from_model, numpy_dynamics, spec_from_model
from .robots import random_inertias, random_robot
from .test_energy import _pe_numpy

G = np.array([0.0, 0.0, -9.81])


def _engine(n=3, seed=3, **kw):
    rng = np.random.default_rng(seed)
    model = random_robot(seed, n)
    random_inertias(model, rng)
    st = SimulationSettings(physics_timestep=0.001, gravity=list(G), **kw)
    return model, SimEngine(model, st)


# ---------------------------------------------------------------- RingBuffer
def test_ring_wrap_keeps_last_in_order():
    rb = RingBuffer(5, 2)
    for i in range(12):
        rb.push(float(i), [i, -i])
    t, d = rb.view()
    assert len(rb) == 5 and rb.dropped == 7
    assert list(t) == [7.0, 8.0, 9.0, 10.0, 11.0]
    assert np.array_equal(d[:, 0], t) and np.array_equal(d[:, 1], -t)
    rb.clear()
    assert len(rb) == 0 and rb.dropped == 0 and rb.view()[0].size == 0


def test_ring_exact_fill_and_partial():
    rb = RingBuffer(4, 1)
    for i in range(3):
        rb.push(i, [i])
    assert list(rb.view()[0]) == [0, 1, 2] and rb.dropped == 0
    rb.push(3, [3])
    assert list(rb.view()[0]) == [0, 1, 2, 3] and rb.dropped == 0
    rb.push(4, [4])
    assert list(rb.view()[0]) == [1, 2, 3, 4] and rb.dropped == 1


def test_ring_bad_args():
    with pytest.raises(ValueError):
        RingBuffer(0, 1)
    with pytest.raises(ValueError):
        RingBuffer(3, 2).push(0.0, [1.0])


def test_memory_stays_under_cap_after_many_pushes():
    rb = RingBuffer(capacity_for(1, 40), 40)
    nb0 = rb.nbytes
    row = np.ones(40)
    tracemalloc.start()
    base = tracemalloc.get_traced_memory()[0]
    for i in range(100000):
        rb.push(i * 1e-3, row)
    grown = tracemalloc.get_traced_memory()[0] - base
    tracemalloc.stop()
    assert rb.nbytes == nb0 <= 1 * 1024 * 1024
    assert len(rb) == rb.capacity and rb.dropped == 100000 - rb.capacity
    assert grown < 64 * 1024   # no buffer growth (only small temporaries)


# ------------------------------------------------------------ capacity / cap
def test_capacity_from_memory_cap_is_exact():
    model, e = _engine(6, 4)
    rec = Recorder(e, model, memory_mb=1)
    layout, width = signal_layout(6)
    assert width == sum(w for _, w, *_ in layout) and rec.width == width
    assert rec.capacity == (1024 * 1024) // (8 * (width + 1))
    assert rec.buffer.nbytes <= 1024 * 1024
    assert rec.buffer.nbytes + 8 * (width + 1) > 1024 * 1024  # exact: one more row would not fit
    assert [s[0] for s in SIGNALS] == [x[0] for x in layout]


def test_60s_1khz_6dof_under_cap():
    _, w = signal_layout(6)
    assert 60_000 > capacity_for(1, w)               # would not fit in 1 MiB, ring overwrites
    cap = capacity_for(64, w)
    assert cap * 8 * (w + 1) <= 64 * 1024 * 1024
    assert RingBuffer(cap, w).nbytes <= 64 * 1024 * 1024


def test_settings_validation_and_roundtrip():
    with pytest.raises(ValueError):
        SimulationSettings(record_memory_mb=0.5)
    with pytest.raises(ValueError):
        SimulationSettings(record_memory_mb=4096)
    with pytest.raises(ValueError):
        SimulationSettings(record_every=0)
    s = SimulationSettings(record_memory_mb=8, record_every=4, record_overwrite=False)
    s2 = SimulationSettings.from_dict(s.to_dict())
    assert (s2.record_memory_mb, s2.record_every, s2.record_overwrite) == (8, 4, False)
    old = s.to_dict()
    for k in ("record_memory_mb", "record_every", "record_overwrite"):
        old.pop(k)
    s3 = SimulationSettings.from_dict(old)
    assert (s3.record_memory_mb, s3.record_every, s3.record_overwrite) == (64.0, 1, True)
    model, e = _engine(2, 1, record_memory_mb=2, record_every=3, record_overwrite=False)
    rec = Recorder(e, model)
    assert (rec.memory_mb, rec.record_every, rec.overwrite) == (2, 3, False)


# ------------------------------------------------------------ record modes
def test_overwrite_false_stops_and_flags():
    model, e = _engine(2, 1)
    rec = Recorder(e, model, memory_mb=1, overwrite=False)
    rec.buffer = RingBuffer(10, rec.width)   # small buffer to reach "full"
    e.set_state(0.0, [0.1, 0.2], [0.0, 0.0])
    e.start()
    for _ in range(25):
        e.step(0.001)
    assert len(rec) == 10 and rec.stopped_full and rec.dropped == 0
    t, _ = rec.buffer.view()
    assert t[0] == pytest.approx(0.001) and t[-1] == pytest.approx(0.010)
    assert "FULL" in rec.status()


def test_overwrite_true_drops_oldest():
    model, e = _engine(2, 1)
    rec = Recorder(e, model, memory_mb=1)
    rec.buffer = RingBuffer(10, rec.width)
    e.start()
    for _ in range(25):
        e.step(0.001)
    assert len(rec) == 10 and rec.dropped == 15 and not rec.stopped_full
    t, _ = rec.buffer.view()
    assert t[-1] == pytest.approx(0.025) and np.all(np.diff(t) > 0)


def test_decimation_keeps_every_nth():
    model, e = _engine(2, 1)
    rec = Recorder(e, model, record_every=3)
    e.start()
    for _ in range(10):
        e.step(0.001)
    t, _ = rec.buffer.view()
    assert len(rec) == 4                               # steps 1, 4, 7, 10
    assert t == pytest.approx([0.001, 0.004, 0.007, 0.010])


def test_records_only_while_running_data_survives_stop_new_run_clears():
    model, e = _engine(2, 1)
    rec = Recorder(e, model)
    e.step(0.001)
    assert len(rec) == 0                               # STOPPED
    e.start()
    e.step(0.001)
    e.pause()
    e.step(0.001)
    assert len(rec) == 1                               # PAUSED not recorded
    e.resume()
    e.step(0.001)
    assert len(rec) == 2
    e.stop()
    assert len(rec) == 2                               # data survives Stop
    e.start()
    assert len(rec) == 0                               # new run clears
    e.step(0.001)
    assert len(rec) == 1
    e.pause()
    assert len(rec) == 1                               # Pause keeps
    e.reset(np.zeros(2))
    assert len(rec) == 0 and not rec.stopped_full


def test_advance_records_every_step():
    model, e = _engine(2, 1)
    rec = Recorder(e, model)
    e.start()
    e.advance(0.01)
    assert len(rec) == e.steps == 10


# ------------------------------------------------------------ signal values
def test_signals_match_independent_oracles():
    n = 3
    model, e = _engine(n, 7)
    rec = Recorder(e, model)
    spec = spec_from_model(model)
    inertias = inertias_from_model(model)
    rng = np.random.default_rng(11)
    e.reset(rng.uniform(-1, 1, n), rng.uniform(-1, 1, n))
    e.set_tau([0.3, -0.2, 0.1])
    e.start()
    for _ in range(5):
        e.step(0.001)
    ex = rec.export_arrays()
    t, q = ex["q"]
    _, qd = ex["qd"]
    _, tau = ex["tau"]
    assert len(t) == 5 and q.shape == (5, n)
    for i in range(5):
        _, tcp = fk_numpy(spec, q[i])
        assert ex["tcp_pos"][1][i] == pytest.approx(tcp[:3, 3], abs=1e-9)
        # quaternion -> rotation matrix equals numpy tcp rotation
        w, x, y, z = ex["tcp_quat"][1][i]
        R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                      [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                      [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
        assert R == pytest.approx(tcp[:3, :3], abs=1e-9)
        M, _, _ = numpy_dynamics(spec, inertias, q[i], qd[i], G)
        ke = 0.5 * qd[i] @ M @ qd[i]
        pe = _pe_numpy(model, q[i], G)
        assert ex["ke"][1][i, 0] == pytest.approx(ke, rel=1e-9, abs=1e-9)
        assert ex["pe"][1][i, 0] == pytest.approx(pe, rel=1e-9, abs=1e-9)
        assert ex["total"][1][i, 0] == pytest.approx(ke + pe, rel=1e-9, abs=1e-9)
        assert ex["power"][1][i] == pytest.approx(tau[i] * qd[i], abs=1e-12)
        assert ex["power_sum"][1][i, 0] == pytest.approx(float(tau[i] @ qd[i]), abs=1e-12)
        # TCP linear velocity = d/dt tcp_pos (central difference of numpy FK along qd)
        h = 1e-6
        v = (fk_numpy(spec, q[i] + h * qd[i])[1][:3, 3] - fk_numpy(spec, q[i] - h * qd[i])[1][:3, 3]) / (2 * h)
        assert ex["tcp_vel"][1][i] == pytest.approx(v, abs=1e-6)
        # sigma_min / manipulability vs numpy SVD of a numeric position+angular Jacobian is covered
        # in R4.5; here: positive and finite, manipulability = prod of singular values >= 0
        sm = ex["sigma_min"][1][i, 0]
        assert np.isfinite(sm) and sm >= 0.0 and ex["manipulability"][1][i, 0] >= 0.0
    assert tau == pytest.approx(np.tile([0.3, -0.2, 0.1], (5, 1)))
    # qdd from independent forward dynamics: M qdd = tau - c - g
    M, c, g = numpy_dynamics(spec, inertias, q[-1], qd[-1], G)
    assert ex["qdd"][1][-1] == pytest.approx(np.linalg.solve(M, tau[-1] - c - g), rel=1e-6, abs=1e-6)


def test_singularity_signals_match_numpy_svd():
    model, e = _engine(3, 7)
    rec = Recorder(e, model)
    rng = np.random.default_rng(2)
    from .numpy_ref import numeric_jacobian_tcp
    spec = spec_from_model(model)
    q = rng.uniform(-1, 1, 3)
    e.reset(q, np.zeros(3))
    e.start()
    e.step(1e-9)
    qq = rec.export_arrays()["q"][1][0]
    sv = np.linalg.svd(numeric_jacobian_tcp(spec, qq), compute_uv=False)
    sv = sv[sv > 1e-9]
    assert rec.export_arrays()["sigma_min"][1][0, 0] == pytest.approx(sv.min(), rel=1e-5)
    assert rec.export_arrays()["manipulability"][1][0, 0] == pytest.approx(np.prod(sv), rel=1e-4)


def test_estimate_torque_uses_rnea():
    import core_bridge as cb
    model, e = _engine(3, 7)
    rec = Recorder(e, model, estimate_torque=True)
    e.reset([0.2, -0.3, 0.4], [0.1, 0.2, -0.1])
    e.start()
    e.step(0.001)
    ex = rec.export_arrays()
    s = e.state
    ref = np.asarray(cb.inverse_dynamics(e.dm, list(s.q), list(s.qd), list(s.qdd), e.gravity, e.friction))
    assert ex["tau"][1][0] == pytest.approx(ref, abs=1e-12)


def test_quaternion_sign_continuous_over_full_rotation():
    model, e = _engine(3, 7)
    rec = Recorder(e, model)
    q = np.array([0.3, -0.4, 0.5])
    for a in np.linspace(0.0, 2 * np.pi, 121):
        qq = q.copy()
        qq[0] = a
        s = e.set_state(a, qq, np.zeros(3))
        rec.sample(s)
    quat = rec.export_arrays()["tcp_quat"][1]
    assert np.allclose(np.linalg.norm(quat, axis=1), 1.0, atol=1e-12)
    dots = np.sum(quat[1:] * quat[:-1], axis=1)
    assert dots.min() > 0.9                       # no sign jump
    assert quat[-1] == pytest.approx(-quat[0], abs=1e-9)   # full turn flips the quaternion sign


def test_export_arrays_shapes_and_time_order_after_wrap():
    model, e = _engine(2, 1)
    rec = Recorder(e, model)
    rec.buffer = RingBuffer(7, rec.width)
    e.start()
    for _ in range(20):
        e.step(0.001)
    ex = rec.export_arrays()
    for name, w, *_ in rec.layout:
        t, d = ex[name]
        assert t.shape == (7,) and d.shape == (7, w)
        assert np.all(np.diff(t) > 0)
    assert ex["q"][0][-1] == pytest.approx(0.020)


def test_recorder_cost_per_sample():
    import time
    model, e = _engine(6, 2)
    rec = Recorder(e, model)
    s = e.state
    rec.sample(s)
    n = 200
    t0 = time.perf_counter()
    for _ in range(n):
        rec._row(s)
    ms = (time.perf_counter() - t0) * 1000 / n
    print(f"recorder ms/sample: {ms:.3f}")
    assert ms < 5.0
