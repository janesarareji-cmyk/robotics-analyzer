"""R6.1 simulation core: state machine, time control, state copy, rest check."""
import numpy as np
import pytest

from app.project.project_data import SimulationSettings
from app.sim import SimClock, SimEngine, SimState
from app.sim.engine import PAUSED, RUNNING, STOPPED

from .robots import random_inertias, random_robot


def _engine(gravity=(0.0, 0.0, 0.0), dt=0.001, **kw):
    rng = np.random.default_rng(5)
    model = random_robot(3, 2)
    random_inertias(model, rng)
    s = SimulationSettings(physics_timestep=dt, gravity=list(gravity))
    return SimEngine(model, s, **kw)


# ------------------------------------------------------------ state machine
def test_clock_legal_transitions():
    c = SimClock()
    seen = []
    c.on_state_changed.append(lambda new, old: seen.append((old, new)))
    assert c.state == STOPPED
    c.start()
    assert c.state == RUNNING
    c.pause()
    assert c.state == PAUSED
    c.resume()
    assert c.state == RUNNING
    c.stop()
    assert c.state == STOPPED
    c.start()
    c.pause()
    c.stop()  # PAUSED -> STOPPED
    assert c.state == STOPPED
    assert seen[0] == (STOPPED, RUNNING) and seen[1] == (RUNNING, PAUSED)
    assert len(seen) == 7


@pytest.mark.parametrize("setup,call", [
    ([], "pause"), ([], "resume"), ([], "stop"),
    (["start"], "start"), (["start"], "resume"),
    (["start", "pause"], "start"), (["start", "pause"], "pause"),
])
def test_clock_illegal_transitions(setup, call):
    c = SimClock()
    for name in setup:
        getattr(c, name)()
    before = c.state
    with pytest.raises(ValueError):
        getattr(c, call)()
    assert c.state == before


# ---------------------------------------------------------------- engine
def test_stop_restores_initial_state():
    eng = _engine(gravity=(0.0, 0.0, -9.81))
    q0 = np.array([0.3, -0.4])
    eng.reset(q0, np.array([0.1, 0.0]))
    init = eng.state.copy()
    eng.start()
    eng.advance(0.05)
    assert eng.state.t > 0.0 and not np.allclose(eng.state.q, q0)
    eng.stop()
    assert eng.clock.state == STOPPED
    assert eng.state.t == 0.0
    np.testing.assert_array_equal(eng.state.q, init.q)
    np.testing.assert_array_equal(eng.state.qd, init.qd)
    np.testing.assert_allclose(eng.state.tcp, init.tcp)


def test_advance_time_scale_two():
    eng = _engine(dt=0.001)
    eng.time_scale = 2.0
    eng.start()
    got = eng.advance(0.05)
    assert got == pytest.approx(0.1, abs=1e-12)
    assert eng.state.t == pytest.approx(0.1, abs=1e-12)
    assert eng.lag == 0.0
    eng.advance(0.0237)  # not a multiple of dt: still exact
    assert eng.state.t == pytest.approx(0.1 + 0.0474, abs=1e-12)


def test_advance_only_when_running():
    eng = _engine()
    assert eng.advance(0.1) == 0.0 and eng.state.t == 0.0
    eng.start()
    eng.pause()
    assert eng.advance(0.1) == 0.0 and eng.state.t == 0.0


def test_time_scale_bounds():
    eng = _engine()
    for bad in (0.0, -1.0, 100.1):
        with pytest.raises(ValueError):
            eng.time_scale = bad
    eng.time_scale = 100.0
    assert eng.time_scale == 100.0


def test_max_steps_cap_sets_lag():
    eng = _engine(dt=0.001, max_steps_per_advance=10)
    calls = []
    inner = eng.integrator

    def counting(accel, t, q, qd, dt):
        calls.append(dt)
        return inner(accel, t, q, qd, dt)

    eng.integrator = counting
    eng.start()
    eng.advance(1.0)  # needs 1000 steps
    assert len(calls) == 10
    assert eng.state.t == pytest.approx(0.010, abs=1e-12)
    assert eng.lag == pytest.approx(0.99, abs=1e-9)
    eng.advance(0.005)  # fits: lag resets
    assert eng.lag == 0.0


def test_simstate_copy_is_deep():
    s = SimState.zeros(2)
    s.q[:] = [1.0, 2.0]
    s.energy["ke"] = 3.0
    c = s.copy()
    c.q[0] = 99.0
    c.tcp[0, 3] = 5.0
    c.energy["ke"] = 7.0
    c.t = 4.0
    assert s.q[0] == 1.0 and s.tcp[0, 3] == 0.0 and s.energy["ke"] == 3.0 and s.t == 0.0
    for name in ("q", "qd", "qdd", "tau", "tau_inv", "tcp"):
        assert not np.shares_memory(getattr(s, name), getattr(c, name))


def test_zero_torque_zero_gravity_keeps_state():
    eng = _engine()
    q0 = np.array([0.7, -0.2])
    eng.reset(q0)
    eng.start()
    eng.advance(0.2)
    assert eng.state.t == pytest.approx(0.2)
    np.testing.assert_allclose(eng.state.q, q0, atol=1e-12)
    np.testing.assert_allclose(eng.state.qd, 0.0, atol=1e-12)
    assert eng.state.energy["total"] == pytest.approx(eng._initial.energy["total"])


def test_state_fields_filled():
    eng = _engine(gravity=(0.0, 0.0, -9.81))
    eng.reset(np.array([0.4, 0.5]))
    s = eng.step()
    assert s.tcp.shape == (4, 4) and s.qdd.shape == (2,) and s.tau_inv.shape == (2,)
    # RNEA torque for qdd equals applied torque (zero)
    np.testing.assert_allclose(s.tau_inv, s.tau, atol=1e-8)
    assert s.energy["total"] == pytest.approx(s.energy["ke"] + s.energy["pe"])


def test_gravity_from_settings_changes_motion():
    e0 = _engine(gravity=(0.0, 0.0, 0.0))
    e1 = _engine(gravity=(0.0, 0.0, -9.81))
    for e in (e0, e1):
        e.reset(np.array([0.3, 0.3]))
        e.start()
        e.advance(0.1)
    assert np.allclose(e0.state.q, [0.3, 0.3])
    assert not np.allclose(e1.state.q, [0.3, 0.3])
