"""R6.2 integrators: euler / rk4 / rk45 vs analytic and numpy oracles."""
import math
import time

import numpy as np
import pytest

import core_bridge as cb
from app.project.project_data import SimulationSettings
from app.sim import RK45, SimEngine
from app.sim.engine import PAUSED, RUNNING
from app.sim.integrators import euler_step, rk4_step

from .robots import fk_test_robots, random_inertias, random_robot
from .test_dynamics import _link, _oa

M_KG, R, I_COM, G0 = 2.0, 0.7, 0.05, 9.81
I_PIV = M_KG * R * R + I_COM          # inertia about the pivot
TH_EQ = -math.pi / 2                   # hanging down (gravity along -y)


def _pendulum():
    model = cb.RobotModel()
    for p in (model.base, model.tool):
        p.xyz, p.rpy = [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
    model.links.append(cb.Link())
    model.links.append(_link(M_KG, (R, 0, 0), I_COM * np.eye(3)))
    model.joints.append(_oa("j0", 0, 1, (0, 0, 0), (0, 0, 0), (0, 0, 1)))
    model.tcp_link = 1
    return model


def _engine(integrator, dt=1e-3, viscous=None, **settings_kw):
    s = SimulationSettings(integrator=integrator, physics_timestep=dt,
                           gravity=[0.0, -G0, 0.0], **settings_kw)
    fr = None
    if viscous is not None:
        fr = cb.Friction()
        fr.viscous, fr.coulomb, fr.eps = [viscous], [0.0], 1e-3
    return SimEngine(_pendulum(), s, friction=fr)


def _ref_accel(th):
    """Independent oracle: I_piv th'' = -m g r cos(th)."""
    return -M_KG * G0 * R * math.cos(th) / I_PIV


def _ref_solution(th0, T, n=200000):
    """Fine fixed-step RK4 in plain Python on the closed-form ODE."""
    h = T / n
    th, w = th0, 0.0
    for _ in range(n):
        k1t, k1w = w, _ref_accel(th)
        k2t, k2w = w + 0.5 * h * k1w, _ref_accel(th + 0.5 * h * k1t)
        k3t, k3w = w + 0.5 * h * k2w, _ref_accel(th + 0.5 * h * k2t)
        k4t, k4w = w + h * k3w, _ref_accel(th + h * k3t)
        th += h / 6 * (k1t + 2 * k2t + 2 * k3t + k4t)
        w += h / 6 * (k1w + 2 * k2w + 2 * k3w + k4w)
    return th


def _run_to(eng, T):
    while eng.state.t < T - 1e-12:
        eng.step(min(eng.dt, T - eng.state.t) if not hasattr(eng.integrator, "next_dt")
                 else min(eng.integrator.next_dt or eng.dt, T - eng.state.t), full=False)
    return eng.state


# ------------------------------------------------------------ period
@pytest.mark.parametrize("name", ["rk4", "rk45"])
def test_small_angle_period(name):
    eng = _engine(name)
    eng.reset([TH_EQ + 0.02])
    ts = []
    prev = eng.state.q[0] - TH_EQ
    while len(ts) < 3 and eng.state.t < 6.0:
        t0 = eng.state.t
        eng.step(full=False)
        cur = eng.state.q[0] - TH_EQ
        if prev < 0.0 <= cur:  # upward zero crossing, linear interpolation
            ts.append(t0 + (eng.state.t - t0) * (-prev) / (cur - prev))
        prev = cur
    assert len(ts) == 3
    period = ts[2] - ts[1]
    expected = 2 * math.pi * math.sqrt(I_PIV / (M_KG * G0 * R))
    assert period == pytest.approx(expected, rel=0.005)


# ------------------------------------------------------- order of accuracy
def _err_at(name, dt, T=0.7, th0=0.5):
    eng = _engine(name, dt=dt)
    eng.reset([th0])
    for _ in range(round(T / dt)):
        eng.step(full=False)
    return abs(eng.state.q[0] - _ref_solution(th0, T))


def test_euler_order_one():
    e1, e2 = _err_at("euler", 0.01), _err_at("euler", 0.005)
    assert 1.7 <= e1 / e2 <= 2.4


def test_rk4_order_four():
    e1, e2 = _err_at("rk4", 0.05), _err_at("rk4", 0.025)
    assert 12 <= e1 / e2 <= 20


def test_rk45_accuracy_vs_reference():
    eng = _engine("rk45")
    eng.reset([0.0])
    _run_to(eng, 1.0)
    assert abs(eng.state.q[0] - _ref_solution(0.0, 1.0)) < 1e-6


# ------------------------------------------------------------ energy
def _total(eng):
    return eng.refresh().energy["total"]


def test_energy_drift_rk4():
    eng = _engine("rk4", dt=1e-3)
    eng.reset([0.0], [0.0])
    e0 = abs(_total(eng))
    e_ref = _total(eng)
    worst = 0.0
    for k in range(5000):
        eng.step(full=False)
        if k % 250 == 249:
            worst = max(worst, abs(_total(eng) - e_ref))
    # potential-energy zero is arbitrary: scale by the energy swing m g r
    scale = max(e0, M_KG * G0 * R)
    assert worst / scale < 1e-8


def test_energy_drift_rk45():
    eng = _engine("rk45")
    eng.reset([0.0], [0.0])
    e_ref = _total(eng)
    worst = 0.0
    nxt = 0.25
    while eng.state.t < 5.0:
        eng.step(min(eng.integrator.next_dt or eng.dt, 5.0 - eng.state.t), full=False)
        if eng.state.t >= nxt:
            worst = max(worst, abs(_total(eng) - e_ref))
            nxt += 0.25
    assert worst / (M_KG * G0 * R) < 1e-6


@pytest.mark.parametrize("name", ["rk4", "rk45"])
def test_damped_energy_never_increases(name):
    eng = _engine(name, viscous=0.4)
    eng.reset([0.3], [2.0])
    prev = _total(eng)
    for _ in range(300):
        for _ in range(5):
            eng.step(full=False)
        e = _total(eng)
        assert e <= prev + 1e-12
        prev = e
    # energy actually dissipated
    eng0 = _engine(name)
    eng0.reset([0.3], [2.0])
    assert prev < _total(eng0) - 0.1


# -------------------------------------------------------------- adaptive
def _accel_pendulum(t, q, qd):
    return np.array([_ref_accel(q[0])])


def test_adaptive_step_shrinks_and_grows():
    smooth = RK45()
    q, qd, t, dts = np.array([TH_EQ + 0.01]), np.zeros(1), 0.0, []
    for _ in range(60):
        q, qd, dt = smooth(_accel_pendulum, t, q, qd, smooth.next_dt or 0.001)
        t += dt
        dts.append(dt)
    assert dts[-1] > 5 * dts[0]            # grows from the initial small request
    assert dts[-1] == pytest.approx(0.01)  # reaches dt_max when smooth

    fast = RK45()
    q, qd, t, dts_f = np.array([0.0]), np.array([3000.0]), 0.0, []
    for _ in range(60):
        q, qd, dt = fast(_accel_pendulum, t, q, qd, 0.01)
        t += dt
        dts_f.append(dt)
    assert fast.n_rejected > 0 or min(dts_f) < 0.01
    assert np.mean(dts_f) < 0.5 * np.mean(dts[-30:])


def test_rk45_accepts_requested_step_when_small_and_reports_next():
    r = RK45()
    q, qd, dt = r(_accel_pendulum, 0.0, np.array([0.1]), np.zeros(1), 1e-4)
    assert dt == 1e-4
    assert r.next_dt is not None and 1e-4 < r.next_dt <= 5e-4 + 1e-12  # factor <= 5


def test_rk45_step_size_underflow():
    r = RK45(atol=1e-30, rtol=1e-30, dt_min=1e-4)
    with pytest.raises(RuntimeError, match="underflow"):
        r(_accel_pendulum, 0.0, np.array([0.0]), np.array([3000.0]), 0.01)


def test_rk45_nan_is_rejected_then_underflow():
    r = RK45()
    with pytest.raises(RuntimeError):
        r(lambda t, q, qd: np.full(1, np.nan), 0.0, np.zeros(1), np.zeros(1), 0.01)


# --------------------------------------------------------- engine wiring
def test_unknown_integrator_name():
    with pytest.raises(ValueError):
        SimulationSettings(integrator="verlet")
    s = SimulationSettings()
    s.integrator = "verlet"
    with pytest.raises(ValueError):
        SimEngine(_pendulum(), s)


@pytest.mark.parametrize("name,fn", [("euler", euler_step), ("rk4", rk4_step)])
def test_registry_fixed_step(name, fn):
    assert _engine(name).integrator is fn


def test_registry_rk45_uses_settings():
    eng = _engine("rk45", rk45_atol=1e-6, rk45_rtol=1e-5, rk45_dt_min=1e-6, rk45_dt_max=0.005)
    ig = eng.integrator
    assert isinstance(ig, RK45)
    assert (ig.atol, ig.rtol, ig.dt_min, ig.dt_max) == (1e-6, 1e-5, 1e-6, 0.005)


def test_settings_round_trip_old_file():
    old = {"integrator": "euler", "physics_timestep": 0.005, "render_framerate": 30.0,
           "gravity": [0.0, 0.0, -9.81]}
    s = SimulationSettings.from_dict(old)
    assert s.integrator == "euler" and s.rk45_atol == 1e-9 and s.rk45_dt_max == 0.01
    assert SimulationSettings.from_dict(s.to_dict()) == s
    for name in ("euler", "rk4", "rk45"):
        assert SimulationSettings.from_dict({"integrator": name}).integrator == name
    with pytest.raises(ValueError):
        SimulationSettings.from_dict({"integrator": "bogus"})


def test_engine_advance_rk45_reaches_target_time():
    eng = _engine("rk45")
    eng.reset([0.5])
    eng.start()
    done = eng.advance(0.25)
    assert done == pytest.approx(0.25, abs=1e-9)
    assert eng.state.t == pytest.approx(0.25, abs=1e-9)
    assert eng.state.energy["total"] == pytest.approx(
        eng.refresh().energy["total"])


def test_advance_fills_signals_at_end():
    eng = _engine("rk4")
    eng.reset([0.5])
    eng.start()
    eng.advance(0.05)
    s = eng.state
    assert np.allclose(s.tau_inv, [0.0], atol=1e-9)  # free pendulum: tau_inv ~ tau = 0
    assert s.energy["total"] == pytest.approx(s.energy["ke"] + s.energy["pe"])
    assert np.isfinite(s.tcp).all() and not np.allclose(s.tcp, np.eye(4))


def test_non_pd_mass_pauses_engine_and_records_error():
    s = SimulationSettings(integrator="rk4")
    rng = np.random.default_rng(1)
    model = random_robot(3, 3)
    random_inertias(model, rng)
    eng = SimEngine(model, s)
    eng.start()
    assert eng.clock.state == RUNNING
    calls = []

    def boom(accel, t, q, qd, dt):
        calls.append(1)
        raise RuntimeError("mass matrix not positive definite")

    eng.integrator = boom
    done = eng.advance(0.01)
    assert eng.clock.state == PAUSED
    assert "positive definite" in eng.error
    assert done == 0.0 and len(calls) == 1
    with pytest.raises(RuntimeError):
        eng.step()


def test_real_singular_mass_matrix_raises():
    model = random_robot(3, 3)  # massless links
    with pytest.raises(RuntimeError):
        SimEngine(model, SimulationSettings())


def test_error_cleared_on_reset():
    eng = _engine("rk4")
    eng.error = "x"
    eng.reset([0.0])
    assert eng.error is None


# ------------------------------------------------------------- speed
def test_rk4_speed_six_dof():
    model = fk_test_robots()["six_dof_fixture"]
    rng = np.random.default_rng(7)
    random_inertias(model, rng)
    eng = SimEngine(model, SimulationSettings(integrator="rk4", physics_timestep=1e-3))
    eng.reset(np.full(eng.n_dof, 0.1))
    for _ in range(20):
        eng.step(full=False)
    n = 1000
    t0 = time.perf_counter()
    for _ in range(n):
        eng.step(full=False)
    rate = n / (time.perf_counter() - t0)
    print(f"RK4 six_dof steps/s: {rate:.0f}")
    assert rate >= 1000
