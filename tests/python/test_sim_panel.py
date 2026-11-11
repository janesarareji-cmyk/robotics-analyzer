"""R6.3: manual simulation mode, Simulation dock, timer, engine rebuild (offscreen Qt)."""
import os
import time
import unittest.mock as mock
from pathlib import Path

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtCore import QSettings, QTimer
from PySide6.QtWidgets import QApplication

import core_bridge as cb
from app.kin_utils import current_q, write_q
from app.main_window import MainWindow
from app.panels.sim_panel import SimPanel
from app.project.project_data import SimulationSettings
from app.settings import AppSettings
from app.sim import DYNAMIC, KINEMATIC, euler_step
from app.sim import mode as sim_mode
from app.sim.engine import PAUSED, RUNNING, STOPPED

from .numpy_ref import inertias_from_model, numpy_dynamics, spec_from_model
from .robots import random_inertias, random_robot

FX = Path(__file__).parent / "fixtures" / "urdf"
TEST_ORG = "RoboticsAnalyzer_test_r63ui"
G = np.array([0.0, 0.0, -9.81])
DT = 0.01


def _model(seed=3, n=2):
    m = random_robot(seed, n)
    random_inertias(m, np.random.default_rng(seed))
    return m


def _settings(dt=DT, **kw):
    return SimulationSettings(physics_timestep=dt, **kw)


@pytest.fixture
def panel(qapp):
    p = SimPanel()
    p.show()
    yield p
    p.shutdown()
    p.hide()


def _loaded(panel, model=None, dt=DT):
    model = model or _model()
    panel.set_model(model, _settings(dt))
    return model


def _pump(ms):
    end = time.perf_counter() + ms / 1000.0
    while time.perf_counter() < end:
        QApplication.processEvents()
        time.sleep(0.002)


# -------------------------------------------------- dynamic free fall vs numpy
def _np_rk4(model, q, qd, dt, steps):
    spec = spec_from_model(model)
    inert = inertias_from_model(model)

    def acc(q_, qd_):
        M, c, grav = numpy_dynamics(spec, inert, q_, qd_, G)
        return np.linalg.solve(M, -(c + grav))

    for _ in range(steps):
        k1q, k1v = qd, acc(q, qd)
        k2q, k2v = qd + 0.5 * dt * k1v, acc(q + 0.5 * dt * k1q, qd + 0.5 * dt * k1v)
        k3q, k3v = qd + 0.5 * dt * k2v, acc(q + 0.5 * dt * k2q, qd + 0.5 * dt * k2v)
        k4q, k4v = qd + dt * k3v, acc(q + dt * k3q, qd + dt * k3v)
        q = q + dt / 6 * (k1q + 2 * k2q + 2 * k3q + k4q)
        qd = qd + dt / 6 * (k1v + 2 * k2v + 2 * k3v + k4v)
    return q, qd


def test_free_fall_matches_numpy_rk4(panel):
    model = _model()
    q0 = np.array([0.4, -0.3])
    write_q(model, q0)
    panel.set_model(model, _settings())
    panel.set_mode(DYNAMIC)
    panel.start()
    eng = panel.engine
    assert eng.clock.state == RUNNING
    for _ in range(5):                      # 5 x 0.1 s wall at scale 1
        eng.advance(0.1)
    assert abs(eng.state.t - 0.5) < 1e-9
    q_ref, qd_ref = _np_rk4(model, q0, np.zeros(2), DT, 50)
    assert np.max(np.abs(eng.state.q - q_ref)) < 1e-6
    assert np.max(np.abs(eng.state.qd - qd_ref)) < 1e-6
    assert np.max(np.abs(eng.state.q - q0)) > 1e-3   # it really moved


# ------------------------------------------------------------ kinematic mode
def test_kinematic_slider_sets_q_keeps_qd_zero(panel):
    model = _loaded(panel)
    assert panel.mode == KINEMATIC
    write_q(model, [0.7, -0.2])
    panel.notify_q_edited()
    s = panel.engine.state
    assert np.allclose(s.q, [0.7, -0.2])
    assert np.all(s.qd == 0.0) and np.all(s.qdd == 0.0)
    assert np.allclose(s.tcp, np.asarray(cb.fk(model, [0.7, -0.2])["tcp"]))


def test_kinematic_estimate_torque_default_off_and_on(panel):
    model = _loaded(panel)
    assert panel.estimated_tau is None
    panel._estimate.setChecked(True)
    for k in range(4):
        write_q(model, [0.1 * k * k, 0.0])
        panel.notify_q_edited()
        time.sleep(0.01)
    tau = panel.estimated_tau
    assert tau is not None and tau.shape == (2,) and np.all(np.isfinite(tau))
    assert np.allclose(panel.engine.state.tau_inv, tau)


# ------------------------------------------------------------- mode rules
def test_mode_switch_rules(panel):
    _loaded(panel)
    eng = panel.engine
    eng.reset(np.array([0.3, 0.2]), np.array([1.0, -1.0]))   # state with velocity
    panel.set_mode(DYNAMIC)
    assert np.allclose(eng.state.q, [0.3, 0.2])               # q kept
    assert np.all(eng.state.qd == 0.0)                        # qd = 0 entering DYNAMIC
    panel.start()
    eng.advance(0.1)
    assert np.any(eng.state.qd != 0.0)
    q_run = eng.state.q.copy()
    panel.set_mode(KINEMATIC)                                 # drops qd, stops clock
    assert np.allclose(eng.state.q, q_run)
    assert np.all(eng.state.qd == 0.0)
    assert eng.clock.state == STOPPED
    with pytest.raises(ValueError):
        sim_mode.check_mode("nope")


def test_button_states_pure():
    f = sim_mode.button_states
    assert f(KINEMATIC, STOPPED) == dict(start=False, pause=False, resume=False, stop=False)
    assert f(DYNAMIC, STOPPED, has_engine=False)["start"] is False
    assert f(DYNAMIC, STOPPED) == dict(start=True, pause=False, resume=False, stop=False)
    assert f(DYNAMIC, RUNNING) == dict(start=False, pause=True, resume=False, stop=True)
    assert f(DYNAMIC, PAUSED) == dict(start=False, pause=False, resume=True, stop=True)


def test_buttons_follow_state(panel):
    assert not any(panel.button_enabled().values())           # no robot
    _loaded(panel)
    assert not any(panel.button_enabled().values())           # kinematic
    panel.set_mode(DYNAMIC)
    assert panel.button_enabled() == dict(start=True, pause=False, resume=False, stop=False)
    panel.start()
    assert panel.button_enabled() == dict(start=False, pause=True, resume=False, stop=True)
    panel.pause()
    assert panel.button_enabled() == dict(start=False, pause=False, resume=True, stop=True)
    panel.resume()
    assert panel.engine.clock.state == RUNNING
    panel.stop()
    assert panel.button_enabled() == dict(start=True, pause=False, resume=False, stop=False)


# ------------------------------------------------------------------ timer
def test_timer_lifecycle(panel):
    _loaded(panel)
    panel.set_mode(DYNAMIC)
    assert not panel.timer.isActive()
    panel.start()
    assert panel.timer.isActive()
    assert panel.timer.interval() == round(1000 / 60)
    panel.pause()
    assert not panel.timer.isActive()
    panel.resume()
    assert panel.timer.isActive()
    panel.stop()
    assert not panel.timer.isActive()
    panel.start()
    assert panel.timer.isActive()
    panel.hide()
    assert not panel.timer.isActive()
    assert panel.engine.clock.state == RUNNING
    panel.show()
    assert panel.timer.isActive()                             # resumes with the panel


def test_timer_ticks_advance_and_emit(panel):
    _loaded(panel)
    panel.set_mode(DYNAMIC)
    seen = []
    panel.state_updated.connect(lambda s: seen.append(s.t))
    panel.start()
    _pump(300)
    panel.pause()
    assert len(seen) >= 3 and seen[-1] > 0.1
    assert "steps=" in panel.status_text


def test_event_loop_not_blocked(panel):
    _loaded(panel)
    panel.set_mode(DYNAMIC)
    panel._scale.setValue(1.0)
    gaps, last = [], [time.perf_counter()]

    def beat():
        now = time.perf_counter()
        gaps.append(now - last[0])
        last[0] = now

    hb = QTimer()
    hb.timeout.connect(beat)
    hb.start(10)
    panel.start()
    _pump(1000)                                               # 1 s wall at scale 1
    t_run = panel.engine.state.t
    panel.stop()
    hb.stop()
    assert len(gaps) > 20
    assert max(gaps) < 0.25, f"UI blocked {max(gaps):.3f}s"
    assert t_run > 0.5                                        # sim really ran
    assert panel.engine.steps == 0                            # stop restored the initial state


def test_torque_gravity_dt_integrator_controls(panel):
    _loaded(panel)
    eng = panel.engine
    panel.tau_boxes[0].setValue(2.5)
    assert np.allclose(eng.tau, [2.5, 0.0])
    panel._gravity.setChecked(False)
    assert eng.gravity == (0.0, 0.0, 0.0)
    panel._gravity.setChecked(True)
    assert eng.gravity == (0.0, 0.0, -9.81)
    panel._friction.setChecked(False)
    assert eng.friction is None
    panel._dt.setValue(0.002)
    assert eng.dt == 0.002
    panel._scale.setValue(2.0)
    assert eng.time_scale == 2.0
    panel._integ_combo.setCurrentText("euler")
    assert eng.integrator is euler_step
    panel._integ_combo.setCurrentText("rk45")
    assert hasattr(eng.integrator, "next_dt")


def test_error_pauses_and_shows(panel):
    _loaded(panel)
    panel.set_mode(DYNAMIC)
    panel.start()
    panel.engine._fail(RuntimeError("boom"))
    assert panel.engine.clock.state == PAUSED and not panel.timer.isActive()
    assert "boom" in panel.status_text


# ------------------------------------------------------- MainWindow wiring
@pytest.fixture
def win(qapp, monkeypatch):
    monkeypatch.setattr(AppSettings, "_ORGANIZATION", TEST_ORG)
    monkeypatch.setattr(AppSettings, "_APPLICATION", TEST_ORG)
    s = QSettings(TEST_ORG, TEST_ORG)
    s.clear()
    s.sync()
    w = MainWindow()
    w.show()
    yield w
    w.project_controller._is_modified = False
    w.sim_dock.hide()
    w.close()
    s.clear()
    s.sync()


def _load(win, name="planar_2r.urdf"):
    with mock.patch("app.main_window.QMessageBox.warning"):
        win.import_urdf_file(str(FX / name))
        assert win.task_runner.wait_all(20000)
        QApplication.processEvents()
    win.project_controller._is_modified = False
    return win.project_controller.robot


def test_dock_hidden_toggle_and_tabified(win):
    assert win.sim_dock.isHidden()
    win._action_view_simulation.setChecked(True)
    assert win.sim_dock.isVisible()
    assert win.sim_dock in win.tabifiedDockWidgets(win.joint_dock)
    win._action_view_simulation.setChecked(False)
    assert win.sim_dock.isHidden()


def test_engine_lifecycle_open_edit_close(win):
    assert win.sim_panel.engine is None
    model = _load(win)
    eng1 = win.sim_panel.engine
    assert eng1 is not None and eng1.n_dof == model.dof()
    assert eng1.clock.state == STOPPED
    # model edit -> new engine
    win.project_controller.set_robot(model)
    assert win.sim_panel.engine is not eng1
    # run, then close the project: engine gone, timer off
    win._action_view_simulation.setChecked(True)
    win.sim_panel.set_mode(DYNAMIC)
    win.sim_panel.start()
    assert win.sim_panel.timer.isActive()
    win.project_controller.close_project(None)
    assert win.sim_panel.engine is None
    assert not win.sim_panel.timer.isActive()
    assert not any(win.sim_panel.button_enabled().values())


def test_tick_pushes_q_to_model_and_joint_panel_without_feedback(win):
    model = _load(win)
    win._action_view_simulation.setChecked(True)
    sp = win.sim_panel
    sp.set_mode(DYNAMIC)
    sp.tau_boxes[0].setValue(2.0)             # planar arm: gravity does not move it
    qc = []
    win.joint_panel.q_changed.connect(lambda: qc.append(1))
    modified = []
    win.project_controller.modified_changed.connect(modified.append)
    sp.start()
    _pump(400)
    sp.pause()
    s = sp.engine.state
    assert np.max(np.abs(s.q)) > 1e-4
    assert np.allclose(current_q(model), s.q)                 # model q = sim q
    assert len(win.joint_panel.rows) == len(s.q)
    assert qc == []                                           # no feedback loop
    assert modified == []                                     # sim does not dirty the project


def test_joint_slider_in_kinematic_updates_engine(win):
    model = _load(win)
    row = win.joint_panel.rows[0]
    row.spinbox.setValue(row.spinbox.value() + 5.0)
    s = win.sim_panel.engine.state
    assert np.allclose(s.q, current_q(model)) and np.all(s.qd == 0.0)
    assert abs(s.q[0]) > 1e-3


def test_hide_dock_stops_timer(win):
    _load(win)
    win._action_view_simulation.setChecked(True)
    win.sim_panel.set_mode(DYNAMIC)
    win.sim_panel.start()
    assert win.sim_panel.timer.isActive()
    # Offscreen Qt delivers dock hide / visibilityChanged lazily for a tabbed dock,
    # so emit the dock signal the way Qt does when the dock closes.
    win.sim_dock.visibilityChanged.emit(False)
    assert not win.sim_panel.timer.isActive()
    assert win.sim_panel.engine.clock.state == RUNNING        # paused display only
