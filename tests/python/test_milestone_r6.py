"""
tests/python/test_milestone_r6.py
R6 stage milestone: trajectory, kinematic and dynamic playback, recorder, graphs and export
on the 6-DOF fixture, driven through an offscreen MainWindow with the real core_bridge.

Oracles are independent of the code under test: the trajectory is checked against its
waypoints, TCP positions against ``numpy_ref.fk_numpy``, energy against its own start value.
"""

import os
import time
import unittest.mock as mock
from pathlib import Path

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtCore import QSettings
from PySide6.QtWidgets import QApplication

from app.graphs.series import KINDS
from app.kin_utils import dof_limits
from app.main_window import MainWindow
from app.project.project_data import SimulationSettings
from app.project.serializer import save
from app.settings import AppSettings
from app.sim import export as sx
from app.sim.engine import PAUSED, RUNNING, STOPPED, SimEngine
from app.sim.mode import DYNAMIC, KINEMATIC
from app.sim.recorder import SIGNALS
from app.sim.trajectory import Trajectory, Waypoint

from .numpy_ref import fk_numpy, spec_from_model

FX = Path(__file__).parent / "fixtures" / "urdf"
URDF = "six_dof_with_gripper.urdf"
TEST_ORG = "RoboticsAnalyzer_test_r6milestone"
DURATION = 4.0
RMS_LIMIT = 1e-3
ENERGY_LIMIT = 1e-8
FK_TOL = 1e-9
RESULTS: dict = {}          # numbers for the stage gate record (printed with -s)


@pytest.fixture
def win(qapp, monkeypatch):
    monkeypatch.setattr(AppSettings, "_ORGANIZATION", TEST_ORG)
    monkeypatch.setattr(AppSettings, "_APPLICATION", TEST_ORG)
    s = QSettings(TEST_ORG, TEST_ORG)
    s.clear()
    s.sync()
    w = MainWindow()
    w.collision_service.enabled = False         # R6 plays trajectories without collision blocking
    w.show()
    yield w
    w.project_controller._is_modified = False
    w.sim_panel.shutdown()
    w.task_runner.wait_all(60000)
    w.sim_dock.hide()
    w.graph_dock.hide()
    w.close()
    s.clear()
    s.sync()


def _pump(win, ms=60000):
    assert win.task_runner.wait_all(ms), "task did not finish"
    QApplication.processEvents()


def _pump_until(cond, timeout=20.0):
    end = time.perf_counter() + timeout
    while time.perf_counter() < end:
        QApplication.processEvents()
        if cond():
            return True
        time.sleep(0.005)
    return cond()


def _load(win):
    """Item 1: import the fixture and set the TCP link to l6."""
    with mock.patch("app.main_window.QMessageBox.warning") as w:
        win.import_urdf_file(str(FX / URDF))
        _pump(win)
    assert w.call_count == 0
    model = win.project_controller.robot
    names = [lk.name for lk in model.links]
    win.set_tcp_link(names.index("l6"))
    win.project_controller._is_modified = False
    assert names[model.tcp_link] == "l6"
    return model


def _trajectory(model):
    """4 cubic waypoints inside the joint limits, first = start pose (all zero)."""
    lo, hi = (np.asarray(v, float) for v in dof_limits(model))
    n = model.dof()
    assert n == 7 and np.all(np.isfinite(lo)) and np.all(np.isfinite(hi))
    half = 0.5 * (hi - lo)
    zero = np.clip(np.zeros(n), lo, hi)
    assert np.allclose(zero, 0.0), "start pose q=0 must be inside the limits"
    fr = [[0.0] * n,
          list(np.linspace(0.2, 0.35, n)),
          list(-np.linspace(0.15, 0.3, n)),
          list(np.linspace(0.1, -0.1, n))]
    wps = [Waypoint(t, np.clip(np.asarray(f) * half, lo, hi) if i else zero)
           for i, (t, f) in enumerate(zip([0.0, 1.3, 2.7, DURATION], fr))]
    tr = Trajectory(wps, "cubic", True)
    assert tr.limit_warnings(lo, hi) == []
    return tr


def _install(win, model, tr):
    win.sim_panel.set_trajectory(tr)
    win.project_controller.current_project.trajectory = win.sim_panel.trajectory
    assert win.sim_panel.trajectory == tr


def _show_dock(win, dock, panel):
    dock.show()
    dock.raise_()
    QApplication.processEvents()
    assert panel.isVisible(), "dock must be visible for its timer"


def _kinematic_run(win, tr):
    sp = win.sim_panel
    sp.set_mode(KINEMATIC)
    _show_dock(win, win.sim_dock, sp)
    sp.timeline._speed.setValue(4.0)
    sp.play_trajectory()
    assert sp.engine.clock.state == RUNNING and sp.timer.isActive()
    assert _pump_until(lambda: sp.engine.clock.state == PAUSED), "playback did not end"
    return sp


# ---------------------------------------------------------------------------
# Item 1, 2: import, trajectory, save, reopen
# ---------------------------------------------------------------------------

def test_items_1_2_trajectory_saved_and_reloaded(win, tmp_path):
    model = _load(win)
    tr = _trajectory(model)
    assert tr.duration == DURATION and len(tr.waypoints) == 4
    assert tr.interpolation == "cubic" and tr.clamped
    _install(win, model, tr)
    pc = win.project_controller
    out = tmp_path / "r6.robotproj"
    save(pc.current_project, out)
    pc._is_modified = False
    pc.close_project(None)
    assert win.sim_panel.trajectory is None
    assert pc.open_project(None, str(out))
    assert pc.current_project.trajectory == tr
    assert win.sim_panel.trajectory == tr
    assert win.sim_panel.timeline.table.rowCount() == 4
    # clamped spline: zero end velocity, hits every waypoint
    assert np.allclose(tr.qd(0.0), 0.0, atol=1e-12) and np.allclose(tr.qd(DURATION), 0.0, atol=1e-12)
    for w in tr.waypoints:
        assert np.allclose(tr.q(w.t), w.q, atol=1e-12)


# ---------------------------------------------------------------------------
# Item 3: kinematic playback
# ---------------------------------------------------------------------------

def test_item_3_kinematic_playback(win):
    model = _load(win)
    tr = _trajectory(model)
    _install(win, model, tr)
    sp = _kinematic_run(win, tr)
    last = np.asarray(tr.waypoints[-1].q)
    err = float(np.max(np.abs(sp.engine.state.q - last)))
    RESULTS["kin_final_q_err"] = err
    assert err < 1e-9
    arrays = sp.recorder.export_arrays()
    assert set(arrays) == {s[0] for s in SIGNALS}
    for name, (t, d) in arrays.items():
        assert len(t) > 10 and d.shape[0] == len(t), name
        assert np.all(np.isfinite(d)), name
    t = arrays["q"][0]
    assert np.all(np.diff(t) > 0) and t[-1] == pytest.approx(DURATION, abs=1e-9)
    RESULTS["kin_samples"] = len(t)
    sp.stop_trajectory()
    assert sp.engine.clock.state == STOPPED


# ---------------------------------------------------------------------------
# Item 4: dynamic playback (computed torque) and energy balance
# ---------------------------------------------------------------------------

def test_item_4_dynamic_tracking(win):
    model = _load(win)
    tr = _trajectory(model)
    _install(win, model, tr)
    sp = win.sim_panel
    sp.set_mode(DYNAMIC)
    _show_dock(win, win.sim_dock, sp)
    e = sp.engine
    assert e.gravity == (0.0, 0.0, -9.81) and e.integrator is not None
    sp.play_trajectory()
    pl = sp.player
    assert pl is not None and pl.mode == DYNAMIC and e.clock.state == RUNNING
    guard = 0
    while not pl.finished and guard < 10000:
        pl.advance(0.02)
        guard += 1
    assert pl.finished and e.error is None
    sp._emit_state()
    arrays = sp.recorder.export_arrays()
    t, q = arrays["q"]
    desired = np.array([tr.q(x) for x in t])
    err = q - desired
    rms = float(np.sqrt(np.mean(err * err)))
    mx = float(np.max(np.abs(err)))
    RESULTS.update(track_rms=rms, track_max=mx, dyn_samples=len(t))
    assert rms < RMS_LIMIT
    assert t[-1] == pytest.approx(DURATION, abs=2e-3)
    assert np.all(np.isfinite(arrays["tau"][1]))
    sp.stop_trajectory()


def test_item_4_energy_balance_free_motion(win):
    """Gravity-free, zero torque, zero friction, RK4: total energy is conserved."""
    model = _load(win)
    st = SimulationSettings(integrator="rk4", physics_timestep=0.001, gravity=[0.0, 0.0, 0.0])
    eng = SimEngine(model, st)
    assert eng.friction is None and eng.integrator is not None
    n = eng.n_dof
    qd0 = np.linspace(0.6, -0.5, n)
    q0 = np.linspace(-0.2, 0.25, n)
    eng.reset(q0, qd0)
    e0 = eng.state.energy["total"]
    assert e0 > 1e-6, "test needs real motion"
    worst = 0.0
    for _ in range(1000):
        s = eng.step()
        worst = max(worst, abs(s.energy["total"] - e0) / e0)
    assert eng.error is None
    RESULTS["energy_drift"] = worst
    RESULTS["energy_E0"] = e0
    assert worst < ENERGY_LIMIT
    assert eng.state.t == pytest.approx(1.0, abs=1e-9)


# ---------------------------------------------------------------------------
# Item 5: all graph kinds draw
# ---------------------------------------------------------------------------

def test_item_5_all_graph_kinds_draw(win):
    model = _load(win)
    tr = _trajectory(model)
    _install(win, model, tr)
    _kinematic_run(win, tr)
    gp = win.graph_panel
    _show_dock(win, win.graph_dock, gp)
    paths = []
    gp.path_3d_changed.connect(paths.append)
    assert len(KINDS) >= 8
    for kind in KINDS:
        gp.set_kind(kind)
        assert gp.last_error is None, (kind, gp.last_error)
        assert gp.plot.grab_ok(), kind
        if kind == "tcp_path_3d":
            assert gp.plot.curve_count == 0
            assert paths[-1] is not None and paths[-1].shape[1] == 3
        else:
            assert gp.plot.curve_count > 0, kind
    win.sim_panel.stop_trajectory()


# ---------------------------------------------------------------------------
# Item 6: export CSV and NPZ, re-import, independent FK
# ---------------------------------------------------------------------------

def _export(win, path, fmt):
    with mock.patch("app.main_window.QFileDialog.getSaveFileName", return_value=(str(path), "")):
        win.export_data("run", fmt)
    _pump(win)
    assert Path(path).exists()


def test_item_6_export_roundtrip_and_independent_fk(win, tmp_path):
    model = _load(win)
    tr = _trajectory(model)
    _install(win, model, tr)
    sp = _kinematic_run(win, tr)
    ref = sp.recorder.export_arrays()
    _export(win, tmp_path / "run.csv", "csv")
    _export(win, tmp_path / "run.npz", "npz")
    spec = spec_from_model(model)
    worst_fk = 0.0
    worst_io = 0.0
    for loader, p in ((sx.load_csv, tmp_path / "run.csv"), (sx.load_npz, tmp_path / "run.npz")):
        arrays, meta = loader(p)
        assert set(arrays) == set(ref)
        for name in ref:
            t0, d0 = ref[name]
            t1, d1 = arrays[name]
            assert t1.shape == t0.shape and d1.shape == d0.shape, name
            worst_io = max(worst_io, float(np.max(np.abs(t1 - t0))), float(np.max(np.abs(d1 - d0))))
            assert np.array_equal(t1, t0) and np.array_equal(d1, d0), (p.suffix, name)
        assert meta.get("robot") is not None and "date" in meta
        q = arrays["q"][1]
        pos = arrays["tcp_pos"][1]
        indep = np.array([fk_numpy(spec, qi)[1][:3, 3] for qi in q])
        worst_fk = max(worst_fk, float(np.max(np.abs(indep - pos))))
    RESULTS.update(export_max_diff=worst_io, export_fk_diff=worst_fk)
    assert worst_fk < FK_TOL
    sp.stop_trajectory()


# ---------------------------------------------------------------------------
# Item 7: memory cap, no leftover timers
# ---------------------------------------------------------------------------

def test_item_7_memory_and_timers(win):
    model = _load(win)
    tr = _trajectory(model)
    _install(win, model, tr)
    sp = _kinematic_run(win, tr)
    gp = win.graph_panel
    _show_dock(win, win.graph_dock, gp)
    rec = sp.recorder
    cap = rec.memory_mb * 1024 * 1024
    nbytes = rec.buffer.nbytes
    RESULTS.update(recorder_bytes=nbytes, recorder_cap=cap, recorder_rows=len(rec))
    assert 0 < nbytes <= cap
    arr = rec.export_arrays()
    used = sum(t.nbytes + d.nbytes for t, d in arr.values())
    assert used <= cap * 2          # exported copies: the time column is repeated per signal
    assert rec.dropped == 0 and not rec.stopped_full
    # paused at the end: no timer should keep running
    assert _pump_until(lambda: not sp.timer.isActive() and not gp.timer.isActive(), 3.0)
    sp.stop_trajectory()
    assert sp.engine.clock.state == STOPPED
    assert not sp.timer.isActive() and not gp.timer.isActive()
    # dynamic run stopped in the middle, then window hidden / closed
    sp.set_mode(DYNAMIC)
    sp.play_trajectory()
    assert sp.engine.clock.state == RUNNING
    sp.stop_trajectory()
    assert not sp.timer.isActive()
    win.sim_dock.hide()
    win.graph_dock.hide()
    QApplication.processEvents()
    assert not sp.timer.isActive() and not gp.timer.isActive()
    assert win.task_runner.wait_all(10000)


def test_zz_print_gate_numbers():
    """Numbers for the PROGRESS.md gate record (run with -s)."""
    print("R6 GATE NUMBERS", {k: (f"{v:.3e}" if isinstance(v, float) else v) for k, v in RESULTS.items()})
