"""U6.4: joint load overlay (torque and speed bars)."""
import os
import time

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
pytest.importorskip("fcl")

from PySide6.QtCore import QCoreApplication

import core_bridge as cb
from app.analysis.load_overlay import LoadOverlay, dof_masters, load_ratios
from app.kin_utils import current_q
from app.main_window import MainWindow
from app.settings import AppSettings
from app.sim.trajectory import Trajectory, Waypoint
from app.viewport.load_bars import GREEN, ORANGE, RED, load_color

from .test_curve_editor import pump_until
from .test_live_blocking import SAMPLES, load, put_q

ORG = "RoboticsAnalyzer_test_u6load"
UR5_Q = [0.3, -1.2, 1.4, -0.5, 0.8, 0.2]


@pytest.fixture
def win(qapp, monkeypatch):
    monkeypatch.setattr(AppSettings, "_ORGANIZATION", ORG)
    monkeypatch.setattr(AppSettings, "_APPLICATION", ORG)
    AppSettings()._settings.clear()
    w = MainWindow()
    w.settings._settings.clear()
    w.show()
    yield w
    w.project_controller._is_modified = False
    w.sim_dock.hide()
    w.close()


def test_colours_by_ratio():
    assert load_color(0.0) == GREEN and load_color(0.7) == GREEN
    assert load_color(0.71) == ORANGE and load_color(1.0) == ORANGE
    assert load_color(1.01) == RED


def test_static_torque_equals_rnea_with_zero_motion(win):
    model = load(win, "ur5")
    cm = cb.compile(model)
    dm = cb.build_dyn_model(model, cm)
    d = load_ratios(model, dm, UR5_Q)
    ref = np.asarray(cb.inverse_dynamics(dm, UR5_Q, [0.0] * 6, [0.0] * 6, [0.0, 0.0, -9.81]))
    assert np.array_equal(d["tau"], ref)
    assert np.allclose(d["tau"], cb.gravity_torque(dm, UR5_Q, [0.0, 0.0, -9.81]))
    assert np.all(d["sratio"] == 0.0)
    masters = dof_masters(model)
    lim = np.array([model.joints[j].limits.effort for j in masters])
    assert np.allclose(d["tratio"], np.abs(ref) / lim)
    assert d["tratio"].max() > 0.01


def test_speed_ratio_and_overload(win):
    model = load(win, "ur5")
    dm = cb.build_dyn_model(model, cb.compile(model))
    masters = dof_masters(model)
    vlim = np.array([model.joints[j].limits.vel for j in masters])
    qd = 0.5 * vlim
    d = load_ratios(model, dm, UR5_Q, qd, [0.0] * 6)
    assert np.allclose(d["sratio"], 0.5)
    d2 = load_ratios(model, dm, UR5_Q, 1.5 * vlim, [0.0] * 6)
    assert np.allclose(d2["sratio"], 1.5)
    # a huge acceleration overloads the torque
    d3 = load_ratios(model, dm, UR5_Q, None, [500.0] * 6)
    assert d3["tratio"].max() > 1.0


def test_overlay_draws_bars_on_slider_move(win):
    model = load(win, "ur5")
    put_q(win, UR5_Q)
    m = win.overlays
    m.set_enabled("load", True)
    ov = m.overlay("load")
    assert ov.last is not None and ov.bars is not None and ov.bars.isVisible()
    assert len(ov.bars.items) == 6                       # six arm joints, no fixed joints
    first = ov.bars.items[0]
    assert 0 <= first[0] <= win.viewport.width() and 0 <= first[1] <= win.viewport.height()
    t0 = np.array(ov.last["tau"])
    put_q(win, [0.3, -0.3, 0.4, -0.5, 0.8, 0.2])
    time.sleep(0.05)
    put_q(win, [0.3, -0.3, 0.4, -0.5, 0.8, 0.2])
    assert not np.allclose(ov.last["tau"], t0)
    # static: no motion, no speed bars
    assert np.all(ov.last["sratio"] == 0)
    m.set_enabled("load", False)
    assert ov.bars.items == [] and not ov.bars.isVisible()


def test_refresh_limit_and_cost(win):
    load(win, "ur5")
    m = win.overlays
    m.set_enabled("load", True)
    ov = m.overlay("load")
    n0 = ov.updates
    t0 = time.perf_counter()
    for i in range(30):                                  # 30 updates as fast as possible
        put_q(win, [0.3 + 0.01 * i, -1.2, 1.4, -0.5, 0.8, 0.2])
    dt = time.perf_counter() - t0
    drawn = ov.updates - n0
    assert drawn <= int(dt * 30) + 3
    pump_until(lambda: not ov.has_pending, 2.0)           # the newest state is drawn by the timer
    assert ov.last["tau"] is not None
    print(f"load overlay: {drawn} draws for 30 updates in {dt:.3f} s, last compute {ov.last_cost_ms:.2f} ms")
    assert ov.last_cost_ms < 2.0
    m.set_enabled("load", False)


def test_playback_gives_speed_from_trajectory(win):
    model = load(win, "ur5")
    q0 = np.array(current_q(model))
    q1 = q0 + np.array([0.6, -0.4, 0.4, 0.3, -0.3, 0.2])
    traj = Trajectory([Waypoint(0.0, q0.tolist()), Waypoint(2.0, q1.tolist())], interpolation="cubic")
    m = win.overlays
    m.set_enabled("load", True)
    ov = m.overlay("load")
    assert win.sim_panel.load_trajectory(traj)
    win.sim_panel.set_mode("kinematic")
    win.sim_panel.seek_trajectory(0.5)
    assert pump_until(lambda: not ov.has_pending, 2.0)
    assert ov.last is not None
    assert ov.last["sratio"].max() > 0.0                  # qd comes from the trajectory
    m.set_enabled("load", False)
