"""D1 milestone: sample robot -> 5 sample targets -> demo tour (A -> B -> C -> A, unreachable skipped)."""
import os
import time

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
pytest.importorskip("fcl")

from PySide6.QtCore import QCoreApplication

import core_bridge as cb
from app.kin_utils import current_q
from app.main_window import MainWindow
from app.planning.collision import CollisionWorld, shapes_from_project
from app.sample_targets import sample_kinds
from app.settings import AppSettings
from tests.python.test_live_blocking import SAMPLES, load as _load


def load(win, name):
    win._import_sample = True        # as if opened from the start screen / sample menu
    return _load(win, name)

RESULTS: dict = {}


@pytest.fixture
def win(qapp, monkeypatch):
    monkeypatch.setattr(AppSettings, "_ORGANIZATION", "RoboticsAnalyzer_test_d1m")
    monkeypatch.setattr(AppSettings, "_APPLICATION", "RoboticsAnalyzer_test_d1m")
    w = MainWindow()
    w.settings._settings.clear()
    w.collision_action.setChecked(True)
    w.show()
    yield w
    w.project_controller._is_modified = False
    w.sim_dock.hide()
    w.close()


def run_demo(win, name):
    """Run the demo with the Simulation dock hidden; playback advanced by hand (deterministic)."""
    model = load(win, name)
    tp = win.target_panel
    sp = win.sim_panel
    win.sim_dock.hide()
    assert not win.sim_dock.isVisible()
    demo = tp.demo
    demo.pause_ms = demo.skip_ms = 0
    kinds = sample_kinds(SAMPLES[name][1])
    statuses, trajs, reached = [], [], []
    demo.status_changed.connect(statuses.append)
    demo.play_requested.connect(lambda tr, info: trajs.append((info["name"], tr)))

    def on_finished():
        tcp = np.asarray(cb.fk(model)["tcp"], float)
        reached.append(tcp)
    sp.playback_finished.connect(on_finished)

    assert win.start_demo()
    t0 = time.time()
    while demo.running and time.time() - t0 < 120:
        QCoreApplication.processEvents()
        if demo.waiting_playback and sp.player is not None:
            sp.advance_playback(0.05)
        else:
            time.sleep(0.002)
    assert not demo.running, statuses[-3:]
    return model, tp, demo, kinds, statuses, trajs, reached


@pytest.mark.parametrize("name", ["ur5", "panda", "xarm5_gripper"])
def test_demo_tour(win, name):
    model, tp, demo, kinds, statuses, trajs, reached = run_demo(win, name)
    s = demo.last_summary
    assert not s.cancelled and not s.failed, (s, statuses[-4:])
    assert s.visited == ["Pick A", "Place B", "Home-side C", "Pick A"]
    assert s.skipped == ["Too far", "Too close"]
    assert 'Target "Too far": unreachable, skipped' in " | ".join(statuses)
    assert any(t.startswith("Step 2 of 6") for t in statuses)
    assert statuses[-1].startswith("Demo finished: 4 visited, 2 skipped")
    # every visited target reached within 1e-3 m (position) at the end of its playback
    assert len(reached) == 4 and len(trajs) == 4
    for (nm, _tr), tcp in zip(trajs, reached):
        want = tp.store[tp.store.index(nm)].pose
        err = float(np.linalg.norm(tcp[:3, 3] - want[:3, 3]))
        assert err < 1e-3, (nm, err)
    # dense recheck with a fresh collision world: 20 ms samples and 1 cm joint steps
    proj = win.project_controller.current_project
    fresh = CollisionWorld(model, [sh for sh in shapes_from_project(proj) if sh.link >= 0], proj.collision)
    for nm, tr in trajs:
        ts = np.append(np.arange(0.0, tr.duration, 0.02), tr.duration)
        qs = np.array([tr.q(t) for t in ts])
        assert not np.asarray(fresh.check_batch(qs)).any(), (name, nm)
    # the path tube of the last motion is drawn, the markers show 3 green and 2 red
    assert win.viewport.robot_view.path_tube_points is not None
    assert pump_markers(tp)
    st = {n: tp.markers.status_of(n) for n in tp.store.names()}
    assert sorted(v == "bad" for v in st.values()) == [False] * 3 + [True] * 2, st
    RESULTS[name] = {"steps": len(statuses), "motions_s": [round(tr.duration, 2) for _n, tr in trajs]}


def pump_markers(tp, timeout=30.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        QCoreApplication.processEvents()
        if len(tp.last_classification) == 5 and not tp.demo.running:
            return True
        time.sleep(0.005)
    return False


def test_cancel_demo(win):
    model = load(win, "ur5")
    tp = win.target_panel
    demo = tp.demo
    demo.pause_ms = demo.skip_ms = 0
    assert win.start_demo()
    t0 = time.time()
    while not demo.waiting_playback and time.time() - t0 < 60:
        QCoreApplication.processEvents()
        time.sleep(0.002)
    assert demo.waiting_playback
    demo.cancel()
    assert not demo.running and demo.last_summary.cancelled
    assert demo.last_status == "Demo cancelled."
    assert tp._btn_demo.isEnabled() and not tp._btn_demo_stop.isEnabled()
    q = current_q(model)
    for _ in range(5):                         # late playback finish after cancel: ignored
        demo.on_playback_finished()
    assert not demo.running and current_q(model) == q


def test_demo_without_robot_or_targets(win):
    assert not win.start_demo()
    assert not win.target_panel.demo.running
