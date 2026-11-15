"""D1-fix2: the demo tour runs on a REAL event loop (timers only, no manual advance) and the UI
stays responsive: a 20 ms heartbeat timer is never starved for long, the demo ends on its own,
visits the 3 reachable targets and skips the 2 unreachable ones."""
import os
import statistics
import time

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
pytest.importorskip("fcl")

from PySide6.QtCore import QCoreApplication, QTimer

from app.sample_targets import sample_kinds
from app.settings import AppSettings
from app.main_window import MainWindow
from tests.python.test_live_blocking import SAMPLES, load as _load

MEDIAN_MAX = 0.100
STALL_MAX = 0.400        # s; offscreen + a loaded test machine (real window: ~0.12 s measured)


def load(win, name):
    win._import_sample = True
    return _load(win, name)


@pytest.fixture
def win(qapp, monkeypatch):
    monkeypatch.setattr(AppSettings, "_ORGANIZATION", "RoboticsAnalyzer_test_d1r")
    monkeypatch.setattr(AppSettings, "_APPLICATION", "RoboticsAnalyzer_test_d1r")
    w = MainWindow()
    w.settings._settings.clear()
    w.collision_action.setChecked(True)
    w.show()
    yield w
    w.project_controller._is_modified = False
    w.sim_dock.hide()
    w.close()


class Heartbeat:
    def __init__(self, ms=20):
        self.gaps = []
        self._last = time.perf_counter()
        self.timer = QTimer()
        self.timer.setInterval(ms)
        self.timer.timeout.connect(self._beat)

    def _beat(self):
        now = time.perf_counter()
        self.gaps.append(now - self._last)
        self._last = now

    def start(self):
        self._last = time.perf_counter()
        self.timer.start()

    def stop(self):
        self.timer.stop()


def pump_until(cond, limit):
    t0 = time.time()
    while not cond() and time.time() - t0 < limit:
        QCoreApplication.processEvents()
        time.sleep(0.002)
    return cond()


@pytest.mark.parametrize("name", ["ur5", "panda", "xarm5_gripper", "xarm6", "ur10"])
def test_demo_runs_on_timers_and_stays_responsive(win, name):
    load(win, name)
    demo = win.target_panel.demo
    demo.pause_ms = demo.skip_ms = 150
    if not win.sim_dock.isHidden():
        win.sim_dock.hide()
    kinds = sample_kinds(SAMPLES[name][1])
    played = []
    demo.play_requested.connect(lambda tr, info: played.append(info["name"]))
    hb = Heartbeat()
    assert win.start_demo()
    hb.start()
    done = pump_until(lambda: not demo.running, 90)
    hb.stop()
    assert done, demo.last_status
    s = demo.last_summary
    assert not s.cancelled and not s.failed
    assert set(s.skipped) == {n for n, k in kinds.items() if k == "unreachable"}
    assert set(s.visited) == {n for n, k in kinds.items() if k == "reachable"}
    assert len(s.visited) == 4 and len(s.skipped) == 2      # A, B, C, back to A; 2 skipped
    assert len(set(s.visited)) == 3
    assert len(played) == 4
    assert len(hb.gaps) > 100
    assert statistics.median(hb.gaps) < MEDIAN_MAX
    assert max(hb.gaps) < STALL_MAX, f"UI stalled {max(hb.gaps):.3f} s"


def test_second_run_and_stop_and_close(win):
    """A second run after the first works, Stop demo stops the motion, closing mid-run is clean."""
    load(win, "ur5")
    demo = win.target_panel.demo
    demo.pause_ms = demo.skip_ms = 50
    win.sim_dock.hide()
    assert win.start_demo()
    assert pump_until(lambda: not demo.running, 90)
    assert len(demo.last_summary.visited) == 4
    # second run (clock left PAUSED by the first one: used to lose the background-play flag)
    assert win.start_demo()
    assert pump_until(lambda: demo.waiting_playback, 30)
    sp = win.sim_panel
    assert sp._bg_play and sp._timer.isActive()
    time.sleep(0.2)
    pump_until(lambda: False, 0.3)
    win.target_panel._btn_demo_stop.click()
    assert not demo.running and demo.last_summary.cancelled
    t = sp.player.t
    pump_until(lambda: False, 0.4)
    assert sp.player.t == t, "robot kept moving after Stop demo"
    # close in the middle of a third run
    assert win.start_demo()
    assert pump_until(lambda: demo.waiting_playback, 30)
    win.project_controller._is_modified = False
    win.sim_dock.hide()
    win.close()
    pump_until(lambda: False, 0.3)
