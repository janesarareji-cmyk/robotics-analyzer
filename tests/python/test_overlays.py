"""U6.1: overlay framework (toggle, opacity, cache, cancel, panel, menu, mode)."""
import os
import time

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
pytest.importorskip("fcl")

from PySide6.QtCore import QCoreApplication

from app.analysis.overlays import Overlay, robot_hash
from app.main_window import MainWindow
from app.modes import MODES
from app.settings import AppSettings

from .test_curve_editor import pump_until
from .test_live_blocking import load

ORG = "RoboticsAnalyzer_test_u6overlays"


class Dummy(Overlay):
    key = "dummy"
    title = "Dummy"
    blurb = "test overlay"
    default_opacity = 0.5
    recompute_options = ("n",)

    def __init__(self, steps=3, slow=0.0):
        self.steps, self.slow = steps, slow
        self.calls = 0
        self.shown = 0

    def default_options(self):
        return {"n": 4, "tint": 0}

    def compute(self, model, opts, cancel, progress):
        self.calls += 1
        for i in range(self.steps):
            if cancel.is_cancelled():
                return {"cancelled": True}
            time.sleep(self.slow)
            progress(i + 1, self.steps)
        c = np.array([[0.1 * i, 0.0, 0.5] for i in range(opts["n"])])
        return {"centers": c, "h": 0.05}

    def show(self, view, data, opts, base, opacity):
        self.shown += 1
        if data is None:
            return 0
        rgb = np.tile(np.array([[255, 0, 0]], np.uint8), (len(data["centers"]), 1))
        return view.set_overlay_voxels(self.key, data["centers"], data["h"], rgb, opacity, base)

    def legend(self, opts):
        return [(0.0, (0, 0, 255)), (1.0, (255, 0, 0))]

    def legend_labels(self, opts):
        return ("low", "high")

    def summary(self, data, opts):
        return f"{len(data['centers'])} cubes"


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


def test_default_overlays_registered_and_off(win):
    m = win.overlays
    assert set(m.keys) <= {"reach", "singularity", "load"}
    for k in m.keys:
        assert not m.enabled(k)
        assert 0.05 <= m.opacity(k) <= 1.0


def test_overlays_dock_in_analyze_mode(win):
    assert MODES["Analyze"][1][0] == "overlays"
    assert win.mode_for_dock("overlays") == "Analyze"
    win.set_mode("Analyze")
    QCoreApplication.processEvents()
    assert not win.overlay_dock.isHidden()


def test_toggle_cache_and_opacity(win):
    load(win, "ur5")
    m = win.overlays
    d = Dummy()
    m.register(d)
    rv = win.viewport.robot_view
    assert not m.enabled("dummy") and m.data("dummy") is None
    m.set_enabled("dummy", True)
    assert m.is_computing("dummy")
    assert pump_until(lambda: rv.overlay_actor("dummy") is not None)
    assert d.calls == 1 and rv.overlay_voxel_count("dummy") == 4
    assert m.status("dummy") == "4 cubes"
    # off removes the drawing, on again uses the cache (no second compute)
    m.set_enabled("dummy", False)
    assert rv.overlay_actor("dummy") is None
    m.set_enabled("dummy", True)
    assert rv.overlay_actor("dummy") is not None and d.calls == 1
    assert m.compute_count["dummy"] == 1
    # opacity changes the actor property
    m.set_opacity("dummy", 0.8)
    assert abs(rv.overlay_actor("dummy").GetProperty().GetOpacity() - 0.8) < 1e-9
    # drawing-only option: no recompute; compute option: recompute
    m.set_option("dummy", "tint", 3)
    assert d.calls == 1
    m.set_option("dummy", "n", 6)
    assert pump_until(lambda: rv.overlay_voxel_count("dummy") == 6)
    assert d.calls == 2
    # recompute button path
    m.recompute("dummy")
    assert pump_until(lambda: d.calls == 3 and not m.is_computing("dummy"))
    m.set_enabled("dummy", False)


def test_cache_key_changes_with_robot_and_tool(win):
    model = load(win, "ur5")
    h1 = robot_hash(model)
    assert robot_hash(model) == h1
    assert robot_hash(model, "acm2") != h1
    other = load(win, "ur3")
    assert robot_hash(other) != h1


def test_cancel_and_stale_result(win):
    load(win, "ur5")
    m = win.overlays
    d = Dummy(steps=200, slow=0.01)
    m.register(d)
    rv = win.viewport.robot_view
    m.set_enabled("dummy", True)
    assert m.is_computing("dummy")
    m.cancel("dummy")
    assert not m.is_computing("dummy")
    win.task_runner.wait_all(10000)
    for _ in range(20):
        QCoreApplication.processEvents()
        time.sleep(0.01)
    assert rv.overlay_actor("dummy") is None and m.data("dummy") is None
    # robot change while computing: result dropped, new robot recomputes
    m.recompute("dummy")
    assert m.is_computing("dummy")
    load(win, "ur3")
    assert pump_until(lambda: m.data("dummy") is not None, 60)
    m.set_enabled("dummy", False)


def test_panel_card_and_menu_follow_manager(win):
    load(win, "ur5")
    m = win.overlays
    m.register(Dummy())
    from app.panels.overlay_panel import OverlayCard
    card = OverlayCard(m, "dummy")
    card.check.setChecked(True)
    assert m.enabled("dummy")
    assert pump_until(lambda: m.data("dummy") is not None)
    assert card.status.text() == "4 cubes"
    assert len(card.bar.stops) == 2
    card.opacity.setValue(70)
    assert abs(m.opacity("dummy") - 0.7) < 1e-9
    card.check.setChecked(False)
    assert not m.enabled("dummy")
    # View menu: the submenu exists and mirrors the manager
    assert win._overlay_menu.title().replace("&", "") == "Analysis overlays"
    for key, act in win._overlay_actions.items():
        act.setChecked(True)
        assert m.enabled(key)
        m.set_enabled(key, False)
        assert not act.isChecked()
    card.deleteLater()


def test_controls_follow_options_set_from_code(win):
    load(win, "ur5")
    m = win.overlays
    card = win.overlay_panel.cards["singularity"].extra
    m.set_option("singularity", "stat", "any")
    m.set_option("singularity", "cut", False)
    m.set_option("singularity", "kinds", ("elbow",))
    assert card.stat_combo.currentData() == "any" and not card.cut_check.isChecked()
    assert [k for k, c in card.kind_boxes.items() if c.isChecked()] == ["elbow"]
    rc = win.overlay_panel.cards["reach"].extra
    m.set_option("reach", "slice", True)
    assert rc.slice_check.isChecked() and rc.slice_slider.isEnabled()


def test_dock_titles_hidden_only_when_a_tab_shows_the_name(win):
    from PySide6.QtWidgets import QTabBar
    win.set_mode("Move")
    assert pump_until(lambda: win._joint_dock.titleBarWidget() is not None, 5)   # Joints | Collision | Matrices tabs
    assert win._log_dock.titleBarWidget() is None or win._log_dock.titleBarWidget().objectName() == "blank_dock_title"
    win.set_mode("Model")
    pump_until(lambda: True, 0.2)
    # the lone DH editor keeps its title bar
    assert win._dh_dock.titleBarWidget() is None
