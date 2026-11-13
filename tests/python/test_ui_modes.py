"""U5.4: work modes (mode bar, docks per mode, shortcuts, memory per project)."""
import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest
from PySide6.QtCore import QCoreApplication
from PySide6.QtWidgets import QDockWidget, QToolBar

from app.modes import MODES, MODE_NAMES
from app.main_window import MainWindow
from app.settings import AppSettings
from tests.python.test_live_blocking import load


@pytest.fixture
def win(qapp, monkeypatch):
    monkeypatch.setattr(AppSettings, "_ORGANIZATION", "RoboticsAnalyzer_test_modes")
    monkeypatch.setattr(AppSettings, "_APPLICATION", "RoboticsAnalyzer_test_modes")
    w = MainWindow()
    w.settings._settings.clear()
    w.resize(1600, 950)
    w.show()
    QCoreApplication.processEvents()
    yield w
    w.project_controller._is_modified = False
    w.sim_dock.hide()
    w.close()


def _docks_of(win, names):
    out = set()
    for n in names:
        action = getattr(win, "_action_view_" + n)
        out.add(next(d for a, d in win._view_action_docks() if a is action))
    return out


def _managed(win):
    return _docks_of(win, {n for _m, (_t, ds) in MODES.items() for n in ds})


@pytest.mark.parametrize("mode", MODE_NAMES)
def test_each_mode_shows_exactly_its_docks(win, mode):
    win.set_mode(mode)
    QCoreApplication.processEvents()
    want = _docks_of(win, MODES[mode][1])
    shown = {d for d in _managed(win) if not d.isHidden()}
    assert shown == want
    assert win.mode == mode and win._modes.action(mode).isChecked()


def test_button_click_and_shortcut_switch_mode(win):
    from PySide6.QtTest import QTest
    from PySide6.QtCore import Qt
    QTest.mouseClick(win.mode_bar.button("Plan"), Qt.MouseButton.LeftButton)
    assert win.mode == "Plan" and not win.planning_dock.isHidden()
    win._modes.action("Simulate").trigger()
    assert win.mode == "Simulate" and win.planning_dock.isHidden() and not win.sim_dock.isHidden()
    assert win._modes.action("Analyze").shortcut().toString() == "Ctrl+6"


def test_other_docks_stay_reachable_from_view_menu(win):
    win.set_mode("Move")
    win._action_view_graphs.trigger()
    QCoreApplication.processEvents()
    assert not win._graph_dock.isHidden() and win._action_view_graphs.isChecked()


def test_mode_leaves_focus_mode(win):
    win.set_focus_mode(True)
    win.set_mode("Analyze")
    assert not win.focus_mode and not win._workspace_dock.isHidden()


def test_target_mode_creates_target_at_tcp(win):
    load(win, "ur5")
    assert win.target_panel.current_index < 0
    win.set_mode("Target")
    assert win.target_panel.current_index == 0 and len(win.target_panel.store) == 1
    win.set_mode("Move")
    win.set_mode("Target")
    assert len(win.target_panel.store) == 1          # not again


def test_mode_saved_per_project(win):
    load(win, "ur3")
    project = win.project_controller.current_project
    project.file_path = str(os.path.join(os.getcwd(), "fake_u5_mode.robotproj"))
    win.set_mode("Simulate")
    assert win._modes.recall() == "Simulate"
    win.set_mode("Plan")
    assert win._modes.recall() == "Plan"
    project.file_path = str(os.path.join(os.getcwd(), "other_u5_mode.robotproj"))
    assert win._modes.recall() is None               # another project has no memory yet
    project.file_path = None
    project.file_path = str(os.path.join(os.getcwd(), "fake_u5_mode.robotproj"))
    win.set_mode("Move")
    win._on_project_opened(project)                  # opening restores the remembered mode
    assert win.mode == "Move"
    win.set_mode("Analyze")
    win._on_project_opened(project)
    assert win.mode == "Analyze"


def test_last_mode_is_restored_with_the_layout(qapp, monkeypatch):
    monkeypatch.setattr(AppSettings, "_ORGANIZATION", "RoboticsAnalyzer_test_modes2")
    monkeypatch.setattr(AppSettings, "_APPLICATION", "RoboticsAnalyzer_test_modes2")
    AppSettings()._settings.clear()
    w = MainWindow()
    w.show()
    w.set_mode("Simulate")
    w.project_controller._is_modified = False
    w.sim_dock.hide()
    w.close()
    w2 = MainWindow()
    w2.show()
    try:
        assert w2.mode == "Simulate" and w2._modes.action("Simulate").isChecked()
    finally:
        w2.project_controller._is_modified = False
        w2.close()
        AppSettings()._settings.clear()
