"""
tests/python/test_ui_layout.py
U1: viewport-first layout, side bar, Focus 3D, Reset layout, layout version.
"""

import os

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtCore import QSettings
from PySide6.QtWidgets import QApplication, QDockWidget, QToolBar

from app.main_window import MainWindow
from app.settings import LAYOUT_VERSION, AppSettings

ORG = "RoboticsAnalyzer_test_layout"


@pytest.fixture(autouse=True)
def isolated_settings(qapp, monkeypatch):
    monkeypatch.setattr(AppSettings, "_ORGANIZATION", ORG)
    monkeypatch.setattr(AppSettings, "_APPLICATION", ORG)
    s = QSettings(ORG, ORG)
    s.clear()
    s.sync()
    yield
    s.clear()
    s.sync()


def _make(w=1400, h=900) -> MainWindow:
    win = MainWindow()
    win.resize(w, h)
    win.show()
    QApplication.processEvents()
    return win


def _close(win):
    win.project_controller._is_modified = False
    win.close()


def _visible(win) -> set:
    return {d.objectName() for d in win.findChildren(QDockWidget) if not d.isHidden()}


def test_default_viewport_is_large():
    win = _make(1700, 1000)
    try:
        c = win.centralWidget()
        assert c.width() >= 0.55 * win.width()
        assert c.height() >= 0.55 * win.height()
        assert _visible(win) == {
            "model_dock", "props_dock", "dh_dock", "joint_dock", "log_dock"}
        # Properties tabbed into the left area, right area holds Joints (raised) + DH
        assert win.dockWidgetArea(win.findChild(QDockWidget, "props_dock")).name == "LeftDockWidgetArea"
        assert win.dockWidgetArea(win.joint_dock).name == "RightDockWidgetArea"
        assert win.joint_dock.isVisible() and win.findChild(QDockWidget, "model_dock").isVisible()
    finally:
        _close(win)


def test_focus_mode_hides_all_and_restores():
    win = _make()
    try:
        before = _visible(win)
        win.set_focus_mode(True)
        QApplication.processEvents()
        assert win.focus_mode
        assert _visible(win) == set()
        assert win._action_focus.isChecked()
        side = win.findChild(QToolBar, "tool_bar_side")
        assert win.centralWidget().width() >= win.width() - side.width() - 20
        win.set_focus_mode(False)
        QApplication.processEvents()
        assert not win.focus_mode
        assert _visible(win) == before
        assert not win._action_focus.isChecked()
    finally:
        _close(win)


def test_focus_action_toggle_and_shortcut():
    win = _make()
    try:
        assert win._action_focus.shortcut().toString() == "F11"
        win._action_focus.trigger()
        assert win.focus_mode
        win._action_focus.trigger()
        assert not win.focus_mode
    finally:
        _close(win)


def test_opening_dock_in_focus_mode_leaves_focus():
    win = _make()
    try:
        win.set_focus_mode(True)
        win._action_view_ik.trigger()
        QApplication.processEvents()
        assert not win.focus_mode
        assert not win._action_focus.isChecked()
        ik = win._ik_dock
        # raised = current tab: Joints no longer the visible tab of the right area
        assert ik.isVisible() and ik.width() > 0 and ik.height() > 0
    finally:
        _close(win)


def test_click_raises_tabbed_dock():
    win = _make()
    try:
        win._action_view_analysis.trigger()
        QApplication.processEvents()
        assert win._analysis_dock.isVisible()
        assert win._analysis_dock.height() > 100        # current tab, laid out
        assert win._action_view_analysis.isChecked()
        from PySide6.QtWidgets import QTabBar
        current = [tb.tabText(tb.currentIndex()) for tb in win.findChildren(QTabBar)
                   if tb.isVisible() and any(tb.tabText(i) == "Analysis" for i in range(tb.count()))]
        assert "Analysis" in current                     # raised = current tab
    finally:
        _close(win)


def test_reset_layout_restores_default():
    win = _make()
    try:
        default = _visible(win)
        win._action_view_ik.trigger()
        win._action_view_graphs.trigger()
        win._action_view_log.trigger()                   # hides the log
        win.set_focus_mode(True)
        win.reset_layout()
        QApplication.processEvents()
        assert not win.focus_mode
        assert _visible(win) == default
        assert not win._action_focus.isChecked()
    finally:
        _close(win)


def test_old_layout_version_is_ignored():
    # Save a state with IK shown, under various versions.
    win = _make()
    win._action_view_ik.trigger()
    QApplication.processEvents()
    win.settings.save_window(win)
    win.settings.sync()
    _close(win)
    for stored in (None, 1):
        s = QSettings(ORG, ORG)
        if stored is None:
            s.remove(AppSettings.KEY_LAYOUT_VERSION)
        else:
            s.setValue(AppSettings.KEY_LAYOUT_VERSION, stored)
        s.sync()
        w2 = _make()
        try:
            assert "ik_dock" not in _visible(w2)
            assert w2.settings.layout_version() == LAYOUT_VERSION
        finally:
            _close(w2)
        # restoring a fresh old state again
        s.setValue(AppSettings.KEY_WINDOW_STATE, w2.saveState())
        s.sync()


def test_current_layout_version_is_restored():
    win = _make()
    win._action_view_ik.trigger()
    QApplication.processEvents()
    win.settings.save_window(win)
    win.settings.sync()
    assert win.settings.layout_version() == LAYOUT_VERSION == 2
    _close(win)
    w2 = _make()
    try:
        assert "ik_dock" in _visible(w2)
    finally:
        _close(w2)


def test_settings_layout_version_api():
    s = AppSettings()
    assert s.layout_version() == 0
    s.set_layout_version(5)
    assert s.layout_version() == 5


def test_side_bar_actions():
    win = _make()
    try:
        bar = win.findChild(QToolBar, "tool_bar_side")
        assert bar is not None
        assert win.toolBarArea(bar).name == "LeftToolBarArea"
        assert not bar.isMovable()
        labels = [a.iconText() for a in bar.actions() if not a.isSeparator()]
        assert labels == ["Model", "Joints", "DH", "IK", "Analysis", "Workspace", "Sim",
                          "Graphs", "Planning", "Ghosts", "Collision", "Physical",
                          "Script", "Log", "Focus 3D", "Reset"]
        assert win._action_view_ik in bar.actions()
        assert win._action_view_ik.text() == "&Inverse Kinematics"   # menu text unchanged
        assert win._action_focus in bar.actions()
        assert win._action_reset_layout in bar.actions()
    finally:
        _close(win)


def test_close_in_focus_mode_saves_normal_layout():
    win = _make()
    win.set_focus_mode(True)
    QApplication.processEvents()
    assert win.focus_mode
    _close(win)
    assert win.focus_mode is False
    w2 = _make()
    try:
        assert not w2.focus_mode
        vis = _visible(w2)
        assert "model_dock" in vis and "joint_dock" in vis
    finally:
        _close(w2)


def test_side_bar_checked_only_for_current_tab():
    win = _make()
    try:
        assert win.joint_dock.width() >= 360
        win._action_view_ik.trigger()
        QApplication.processEvents()
        win._action_view_graphs.trigger()
        QApplication.processEvents()
        assert win._action_view_graphs.isChecked()
        assert not win._action_view_ik.isChecked()
        assert not win._action_view_joints.isChecked()
        assert not win._ik_dock.isHidden() and not win.joint_dock.isHidden()
        win._action_view_ik.trigger()          # covered but open: raise, do not hide
        QApplication.processEvents()
        assert win._action_view_ik.isChecked()
        assert not win._action_view_graphs.isChecked()
        assert not win._graph_dock.isHidden()
        assert not win._action_view_joints.isChecked()
    finally:
        _close(win)
