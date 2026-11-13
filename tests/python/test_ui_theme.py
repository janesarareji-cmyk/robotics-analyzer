"""U5.10: one theme (Fusion + blue accent), grouped icon toolbar with shortcut tooltips, plain labels."""
import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest
from PySide6.QtGui import QColor, QPalette
from PySide6.QtWidgets import QApplication, QLabel, QToolBar, QToolButton

from app import theme
from app.main_window import MainWindow
from app.settings import AppSettings
from tests.python.test_live_blocking import load


@pytest.fixture
def win(qapp, monkeypatch):
    monkeypatch.setattr(AppSettings, "_ORGANIZATION", "RoboticsAnalyzer_test_theme")
    monkeypatch.setattr(AppSettings, "_APPLICATION", "RoboticsAnalyzer_test_theme")
    w = MainWindow()
    w.settings._settings.clear()
    w.resize(1500, 900)
    w.show()
    yield w
    w.project_controller._is_modified = False
    w.sim_dock.hide()
    w.close()


def _lum(c: QColor) -> float:
    def ch(v):
        v /= 255.0
        return v / 12.92 if v <= 0.03928 else ((v + 0.055) / 1.055) ** 2.4
    return 0.2126 * ch(c.red()) + 0.7152 * ch(c.green()) + 0.0722 * ch(c.blue())


def ratio(a: str, b: str) -> float:
    la, lb = _lum(QColor(a)), _lum(QColor(b))
    hi, lo = max(la, lb), min(la, lb)
    return (hi + 0.05) / (lo + 0.05)


def test_theme_is_fusion_with_one_accent(win):
    app = QApplication.instance()
    assert app.style().name().lower() == "fusion"
    assert app.palette().color(QPalette.ColorRole.Highlight).name() == theme.ACCENT


def test_text_contrast_meets_wcag_aa():
    assert ratio(theme.TEXT, theme.WINDOW) >= 4.5
    assert ratio(theme.MUTED, theme.WINDOW) >= 4.5
    assert ratio("#ffffff", theme.ACCENT) >= 4.5
    assert ratio("#d0d0d0", "#202020") >= 4.5               # mode bar text
    assert ratio("#ffffff", "#2a2f3a") >= 4.5               # checked mode button
    assert ratio("#e8e8f0", "#24243a") >= 4.5               # start screen card
    assert ratio("#c8c8d8", "#24243a") >= 4.5               # tour skip text
    assert ratio("#a8a8a8", "#1e1e1e") >= 4.5               # toolbar group captions


def test_toolbar_groups_icons_and_tooltips(win):
    bar = win.findChild(QToolBar, "main_toolbar")
    assert bar is not None and bar.toolButtonStyle().name == "ToolButtonTextBesideIcon"
    labels = [w.text() for w in bar.findChildren(QLabel) if w.accessibleName().endswith(" group")]
    assert labels == ["Move", "Scene", "View"]
    for act in (win._action_new, win._action_open, win._action_save, win._action_drag_tcp,
                win._action_collision, win._action_reset_camera, win._action_focus):
        assert not act.icon().isNull(), act.text()
        assert act in bar.actions()
    assert "Ctrl+N" in win._action_new.toolTip() and "F11" in win._action_focus.toolTip()
    assert win._action_undo in bar.actions() and win._action_redo in bar.actions()
    btn = bar.widgetForAction(win._action_new)
    assert isinstance(btn, QToolButton)
    assert not win._sample_button.icon().isNull() and not win._objects_button.icon().isNull()


def test_status_bar_shows_real_compute_backend(win):
    assert win._gpu_label.text() in ("Compute: GPU (CUDA)", "Compute: CPU")
    assert "RTX" not in win._gpu_label.text()


def test_planning_table_uses_joint_names(win):
    load(win, "ur5")
    table = win.planning_panel._table
    heads = [table.verticalHeaderItem(i).text() for i in range(table.rowCount())]
    assert heads == [r.name_label.text() for r in win.joint_panel.rows]
    assert "shoulder_pan_joint" in heads[0]
