"""U5.9: F1 help overlay and the first-run tour."""
import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest
from PySide6.QtCore import Qt
from PySide6.QtTest import QTest

from app.help import PANEL_HELP, SHORTCUTS, TOUR_STEPS, help_html
from app.main_window import MainWindow
from app.modes import MODES
from app.settings import AppSettings


@pytest.fixture
def win(qapp, monkeypatch):
    monkeypatch.setattr(AppSettings, "_ORGANIZATION", "RoboticsAnalyzer_test_help")
    monkeypatch.setattr(AppSettings, "_APPLICATION", "RoboticsAnalyzer_test_help")
    w = MainWindow()
    w.settings._settings.clear()
    w.resize(1400, 900)
    w.show()
    yield w
    w.project_controller._is_modified = False
    w.sim_dock.hide()
    w.close()


def test_f1_opens_help_with_shortcuts_and_current_mode(win):
    assert win._action_help.shortcut().toString() == "F1"
    win.set_mode("Plan")
    win._action_help.trigger()
    d = win._help_dialog
    assert d is not None and d.isVisible()
    t = d.text
    for keys, _what in SHORTCUTS:
        assert keys in t
    assert "The Plan mode" in t and "Planning" in t
    assert "Ctrl+1" in t and "Ctrl+Z" in t
    win.set_mode("Move")
    win.show_help()
    assert "The Move mode" in d.text
    d.close()


def test_every_dock_of_every_mode_has_help_text():
    for mode, (_tip, docks) in MODES.items():
        for name in docks:
            assert PANEL_HELP.get(name), f"no help for {name}"
    assert "The Analyze mode" in help_html("Analyze")


def test_tour_has_four_steps_and_is_shown_once(win):
    assert len(TOUR_STEPS) == 4
    assert win.maybe_start_tour() is True               # first run
    card = win._tour
    assert card.isVisible() and card.index == 0 and not card.btn_back.isEnabled()
    QTest.mouseClick(card.btn_next, Qt.MouseButton.LeftButton)
    QTest.mouseClick(card.btn_next, Qt.MouseButton.LeftButton)
    assert card.index == 2 and card.btn_back.isEnabled()
    QTest.mouseClick(card.btn_back, Qt.MouseButton.LeftButton)
    assert card.index == 1
    QTest.mouseClick(card.btn_next, Qt.MouseButton.LeftButton)
    QTest.mouseClick(card.btn_next, Qt.MouseButton.LeftButton)
    assert card.btn_next.text() == "Done"
    QTest.mouseClick(card.btn_next, Qt.MouseButton.LeftButton)
    assert not card.isVisible()
    assert win.maybe_start_tour() is False              # not again
    assert not card.isVisible()


def test_tour_can_be_skipped_and_stays_off(win):
    assert win.maybe_start_tour()
    QTest.mouseClick(win._tour.btn_skip, Qt.MouseButton.LeftButton)
    assert not win._tour.isVisible()
    assert win.maybe_start_tour() is False
    win.start_tour()                                    # but Help can show it again
    assert win._tour.isVisible()
