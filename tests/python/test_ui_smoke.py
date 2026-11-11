"""
tests/python/test_ui_smoke.py
R1.1: Main window smoke tests — verifies shell with menu, toolbar, panels.
"""

import os
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication
from app.main import create_app
from app.main_window import MainWindow


def test_pyside6_import():
    """PySide6 imports without error."""
    from PySide6.QtWidgets import QApplication, QMainWindow
    from PySide6.QtCore import Qt
    assert QApplication is not None
    assert QMainWindow is not None


def test_main_window_creation(qapp):
    """MainWindow creates without raising an exception."""
    window = MainWindow()
    assert window is not None
    assert "Robotics Analyzer" in window.windowTitle()
    window.close()


def test_main_window_minimum_size(qapp):
    """MainWindow minimum size is at least 1024x768."""
    window = MainWindow()
    assert window.minimumWidth() >= 1024
    assert window.minimumHeight() >= 768
    window.close()


def test_main_window_has_menu_bar(qapp):
    """MainWindow has a menu bar with File, View, Help menus."""
    window = MainWindow()
    menubar = window.menuBar()
    assert menubar is not None
    menu_titles = [menubar.actions()[i].text() for i in range(len(menubar.actions()))]
    assert any("File" in t for t in menu_titles)
    assert any("View" in t for t in menu_titles)
    assert any("Help" in t for t in menu_titles)
    window.close()


def test_main_window_has_status_bar(qapp):
    """MainWindow has a status bar."""
    window = MainWindow()
    assert window.statusBar() is not None
    window.close()


def test_main_window_has_log_panel(qapp):
    """MainWindow log panel exists and accepts messages."""
    window = MainWindow()
    window.log("[TEST] Log panel works.")
    assert window.log_panel is not None
    window.close()


def test_main_window_set_status(qapp):
    """MainWindow set_status updates the status bar."""
    window = MainWindow()
    window.set_status("Test status message")
    window.close()


def test_main_window_log_panel_accepts_messages(qapp):
    """MainWindow log panel accepts append calls without error."""
    window = MainWindow()
    window.log_panel.append("[TEST] Direct panel message.")
    window.log_panel.append("[TEST] Second message.")
    window.close()

