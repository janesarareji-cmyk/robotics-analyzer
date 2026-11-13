"""U5.5: start screen over the 3D view until a robot is loaded."""
import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest
from PySide6.QtCore import QCoreApplication, Qt
from PySide6.QtTest import QTest

from app.main_window import MainWindow
from app.sample_robots import list_sample_robots
from app.settings import AppSettings


@pytest.fixture
def win(qapp, monkeypatch):
    monkeypatch.setattr(AppSettings, "_ORGANIZATION", "RoboticsAnalyzer_test_start")
    monkeypatch.setattr(AppSettings, "_APPLICATION", "RoboticsAnalyzer_test_start")
    w = MainWindow()
    w.settings._settings.clear()
    w.resize(1500, 900)
    w.show()
    QCoreApplication.processEvents()
    yield w
    w.project_controller._is_modified = False
    w.sim_dock.hide()
    w.close()


def test_start_screen_shows_nine_samples_and_covers_the_view(win):
    s = win.start_screen
    assert s.isVisible()
    assert len(s.card_stems) == 9 == len(list_sample_robots())
    assert s.geometry() == win.viewport.rect()
    assert s.btn_import.isVisible() and s.btn_open.isVisible()
    from pathlib import Path
    from app.sample_robots import sample_robots_dir
    assert all((sample_robots_dir() / st / "thumb.png").is_file() for st in s.card_stems)


def test_click_card_loads_robot_and_hides_screen(win):
    QTest.mouseClick(win.start_screen.card("ur3"), Qt.MouseButton.LeftButton)
    assert win.task_runner.wait_all(30000)
    QCoreApplication.processEvents()
    assert win.project_controller.robot is not None
    assert not win.start_screen.isVisible()
    win.project_controller._is_modified = False
    win.project_controller.close_project(win)
    QCoreApplication.processEvents()
    assert win.start_screen.isVisible()


def test_buttons_call_import_and_open(win, monkeypatch):
    calls = []
    monkeypatch.setattr(MainWindow, "_on_import_urdf", lambda self: calls.append("import"))
    monkeypatch.setattr(MainWindow, "_on_open_project", lambda self: calls.append("open"))
    win2 = MainWindow()           # connections bind the patched methods
    win2.settings._settings.clear()
    win2.show()
    QTest.mouseClick(win2.start_screen.btn_import, Qt.MouseButton.LeftButton)
    QTest.mouseClick(win2.start_screen.btn_open, Qt.MouseButton.LeftButton)
    assert calls == ["import", "open"]
    win2.close()


def test_recent_projects_listed_and_open(win, monkeypatch):
    win.settings.add_recent_project("D:/x/one.robotproj")
    win.settings.add_recent_project("D:/x/two.robotproj")
    opened = []
    monkeypatch.setattr(win.project_controller, "open_project", lambda parent, p=None: opened.append(p) or True)
    win._refresh_start_screen()
    bs = win.start_screen.recent_buttons
    assert [b.toolTip() for b in bs] == ["D:/x/two.robotproj", "D:/x/one.robotproj"]
    QTest.mouseClick(bs[0], Qt.MouseButton.LeftButton)
    assert opened == ["D:/x/two.robotproj"]
