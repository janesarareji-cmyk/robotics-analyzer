"""
tests/python/test_settings.py
R1.2: AppSettings unit tests.
"""

import os
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtCore import QSettings
from app.settings import AppSettings


@pytest.fixture(autouse=True)
def clean_settings():
    """Clear test settings before and after each test."""
    s = QSettings("RoboticsAnalyzer_test", "RoboticsAnalyzer_test")
    s.clear()
    s.sync()
    yield
    s.clear()
    s.sync()


@pytest.fixture
def settings():
    """Return a fresh AppSettings instance for testing."""
    # Patch to use test organization to avoid polluting real settings
    original_org = AppSettings._ORGANIZATION
    original_app = AppSettings._APPLICATION
    AppSettings._ORGANIZATION = "RoboticsAnalyzer_test"
    AppSettings._APPLICATION = "RoboticsAnalyzer_test"
    s = AppSettings()
    yield s
    AppSettings._ORGANIZATION = original_org
    AppSettings._APPLICATION = original_app


def test_settings_creation(settings):
    """AppSettings creates without error."""
    assert settings is not None


def test_settings_log_level_default(settings):
    """Default log level is INFO."""
    assert settings.log_level() == "INFO"


def test_settings_log_level_set(settings):
    """Setting log level persists and reads back correctly."""
    settings.set_log_level("DEBUG")
    assert settings.log_level() == "DEBUG"


def test_settings_recent_projects_empty(settings):
    """Recent projects list is empty by default."""
    assert settings.recent_projects() == []


def test_settings_add_recent_project(settings):
    """Adding a recent project stores it at the front of the list."""
    settings.add_recent_project("C:/projects/robot1.robotproj")
    settings.add_recent_project("C:/projects/robot2.robotproj")
    recent = settings.recent_projects()
    assert recent[0] == "C:/projects/robot2.robotproj"
    assert recent[1] == "C:/projects/robot1.robotproj"


def test_settings_recent_project_deduplication(settings):
    """Adding the same project twice does not create duplicates."""
    settings.add_recent_project("C:/projects/robot1.robotproj")
    settings.add_recent_project("C:/projects/robot1.robotproj")
    recent = settings.recent_projects()
    assert recent.count("C:/projects/robot1.robotproj") == 1


def test_settings_viewport_bg_color_default(settings):
    """Default viewport background color is set."""
    color = settings.viewport_bg_color()
    assert color.startswith("#")
    assert len(color) == 7


def test_settings_viewport_bg_color_set(settings):
    """Setting viewport background color persists."""
    settings.set_viewport_bg_color("#112233")
    assert settings.viewport_bg_color() == "#112233"


def test_settings_show_grid_default(settings):
    """Show grid defaults to True."""
    assert settings.show_grid() is True


def test_settings_show_grid_set(settings):
    """Setting show_grid to False persists."""
    settings.set_show_grid(False)
    assert settings.show_grid() is False


def test_settings_show_coordinate_frames_default(settings):
    """Show coordinate frames defaults to True."""
    assert settings.show_coordinate_frames() is True


def test_settings_generic_get_set(settings):
    """Generic get/set works for custom keys."""
    settings.set("custom/test_key", "hello")
    assert settings.get("custom/test_key") == "hello"


def test_main_window_has_settings(qapp):
    """MainWindow exposes an AppSettings instance via .settings property."""
    from app.main_window import MainWindow
    window = MainWindow()
    assert window.settings is not None
    assert isinstance(window.settings, AppSettings)
    window.close()
