"""
tests/python/test_project_controller.py
R1.6: ProjectController unit tests.
Tests use a mock parent_widget to avoid real file dialogs.
"""

import os
import pytest
from pathlib import Path
from unittest.mock import patch, MagicMock

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtCore import QSettings
from app.settings import AppSettings
from app.project.project_controller import ProjectController
from app.project.project_data import ProjectData
from app.project.serializer import ROBOTPROJ_EXTENSION


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
def test_settings():
    """Return AppSettings using test organization."""
    original_org = AppSettings._ORGANIZATION
    original_app = AppSettings._APPLICATION
    AppSettings._ORGANIZATION = "RoboticsAnalyzer_test"
    AppSettings._APPLICATION = "RoboticsAnalyzer_test"
    s = AppSettings()
    yield s
    AppSettings._ORGANIZATION = original_org
    AppSettings._APPLICATION = original_app


@pytest.fixture
def controller(qapp, test_settings):
    """Return a fresh ProjectController for testing."""
    return ProjectController(settings=test_settings)


@pytest.fixture
def mock_widget():
    """Return a mock parent widget for dialogs."""
    return MagicMock()


def test_controller_creation(controller):
    """ProjectController creates without error."""
    assert controller is not None
    assert not controller.has_project
    assert controller.current_project is None
    assert not controller.is_modified


def test_new_project_creates_project(controller, mock_widget):
    """new_project() creates a ProjectData when user enters a name."""
    with patch("app.project.project_controller.QInputDialog.getText",
               return_value=("TestRobot", True)):
        result = controller.new_project(mock_widget)

    assert result is True
    assert controller.has_project
    assert controller.current_project is not None
    assert controller.current_project.name == "TestRobot"
    assert controller.is_modified  # New project is modified (unsaved)


def test_new_project_cancelled(controller, mock_widget):
    """new_project() returns False if user cancels the dialog."""
    with patch("app.project.project_controller.QInputDialog.getText",
               return_value=("", False)):
        result = controller.new_project(mock_widget)

    assert result is False
    assert not controller.has_project


def test_new_project_empty_name_cancelled(controller, mock_widget):
    """new_project() returns False if user enters an empty name."""
    with patch("app.project.project_controller.QInputDialog.getText",
               return_value=("   ", True)):
        result = controller.new_project(mock_widget)

    assert result is False
    assert not controller.has_project


def test_new_project_emits_project_opened(controller, mock_widget):
    """new_project() emits the project_opened signal."""
    received = []
    controller.project_opened.connect(received.append)

    with patch("app.project.project_controller.QInputDialog.getText",
               return_value=("Robot1", True)):
        controller.new_project(mock_widget)

    assert len(received) == 1
    assert isinstance(received[0], ProjectData)


def test_open_project_loads_file(controller, mock_widget, tmp_path):
    """open_project() loads a valid .robotproj file."""
    # Create a project file first
    from app.project.serializer import save as save_proj
    proj = ProjectData.new("LoadTest")
    path = tmp_path / "load_test.robotproj"
    save_proj(proj, path)

    # Open it via controller
    with patch("app.project.project_controller.QFileDialog.getOpenFileName",
               return_value=(str(path), "")):
        result = controller.open_project(mock_widget)

    assert result is True
    assert controller.has_project
    assert controller.current_project.name == "LoadTest"
    assert not controller.is_modified  # Opened project is not modified


def test_save_project_saves_to_file(controller, mock_widget, tmp_path):
    """save_project() saves the current project to disk."""
    # Create a new project
    with patch("app.project.project_controller.QInputDialog.getText",
               return_value=("SaveTest", True)):
        controller.new_project(mock_widget)

    path = tmp_path / "saved.robotproj"

    # Mock the save-as dialog
    with patch("app.project.project_controller.QFileDialog.getSaveFileName",
               return_value=(str(path), "")):
        result = controller.save_project(mock_widget)

    assert result is True
    assert path.exists()
    assert controller.current_project.file_path == str(path)
    assert not controller.is_modified  # Saved project is not modified


def test_save_project_emits_project_saved(controller, mock_widget, tmp_path):
    """save_project() emits the project_saved signal."""
    saved = []
    controller.project_saved.connect(saved.append)

    with patch("app.project.project_controller.QInputDialog.getText",
               return_value=("SigTest", True)):
        controller.new_project(mock_widget)

    path = tmp_path / "sig_test.robotproj"
    with patch("app.project.project_controller.QFileDialog.getSaveFileName",
               return_value=(str(path), "")):
        controller.save_project(mock_widget)

    assert len(saved) == 1


def test_mark_modified_emits_signal(controller, mock_widget):
    """mark_modified() emits modified_changed signal."""
    changes = []
    controller.modified_changed.connect(changes.append)

    with patch("app.project.project_controller.QInputDialog.getText",
               return_value=("Mod", True)):
        controller.new_project(mock_widget)

    # Clear recorded changes from new_project
    changes.clear()
    controller._is_modified = False

    controller.mark_modified()
    assert True in changes


def test_main_window_has_project_controller(qapp):
    """MainWindow exposes a ProjectController via .project_controller."""
    from app.main_window import MainWindow
    from app.project.project_controller import ProjectController
    window = MainWindow()
    assert isinstance(window.project_controller, ProjectController)
    window.close()


def test_main_window_title_updates_on_new_project(qapp):
    """MainWindow title includes project name after new project created."""
    from app.main_window import MainWindow
    window = MainWindow()

    with patch("app.project.project_controller.QInputDialog.getText",
               return_value=("TitleTest", True)):
        window.project_controller.new_project(window)

    assert "TitleTest" in window.windowTitle()
    window.close()
