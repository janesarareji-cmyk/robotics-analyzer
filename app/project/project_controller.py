"""
app/project/project_controller.py
Robotics Analyzer — project lifecycle controller.
R1.6: Manages the active project: new, open, save, save-as, close.
Emits Qt signals on state changes so the UI can react.
"""

from __future__ import annotations

import logging
from pathlib import Path

from PySide6.QtCore import QObject, Signal
from PySide6.QtWidgets import QWidget, QFileDialog, QMessageBox, QInputDialog

from app.project.project_data import ProjectData
from app.project.serializer import (
    save as save_project_file,
    load as load_project_file,
    ProjectSerializationError,
    ProjectFormatError,
    ROBOTPROJ_EXTENSION,
)
from app.settings import AppSettings

log = logging.getLogger("robotics_analyzer.project")

FILE_FILTER = f"Robot Projects (*{ROBOTPROJ_EXTENSION});;All Files (*)"


class ProjectController(QObject):
    """
    Manages the active project.
    UI code connects to signals; it does NOT call serializer directly.

    Signals:
        project_opened(ProjectData)  — New or loaded project is now active.
        project_saved(ProjectData)   — Active project was written to disk.
        project_closed()             — Active project was closed.
        modified_changed(bool)       — Unsaved-changes flag changed.
        robot_changed(RobotModel)    — Active robot model changed.
    """

    project_opened = Signal(object)   # ProjectData
    project_saved = Signal(object)    # ProjectData
    project_closed = Signal()
    modified_changed = Signal(bool)
    robot_changed = Signal(object)    # cb.RobotModel

    def __init__(self, settings: AppSettings, parent: QObject | None = None):
        super().__init__(parent)
        self._settings = settings
        self._project: ProjectData | None = None
        self._is_modified: bool = False

    # ------------------------------------------------------------------
    # Public state
    # ------------------------------------------------------------------

    @property
    def current_project(self) -> ProjectData | None:
        """Return the active project, or None if no project is open."""
        return self._project

    @property
    def has_project(self) -> bool:
        """Return True if a project is currently open."""
        return self._project is not None

    @property
    def is_modified(self) -> bool:
        """Return True if the project has unsaved changes."""
        return self._is_modified

    @property
    def robot(self) -> Any:
        """Return the current robot model, or None if no project is open."""
        if self._project is None:
            return None
        return self._project.robot_model

    def set_robot(self, model: Any) -> None:
        """Set the robot model for the active project and mark dirty."""
        if self._project is None:
            return
        self._project.robot_model = model
        self.mark_modified()
        self.robot_changed.emit(model)

    def import_robot(self, parent_widget: QWidget, model: Any, name: str) -> bool:
        """
        Create a NEW unsaved project holding an imported robot model.
        Asks to discard if the current project is modified (Cancel -> False, nothing changes).
        """
        if not self._confirm_discard(parent_widget):
            return False
        project = ProjectData.new((name or "").strip() or "Imported Robot")
        project.robot_model = model
        self._set_project(project, modified=True)
        log.info("Robot imported into new project: %s", project.name)
        return True

    # ------------------------------------------------------------------
    # Project actions
    # ------------------------------------------------------------------

    def new_project(self, parent_widget: QWidget) -> bool:
        """
        Prompt for a project name and create a new project.
        Returns True if a new project was created, False if cancelled.
        """
        if not self._confirm_discard(parent_widget):
            return False

        name, ok = QInputDialog.getText(
            parent_widget,
            "New Project",
            "Project name:",
        )
        if not ok or not name.strip():
            return False

        project = ProjectData.new(name.strip())
        self._set_project(project, modified=True)
        log.info("New project created: %s", project.name)
        return True

    def open_project(self, parent_widget: QWidget, path: str | None = None) -> bool:
        """
        Open a project from disk.
        If path is None, show a file dialog.
        Returns True if a project was opened, False if cancelled or failed.
        """
        if not self._confirm_discard(parent_widget):
            return False

        if path is None:
            path, _ = QFileDialog.getOpenFileName(
                parent_widget,
                "Open Project",
                self._settings.last_project_dir(),
                FILE_FILTER,
            )
            if not path:
                return False

        try:
            project = load_project_file(path)
        except ProjectFormatError as exc:
            QMessageBox.critical(
                parent_widget,
                "Cannot Open Project",
                f"The file is not a valid Robotics Analyzer project.\n\n{exc}",
            )
            log.error("Format error opening %s: %s", path, exc)
            return False
        except ProjectSerializationError as exc:
            QMessageBox.critical(
                parent_widget,
                "Cannot Open Project",
                f"Failed to read the project file.\n\n{exc}",
            )
            log.error("I/O error opening %s: %s", path, exc)
            return False

        self._settings.add_recent_project(path)
        self._settings.set_last_project_dir(str(Path(path).parent))
        self._set_project(project, modified=False)
        log.info("Project opened: %s (%s)", project.name, path)
        return True

    def close_project(self, parent_widget: QWidget | None = None) -> bool:
        """
        Close the active project.
        Prompts to save if modified.
        Returns True if closed, False if cancelled.
        """
        if self._project is None:
            return True
        if parent_widget is not None and not self._confirm_discard(parent_widget):
            return False

        self._project = None
        self._is_modified = False
        self.project_closed.emit()
        self.modified_changed.emit(False)
        log.info("Project closed.")
        return True

    def save_project(self, parent_widget: QWidget) -> bool:
        """
        Save the current project.
        If no file path is set, delegates to save_project_as().
        Returns True if saved, False if cancelled or failed.
        """
        if self._project is None:
            return False
        if self._project.file_path is None:
            return self.save_project_as(parent_widget)
        return self._save_to_path(parent_widget, self._project.file_path)

    def save_project_as(
        self, parent_widget: QWidget, path: str | None = None
    ) -> bool:
        """
        Prompt for a save path (or use provided path) and save the current project there.
        Returns True if saved, False if cancelled or failed.
        """
        if self._project is None:
            return False

        if path is None:
            default_name = (
                self._project.name.replace(" ", "_") + ROBOTPROJ_EXTENSION
            )
            default_dir = self._settings.last_project_dir()
            default_path = str(Path(default_dir) / default_name)

            path, _ = QFileDialog.getSaveFileName(
                parent_widget,
                "Save Project As",
                default_path,
                FILE_FILTER,
            )
            if not path:
                return False

        if not path.endswith(ROBOTPROJ_EXTENSION):
            path += ROBOTPROJ_EXTENSION

        return self._save_to_path(parent_widget, path)

    def mark_modified(self) -> None:
        """Mark the current project as having unsaved changes."""
        if not self._is_modified:
            self._is_modified = True
            self.modified_changed.emit(True)

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _save_to_path(self, parent_widget: QWidget, path: str) -> bool:
        """Save the current project to path. Returns True on success."""
        assert self._project is not None
        try:
            save_project_file(self._project, path)
        except ProjectSerializationError as exc:
            QMessageBox.critical(
                parent_widget,
                "Cannot Save Project",
                f"Failed to write the project file.\n\n{exc}",
            )
            log.error("I/O error saving %s: %s", path, exc)
            return False

        self._settings.add_recent_project(path)
        self._settings.set_last_project_dir(str(Path(path).parent))
        self._is_modified = False
        self.modified_changed.emit(False)
        self.project_saved.emit(self._project)
        log.info("Project saved: %s (%s)", self._project.name, path)
        return True

    def _set_project(self, project: ProjectData, modified: bool) -> None:
        """Set the active project and emit project_opened."""
        self._project = project
        self._is_modified = modified
        self.project_opened.emit(project)
        self.modified_changed.emit(modified)

    def _confirm_discard(self, parent_widget: QWidget) -> bool:
        """
        If there are unsaved changes, ask the user whether to discard them.
        Returns True if it is safe to proceed, False if the user cancels.
        """
        if not self._is_modified or self._project is None:
            return True

        answer = QMessageBox.question(
            parent_widget,
            "Unsaved Changes",
            f"Project '{self._project.name}' has unsaved changes.\n"
            "Discard changes and continue?",
            QMessageBox.StandardButton.Discard | QMessageBox.StandardButton.Cancel,
            QMessageBox.StandardButton.Cancel,
        )
        return answer == QMessageBox.StandardButton.Discard
