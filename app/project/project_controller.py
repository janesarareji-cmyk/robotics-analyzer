"""
app/project/project_controller.py
Robotics Analyzer — project lifecycle controller.
R1.6: Manages the active project: new, open, save, save-as, close.
Emits Qt signals on state changes so the UI can react.
"""

from __future__ import annotations

import logging
from pathlib import Path

from PySide6.QtCore import QObject, QTimer, Signal
from PySide6.QtWidgets import QWidget, QFileDialog, QMessageBox, QInputDialog

import json
import zipfile

from app.project.migrations import is_older, make_backup
from app.project.project_data import ProjectData
from app.project.recovery import (
    load_best,
    newer_autosave,
    remove_autosave,
    write_autosave,
)
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

    def __init__(self, settings: AppSettings, parent: QObject | None = None,
                 autosave_interval_ms: int | None = None):
        super().__init__(parent)
        self._settings = settings
        self._project: ProjectData | None = None
        self._is_modified: bool = False
        # R8.4 autosave: interval from the setting (default 120 s, min 15 s);
        # autosave_interval_ms is an injection point for tests.
        self._autosave_ms_override = autosave_interval_ms
        self._autosave_timer = QTimer(self)
        self._autosave_timer.timeout.connect(self.autosave_now)

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

    def import_robot(self, parent_widget: QWidget, model: Any, name: str,
                     visuals: Any = None, collisions: Any = None) -> bool:
        """
        Create a NEW unsaved project holding an imported robot model.
        Asks to discard if the current project is modified (Cancel -> False, nothing changes).
        """
        if not self._confirm_discard(parent_widget):
            return False
        project = ProjectData.new((name or "").strip() or "Imported Robot")
        project.robot_model = model
        project.visuals = list(visuals or [])
        project.collisions = list(collisions or [])
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

        # R8.4: old-format file -> backup once, BEFORE anything is migrated
        old_version = _peek_old_version(path)
        if old_version is not None:
            make_backup(path, old_version)

        recovered = False
        used_prev = False
        autosave = newer_autosave(path)
        if autosave is not None:
            answer = QMessageBox.question(
                parent_widget,
                "Recover Project",
                f"An autosave newer than '{Path(path).name}' was found "
                "(the application may have closed unexpectedly).\n\nRecover it?",
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                QMessageBox.StandardButton.Yes,
            )
            if answer == QMessageBox.StandardButton.Yes:
                recovered = True
            else:
                remove_autosave(path)

        try:
            if recovered:
                project = load_project_file(autosave)
                project.file_path = str(path)
            else:
                project, opened = load_best(path)
                used_prev = Path(opened) != Path(path)
                if used_prev:
                    project.file_path = str(path)
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

        if used_prev:
            QMessageBox.warning(
                parent_widget,
                "Previous Copy Opened",
                f"'{Path(path).name}' could not be read; the previous copy "
                "(.prev) was opened instead. Save to repair the file.",
            )
        self._settings.add_recent_project(path)
        self._settings.set_last_project_dir(str(Path(path).parent))
        self._set_project(project, modified=recovered or used_prev)
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

        self._autosave_timer.stop()
        if self._project.file_path:
            remove_autosave(self._project.file_path)
        self._project.cleanup_assets()
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
    # Autosave and recovery (R8.4)
    # ------------------------------------------------------------------

    @property
    def autosave_interval_ms(self) -> int:
        if self._autosave_ms_override is not None:
            return int(self._autosave_ms_override)
        return int(self._settings.autosave_interval_s()) * 1000

    def set_autosave_interval_ms(self, ms: int | None) -> None:
        """Override the interval (tests); None goes back to the setting."""
        self._autosave_ms_override = ms
        if self._autosave_timer.isActive():
            self._autosave_timer.start(self.autosave_interval_ms)

    @property
    def autosave_active(self) -> bool:
        """True while the autosave timer runs (only while a project is open)."""
        return self._autosave_timer.isActive()

    def autosave_now(self) -> Path | None:
        """
        Write <name>.autosave.robotproj atomically. Only if a project with a file
        path is open AND modified. Never shows a dialog. Returns the path or None.
        """
        project = self._project
        if project is None or not self._is_modified or not project.file_path:
            return None
        try:
            return write_autosave(project, project.file_path)
        except ProjectSerializationError as exc:
            log.warning("Autosave failed: %s", exc)
            return None

    def check_startup_recovery(self, parent_widget: QWidget) -> bool:
        """
        Startup check: if a recent project has an autosave newer than itself,
        open it (the open path asks "Recover?"). Returns True if a project was opened.
        """
        if self._project is not None:
            return False
        for recent in self._settings.recent_projects():
            if newer_autosave(recent) is not None:
                return self.open_project(parent_widget, recent)
        return False

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _save_to_path(self, parent_widget: QWidget, path: str) -> bool:
        """Save the current project to path. Returns True on success."""
        assert self._project is not None
        old_path = self._project.file_path
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

        remove_autosave(path)
        if old_path and Path(old_path) != Path(path):
            remove_autosave(old_path)
        self._settings.add_recent_project(path)
        self._settings.set_last_project_dir(str(Path(path).parent))
        self._is_modified = False
        self.modified_changed.emit(False)
        self.project_saved.emit(self._project)
        log.info("Project saved: %s (%s)", self._project.name, path)
        return True

    def _set_project(self, project: ProjectData, modified: bool) -> None:
        """Set the active project and emit project_opened."""
        old = self._project
        if old is not None and old is not project:
            # A1.1: the replaced project is gone (saved or discarded): drop its leftovers
            if old.file_path and (not project.file_path
                                  or Path(old.file_path) != Path(project.file_path)):
                remove_autosave(old.file_path)
            old.cleanup_assets()
        self._project = project
        self._is_modified = modified
        self._autosave_timer.start(self.autosave_interval_ms)
        self.project_opened.emit(project)
        self.modified_changed.emit(modified)

    def _confirm_discard(self, parent_widget: QWidget) -> bool:
        """
        If there are unsaved changes, ask the user whether to discard them.
        Returns True if it is safe to proceed, False if the user cancels.
        """
        if not self._is_modified or self._project is None:
            return True

        SB = QMessageBox.StandardButton
        answer = QMessageBox.question(
            parent_widget,
            "Unsaved Changes",
            f"Project '{self._project.name}' has unsaved changes.\n"
            "Save, discard the changes, or cancel?",
            SB.Save | SB.Discard | SB.Cancel,
            SB.Cancel,
        )
        if answer == SB.Save:
            return self.save_project(parent_widget)   # False if failed or cancelled
        return answer == SB.Discard


def _peek_old_version(path: str) -> str | None:
    """Format version of an OLDER file (needs a backup before migration), else None."""
    try:
        with zipfile.ZipFile(path) as zf:
            meta = json.loads(zf.read("project_metadata.json").decode("utf-8"))
        v = meta.get("format_version")
    except (OSError, KeyError, ValueError, zipfile.BadZipFile, AttributeError):
        return None
    return v if isinstance(v, str) and is_older(v) else None
