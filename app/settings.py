"""
app/settings.py
Robotics Analyzer — application settings manager.
R1.2: Typed QSettings wrapper for persistent application preferences.
"""

from PySide6.QtCore import QSettings, QByteArray
from PySide6.QtWidgets import QMainWindow
from typing import Any, Optional


class AppSettings:
    """
    Typed wrapper around QSettings.
    Provides named, typed getters and setters for all application preferences.
    All settings persist across application restarts.
    """

    _ORGANIZATION = "RoboticsAnalyzer"
    _APPLICATION = "RoboticsAnalyzer"

    # Setting keys
    KEY_WINDOW_GEOMETRY = "window/geometry"
    KEY_WINDOW_STATE = "window/state"
    KEY_RECENT_PROJECTS = "project/recent_files"
    KEY_RECENT_MAX = "project/recent_max"
    KEY_LAST_PROJECT_DIR = "project/last_dir"
    KEY_LOG_LEVEL = "logging/level"
    KEY_VIEWPORT_BG_COLOR = "viewport/bg_color"
    KEY_SHOW_COORDINATE_FRAMES = "viewport/show_frames"
    KEY_SHOW_GRID = "viewport/show_grid"

    def __init__(self):
        self._settings = QSettings(self._ORGANIZATION, self._APPLICATION)

    # ------------------------------------------------------------------
    # Window geometry
    # ------------------------------------------------------------------

    def save_window(self, window: QMainWindow) -> None:
        """Save window geometry and dock layout to settings."""
        self._settings.setValue(self.KEY_WINDOW_GEOMETRY, window.saveGeometry())
        self._settings.setValue(self.KEY_WINDOW_STATE, window.saveState())

    def restore_window(self, window: QMainWindow) -> bool:
        """
        Restore window geometry and dock layout from settings.
        Returns True if geometry was restored, False if no saved state exists.
        """
        geometry = self._settings.value(self.KEY_WINDOW_GEOMETRY)
        state = self._settings.value(self.KEY_WINDOW_STATE)

        if geometry is not None:
            window.restoreGeometry(geometry)
        if state is not None:
            window.restoreState(state)

        return geometry is not None

    # ------------------------------------------------------------------
    # Recent projects
    # ------------------------------------------------------------------

    def recent_projects(self) -> list[str]:
        """Return the list of recently opened project file paths."""
        raw = self._settings.value(self.KEY_RECENT_PROJECTS, [])
        if isinstance(raw, str):
            return [raw] if raw else []
        return list(raw) if raw else []

    def add_recent_project(self, path: str) -> None:
        """Add a project path to the recent projects list (most-recent-first)."""
        recent = self.recent_projects()
        if path in recent:
            recent.remove(path)
        recent.insert(0, path)
        max_count = self.recent_max()
        self._settings.setValue(self.KEY_RECENT_PROJECTS, recent[:max_count])

    def recent_max(self) -> int:
        """Return the maximum number of recent project entries to keep."""
        return int(self._settings.value(self.KEY_RECENT_MAX, 10))

    def last_project_dir(self) -> str:
        """Return the last directory used for opening/saving projects."""
        import os
        default = os.path.expanduser("~")
        return str(self._settings.value(self.KEY_LAST_PROJECT_DIR, default))

    def set_last_project_dir(self, path: str) -> None:
        """Save the last directory used for opening/saving projects."""
        self._settings.setValue(self.KEY_LAST_PROJECT_DIR, path)

    # ------------------------------------------------------------------
    # Logging
    # ------------------------------------------------------------------

    def log_level(self) -> str:
        """Return the application log level (DEBUG, INFO, WARNING, ERROR)."""
        return str(self._settings.value(self.KEY_LOG_LEVEL, "INFO"))

    def set_log_level(self, level: str) -> None:
        """Set the application log level."""
        assert level in ("DEBUG", "INFO", "WARNING", "ERROR")
        self._settings.setValue(self.KEY_LOG_LEVEL, level)

    # ------------------------------------------------------------------
    # Viewport
    # ------------------------------------------------------------------

    def viewport_bg_color(self) -> str:
        """Return the viewport background color as a hex string."""
        return str(self._settings.value(self.KEY_VIEWPORT_BG_COLOR, "#1a1a2e"))

    def set_viewport_bg_color(self, color: str) -> None:
        """Set the viewport background color."""
        self._settings.setValue(self.KEY_VIEWPORT_BG_COLOR, color)

    def show_coordinate_frames(self) -> bool:
        """Return True if coordinate frames are visible in the viewport."""
        val = self._settings.value(self.KEY_SHOW_COORDINATE_FRAMES, True)
        if isinstance(val, str):
            return val.lower() == "true"
        return bool(val)

    def set_show_coordinate_frames(self, visible: bool) -> None:
        """Set coordinate frame visibility in the viewport."""
        self._settings.setValue(self.KEY_SHOW_COORDINATE_FRAMES, visible)

    def show_grid(self) -> bool:
        """Return True if the grid is visible in the viewport."""
        val = self._settings.value(self.KEY_SHOW_GRID, True)
        if isinstance(val, str):
            return val.lower() == "true"
        return bool(val)

    def set_show_grid(self, visible: bool) -> None:
        """Set grid visibility in the viewport."""
        self._settings.setValue(self.KEY_SHOW_GRID, visible)

    # ------------------------------------------------------------------
    # Generic access
    # ------------------------------------------------------------------

    def mesh_search_roots(self) -> list[str]:
        """Folders searched for package:// meshes (R3.5)."""
        v = self._settings.value("import/mesh_search_roots", [])
        if v is None or v == "":
            return []
        if isinstance(v, str):
            return [v]
        return [str(x) for x in v]

    def set_mesh_search_roots(self, roots: list[str]) -> None:
        self._settings.setValue("import/mesh_search_roots", list(roots))

    def get(self, key: str, default: Any = None) -> Any:
        """Get a setting value by key with an optional default."""
        return self._settings.value(key, default)

    def set(self, key: str, value: Any) -> None:
        """Set a setting value by key."""
        self._settings.setValue(key, value)

    def sync(self) -> None:
        """Flush all pending settings writes to disk."""
        self._settings.sync()
