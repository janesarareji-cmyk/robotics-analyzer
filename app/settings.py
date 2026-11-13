"""
app/settings.py
Robotics Analyzer — application settings manager.
R1.2: Typed QSettings wrapper for persistent application preferences.
"""

from PySide6.QtCore import QSettings, QByteArray
from PySide6.QtWidgets import QMainWindow
from typing import Any, Optional

# U1: bump when the default dock layout changes. A saved window state with another
# (or no) version is ignored once, so users get the new default layout.
LAYOUT_VERSION = 2


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
    KEY_LAYOUT_VERSION = "window/layout_version"
    KEY_RECENT_PROJECTS = "project/recent_files"
    KEY_RECENT_MAX = "project/recent_max"
    KEY_LAST_PROJECT_DIR = "project/last_dir"
    KEY_LOG_LEVEL = "logging/level"
    KEY_VIEWPORT_BG_COLOR = "viewport/bg_color"
    KEY_SHOW_COORDINATE_FRAMES = "viewport/show_frames"
    KEY_SHOW_GRID = "viewport/show_grid"
    KEY_SHOW_FLOOR = "viewport/show_floor"             # U2.3
    KEY_COLLISION_ENABLED = "collision/enabled"        # U3.4
    KEY_IK_RANK_WEIGHTS = "target/rank_weights"        # U4a.2
    KEY_COMPUTE_BACKEND = "compute/backend"            # "auto" | "cpu" | "cuda"  (R5.5)
    KEY_VRAM_BUDGET_MB = "compute/vram_budget_mb"      # 256..7168, default 6144 (R5.5)

    def __init__(self):
        self._settings = QSettings(self._ORGANIZATION, self._APPLICATION)

    # ------------------------------------------------------------------
    # Window geometry
    # ------------------------------------------------------------------

    def layout_version(self) -> int:
        """Layout version stored with the window state (0 = none stored)."""
        try:
            return int(self._settings.value(self.KEY_LAYOUT_VERSION, 0))
        except (TypeError, ValueError):
            return 0

    def set_layout_version(self, version: int) -> None:
        self._settings.setValue(self.KEY_LAYOUT_VERSION, int(version))

    def save_window(self, window: QMainWindow) -> None:
        """Save window geometry, dock layout and layout version to settings."""
        self._settings.setValue(self.KEY_WINDOW_GEOMETRY, window.saveGeometry())
        self._settings.setValue(self.KEY_WINDOW_STATE, window.saveState())
        self.set_layout_version(LAYOUT_VERSION)

    def restore_window(self, window: QMainWindow) -> bool:
        """
        Restore window geometry and dock layout from settings.
        A dock state saved with another layout version is ignored (geometry is kept)
        and dropped, and the current version is stored (U1).
        Returns True if geometry was restored, False if no saved state exists.
        """
        geometry = self._settings.value(self.KEY_WINDOW_GEOMETRY)
        state = self._settings.value(self.KEY_WINDOW_STATE)

        if geometry is not None:
            window.restoreGeometry(geometry)
        if self.layout_version() != LAYOUT_VERSION:
            self._settings.remove(self.KEY_WINDOW_STATE)
            self.set_layout_version(LAYOUT_VERSION)
        elif state is not None:
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
        try:
            n = int(self._settings.value(self.KEY_RECENT_MAX, 10))
        except (TypeError, ValueError):
            return 10
        return n if n > 0 else 10

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

    def show_floor(self) -> bool:
        """Return True if the floor grid and world axes are shown (default True)."""
        val = self._settings.value(self.KEY_SHOW_FLOOR, True)
        if isinstance(val, str):
            return val.lower() == "true"
        return bool(val)

    def set_show_floor(self, visible: bool) -> None:
        """Set floor grid visibility."""
        self._settings.setValue(self.KEY_SHOW_FLOOR, bool(visible))

    def ik_rank_weights(self) -> tuple:
        """Weights of the IK ranking (motion, limits, manipulability, clearance); bad value = default."""
        default = (0.4, 0.2, 0.2, 0.2)
        val = self._settings.value(self.KEY_IK_RANK_WEIGHTS, None)
        try:
            parts = [float(v) for v in (val.split(",") if isinstance(val, str) else val)]
            if len(parts) == 4 and all(p >= 0 for p in parts) and sum(parts) > 0:
                return tuple(parts)
        except (TypeError, ValueError):
            pass
        return default

    def set_ik_rank_weights(self, weights) -> None:
        self._settings.setValue(self.KEY_IK_RANK_WEIGHTS, ",".join(f"{float(w):.6g}" for w in weights))

    def collision_enabled(self) -> bool:
        """Return True if live collision blocking is on (default True; off = pass-through)."""
        val = self._settings.value(self.KEY_COLLISION_ENABLED, True)
        if isinstance(val, str):
            return val.lower() == "true"
        return bool(val)

    def set_collision_enabled(self, on: bool) -> None:
        self._settings.setValue(self.KEY_COLLISION_ENABLED, bool(on))

    # ------------------------------------------------------------------
    # Compute (R5.5)
    # ------------------------------------------------------------------

    def compute_backend(self) -> str:
        """Return the preferred compute backend: auto, cpu or cuda."""
        v = str(self._settings.value(self.KEY_COMPUTE_BACKEND, "auto"))
        return v if v in ("auto", "cpu", "cuda") else "auto"

    def set_compute_backend(self, value: str) -> None:
        assert value in ("auto", "cpu", "cuda")
        self._settings.setValue(self.KEY_COMPUTE_BACKEND, value)

    def vram_budget_mb(self) -> int:
        """Return the VRAM budget in MiB (clamped to 256..7168, default 6144)."""
        try:
            v = int(float(self._settings.value(self.KEY_VRAM_BUDGET_MB, 6144)))
        except (TypeError, ValueError):
            v = 6144
        return max(256, min(7168, v))

    def set_vram_budget_mb(self, mb: int) -> None:
        self._settings.setValue(self.KEY_VRAM_BUDGET_MB, max(256, min(7168, int(mb))))

    # ------------------------------------------------------------------
    # Autosave (R8.4)
    # ------------------------------------------------------------------

    AUTOSAVE_DEFAULT_S = 120
    AUTOSAVE_MIN_S = 15

    def autosave_interval_s(self) -> int:
        """Autosave interval in seconds (default 120, minimum 15)."""
        try:
            v = int(float(self._settings.value("project/autosave_s", self.AUTOSAVE_DEFAULT_S)))
        except (TypeError, ValueError):
            v = self.AUTOSAVE_DEFAULT_S
        return max(self.AUTOSAVE_MIN_S, v)

    def set_autosave_interval_s(self, seconds: int) -> None:
        self._settings.setValue("project/autosave_s",
                                max(self.AUTOSAVE_MIN_S, int(seconds)))

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

    def plugins_folder(self) -> str:
        """Folder scanned for plugins (R8.2); default %APPDATA%/RoboticsAnalyzer/plugins."""
        import os
        default = os.path.join(os.environ.get("APPDATA") or os.path.expanduser("~"),
                               self._APPLICATION, "plugins")
        v = str(self._settings.value("plugins/folder", default) or "").strip()
        return v or default

    def set_plugins_folder(self, path: str) -> None:
        self._settings.setValue("plugins/folder", path)

    def get(self, key: str, default: Any = None) -> Any:
        """Get a setting value by key with an optional default."""
        return self._settings.value(key, default)

    def set(self, key: str, value: Any) -> None:
        """Set a setting value by key."""
        self._settings.setValue(key, value)

    def sync(self) -> None:
        """Flush all pending settings writes to disk."""
        self._settings.sync()
