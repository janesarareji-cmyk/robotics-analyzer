"""U5.4: work modes. One bar at the top replaces the 18-button side bar.

Each mode is a saved dock layout: it shows only its docks (names are the suffix of the
``_action_view_<name>`` actions of MainWindow). Every other dock stays reachable from the View menu.
"""
from __future__ import annotations

import hashlib
from typing import Any

from PySide6.QtCore import Qt
from PySide6.QtGui import QAction, QActionGroup, QKeySequence
from PySide6.QtWidgets import QToolBar, QToolButton

DEFAULT_MODE = "Move"

# name -> (tooltip, view action names; the first one is raised)
MODES: dict[str, tuple[str, tuple[str, ...]]] = {
    "Model": ("Robot structure: tree, properties, DH table, mass and inertia", ("model", "props", "dh", "physical")),
    "Move": ("Move the robot: joint sliders, live matrices, collision", ("joints", "matrices", "collision")),
    "Target": ("Set a 3D target or a curve and let the robot follow it", ("target", "curve")),
    "Plan": ("Plan a path around objects", ("planning", "collision", "ghosts")),
    "Simulate": ("Run a simulation and look at graphs", ("simulation", "graphs")),
    "Analyze": ("Overlays, workspace, IK, dynamics, compare robots", ("overlays", "workspace", "ik", "analysis", "compare")),
}
MODE_NAMES = list(MODES)
LEFT_DOCKS = ("model", "props", "physical")

BAR_STYLE = """
QToolBar#mode_bar { background: #202020; border: none; border-bottom: 1px solid #333333;
                    spacing: 4px; padding: 3px 8px; }
QToolBar#mode_bar QToolButton { color: #d0d0d0; font-size: 10pt; font-weight: 600;
                    padding: 5px 16px; min-height: 22px; border: none;
                    border-bottom: 3px solid transparent; background: transparent; }
QToolBar#mode_bar QToolButton:hover { background: #2e2e2e; }
QToolBar#mode_bar QToolButton:checked { color: #ffffff; border-bottom: 3px solid #4f8cff; background: #2a2f3a; }
QToolBar#mode_bar QToolButton:focus { outline: none; border: 2px solid #ffffff; }
"""


def mode_for_dock(name: str) -> str:
    """The first mode that shows the dock of view action ``name``."""
    for mode, (_tip, docks) in MODES.items():
        if name in docks:
            return mode
    return DEFAULT_MODE


class ModeController:
    """Builds the mode bar and applies a mode to the MainWindow docks."""

    def __init__(self, win: Any) -> None:
        self.win = win
        self.current: str | None = None
        self._buttons: dict[str, QToolButton] = {}
        self._actions: dict[str, QAction] = {}
        self._applying = False
        bar = QToolBar("Modes", win)
        bar.setObjectName("mode_bar")
        bar.setMovable(False)
        bar.setFloatable(False)
        bar.setToolButtonStyle(Qt.ToolButtonStyle.ToolButtonTextOnly)
        bar.setStyleSheet(BAR_STYLE)
        bar.setAccessibleName("Work modes")
        self.bar = bar
        group = QActionGroup(win)
        group.setExclusive(True)
        for i, (name, (tip, _docks)) in enumerate(MODES.items(), start=1):
            act = QAction(name, win)
            act.setCheckable(True)
            act.setShortcut(QKeySequence(f"Ctrl+{i}"))
            act.setToolTip(f"{tip} (Ctrl+{i})")
            act.setStatusTip(tip)
            act.triggered.connect(lambda _c=False, n=name: self.apply(n))
            group.addAction(act)
            bar.addAction(act)
            self._actions[name] = act
            btn = bar.widgetForAction(act)
            btn.setAccessibleName(f"{name} mode")
            btn.setAccessibleDescription(tip)
            self._buttons[name] = btn
        self.group = group

    # ------------------------------------------------------------------ queries
    def button(self, mode: str) -> QToolButton:
        return self._buttons[mode]

    def action(self, mode: str) -> QAction:
        return self._actions[mode]

    @staticmethod
    def docks_of(mode: str) -> tuple[str, ...]:
        return MODES[mode][1]

    # ------------------------------------------------------------------ apply
    def _dock(self, name: str):
        action = getattr(self.win, "_action_view_" + name)
        for a, d in self.win._view_action_docks():
            if a is action:
                return d
        return None

    def apply(self, mode: str, remember: bool = True) -> None:
        """Show only the docks of ``mode`` (managed docks only; Log, Script, Plugins stay as they are)."""
        if mode not in MODES or self._applying:
            return
        self._applying = True
        try:
            win = self.win
            if win.focus_mode:
                win.set_focus_mode(False)
            wanted = MODES[mode][1]
            managed = {n for _m, (_t, ds) in MODES.items() for n in ds}
            for name in managed - set(wanted):
                d = self._dock(name)
                if d is not None:
                    d.hide()
            for name in wanted:
                d = self._dock(name)
                if d is not None:
                    d.show()
            first = self._dock(wanted[0])
            if first is not None:
                first.raise_()
            self.current = mode
            self._actions[mode].setChecked(True)
            self._on_applied(mode)
        finally:
            self._applying = False
        if remember:
            self.remember()

    def set_current(self, mode: str) -> None:
        """Mark a mode as current without touching the docks (after a layout reset)."""
        self.current = mode
        self._actions[mode].setChecked(True)

    def _on_applied(self, mode: str) -> None:
        win = self.win
        if mode == "Target" and win.project_controller.robot is not None:
            tp = win.target_panel
            if tp.current_index < 0:
                tp.add_from_tcp()
        elif mode == "Plan" and win.project_controller.robot is not None:
            win.planning_panel.capture_current("start")      # the plan starts where the robot is

    # ------------------------------------------------------------------ per project memory
    def _key(self) -> str | None:
        project = self.win.project_controller.current_project
        path = getattr(project, "file_path", None)
        if not path:
            return None
        return "ui/mode/" + hashlib.sha1(str(path).encode("utf-8", "replace")).hexdigest()[:16]

    def remember(self) -> None:
        try:
            self.win.settings._settings.setValue("ui/last_mode", self.current)
        except Exception:
            pass
        key = self._key()
        if key and self.current:
            try:
                self.win.settings._settings.setValue(key, self.current)
            except Exception:
                pass

    def last_mode(self) -> str:
        """Mode of the last session (the restored dock layout belongs to it)."""
        try:
            val = str(self.win.settings._settings.value("ui/last_mode", "") or "")
        except Exception:
            val = ""
        return val if val in MODES else DEFAULT_MODE

    def recall(self) -> str | None:
        key = self._key()
        if key:
            try:
                val = str(self.win.settings._settings.value(key, "") or "")
            except Exception:
                val = ""
            if val in MODES:
                return val
        return None
