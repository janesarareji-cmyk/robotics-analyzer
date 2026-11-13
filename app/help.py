"""U5.9: F1 help overlay (shortcuts + what each panel does) and the first-run tour."""
from __future__ import annotations

from typing import Any

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (QDialog, QDialogButtonBox, QFrame, QHBoxLayout, QLabel, QPushButton,
                               QTextBrowser, QVBoxLayout, QWidget)

from app.modes import MODES

SHORTCUTS = [
    ("Ctrl+1 ... Ctrl+6", "Switch work mode: Model, Move, Target, Plan, Simulate, Analyze"),
    ("F1", "This help"),
    ("F11", "Focus 3D view (hide every panel)"),
    ("Ctrl+Z / Ctrl+Y", "Undo / redo joint moves, target and object edits"),
    ("T", "Drag the tool (TCP) with the mouse"),
    ("Ctrl+O / Ctrl+S", "Open / save the project"),
    ("Mouse in the 3D view", "Left drag: turn. Middle drag or Shift+left: move. Wheel: zoom. Click a link: select it"),
    ("Hold a jog button", "Repeat the step"),
]

PANEL_HELP = {
    "model": "Robot tree: links and joints. Click one to see its properties.",
    "props": "Properties of the selected link or joint.",
    "dh": "DH table: edit the joint parameters of the robot.",
    "physical": "Mass, centre of mass and inertia of each link.",
    "joints": "Joint sliders (degrees or mm), poses (Home, Ready, saved), Cartesian jog and the tool pose.",
    "matrices": "Live transforms, Jacobian and manipulability, with copy buttons.",
    "collision": "Objects in the scene and which link pairs may touch.",
    "target": "A 3D target for the tool: ranked IK solutions, preview and Play to target.",
    "curve": "A line, arc or spline for the tool to follow, with a check of reach and collisions.",
    "planning": "Plan a collision-free path between two poses.",
    "ghosts": "Saved poses shown as ghost robots; compare them.",
    "simulation": "Run the robot with a simulation or a trajectory; teach waypoints.",
    "graphs": "Plots of joint and tool signals from the run.",
    "workspace": "Where the tool can reach: point cloud, volume, slice view.",
    "ik": "Inverse kinematics for a typed pose with all solutions.",
    "analysis": "Jacobian, singularity, torques and energy at the current pose.",
    "compare": "Compare reach, volume and payload of several robots.",
    "overlays": "Show reach, singularity zones and joint load as colours in the 3D view.",
    "performance": "Benchmarks of the compute backend.",
}


def help_html(mode: str | None = None) -> str:
    rows = "".join(f"<tr><td><b>{k}</b></td><td>{v}</td></tr>" for k, v in SHORTCUTS)
    out = [f"<h3>Shortcuts</h3><table cellpadding='4'>{rows}</table>"]
    if mode in MODES:
        out.append(f"<h3>The {mode} mode</h3><p>{MODES[mode][0]}</p><ul>")
        for name in MODES[mode][1]:
            out.append(f"<li><b>{name.capitalize()}</b>: {PANEL_HELP.get(name, '')}</li>")
        out.append("</ul>")
    out.append("<h3>Modes</h3><ul>" + "".join(
        f"<li><b>{m}</b>: {t}</li>" for m, (t, _d) in MODES.items()) + "</ul>")
    return "".join(out)


class HelpDialog(QDialog):
    def __init__(self, parent: QWidget | None = None, mode: str | None = None):
        super().__init__(parent)
        self.setWindowTitle("Help: shortcuts and panels")
        self.setMinimumSize(560, 460)
        lay = QVBoxLayout(self)
        self.browser = QTextBrowser()
        self.browser.setOpenExternalLinks(False)
        lay.addWidget(self.browser)
        box = QDialogButtonBox(QDialogButtonBox.StandardButton.Close)
        box.rejected.connect(self.reject)
        box.button(QDialogButtonBox.StandardButton.Close).setText("Close")
        lay.addWidget(box)
        self.set_mode(mode)

    def set_mode(self, mode: str | None) -> None:
        self.browser.setHtml(help_html(mode))

    @property
    def text(self) -> str:
        return self.browser.toPlainText()


TOUR_STEPS = [
    ("Pick a robot", "Click a sample robot on the start screen, or use Import URDF... / Open project..."),
    ("Work modes", "The bar at the top has six modes (Ctrl+1 to Ctrl+6). Each mode shows only the panels you need."),
    ("Move the robot", "In Move mode use the joint sliders, or the Cartesian jog buttons to move the tool in X, Y, Z."),
    ("Need help?", "Press F1 for shortcuts and what each panel does. Hover any button for a tip."),
]


class TourCard(QFrame):
    """Small card over the 3D view: four steps, Next / Back / Skip."""

    finished = Signal(bool)         # True = reached the end, False = skipped

    def __init__(self, parent: QWidget):
        super().__init__(parent)
        self.setObjectName("tour_card")
        self.setStyleSheet(
            "QFrame#tour_card { background: #24243a; border: 2px solid #4f8cff; border-radius: 8px; }"
            "QLabel { color: #f0f0f8; } QLabel#tour_title { font-size: 12pt; font-weight: 600; }"
            "QPushButton { color: #ffffff; background: #3d6fcc; border: none; border-radius: 4px; padding: 5px 14px; }"
            "QPushButton:focus { border: 2px solid #ffffff; }"
            "QPushButton#tour_skip { background: transparent; color: #c8c8d8; }")
        self.setFixedWidth(380)
        lay = QVBoxLayout(self)
        self._step_label = QLabel()
        self._title = QLabel()
        self._title.setObjectName("tour_title")
        self._body = QLabel()
        self._body.setWordWrap(True)
        row = QHBoxLayout()
        self.btn_skip = QPushButton("Skip tour")
        self.btn_skip.setObjectName("tour_skip")
        self.btn_back = QPushButton("Back")
        self.btn_next = QPushButton("Next")
        row.addWidget(self.btn_skip)
        row.addStretch(1)
        row.addWidget(self.btn_back)
        row.addWidget(self.btn_next)
        for w in (self._step_label, self._title, self._body):
            lay.addWidget(w)
        lay.addLayout(row)
        self.btn_skip.clicked.connect(lambda _c=False: self._end(False))
        self.btn_back.clicked.connect(lambda _c=False: self.go(self.index - 1))
        self.btn_next.clicked.connect(lambda _c=False: self._next())
        self.index = 0
        self.hide()

    def start(self) -> None:
        self.go(0)
        self.show()
        self.raise_()
        self.btn_next.setFocus()

    def go(self, i: int) -> None:
        self.index = min(max(i, 0), len(TOUR_STEPS) - 1)
        title, body = TOUR_STEPS[self.index]
        self._step_label.setText(f"Step {self.index + 1} of {len(TOUR_STEPS)}")
        self._title.setText(title)
        self._body.setText(body)
        self.btn_back.setEnabled(self.index > 0)
        self.btn_next.setText("Done" if self.index == len(TOUR_STEPS) - 1 else "Next")
        self.adjustSize()
        self._place()

    def _place(self) -> None:
        p = self.parentWidget()
        if p is not None:
            self.move(max(0, (p.width() - self.width()) // 2), max(0, p.height() - self.height() - 24))

    def _next(self) -> None:
        if self.index >= len(TOUR_STEPS) - 1:
            self._end(True)
        else:
            self.go(self.index + 1)

    def _end(self, completed: bool) -> None:
        self.hide()
        self.finished.emit(completed)
