"""U6.5: playback bar at the bottom of the 3D view: play / pause, frame step, scrub slider, loop.

An overlay widget like the view buttons. It only shows when a trajectory is loaded (Simulation dock,
a curve, a target motion). It talks to ``SimPanel`` (no playback logic here): ``play_trajectory``,
``pause_trajectory``, ``seek_trajectory``, ``step_trajectory``, the loop flag of the timeline.
"""
from __future__ import annotations

from typing import Any

from PySide6.QtCore import QEvent, Qt, Signal
from PySide6.QtWidgets import QCheckBox, QHBoxLayout, QLabel, QPushButton, QSlider, QWidget

FPS_STEP = 30.0                 # one frame step = 1/30 s
SLIDER_STEPS = 1000

BAR_STYLE = """
QWidget#playback_bar { background: rgba(32, 32, 40, 215); border: 1px solid #3a3a55; border-radius: 6px; }
QWidget#playback_bar QPushButton { color: #e8e8f0; background: transparent; border: none;
                                   padding: 3px 10px; min-height: 26px; min-width: 28px; font-size: 13pt; }
QWidget#playback_bar QPushButton:hover { background: #3d6fcc; color: #ffffff; }
QWidget#playback_bar QPushButton:focus { border: 2px solid #ffffff; }
QWidget#playback_bar QPushButton:disabled { color: #7d7d7d; }
QWidget#playback_bar QLabel, QWidget#playback_bar QCheckBox { color: #e8e8f0; }
"""


class PlaybackBar(QWidget):
    play_clicked = Signal()
    pause_clicked = Signal()
    step_requested = Signal(float)          # seconds, negative = back
    seek_requested = Signal(float)          # seconds
    loop_toggled = Signal(bool)

    def __init__(self, parent: QWidget):
        super().__init__(parent)
        self.setObjectName("playback_bar")
        self.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        self.setStyleSheet(BAR_STYLE)
        self.setAccessibleName("Playback bar")
        self._duration = 0.0
        self._quiet = False
        self._playing = False
        lay = QHBoxLayout(self)
        lay.setContentsMargins(8, 3, 8, 3)
        lay.setSpacing(2)
        self.btn_back = QPushButton("◀│")
        self.btn_back.setToolTip("One frame back (1/30 s)")
        self.btn_back.setAccessibleName("Step back one frame")
        self.btn_play = QPushButton("▶")
        self.btn_play.setToolTip("Play")
        self.btn_play.setAccessibleName("Play")
        self.btn_fwd = QPushButton("│▶")
        self.btn_fwd.setToolTip("One frame forward (1/30 s)")
        self.btn_fwd.setAccessibleName("Step forward one frame")
        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setRange(0, SLIDER_STEPS)
        self.slider.setMinimumWidth(180)
        self.slider.setAccessibleName("Playback position")
        self.slider.setToolTip("Drag to move through the motion")
        self.label = QLabel("0.00 / 0.00 s")
        self.label.setMinimumWidth(92)
        self.loop = QCheckBox("Loop")
        self.loop.setToolTip("Play again from the start when the motion ends")
        for w in (self.btn_back, self.btn_play, self.btn_fwd):
            lay.addWidget(w)
        lay.addWidget(self.slider, 1)
        lay.addWidget(self.label)
        lay.addWidget(self.loop)
        self.btn_play.clicked.connect(self._on_play)
        self.btn_back.clicked.connect(lambda: self.step_requested.emit(-1.0 / FPS_STEP))
        self.btn_fwd.clicked.connect(lambda: self.step_requested.emit(1.0 / FPS_STEP))
        self.slider.valueChanged.connect(self._on_slider)
        self.loop.toggled.connect(self.loop_toggled)
        parent.installEventFilter(self)
        self.hide()

    # ------------------------------------------------------------------ geometry
    def eventFilter(self, obj: Any, ev: Any) -> bool:
        if ev.type() == QEvent.Type.Resize:
            self._place()
        return False

    def _place(self) -> None:
        p = self.parentWidget()
        if p is None:
            return
        w = min(max(p.width() - 40, 280), 720)
        self.resize(w, self.sizeHint().height())
        self.move(max(0, (p.width() - w) // 2), max(0, p.height() - self.height() - 12))
        self.raise_()

    # ------------------------------------------------------------------ state
    @property
    def duration(self) -> float:
        return self._duration

    @property
    def playing(self) -> bool:
        return self._playing

    def set_duration(self, duration: float) -> None:
        """0 (no trajectory) hides the bar."""
        self._duration = max(float(duration), 0.0)
        has = self._duration > 0.0
        self.setVisible(has)
        if has:
            self._place()
            self.set_position(0.0)

    def set_position(self, t: float) -> None:
        T = self._duration
        self._quiet = True
        try:
            self.slider.setValue(int(round(SLIDER_STEPS * min(max(t, 0.0), T) / T)) if T > 0 else 0)
        finally:
            self._quiet = False
        self.label.setText(f"{min(max(t, 0.0), T):.2f} / {T:.2f} s")

    def set_playing(self, playing: bool) -> None:
        self._playing = bool(playing)
        self.btn_play.setText("Ⅱ" if playing else "▶")
        self.btn_play.setToolTip("Pause" if playing else "Play")
        self.btn_play.setAccessibleName("Pause" if playing else "Play")

    def set_loop(self, on: bool) -> None:
        if self.loop.isChecked() != bool(on):
            self.loop.blockSignals(True)
            self.loop.setChecked(bool(on))
            self.loop.blockSignals(False)

    # ------------------------------------------------------------------ user input
    def _on_play(self) -> None:
        (self.pause_clicked if self._playing else self.play_clicked).emit()

    def _on_slider(self, v: int) -> None:
        if self._quiet or self._duration <= 0:
            return
        t = self._duration * v / SLIDER_STEPS
        self.label.setText(f"{t:.2f} / {self._duration:.2f} s")
        self.seek_requested.emit(t)
