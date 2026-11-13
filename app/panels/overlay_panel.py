"""U6.1: "Overlays" dock (Analyze mode). One card per overlay: toggle, opacity, colour bar, status.

The cards are built from the registered overlays of an ``OverlayManager``; an overlay may add its own
controls through ``make_controls``. All numbers and drawing live in ``app.analysis.overlays``.
"""
from __future__ import annotations

from typing import Any

from PySide6.QtCore import Qt
from PySide6.QtGui import QColor, QLinearGradient, QPainter
from PySide6.QtWidgets import (
    QCheckBox, QFrame, QHBoxLayout, QLabel, QProgressBar, QPushButton, QSlider, QVBoxLayout, QWidget,
)

from app.theme import MUTED


class ColorBar(QWidget):
    """Horizontal gradient with two end labels (legend of an overlay)."""

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self._stops: list = []
        self._left = QLabel("")
        self._right = QLabel("")
        for lab in (self._left, self._right):
            lab.setStyleSheet(f"color: {MUTED}; font-size: 8.5pt;")
        self._bar = _Gradient()
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(1)
        lay.addWidget(self._bar)
        row = QHBoxLayout()
        row.setContentsMargins(0, 0, 0, 0)
        row.addWidget(self._left)
        row.addStretch(1)
        row.addWidget(self._right)
        lay.addLayout(row)

    def set_legend(self, stops: list, left: str, right: str) -> None:
        self._stops = list(stops)
        self._bar.set_stops(stops)
        self._left.setText(left)
        self._right.setText(right)
        self.setVisible(bool(stops))
        self.setAccessibleDescription(f"Colour bar from {left} to {right}")

    @property
    def stops(self) -> list:
        return list(self._stops)


class _Gradient(QWidget):
    def __init__(self):
        super().__init__()
        self._stops: list = []
        self.setFixedHeight(10)

    def set_stops(self, stops: list) -> None:
        self._stops = list(stops)
        self.update()

    def paintEvent(self, _ev) -> None:
        p = QPainter(self)
        g = QLinearGradient(0, 0, self.width(), 0)
        for pos, rgb in self._stops:
            g.setColorAt(float(pos), QColor(*[int(v) for v in rgb]))
        p.fillRect(self.rect(), g)
        p.setPen(QColor("#555555"))
        p.drawRect(self.rect().adjusted(0, 0, -1, -1))
        p.end()


class OverlayCard(QFrame):
    def __init__(self, manager: Any, key: str, parent: QWidget | None = None):
        super().__init__(parent)
        ov = manager.overlay(key)
        self._m, self._key = manager, key
        self.setObjectName(f"overlay_card_{key}")
        self.setFrameShape(QFrame.Shape.StyledPanel)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(8, 6, 8, 8)
        lay.setSpacing(4)
        self.check = QCheckBox(ov.title)
        f = self.check.font()
        f.setBold(True)
        self.check.setFont(f)
        self.check.setToolTip(ov.blurb)
        self.check.setAccessibleName(f"{ov.title} overlay")
        lay.addWidget(self.check)
        blurb = QLabel(ov.blurb)
        blurb.setWordWrap(True)
        blurb.setStyleSheet(f"color: {MUTED};")
        lay.addWidget(blurb)

        self.extra = ov.make_controls(manager)
        if self.extra is not None:
            lay.addWidget(self.extra)

        row = QHBoxLayout()
        lab = QLabel("Opacity")
        row.addWidget(lab)
        self.opacity = QSlider(Qt.Orientation.Horizontal)
        self.opacity.setRange(5, 100)
        self.opacity.setValue(int(round(manager.opacity(key) * 100)))
        self.opacity.setAccessibleName(f"{ov.title} opacity")
        self.opacity.setToolTip("Transparency of the overlay in the 3D view")
        self.opacity_label = QLabel(f"{self.opacity.value()} %")
        self.opacity_label.setMinimumWidth(38)
        row.addWidget(self.opacity, 1)
        row.addWidget(self.opacity_label)
        lay.addLayout(row)

        self.bar = ColorBar()
        lay.addWidget(self.bar)

        self.status = QLabel("Off")
        self.status.setWordWrap(True)
        self.status.setStyleSheet(f"color: {MUTED};")
        lay.addWidget(self.status)
        self.progress = QProgressBar()
        self.progress.setRange(0, 100)
        self.progress.setTextVisible(False)
        self.progress.setFixedHeight(8)
        self.progress.hide()
        lay.addWidget(self.progress)
        brow = QHBoxLayout()
        self.recompute = QPushButton("Recompute")
        self.recompute.setToolTip("Throw away the stored result and compute it again")
        self.cancel = QPushButton("Cancel")
        self.cancel.setVisible(False)
        brow.addWidget(self.recompute)
        brow.addWidget(self.cancel)
        brow.addStretch(1)
        if ov.live:
            self.recompute.hide()
        lay.addLayout(brow)

        self.check.toggled.connect(lambda on: manager.set_enabled(key, on))
        self.opacity.valueChanged.connect(self._on_opacity)
        self.recompute.clicked.connect(lambda: manager.recompute(key))
        self.cancel.clicked.connect(lambda: manager.cancel(key))
        manager.enabled_changed.connect(self._on_enabled)
        manager.status_changed.connect(self._on_status)
        manager.progress_changed.connect(self._on_progress)
        manager.computing_changed.connect(self._on_computing)
        manager.changed.connect(self._on_changed)
        self._refresh_legend()

    def _on_opacity(self, v: int) -> None:
        self.opacity_label.setText(f"{v} %")
        self._m.set_opacity(self._key, v / 100.0)

    def _on_enabled(self, key: str, on: bool) -> None:
        if key == self._key:
            if self.check.isChecked() != on:
                self.check.blockSignals(True)
                self.check.setChecked(on)
                self.check.blockSignals(False)
            if not on:
                self.status.setText("Off")

    def _on_status(self, key: str, text: str) -> None:
        if key == self._key and self._m.enabled(key):
            self.status.setText(text)

    def _on_progress(self, key: str, pct: int) -> None:
        if key == self._key:
            self.progress.setVisible(pct >= 0)
            self.progress.setValue(max(pct, 0))

    def _on_computing(self, key: str, on: bool) -> None:
        if key == self._key:
            self.cancel.setVisible(on)

    def _on_changed(self, key: str) -> None:
        if key == self._key:
            self._refresh_legend()

    def _refresh_legend(self) -> None:
        ov = self._m.overlay(self._key)
        opts = self._m.options(self._key)
        left, right = ov.legend_labels(opts)
        self.bar.set_legend(ov.legend(opts), left, right)


class OverlayPanel(QWidget):
    """Dock content: the cards of all overlays."""

    def __init__(self, manager: Any, parent: QWidget | None = None):
        super().__init__(parent)
        self._m = manager
        lay = QVBoxLayout(self)
        lay.setContentsMargins(6, 6, 6, 6)
        lay.setSpacing(8)
        intro = QLabel("Switch an overlay on to see it in the 3D view. Results are kept until the "
                       "robot or its tool changes.")
        intro.setWordWrap(True)
        intro.setStyleSheet(f"color: {MUTED};")
        lay.addWidget(intro)
        self.cards: dict[str, OverlayCard] = {}
        for key in manager.keys:
            card = OverlayCard(manager, key)
            self.cards[key] = card
            lay.addWidget(card)
        lay.addStretch(1)
