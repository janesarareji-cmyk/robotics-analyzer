"""U6.4: 2D bars drawn over the 3D view next to every joint (torque and speed load).

``LoadBars`` is a transparent widget on top of the viewport (mouse events pass through). Each item is
``(x, y, label, torque_ratio, speed_ratio)`` in widget pixels. A bar is green up to 70 % of the limit,
orange up to 100 %, red above; the bar is full at 100 % and a red end mark shows an overload.
"""
from __future__ import annotations

from typing import Any, Sequence

from PySide6.QtCore import QEvent, QRectF, Qt
from PySide6.QtGui import QColor, QFont, QPainter, QPen
from PySide6.QtWidgets import QWidget

GREEN = "#66bb6a"
ORANGE = "#ff9800"
RED = "#ff3030"
WARN_AT = 0.7
BAR_W, BAR_H = 44, 5


def load_color(ratio: float) -> str:
    """Bar colour for a load ratio (value / limit)."""
    if ratio > 1.0:
        return RED
    if ratio > WARN_AT:
        return ORANGE
    return GREEN


class LoadBars(QWidget):
    def __init__(self, parent: QWidget):
        super().__init__(parent)
        self.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents, True)
        self.setAttribute(Qt.WidgetAttribute.WA_TranslucentBackground, True)
        self.setAttribute(Qt.WidgetAttribute.WA_NoSystemBackground, True)
        self.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.items: list[tuple] = []
        self.setAccessibleName("Joint load bars")
        parent.installEventFilter(self)
        self._fit()

    def eventFilter(self, obj: Any, ev: Any) -> bool:
        if ev.type() == QEvent.Type.Resize:
            self._fit()
        return False

    def _fit(self) -> None:
        p = self.parentWidget()
        if p is not None:
            self.setGeometry(0, 0, p.width(), p.height())
            self.raise_()

    def set_items(self, items: Sequence[tuple]) -> None:
        self.items = list(items)
        self.update()

    def paintEvent(self, _ev) -> None:
        if not self.items:
            return
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        f = QFont()
        f.setPointSizeF(8.5)
        f.setBold(True)
        p.setFont(f)
        for x, y, label, tr, sr in self.items:
            if x != x or y != y:                      # NaN: not drawable
                continue
            bx, by = x + 10.0, y - 11.0
            # backing plate so the bars read on any colour
            p.setPen(Qt.PenStyle.NoPen)
            p.setBrush(QColor(20, 20, 24, 200))
            p.drawRoundedRect(QRectF(bx - 3, by - 13, BAR_W + 6 + 22, 2 * BAR_H + 22), 3, 3)
            p.setPen(QPen(QColor("#e8e8e8")))
            p.drawText(QRectF(bx, by - 13, 60, 12), Qt.AlignmentFlag.AlignLeft, str(label))
            for row, (ratio, tag) in enumerate(((tr, "T"), (sr, "S"))):
                yy = by + row * (BAR_H + 3)
                p.setPen(Qt.PenStyle.NoPen)
                p.setBrush(QColor("#44444c"))
                p.drawRect(QRectF(bx, yy, BAR_W, BAR_H))
                p.setBrush(QColor(load_color(ratio)))
                p.drawRect(QRectF(bx, yy, BAR_W * min(max(ratio, 0.0), 1.0), BAR_H))
                p.setPen(QPen(QColor("#bdbdbd")))
                p.drawText(QRectF(bx + BAR_W + 3, yy - 3, 14, BAR_H + 6), Qt.AlignmentFlag.AlignLeft, tag)
        p.end()
