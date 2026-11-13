"""U6.4: 2D bars drawn over the 3D view next to every joint (torque and speed load).

``LoadBars`` is a transparent widget on top of the viewport (mouse events pass through). Each item is
``(x, y, label, torque_ratio, speed_ratio)`` in widget pixels. A small plate per joint holds the joint
name and two bars, T (torque / effort limit) and S (speed / velocity limit), each with its percentage.
A bar is green up to 70 % of the limit, orange up to 100 %, red above (the bar is full and the number
tells the overload). Plates that would overlap are pushed apart and joined to their joint by a thin line.
"""
from __future__ import annotations

from typing import Any, Sequence

from PySide6.QtCore import QEvent, QPointF, QRectF, Qt, Signal
from PySide6.QtGui import QColor, QFont, QPainter, QPen
from PySide6.QtWidgets import QWidget

GREEN = "#66bb6a"
ORANGE = "#ff9800"
RED = "#ff3030"
WARN_AT = 0.7
PLATE_W, PLATE_H = 112, 38
BAR_W, BAR_H = 52, 6
GAP = 3


def load_color(ratio: float) -> str:
    """Bar colour for a load ratio (value / limit)."""
    if ratio > 1.0:
        return RED
    if ratio > WARN_AT:
        return ORANGE
    return GREEN


def layout_plates(anchors: Sequence[tuple[float, float]], width: float, height: float) -> list[QRectF]:
    """Plate rectangle for every anchor: right of the joint, pushed down until nothing overlaps."""
    placed: list[QRectF] = []
    out: list[QRectF | None] = [None] * len(anchors)
    order = sorted(range(len(anchors)), key=lambda i: (anchors[i][1], anchors[i][0]))
    for i in order:
        x, y = anchors[i]
        if x != x or y != y:
            continue
        r = QRectF(x + 20, y - PLATE_H / 2, PLATE_W, PLATE_H)
        for _ in range(40):
            hit = next((p for p in placed if p.adjusted(-GAP, -GAP, GAP, GAP).intersects(r)), None)
            if hit is None:
                break
            r.moveTop(hit.bottom() + GAP)
        if width > 0 and r.right() > width:
            r.moveRight(max(width - 2, PLATE_W))
        if height > 0 and r.bottom() > height:
            r.moveBottom(height - 2)
        placed.append(r)
        out[i] = r
    return out  # type: ignore[return-value]


class LoadBars(QWidget):
    resized = Signal()

    def __init__(self, parent: QWidget):
        super().__init__(parent)
        self.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents, True)
        self.setAttribute(Qt.WidgetAttribute.WA_TranslucentBackground, True)
        self.setAttribute(Qt.WidgetAttribute.WA_NoSystemBackground, True)
        self.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.items: list[tuple] = []
        self.plates: list[QRectF | None] = []
        self.setAccessibleName("Joint load bars")
        parent.installEventFilter(self)
        self._fit()

    def eventFilter(self, obj: Any, ev: Any) -> bool:
        if ev.type() == QEvent.Type.Resize:
            self._fit()
            self.resized.emit()
        return False

    def _fit(self) -> None:
        p = self.parentWidget()
        if p is not None:
            self.setGeometry(0, 0, p.width(), p.height())
            self.raise_()

    def set_items(self, items: Sequence[tuple]) -> None:
        self.items = list(items)
        self.plates = layout_plates([(it[0], it[1]) for it in self.items], self.width(), self.height())
        self.setAccessibleDescription("; ".join(
            f"{it[2]}: torque {it[3] * 100:.0f} percent, speed {it[4] * 100:.0f} percent" for it in self.items))
        self.update()

    def paintEvent(self, _ev) -> None:
        if not self.items:
            return
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        f = QFont()
        f.setPointSizeF(8.5)
        p.setFont(f)
        bold = QFont(f)
        bold.setBold(True)
        pairs = [(it, r) for it, r in zip(self.items, self.plates) if r is not None]
        for (x, y, _l, _t, _s), r in pairs:                        # 1. leader lines (under everything)
            p.setPen(QPen(QColor(210, 210, 215, 170), 1.0))
            p.drawLine(QPointF(x, y), QPointF(r.left(), r.center().y()))
        for (x, y, label, tr, sr), r in pairs:                     # 2. plates
            # plate
            p.setPen(Qt.PenStyle.NoPen)
            p.setBrush(QColor(20, 20, 26, 225))
            p.drawRoundedRect(r, 4, 4)
            p.setFont(bold)
            p.setPen(QPen(QColor("#f2f2f2")))
            p.drawText(QRectF(r.left() + 6, r.top() + 1, 40, 14), Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter,
                       str(label))
            p.setFont(f)
            for row, (ratio, tag) in enumerate(((tr, "T"), (sr, "S"))):
                yy = r.top() + 16 + row * (BAR_H + GAP + 2)
                p.setPen(QPen(QColor("#cfcfd6")))
                p.drawText(QRectF(r.left() + 6, yy - 4, 10, BAR_H + 8), Qt.AlignmentFlag.AlignCenter, tag)
                bx = r.left() + 19
                p.setPen(Qt.PenStyle.NoPen)
                p.setBrush(QColor("#4a4a54"))
                p.drawRect(QRectF(bx, yy, BAR_W, BAR_H))
                p.setBrush(QColor(load_color(ratio)))
                p.drawRect(QRectF(bx, yy, BAR_W * min(max(ratio, 0.0), 1.0), BAR_H))
                p.setPen(QPen(QColor(RED if ratio > 1.0 else "#e8e8e8")))
                p.drawText(QRectF(bx + BAR_W + 4, yy - 4, 36, BAR_H + 8),
                           Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter,
                           f"{ratio * 100:.0f} %")
        for (x, y, *_r), _plate in pairs:                          # 3. anchor dots on top
            p.setPen(QPen(QColor(20, 20, 24), 1.5))
            p.setBrush(QColor("#f2f2f2"))
            p.drawEllipse(QPointF(x, y), 3.2, 3.2)
        p.end()
