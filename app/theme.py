"""U5.10: one theme: Fusion style, dark palette with one accent (blue), glyph icons for the toolbar.

Fusion draws the same controls on every machine (the native Windows style would take the system
accent colour, e.g. coral, for sliders and check boxes) and shows a clear focus frame on Tab.
"""
from __future__ import annotations

from PySide6.QtCore import QRectF, QSize, Qt
from PySide6.QtGui import QColor, QFont, QIcon, QPainter, QPalette, QPixmap
from PySide6.QtWidgets import QApplication

ACCENT = "#3d6fcc"
ACCENT_LIGHT = "#4f8cff"
WINDOW = "#1e1e1e"
BASE = "#181818"
TEXT = "#e0e0e0"
MUTED = "#a8a8a8"            # on WINDOW: about 7:1
DISABLED = "#7d7d7d"


def dark_palette() -> QPalette:
    p = QPalette()
    p.setColor(QPalette.ColorRole.Window, QColor(WINDOW))
    p.setColor(QPalette.ColorRole.WindowText, QColor(TEXT))
    p.setColor(QPalette.ColorRole.Base, QColor(BASE))
    p.setColor(QPalette.ColorRole.AlternateBase, QColor("#222222"))
    p.setColor(QPalette.ColorRole.Text, QColor(TEXT))
    p.setColor(QPalette.ColorRole.Button, QColor("#333333"))
    p.setColor(QPalette.ColorRole.ButtonText, QColor(TEXT))
    p.setColor(QPalette.ColorRole.BrightText, QColor("#ffffff"))
    p.setColor(QPalette.ColorRole.ToolTipBase, QColor("#2b2b36"))
    p.setColor(QPalette.ColorRole.ToolTipText, QColor("#f0f0f0"))
    p.setColor(QPalette.ColorRole.PlaceholderText, QColor(MUTED))
    p.setColor(QPalette.ColorRole.Highlight, QColor(ACCENT))
    p.setColor(QPalette.ColorRole.HighlightedText, QColor("#ffffff"))
    p.setColor(QPalette.ColorRole.Link, QColor(ACCENT_LIGHT))
    p.setColor(QPalette.ColorRole.Light, QColor("#454545"))
    p.setColor(QPalette.ColorRole.Mid, QColor("#2a2a2a"))
    p.setColor(QPalette.ColorRole.Dark, QColor("#141414"))
    for role in (QPalette.ColorRole.WindowText, QPalette.ColorRole.Text, QPalette.ColorRole.ButtonText):
        p.setColor(QPalette.ColorGroup.Disabled, role, QColor(DISABLED))
    return p


def apply_theme(app: QApplication | None = None) -> None:
    app = app or QApplication.instance()
    if app is None:
        return
    if app.style().name().lower() != "fusion":
        app.setStyle("Fusion")
    if app.palette().color(QPalette.ColorRole.Highlight).name() != ACCENT:
        app.setPalette(dark_palette())


def glyph_icon(char: str, color: str = "#d8d8d8", size: int = 20, active: str | None = None) -> QIcon:
    """Monochrome icon from one Unicode symbol (Segoe UI Symbol). ``active``: colour of the On state."""
    def pix(col: str) -> QPixmap:
        pm = QPixmap(size * 2, size * 2)
        pm.fill(Qt.GlobalColor.transparent)
        pr = QPainter(pm)
        pr.setRenderHint(QPainter.RenderHint.TextAntialiasing)
        f = QFont("Segoe UI Symbol")
        f.setPixelSize(int(size * 1.55))
        pr.setFont(f)
        pr.setPen(QColor(col))
        pr.drawText(QRectF(0, 0, size * 2, size * 2), Qt.AlignmentFlag.AlignCenter, char)
        pr.end()
        pm.setDevicePixelRatio(2.0)
        return pm
    icon = QIcon()
    icon.addPixmap(pix(color), QIcon.Mode.Normal, QIcon.State.Off)
    icon.addPixmap(pix(active or color), QIcon.Mode.Normal, QIcon.State.On)
    return icon
