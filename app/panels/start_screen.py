"""U5.5: start screen shown over the 3D view while no robot is loaded.

Grid of the sample robots (thumbnail + name), Import URDF, Open project, recent projects.
"""
from __future__ import annotations

import os
from pathlib import Path
from typing import Any

from PySide6.QtCore import QEvent, QObject, QSize, Qt, Signal
from PySide6.QtGui import QIcon, QPixmap
from PySide6.QtWidgets import (QFrame, QGridLayout, QHBoxLayout, QLabel, QPushButton, QScrollArea,
                               QToolButton, QVBoxLayout, QWidget)

from app.sample_robots import SAMPLE_INFO, list_sample_robots, sample_info

THUMB = QSize(156, 102)
CARD = QSize(176, 222)         # fixed for every card: name wraps to 2 lines, never elided
GAP = 12

STYLE = """
QWidget#start_screen { background: #1a1a2e; }
QLabel#start_title { color: #ffffff; font-size: 20pt; font-weight: 600; }
QLabel#start_sub { color: #c8c8d4; font-size: 11pt; }
QLabel#start_head { color: #e0e0e8; font-size: 12pt; font-weight: 600; }
QToolButton#start_card { background: #24243a; border: 1px solid #3a3a55; border-radius: 6px; }
QLabel#card_name { color: #ffffff; font-size: 10.5pt; font-weight: 600; background: transparent; }
QLabel#card_maker { color: #c0c0d0; font-size: 9pt; background: transparent; }
QLabel#card_kind { color: #9fc0ff; font-size: 9pt; background: transparent; }
QLabel#card_thumb { background: #1a1a2e; border-radius: 4px; }
QScrollArea#start_scroll, QScrollArea#start_scroll > QWidget > QWidget { background: transparent; }
QScrollArea#start_scroll { border: none; }
QToolButton#start_card:hover { background: #2f2f4d; border-color: #4f8cff; }
QToolButton#start_card:focus { border: 2px solid #ffffff; }
QPushButton#start_action { color: #ffffff; background: #3d6fcc; border: none; border-radius: 4px;
                           padding: 8px 18px; font-size: 10.5pt; }
QPushButton#start_action:hover { background: #4f80dd; }
QPushButton#start_action:focus { border: 2px solid #ffffff; }
QPushButton#start_recent { color: #d8d8e8; background: transparent; border: none; text-align: left;
                           padding: 4px 6px; font-size: 10pt; }
QPushButton#start_recent:hover { color: #ffffff; text-decoration: underline; }
QPushButton#start_recent:focus { border: 1px solid #ffffff; }
"""


class StartScreen(QWidget):
    sample_requested = Signal(str)       # URDF path
    import_requested = Signal()
    open_requested = Signal()
    recent_requested = Signal(str)       # project path

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self.setObjectName("start_screen")
        self.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        self.setStyleSheet(STYLE)
        self._cards: dict[str, QToolButton] = {}
        self._recent_buttons: list[QPushButton] = []
        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        self._scroll = QScrollArea()
        self._scroll.setObjectName("start_scroll")
        self._scroll.setWidgetResizable(True)
        self._scroll.setFrameShape(QFrame.Shape.NoFrame)
        self._scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        root.addWidget(self._scroll)
        content = QWidget()
        content.setObjectName("start_content")
        self._scroll.setWidget(content)
        outer = QVBoxLayout(content)
        outer.setContentsMargins(32, 16, 32, 16)
        outer.setSpacing(8)
        title = QLabel("Robotics Analyzer")
        title.setObjectName("start_title")
        sub = QLabel("Pick a sample robot to start. Then move it, set a target, plan around objects and simulate.")
        sub.setObjectName("start_sub")
        sub.setWordWrap(True)
        outer.addWidget(title)
        outer.addWidget(sub)
        head = QLabel("Sample robots")
        head.setObjectName("start_head")
        outer.addWidget(head)
        self._grid = QGridLayout()
        self._grid.setSpacing(12)
        self._grid.setAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignTop)
        self._cols = 0
        outer.addLayout(self._grid)
        self._build_cards()
        row = QHBoxLayout()
        self.btn_import = QPushButton("Import URDF...")
        self.btn_import.setObjectName("start_action")
        self.btn_import.setToolTip("Import a robot from a URDF file (or drop the file on the window)")
        self.btn_import.clicked.connect(lambda _c=False: self.import_requested.emit())
        self.btn_open = QPushButton("Open project...")
        self.btn_open.setObjectName("start_action")
        self.btn_open.setToolTip("Open a saved .robotproj project (Ctrl+O)")
        self.btn_open.clicked.connect(lambda _c=False: self.open_requested.emit())
        row.addWidget(self.btn_import)
        row.addWidget(self.btn_open)
        row.addStretch(1)
        outer.addLayout(row)
        self._recent_head = QLabel("Recent projects")
        self._recent_head.setObjectName("start_head")
        outer.addWidget(self._recent_head)
        self._recent_box = QVBoxLayout()
        self._recent_box.setSpacing(0)
        outer.addLayout(self._recent_box)
        outer.addStretch(1)

    # ------------------------------------------------------------------ content
    def _build_cards(self) -> None:
        order = list(SAMPLE_INFO)
        items = sorted(list_sample_robots(), key=lambda t: (order.index(t[1].parent.name)
                                                          if t[1].parent.name in order else 99, t[0]))
        for title, path in items:
            stem = Path(path).parent.name
            info = sample_info(stem, title)
            btn = QToolButton()
            btn.setObjectName("start_card")
            btn.setFixedSize(CARD)
            lay = QVBoxLayout(btn)
            lay.setContentsMargins(9, 9, 9, 6)
            lay.setSpacing(2)
            thumb = QLabel()
            thumb.setObjectName("card_thumb")
            thumb.setFixedSize(THUMB)
            thumb.setAlignment(Qt.AlignmentFlag.AlignCenter)
            tp = Path(path).parent / "thumb.png"
            if tp.is_file():
                thumb.setPixmap(QPixmap(str(tp)).scaled(
                    THUMB, Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.SmoothTransformation))
            name = QLabel(info["name"])
            name.setObjectName("card_name")
            name.setWordWrap(True)
            name.setFixedHeight(40)          # two lines reserved: all cards the same height
            name.setAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignTop)
            maker = QLabel(info["maker"])
            maker.setObjectName("card_maker")
            dof = info["dof"]
            kind = QLabel(f"{dof} DOF - {info['kind']}" if dof else info["kind"])
            kind.setObjectName("card_kind")
            kind.setWordWrap(True)
            kind.setMinimumHeight(30)
            kind.setAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignTop)
            for w in (thumb, name, maker, kind):
                w.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents, True)
            lay.addWidget(thumb, 0, Qt.AlignmentFlag.AlignHCenter)
            lay.addWidget(name)
            lay.addWidget(maker)
            lay.addWidget(kind)
            lay.addStretch(1)
            btn.setToolTip(f"Open the sample robot {info['name']} ({info['maker']}, {info['kind']})")
            btn.setAccessibleName(f"Open sample robot {info['name']}")
            btn.setCursor(Qt.CursorShape.PointingHandCursor)
            btn.clicked.connect(lambda _c=False, p=str(path): self.sample_requested.emit(p))
            self._cards[stem] = btn
        self._relayout(force=True)

    def _relayout(self, force: bool = False) -> None:
        """Columns follow the width; the screen scrolls when the cards do not fit."""
        avail = max(self.width() - 64, 0)
        n = len(self._cards) or 1
        cols = max(1, min(n, (avail + GAP) // (CARD.width() + GAP)))
        cols = -(-n // (-(-n // cols)))          # even rows (9 cards -> 5+4, or 3+3+3)
        if cols == self._cols and not force:
            return
        self._cols = cols
        for i, btn in enumerate(self._cards.values()):
            self._grid.addWidget(btn, i // cols, i % cols)

    def resizeEvent(self, ev) -> None:           # noqa: N802
        super().resizeEvent(ev)
        self._relayout()

    @property
    def columns(self) -> int:
        return self._cols

    def card_name(self, stem: str) -> str:
        """Display name shown on the card of ``stem`` (full text, never elided)."""
        for lab in self._cards[stem].findChildren(QLabel, "card_name"):
            return lab.text()
        return ""

    def card(self, stem: str) -> QToolButton:
        """Card of the sample robot whose folder name is ``stem``."""
        return self._cards[stem]

    @property
    def card_stems(self) -> list[str]:
        return list(self._cards)

    def set_recent(self, paths: list[str]) -> None:
        for b in self._recent_buttons:
            b.setParent(None)
            b.deleteLater()
        self._recent_buttons = []
        shown = [p for p in paths if p][:5]
        self._recent_head.setVisible(bool(shown))
        for p in shown:
            b = QPushButton(f"{Path(p).stem}    {p}")
            b.setObjectName("start_recent")
            b.setToolTip(p)
            b.setCursor(Qt.CursorShape.PointingHandCursor)
            b.clicked.connect(lambda _c=False, q=p: self.recent_requested.emit(q))
            self._recent_box.addWidget(b)
            self._recent_buttons.append(b)

    @property
    def recent_buttons(self) -> list[QPushButton]:
        return self._recent_buttons


class _Resizer(QObject):
    """Keeps the start screen the size of its parent (the 3D view)."""

    def __init__(self, screen: StartScreen, parent: QWidget):
        super().__init__(parent)
        self._screen = screen

    def eventFilter(self, obj: Any, ev: Any) -> bool:
        if ev.type() == QEvent.Type.Resize:
            self._screen.setGeometry(0, 0, obj.width(), obj.height())
        return False


def attach(screen: StartScreen, host: QWidget) -> None:
    """Make ``screen`` a child of ``host`` covering it."""
    screen.setParent(host)
    host.installEventFilter(_Resizer(screen, host))
    screen.setGeometry(0, 0, host.width(), host.height())
    screen.raise_()
