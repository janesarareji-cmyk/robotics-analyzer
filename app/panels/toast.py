"""Small non-modal hint card at the bottom of the 3D view (D1.3).

``Toast(host)`` floats over ``host`` (the viewport), shows a message and an optional action button,
closes with the x button or after ``timeout_ms``. ``show_message(text, action_text, callback)``.
"""
from __future__ import annotations

from typing import Any, Callable

from PySide6.QtCore import QEvent, QObject, QTimer, Qt
from PySide6.QtWidgets import QFrame, QHBoxLayout, QLabel, QPushButton, QWidget

STYLE = """
QFrame#toast { background: #263248; border: 1px solid #4f8cff; border-radius: 8px; }
QLabel#toast_text { color: #ffffff; font-size: 10.5pt; background: transparent; }
QPushButton#toast_action { color: #ffffff; background: #3d6fcc; border: none; border-radius: 4px;
                           padding: 6px 14px; font-size: 10.5pt; }
QPushButton#toast_action:hover { background: #4f80dd; }
QPushButton#toast_close { color: #d0d8ea; background: transparent; border: none; font-size: 12pt;
                          padding: 2px 8px; }
QPushButton#toast_close:hover { color: #ffffff; }
"""


class Toast(QFrame):
    def __init__(self, host: QWidget):
        super().__init__(host)
        self.setObjectName("toast")
        self.setStyleSheet(STYLE)
        self._host = host
        lay = QHBoxLayout(self)
        lay.setContentsMargins(14, 8, 8, 8)
        lay.setSpacing(10)
        self._label = QLabel()
        self._label.setObjectName("toast_text")
        self._label.setWordWrap(True)
        self._action = QPushButton()
        self._action.setObjectName("toast_action")
        self._action.clicked.connect(self._on_action)
        self._close = QPushButton("x")
        self._close.setObjectName("toast_close")
        self._close.setToolTip("Dismiss")
        self._close.clicked.connect(self.hide)
        lay.addWidget(self._label, 1)
        lay.addWidget(self._action)
        lay.addWidget(self._close)
        self._callback: Callable[[], Any] | None = None
        self._timer = QTimer(self)
        self._timer.setSingleShot(True)
        self._timer.timeout.connect(self.hide)
        host.installEventFilter(_HostWatcher(self, host))
        self.hide()

    @property
    def text(self) -> str:
        return self._label.text()

    @property
    def action_button(self) -> QPushButton:
        return self._action

    def show_message(self, text: str, action_text: str = "", callback: Callable[[], Any] | None = None,
                     timeout_ms: int = 15000) -> None:
        self._label.setText(text)
        self._callback = callback
        self._action.setText(action_text)
        self._action.setVisible(bool(action_text) and callback is not None)
        self._place()
        self.show()
        self.raise_()
        if timeout_ms > 0:
            self._timer.start(timeout_ms)
        else:
            self._timer.stop()

    def _on_action(self) -> None:
        cb, self._callback = self._callback, None
        self.hide()
        if cb is not None:
            cb()

    def _place(self) -> None:
        w = min(max(360, self._host.width() - 80), 760)
        self.setFixedWidth(w)
        self.adjustSize()
        self.move(max(0, (self._host.width() - self.width()) // 2),
                  max(0, self._host.height() - self.height() - 24))

    def hideEvent(self, e) -> None:           # noqa: N802
        self._timer.stop()
        super().hideEvent(e)


class _HostWatcher(QObject):
    def __init__(self, toast: Toast, host: QWidget):
        super().__init__(toast)
        self._toast = toast

    def eventFilter(self, obj: Any, ev: Any) -> bool:
        if ev.type() == QEvent.Type.Resize and self._toast.isVisible():
            self._toast._place()
        return False
