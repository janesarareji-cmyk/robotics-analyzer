"""
app/panels/log_panel.py
Log output dock panel — placeholder for R1.1.
Full logging integration added in R1.3.
"""

from PySide6.QtWidgets import QWidget, QVBoxLayout, QTextEdit, QLabel
from PySide6.QtCore import Qt
from PySide6.QtGui import QFont


class LogPanel(QWidget):
    """Log output panel — displays application log messages."""

    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(2)

        label = QLabel("Log Output")
        label.setStyleSheet("font-weight: bold; color: #aaaaaa;")
        layout.addWidget(label)

        self._text = QTextEdit()
        self._text.setReadOnly(True)
        self._text.setFont(QFont("Consolas", 9))
        self._text.setStyleSheet("background: #1a1a1a; color: #cccccc; border: none;")
        self._text.setPlaceholderText("Application log messages appear here...")
        layout.addWidget(self._text)

    def append(self, message: str) -> None:
        """Append a log message to the panel."""
        self._text.append(message)

    def clear_log(self) -> None:
        """Clear all log messages."""
        self._text.clear()
