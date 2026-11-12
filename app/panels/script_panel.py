"""
app/panels/script_panel.py
R8.1: script console dock: editor, Run / Cancel, output.

The panel only talks to a ``ScriptRunner`` (``set_runner``). The runner executes the script in a
worker thread, so the UI stays responsive while it runs. Output lines arrive through the runner
``output`` signal (queued to the GUI thread).
"""

from __future__ import annotations

from typing import Any

from PySide6.QtGui import QFont
from PySide6.QtWidgets import (
    QHBoxLayout, QLabel, QPlainTextEdit, QPushButton, QVBoxLayout, QWidget,
)

EXAMPLE_TEXT = (
    "# robotics_api and app are already imported.\n"
    "print('API version', api.__api_version__)\n"
    "print('project:', app.project_name)\n"
)


class ScriptPanel(QWidget):
    """Script editor with Run / Cancel buttons and an output view."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._runner: Any = None

        layout = QVBoxLayout(self)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(4)

        self._editor = QPlainTextEdit()
        self._editor.setFont(QFont("Consolas", 10))
        self._editor.setPlainText(EXAMPLE_TEXT)
        self._editor.setTabStopDistance(32)
        layout.addWidget(self._editor, 3)

        row = QHBoxLayout()
        self._run = QPushButton("Run")
        self._run.setToolTip("Run the script in the editor (worker thread)")
        self._cancel = QPushButton("Cancel")
        self._cancel.setToolTip("Stop the script at the next API call that checks for cancel")
        self._cancel.setEnabled(False)
        self._clear = QPushButton("Clear output")
        self._status = QLabel("Idle")
        for w in (self._run, self._cancel, self._clear):
            row.addWidget(w)
        row.addWidget(self._status, 1)
        layout.addLayout(row)

        self._out = QPlainTextEdit()
        self._out.setReadOnly(True)
        self._out.setFont(QFont("Consolas", 9))
        self._out.setPlaceholderText("Script output appears here...")
        layout.addWidget(self._out, 2)

        self._run.clicked.connect(self.run_editor)
        self._cancel.clicked.connect(self.cancel)
        self._clear.clicked.connect(self._out.clear)

    # ----------------------------------------------------------------- wiring
    def set_runner(self, runner: Any) -> None:
        self._runner = runner
        runner.started.connect(self._on_started)
        runner.output.connect(self._on_output)
        runner.finished.connect(self._on_finished)

    # ----------------------------------------------------------------- actions
    def run_editor(self) -> None:
        if self._runner is not None:
            self._runner.run_string(self._editor.toPlainText(), "<editor>")

    def cancel(self) -> None:
        if self._runner is not None:
            self._runner.cancel()
            self._status.setText("Cancelling...")

    def load_text(self, text: str) -> None:
        self._editor.setPlainText(text)

    # ----------------------------------------------------------------- runner slots
    def _on_started(self) -> None:
        self._run.setEnabled(False)
        self._cancel.setEnabled(True)
        self._status.setText("Running...")

    def _on_output(self, text: str, is_err: bool) -> None:
        self._out.appendPlainText(("[err] " if is_err else "") + text)

    def _on_finished(self, status: str, message: str) -> None:
        self._run.setEnabled(True)
        self._cancel.setEnabled(False)
        self._status.setText({"ok": "Finished", "error": "Error (see output)",
                              "cancelled": "Cancelled", "timeout": "Time limit reached"}
                             .get(status, status))
        if status in ("cancelled", "timeout"):
            self._out.appendPlainText(f"[{status}] {message}")

    # ----------------------------------------------------------------- test access
    @property
    def editor(self) -> QPlainTextEdit:
        return self._editor

    @property
    def run_button(self) -> QPushButton:
        return self._run

    @property
    def cancel_button(self) -> QPushButton:
        return self._cancel

    def output_text(self) -> str:
        return self._out.toPlainText()

    def status_text(self) -> str:
        return self._status.text()
