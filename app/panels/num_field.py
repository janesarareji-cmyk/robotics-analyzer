"""
app/panels/num_field.py
R4.9: numeric line edit. Accepts scientific notation ("1e-3"). Invalid text
keeps the previous valid value and turns the field red.
"""

from __future__ import annotations

from PySide6.QtCore import Signal
from PySide6.QtWidgets import QLineEdit

_BAD_STYLE = "background-color: #7a2020; color: #ffffff;"


class NumField(QLineEdit):
    """Float field. `value()` is the last valid value; `valid()` the text state."""

    value_changed = Signal(float)

    def __init__(self, value: float = 0.0, parent=None):
        super().__init__(parent)
        self._value = float(value)
        self._valid = True
        self.setText(f"{self._value:g}")
        self.textChanged.connect(self._on_text)
        self.editingFinished.connect(self._on_finished)

    def value(self) -> float:
        return self._value

    def valid(self) -> bool:
        return self._valid

    def set_value(self, v: float) -> None:
        """Set programmatically (emits value_changed only when the number changed)."""
        self.setText(f"{float(v):.9g}")

    def _on_text(self, text: str) -> None:
        try:
            v = float(text.strip())
            if v != v or v in (float("inf"), float("-inf")):
                raise ValueError
        except ValueError:
            self._valid = False
            self.setStyleSheet(_BAD_STYLE)
            return
        self._valid = True
        self.setStyleSheet("")
        if v != self._value:
            self._value = v
            self.value_changed.emit(v)
        else:
            self._value = v

    def _on_finished(self) -> None:
        # Invalid text stays visible and red; the value stays the previous one.
        pass
