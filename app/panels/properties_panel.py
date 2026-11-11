"""
app/panels/properties_panel.py
Properties dock panel — placeholder for R1.1.
Full property editor added in R2.
"""

from PySide6.QtWidgets import QWidget, QVBoxLayout, QLabel
from PySide6.QtCore import Qt


class PropertiesPanel(QWidget):
    """Properties panel — displays properties of the selected item."""

    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)

        label = QLabel("Properties")
        label.setStyleSheet("font-weight: bold; color: #aaaaaa;")
        layout.addWidget(label)

        placeholder = QLabel("Select an item to view its properties.")
        placeholder.setWordWrap(True)
        placeholder.setAlignment(Qt.AlignmentFlag.AlignTop)
        placeholder.setStyleSheet("color: #666666; padding: 8px;")
        layout.addWidget(placeholder)
        layout.addStretch()
