"""
app/panels/model_panel.py
Robot model tree panel — placeholder for R1.1.
Full robot model tree added in R2.
"""

from PySide6.QtWidgets import QWidget, QVBoxLayout, QLabel, QTreeWidget
from PySide6.QtCore import Qt


class ModelPanel(QWidget):
    """Robot model panel — displays the robot link/joint tree."""

    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(2)

        label = QLabel("Robot Model")
        label.setStyleSheet("font-weight: bold; color: #aaaaaa;")
        layout.addWidget(label)

        self._tree = QTreeWidget()
        self._tree.setHeaderLabel("No robot loaded")
        self._tree.setStyleSheet("background: #1e1e1e; color: #cccccc; border: none;")
        layout.addWidget(self._tree)
