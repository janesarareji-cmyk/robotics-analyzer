"""
app/panels/dh_editor.py
Robotics Analyzer — Standard DH Parameter Editor panel.
R2.3: Table widget for viewing and editing standard DH parameters per joint.
Backed by a urm.RobotModel; emits model_changed when any cell is edited.
"""

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QTableWidget, QTableWidgetItem,
    QLabel, QPushButton, QHeaderView, QAbstractItemView
)
from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QColor
import math


# Column indices for the DH table
COL_JOINT   = 0
COL_TYPE    = 1
COL_A       = 2
COL_ALPHA   = 3
COL_D       = 4
COL_THETA   = 5
COL_Q       = 6
COL_COUNT   = 7

HEADERS = ["Joint", "Type", "a [m]", "α [rad]", "d [m]", "θ [rad]", "q [rad/m]"]


class DHEditorWidget(QWidget):
    """
    Table editor for standard DH parameters.

    Usage:
        editor = DHEditorWidget()
        editor.set_model(robot_model)   # populate from RobotModel
        editor.model_changed.connect(handler)  # notified on every edit
    """

    model_changed = Signal()  # Emitted when any DH parameter is edited

    def __init__(self, parent=None):
        super().__init__(parent)
        self._model = None
        self._updating = False   # Guard against recursive signal loops
        self._setup_ui()

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(4)

        # Header label
        header = QLabel("Standard DH Parameters")
        header.setStyleSheet("font-weight: bold; font-size: 13px;")
        layout.addWidget(header)

        # Table
        self._table = QTableWidget(0, COL_COUNT)
        self._table.setHorizontalHeaderLabels(HEADERS)
        self._table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self._table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self._table.setAlternatingRowColors(True)
        self._table.setStyleSheet("QTableWidget { font-size: 12px; }")
        self._table.itemChanged.connect(self._on_item_changed)
        layout.addWidget(self._table)

        # Buttons row
        btn_row = QHBoxLayout()
        self._btn_add = QPushButton("Add Joint")
        self._btn_add.clicked.connect(self._on_add_joint)
        self._btn_remove = QPushButton("Remove Joint")
        self._btn_remove.clicked.connect(self._on_remove_joint)
        btn_row.addWidget(self._btn_add)
        btn_row.addWidget(self._btn_remove)
        btn_row.addStretch()
        layout.addLayout(btn_row)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def set_model(self, model) -> None:
        """Populate the table from a urm.RobotModel."""
        self._model = model
        self._refresh_table()

    def get_model(self):
        """Return the current RobotModel (may be None)."""
        return self._model

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _refresh_table(self) -> None:
        """Rebuild the table from the current model."""
        self._updating = True
        try:
            self._table.setRowCount(0)
            if self._model is None:
                return
            for row, joint in enumerate(self._model.joints):
                self._table.insertRow(row)
                self._set_row(row, joint)
        finally:
            self._updating = False

    def _set_row(self, row: int, joint) -> None:
        """Fill one table row from a Joint object."""
        import core_bridge as cb

        # Joint name — editable
        self._table.setItem(row, COL_JOINT, QTableWidgetItem(joint.name))

        # Type — read-only label
        type_names = {
            cb.JointType.Revolute:  "Revolute",
            cb.JointType.Prismatic: "Prismatic",
            cb.JointType.Fixed:     "Fixed",
        }
        type_item = QTableWidgetItem(type_names.get(joint.type, "?"))
        type_item.setFlags(type_item.flags() & ~Qt.ItemFlag.ItemIsEditable)
        type_item.setBackground(QColor("#2a2a3e"))
        self._table.setItem(row, COL_TYPE, type_item)

        # DH params — editable
        self._table.setItem(row, COL_A,     QTableWidgetItem(f"{joint.dh.a:.6f}"))
        self._table.setItem(row, COL_ALPHA, QTableWidgetItem(f"{joint.dh.alpha:.6f}"))
        self._table.setItem(row, COL_D,     QTableWidgetItem(f"{joint.dh.d:.6f}"))
        self._table.setItem(row, COL_THETA, QTableWidgetItem(f"{joint.dh.theta:.6f}"))
        self._table.setItem(row, COL_Q,     QTableWidgetItem(f"{joint.q:.6f}"))

    def _on_item_changed(self, item: QTableWidgetItem) -> None:
        """Write edited value back to the model."""
        if self._updating or self._model is None:
            return
        row = item.row()
        col = item.column()
        if row < 0 or row >= len(self._model.joints):
            return

        joint = self._model.joints[row]
        text = item.text().strip()

        try:
            if col == COL_JOINT:
                joint.name = text
            elif col == COL_A:
                joint.dh.a = float(text)
            elif col == COL_ALPHA:
                joint.dh.alpha = float(text)
            elif col == COL_D:
                joint.dh.d = float(text)
            elif col == COL_THETA:
                joint.dh.theta = float(text)
            elif col == COL_Q:
                joint.q = float(text)
            else:
                return
            self.model_changed.emit()
        except ValueError:
            # Invalid float — revert to current model value
            self._updating = True
            self._set_row(row, joint)
            self._updating = False

    def _on_add_joint(self) -> None:
        """Add a new default revolute joint to the model."""
        if self._model is None:
            return
        import core_bridge as cb
        j = cb.Joint()
        j.name = f"joint{len(self._model.joints) + 1}"
        j.type = cb.JointType.Revolute
        self._model.joints.append(j)
        self._refresh_table()
        self.model_changed.emit()

    def _on_remove_joint(self) -> None:
        """Remove the selected joint from the model."""
        if self._model is None:
            return
        rows = self._table.selectionModel().selectedRows()
        if not rows:
            return
        # Remove in reverse order to avoid index shifting
        indices = sorted([r.row() for r in rows], reverse=True)
        # Build new joints list without removed indices
        joints_list = list(self._model.joints)
        for idx in indices:
            if 0 <= idx < len(joints_list):
                joints_list.pop(idx)
        # Rebuild model joints
        self._model.joints.clear()
        for j in joints_list:
            self._model.joints.append(j)
        self._refresh_table()
        self.model_changed.emit()
