"""
app/panels/dh_editor.py
Robotics Analyzer — Standard / Modified DH Parameter Editor panel.
R2.3, R2.6: Table widget for viewing and editing DH parameters per joint,
convention, base/tool poses, and model validation status.
Backed by a urm.RobotModel; emits model_changed on every edit.
"""

from __future__ import annotations

import math
from typing import Any

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QTableWidget,
    QTableWidgetItem, QLabel, QPushButton, QHeaderView, QAbstractItemView,
    QComboBox, QDoubleSpinBox, QGroupBox, QScrollArea
)
from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QColor

import core_bridge as cb


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
    Table and configuration editor for robot kinematic parameters.

    Usage:
        editor = DHEditorWidget()
        editor.set_model(robot_model)          # populate from RobotModel
        editor.model_changed.connect(handler)  # notified on every edit
    """

    model_changed = Signal()  # Emitted when any DH parameter, pose, or setting is edited

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self._model: Any = None
        self._updating: bool = False  # Guard against recursive signal loops
        self._setup_ui()

    def _setup_ui(self) -> None:
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(4, 4, 4, 4)
        main_layout.setSpacing(6)

        # Header
        header = QLabel("Robot Kinematics (DH) Editor")
        header.setStyleSheet("font-weight: bold; font-size: 13px;")
        main_layout.addWidget(header)

        # Configuration Row: Convention
        conv_row = QHBoxLayout()
        conv_lbl = QLabel("Convention:")
        conv_lbl.setStyleSheet("font-weight: 500;")
        self._combo_convention = QComboBox()
        self._combo_convention.addItems(["Standard", "Modified"])
        self._combo_convention.currentIndexChanged.connect(self._on_convention_changed)
        conv_row.addWidget(conv_lbl)
        conv_row.addWidget(self._combo_convention)
        conv_row.addStretch()
        main_layout.addLayout(conv_row)

        # Poses: Base & Tool
        poses_box = QGroupBox("Base & Tool Offsets")
        poses_layout = QGridLayout(poses_box)
        poses_layout.setContentsMargins(6, 6, 6, 6)
        poses_layout.setSpacing(4)

        # Base Pose Spinboxes
        poses_layout.addWidget(QLabel("Base XYZ [m]:"), 0, 0)
        self._base_x = self._create_spinbox(-1000.0, 1000.0, 4, 0.01)
        self._base_y = self._create_spinbox(-1000.0, 1000.0, 4, 0.01)
        self._base_z = self._create_spinbox(-1000.0, 1000.0, 4, 0.01)
        poses_layout.addWidget(self._base_x, 0, 1)
        poses_layout.addWidget(self._base_y, 0, 2)
        poses_layout.addWidget(self._base_z, 0, 3)

        poses_layout.addWidget(QLabel("Base RPY [rad]:"), 0, 4)
        self._base_r = self._create_spinbox(-100.0, 100.0, 4, 0.05)
        self._base_p = self._create_spinbox(-100.0, 100.0, 4, 0.05)
        self._base_yaw = self._create_spinbox(-100.0, 100.0, 4, 0.05)
        poses_layout.addWidget(self._base_r, 0, 5)
        poses_layout.addWidget(self._base_p, 0, 6)
        poses_layout.addWidget(self._base_yaw, 0, 7)

        # Tool Pose Spinboxes
        poses_layout.addWidget(QLabel("Tool XYZ [m]:"), 1, 0)
        self._tool_x = self._create_spinbox(-1000.0, 1000.0, 4, 0.01)
        self._tool_y = self._create_spinbox(-1000.0, 1000.0, 4, 0.01)
        self._tool_z = self._create_spinbox(-1000.0, 1000.0, 4, 0.01)
        poses_layout.addWidget(self._tool_x, 1, 1)
        poses_layout.addWidget(self._tool_y, 1, 2)
        poses_layout.addWidget(self._tool_z, 1, 3)

        poses_layout.addWidget(QLabel("Tool RPY [rad]:"), 1, 4)
        self._tool_r = self._create_spinbox(-100.0, 100.0, 4, 0.05)
        self._tool_p = self._create_spinbox(-100.0, 100.0, 4, 0.05)
        self._tool_yaw = self._create_spinbox(-100.0, 100.0, 4, 0.05)
        poses_layout.addWidget(self._tool_r, 1, 5)
        poses_layout.addWidget(self._tool_p, 1, 6)
        poses_layout.addWidget(self._tool_yaw, 1, 7)

        # Connect base/tool signals
        for sb in [self._base_x, self._base_y, self._base_z, self._base_r, self._base_p, self._base_yaw]:
            sb.valueChanged.connect(self._on_base_changed)
        for sb in [self._tool_x, self._tool_y, self._tool_z, self._tool_r, self._tool_p, self._tool_yaw]:
            sb.valueChanged.connect(self._on_tool_changed)

        main_layout.addWidget(poses_box)

        # Table
        self._table = QTableWidget(0, COL_COUNT)
        self._table.setHorizontalHeaderLabels(HEADERS)
        self._table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self._table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self._table.setAlternatingRowColors(True)
        self._table.setStyleSheet("QTableWidget { font-size: 12px; }")
        self._table.itemChanged.connect(self._on_item_changed)
        main_layout.addWidget(self._table)

        # Buttons row
        btn_row = QHBoxLayout()
        self._btn_add = QPushButton("Add Joint")
        self._btn_add.clicked.connect(self._on_add_joint)
        self._btn_remove = QPushButton("Remove Joint")
        self._btn_remove.clicked.connect(self._on_remove_joint)
        btn_row.addWidget(self._btn_add)
        btn_row.addWidget(self._btn_remove)
        btn_row.addStretch()
        main_layout.addLayout(btn_row)

        # Validation Error Label (empty = hidden)
        self._error_label = QLabel("")
        self._error_label.setStyleSheet(
            "color: #ff6b6b; font-size: 11px; background: rgba(255, 50, 50, 0.1); "
            "border: 1px solid #ff4444; border-radius: 3px; padding: 4px;"
        )
        self._error_label.setWordWrap(True)
        self._error_label.setVisible(False)
        main_layout.addWidget(self._error_label)

    def _create_spinbox(self, min_val: float, max_val: float, decimals: int, step: float) -> QDoubleSpinBox:
        sb = QDoubleSpinBox()
        sb.setRange(min_val, max_val)
        sb.setDecimals(decimals)
        sb.setSingleStep(step)
        return sb

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def set_model(self, model: Any) -> None:
        """Populate the table and controls from a urm.RobotModel."""
        self._updating = True
        try:
            self._model = model
            if self._model is None:
                self._table.setRowCount(0)
                self._error_label.setText("")
                self._error_label.setVisible(False)
                return

            # Convention
            if self._model.convention == cb.DHConvention.Modified:
                self._combo_convention.setCurrentIndex(1)
            else:
                self._combo_convention.setCurrentIndex(0)

            # Base pose
            bx, by, bz = self._model.base.xyz
            br, bp, byaw = self._model.base.rpy
            self._base_x.setValue(bx)
            self._base_y.setValue(by)
            self._base_z.setValue(bz)
            self._base_r.setValue(br)
            self._base_p.setValue(bp)
            self._base_yaw.setValue(byaw)

            # Tool pose
            tx, ty, tz = self._model.tool.xyz
            tr, tp, tyaw = self._model.tool.rpy
            self._tool_x.setValue(tx)
            self._tool_y.setValue(ty)
            self._tool_z.setValue(tz)
            self._tool_r.setValue(tr)
            self._tool_p.setValue(tp)
            self._tool_yaw.setValue(tyaw)

            self._refresh_table_locked()
            self._update_validation()
        finally:
            self._updating = False

    def get_model(self) -> Any:
        """Return the current RobotModel (may be None)."""
        return self._model

    def set_joint_type(self, row: int, joint_type: cb.JointType) -> None:
        """Programmatically set joint type for row, applying default limits."""
        if self._model is None or row < 0 or row >= len(self._model.joints):
            return
        joint = self._model.joints[row]
        joint.type = joint_type
        joint.limits = cb.default_limits(joint_type)
        widget = self._table.cellWidget(row, COL_TYPE)
        type_names = {
            cb.JointType.Revolute:  "Revolute",
            cb.JointType.Prismatic: "Prismatic",
            cb.JointType.Fixed:     "Fixed",
        }
        if isinstance(widget, QComboBox):
            self._updating = True
            widget.setCurrentText(type_names.get(joint_type, "Revolute"))
            self._updating = False
        self._validate_and_notify()

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _refresh_table(self) -> None:
        """Rebuild table with updating guard."""
        self._updating = True
        try:
            self._refresh_table_locked()
            self._update_validation()
        finally:
            self._updating = False

    def _refresh_table_locked(self) -> None:
        """Rebuild table assuming self._updating is True."""
        self._table.setRowCount(0)
        if self._model is None:
            return
        for row, joint in enumerate(self._model.joints):
            self._table.insertRow(row)
            self._set_row(row, joint)

    def _set_row(self, row: int, joint: Any) -> None:
        """Fill one table row from a Joint object."""
        # Joint name — editable
        self._table.setItem(row, COL_JOINT, QTableWidgetItem(joint.name))

        # Type — editable QComboBox cell widget
        type_combo = QComboBox()
        type_combo.addItems(["Revolute", "Prismatic", "Fixed"])
        type_names = {
            cb.JointType.Revolute:  "Revolute",
            cb.JointType.Prismatic: "Prismatic",
            cb.JointType.Fixed:     "Fixed",
        }
        current_name = type_names.get(joint.type, "Revolute")
        type_combo.setCurrentText(current_name)
        # Capture current row via default argument in lambda
        type_combo.currentIndexChanged.connect(
            lambda idx, r=row, c=type_combo: self._on_type_combo_changed(r, c)
        )
        self._table.setCellWidget(row, COL_TYPE, type_combo)

        # Fallback table item for text compatibility
        item_type = QTableWidgetItem(current_name)
        self._table.setItem(row, COL_TYPE, item_type)

        # DH params
        if self._model.convention == cb.DHConvention.Modified:
            a_val = joint.mdh.a
            alpha_val = joint.mdh.alpha
            d_val = joint.mdh.d
            theta_val = joint.mdh.theta
        else:
            a_val = joint.dh.a
            alpha_val = joint.dh.alpha
            d_val = joint.dh.d
            theta_val = joint.dh.theta

        self._table.setItem(row, COL_A,     QTableWidgetItem(f"{a_val:.6f}"))
        self._table.setItem(row, COL_ALPHA, QTableWidgetItem(f"{alpha_val:.6f}"))
        self._table.setItem(row, COL_D,     QTableWidgetItem(f"{d_val:.6f}"))
        self._table.setItem(row, COL_THETA, QTableWidgetItem(f"{theta_val:.6f}"))
        self._table.setItem(row, COL_Q,     QTableWidgetItem(f"{joint.q:.6f}"))

    def _on_type_combo_changed(self, row: int, combo: QComboBox) -> None:
        """Handle joint type change from cell widget combobox."""
        if self._updating or self._model is None:
            return
        if row < 0 or row >= len(self._model.joints):
            return
        type_map = {
            "Revolute":  cb.JointType.Revolute,
            "Prismatic": cb.JointType.Prismatic,
            "Fixed":     cb.JointType.Fixed,
        }
        new_type = type_map.get(combo.currentText())
        if new_type is not None:
            joint = self._model.joints[row]
            joint.type = new_type
            joint.limits = cb.default_limits(new_type)
            # Update background item text as well
            item = self._table.item(row, COL_TYPE)
            if item is not None:
                self._updating = True
                item.setText(combo.currentText())
                self._updating = False
            self._validate_and_notify()

    def _on_item_changed(self, item: QTableWidgetItem) -> None:
        """Write edited cell value back to the model."""
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
            elif col == COL_TYPE:
                type_map = {
                    "revolute":  cb.JointType.Revolute,
                    "prismatic": cb.JointType.Prismatic,
                    "fixed":     cb.JointType.Fixed,
                }
                t = type_map.get(text.lower())
                if t is not None:
                    joint.type = t
                    joint.limits = cb.default_limits(t)
                    widget = self._table.cellWidget(row, COL_TYPE)
                    if isinstance(widget, QComboBox):
                        self._updating = True
                        widget.setCurrentText(text.capitalize())
                        self._updating = False
            elif col == COL_A:
                val = float(text)
                joint.dh.a = val
                joint.mdh.a = val
            elif col == COL_ALPHA:
                val = float(text)
                joint.dh.alpha = val
                joint.mdh.alpha = val
            elif col == COL_D:
                val = float(text)
                joint.dh.d = val
                joint.mdh.d = val
            elif col == COL_THETA:
                val = float(text)
                joint.dh.theta = val
                joint.mdh.theta = val
            elif col == COL_Q:
                val = float(text)
                joint.q = val
            else:
                return
            self._validate_and_notify()
        except ValueError:
            # Revert invalid float
            self._updating = True
            self._set_row(row, joint)
            self._updating = False

    def _on_convention_changed(self, index: int) -> None:
        """Handle convention change (0: Standard, 1: Modified)."""
        if self._updating or self._model is None:
            return
        if index == 1:
            self._model.convention = cb.DHConvention.Modified
        else:
            self._model.convention = cb.DHConvention.Standard
        self._validate_and_notify()

    def _on_base_changed(self) -> None:
        """Assign whole list to Pose.xyz and Pose.rpy."""
        if self._updating or self._model is None:
            return
        self._model.base.xyz = [
            self._base_x.value(),
            self._base_y.value(),
            self._base_z.value(),
        ]
        self._model.base.rpy = [
            self._base_r.value(),
            self._base_p.value(),
            self._base_yaw.value(),
        ]
        self._validate_and_notify()

    def _on_tool_changed(self) -> None:
        """Assign whole list to Pose.xyz and Pose.rpy."""
        if self._updating or self._model is None:
            return
        self._model.tool.xyz = [
            self._tool_x.value(),
            self._tool_y.value(),
            self._tool_z.value(),
        ]
        self._model.tool.rpy = [
            self._tool_r.value(),
            self._tool_p.value(),
            self._tool_yaw.value(),
        ]
        self._validate_and_notify()

    def _on_add_joint(self) -> None:
        """Add a new revolute joint and link to the model."""
        if self._model is None:
            return
        j = cb.Joint()
        j.name = f"joint{len(self._model.joints) + 1}"
        j.type = cb.JointType.Revolute
        j.limits = cb.default_limits(j.type)
        self._model.joints.append(j)
        while len(self._model.links) < len(self._model.joints) + 1:
            lk = cb.Link()
            lk.name = f"link{len(self._model.links)}"
            self._model.links.append(lk)

        self._refresh_table()
        self._validate_and_notify()

    def _on_remove_joint(self) -> None:
        """Remove selected joint(s) from the model."""
        if self._model is None:
            return
        rows = self._table.selectionModel().selectedRows()
        if not rows:
            return
        indices = sorted([r.row() for r in rows], reverse=True)
        joints_list = list(self._model.joints)
        for idx in indices:
            if 0 <= idx < len(joints_list):
                joints_list.pop(idx)
        self._model.joints.clear()
        for j in joints_list:
            self._model.joints.append(j)

        self._refresh_table()
        self._validate_and_notify()

    def _update_validation(self) -> list[str]:
        """Update inline validation error label (empty = hidden)."""
        if self._model is None:
            self._error_label.setText("")
            self._error_label.setVisible(False)
            return []
        errors = self._model.validate()
        if errors:
            self._error_label.setText("\n".join(errors))
            self._error_label.setVisible(True)
        else:
            self._error_label.setText("")
            self._error_label.setVisible(False)
        return errors

    def _validate_and_notify(self) -> None:
        """Update validation state and emit model_changed."""
        self._update_validation()
        self.model_changed.emit()
