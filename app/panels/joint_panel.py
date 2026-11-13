"""
app/panels/joint_panel.py
Robotics Analyzer — Joint control panel & live TCP pose readout.
R2.8: Live slider and spinbox control per non-fixed joint,
FK TCP readout (mm and deg), gimbal-lock safe RPY extraction,
and Zero all / Reset actions.
"""

from __future__ import annotations

import math
from typing import Any
import numpy as np

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel,
    QSlider, QDoubleSpinBox, QPushButton, QGroupBox, QScrollArea,
    QApplication
)
from PySide6.QtCore import Qt, Signal

import core_bridge as cb
from app.gripper import gripper_dofs
from app.kin_utils import current_q, write_q
from app.panels.model_panel import joint_links, resolved_tcp_link

# Multiplier for mapping continuous angles/displacements to integer QSlider
SLIDER_SCALE = 100.0


def rotation_to_rpy(R: np.ndarray) -> tuple[float, float, float]:
    """
    Extract roll, pitch, yaw from a 3x3 rotation matrix R = Rz(yaw)·Ry(pitch)·Rx(roll).
    Consistent with pose_to_iso order: R = Rz(yaw)·Ry(pitch)·Rx(roll).
    Handles gimbal lock at pitch = ±90°.
    Returns (roll, pitch, yaw) in radians.
    """
    R = np.asarray(R, dtype=float)[:3, :3]
    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6

    if not singular:
        roll = math.atan2(R[2, 1], R[2, 2])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        roll = 0.0
        pitch = math.atan2(-R[2, 0], sy)
        yaw = math.atan2(-R[0, 1], R[1, 1])

    return roll, pitch, yaw


class JointRow:
    """Manages widgets for one non-fixed joint."""

    def __init__(
        self,
        joint_index: int,
        joint_name: str,
        joint_type: cb.JointType,
        lower: float,
        upper: float,
        on_change_callback: Any,
    ):
        self.joint_index = joint_index
        self.joint_type = joint_type
        self.on_change = on_change_callback

        # Conversion factors
        # Revolute: rad <-> deg; Prismatic: m <-> mm
        if joint_type == cb.JointType.Revolute:
            self.to_display = 180.0 / math.pi
            self.to_model = math.pi / 180.0
            self.unit = "deg"
            self.decimals = 2
            self.step = 1.0
        else:  # Prismatic
            self.to_display = 1000.0
            self.to_model = 0.001
            self.unit = "mm"
            self.decimals = 2
            self.step = 5.0

        self.lower_model = lower
        self.upper_model = upper
        self.lower_disp = lower * self.to_display
        self.upper_disp = upper * self.to_display

        # Widgets
        self.name_label = QLabel(joint_name)
        self.name_label.setMinimumWidth(50)
        self.name_label.setMaximumWidth(110)
        self.name_label.setToolTip(joint_name)
        self.name_label.setStyleSheet("font-weight: 500;")

        self.slider = QSlider(Qt.Orientation.Horizontal)
        min_slider = int(math.floor(self.lower_disp * SLIDER_SCALE))
        max_slider = int(math.ceil(self.upper_disp * SLIDER_SCALE))
        self.slider.setRange(min_slider, max_slider)
        self.slider.setTracking(True)
        self.slider.setMinimumWidth(60)

        self.spinbox = QDoubleSpinBox()
        self.spinbox.setRange(self.lower_disp, self.upper_disp)
        self.spinbox.setDecimals(self.decimals)
        self.spinbox.setSingleStep(self.step)
        self.spinbox.setMinimumWidth(70)
        self.spinbox.setMaximumWidth(90)

        self.unit_label = QLabel(self.unit)
        self.unit_label.setMinimumWidth(28)
        self.unit_label.setStyleSheet("color: #888888;")

        self.warning_label = QLabel("⚠️")
        self.warning_label.setToolTip("Joint value is out of defined limits")
        self.warning_label.setStyleSheet("color: #ffaa00; font-weight: bold;")
        self.warning_label.setVisible(False)

        # Connections
        self.slider.valueChanged.connect(self._on_slider_changed)
        self.spinbox.valueChanged.connect(self._on_spinbox_changed)

    def set_value(self, model_q: float, updating_guard: bool = False) -> None:
        """Update slider and spinbox from model q, checking limits."""
        disp_val = model_q * self.to_display
        out_of_limits = (model_q < self.lower_model - 1e-9) or (model_q > self.upper_model + 1e-9)
        self.warning_label.setVisible(out_of_limits)

        # Clamped value for slider and spinbox
        clamped_disp = min(max(disp_val, self.lower_disp), self.upper_disp)

        self.spinbox.blockSignals(True)
        self.slider.blockSignals(True)
        try:
            self.spinbox.setValue(clamped_disp)
            slider_int = int(round(clamped_disp * SLIDER_SCALE))
            self.slider.setValue(slider_int)
        finally:
            self.spinbox.blockSignals(False)
            self.slider.blockSignals(False)

    def _on_slider_changed(self, value: int) -> None:
        disp_val = value / SLIDER_SCALE
        self.spinbox.blockSignals(True)
        try:
            self.spinbox.setValue(disp_val)
        finally:
            self.spinbox.blockSignals(False)

        model_q = disp_val * self.to_model
        self.on_change(self.joint_index, model_q)

    def _on_spinbox_changed(self, disp_val: float) -> None:
        self.slider.blockSignals(True)
        try:
            slider_int = int(round(disp_val * SLIDER_SCALE))
            self.slider.setValue(slider_int)
        finally:
            self.slider.blockSignals(False)

        model_q = disp_val * self.to_model
        self.on_change(self.joint_index, model_q)


class JointPanel(QWidget):
    """
    Panel for joint control and live TCP pose display.

    Usage:
        panel = JointPanel()
        panel.set_model(robot_model)
        panel.q_changed.connect(on_q_changed)
        panel.update_tcp_readout(tcp_matrix)
    """

    q_changed = Signal()
    gripper_moved = Signal(float, float, float)   # U3.7: closed fraction before, requested, reached
    blocked = Signal(str)       # U3.4: a move was stopped by the motion guard (status text)
    log_requested = Signal(str)

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self._model: Any = None
        self._updating: bool = False
        self._rows: list[JointRow] = []          # active-chain rows (top section)
        self._branch_rows: list[JointRow] = []   # "Other branches" rows
        self._baseline_q: list[float] = []

        self._current_joint_signature: list[Any] = []

        self._setup_ui()

    def _setup_ui(self) -> None:
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(6, 6, 6, 6)
        main_layout.setSpacing(6)

        # Joint Sliders Group
        self._joints_group = QGroupBox("Joint Controls")
        joints_vbox = QVBoxLayout(self._joints_group)
        joints_vbox.setContentsMargins(4, 4, 4, 4)

        # No inner scroll area: the dock's outer scroll area handles overflow.
        self._rows_container = QWidget()
        self._rows_layout = QGridLayout(self._rows_container)
        self._rows_layout.setContentsMargins(2, 2, 2, 2)
        self._rows_layout.setSpacing(4)
        self._rows_stretch_row = -1
        joints_vbox.addWidget(self._rows_container)

        # Other branches: collapsible group (hidden when empty)
        self._branch_group = QGroupBox("Other branches")
        self._branch_group.setCheckable(True)
        self._branch_group.setChecked(False)  # collapsed
        branch_vbox = QVBoxLayout(self._branch_group)
        branch_vbox.setContentsMargins(4, 4, 4, 4)
        self._branch_container = QWidget()
        self._branch_layout = QGridLayout(self._branch_container)
        self._branch_layout.setContentsMargins(2, 2, 2, 2)
        self._branch_layout.setSpacing(4)
        branch_vbox.addWidget(self._branch_container)
        self._branch_container.setVisible(False)
        self._branch_group.toggled.connect(self._branch_container.setVisible)
        self._branch_group.setVisible(False)
        joints_vbox.addWidget(self._branch_group)

        # Buttons row
        btn_layout = QHBoxLayout()
        self._btn_zero = QPushButton("Zero all")
        self._btn_zero.clicked.connect(self._on_zero_all)
        self._btn_reset = QPushButton("Reset")
        self._btn_reset.clicked.connect(self._on_reset)
        btn_layout.addWidget(self._btn_zero)
        btn_layout.addWidget(self._btn_reset)
        joints_vbox.addLayout(btn_layout)

        main_layout.addWidget(self._joints_group, stretch=0)

        # Gripper group (U2.4): only when the robot has gripper DOFs.
        self._gripper_group = QGroupBox("Gripper")
        g_lay = QHBoxLayout(self._gripper_group)
        self._btn_g_open = QPushButton("Open")
        self._btn_g_close = QPushButton("Close")
        self._g_slider = QSlider(Qt.Orientation.Horizontal)
        self._g_slider.setRange(0, 100)
        self._g_label = QLabel("Open 0 %")
        g_lay.addWidget(self._btn_g_open)
        g_lay.addWidget(self._btn_g_close)
        g_lay.addWidget(self._g_slider, stretch=1)
        g_lay.addWidget(self._g_label)
        self._btn_g_open.clicked.connect(lambda: self.set_gripper(0.0))
        self._btn_g_close.clicked.connect(lambda: self.set_gripper(1.0))
        self._g_slider.valueChanged.connect(lambda v: self.set_gripper(v / 100.0))
        self._gripper_group.setVisible(False)
        self._gripper: list = []
        main_layout.addWidget(self._gripper_group, stretch=0)

        # TCP Pose Readout Group
        tcp_group = QGroupBox("TCP Pose (World Frame)")
        tcp_layout = QVBoxLayout(tcp_group)
        tcp_layout.setContentsMargins(6, 6, 6, 6)
        tcp_layout.setSpacing(4)

        self._lbl_pos = QLabel("Position: X: 0.00 mm, Y: 0.00 mm, Z: 0.00 mm")
        self._lbl_pos.setStyleSheet("font-family: Consolas, monospace; font-size: 11px;")
        self._lbl_rot = QLabel("Rotation: Roll: 0.00°, Pitch: 0.00°, Yaw: 0.00°")
        self._lbl_rot.setStyleSheet("font-family: Consolas, monospace; font-size: 11px;")
        self._lbl_pos.setWordWrap(True)
        self._lbl_rot.setWordWrap(True)
        tcp_layout.addWidget(self._lbl_pos)
        tcp_layout.addWidget(self._lbl_rot)

        self._btn_copy = QPushButton("Copy pose")
        self._btn_copy.setToolTip("Copy TCP pose text to clipboard")
        self._btn_copy.clicked.connect(self._on_copy_pose)
        tcp_layout.addWidget(self._btn_copy)

        main_layout.addWidget(tcp_group, stretch=0)
        main_layout.addStretch(1)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    @property
    def rows(self) -> list[JointRow]:
        """Return JointRow objects of the active chain (top section)."""
        return self._rows

    @property
    def branch_rows(self) -> list[JointRow]:
        """Return JointRow objects of the "Other branches" group."""
        return self._branch_rows

    @property
    def branch_group(self) -> QGroupBox:
        return self._branch_group

    def _all_rows(self) -> list[JointRow]:
        return self._rows + self._branch_rows

    @property
    def tcp_text(self) -> str:
        """Return combined TCP position and orientation text."""
        return f"{self._lbl_pos.text()}\n{self._lbl_rot.text()}"

    def set_model(self, model: Any, save_baseline: bool = True) -> None:
        """Populate or update panel from a RobotModel."""
        self._model = model
        if self._model is None:
            self._clear_rows()
            self._baseline_q.clear()
            self._update_gripper()
            return

        if save_baseline or not self._baseline_q:
            self._baseline_q = [j.q for j in model.joints]

        chain, others = self._split_joints(model)
        new_sig = (
            tuple(
                (j.name, j.type, j.limits.lower, j.limits.upper,
                 j.mimic is not None, j.parent_link, j.child_link)
                for j in model.joints
            ),
            resolved_tcp_link(model),
            tuple(chain),
            tuple(others),
        )

        if new_sig != self._current_joint_signature:
            self._rebuild_rows(new_sig, chain, others)
        else:
            self._sync_values_to_ui()
        self._update_gripper()

    @property
    def gripper_group(self) -> QGroupBox:
        return self._gripper_group

    def set_guard(self, fn: Any) -> None:
        """Collision guard ``fn(q_new, q_old=None) -> (q_final, blocked)`` (None = off)."""
        self._guard = fn

    def _guarded(self, new_q: list[float]) -> tuple[list[float], bool]:
        """Filter a wanted full q through the guard. ``(q, blocked)``."""
        guard = getattr(self, "_guard", None)
        if guard is None or self._model is None:
            return new_q, False
        return guard(new_q)

    def set_gripper(self, closed_fraction: float) -> None:
        """Move every gripper DOF to ``closed_fraction`` (0 = open, 1 = closed).

        With collision blocking the fingers stop at the first contact; the slider shows the
        fraction actually reached.
        """
        if self._model is None or not self._gripper:
            return
        f = min(max(float(closed_fraction), 0.0), 1.0)
        want = current_q(self._model)
        f_before = self._gripper[0].fraction_of(want[self._gripper[0].dof_index])
        f_requested = f
        for g in self._gripper:
            want[g.dof_index] = g.value_at(f)
        final, blocked = self._guarded(want)
        if blocked:
            f = self._gripper[0].fraction_of(final[self._gripper[0].dof_index])
            want = final
        self._updating = True
        try:
            for g in self._gripper:
                self._model.joints[g.joint_index].q = want[g.dof_index]
            self._sync_gripper_slider(f)
            self._sync_values_to_ui_force()
        finally:
            self._updating = False
        self.q_changed.emit()
        self.gripper_moved.emit(float(f_before), float(f_requested), float(f))

    def _sync_values_to_ui_force(self) -> None:
        for row in self._all_rows():
            row.set_value(self._model.joints[row.joint_index].q)

    def _sync_gripper_slider(self, f: float) -> None:
        self._g_slider.blockSignals(True)
        try:
            self._g_slider.setValue(int(round(f * 100)))
        finally:
            self._g_slider.blockSignals(False)
        self._g_label.setText(f"Closed {int(round(f * 100))} %")

    def _update_gripper(self) -> None:
        try:
            self._gripper = gripper_dofs(self._model) if self._model is not None else []
        except Exception:
            self._gripper = []
        self._gripper_group.setVisible(bool(self._gripper))
        if self._gripper:
            g = self._gripper[0]
            self._sync_gripper_slider(g.fraction_of(self._model.joints[g.joint_index].q))

    @staticmethod
    def _split_joints(model: Any) -> tuple[list[int], list[int]]:
        """Return (active-chain row joints, other-branch row joints): non-fixed, non-mimic."""
        try:
            active = list(cb.active_chain(model))
        except Exception:
            active = list(range(len(model.joints)))
        aset = set(active)

        def eligible(k: int) -> bool:
            j = model.joints[k]
            return j.type != cb.JointType.Fixed and j.mimic is None

        chain = [k for k in active if eligible(k)]
        others = [k for k in range(len(model.joints)) if k not in aset and eligible(k)]
        return chain, others

    def update_tcp_readout(self, tcp: np.ndarray) -> None:
        """Update live TCP pose readout labels from a 4x4 transform."""
        tcp = np.asarray(tcp, dtype=float)
        x_mm = tcp[0, 3] * 1000.0
        y_mm = tcp[1, 3] * 1000.0
        z_mm = tcp[2, 3] * 1000.0

        r_rad, p_rad, yaw_rad = rotation_to_rpy(tcp[:3, :3])
        r_deg = math.degrees(r_rad)
        p_deg = math.degrees(p_rad)
        yaw_deg = math.degrees(yaw_rad)

        self._lbl_pos.setText(f"Position: X: {x_mm:.2f} mm, Y: {y_mm:.2f} mm, Z: {z_mm:.2f} mm")
        self._lbl_rot.setText(f"Rotation: Roll: {r_deg:.2f}°, Pitch: {p_deg:.2f}°, Yaw: {yaw_deg:.2f}°")

    def save_baseline(self) -> None:
        """Record current model joint values as the reset baseline."""
        if self._model is not None:
            self._baseline_q = [j.q for j in self._model.joints]

    # ------------------------------------------------------------------
    # Internal Helpers
    # ------------------------------------------------------------------

    def _clear_rows(self) -> None:
        """Remove all row widgets from the layouts."""
        for layout in (self._rows_layout, self._branch_layout):
            while layout.count():
                item = layout.takeAt(0)
                widget = item.widget()
                if widget is not None:
                    widget.deleteLater()
        self._rows.clear()
        self._branch_rows.clear()
        if self._rows_stretch_row >= 0:
            self._rows_layout.setRowStretch(self._rows_stretch_row, 0)
            self._rows_stretch_row = -1
        self._branch_group.setVisible(False)
        self._current_joint_signature = []

    def _make_row(self, i: int, layout: QGridLayout, row_idx: int) -> JointRow:
        j = self._model.joints[i]
        row = JointRow(
            joint_index=i,
            joint_name=j.name,
            joint_type=j.type,
            lower=j.limits.lower,
            upper=j.limits.upper,
            on_change_callback=self._on_joint_value_edited,
        )
        row.set_value(j.q)
        layout.addWidget(row.name_label, row_idx, 0)
        layout.addWidget(row.slider, row_idx, 1)
        layout.addWidget(row.spinbox, row_idx, 2)
        layout.addWidget(row.unit_label, row_idx, 3)
        layout.addWidget(row.warning_label, row_idx, 4)
        return row

    def _rebuild_rows(self, signature: Any, chain: list[int], others: list[int]) -> None:
        """Reconstruct row layouts: active chain on top, other branches below."""
        self._updating = True
        try:
            self._clear_rows()
            if self._model is None:
                return
            for r, i in enumerate(chain):
                self._rows.append(self._make_row(i, self._rows_layout, r))
            for r, i in enumerate(others):
                self._branch_rows.append(self._make_row(i, self._branch_layout, r))
            # Rows pack at the top: an empty stretch row absorbs spare height.
            self._rows_stretch_row = len(chain)
            self._rows_layout.setRowStretch(self._rows_stretch_row, 1)
            self._branch_group.setVisible(bool(others))
            self._current_joint_signature = signature
        finally:
            self._updating = False

    def _sync_values_to_ui(self) -> None:
        """Update existing widget values without rebuilding layout."""
        if self._model is None or self._updating:
            return
        self._updating = True
        try:
            for row in self._all_rows():
                q = self._model.joints[row.joint_index].q
                row.set_value(q)
        finally:
            self._updating = False

    def _on_joint_value_edited(self, joint_index: int, new_q: float) -> None:
        """Handle value change from slider or spinbox."""
        if self._updating or self._model is None:
            return
        if 0 <= joint_index < len(self._model.joints):
            dof = self._model.q_index_map()[joint_index]
            if dof >= 0 and getattr(self, "_guard", None) is not None:
                want = current_q(self._model)
                want[dof] = new_q
                final, blocked = self._guarded(want)
                if blocked:                       # U3.4: slider goes back to the last free q
                    write_q(self._model, final)
                    self._sync_values_to_ui()
                    self.blocked.emit(self._guard_text())
                    self.q_changed.emit()
                    return
            self._model.joints[joint_index].q = new_q
            # Re-check warning label for this row
            for row in self._all_rows():
                if row.joint_index == joint_index:
                    out = (new_q < row.lower_model - 1e-9) or (new_q > row.upper_model + 1e-9)
                    row.warning_label.setVisible(out)
                    break
            self.q_changed.emit()

    def _guard_text(self) -> str:
        g = getattr(self, "_guard", None)
        owner = getattr(g, "__self__", None)
        return getattr(owner, "text", "") or "Blocked by collision"

    def _on_zero_all(self) -> None:
        """Set all non-fixed joint values to 0.0 (clamped to limits)."""
        if self._model is None:
            return
        self._updating = True
        try:
            for row in self._all_rows():
                zero_clamped = min(max(0.0, row.lower_model), row.upper_model)
                self._model.joints[row.joint_index].q = zero_clamped
                row.set_value(zero_clamped)
        finally:
            self._updating = False

        self.q_changed.emit()
        self.log_requested.emit("[INFO] Joint controls: Zeroed all joint values.")

    def _on_reset(self) -> None:
        """Reset joint values to saved baseline."""
        if self._model is None or not self._baseline_q:
            return
        self._updating = True
        try:
            for row in self._all_rows():
                if row.joint_index < len(self._baseline_q):
                    orig_q = self._baseline_q[row.joint_index]
                    self._model.joints[row.joint_index].q = orig_q
                    row.set_value(orig_q)
        finally:
            self._updating = False

        self.q_changed.emit()
        self.log_requested.emit("[INFO] Joint controls: Reset joint values to baseline.")

    def _on_copy_pose(self) -> None:
        """Copy current TCP pose readout text to clipboard."""
        clipboard = QApplication.clipboard()
        if clipboard is not None:
            clipboard.setText(self.tcp_text)

