"""
app/panels/ik_panel.py
R4.9: "Inverse Kinematics" dock. Target pose (x y z in m, roll pitch yaw in deg),
multi-start damped least squares IK in a TaskRunner worker, solution table with
preview (not committed), Apply and Cancel preview.
"""

from __future__ import annotations

import math
from typing import Any

import numpy as np
from PySide6.QtCore import Signal
from PySide6.QtWidgets import (
    QAbstractItemView, QCheckBox, QGridLayout, QHBoxLayout, QLabel, QPushButton,
    QSpinBox, QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget,
)

import core_bridge as cb
from app.kin_utils import CompiledCache, current_q, dof_limits, write_q
from app.panels.joint_panel import rotation_to_rpy
from app.panels.num_field import NumField

HEADERS = ["#", "Distance", "Pos error [m]", "Rot error [rad]"]
NO_SOLUTION_TEXT = "No solution found"


def target_iso(xyz, rpy_deg) -> np.ndarray:
    """4x4 world pose from x y z [m] and roll pitch yaw [deg] (Rz Ry Rx)."""
    pose = cb.Pose()
    pose.xyz = [float(v) for v in xyz]
    pose.rpy = [math.radians(float(v)) for v in rpy_deg]
    return np.asarray(cb.pose_to_iso(pose), dtype=float)


class IKPanel(QWidget):
    """Dock content for inverse kinematics."""

    target_changed = Signal(object)    # 4x4 iso or None (target triad)
    preview_changed = Signal()         # model q written for a preview / cancel
    applied = Signal()                 # preview committed (project is dirty)
    log_requested = Signal(str)

    def __init__(self, task_runner: Any, parent=None):
        super().__init__(parent)
        self._runner = task_runner
        self._model: Any = None
        self._cache = CompiledCache()
        self._solutions: list[dict] = []
        self._baseline_q: list[float] | None = None   # q before the first preview
        self._previewing = False
        self._running = False
        self._generation = 0       # bumped on model change: stale results dropped
        self._task: Any = None

        layout = QVBoxLayout(self)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(4)

        grid = QGridLayout()
        self._fields: list[NumField] = []
        labels = ["x [m]", "y [m]", "z [m]", "roll [deg]", "pitch [deg]", "yaw [deg]"]
        for k, text in enumerate(labels):
            f = NumField(0.0)
            f.value_changed.connect(self._on_target_edited)
            self._fields.append(f)
            grid.addWidget(QLabel(text), k // 3 * 2, k % 3)
            grid.addWidget(f, k // 3 * 2 + 1, k % 3)
        layout.addLayout(grid)

        row = QHBoxLayout()
        self._btn_tcp = QPushButton("Use current TCP")
        self._btn_tcp.clicked.connect(self.use_current_tcp)
        row.addWidget(self._btn_tcp)
        self._chk_pos_only = QCheckBox("Position only")
        row.addWidget(self._chk_pos_only)
        row.addStretch(1)
        layout.addLayout(row)

        opt = QHBoxLayout()
        opt.addWidget(QLabel("Damping"))
        self._damping = NumField(0.05)
        self._damping.setMaximumWidth(80)
        opt.addWidget(self._damping)
        opt.addWidget(QLabel("Max iter"))
        self._max_iter = QSpinBox()
        self._max_iter.setRange(1, 10000)
        self._max_iter.setValue(100)
        opt.addWidget(self._max_iter)
        opt.addWidget(QLabel("Starts"))
        self._n_starts = QSpinBox()
        self._n_starts.setRange(1, 1000)
        self._n_starts.setValue(30)
        opt.addWidget(self._n_starts)
        opt.addStretch(1)
        layout.addLayout(opt)

        self._btn_solve = QPushButton("Solve")
        self._btn_solve.clicked.connect(self.solve)
        layout.addWidget(self._btn_solve)

        self._status = QLabel("")
        layout.addWidget(self._status)

        self._table = QTableWidget(0, len(HEADERS))
        self._table.setHorizontalHeaderLabels(HEADERS)
        self._table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self._table.setSelectionMode(QAbstractItemView.SelectionMode.SingleSelection)
        self._table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self._table.itemSelectionChanged.connect(self._on_selection_changed)
        layout.addWidget(self._table, stretch=1)

        btns = QHBoxLayout()
        self._btn_apply = QPushButton("Apply")
        self._btn_apply.clicked.connect(self.apply)
        btns.addWidget(self._btn_apply)
        self._btn_cancel = QPushButton("Cancel preview")
        self._btn_cancel.clicked.connect(self.cancel_preview)
        btns.addWidget(self._btn_cancel)
        layout.addLayout(btns)

        self._update_buttons()

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    @property
    def target_fields(self) -> list[NumField]:
        return self._fields

    @property
    def damping_field(self) -> NumField:
        return self._damping

    @property
    def max_iter_spin(self) -> QSpinBox:
        return self._max_iter

    @property
    def starts_spin(self) -> QSpinBox:
        return self._n_starts

    @property
    def solutions(self) -> list[dict]:
        return self._solutions

    @property
    def table(self) -> QTableWidget:
        return self._table

    @property
    def status_text(self) -> str:
        return self._status.text()

    @property
    def is_running(self) -> bool:
        return self._running

    @property
    def is_previewing(self) -> bool:
        return self._previewing

    @property
    def cache(self) -> CompiledCache:
        return self._cache

    @property
    def solve_button(self) -> QPushButton:
        return self._btn_solve

    @property
    def apply_button(self) -> QPushButton:
        return self._btn_apply

    @property
    def cancel_button(self) -> QPushButton:
        return self._btn_cancel

    @property
    def tcp_button(self) -> QPushButton:
        return self._btn_tcp

    @property
    def position_only_checkbox(self) -> QCheckBox:
        return self._chk_pos_only

    def set_target(self, xyz, rpy_deg) -> None:
        """Fill the six target fields (m, deg)."""
        vals = list(xyz) + list(rpy_deg)
        for f, v in zip(self._fields, vals):
            f.set_value(v)

    def set_model(self, model: Any) -> None:
        """Robot replaced or edited (or None on close). Resets results, rebuilds lazily."""
        self._generation += 1
        self._cache.invalidate()
        self._solutions = []
        self._table.blockSignals(True)
        self._table.setRowCount(0)
        self._table.blockSignals(False)
        self._baseline_q = None
        self._previewing = False
        self._running = False
        self._status.setText("")
        usable = model is not None and len(model.joints) > 0 and model.dof() > 0
        self._model = model if usable else None
        if self._model is None:
            self.target_changed.emit(None)
        self._update_buttons()

    def notify_q_edited(self) -> None:
        """The user moved a joint: a running preview is no longer a preview."""
        self._baseline_q = None
        self._previewing = False
        self._table.blockSignals(True)
        self._table.clearSelection()
        self._table.blockSignals(False)
        self._update_buttons()

    def use_current_tcp(self) -> None:
        if self._model is None:
            return
        tcp = np.asarray(cb.fk(self._model)["tcp"], dtype=float)
        r, p, y = rotation_to_rpy(tcp[:3, :3])
        self.set_target(tcp[:3, 3], [math.degrees(r), math.degrees(p), math.degrees(y)])
        self._emit_target()

    def target_pose(self) -> np.ndarray | None:
        """4x4 target from the fields (last valid values), None when no model."""
        if self._model is None:
            return None
        v = [f.value() for f in self._fields]
        return target_iso(v[:3], v[3:])

    def solve(self) -> None:
        """Start the multi-start IK in a worker task. Returns at once."""
        if self._model is None or self._running:
            return
        model = self._model
        try:
            cm = self._cache.get_cm(model)
            lo, hi = dof_limits(model)
        except Exception as exc:
            self._status.setText(f"IK setup failed: {exc}")
            return
        self.cancel_preview()
        q_cur = current_q(model)
        target = self.target_pose()
        options = {
            "position_only": self._chk_pos_only.isChecked(),
            "damping": self._damping.value(),
            "max_iter": int(self._max_iter.value()),
        }
        n_starts = int(self._n_starts.value())

        self._running = True
        self._status.setText("Solving...")
        self._update_buttons()
        gen = self._generation

        def work():
            return cb.solve_ik_multi(cm, lo, hi, target, q_cur, options, n_starts)

        from app.task_runner import FunctionTask
        task = FunctionTask(work)       # connect before submit: a fast task must not lose its result
        task.signals.result.connect(lambda res, g=gen: self._on_result(res, g))
        task.signals.error.connect(lambda msg, g=gen: self._on_error(msg, g))
        self._task = task
        self._runner.submit(task)

    def apply(self) -> None:
        """Commit the previewed solution to the model q (project becomes dirty)."""
        row = self._selected_row()
        if self._model is None or row is None:
            return
        write_q(self._model, self._solutions[row]["q"])
        self._baseline_q = None
        self._previewing = False
        self.applied.emit()
        self.log_requested.emit(f"[INFO] IK: applied solution {row}.")
        self._update_buttons()

    def cancel_preview(self) -> None:
        """Restore the q from before the preview."""
        if self._baseline_q is not None and self._model is not None:
            write_q(self._model, self._baseline_q)
            self.preview_changed.emit()
        self._baseline_q = None
        self._previewing = False
        self._table.blockSignals(True)
        self._table.clearSelection()
        self._table.blockSignals(False)
        self._update_buttons()

    # ------------------------------------------------------------------
    # Internals
    # ------------------------------------------------------------------

    def _emit_target(self) -> None:
        self.target_changed.emit(self.target_pose())

    def _on_target_edited(self, _v: float) -> None:
        self._emit_target()

    def _selected_row(self) -> int | None:
        sel = self._table.selectionModel()
        rows = sel.selectedRows() if sel is not None else []
        if not rows:
            return None
        r = rows[0].row()
        return r if 0 <= r < len(self._solutions) else None

    def _on_selection_changed(self) -> None:
        row = self._selected_row()
        if row is None or self._model is None:
            self._update_buttons()
            return
        if self._baseline_q is None:
            self._baseline_q = current_q(self._model)
        write_q(self._model, self._solutions[row]["q"])
        self._previewing = True
        self.preview_changed.emit()
        self._update_buttons()

    def _on_result(self, sols: Any, gen: int) -> None:
        if gen != self._generation:
            return
        self._running = False
        self._solutions = list(sols)
        self._table.blockSignals(True)
        self._table.setRowCount(len(self._solutions))
        for r, s in enumerate(self._solutions):
            cells = [str(r), f"{s['distance']:.4g}", f"{s['pos_error']:.3e}",
                     f"{s['rot_error']:.3e}"]
            for c, text in enumerate(cells):
                self._table.setItem(r, c, QTableWidgetItem(text))
        self._table.blockSignals(False)
        if self._solutions:
            self._status.setText(f"{len(self._solutions)} solution(s)")
        else:
            self._status.setText(NO_SOLUTION_TEXT)
        self._update_buttons()

    def _on_error(self, msg: str, gen: int) -> None:
        if gen != self._generation:
            return
        self._running = False
        lines = msg.strip().splitlines()
        self._status.setText(f"IK failed: {lines[-1] if lines else 'error'}")
        self._update_buttons()

    def _update_buttons(self) -> None:
        has = self._model is not None
        self._btn_solve.setEnabled(has and not self._running)
        self._btn_tcp.setEnabled(has)
        self._btn_apply.setEnabled(has and self._previewing and self._selected_row() is not None)
        self._btn_cancel.setEnabled(self._previewing)
