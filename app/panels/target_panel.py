"""Target dock (U4a.4): set a target in 3D, rank the IK solutions, preview, play to the target.

Top to bottom: saved targets (add from TCP / by click on a surface / delete / rename), typed pose
(X Y Z in mm, roll pitch yaw in deg) with rotate steps, Solve + ranked table, ghost preview
options, and the motion block (approach / retreat, "Play to target").

The panel owns the 3D target frame (``TargetController``) and the ghost preview
(``TargetPreview``) once ``attach_viewport`` is called. Plain methods for tests: ``solve``,
``select_row``, ``play_to_target`` (all return at once, results arrive through the task runner).
"""
from __future__ import annotations

import threading
from typing import Any, Callable

import numpy as np
from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QAbstractItemView, QCheckBox, QComboBox, QDialog, QDialogButtonBox, QDoubleSpinBox,
    QFormLayout, QGroupBox, QHBoxLayout, QHeaderView, QLabel, QLineEdit, QProgressBar, QPushButton,
    QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget,
)

import core_bridge as cb
from app.analysis.ik_rank import DEFAULT_WEIGHTS, PART_NAMES, rank_solutions, rerank
from app.analysis.target_motion import plan_target_motion
from app.kin_utils import CompiledCache, current_q
from app.target import TargetStore, pose_from_xyz_rpy, rotate_pose, xyz_rpy_from_pose
from app.target_preview import TargetPreview, target_status
from app.task_runner import FunctionTask

ROTATE_STEP_DEG = 5.0
COLUMNS = ("#", "Score", "Motion", "Limits", "Manip.", "Clear.", "State")
_STATUS_TEXT = {"neutral": "Not solved yet.", "ok": "Reachable.", "bad": "No valid solution.",
                "warn": "Reachable, near a singularity."}


class WeightsDialog(QDialog):
    """Weights of the four score parts (motion, limits, manipulability, clearance)."""

    def __init__(self, weights, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Ranking weights")
        form = QFormLayout(self)
        self.boxes: list[QDoubleSpinBox] = []
        for name, w in zip(("Motion (short joint travel)", "Limit margin", "Manipulability",
                            "Clearance"), weights):
            b = QDoubleSpinBox()
            b.setRange(0.0, 10.0)
            b.setSingleStep(0.05)
            b.setDecimals(2)
            b.setValue(float(w))
            form.addRow(name, b)
            self.boxes.append(b)
        note = QLabel("Weights are relative; they are normalised to a sum of 1.")
        note.setWordWrap(True)
        form.addRow(note)
        bb = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok |
                              QDialogButtonBox.StandardButton.Cancel |
                              QDialogButtonBox.StandardButton.RestoreDefaults)
        bb.accepted.connect(self.accept)
        bb.rejected.connect(self.reject)
        bb.button(QDialogButtonBox.StandardButton.RestoreDefaults).clicked.connect(self._defaults)
        form.addRow(bb)

    def _defaults(self) -> None:
        for b, w in zip(self.boxes, DEFAULT_WEIGHTS):
            b.setValue(w)

    def weights(self) -> tuple:
        return tuple(b.value() for b in self.boxes)


def _spin(lo, hi, dec, suffix, step=1.0) -> QDoubleSpinBox:
    b = QDoubleSpinBox()
    b.setRange(lo, hi)
    b.setDecimals(dec)
    b.setSuffix(suffix)
    b.setSingleStep(step)
    b.setKeyboardTracking(False)
    b.setMinimumWidth(66)
    b.setButtonSymbols(QDoubleSpinBox.ButtonSymbols.NoButtons)
    return b


class TargetPanel(QWidget):
    log_requested = Signal(str)
    trajectory_ready = Signal(object, dict)      # Trajectory, info (segments, q_goal)
    targets_changed = Signal()                   # store edited (project dirty)
    status_message = Signal(str)
    _solved = Signal(int, object)
    _failed = Signal(int, str)
    _planned = Signal(int, object)
    _failed_m = Signal(int, str)

    def __init__(self, task_runner: Any, parent=None):
        super().__init__(parent)
        self._runner = task_runner
        self._model: Any = None
        self._project: Any = None
        self._store = TargetStore()
        self._cache = CompiledCache()
        self._cache_id: int | None = None
        self._svc: Any = None
        self._settings: Any = None
        self._weights = DEFAULT_WEIGHTS
        self._viewport: Any = None
        self.controller: Any = None
        self.preview: TargetPreview | None = None
        self._solutions: list = []
        self._rank_note = ""
        self._generation = 0
        self._motion_gen = 0
        self._solving = False
        self._planning = False
        self._cancel_evt: threading.Event | None = None
        self._task: Any = None
        self._loading = False
        self._active = False
        self._motion: Any = None
        self.last_rank_ms = 0.0
        self.last_motion: Any = None
        self.auto_solve = True
        self._disposed = False

        lay = QVBoxLayout(self)
        lay.setContentsMargins(6, 6, 6, 6)
        lay.setSpacing(6)

        # ---- 1. targets
        g1 = QGroupBox("Target")
        l1 = QVBoxLayout(g1)
        row = QHBoxLayout()
        self._combo = QComboBox()
        self._combo.setMinimumWidth(120)
        self._combo.setToolTip("Saved targets of this project")
        self._combo.currentIndexChanged.connect(self._on_combo)
        row.addWidget(self._combo, 1)
        self._btn_delete = QPushButton("Delete")
        self._btn_delete.clicked.connect(self.delete_target)
        row.addWidget(self._btn_delete)
        l1.addLayout(row)
        row = QHBoxLayout()
        self._btn_add = QPushButton("New at TCP")
        self._btn_add.setToolTip("New target at the current tool pose")
        self._btn_add.clicked.connect(self.add_from_tcp)
        self._btn_click = QPushButton("Pick on surface")
        self._btn_click.setCheckable(True)
        self._btn_click.setToolTip("Click a surface in the 3D view: the tool points into it")
        self._btn_click.toggled.connect(self._on_pick_toggled)
        row.addWidget(self._btn_add)
        row.addWidget(self._btn_click)
        l1.addLayout(row)
        row = QHBoxLayout()
        row.addWidget(QLabel("Lift off surface"))
        self._standoff = _spin(0.0, 500.0, 0, " mm", 5.0)
        self._standoff.setValue(10.0)
        self._standoff.setToolTip("Pick on surface puts the target this far above the surface, so a "
                                  "gripper does not start in contact")
        self._standoff.valueChanged.connect(self._on_standoff)
        row.addWidget(self._standoff)
        row.addStretch(1)
        l1.addLayout(row)
        row = QHBoxLayout()
        row.addWidget(QLabel("Name"))
        self._name = QLineEdit()
        self._name.editingFinished.connect(self._on_rename)
        row.addWidget(self._name, 1)
        l1.addLayout(row)
        self._hint = QLabel("No target yet. Use New at TCP, or Pick on surface.")
        self._hint.setWordWrap(True)
        l1.addWidget(self._hint)
        lay.addWidget(g1)

        # ---- 2. pose
        g2 = QGroupBox("Pose")
        l2 = QVBoxLayout(g2)
        row = QHBoxLayout()
        self._xyz = [_spin(-9999.0, 9999.0, 1, " mm") for _ in range(3)]
        for ax, b in zip("XYZ", self._xyz):
            row.addWidget(QLabel(ax))
            row.addWidget(b, 1)
            b.valueChanged.connect(self._on_fields)
        l2.addLayout(row)
        row = QHBoxLayout()
        self._rpy = [_spin(-360.0, 360.0, 1, " deg") for _ in range(3)]
        for ax, b in zip(("Roll", "Pitch", "Yaw"), self._rpy):
            row.addWidget(QLabel(ax[0]))
            row.addWidget(b, 1)
            b.valueChanged.connect(self._on_fields)
        l2.addLayout(row)
        self._rot_btns: list[QPushButton] = []
        row = QHBoxLayout()
        row.addWidget(QLabel(f"Rotate {ROTATE_STEP_DEG:g} deg"))
        for ax in "xyz":
            for sign, sym in ((-1, "-"), (1, "+")):
                b = QPushButton(f"{ax.upper()}{sym}")
                b.setMaximumWidth(38)
                b.setToolTip(f"Rotate about the target's own {ax.upper()} axis")
                b.clicked.connect(lambda _c=False, a=ax, s=sign: self.rotate(a, s * ROTATE_STEP_DEG))
                row.addWidget(b)
                self._rot_btns.append(b)
        row.addStretch(1)
        l2.addLayout(row)
        lay.addWidget(g2)

        # ---- 3. solve + ranked table
        g3 = QGroupBox("Inverse kinematics")
        l3 = QVBoxLayout(g3)
        row = QHBoxLayout()
        self._btn_solve = QPushButton("Solve")
        self._btn_solve.clicked.connect(self.solve)
        self._chk_auto = QCheckBox("Solve after each move")
        self._chk_auto.setChecked(True)
        self._chk_auto.toggled.connect(lambda on: setattr(self, "auto_solve", bool(on)))
        self._btn_weights = QPushButton("Weights...")
        self._btn_weights.clicked.connect(self.edit_weights)
        row.addWidget(self._btn_solve)
        row.addWidget(self._chk_auto, 1)
        row.addWidget(self._btn_weights)
        l3.addLayout(row)
        self._status = QLabel(_STATUS_TEXT["neutral"])
        self._status.setWordWrap(True)
        l3.addWidget(self._status)
        self._table = QTableWidget(0, len(COLUMNS))
        self._table.setHorizontalHeaderLabels(COLUMNS)
        self._table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self._table.setSelectionMode(QAbstractItemView.SelectionMode.SingleSelection)
        self._table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self._table.verticalHeader().setVisible(False)
        self._table.setMinimumHeight(150)
        self._table.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        hh = self._table.horizontalHeader()
        hh.setStretchLastSection(True)
        for c, w in ((0, 22), (1, 58), (2, 44), (3, 40), (4, 44), (5, 38)):
            self._table.setColumnWidth(c, w)
        for c, tip in ((2, "Short joint travel from the current pose"), (3, "Distance to the joint limits"),
                       (4, "Manipulability (far from singularities)"), (5, "Collision clearance")):
            self._table.horizontalHeaderItem(c).setToolTip(tip)
        self._table.itemSelectionChanged.connect(self._on_row_selected)
        l3.addWidget(self._table)
        self._chk_all = QCheckBox("Show all solutions as ghosts")
        self._chk_all.toggled.connect(self._on_show_all)
        l3.addWidget(self._chk_all)
        lay.addWidget(g3, 1)

        # ---- 4. motion
        g4 = QGroupBox("Move to the target")
        l4 = QVBoxLayout(g4)
        row = QHBoxLayout()
        self._chk_approach = QCheckBox("Approach from")
        self._chk_approach.setToolTip("Go to a pose in front of the target first, then move straight "
                                      "along the tool z axis")
        self._appr = _spin(1.0, 2000.0, 0, " mm", 10.0)
        self._appr.setValue(100.0)
        self._appr.setEnabled(False)
        self._chk_approach.toggled.connect(self._appr.setEnabled)
        self._chk_retreat = QCheckBox("Retreat after")
        self._chk_retreat.setEnabled(False)
        self._chk_approach.toggled.connect(self._chk_retreat.setEnabled)
        row.addWidget(self._chk_approach)
        row.addWidget(self._appr)
        row.addWidget(self._chk_retreat)
        row.addStretch(1)
        l4.addLayout(row)
        row = QHBoxLayout()
        self._btn_play = QPushButton("Play to target")
        self._btn_play.clicked.connect(self.play_to_target)
        self._btn_cancel = QPushButton("Cancel")
        self._btn_cancel.clicked.connect(self.cancel)
        row.addWidget(self._btn_play, 1)
        row.addWidget(self._btn_cancel)
        l4.addLayout(row)
        self._motion_status = QLabel("")
        self._motion_status.setWordWrap(True)
        l4.addWidget(self._motion_status)
        lay.addWidget(g4)

        self._solved.connect(self._on_solved)
        self._failed.connect(self._on_failed)
        self._planned.connect(self._on_planned)
        self._failed_m.connect(self._failed_motion)
        self._update_buttons()

    # ------------------------------------------------------------------ setup

    def attach_viewport(self, viewport: Any) -> None:
        from app.viewport.target_widget import TargetController
        self._viewport = viewport
        self.controller = TargetController(viewport, get_ref_rot=self._tcp_rot, parent=self)
        self.controller.standoff = self._standoff.value() / 1000.0
        self.controller.pose_changed.connect(self._on_controller_pose)
        self.controller.committed.connect(self._on_controller_committed)
        self.preview = TargetPreview(lambda: getattr(viewport, "robot_view", None),
                                     lambda: self._model)

    def set_collision_service(self, svc: Any) -> None:
        self._svc = svc

    def set_settings(self, settings: Any) -> None:
        self._settings = settings
        self._weights = settings.ik_rank_weights() if settings is not None else DEFAULT_WEIGHTS

    @property
    def weights(self) -> tuple:
        return tuple(self._weights)

    def set_weights(self, weights) -> None:
        self._weights = tuple(float(w) for w in weights)
        if self._settings is not None:
            self._settings.set_ik_rank_weights(self._weights)
        if self._solutions:
            sel = self.selected_row()
            self._solutions = rerank(self._solutions, self._weights)
            self._fill_table()
            self.select_row(0 if sel >= 0 else -1)

    def edit_weights(self) -> None:
        dlg = WeightsDialog(self._weights, self)
        if dlg.exec() == QDialog.DialogCode.Accepted:
            try:
                from app.analysis.ik_rank import normalise_weights
                normalise_weights(dlg.weights())
            except ValueError:
                self.status_message.emit("Weights: at least one must be above 0.")
                return
            self.set_weights(dlg.weights())

    def set_project(self, project: Any) -> None:
        if project is self._project and (project is not None or len(self._store) == 0):
            return
        self._cancel_work()
        self._project = project
        self._store = project.targets if project is not None and getattr(project, "targets", None) \
            is not None else TargetStore()
        if project is not None and getattr(project, "targets", None) is None:
            project.targets = self._store
        self._reload_combo()

    def set_model(self, model: Any) -> None:
        """Robot replaced or edited (None on close): drop results and the preview."""
        self._cancel_work()
        self._model = model
        self._cache.invalidate()
        self._clear_results()
        self._update_buttons()

    def set_active(self, on: bool) -> None:
        """Dock visible: show the 3D target frame; hidden: remove it (and the ghosts)."""
        self._active = bool(on)
        if self.controller is None or self._disposed:
            return
        if on and self._model is not None and self._current_index() >= 0:
            self.controller.show(True)
        else:
            self.controller.show(False)
            if self._btn_click.isChecked():
                self._btn_click.setChecked(False)
        if not on and self.preview is not None:
            self.preview.clear()
        elif on and self._solutions:
            self._show_preview()

    def showEvent(self, e) -> None:
        super().showEvent(e)
        self.set_active(True)

    def hideEvent(self, e) -> None:
        super().hideEvent(e)
        try:
            self.set_active(False)
        except RuntimeError:            # widgets already deleted at interpreter exit
            pass

    def dispose(self) -> None:
        self._disposed = True
        self._cancel_work()
        if self.preview is not None:
            self.preview.clear()
        if self.controller is not None:
            self.controller.dispose()

    shutdown = dispose

    # ------------------------------------------------------------------ state access

    @property
    def store(self) -> TargetStore:
        return self._store

    @property
    def solutions(self) -> list:
        return list(self._solutions)

    @property
    def is_solving(self) -> bool:
        return self._solving

    @property
    def is_planning(self) -> bool:
        return self._planning

    @property
    def status_text(self) -> str:
        return self._status.text()

    @property
    def motion_status_text(self) -> str:
        return self._motion_status.text()

    @property
    def table(self) -> QTableWidget:
        return self._table

    @property
    def approach_check(self) -> QCheckBox:
        return self._chk_approach

    @property
    def retreat_check(self) -> QCheckBox:
        return self._chk_retreat

    @property
    def approach_mm(self) -> float:
        return float(self._appr.value())

    def set_approach(self, on: bool, mm: float | None = None, retreat: bool = False) -> None:
        self._chk_approach.setChecked(bool(on))
        if mm is not None:
            self._appr.setValue(float(mm))
        self._chk_retreat.setChecked(bool(retreat))

    def button_enabled(self) -> dict:
        return {"solve": self._btn_solve.isEnabled(), "play": self._btn_play.isEnabled(),
                "cancel": self._btn_cancel.isEnabled(), "delete": self._btn_delete.isEnabled(),
                "new": self._btn_add.isEnabled()}

    def selected_row(self) -> int:
        rows = self._table.selectionModel().selectedRows() if self._table.selectionModel() else []
        return rows[0].row() if rows else -1

    def current_pose(self) -> np.ndarray | None:
        i = self._current_index()
        return None if i < 0 else self._store[i].pose.copy()

    def _current_index(self) -> int:
        i = self._combo.currentIndex()
        return i if 0 <= i < len(self._store) else -1

    def _tcp(self) -> np.ndarray | None:
        if self._model is None:
            return None
        try:
            return np.asarray(cb.fk(self._model)["tcp"], float)
        except Exception:
            return None

    def _tcp_rot(self):
        t = self._tcp()
        return None if t is None else t[:3, :3]

    # ------------------------------------------------------------------ targets

    def _reload_combo(self) -> None:
        self._loading = True
        self._combo.clear()
        self._combo.addItems(self._store.names())
        self._loading = False
        self._clear_results()
        if len(self._store):
            self._combo.setCurrentIndex(0)
        self._after_target_change(solve=False)

    def add_from_tcp(self, pose: Any = None, name: str | None = None) -> bool:
        """New target at the current TCP pose (or ``pose``); selects it."""
        T = self._tcp() if pose is None else np.asarray(pose, float)
        if T is None:
            return False
        t = self._store.add(T, name)
        self._loading = True
        self._combo.addItem(t.name)
        self._combo.setCurrentIndex(len(self._store) - 1)
        self._loading = False
        self.targets_changed.emit()
        self._after_target_change(solve=False)
        return True

    def delete_target(self) -> None:
        i = self._current_index()
        if i < 0:
            return
        self._store.remove(i)
        self._reload_combo()
        self.targets_changed.emit()

    def select_target(self, i: int) -> None:
        self._combo.setCurrentIndex(i)

    def _on_combo(self, _i: int) -> None:
        if self._loading:
            return
        self._clear_results()
        self._after_target_change(solve=False)

    def _on_rename(self) -> None:
        i = self._current_index()
        if i < 0:
            return
        if self._name.text().strip() == self._store[i].name:
            return
        if self._store.rename(i, self._name.text()):
            self._loading = True
            self._combo.setItemText(i, self._store[i].name)
            self._loading = False
            self.targets_changed.emit()
        else:
            self._name.setText(self._store[i].name)
            self.status_message.emit("Target names must be unique and not empty.")

    def _after_target_change(self, solve: bool) -> None:
        i = self._current_index()
        has = i >= 0
        self._hint.setVisible(not has)
        for w in (self._name, *self._xyz, *self._rpy, *self._rot_btns):
            w.setEnabled(has)
        if has:
            T = self._store[i].pose
            self._set_fields(T)
            self._loading = True
            self._name.setText(self._store[i].name)
            self._loading = False
            if self.controller is not None:
                self.controller.set_pose(T, emit=False)
                self.controller.set_status("neutral")
                self.controller.show(self._active)
        else:
            self._name.setText("")
            if self.controller is not None:
                self.controller.show(False)
        self._status.setText(_STATUS_TEXT["neutral"])
        self._update_buttons()
        if has and solve and self.auto_solve:
            self.solve()

    # ------------------------------------------------------------------ pose editing

    def _set_fields(self, T: np.ndarray) -> None:
        xyz, rpy = xyz_rpy_from_pose(T)
        self._loading = True
        for b, v in zip(self._xyz, xyz * 1000.0):
            b.setValue(float(v))
        for b, v in zip(self._rpy, np.degrees(rpy)):
            b.setValue(float(v))
        self._loading = False

    def set_pose(self, xyz_mm=None, rpy_deg=None) -> None:
        """Typed entry (what the fields do): replace position and / or orientation."""
        i = self._current_index()
        if i < 0:
            return
        T = self._store[i].pose
        xyz, rpy = xyz_rpy_from_pose(T)
        xyz = xyz if xyz_mm is None else np.asarray(xyz_mm, float) / 1000.0
        rpy = rpy if rpy_deg is None else np.radians(np.asarray(rpy_deg, float))
        self._apply_pose(pose_from_xyz_rpy(xyz, rpy), solve=True)

    def _on_fields(self, _v: float) -> None:
        if self._loading or self._current_index() < 0:
            return
        xyz = [b.value() / 1000.0 for b in self._xyz]
        rpy = np.radians([b.value() for b in self._rpy])
        self._apply_pose(pose_from_xyz_rpy(xyz, rpy), solve=True, fields=False)

    def rotate(self, axis: str, deg: float) -> None:
        """Rotate the target about its own axis (3D handle buttons)."""
        i = self._current_index()
        if i < 0:
            return
        self._apply_pose(rotate_pose(self._store[i].pose, axis, deg), solve=True)

    def move_to(self, xyz_m) -> None:
        i = self._current_index()
        if i < 0:
            return
        T = self._store[i].pose.copy()
        T[:3, 3] = np.asarray(xyz_m, float)
        self._apply_pose(T, solve=True)

    def _apply_pose(self, T: np.ndarray, solve: bool, fields: bool = True) -> None:
        i = self._current_index()
        if i < 0:
            return
        self._store.set_pose(i, T)
        if fields:
            self._set_fields(T)
        if self.controller is not None:
            self.controller.set_pose(T, emit=False)
        self._after_pose_edit(solve)

    def _after_pose_edit(self, solve: bool) -> None:
        self._clear_results()
        self.targets_changed.emit()
        if solve and self.auto_solve:
            self.solve()

    def _on_controller_pose(self, T) -> None:
        """Drag / rotate / typed edit in the 3D controller: keep store and fields in step."""
        i = self._current_index()
        if i < 0:
            return
        self._store.set_pose(i, T)
        self._set_fields(T)
        self._clear_results(keep_status=True)
        self.targets_changed.emit()

    def _on_controller_committed(self, T) -> None:
        i = self._current_index()
        if i < 0:
            return
        self._store.set_pose(i, T)
        self._set_fields(T)
        self._after_pose_edit(solve=True)

    def _on_standoff(self, v: float) -> None:
        if self.controller is not None:
            self.controller.standoff = float(v) / 1000.0

    def set_standoff(self, mm: float) -> None:
        self._standoff.setValue(float(mm))

    def _on_pick_toggled(self, on: bool) -> None:
        if self.controller is None:
            return
        if on and self._current_index() < 0:
            if not self.add_from_tcp():
                self._btn_click.setChecked(False)
                return
            self.controller.show(self._active)
        self.controller.set_pick_mode(on)
        self.status_message.emit("Click a surface in the 3D view to place the target." if on else "Ready")

    def place_on_surface(self, point, normal) -> None:
        """Place the current (or a new) target on a surface point; tool z = -normal."""
        if self.controller is None:
            return
        if self._current_index() < 0 and not self.add_from_tcp():
            return
        self.controller.show(True)
        self.controller.place_at(point, normal)

    # ------------------------------------------------------------------ IK

    def _clear_results(self, keep_status: bool = False) -> None:
        self._generation += 1
        self._solving = False
        self._solutions = []
        self._rank_note = ""
        self._table.setRowCount(0)
        if self._planning:                    # the target changed: the plan is for the old one
            self._motion_gen += 1
            if self._cancel_evt is not None:
                self._cancel_evt.set()
            self._planning = False
        self._motion_status.setText("")
        if self.preview is not None:
            self.preview.clear()
        if self.controller is not None and not keep_status:
            self.controller.set_status("neutral")
        if not keep_status:
            self._status.setText(_STATUS_TEXT["neutral"])
        self._update_buttons()

    def solve(self) -> None:
        """Rank the IK solutions of the current target in a worker. Returns at once."""
        i = self._current_index()
        if self._model is None or i < 0 or self._solving:
            return
        model = self._model
        T = self._store[i].pose.copy()
        q_cur = current_q(model)
        weights = self._weights
        svc = self._svc
        try:
            if self._cache_id != id(model):
                self._cache.invalidate()
                self._cache_id = id(model)
            cm = self._cache.get_cm(model)
        except Exception as exc:
            self._status.setText(f"IK setup failed: {exc}")
            return
        self._generation += 1
        gen = self._generation
        self._solving = True
        self._status.setText("Solving...")
        self._update_buttons()

        def work():
            session = svc.session() if svc is not None else None
            return rank_solutions(model, T, q_cur, session, weights=weights, cm=cm)

        task = FunctionTask(work)
        task.signals.result.connect(lambda r, g=gen: self._solved.emit(g, r))
        task.signals.error.connect(lambda m, g=gen: self._failed.emit(g, m))
        self._task = task
        self._runner.submit(task)

    def _on_failed(self, gen: int, message: str) -> None:
        if gen != self._generation:
            return
        self._solving = False
        last = message.strip().splitlines()[-1] if message.strip() else "error"
        self._status.setText(f"IK failed: {last}")
        self.log_requested.emit(f"[ERROR] Target IK failed: {last}")
        self._update_buttons()

    def _on_solved(self, gen: int, res: Any) -> None:
        if gen != self._generation:
            return
        self._solving = False
        self._solutions = list(res.solutions)
        self._rank_note = res.note
        self.last_rank_ms = float(res.elapsed_ms)
        n_ok = sum(s.valid for s in self._solutions)
        st = target_status(self._solutions)
        text = _STATUS_TEXT[st]
        text += f" {len(self._solutions)} solutions, {n_ok} valid ({res.elapsed_ms:.0f} ms)."
        if res.note:
            text += " " + res.note
        self._status.setText(text)
        if self.controller is not None:
            self.controller.set_status(st)
        self._fill_table()
        self._update_buttons()
        if n_ok:
            self.select_row(next(k for k, s in enumerate(self._solutions) if s.valid))
        else:
            self._show_preview()

    def _fill_table(self) -> None:
        t = self._table
        t.blockSignals(True)
        t.setRowCount(len(self._solutions))
        for r, s in enumerate(self._solutions):
            t.setItem(r, 0, QTableWidgetItem(str(r + 1)))
            bar = QProgressBar()
            bar.setRange(0, 100)
            bar.setValue(int(round(100 * s.total)))
            bar.setFormat(f"{100 * s.total:.0f}")
            bar.setTextVisible(True)
            bar.setAlignment(Qt.AlignmentFlag.AlignCenter)
            bar.setMaximumHeight(16)
            bar.setStyleSheet("QProgressBar{border:1px solid #666;border-radius:2px;background:#222;"
                              "color:#fff;text-align:center}QProgressBar::chunk{background:"
                              + ("#4caf50" if s.valid else "#b04040") + "}")
            t.setCellWidget(r, 1, bar)
            for c, key in enumerate(PART_NAMES, start=2):
                t.setItem(r, c, QTableWidgetItem(f"{100 * s.parts[key]:.0f}"))
            if s.valid:
                txt = "near singular" if s.near_singular else "OK"
            else:
                txt = s.reason.split(":")[0] or "invalid"
            item = QTableWidgetItem(txt)
            item.setToolTip(s.reason or ("sigma_min " + f"{s.raw.get('sigma_min', 0):.3f}"))
            t.setItem(r, 6, item)
            for c in range(t.columnCount()):
                it = t.item(r, c)
                if it is not None and not s.valid:
                    it.setForeground(Qt.GlobalColor.red)
        t.blockSignals(False)

    def select_row(self, row: int) -> None:
        if row < 0:
            self._table.clearSelection()
        else:
            self._table.selectRow(row)
        self._show_preview()
        self._update_buttons()

    def _on_row_selected(self) -> None:
        self._show_preview()
        self._update_buttons()

    def _on_show_all(self, on: bool) -> None:
        if self.preview is not None:
            self.preview.show_all = bool(on)
        self._show_preview()

    def _show_preview(self) -> None:
        if self.preview is None or not self._active:
            return
        self.preview.show(self._solutions, self.selected_row(), self._chk_all.isChecked())

    def redraw_preview(self) -> None:
        """The Ghosts dock cleared every ghost: draw the IK preview again."""
        self._show_preview()

    # ------------------------------------------------------------------ motion

    def play_to_target(self) -> None:
        """Plan (collision-checked) from the current q to the selected solution and send it to the
        timeline. Returns at once; ``trajectory_ready`` fires when the plan is done."""
        row = self.selected_row()
        i = self._current_index()
        if self._model is None or row < 0 or i < 0 or self._planning:
            return
        sol = self._solutions[row]
        if not sol.valid:
            self._motion_status.setText(f"This solution is not valid ({sol.reason}).")
            return
        model, svc = self._model, self._svc
        T = self._store[i].pose.copy()
        q_start, q_goal = current_q(model), list(sol.q)
        approach = self._appr.value() / 1000.0 if self._chk_approach.isChecked() else 0.0
        retreat = self._chk_approach.isChecked() and self._chk_retreat.isChecked()
        evt = threading.Event()
        self._cancel_evt = evt
        self._motion_gen += 1
        gen = self._motion_gen
        self._planning = True
        self._motion_status.setText("Planning the motion...")
        self._update_buttons()

        def work():
            sess = svc.session() if svc is not None else None
            world = sess.world if sess is not None else None
            return plan_target_motion(model, q_start, q_goal, T, world, approach, retreat,
                                      cancel=evt)

        task = FunctionTask(work)
        task.signals.result.connect(lambda r, g=gen: self._planned.emit(g, r))
        task.signals.error.connect(lambda m, g=gen: self._failed_m.emit(g, m))
        self._task = task
        self._runner.submit(task)

    def _failed_motion(self, gen: int, message: str) -> None:
        if gen != self._motion_gen:
            return
        self._planning = False
        last = message.strip().splitlines()[-1] if message.strip() else "error"
        self._motion_status.setText(f"Planning failed: {last}")
        self.log_requested.emit(f"[ERROR] Target motion failed: {last}")
        self._update_buttons()

    def _on_planned(self, gen: int, res: Any) -> None:
        if gen != self._motion_gen:
            return
        self._planning = False
        self._cancel_evt = None
        self.last_motion = res
        if not res.ok:
            self._motion_status.setText(f"No motion: {res.message}.")
            self.log_requested.emit(f"[WARNING] Target motion: {res.message}")
            self._update_buttons()
            return
        parts = ", ".join(f"{k} {b - a:.1f} s" for k, (a, b) in res.segments.items())
        self._motion_status.setText(f"Playing: {res.trajectory.duration:.1f} s ({parts}).")
        self.log_requested.emit(f"[INFO] Target motion: {res.trajectory.duration:.2f} s ({parts}).")
        self._update_buttons()
        self.trajectory_ready.emit(res.trajectory, {"segments": dict(res.segments),
                                                    "q_goal": list(self._solutions[self.selected_row()].q)
                                                    if self.selected_row() >= 0 else None})

    def cancel(self) -> None:
        if self._cancel_evt is not None and self._planning:
            self._cancel_evt.set()
            self._motion_status.setText("Cancelling...")
        elif self._solving:
            self._generation += 1
            self._solving = False
            self._status.setText("Cancelled.")
            self._update_buttons()

    def _cancel_work(self) -> None:
        self._generation += 1
        self._motion_gen += 1
        if self._cancel_evt is not None:
            self._cancel_evt.set()
        self._solving = False
        self._planning = False
        self._cancel_evt = None

    def _update_buttons(self) -> None:
        has_model = self._model is not None
        has_t = self._current_index() >= 0
        busy = self._solving or self._planning
        self._btn_add.setEnabled(has_model and not busy)
        self._btn_click.setEnabled(has_model)
        self._btn_delete.setEnabled(has_t and not busy)
        self._btn_solve.setEnabled(has_model and has_t and not busy)
        row = self.selected_row()
        ok_row = 0 <= row < len(self._solutions) and self._solutions[row].valid
        self._btn_play.setEnabled(has_model and ok_row and not busy)
        self._btn_cancel.setEnabled(busy)
        self._btn_weights.setEnabled(True)
        self._combo.setEnabled(not busy and len(self._store) > 0)
