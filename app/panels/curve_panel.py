"""Curve dock (U4b.3): a curve from the current pose to the selected target(s).

Start = the live arm pose. End = the target chosen in the Target dock (or a chain A -> B -> C of
checked targets). Type: Line / Arc (one via point) / Spline (any number of via points, dragged in
3D or typed). Every change (curve type, via point, target, arm pose) re-runs the path and the check
(debounced 150 ms, worker thread) and redraws the 3D tube: green ok, orange near a singularity or
tight, red unreachable / colliding / branch flip. Play stays off while a segment is red and the
tooltip says where. "Generate" times the curve (constant TCP speed or time-optimal TOPP) and sends
the trajectory to the timeline.

Plain methods for tests: ``refresh_now``, ``generate``, ``add_via``, ``delete_via``,
``set_via_xyz``. Results arrive through the task runner; ``busy`` says when they are still due.
"""
from __future__ import annotations

from typing import Any

import numpy as np
from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtWidgets import (
    QCheckBox, QComboBox, QDoubleSpinBox, QGroupBox, QHBoxLayout, QLabel, QListWidget,
    QListWidgetItem, QPushButton, QVBoxLayout, QWidget,
)

import core_bridge as cb
from app.kin_utils import CompiledCache, current_q
from app.planning.curve import (
    DEFAULT_SPEED, KINDS, CurveResult, build_curve, longest_segment, tcp_at_time, timed_trajectory,
    trajectory_speeds,
)
from app.planning.path_check import TUBE_MODES, tcp_speed, tube_colors
from app.task_runner import FunctionTask

DEBOUNCE_MS = 150
KIND_LABELS = {"line": "Line", "arc": "Arc (one via point)", "spline": "Spline (via points)"}
TIMING_LABELS = (("speed", "Constant TCP speed"), ("topp", "Time-optimal (torque limits)"))
MODE_LABELS = {"status": "Problems (green / orange / red)", "speed": "TCP speed",
               "clearance": "Collision clearance", "singularity": "Singularity closeness"}
GHOST_KEY = ("curve", "end")
GHOST_COLOR = "#7e57c2"


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


class CurvePanel(QWidget):
    log_requested = Signal(str)
    status_message = Signal(str)
    trajectory_ready = Signal(object, dict)      # Trajectory, {"play", "q_end", "duration", ...}
    _computed = Signal(int, object)
    _compute_failed = Signal(int, str)
    _timed = Signal(int, object, bool)
    _time_failed = Signal(int, str)

    def __init__(self, task_runner: Any, parent=None):
        super().__init__(parent)
        self._runner = task_runner
        self._model: Any = None
        self._project: Any = None
        self._tp: Any = None                     # Target panel (store + selection)
        self._svc: Any = None
        self._viewport: Any = None
        self.handles: Any = None
        self._cache = CompiledCache()
        self._cache_id: int | None = None
        self._via: dict[str, list] = {}          # target name -> via points (m)
        self._result: CurveResult | None = None
        self._stop_names: list[str] = []
        self._traj: Any = None
        self._traj_speeds: np.ndarray | None = None
        self._gen = 0
        self._time_gen = 0
        self._computing = False
        self._generating = False
        self._active = False
        self._disposed = False
        self._loading = False
        self.last_compute_ms = 0.0
        self.last_timing_note = ""
        self._error = ""
        self._task: Any = None
        self._task_t: Any = None

        self._timer = QTimer(self)
        self._timer.setSingleShot(True)
        self._timer.setInterval(DEBOUNCE_MS)
        self._timer.timeout.connect(self.refresh_now)

        lay = QVBoxLayout(self)
        lay.setContentsMargins(6, 6, 6, 6)
        lay.setSpacing(6)

        # ---- 1. curve
        g1 = QGroupBox("Curve from the current pose")
        l1 = QVBoxLayout(g1)
        self._route = QLabel("")
        self._route.setWordWrap(True)
        l1.addWidget(self._route)
        row = QHBoxLayout()
        row.addWidget(QLabel("Type"))
        self._kind = QComboBox()
        for k in KINDS:
            self._kind.addItem(KIND_LABELS[k], k)
        self._kind.setToolTip("Line: straight. Arc: circle through one via point. "
                              "Spline: smooth curve through any number of via points.")
        self._kind.currentIndexChanged.connect(self._on_kind)
        row.addWidget(self._kind, 1)
        l1.addLayout(row)
        self._chain = QCheckBox("Chain several targets (A -> B -> C)")
        self._chain.setToolTip("Tick the targets to visit, in the order of the target list. "
                               "Each curve starts where the previous one ended.")
        self._chain.toggled.connect(self._on_chain)
        l1.addWidget(self._chain)
        self._chain_list = QListWidget()
        self._chain_list.setMaximumHeight(84)
        self._chain_list.itemChanged.connect(self._on_chain_item)
        self._chain_list.setVisible(False)
        l1.addWidget(self._chain_list)
        self._hint = QLabel("No target yet. Make one in the Target tab first.")
        self._hint.setWordWrap(True)
        l1.addWidget(self._hint)
        lay.addWidget(g1)

        # ---- 2. via points
        g2 = QGroupBox("Via points")
        l2 = QVBoxLayout(g2)
        self._via_list = QListWidget()
        self._via_list.setMaximumHeight(84)
        self._via_list.currentRowChanged.connect(self._on_via_row)
        l2.addWidget(self._via_list)
        row = QHBoxLayout()
        self._via_xyz = [_spin(-9999.0, 9999.0, 1, " mm") for _ in range(3)]
        for ax, b in zip("XYZ", self._via_xyz):
            row.addWidget(QLabel(ax))
            row.addWidget(b, 1)
            b.valueChanged.connect(self._on_via_fields)
        l2.addLayout(row)
        row = QHBoxLayout()
        self._btn_add = QPushButton("Add via point")
        self._btn_add.setToolTip("New via point in the middle of the longest part of the curve. "
                                 "Drag the yellow sphere in the 3D view to move it.")
        self._btn_add.clicked.connect(self.add_via)
        self._btn_del = QPushButton("Delete")
        self._btn_del.clicked.connect(self.delete_via)
        row.addWidget(self._btn_add, 1)
        row.addWidget(self._btn_del)
        l2.addLayout(row)
        self._via_note = QLabel("")
        self._via_note.setWordWrap(True)
        l2.addWidget(self._via_note)
        lay.addWidget(g2)

        # ---- 3. timing + colours
        g3 = QGroupBox("Timing and colours")
        l3 = QVBoxLayout(g3)
        row = QHBoxLayout()
        row.addWidget(QLabel("Timing"))
        self._timing = QComboBox()
        for k, label in TIMING_LABELS:
            self._timing.addItem(label, k)
        self._timing.setToolTip("Constant TCP speed, or the fastest motion the joint speed, "
                                "acceleration and torque limits allow")
        self._timing.currentIndexChanged.connect(self._on_timing)
        row.addWidget(self._timing, 1)
        l3.addLayout(row)
        row = QHBoxLayout()
        row.addWidget(QLabel("TCP speed"))
        self._speed = _spin(1.0, 2000.0, 0, " mm/s", 10.0)
        self._speed.setValue(DEFAULT_SPEED * 1000.0)
        self._speed.valueChanged.connect(self._on_speed)
        row.addWidget(self._speed)
        row.addStretch(1)
        l3.addLayout(row)
        row = QHBoxLayout()
        row.addWidget(QLabel("Colour by"))
        self._mode = QComboBox()
        for m in TUBE_MODES:
            self._mode.addItem(MODE_LABELS[m], m)
        self._mode.currentIndexChanged.connect(lambda _i: self._redraw_tube())
        row.addWidget(self._mode, 1)
        l3.addLayout(row)
        lay.addWidget(g3)

        # ---- 4. check + generate
        g4 = QGroupBox("Check and play")
        l4 = QVBoxLayout(g4)
        self._check_label = QLabel("")
        self._check_label.setWordWrap(True)
        l4.addWidget(self._check_label)
        self._cycle = QLabel("")
        l4.addWidget(self._cycle)
        row = QHBoxLayout()
        self._btn_generate = QPushButton("Generate")
        self._btn_generate.setToolTip("Time the curve and put it into the timeline (Simulation dock)")
        self._btn_generate.clicked.connect(lambda: self.generate(False))
        self._btn_play = QPushButton("Play")
        self._btn_play.clicked.connect(lambda: self.generate(True))
        row.addWidget(self._btn_generate, 1)
        row.addWidget(self._btn_play, 1)
        l4.addLayout(row)
        lay.addWidget(g4)
        lay.addStretch(1)

        self._computed.connect(self._on_computed)
        self._compute_failed.connect(self._on_compute_failed)
        self._timed.connect(self._on_timed)
        self._time_failed.connect(self._on_time_failed)
        self._update_controls()

    # ------------------------------------------------------------------ setup

    def attach_viewport(self, viewport: Any) -> None:
        from app.viewport.curve_widget import ViaHandles
        self._viewport = viewport
        self.handles = ViaHandles(viewport, parent=self)
        self.handles.point_moved.connect(self._on_handle_moved)

    def attach_target_panel(self, tp: Any) -> None:
        self._tp = tp
        tp.targets_changed.connect(self._on_targets_changed)
        tp.target_combo.currentIndexChanged.connect(lambda _i: self._on_selection())

    def set_collision_service(self, svc: Any) -> None:
        self._svc = svc

    def set_project(self, project: Any) -> None:
        self._project = project
        self._via.clear()
        self._invalidate()
        self._reload_chain()
        self._schedule()

    def set_model(self, model: Any) -> None:
        self._model = model
        self._cache.invalidate()
        self._via.clear()
        self._invalidate()
        self._update_controls()
        self._schedule()

    def set_active(self, on: bool) -> None:
        """Dock visible: draw the tube and the via handles; hidden: remove them."""
        self._active = bool(on)
        if self._disposed:
            return
        if on:
            self._schedule()
            self._redraw_tube()
        else:
            self._timer.stop()
            if self._traj is None:               # a generated curve stays drawn (marker, end ghost)
                rv = self._rv()                  # while its trajectory is in the timeline
                if rv is not None:
                    rv.set_path_tube(None)
                    rv.remove_ghost(GHOST_KEY)
            if self.handles is not None:
                self.handles.show(False)

    def showEvent(self, e) -> None:
        super().showEvent(e)
        self.set_active(True)

    def hideEvent(self, e) -> None:
        super().hideEvent(e)
        try:
            self.set_active(False)
        except RuntimeError:
            pass

    def dispose(self) -> None:
        self._disposed = True
        self._timer.stop()
        self._gen += 1
        self._time_gen += 1
        if self.handles is not None:
            self.handles.dispose()

    shutdown = dispose

    def _rv(self) -> Any:
        return getattr(self._viewport, "robot_view", None) if self._viewport is not None else None

    # ------------------------------------------------------------------ state access

    @property
    def result(self) -> CurveResult | None:
        return self._result

    @property
    def trajectory(self) -> Any:
        return self._traj

    @property
    def busy(self) -> bool:
        return self._computing or self._generating or self._timer.isActive()

    @property
    def kind(self) -> str:
        return str(self._kind.currentData())

    @property
    def timing(self) -> str:
        return str(self._timing.currentData())

    @property
    def color_mode(self) -> str:
        return str(self._mode.currentData())

    @property
    def check_text(self) -> str:
        return self._check_label.text()

    @property
    def cycle_text(self) -> str:
        return self._cycle.text()

    @property
    def hint_text(self) -> str:
        return self._hint.text()

    @property
    def via_list(self) -> QListWidget:
        return self._via_list

    @property
    def chain_list(self) -> QListWidget:
        return self._chain_list

    @property
    def play_tooltip(self) -> str:
        return self._btn_play.toolTip()

    def button_enabled(self) -> dict:
        return {"generate": self._btn_generate.isEnabled(), "play": self._btn_play.isEnabled(),
                "add": self._btn_add.isEnabled(), "delete": self._btn_del.isEnabled()}

    def set_kind(self, kind: str) -> None:
        self._kind.setCurrentIndex(KINDS.index(kind))

    def set_timing(self, timing: str) -> None:
        self._timing.setCurrentIndex([k for k, _ in TIMING_LABELS].index(timing))

    def set_speed_mm_s(self, v: float) -> None:
        self._speed.setValue(float(v))

    def set_color_mode(self, mode: str) -> None:
        self._mode.setCurrentIndex(TUBE_MODES.index(mode))

    def set_chain(self, on: bool, names: list | None = None) -> None:
        self._chain.setChecked(bool(on))
        if names is not None:
            self._loading = True
            for i in range(self._chain_list.count()):
                it = self._chain_list.item(i)
                it.setCheckState(Qt.CheckState.Checked if it.text() in names
                                 else Qt.CheckState.Unchecked)
            self._loading = False
            self._on_chain_item(None)

    # ------------------------------------------------------------------ targets

    def _store(self) -> Any:
        return self._tp.store if self._tp is not None else None

    def _selected_name(self) -> str | None:
        store = self._store()
        if store is None:
            return None
        i = self._tp.current_index
        return store[i].name if 0 <= i < len(store) else None

    def _stops(self) -> list[tuple[str, np.ndarray]]:
        """``[(name, pose)]`` of the legs: the checked targets (chain) or the selected one."""
        store = self._store()
        if store is None or len(store) == 0:
            return []
        if self._chain.isChecked():
            names = [self._chain_list.item(i).text() for i in range(self._chain_list.count())
                     if self._chain_list.item(i).checkState() == Qt.CheckState.Checked]
            out = []
            for t in store:
                if t.name in names:
                    out.append((t.name, np.asarray(t.pose, float)))
            return out
        i = self._tp.current_index
        return [(store[i].name, np.asarray(store[i].pose, float))] if 0 <= i < len(store) else []

    def _reload_chain(self) -> None:
        store = self._store()
        off = {self._chain_list.item(i).text() for i in range(self._chain_list.count())
               if self._chain_list.item(i).checkState() != Qt.CheckState.Checked}
        self._loading = True
        self._chain_list.clear()
        for t in (list(store) if store is not None else []):
            it = QListWidgetItem(t.name)
            it.setFlags(it.flags() | Qt.ItemFlag.ItemIsUserCheckable)
            it.setCheckState(Qt.CheckState.Unchecked if t.name in off else Qt.CheckState.Checked)
            self._chain_list.addItem(it)
        self._loading = False

    def _on_targets_changed(self) -> None:
        store = self._store()
        if store is not None and [self._chain_list.item(i).text()
                                  for i in range(self._chain_list.count())] != store.names():
            self._reload_chain()
        self._invalidate()
        self._schedule()

    def _on_selection(self) -> None:
        self._invalidate()
        self._fill_via_list()
        self._sync_handles()
        self._schedule()

    def _on_chain(self, on: bool) -> None:
        self._chain_list.setVisible(bool(on))
        self._invalidate()
        self._schedule()

    def _on_chain_item(self, _item) -> None:
        if self._loading:
            return
        self._invalidate()
        self._schedule()

    def _on_kind(self, _i: int) -> None:
        self._invalidate()
        self._fill_via_list()
        self._sync_handles()
        self._update_controls()
        self._schedule()

    def _on_timing(self, _i: int) -> None:
        self._speed.setEnabled(self.timing == "speed")
        self._traj = None
        self._traj_speeds = None
        self._update_controls()
        self._refresh_cycle()
        self._redraw_tube()

    def _on_speed(self, _v: float) -> None:
        self._invalidate()
        self._schedule()

    def notify_q_changed(self) -> None:
        """The arm was moved by hand (joint panel, drag, IK apply): the curve starts there now.
        Not called while a trajectory plays."""
        if self._model is None or not self._active:
            return
        self._invalidate(keep_tube=True)
        self._schedule()

    # ------------------------------------------------------------------ via points

    def _via_for(self, name: str) -> list:
        return self._via.setdefault(name, [])

    def _leg_ends(self) -> tuple[np.ndarray, np.ndarray] | None:
        """Start and end position of the leg whose via points are edited."""
        name = self._selected_name()
        stops = dict(self._stops())
        if name is None or name not in stops or self._model is None:
            return None
        try:
            start = np.asarray(cb.fk(self._model)["tcp"], float)[:3, 3]
        except Exception:
            return None
        names = [n for n, _ in self._stops()]
        if name in names and names.index(name) > 0:
            start = stops[names[names.index(name) - 1]][:3, 3]
        return start, stops[name][:3, 3]

    def add_via(self) -> bool:
        name = self._selected_name()
        ends = self._leg_ends()
        if name is None or ends is None or self.kind != "spline":
            return False
        via = self._via_for(name)
        i, p = longest_segment(ends[0], via, ends[1])
        via.insert(i, np.asarray(p, float))
        self._after_via_edit(select=i)
        return True

    def delete_via(self) -> bool:
        name = self._selected_name()
        row = self._via_list.currentRow()
        if name is None or row < 0 or row >= len(self._via_for(name)) or self.kind != "spline":
            return False
        del self._via[name][row]
        self._after_via_edit(select=min(row, len(self._via[name]) - 1))
        return True

    def set_via_xyz(self, index: int, xyz_mm) -> None:
        """Typed entry: move via point ``index`` to ``xyz_mm`` (mm)."""
        name = self._selected_name()
        if name is None or not (0 <= index < len(self._via_for(name))):
            return
        self._via[name][index] = np.asarray(xyz_mm, float) / 1000.0
        self._after_via_edit(select=index)

    def via_points(self) -> list:
        name = self._selected_name()
        return [] if name is None else [v.copy() for v in self._via.get(name, [])]

    def _after_via_edit(self, select: int = -1) -> None:
        self._fill_via_list(select)
        self._sync_handles()
        self._invalidate(keep_tube=True)
        self._update_controls()
        self._schedule()

    def _fill_via_list(self, select: int = -1) -> None:
        name = self._selected_name()
        via = self._via.get(name, []) if name is not None and self.kind != "line" else []
        self._loading = True
        self._via_list.clear()
        for i, v in enumerate(via):
            self._via_list.addItem(f"Via {i + 1}   ({v[0] * 1000:.0f}, {v[1] * 1000:.0f}, "
                                   f"{v[2] * 1000:.0f}) mm")
        if 0 <= select < len(via):
            self._via_list.setCurrentRow(select)
        self._loading = False
        self._on_via_row(self._via_list.currentRow())

    def _on_via_row(self, row: int) -> None:
        name = self._selected_name()
        via = self._via.get(name, []) if name is not None else []
        ok = 0 <= row < len(via)
        self._loading = True
        for b, v in zip(self._via_xyz, via[row] * 1000.0 if ok else (0.0, 0.0, 0.0)):
            b.setValue(float(v))
        self._loading = False
        for b in self._via_xyz:
            b.setEnabled(ok)
        self._update_controls()

    def _on_via_fields(self, _v: float) -> None:
        if self._loading:
            return
        row = self._via_list.currentRow()
        if row >= 0:
            self.set_via_xyz(row, [b.value() for b in self._via_xyz])

    def _on_handle_moved(self, i: int, xyz) -> None:
        name = self._selected_name()
        if name is None or not (0 <= i < len(self._via_for(name))):
            return
        self._via[name][i] = np.asarray(xyz, float)
        row = self._via_list.currentRow()
        self._fill_via_list(select=i if row < 0 else i)
        self._invalidate(keep_tube=True)
        self._schedule()

    def _sync_handles(self) -> None:
        if self.handles is None:
            return
        name = self._selected_name()
        show = self._active and self.kind in ("arc", "spline") and name is not None
        self.handles.set_points(self._via.get(name, []) if name is not None else [])
        self.handles.show(show)

    # ------------------------------------------------------------------ compute

    def _invalidate(self, keep_tube: bool = False) -> None:
        """Curve inputs changed: the generated trajectory (and the end ghost) is stale."""
        self._gen += 1
        self._time_gen += 1
        self._computing = False
        self._generating = False
        self._error = ""
        self._traj = None
        self._traj_speeds = None
        rv = self._rv()
        if rv is not None:
            rv.set_path_marker(None)
            rv.remove_ghost(GHOST_KEY)
        if not keep_tube:
            self._result = None
        if not self._active and rv is not None:
            rv.set_path_tube(None)
        self._update_controls()

    def _schedule(self) -> None:
        if self._disposed:
            return
        if self._active:
            self._timer.start()
        self._update_controls()

    def refresh_now(self) -> None:
        """Run path + check in a worker now (the debounce timer calls this)."""
        self._timer.stop()
        stops = self._stops()
        model = self._model
        if self._disposed or model is None or not stops:
            self._result = None
            self._redraw_tube()
            self._update_controls()
            return
        try:
            if self._cache_id != id(model):
                self._cache.invalidate()
                self._cache_id = id(model)
            cm = self._cache.get_cm(model)
        except Exception as exc:
            self._error = f"Setup failed: {exc}"
            self._update_controls()
            return
        q0 = current_q(model)
        names = [n for n, _ in stops]
        legs = [(T, [v.copy() for v in self._via.get(n, [])]) for n, T in stops]
        kind, speed = self.kind, self._speed.value() / 1000.0
        svc = self._svc
        self._gen += 1
        gen = self._gen
        self._computing = True
        self._stop_names = names
        self._error = ""
        self._update_controls()

        def work():
            import time
            t0 = time.perf_counter()
            sess = svc.session() if svc is not None else None
            res = build_curve(model, q0, legs, kind, speed, sess, cm)
            return res, (time.perf_counter() - t0) * 1000.0

        task = FunctionTask(work)
        task.signals.result.connect(lambda r, g=gen: self._computed.emit(g, r))
        task.signals.error.connect(lambda m, g=gen: self._compute_failed.emit(g, m))
        self._task = task
        self._runner.submit(task)

    def _on_compute_failed(self, gen: int, message: str) -> None:
        if gen != self._gen:
            return
        self._computing = False
        last = message.strip().splitlines()[-1] if message.strip() else "error"
        self._error = f"Curve failed: {last}"
        self.log_requested.emit(f"[ERROR] Curve failed: {last}")
        self._update_controls()

    def _on_computed(self, gen: int, payload: Any) -> None:
        if gen != self._gen:
            return
        res, ms = payload
        self._computing = False
        self.last_compute_ms = float(ms)
        self._result = res
        if res.legs and res.legs[0].kind == "arc":
            for name, leg in zip(self._stop_names, res.legs):      # adopt the default arc via point
                if not self._via.get(name) and leg.via:
                    self._via[name] = [v.copy() for v in leg.via]
        self._fill_via_list(self._via_list.currentRow())
        self._sync_handles()
        self._redraw_tube()
        self._update_controls()

    # ------------------------------------------------------------------ drawing

    def _speeds(self, n: int) -> np.ndarray:
        res = self._result
        if self._traj is not None and self._model is not None:
            if self._traj_speeds is None or len(self._traj_speeds) != n:
                self._traj_speeds = trajectory_speeds(self._model, self._traj, n)
            return self._traj_speeds
        return tcp_speed(self._points_as_targets(res), res.t) if res is not None else np.zeros(n)

    @staticmethod
    def _points_as_targets(res: CurveResult) -> np.ndarray:
        T = np.tile(np.eye(4), (len(res.points), 1, 1))
        T[:, :3, 3] = res.points
        return T

    def _redraw_tube(self) -> None:
        rv = self._rv()
        if rv is None or self._disposed:
            return
        res = self._result
        shown = self._active or self._traj is not None
        if not shown or res is None or len(res.points) < 2 or res.check is None:
            rv.set_path_tube(None)
            return
        mode = self.color_mode
        n = len(res.points)
        if mode == "speed":
            col = tube_colors(None, "speed", self._speeds(n))
        else:
            col = tube_colors(res.check, mode)
        if len(col) != n:                                   # a failed path may have fewer samples
            col = np.vstack([col, np.tile(col[-1:] if len(col) else [[1.0, 0.3, 0.3]],
                                          (n - len(col), 1))])
        rv.set_path_tube(res.points, col, via_points=res.via_points)

    def redraw_ghost(self) -> None:
        """Ghost of the end pose of the generated trajectory (stays visible while it plays)."""
        rv = self._rv()
        if rv is None or self._disposed:
            return
        rv.remove_ghost(GHOST_KEY)
        if self._traj is None or self._model is None:
            return
        try:
            q_end = self._traj.q(self._traj.duration)
            cm = self._cache.get_cm(self._model)
            rv.add_ghost(GHOST_KEY, self._model, cb.fk_compiled(cm, list(q_end)), GHOST_COLOR)
        except Exception:
            pass

    def set_playback_time(self, t: float) -> None:
        """Playback / scrub time of the generated trajectory: moving marker on the tube."""
        rv = self._rv()
        if rv is None or self._traj is None or self._model is None:
            return
        try:
            rv.set_path_marker(tcp_at_time(self._model, self._traj, t, self._cache.get_cm(self._model)))
        except Exception:
            pass

    # ------------------------------------------------------------------ generate

    def generate(self, play: bool = False) -> None:
        """Time the curve and send it to the timeline. Returns at once; ``trajectory_ready`` fires."""
        res, model = self._result, self._model
        if res is None or model is None or not res.ok or self._generating or self._computing:
            return
        timing = self.timing
        speed = self._speed.value() / 1000.0
        svc = self._svc
        self._time_gen += 1
        gen = self._time_gen
        self._generating = True
        self._error = ""
        self._update_controls()

        def work():
            sess = svc.session() if svc is not None else None
            return timed_trajectory(model, res, timing, sess.world if sess is not None else None,
                                    speed)

        task = FunctionTask(work)
        task.signals.result.connect(lambda r, g=gen, p=play: self._timed.emit(g, r, p))
        task.signals.error.connect(lambda m, g=gen: self._time_failed.emit(g, m))
        self._task_t = task
        self._runner.submit(task)

    def _on_time_failed(self, gen: int, message: str) -> None:
        if gen != self._time_gen:
            return
        self._generating = False
        last = message.strip().splitlines()[-1] if message.strip() else "error"
        self._error = f"Timing failed: {last}"
        self.log_requested.emit(f"[WARNING] Curve timing failed: {last}")
        self._update_controls()

    def _on_timed(self, gen: int, traj: Any, play: bool) -> None:
        if gen != self._time_gen:
            return
        self._generating = False
        self._traj = traj
        self._traj_speeds = None
        self._refresh_check_text()
        self._refresh_cycle()
        self._redraw_tube()
        self.redraw_ghost()
        self._update_controls()
        self.log_requested.emit(f"[INFO] Curve: {traj.duration:.2f} s, {len(self._result.legs)} leg(s), "
                                f"{self.timing}.")
        self.trajectory_ready.emit(traj, {"play": bool(play), "duration": float(traj.duration),
                                          "q_end": [float(v) for v in traj.q(traj.duration)]})

    # ------------------------------------------------------------------ labels, buttons

    def _refresh_check_text(self) -> None:
        res = self._result
        if self._error:
            self._check_label.setText(self._error)
            self._check_label.setStyleSheet("color: #ff6b6b; font-weight: bold;")
            return
        if self._computing or self._generating:
            self._check_label.setStyleSheet("")
            self._check_label.setText("Checking the curve..." if self._computing else
                                      ("Timing the curve..." if self.timing == "topp"
                                       else "Generating..."))
            return
        if res is None or res.check is None:
            self._check_label.setText("")
            self._check_label.setStyleSheet("")
            return
        if res.message:
            text, color = f"Problem: {res.message}", "#ff6b6b"
            if not res.check.ok and res.check.problem_text():
                text += " " + res.check.problem_text()
        elif not res.check.ok:
            text, color = "Problem: " + res.check.problem_text(), "#ff6b6b"
        elif res.check.n_warn:
            text = res.check.summary()
            color = "#ffb74d"
        else:
            text, color = "Curve is clear.", "#81c784"
        note = next((leg.note for leg in res.legs if leg.note), "")
        self._check_label.setText(text + (" " + note if note else ""))
        self._check_label.setStyleSheet(f"color: {color}; font-weight: bold;")

    def _refresh_cycle(self) -> None:
        res = self._result
        if res is None or not res.legs:
            self._cycle.setText("")
            return
        legs = f" ({len(res.legs)} legs)" if len(res.legs) > 1 else ""
        if self._traj is not None:
            self._cycle.setText(f"Cycle time: {self._traj.duration:.2f} s{legs}")
        elif self.timing == "speed" and res.ok:
            self._cycle.setText(f"Cycle time: {res.duration:.2f} s{legs} at "
                                f"{self._speed.value():.0f} mm/s")
        else:
            self._cycle.setText(f"Length {res.length * 1000:.0f} mm{legs}. Cycle time after Generate.")

    def _update_controls(self) -> None:
        has_model = self._model is not None
        stops = self._stops() if self._tp is not None else []
        self._hint.setVisible(not stops)
        if not stops and self._tp is not None and len(self._tp.store):
            self._hint.setText("Chain: tick at least one target." if self._chain.isChecked()
                               else "Choose a target in the Target tab.")
        elif not stops:
            self._hint.setText("No target yet. Make one in the Target tab first.")
        names = [n for n, _ in stops]
        if names and has_model:
            self._route.setText("From the current pose to " + " -> ".join(names) + ".")
        else:
            self._route.setText("")
        spline = self.kind == "spline"
        sel = self._selected_name()
        n_via = len(self._via.get(sel, [])) if sel is not None else 0
        busy = self._generating
        self._btn_add.setEnabled(has_model and bool(stops) and spline and not busy)
        self._btn_del.setEnabled(spline and self._via_list.currentRow() >= 0 and not busy)
        self._via_list.setEnabled(self.kind != "line")
        if self.kind == "line":
            self._via_note.setText("A line has no via points.")
        elif self.kind == "arc":
            self._via_note.setText("The arc passes through its one via point. Drag it in 3D or type it.")
        else:
            self._via_note.setText("" if n_via else "No via point: the curve is a straight line.")
        self._speed.setEnabled(self.timing == "speed")
        res = self._result
        self._refresh_check_text()
        self._refresh_cycle()
        good = (res is not None and res.ok and not self._computing and not self._timer.isActive()
                and not self._generating)
        self._btn_generate.setEnabled(has_model and good)
        self._btn_play.setEnabled(has_model and good)
        tip = ""
        if res is not None and not res.ok:
            tip = "Play is off: " + (res.problem_text() or "the curve is not solved")
        self._btn_play.setToolTip(tip or "Generate the trajectory and play it")
        self._btn_generate.setToolTip(tip or "Time the curve and put it into the timeline "
                                      "(Simulation dock)")
