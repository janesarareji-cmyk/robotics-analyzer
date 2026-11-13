"""
app/panels/timeline_panel.py
R6.4: waypoint table + timeline controls for a joint-space trajectory. Embedded in the
Simulation dock (SimPanel owns the engine and the playback). No simulation logic here.

Table: one row per waypoint, columns t and one per DOF. Invalid rows are marked red
(tooltip = reason), never dropped; limit violations are marked yellow (warning only).
A trajectory is built only while every row is valid and there are at least 2 rows; the
last valid one is kept for saving (``trajectory``) and playback needs a currently valid
table (``playable``).
"""

from __future__ import annotations

import math
from typing import Callable

from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QBrush, QColor
from PySide6.QtWidgets import (
    QCheckBox, QComboBox, QDoubleSpinBox, QGridLayout, QGroupBox, QHBoxLayout, QLabel, QPushButton,
    QSlider, QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget,
)

from app.sim.playback import MAX_SPEED, MIN_SPEED
from app.sim.trajectory import INTERPOLATIONS, Trajectory, Waypoint, waypoint_problems

SLIDER_STEPS = 1000
TEACH_SPEED = 1.0       # default joint speed [rad/s or m/s] for taught segment times
TEACH_MIN_SEGMENT = 0.5  # shortest taught segment [s]
_BAD = QBrush(QColor(210, 70, 70))
_WARN = QBrush(QColor(230, 200, 70))
_BLACK = QBrush(QColor(0, 0, 0))


def _fmt(x: float) -> str:
    return repr(float(x))


class TimelinePanel(QWidget):
    trajectory_changed = Signal(object)    # Trajectory or None (valid edit / cleared / dropped)
    play_requested = Signal()
    pause_requested = Signal()
    stop_requested = Signal()
    seek_requested = Signal(float)
    speed_changed = Signal(float)
    loop_changed = Signal(bool)
    log_requested = Signal(str)

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self._n_dof = 0
        self._limits: tuple[list[float], list[float]] = ([], [])
        self._traj: Trajectory | None = None
        self._playable = False
        self._loading = False
        self._quiet = False
        self._want = (True, False, False)   # SimPanel's play / pause / stop request
        self._q_provider: Callable[[], list[float]] | None = None
        self._pose_check: Callable[[list[float]], str | None] | None = None
        self._build_ui()
        self._update_controls()

    def detach_teach_group(self) -> QWidget:
        """Remove the Teach group from this panel and return it (SimPanel puts it on top)."""
        self._outer.removeWidget(self._teach_group)
        return self._teach_group

    # ------------------------------------------------------------------ UI
    def _build_ui(self) -> None:
        box = QGroupBox("Trajectory")
        lay = QVBoxLayout(box)
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        # Teach group (U2.5): record the current pose as a waypoint, then play.
        teach = QGroupBox("Teach waypoints")
        self._teach_group = teach
        tlay = QGridLayout(teach)
        self._btn_teach = QPushButton("Add pose")
        self._btn_teach.setToolTip("Add current pose as waypoint")
        self._btn_teach_undo = QPushButton("Undo last")
        self._btn_teach_clear = QPushButton("Clear")
        self._btn_teach_clear.setToolTip("Clear waypoints")
        self._btn_teach_play = QPushButton("Play")
        self._teach_count = QLabel("Waypoints: 0")
        tlay.addWidget(self._btn_teach, 0, 0)
        tlay.addWidget(self._btn_teach_undo, 0, 1)
        tlay.addWidget(self._btn_teach_clear, 1, 0)
        tlay.addWidget(self._btn_teach_play, 1, 1)
        tlay.addWidget(self._teach_count, 2, 0, 1, 2)
        self._outer = outer
        outer.addWidget(teach)
        outer.addWidget(box)
        self._btn_teach.clicked.connect(self.teach_add)
        self._btn_teach_undo.clicked.connect(self.teach_undo)
        self._btn_teach_clear.clicked.connect(self.teach_clear)
        self._btn_teach_play.clicked.connect(self.teach_play)

        row = QHBoxLayout()
        self._interp = QComboBox()
        self._interp.addItems(list(INTERPOLATIONS))
        self._clamped = QCheckBox("Clamped ends")
        self._clamped.setChecked(True)
        row.addWidget(QLabel("Interpolation"))
        row.addWidget(self._interp, 1)
        lay.addLayout(row)
        lay.addWidget(self._clamped)

        self._table = QTableWidget(0, 1)
        self._table.setMinimumHeight(120)
        lay.addWidget(self._table)

        row = QHBoxLayout()
        self._btn_add = QPushButton("Add")
        self._btn_del = QPushButton("Delete")
        self._btn_up = QPushButton("Up")
        self._btn_down = QPushButton("Down")
        self._btn_capture = QPushButton("Capture")
        self._btn_capture.setToolTip("Capture current pose")
        grid = QGridLayout()
        for i, b in enumerate((self._btn_add, self._btn_del, self._btn_up, self._btn_down,
                               self._btn_capture)):
            grid.addWidget(b, i // 3, i % 3)
        lay.addLayout(grid)

        self._info = QLabel("")
        self._info.setWordWrap(True)
        lay.addWidget(self._info)

        row = QHBoxLayout()
        self._btn_play = QPushButton("Play")
        self._btn_pause = QPushButton("Pause")
        self._btn_stop = QPushButton("Stop")
        for b in (self._btn_play, self._btn_pause, self._btn_stop):
            row.addWidget(b)
        lay.addLayout(row)

        self._slider = QSlider(Qt.Orientation.Horizontal)
        self._slider.setRange(0, SLIDER_STEPS)
        lay.addWidget(self._slider)
        row = QHBoxLayout()
        self._time_label = QLabel("t = 0.000 / 0.000 s")
        self._speed = QDoubleSpinBox()
        self._speed.setDecimals(2)
        self._speed.setRange(MIN_SPEED, MAX_SPEED)
        self._speed.setSingleStep(0.1)
        self._speed.setValue(1.0)
        self._speed.setSuffix("x")
        self._loop = QCheckBox("Loop")
        lay.addWidget(self._time_label)
        row.addStretch(1)
        row.addWidget(self._speed)
        row.addWidget(self._loop)
        lay.addLayout(row)

        self._table.itemChanged.connect(self._on_item_changed)
        self._interp.currentTextChanged.connect(lambda _s: self._rebuild())
        self._clamped.toggled.connect(lambda _b: self._rebuild())
        self._btn_add.clicked.connect(self.add_waypoint)
        self._btn_del.clicked.connect(self.delete_selected)
        self._btn_up.clicked.connect(lambda: self.move_selected(-1))
        self._btn_down.clicked.connect(lambda: self.move_selected(1))
        self._btn_capture.clicked.connect(self.capture_current_pose)
        self._btn_play.clicked.connect(self.play_requested)
        self._btn_pause.clicked.connect(self.pause_requested)
        self._btn_stop.clicked.connect(self.stop_requested)
        self._slider.valueChanged.connect(self._on_slider)
        self._speed.valueChanged.connect(self.speed_changed)
        self._loop.toggled.connect(self.loop_changed)

    # ----------------------------------------------------------- properties
    @property
    def trajectory(self) -> Trajectory | None:
        """Last valid trajectory (what the project stores)."""
        return self._traj

    @property
    def playable(self) -> Trajectory | None:
        """The trajectory for playback: only while the whole table is valid."""
        return self._traj if self._playable else None

    @property
    def table(self) -> QTableWidget:
        return self._table

    @property
    def info_text(self) -> str:
        return self._info.text()

    @property
    def speed(self) -> float:
        return float(self._speed.value())

    @property
    def loop(self) -> bool:
        return self._loop.isChecked()

    def transport_enabled(self) -> dict[str, bool]:
        return dict(play=self._btn_play.isEnabled(), pause=self._btn_pause.isEnabled(),
                    stop=self._btn_stop.isEnabled())

    def set_q_provider(self, fn: Callable[[], list[float]] | None) -> None:
        self._q_provider = fn

    # ---------------------------------------------------------------- model
    def set_model(self, n_dof: int, names: list[str] | None, limits) -> None:
        """Robot opened / replaced / edited (``n_dof`` 0 = closed). A trajectory of
        another DOF count is dropped with a log warning and ``trajectory_changed(None)``."""
        self._limits = (list(limits[0]), list(limits[1])) if limits else ([], [])
        if n_dof == self._n_dof:
            self._set_headers(names)
            self._rebuild()
            return
        had = self._table.rowCount() > 0
        self._n_dof = int(n_dof)
        self._set_headers(names)
        self._load_rows([])
        dropped = self._traj is not None or had
        self._traj, self._playable = None, False
        if dropped and n_dof > 0:
            self.log_requested.emit("[WARNING] Robot DOF changed: trajectory dropped.")
            self.trajectory_changed.emit(None)
        self._refresh_info([], {}, [])
        self._update_controls()

    def _set_headers(self, names: list[str] | None) -> None:
        names = names or [f"q{i + 1}" for i in range(self._n_dof)]
        self._loading = True
        try:
            self._table.setColumnCount(1 + self._n_dof)
            self._table.setHorizontalHeaderLabels(["t [s]"] + list(names)[: self._n_dof])
        finally:
            self._loading = False

    def clear(self) -> None:
        """Silent reset (no signal): project closed / another project opens."""
        self._n_dof = 0
        self._load_rows([])
        self._table.setColumnCount(1)
        self._traj, self._playable = None, False
        self._refresh_info([], {}, [])
        self._update_controls()

    def set_trajectory(self, traj: Trajectory | None) -> bool:
        """Show ``traj`` (silent). Wrong DOF count: dropped with a log warning, False."""
        if traj is None:
            self._load_rows([])
            self._traj, self._playable = None, False
            self._refresh_info([], {}, [])
            self._update_controls()
            return True
        if traj.n_dof != self._n_dof:
            self.log_requested.emit(
                f"[WARNING] Trajectory has {traj.n_dof} DOF, robot has {self._n_dof}: dropped.")
            return False
        self._quiet = True
        try:
            self._interp.setCurrentText(traj.interpolation)
            self._clamped.setChecked(traj.clamped)
        finally:
            self._quiet = False
        self._load_rows([(w.t, list(w.q)) for w in traj.waypoints])
        self._rebuild(emit=False)
        return True

    def load_trajectory(self, traj: Trajectory | None) -> bool:
        """Like ``set_trajectory`` but emits ``trajectory_changed`` for a valid trajectory."""
        if not self.set_trajectory(traj):
            return False
        if traj is not None and self._traj is not None:
            self.trajectory_changed.emit(self._traj)
        return True

    # ---------------------------------------------------------------- table
    def _load_rows(self, rows) -> None:
        self._loading = True
        try:
            self._table.setRowCount(0)
            for t, q in rows:
                self._append_row(t, q)
        finally:
            self._loading = False

    def _append_row(self, t, q) -> None:
        r = self._table.rowCount()
        self._table.insertRow(r)
        self._set_row(r, t, q)

    def _set_row(self, r: int, t, q) -> None:
        vals = [t] + list(q)
        for c in range(self._table.columnCount()):
            v = vals[c] if c < len(vals) else 0.0
            item = QTableWidgetItem(v if isinstance(v, str) else _fmt(v))
            self._table.setItem(r, c, item)

    def _row_values(self, r: int):
        out = []
        for c in range(self._table.columnCount()):
            item = self._table.item(r, c)
            try:
                v = float(item.text()) if item is not None else float("nan")
            except ValueError:
                v = float("nan")
            out.append(v)
        return out[0], out[1:]

    def set_pose_check(self, fn: Callable[[list[float]], str | None] | None) -> None:
        """U3.4: ``fn(q) -> None`` when q is free, else the collision text; Teach Add refuses."""
        self._pose_check = fn

    def _cur_q(self) -> list[float]:
        q = self._q_provider() if self._q_provider is not None else None
        q = list(q) if q is not None else [0.0] * self._n_dof
        return [round(float(x), 9) for x in q]

    def add_waypoint(self) -> None:
        """Append a row with the current q, t = last t + 1 s."""
        if self._n_dof == 0:
            return
        t = 0.0
        if self._table.rowCount():
            last, _q = self._row_values(self._table.rowCount() - 1)
            t = (last if math.isfinite(last) else 0.0) + 1.0
        self._loading = True
        try:
            self._append_row(t, self._cur_q())
        finally:
            self._loading = False
        self._rebuild()

    # ------------------------------------------------------- teach (U2.5)
    def teach_add(self) -> None:
        """Append the current q as a waypoint. Segment time = max joint distance /
        TEACH_SPEED, at least TEACH_MIN_SEGMENT. The first taught waypoint switches the
        interpolation to cubic."""
        if self._n_dof == 0:
            return
        q = self._cur_q()
        if self._pose_check is not None:
            msg = self._pose_check(q)
            if msg:
                self.log_requested.emit(f"[WARNING] Teach: pose not added, {msg}")
                return
        n = self._table.rowCount()
        t = 0.0
        if n:
            last_t, last_q = self._row_values(n - 1)
            if not math.isfinite(last_t) or any(not math.isfinite(x) for x in last_q):
                return
            dist = max((abs(a - b) for a, b in zip(q, last_q)), default=0.0)
            t = last_t + max(TEACH_MIN_SEGMENT, dist / TEACH_SPEED)
        else:
            self._interp.blockSignals(True)
            try:
                self._interp.setCurrentText("cubic")
            finally:
                self._interp.blockSignals(False)
        self._loading = True
        try:
            self._append_row(round(t, 9), q)
        finally:
            self._loading = False
        self._rebuild()

    def teach_undo(self) -> None:
        n = self._table.rowCount()
        if n == 0:
            return
        self._loading = True
        try:
            self._table.removeRow(n - 1)
        finally:
            self._loading = False
        self._rebuild()

    def teach_clear(self) -> None:
        if self._table.rowCount() == 0:
            return
        self._load_rows([])
        self._rebuild()

    def teach_play(self) -> None:
        if self._playable:
            self.play_requested.emit()

    def delete_selected(self) -> None:
        rows = sorted({i.row() for i in self._table.selectedIndexes()}, reverse=True)
        if not rows:
            return
        self._loading = True
        try:
            for r in rows:
                self._table.removeRow(r)
        finally:
            self._loading = False
        self._rebuild()

    def move_selected(self, delta: int) -> None:
        """Move the selected row one place up (-1) or down (+1), whole row."""
        rows = sorted({i.row() for i in self._table.selectedIndexes()})
        if len(rows) != 1:
            return
        r = rows[0]
        r2 = r + delta
        if not (0 <= r2 < self._table.rowCount()):
            return
        a, b = self._row_texts(r), self._row_texts(r2)
        self._loading = True
        try:
            self._set_row_text(r, b)
            self._set_row_text(r2, a)
        finally:
            self._loading = False
        self._table.selectRow(r2)
        self._rebuild()

    def _row_texts(self, r: int) -> list[str]:
        return [(self._table.item(r, c).text() if self._table.item(r, c) else "")
                for c in range(self._table.columnCount())]

    def _set_row_text(self, r: int, texts: list[str]) -> None:
        for c, s in enumerate(texts):
            self._table.setItem(r, c, QTableWidgetItem(s))

    def capture_current_pose(self) -> None:
        """Overwrite the q of the selected row with the current q."""
        rows = sorted({i.row() for i in self._table.selectedIndexes()})
        if not rows or self._n_dof == 0:
            return
        r = rows[0]
        t_text = self._table.item(r, 0).text() if self._table.item(r, 0) else _fmt(0.0)
        self._loading = True
        try:
            self._set_row(r, t_text, self._cur_q())
        finally:
            self._loading = False
        self._rebuild()

    def _on_item_changed(self, _item) -> None:
        if not self._loading:
            self._rebuild()

    # ------------------------------------------------------------- validate
    def _rebuild(self, emit: bool = True) -> None:
        if self._quiet or self._loading:
            return
        n = self._table.rowCount()
        ts, qs = [], []
        for r in range(n):
            t, q = self._row_values(r)
            ts.append(t)
            qs.append(q)
        probs = waypoint_problems(ts, qs, self._n_dof) if n else {}
        warns: list[str] = []
        valid = n >= 2 and not probs
        if valid:
            try:
                traj = Trajectory([Waypoint(t, q) for t, q in zip(ts, qs)],
                                  self._interp.currentText(), self._clamped.isChecked())
            except ValueError as exc:   # defensive: same rules as waypoint_problems
                valid = False
                probs = {0: str(exc)}
        warn_cells: set[tuple[int, int]] = set()
        if valid:
            lo, hi = self._limits
            for r, q in enumerate(qs):
                for j, x in enumerate(q):
                    if j < len(lo) and (x < lo[j] - 1e-12 or x > hi[j] + 1e-12):
                        warn_cells.add((r, j + 1))
                        warns.append(f"waypoint {r} dof {j}: {x:.4g} outside "
                                     f"[{lo[j]:.4g}, {hi[j]:.4g}]")
        self._playable = valid
        self._mark(probs, warn_cells)
        self._refresh_info(ts, probs, warns, n)
        changed = False
        if valid:
            changed = traj != self._traj
            self._traj = traj
        elif n == 0 and self._traj is not None:
            self._traj = None
            changed = True
        self._update_controls()
        if emit and changed:
            self.trajectory_changed.emit(self._traj)

    def _mark(self, probs: dict[int, str], warn_cells) -> None:
        self._loading = True
        try:
            for r in range(self._table.rowCount()):
                for c in range(self._table.columnCount()):
                    item = self._table.item(r, c)
                    if item is None:
                        continue
                    if r in probs:
                        item.setBackground(_BAD)
                        item.setForeground(_BLACK)
                        item.setToolTip(probs[r])
                    elif (r, c) in warn_cells:
                        item.setBackground(_WARN)
                        item.setForeground(_BLACK)
                        item.setToolTip("outside joint limits (warning)")
                    else:
                        item.setData(Qt.ItemDataRole.BackgroundRole, None)
                        item.setData(Qt.ItemDataRole.ForegroundRole, None)
                        item.setToolTip("")
        finally:
            self._loading = False

    def _refresh_info(self, ts, probs, warns, n: int = 0) -> None:
        lines = []
        if probs:
            lines.append("Invalid rows: " + "; ".join(
                f"{r}: {m}" for r, m in sorted(probs.items())))
        elif 0 < n < 2:
            lines.append("Need at least 2 waypoints.")
        if warns:
            lines.append(f"{len(warns)} limit warning(s): " + "; ".join(warns[:3])
                         + (" ..." if len(warns) > 3 else ""))
        self._info.setText("\n".join(lines))

    # ----------------------------------------------------------- transport
    def _update_controls(self) -> None:
        has = self._n_dof > 0
        for w in (self._btn_add, self._interp, self._clamped, self._table):
            w.setEnabled(has)
        sel = has and self._table.rowCount() > 0
        for w in (self._btn_del, self._btn_up, self._btn_down, self._btn_capture):
            w.setEnabled(sel)
        nrows = self._table.rowCount()
        self._btn_teach.setEnabled(has)
        self._btn_teach_undo.setEnabled(nrows > 0)
        self._btn_teach_clear.setEnabled(nrows > 0)
        self._teach_count.setText(f"Waypoints: {nrows}")
        T = self._traj.duration if self._traj is not None else 0.0
        can = self._playable
        self._slider.setEnabled(can)
        self._speed.setEnabled(has)
        self._loop.setEnabled(has)
        self._apply_transport()
        self._time_label.setText(f"t = {0.0:.3f} / {T:.3f} s")

    def set_transport(self, play: bool, pause: bool, stop: bool) -> None:
        self._want = (play, pause, stop)
        self._apply_transport()

    def _apply_transport(self) -> None:
        play, pause, stop = self._want
        self._btn_play.setEnabled(play and self._playable)
        self._btn_teach_play.setEnabled(play and self._playable)
        self._btn_pause.setEnabled(pause)
        self._btn_stop.setEnabled(stop)

    def set_position(self, t: float) -> None:
        """Show playback time ``t`` (no seek signal)."""
        T = self._traj.duration if self._traj is not None else 0.0
        self._quiet = True
        try:
            self._slider.setValue(int(round(SLIDER_STEPS * t / T)) if T > 0 else 0)
        finally:
            self._quiet = False
        self._time_label.setText(f"t = {t:.3f} / {T:.3f} s")

    def _on_slider(self, v: int) -> None:
        if self._quiet or self._traj is None:
            return
        t = self._traj.duration * v / SLIDER_STEPS
        self._time_label.setText(f"t = {t:.3f} / {self._traj.duration:.3f} s")
        self.seek_requested.emit(t)
