"""
app/panels/graph_panel.py
R6.6: "Graphs" dock. Plots the recorded signals (app.sim.Recorder) by graph kind, with a
joint checklist, auto-range, pause (freeze), clear and a vertical cursor linked to the
trajectory timeline. Live refresh every 100 ms while the sim is RUNNING and the dock is
visible; data are taken from the recorder buffer (views) and decimated to <= 5000 points
per series. "TCP path 3D" is not drawn in 2D: ``path_3d_changed`` carries the polyline for
the viewport.
"""

from __future__ import annotations

from typing import Any, Callable

import numpy as np
from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtWidgets import (
    QCheckBox, QComboBox, QHBoxLayout, QLabel, QListWidget, QListWidgetItem, QPushButton,
    QVBoxLayout, QWidget,
)

from app.graphs.plot_backend import PlotView
from app.graphs.series import JOINT_KINDS, KIND_LABELS, KINDS, MAX_POINTS, graph_data
from app.sim.engine import RUNNING

REFRESH_MS = 100


class GraphPanel(QWidget):
    seek_requested = Signal(float)        # user moved the cursor
    path_3d_changed = Signal(object)      # (N,3) ndarray or None (viewport polyline)
    export_requested = Signal()           # "Export graph..." (R6.7)

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self._recorder_fn: Callable[[], Any] = lambda: None
        self._names_fn: Callable[[], list[str]] = lambda: []
        self._hooked_clock: Any = None
        self.refresh_count = 0
        self.last_error: str | None = None
        self._timer = QTimer(self)
        self._timer.setInterval(REFRESH_MS)
        self._timer.timeout.connect(self._on_tick)

        lay = QVBoxLayout(self)
        row = QHBoxLayout()
        self._kind = QComboBox()
        for k in KINDS:
            self._kind.addItem(KIND_LABELS[k], k)
        self._auto = QCheckBox("Auto-range")
        self._auto.setChecked(True)
        self._pause = QCheckBox("Pause")
        self._clear_btn = QPushButton("Clear")
        self._export_btn = QPushButton("Export...")
        self._export_btn.setToolTip("Export the shown series to CSV / NPZ")
        for w in (self._kind, self._auto, self._pause, self._clear_btn, self._export_btn):
            row.addWidget(w)
        lay.addLayout(row)
        self._joints = QListWidget()
        self._joints.setMaximumHeight(80)
        self._joints.setFlow(QListWidget.Flow.LeftToRight)
        self._joints.setWrapping(True)
        lay.addWidget(self._joints)
        self.plot = PlotView(self)
        lay.addWidget(self.plot.widget, 1)
        self._info = QLabel("")
        lay.addWidget(self._info)

        self._kind.currentIndexChanged.connect(self._on_kind)
        self._joints.itemChanged.connect(lambda _i: self.refresh(force=True))
        self._auto.toggled.connect(lambda _b: self.refresh(force=True))
        self._pause.toggled.connect(self._on_pause)
        self._clear_btn.clicked.connect(self.clear)
        self._export_btn.clicked.connect(lambda _c=False: self.export_requested.emit())
        self.plot.cursor_moved.connect(self.seek_requested)
        self._on_kind()

    # ------------------------------------------------------------- source
    def set_source(self, recorder_fn: Callable[[], Any], names_fn: Callable[[], list[str]],
                   types_fn: Callable[[], list[str]] | None = None) -> None:
        """``types_fn`` (optional) gives "revolute"/"prismatic" per DOF for the axis units."""
        self._recorder_fn, self._names_fn, self._types_fn = recorder_fn, names_fn, types_fn
        self.model_changed()

    def _joint_types(self):
        fn = getattr(self, "_types_fn", None)
        return (fn() or None) if fn is not None else None

    def model_changed(self) -> None:
        """Recorder or DOF set replaced: rebuild the joint list, rehook the clock."""
        names = self._names_fn() or []
        self._joints.blockSignals(True)
        self._joints.clear()
        for n in names:
            it = QListWidgetItem(n)
            it.setFlags(it.flags() | Qt.ItemFlag.ItemIsUserCheckable)
            it.setCheckState(Qt.CheckState.Checked)
            self._joints.addItem(it)
        self._joints.blockSignals(False)
        rec = self._recorder_fn()
        clock = rec.engine.clock if rec is not None else None
        if clock is not self._hooked_clock:
            self._hooked_clock = clock
            if clock is not None:
                clock.on_state_changed.append(self._on_clock)
        self.plot.clear()
        self.path_3d_changed.emit(None)
        self._info.setText("")
        self._update_timer()

    # ----------------------------------------------------------- properties
    @property
    def timer(self) -> QTimer:
        return self._timer

    @property
    def kind(self) -> str:
        return self._kind.currentData()

    @property
    def paused(self) -> bool:
        return self._pause.isChecked()

    @property
    def info_text(self) -> str:
        return self._info.text()

    def set_kind(self, kind: str) -> None:
        i = self._kind.findData(kind)
        if i < 0:
            raise ValueError(f"unknown graph kind {kind!r}")
        self._kind.setCurrentIndex(i)

    def selected_joints(self) -> list[int]:
        return [i for i in range(self._joints.count())
                if self._joints.item(i).checkState() == Qt.CheckState.Checked]

    def set_joint_checked(self, i: int, on: bool) -> None:
        self._joints.item(i).setCheckState(Qt.CheckState.Checked if on else Qt.CheckState.Unchecked)

    # ---------------------------------------------------------------- slots
    def _on_kind(self, _i: int = 0) -> None:
        self._joints.setEnabled(self.kind in JOINT_KINDS)
        self.refresh(force=True)

    def _on_pause(self, on: bool) -> None:
        if not on:
            self.refresh(force=True)

    def _on_clock(self, new: str, old: str) -> None:
        self._update_timer()
        if not (new == RUNNING and old == RUNNING):
            self.refresh(force=True)   # run start (cleared recorder), pause, stop: final view

    def _shown(self) -> bool:
        return self.isVisible()

    def _running(self) -> bool:
        rec = self._recorder_fn()
        return rec is not None and rec.engine.clock.state == RUNNING

    def _update_timer(self) -> None:
        if self._shown() and self._running():
            if not self._timer.isActive():
                self._timer.start()
        else:
            self._timer.stop()

    def on_visibility_changed(self, visible: bool) -> None:
        """Dock visibility (a hidden tabbed dock does not always deliver hideEvent)."""
        if visible:
            self._update_timer()
            self.refresh(force=True)
        else:
            self._timer.stop()

    def showEvent(self, e) -> None:
        super().showEvent(e)
        self._update_timer()

    def hideEvent(self, e) -> None:
        self._timer.stop()
        super().hideEvent(e)

    def _on_tick(self) -> None:
        if not self._shown() or not self._running():
            self._timer.stop()
            return
        self.refresh()

    def shown_series(self):
        """Full-resolution (not decimated) series of the plotted kind, or [] (R6.7)."""
        rec = self._recorder_fn()
        if rec is None:
            return []
        return graph_data(rec.export_arrays(), self.kind, joints=self.selected_joints(),
                          names=self._names_fn() or None, max_points=10 ** 12,
                          joint_types=self._joint_types())

    # --------------------------------------------------------------- cursor
    def set_cursor(self, t: float) -> None:
        """Timeline position -> cursor (does not seek back)."""
        self.plot.set_cursor(t)

    # ----------------------------------------------------------------- draw
    def clear(self) -> None:
        """Empty the recorded data and the plot."""
        rec = self._recorder_fn()
        if rec is not None:
            rec.clear()
        self.plot.clear()
        self.path_3d_changed.emit(None)
        self._info.setText("")

    def refresh(self, force: bool = False) -> None:
        """Redraw from the recorder (UI thread; decimated). Frozen while Pause is on."""
        if self.paused and not force:
            return
        rec = self._recorder_fn()
        if rec is None:
            self.plot.clear()
            self.path_3d_changed.emit(None)
            return
        kind = self.kind
        try:
            arrays = rec.export_arrays(copy=False)
            series = graph_data(arrays, kind, joints=self.selected_joints(),
                                names=self._names_fn() or None, max_points=MAX_POINTS,
                                joint_types=self._joint_types())
            self.last_error = None
        except Exception as exc:  # noqa: BLE001  (a bad draw never breaks the UI)
            self.last_error = str(exc)
            self._info.setText(f"Graph error: {exc}")
            return
        self.refresh_count += 1
        if kind == "tcp_path_3d":
            self.plot.clear()
            n = len(series[0].t) if series else 0
            self.path_3d_changed.emit(np.column_stack([s.y for s in series]) if n >= 2 else None)
            self._info.setText(f"TCP path: {n} points (shown in the 3D viewport)")
        else:
            self.plot.set_series(series, auto_range=self._auto.isChecked())
            self.path_3d_changed.emit(None)
            self._info.setText(f"{len(rec)} samples recorded; {rec.status()}")
