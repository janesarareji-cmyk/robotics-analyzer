"""
app/panels/compare_panel.py
U2.7: "Compare" dock. Pick two or more robots (bundled samples and/or URDF files), run the
comparison in a TaskRunner worker (progress, Cancel), show a table, export it as CSV.
The numbers come from ``app.analysis.compare`` (no logic here).
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QAbstractItemView, QDoubleSpinBox, QFileDialog, QHBoxLayout, QLabel, QListWidget,
    QListWidgetItem, QProgressBar, QPushButton, QSpinBox, QTableWidget, QTableWidgetItem,
    QVBoxLayout, QWidget,
)

from app.analysis import compare as cmp_mod
from app.sample_robots import list_sample_robots
from app.task_runner import Task

MIN_ROBOTS = 2


class CompareTask(Task):
    """Load and study every selected robot in order (worker thread)."""

    def __init__(self, entries: list[tuple[str, str]], n_samples: int, voxel: float):
        super().__init__()
        self._entries = entries            # (title, urdf path)
        self._n_samples = n_samples
        self._voxel = voxel

    class _Token:
        def __init__(self, task: "CompareTask"):
            self._t = task

        def is_cancelled(self) -> bool:
            return self._t.is_cancelled

    def execute(self) -> Any:
        out = []
        n = len(self._entries)
        token = CompareTask._Token(self)
        for i, (title, path) in enumerate(self._entries):
            if self.is_cancelled:
                return None
            _name, model = cmp_mod.load_robot(path)
            res = cmp_mod.compare_robot(
                model, title, n_samples=self._n_samples, voxel_size=self._voxel, cancel=token,
                progress=lambda pct, msg, i=i: self.report_progress(
                    int((i * 100 + pct) / n), msg))
            if res.get("cancelled") or self.is_cancelled:
                return None
            out.append(res)
        return out


class ComparePanel(QWidget):
    """Dock content for the robot comparison."""

    log_requested = Signal(str)
    finished = Signal(bool)         # True = results ready, False = cancelled / failed

    def __init__(self, task_runner: Any, parent=None):
        super().__init__(parent)
        self._runner = task_runner
        self._task: CompareTask | None = None
        self._results: list[dict] = []
        self._running = False
        self._generation = 0

        layout = QVBoxLayout(self)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(4)
        layout.addWidget(QLabel("Robots to compare (tick two or more):"))

        self._list = QListWidget()
        self._list.setSelectionMode(QAbstractItemView.SelectionMode.NoSelection)
        self._list.setMaximumHeight(170)
        self._list.itemChanged.connect(lambda _i: self._update_buttons())
        layout.addWidget(self._list)
        for title, path in list_sample_robots():
            self._add_item(title, str(path), checked=False)

        row = QHBoxLayout()
        self._btn_add = QPushButton("Add URDF...")
        self._btn_add.clicked.connect(self._on_add_clicked)
        row.addWidget(self._btn_add)
        row.addStretch(1)
        layout.addLayout(row)

        opt = QHBoxLayout()
        opt.addWidget(QLabel("Samples"))
        self._samples = QSpinBox()
        self._samples.setRange(1000, 5_000_000)
        self._samples.setSingleStep(5000)
        self._samples.setValue(cmp_mod.DEFAULT_SAMPLES)
        opt.addWidget(self._samples)
        opt.addWidget(QLabel("Voxel [m]"))
        self._voxel = QDoubleSpinBox()
        self._voxel.setDecimals(3)
        self._voxel.setRange(0.005, 1.0)
        self._voxel.setSingleStep(0.01)
        self._voxel.setValue(cmp_mod.DEFAULT_VOXEL)
        opt.addWidget(self._voxel)
        opt.addStretch(1)
        layout.addLayout(opt)

        btns = QHBoxLayout()
        self._btn_run = QPushButton("Compare")
        self._btn_run.clicked.connect(self.start)
        btns.addWidget(self._btn_run)
        self._btn_cancel = QPushButton("Cancel")
        self._btn_cancel.clicked.connect(self.cancel)
        btns.addWidget(self._btn_cancel)
        self._btn_export = QPushButton("Export CSV...")
        self._btn_export.clicked.connect(self._on_export_clicked)
        btns.addWidget(self._btn_export)
        layout.addLayout(btns)

        self._bar = QProgressBar()
        self._bar.setRange(0, 100)
        layout.addWidget(self._bar)
        self._status = QLabel("")
        self._status.setWordWrap(True)
        layout.addWidget(self._status)

        self._table = QTableWidget(0, len(cmp_mod.HEADERS))
        self._table.setHorizontalHeaderLabels(cmp_mod.HEADERS)
        self._table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self._table.setMinimumHeight(140)
        layout.addWidget(self._table, stretch=1)
        self._update_buttons()

    # ------------------------------------------------------------------ public API

    @property
    def table(self) -> QTableWidget:
        return self._table

    @property
    def robot_list(self) -> QListWidget:
        return self._list

    @property
    def results(self) -> list[dict]:
        return self._results

    @property
    def is_running(self) -> bool:
        return self._running

    @property
    def status_text(self) -> str:
        return self._status.text()

    @property
    def progress_bar(self) -> QProgressBar:
        return self._bar

    @property
    def samples_spin(self) -> QSpinBox:
        return self._samples

    @property
    def voxel_spin(self) -> QDoubleSpinBox:
        return self._voxel

    @property
    def run_button(self) -> QPushButton:
        return self._btn_run

    @property
    def cancel_button(self) -> QPushButton:
        return self._btn_cancel

    @property
    def export_button(self) -> QPushButton:
        return self._btn_export

    def entries(self) -> list[tuple[str, str]]:
        """Ticked robots as (title, urdf path)."""
        out = []
        for i in range(self._list.count()):
            it = self._list.item(i)
            if it.checkState() == Qt.CheckState.Checked:
                out.append((it.text(), str(it.data(Qt.ItemDataRole.UserRole))))
        return out

    def set_checked(self, titles_or_stems) -> None:
        """Tick exactly the items whose title or URDF file stem is in ``titles_or_stems``."""
        want = {str(t).lower() for t in titles_or_stems}
        for i in range(self._list.count()):
            it = self._list.item(i)
            stem = Path(str(it.data(Qt.ItemDataRole.UserRole))).stem.lower()
            hit = it.text().lower() in want or stem in want
            it.setCheckState(Qt.CheckState.Checked if hit else Qt.CheckState.Unchecked)

    def add_urdf(self, path: str, checked: bool = True) -> None:
        p = Path(path)
        for i in range(self._list.count()):
            if str(self._list.item(i).data(Qt.ItemDataRole.UserRole)) == str(p):
                self._list.item(i).setCheckState(
                    Qt.CheckState.Checked if checked else Qt.CheckState.Unchecked)
                return
        self._add_item(p.stem, str(p), checked)

    def start(self) -> None:
        """Run the comparison of the ticked robots in a worker task. Returns at once."""
        if self._running:
            return
        entries = self.entries()
        if len(entries) < MIN_ROBOTS:
            self._status.setText(f"Tick at least {MIN_ROBOTS} robots.")
            return
        task = CompareTask(entries, int(self._samples.value()), float(self._voxel.value()))
        self._generation += 1
        gen = self._generation
        task.signals.progress.connect(lambda p, m, g=gen: self._on_progress(p, m, g))
        task.signals.result.connect(lambda res, g=gen: self._on_result(res, g))
        task.signals.error.connect(lambda msg, g=gen: self._on_error(msg, g))
        task.signals.cancelled.connect(lambda g=gen: self._on_cancelled(g))
        self._task = task
        self._running = True
        self._bar.setValue(0)
        self._status.setText("Comparing...")
        self._update_buttons()
        self._runner.submit(task)

    def cancel(self) -> None:
        if self._task is not None and self._running:
            self._task.cancel()

    def export_csv(self, path: str) -> int:
        """Write the table to ``path`` (CSV). Returns the row count."""
        if not self._results:
            raise ValueError("no results to export")
        return cmp_mod.export_csv(self._results, path)

    def shutdown(self) -> None:
        self.cancel()

    # ------------------------------------------------------------------ internals

    def _add_item(self, title: str, path: str, checked: bool) -> None:
        it = QListWidgetItem(title)
        it.setFlags(it.flags() | Qt.ItemFlag.ItemIsUserCheckable)
        it.setData(Qt.ItemDataRole.UserRole, path)
        it.setToolTip(path)
        it.setCheckState(Qt.CheckState.Checked if checked else Qt.CheckState.Unchecked)
        self._list.addItem(it)

    def _on_add_clicked(self) -> None:
        paths, _ = QFileDialog.getOpenFileNames(self, "Add URDF", "", "URDF (*.urdf)")
        for p in paths:
            self.add_urdf(p)

    def _on_export_clicked(self) -> None:
        if not self._results:
            return
        path, _ = QFileDialog.getSaveFileName(self, "Export comparison", "comparison.csv",
                                              "CSV (*.csv)")
        if path:
            try:
                n = self.export_csv(path)
                self.log_requested.emit(f"[INFO] Comparison exported: {path} ({n} robots).")
            except Exception as exc:
                self.log_requested.emit(f"[ERROR] Comparison export failed: {exc}")

    def _on_progress(self, pct: int, msg: str, gen: int) -> None:
        if gen != self._generation:
            return
        self._bar.setValue(int(pct))
        self._status.setText(msg)

    def _on_result(self, res: Any, gen: int) -> None:
        if gen != self._generation:
            return
        self._running = False
        self._results = list(res or [])
        rows = cmp_mod.table_rows(self._results)
        self._table.setRowCount(len(rows))
        for r, cells in enumerate(rows):
            for c, text in enumerate(cells):
                self._table.setItem(r, c, QTableWidgetItem(text))
        self._table.resizeColumnsToContents()
        self._bar.setValue(100)
        self._status.setText(f"Compared {len(rows)} robots (samples {self._samples.value()}, "
                             f"voxel {self._voxel.value():g} m).")
        self._update_buttons()
        self.finished.emit(True)

    def _on_error(self, msg: str, gen: int) -> None:
        if gen != self._generation:
            return
        self._running = False
        lines = msg.strip().splitlines()
        self._status.setText(f"Comparison failed: {lines[-1] if lines else 'error'}")
        self.log_requested.emit(f"[ERROR] Comparison failed: {lines[-1] if lines else 'error'}")
        self._update_buttons()
        self.finished.emit(False)

    def _on_cancelled(self, gen: int) -> None:
        if gen != self._generation:
            return
        self._running = False
        self._status.setText("Comparison cancelled.")
        self._update_buttons()
        self.finished.emit(False)

    def _update_buttons(self) -> None:
        n = len(self.entries())
        self._btn_run.setEnabled(not self._running and n >= MIN_ROBOTS)
        self._btn_cancel.setEnabled(self._running)
        self._btn_export.setEnabled(not self._running and bool(self._results))
        self._btn_add.setEnabled(not self._running)
