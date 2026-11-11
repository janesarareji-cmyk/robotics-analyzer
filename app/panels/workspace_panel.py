"""
app/panels/workspace_panel.py
R5.5: "Workspace" dock. Runs a workspace study (random joint sampling -> reachable TCP
points / occupied voxels) through a compute backend in a TaskRunner worker, with a
progress bar, Cancel, and a status line. The viewport is updated through the
`display_changed` signal (MainWindow forwards it to RobotView).
"""

from __future__ import annotations

from typing import Any

import numpy as np
from PySide6.QtCore import Signal
from PySide6.QtWidgets import (
    QCheckBox, QComboBox, QFormLayout, QGroupBox, QHBoxLayout, QLabel, QProgressBar,
    QPushButton, QSpinBox, QVBoxLayout, QWidget,
)

import core_bridge as cb
from app.compute import last_run
from app.compute.backend import PREFERENCES, estimate_reach, get_backend
from app.compute.budget import MAX_VRAM_BUDGET_MB, MIN_VRAM_BUDGET_MB, vram_budget_bytes
from app.logger import log
from app.panels.num_field import NumField
from app.settings import AppSettings

DEFAULT_SAMPLES = 1_000_000
MAX_SAMPLES = 2_000_000_000
MAX_POINTS = 2_000_000


class WorkspacePanel(QWidget):
    """Dock content for workspace sampling."""

    display_changed = Signal(object)   # dict(points, centers, voxel_size, base) or None
    log_requested = Signal(str)
    _progress = Signal(object, object)  # (done, total), emitted from the worker thread

    def __init__(self, task_runner: Any, settings: AppSettings | None = None,
                 backend_factory: Any = None, parent=None):
        super().__init__(parent)
        self._runner = task_runner
        self._settings = settings if settings is not None else AppSettings()
        self._backend_factory = backend_factory
        self._model: Any = None
        self._generation = 0
        self._running = False
        self._task: Any = None
        self._token: Any = None
        self._result: dict | None = None
        self._base: np.ndarray = np.eye(4)
        self._voxel_edited = False
        self.last_info: dict | None = None
        self.last_backend_name = ""
        self.last_options: dict | None = None

        layout = QVBoxLayout(self)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(4)

        form = QFormLayout()
        self._samples = QSpinBox()
        self._samples.setRange(1000, MAX_SAMPLES)
        self._samples.setSingleStep(100_000)
        self._samples.setValue(DEFAULT_SAMPLES)
        form.addRow("Samples", self._samples)
        self._voxel = NumField(0.01)
        self._voxel.value_changed.connect(lambda _v: setattr(self, "_voxel_edited", True))
        form.addRow("Voxel size [m]", self._voxel)
        self._seed = QSpinBox()
        self._seed.setRange(0, 2_000_000_000)
        self._seed.setValue(42)
        form.addRow("Seed", self._seed)
        self._backend = QComboBox()
        self._backend.addItems(list(PREFERENCES))
        self._backend.setCurrentText(self._settings.compute_backend())
        self._backend.currentTextChanged.connect(self._on_backend_changed)
        form.addRow("Backend", self._backend)
        layout.addLayout(form)

        show = QHBoxLayout()
        self._chk_points = QCheckBox("Show points")
        self._chk_points.setChecked(True)
        self._chk_voxels = QCheckBox("Show voxels")
        self._chk_voxels.setChecked(False)
        self._chk_points.toggled.connect(self._emit_display)
        self._chk_voxels.toggled.connect(self._emit_display)
        show.addWidget(self._chk_points)
        show.addWidget(self._chk_voxels)
        show.addStretch(1)
        layout.addLayout(show)

        row = QHBoxLayout()
        self._btn_run = QPushButton("Run")
        self._btn_run.clicked.connect(self.run)
        self._btn_cancel = QPushButton("Cancel")
        self._btn_cancel.clicked.connect(self.cancel)
        row.addWidget(self._btn_run)
        row.addWidget(self._btn_cancel)
        row.addStretch(1)
        layout.addLayout(row)

        self._bar = QProgressBar()
        self._bar.setRange(0, 100)
        self._bar.setValue(0)
        layout.addWidget(self._bar)
        self._status = QLabel("")
        self._status.setWordWrap(True)
        layout.addWidget(self._status)

        box = QGroupBox("Compute")
        bl = QFormLayout(box)
        self._vram = QSpinBox()
        self._vram.setRange(MIN_VRAM_BUDGET_MB, MAX_VRAM_BUDGET_MB)
        self._vram.setSuffix(" MiB")
        self._vram.setValue(self._settings.vram_budget_mb())
        self._vram.editingFinished.connect(self._on_vram_changed)
        bl.addRow("VRAM budget", self._vram)
        layout.addWidget(box)
        layout.addStretch(1)

        self._progress.connect(self._on_progress)
        self._update_buttons()

    # ------------------------------------------------------------------
    # State
    # ------------------------------------------------------------------

    @property
    def is_running(self) -> bool:
        return self._running

    @property
    def result(self) -> dict | None:
        return self._result

    @property
    def progress_value(self) -> int:
        return self._bar.value()

    @property
    def status_text(self) -> str:
        return self._status.text()

    def set_model(self, model: Any) -> None:
        """Robot replaced or edited (or None on close): cancel, clear results."""
        self._generation += 1
        if self._token is not None:
            self._token.cancel()
        self._running = False
        self._task = None
        self._token = None
        self._result = None
        self._bar.setValue(0)
        self._status.setText("")
        usable = model is not None and len(model.joints) > 0 and model.dof() > 0
        self._model = model if usable else None
        if self._model is not None:
            try:
                if not self._voxel_edited:
                    reach = estimate_reach(self._model)
                    self._voxel.set_value(max(reach / 100.0, 1e-4))
                    self._voxel_edited = False
            except Exception:
                pass
        self.display_changed.emit(None)
        self._update_buttons()

    def _update_buttons(self) -> None:
        self._btn_run.setEnabled(self._model is not None and not self._running)
        self._btn_cancel.setEnabled(self._running)

    def _on_backend_changed(self, text: str) -> None:
        self._settings.set_compute_backend(text)

    def _on_vram_changed(self) -> None:
        self._settings.set_vram_budget_mb(self._vram.value())

    # ------------------------------------------------------------------
    # Run / cancel
    # ------------------------------------------------------------------

    def _make_backend(self):
        if self._backend_factory is not None:
            return self._backend_factory(self._backend.currentText())
        return get_backend(self._backend.currentText(), self._settings)

    def options(self) -> dict:
        opt = dict(
            mode="random", n_samples=int(self._samples.value()), seed=int(self._seed.value()),
            voxel_size=float(self._voxel.value()), want_counts=False,
            want_points=True, max_points=MAX_POINTS,
            budget_bytes=int(vram_budget_bytes(self._settings)),
        )
        return opt

    def run(self) -> None:
        """Start the study in a worker. Returns at once (the UI thread is not blocked)."""
        if self._model is None or self._running:
            return
        try:
            backend = self._make_backend()
            opts = self.options()
            self._base = np.asarray(cb.compile_arrays(self._model)["base"], dtype=float).reshape(4, 4)
        except Exception as e:
            self._status.setText(f"Cannot start: {e}")
            return
        model = self._model
        token = cb.CancelToken()
        gen = self._generation
        self._token = token
        self._running = True
        self._result = None
        self._bar.setValue(0)
        self._status.setText(f"Running on {backend.name} ({backend.reason})...")
        self.last_options = opts
        self.log_requested.emit(
            f"[INFO] Workspace run: {opts['n_samples']} samples, voxel {opts['voxel_size']:.4g} m, "
            f"backend {backend.name} ({backend.reason}).")

        def progress(done, total):
            if gen == self._generation:
                self._progress.emit(done, total)

        def work():
            return backend.sample_workspace(model, opts, token, progress)

        task = self._runner.submit_fn(work)
        self._task = task
        task.signals.result.connect(lambda r, g=gen, b=backend: self._on_result(g, b, r))
        task.signals.error.connect(lambda m, g=gen: self._on_error(g, m))
        self._update_buttons()

    def cancel(self) -> None:
        if self._token is not None and self._running:
            self._token.cancel()
            self._status.setText("Cancelling...")

    def _on_progress(self, done, total) -> None:
        if not self._running:
            return
        pct = int(100.0 * float(done) / float(total)) if total else 0
        self._bar.setValue(max(0, min(100, pct)))

    def _on_error(self, gen: int, message: str) -> None:
        if gen != self._generation:
            return
        self._running = False
        self._token = None
        last = message.strip().splitlines()[-1] if message.strip() else "error"
        self._status.setText(f"Failed: {last}")
        self.log_requested.emit(f"[ERROR] Workspace run failed: {last}")
        self._update_buttons()

    def _on_result(self, gen: int, backend: Any, res: dict) -> None:
        if gen != self._generation:      # robot changed or project closed meanwhile
            return
        self._running = False
        self._token = None
        self._result = res
        info = res.get("info", {})
        self.last_info = info
        self.last_backend_name = backend.name
        if not res.get("cancelled"):
            last_run.set_last_run(backend.name, self.last_options, info)
        done = int(res.get("n_samples_done", 0))
        total = int(res.get("n_samples", 0))
        if res.get("cancelled"):
            self._status.setText(f"Cancelled after {done} of {total} samples.")
        else:
            self._bar.setValue(100)
            ms = float(info.get("total_ms", 0.0))
            rate = done / (ms / 1000.0) if ms > 0 else 0.0
            peak = float(info.get("vram_peak_bytes", 0)) / (1024 * 1024)
            text = (f"{done} samples, {rate / 1e6:.2f} M samples/s, backend {info.get('backend')}, "
                    f"VRAM peak {peak:.0f} MiB, {len(res['occupied_voxels'])} voxels")
            if info.get("fallback"):
                text += f", FALLBACK to CPU: {info.get('fallback_reason')}"
            self._status.setText(text)
            self.log_requested.emit(
                f"[INFO] Workspace done: {text}; chunk {info.get('chunk')}, chunks {info.get('chunks')}.")
        if info.get("fallback"):
            log.warning("workspace fell back to the CPU: %s", info.get("fallback_reason"))
        self._emit_display()
        self._update_buttons()

    # ------------------------------------------------------------------
    # Viewport output
    # ------------------------------------------------------------------

    def _emit_display(self, *_args) -> None:
        res = self._result
        if res is None:
            self.display_changed.emit(None)
            return
        points = res.get("points") if self._chk_points.isChecked() else None
        centers = None
        vs = float(res["voxel_size"])
        if self._chk_voxels.isChecked() and len(res["occupied_voxels"]):
            idx = np.asarray(res["occupied_voxels"], dtype=np.float64)
            centers = np.asarray(res["origin"], dtype=np.float64) + (idx + 0.5) * vs
        self.display_changed.emit(dict(points=points, centers=centers, voxel_size=vs,
                                       base=self._base))
