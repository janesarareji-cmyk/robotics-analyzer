"""
app/panels/performance_panel.py
R5.6: "Performance" dock. Device summary, live VRAM (1 s timer, only while visible),
last workspace run (from app.compute.last_run), and a benchmark (CPU vs backend).
"""

from __future__ import annotations

import threading
from typing import Any

from PySide6.QtCore import QTimer
from PySide6.QtWidgets import (
    QFileDialog, QFormLayout, QGroupBox, QLabel, QPushButton, QVBoxLayout, QWidget,
)

from app.compute import last_run
from app.compute.backend import get_backend
from app.compute.benchmark import run_benchmark, write_benchmark
from app.compute.budget import vram_budget_bytes
from app.settings import AppSettings

BENCH_SAMPLES = 1_000_000
REFRESH_MS = 1000
_MIB = 1024 * 1024


def _fmt_rate(r) -> str:
    return "-" if not r else f"{r / 1e6:.2f} M/s"


class PerformancePanel(QWidget):
    def __init__(self, task_runner: Any, settings: AppSettings | None = None,
                 backend_factory: Any = None, parent=None):
        super().__init__(parent)
        self._runner = task_runner
        self._settings = settings if settings is not None else AppSettings()
        self._backend_factory = backend_factory
        self._model: Any = None
        self._running = False
        self._task: Any = None
        self._generation = 0
        self.last_result: dict | None = None
        self.bench_samples = BENCH_SAMPLES

        layout = QVBoxLayout(self)
        layout.setContentsMargins(6, 6, 6, 6)
        self._device = QLabel("")
        self._device.setWordWrap(True)
        layout.addWidget(self._device)

        box = QGroupBox("Memory")
        f = QFormLayout(box)
        self._vram = QLabel("n/a")
        self._budget = QLabel("")
        f.addRow("VRAM used / free / total", self._vram)
        f.addRow("Budget", self._budget)
        layout.addWidget(box)

        box2 = QGroupBox("Last workspace run")
        f2 = QFormLayout(box2)
        self._last_settings = QLabel("no run yet")
        self._last_settings.setWordWrap(True)
        self._last_peak = QLabel("-")
        f2.addRow("Settings", self._last_settings)
        f2.addRow("VRAM peak", self._last_peak)
        layout.addWidget(box2)

        box3 = QGroupBox("Benchmark")
        f3 = QFormLayout(box3)
        self._rows = {k: QLabel("-") for k in ("fk", "workspace", "ik")}
        for k in self._rows.values():
            k.setWordWrap(True)
        f3.addRow("FK", self._rows["fk"])
        f3.addRow("Workspace", self._rows["workspace"])
        f3.addRow("Batch IK (CPU)", self._rows["ik"])
        self._bench_status = QLabel("")
        self._bench_status.setWordWrap(True)
        f3.addRow(self._bench_status)
        self._btn_run = QPushButton("Run benchmark")
        self._btn_run.clicked.connect(self.run_benchmark)
        self._btn_save = QPushButton("Save benchmark...")
        self._btn_save.clicked.connect(self.save_benchmark)
        f3.addRow(self._btn_run)
        f3.addRow(self._btn_save)
        layout.addWidget(box3)
        layout.addStretch(1)

        self._timer = QTimer(self)
        self._timer.setInterval(REFRESH_MS)
        self._timer.timeout.connect(self.refresh)
        self.refresh()
        self._update_buttons()

    # ------------------------------------------------------------------
    @property
    def timer_active(self) -> bool:
        return self._timer.isActive()

    @property
    def is_running(self) -> bool:
        return self._running

    @property
    def device_text(self) -> str:
        return self._device.text()

    @property
    def vram_text(self) -> str:
        return self._vram.text()

    @property
    def last_settings_text(self) -> str:
        return self._last_settings.text()

    @property
    def last_peak_text(self) -> str:
        return self._last_peak.text()

    @property
    def fk_text(self) -> str:
        return self._rows["fk"].text()

    def _backend(self):
        if self._backend_factory is not None:
            return self._backend_factory()
        return get_backend(None, self._settings)

    def set_model(self, model: Any) -> None:
        self._generation += 1
        self._running = False
        usable = model is not None and len(model.joints) > 0 and model.dof() > 0
        self._model = model if usable else None
        self._update_buttons()

    def _update_buttons(self) -> None:
        self._btn_run.setEnabled(self._model is not None and not self._running)
        self._btn_save.setEnabled(self.last_result is not None)

    # Timer only while visible --------------------------------------------
    def showEvent(self, e) -> None:
        super().showEvent(e)
        self.refresh()
        self._timer.start()

    def hideEvent(self, e) -> None:
        self._timer.stop()
        super().hideEvent(e)

    def refresh(self) -> None:
        """Update device summary, VRAM line and last-run values."""
        try:
            be = self._backend()
            d = be.device_info()
        except Exception as ex:
            be, d = None, {"available": False, "name": "unknown", "reason": str(ex)}
        if d.get("available"):
            self._device.setText(
                f"{d.get('name')}  SM {d.get('sm_major')}.{d.get('sm_minor')}, "
                f"driver {d.get('driver_version')}, runtime {d.get('runtime_version')}; "
                f"backend {be.name} ({be.reason})")
            try:
                m = be.mem_info() if hasattr(be, "mem_info") else {}
                tot, free = int(m.get("total_bytes", 0)), int(m.get("free_bytes", 0))
                if m.get("ok", True) and tot:
                    self._vram.setText(f"{(tot - free) // _MIB} / {free // _MIB} / {tot // _MIB} MiB")
                else:
                    self._vram.setText("n/a")
            except Exception:
                self._vram.setText("n/a")
        else:
            name = be.name if be is not None else "cpu"
            why = f" ({be.reason})" if be is not None and be.reason else ""
            self._device.setText(f"No GPU in use. Backend {name}{why}; "
                                 f"{d.get('threads', '?')} CPU threads")
            self._vram.setText("n/a")
        try:
            self._budget.setText(f"{vram_budget_bytes(self._settings) // _MIB} MiB")
        except Exception:
            self._budget.setText("")
        lr = last_run.get_last_run()
        if lr is None:
            self._last_settings.setText("no run yet")
            self._last_peak.setText("-")
        else:
            o, i = lr["options"], lr["info"]
            self._last_settings.setText(
                f"backend {lr['backend']}, {o.get('n_samples', '?')} samples, voxel "
                f"{float(o.get('voxel_size', 0)):.4g} m, chunk {i.get('chunk', '-')}, "
                f"chunks {i.get('chunks', '-')}")
            self._last_peak.setText(f"{float(i.get('vram_peak_bytes', 0)) / _MIB:.0f} MiB")

    # Benchmark -----------------------------------------------------------
    def run_benchmark(self) -> None:
        if self._model is None or self._running:
            return
        try:
            backend = self._backend()
            budget = int(vram_budget_bytes(self._settings))
        except Exception as e:
            self._bench_status.setText(f"Cannot start: {e}")
            return
        model, n, gen = self._model, int(self.bench_samples), self._generation
        self._running = True
        self._bench_status.setText(f"Running benchmark on {backend.name} and cpu...")
        self._update_buttons()
        gate = threading.Event()   # a fast task must not finish before the signals are connected

        def work():
            gate.wait(5.0)
            return run_benchmark(backend, model, n, budget_bytes=budget)

        task = self._runner.submit_fn(work)
        self._task = task   # keep the task (and its signals) alive until the result is delivered
        task.signals.result.connect(lambda r, g=gen: self._on_result(g, r))
        task.signals.error.connect(lambda m, g=gen: self._on_error(g, m))
        gate.set()

    def _on_error(self, gen: int, message: str) -> None:
        if gen != self._generation:
            return
        self._running = False
        last = message.strip().splitlines()[-1] if message.strip() else "error"
        self._bench_status.setText(f"Failed: {last}")
        self._update_buttons()

    def _on_result(self, gen: int, res: dict) -> None:
        if gen != self._generation:
            return
        self._running = False
        self.last_result = res
        for k, lab in self._rows.items():
            r = res[k]
            text = f"CPU {_fmt_rate(r['cpu']['samples_per_s'])}"
            if r["backend"] is not None and r["speedup"]:
                text += (f"  |  {res['backend']} {_fmt_rate(r['backend']['samples_per_s'])}"
                         f"  |  speedup {r['speedup']:.2f}x")
            lab.setText(text)
        self._bench_status.setText(f"Done ({res['n']} samples, IK {res['n_ik']}).")
        self._update_buttons()

    def save_benchmark(self, path: str | None = None) -> None:
        if self.last_result is None:
            return
        if not isinstance(path, str) or not path:   # clicked() passes a bool
            path, _ = QFileDialog.getSaveFileName(self, "Save benchmark", "benchmark.json",
                                                  "JSON (*.json)")
        if not path:
            return
        write_benchmark(self.last_result, path)
        self._bench_status.setText(f"Saved {path}")
