"""U6.5: "Playback video" dialog: format, frame rate, size, file; progress and Cancel.

The export itself is ``app.sim.video_export.VideoExporter`` (frames on the GUI thread, encoding in a
worker). The dialog only collects the settings and shows the progress.
"""
from __future__ import annotations

from pathlib import Path
from typing import Any, Callable

from PySide6.QtWidgets import (
    QComboBox, QDialog, QDialogButtonBox, QFileDialog, QFormLayout, QHBoxLayout, QLabel, QLineEdit,
    QProgressBar, QPushButton, QSpinBox, QVBoxLayout,
)

from app.sim import video_export as ve
from app.theme import MUTED


class VideoExportDialog(QDialog):
    def __init__(self, exporter: ve.VideoExporter, duration: float,
                 make_grab: Callable[[tuple | None], Callable], seek_fn: Callable[[float], None],
                 default_dir: str = "", parent=None):
        super().__init__(parent)
        self.setWindowTitle("Export playback video")
        self.setObjectName("video_dialog")
        self._exporter = exporter
        self._duration = float(duration)
        self._make_grab = make_grab
        self._seek = seek_fn
        self._default_dir = default_dir
        self.result_path = ""

        lay = QVBoxLayout(self)
        form = QFormLayout()
        self.format = QComboBox()
        for key, label in ve.available_formats().items():
            self.format.addItem(label, key)
        self.format.setAccessibleName("Video format")
        form.addRow("Format", self.format)
        self.fps = QSpinBox()
        self.fps.setRange(1, 60)
        self.fps.setValue(ve.DEFAULT_FPS)
        self.fps.setSuffix(" fps")
        self.fps.setAccessibleName("Frames per second")
        form.addRow("Frame rate", self.fps)
        self.size = QComboBox()
        self.size.addItems(list(ve.RESOLUTIONS))
        self.size.setAccessibleName("Video size")
        self.size.setToolTip("Viewport size keeps the joint load bars; the other sizes scale the view")
        form.addRow("Size", self.size)
        row = QHBoxLayout()
        self.path = QLineEdit()
        self.path.setAccessibleName("Output file")
        self.browse = QPushButton("Browse...")
        row.addWidget(self.path, 1)
        row.addWidget(self.browse)
        form.addRow("Save as", row)
        lay.addLayout(form)

        self.info = QLabel("")
        self.info.setWordWrap(True)
        self.info.setStyleSheet(f"color: {MUTED};")
        lay.addWidget(self.info)
        self.bar = QProgressBar()
        self.bar.setRange(0, 100)
        self.bar.setTextVisible(True)
        self.bar.hide()
        lay.addWidget(self.bar)
        self.status = QLabel("")
        self.status.setWordWrap(True)
        lay.addWidget(self.status)
        self.buttons = QDialogButtonBox()
        self.start_btn = self.buttons.addButton("Export", QDialogButtonBox.ButtonRole.AcceptRole)
        self.cancel_btn = self.buttons.addButton("Cancel", QDialogButtonBox.ButtonRole.RejectRole)
        lay.addWidget(self.buttons)

        self.browse.clicked.connect(self._on_browse)
        self.start_btn.clicked.connect(self.start)
        self.cancel_btn.clicked.connect(self._on_cancel)
        self.fps.valueChanged.connect(self._update_info)
        self.format.currentIndexChanged.connect(self._on_format)
        exporter.progress.connect(self._on_progress)
        exporter.finished.connect(self._on_finished)
        exporter.failed.connect(self._on_failed)
        exporter.cancelled.connect(self._on_cancelled)
        self._on_format()
        self._update_info()

    # ------------------------------------------------------------------
    @property
    def fmt(self) -> str:
        return str(self.format.currentData())

    def _update_info(self) -> None:
        n = ve.frame_count(self._duration, self.fps.value())
        self.info.setText(f"{self._duration:.2f} s of motion, {n} frames. Overlays are drawn as they "
                          "are shown now.")

    def _on_format(self) -> None:
        name = "playback" + ("_frames" if self.fmt == "png" else "")
        base = Path(self._default_dir or ".") / name
        self.path.setText(ve.normalize_path(str(base), self.fmt))

    def _on_browse(self) -> None:
        if self.fmt == "png":
            p = QFileDialog.getExistingDirectory(self, "Folder for the PNG frames", self._default_dir)
        else:
            ext = "mp4" if self.fmt == "mp4" else "gif"
            p, _ = QFileDialog.getSaveFileName(self, "Save video", self.path.text(), f"*.{ext}")
        if p:
            self.path.setText(ve.normalize_path(p, self.fmt))

    def start(self) -> None:
        if self._exporter.running:
            return
        path = ve.normalize_path(self.path.text().strip(), self.fmt)
        if not path:
            self.status.setText("Choose a file name.")
            return
        size = ve.RESOLUTIONS[self.size.currentText()]
        self.set_busy(True)
        self.status.setText("Rendering frames...")
        try:
            self._exporter.start(path, self.fmt, self.fps.value(), self._duration,
                                 self._make_grab(size), self._seek)
        except Exception as e:
            self.set_busy(False)
            self.status.setText(f"Cannot start: {e}")

    def set_busy(self, busy: bool) -> None:
        for w in (self.format, self.fps, self.size, self.path, self.browse, self.start_btn):
            w.setEnabled(not busy)
        self.bar.setVisible(busy)
        if busy:
            self.bar.setValue(0)

    def _on_cancel(self) -> None:
        if self._exporter.running:
            self.status.setText("Cancelling...")
            self._exporter.cancel()
        else:
            self.reject()

    def _on_progress(self, i: int, n: int) -> None:
        self.bar.setValue(int(100 * i / max(n, 1)))
        self.status.setText(f"Frame {i} of {n}")

    def _on_finished(self, path: str, n: int) -> None:
        self.result_path = path
        self.set_busy(False)
        self.status.setText(f"Saved {n} frames: {path}")
        self.cancel_btn.setText("Close")

    def _on_failed(self, msg: str) -> None:
        self.set_busy(False)
        self.status.setText(f"Failed: {msg}")

    def _on_cancelled(self) -> None:
        self.set_busy(False)
        self.status.setText("Cancelled. No file was kept.")

    def done(self, r: int) -> None:
        if self._exporter.running:
            self._exporter.cancel()
        for sig, slot in ((self._exporter.progress, self._on_progress),
                          (self._exporter.finished, self._on_finished),
                          (self._exporter.failed, self._on_failed),
                          (self._exporter.cancelled, self._on_cancelled)):
            try:
                sig.disconnect(slot)
            except (RuntimeError, TypeError):
                pass
        super().done(r)
