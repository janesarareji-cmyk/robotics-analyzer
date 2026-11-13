"""Matrices dock (U4b.5): the transforms of the arm, live (at most 30 Hz).

* Tool pose: 4x4 ``T_base_tcp`` (and ``T_world_tcp`` when the base pose is not the identity), 4 decimals,
  translation in mm or m.
* Joint transform: pick a joint; ``T_parent_child`` at the current q and the cumulative ``T_base_link``.
* Joint values (degrees for revolute, mm for prismatic joints).
* Jacobian 6 x N (world frame, toggle), manipulability w and sigma_min with a bar that turns red
  below ``SINGULAR_WARN``.
* "Pause updates" freezes the numbers; "Copy" puts everything on the clipboard as text, numpy or LaTeX.

Any q change reaches the panel through ``notify(model, fk)`` (called from the main window's robot
view update). Hidden dock: nothing is computed. Cost of one refresh: well under 2 ms.
"""
from __future__ import annotations

import time
from typing import Any

from PySide6.QtCore import QTimer, Qt
from PySide6.QtGui import QFontDatabase, QGuiApplication
from PySide6.QtWidgets import (
    QCheckBox, QComboBox, QGroupBox, QHBoxLayout, QLabel, QPlainTextEdit, QProgressBar, QPushButton,
    QVBoxLayout, QWidget,
)

from app.analysis.matrices import (
    FORMATS, MatrixData, compute_matrices, export_text, format_matrix, joint_names, pose_for_units,
    q_row,
)
from app.kin_utils import SINGULAR_WARN, CompiledCache

MIN_INTERVAL_S = 1.0 / 30.0
SIGMA_FULL = 0.30                 # sigma_min shown as a full bar


def _block() -> QPlainTextEdit:
    w = QPlainTextEdit()
    w.setReadOnly(True)
    w.setLineWrapMode(QPlainTextEdit.LineWrapMode.NoWrap)
    w.setFont(QFontDatabase.systemFont(QFontDatabase.SystemFont.FixedFont))
    w.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
    w.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
    return w


def _fit_height(w: QPlainTextEdit, lines: int) -> None:
    fm = w.fontMetrics()
    w.setFixedHeight(int(lines * fm.lineSpacing() + 2 * w.frameWidth() + 14 + 12))


class MatrixPanel(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._model: Any = None
        self._fk: Any = None
        self._cache = CompiledCache()
        self._cache_id: int | None = None
        self._data: MatrixData | None = None
        self._paused = False
        self._dirty = False
        self._last_run = 0.0
        self._active = False
        self._disposed = False
        self.last_refresh_ms = 0.0
        self.refresh_count = 0

        self._timer = QTimer(self)
        self._timer.setSingleShot(True)
        self._timer.timeout.connect(self._tick)

        lay = QVBoxLayout(self)
        lay.setContentsMargins(6, 6, 6, 6)
        lay.setSpacing(6)

        row = QHBoxLayout()
        row.addWidget(QLabel("Translation in"))
        self._units = QComboBox()
        self._units.addItems(["mm", "m"])
        self._units.currentIndexChanged.connect(lambda _i: self._draw())
        row.addWidget(self._units)
        self._chk_pause = QCheckBox("Pause updates")
        self._chk_pause.toggled.connect(self._on_pause)
        row.addWidget(self._chk_pause)
        row.addStretch(1)
        lay.addLayout(row)

        g1 = QGroupBox("Tool pose (4 x 4)")
        l1 = QVBoxLayout(g1)
        self._lbl_base = QLabel("T base to tool")
        l1.addWidget(self._lbl_base)
        self._tcp = _block()
        _fit_height(self._tcp, 4)
        l1.addWidget(self._tcp)
        self._lbl_world = QLabel("T world to tool (base pose is not the identity)")
        l1.addWidget(self._lbl_world)
        self._tcp_world = _block()
        _fit_height(self._tcp_world, 4)
        l1.addWidget(self._tcp_world)
        lay.addWidget(g1)

        g2 = QGroupBox("Joint transform")
        l2 = QVBoxLayout(g2)
        row = QHBoxLayout()
        row.addWidget(QLabel("Joint"))
        self._joint = QComboBox()
        self._joint.currentIndexChanged.connect(self._on_joint_changed)
        row.addWidget(self._joint, 1)
        l2.addLayout(row)
        l2.addWidget(QLabel("T parent to child, at the current q"))
        self._tpc = _block()
        _fit_height(self._tpc, 4)
        l2.addWidget(self._tpc)
        l2.addWidget(QLabel("T base to child link (cumulative)"))
        self._tbl = _block()
        _fit_height(self._tbl, 4)
        l2.addWidget(self._tbl)
        lay.addWidget(g2)

        g3 = QGroupBox("Joint values")
        l3 = QVBoxLayout(g3)
        self._qtext = _block()
        l3.addWidget(self._qtext)
        lay.addWidget(g3)

        g4 = QGroupBox("Manipulability")
        l4 = QVBoxLayout(g4)
        self._sigma_bar = QProgressBar()
        self._sigma_bar.setRange(0, 100)
        self._sigma_bar.setTextVisible(False)
        self._sigma_bar.setMaximumHeight(12)
        l4.addWidget(self._sigma_bar)
        self._sigma_text = QLabel("")
        self._sigma_text.setWordWrap(True)
        l4.addWidget(self._sigma_text)
        self._chk_jac = QCheckBox("Show the Jacobian (6 x N, world frame)")
        self._chk_jac.toggled.connect(self._on_jac_toggled)
        l4.addWidget(self._chk_jac)
        self._jac = _block()
        self._jac.setVisible(False)
        l4.addWidget(self._jac)
        lay.addWidget(g4)

        row = QHBoxLayout()
        row.addWidget(QLabel("Copy as"))
        self._copy_buttons: dict[str, QPushButton] = {}
        for fmt, label in (("text", "Text"), ("numpy", "numpy"), ("latex", "LaTeX")):
            b = QPushButton(label)
            b.setToolTip(f"Copy every matrix on this panel to the clipboard ({label})")
            b.clicked.connect(lambda _c=False, f=fmt: self.copy(f))
            row.addWidget(b)
            self._copy_buttons[fmt] = b
        row.addStretch(1)
        lay.addLayout(row)
        self._note = QLabel("")
        self._note.setWordWrap(True)
        lay.addWidget(self._note)
        lay.addStretch(1)
        self._draw()

    # ------------------------------------------------------------------ setup

    def set_model(self, model: Any) -> None:
        self._model = model
        self._fk = None
        self._cache.invalidate()
        self._data = None
        self._joint.blockSignals(True)
        self._joint.clear()
        if model is not None:
            self._joint.addItems(joint_names(model))
        self._joint.blockSignals(False)
        self._dirty = True
        if self._active and not self._paused:
            self.refresh_now()
        else:
            self._draw()

    def set_active(self, on: bool) -> None:
        self._active = bool(on)
        if on and self._dirty and not self._paused:
            self._schedule()

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

    shutdown = dispose

    # ------------------------------------------------------------------ state access

    @property
    def data(self) -> MatrixData | None:
        return self._data

    @property
    def paused(self) -> bool:
        return self._paused

    @property
    def units(self) -> str:
        return self._units.currentText()

    @property
    def tcp_text(self) -> str:
        return self._tcp.toPlainText()

    @property
    def world_text(self) -> str:
        return self._tcp_world.toPlainText()

    @property
    def joint_matrix_text(self) -> str:
        return self._tpc.toPlainText()

    @property
    def base_link_text(self) -> str:
        return self._tbl.toPlainText()

    @property
    def q_text(self) -> str:
        return self._qtext.toPlainText()

    @property
    def jacobian_text(self) -> str:
        return self._jac.toPlainText()

    @property
    def sigma_text(self) -> str:
        return self._sigma_text.text()

    @property
    def sigma_bar_value(self) -> int:
        return self._sigma_bar.value()

    @property
    def world_visible(self) -> bool:
        return not self._tcp_world.isHidden()

    def set_units(self, units: str) -> None:
        self._units.setCurrentText(units)

    def set_joint(self, index: int) -> None:
        self._joint.setCurrentIndex(index)

    def set_paused(self, on: bool) -> None:
        self._chk_pause.setChecked(bool(on))

    def show_jacobian(self, on: bool) -> None:
        self._chk_jac.setChecked(bool(on))

    # ------------------------------------------------------------------ refresh

    def notify(self, model: Any, fk: dict | None = None) -> None:
        """The arm moved (any source). Cheap when hidden or paused; at most 30 refreshes a second."""
        if self._disposed:
            return
        if model is not self._model:
            self.set_model(model)
        self._fk = fk
        self._dirty = True
        if not self._active or self._paused:
            return
        self._schedule()

    def _schedule(self) -> None:
        if self._timer.isActive():
            return
        wait = MIN_INTERVAL_S - (time.perf_counter() - self._last_run)
        if wait <= 0:
            self.refresh_now()
        else:
            self._timer.start(max(1, int(wait * 1000)))

    def _tick(self) -> None:
        if self._dirty and self._active and not self._paused:
            self.refresh_now()

    def refresh_now(self) -> None:
        """Compute and draw the matrices for the current q."""
        t0 = time.perf_counter()
        self._last_run = t0
        self._dirty = False
        model = self._model
        if model is None or self._disposed:
            self._data = None
            self._draw()
            return
        try:
            if self._cache_id != id(model):
                self._cache.invalidate()
                self._cache_id = id(model)
            cm = self._cache.get_cm(model)
            fk = self._fk
            self._data = compute_matrices(model, None, fk, cm, max(self._joint.currentIndex(), 0),
                                          jacobian=True)
            self._note.setText("")
        except Exception as exc:
            self._data = None
            self._note.setText(f"Cannot compute: {exc}")
        self._draw()
        self.last_refresh_ms = (time.perf_counter() - t0) * 1000.0
        self.refresh_count += 1

    def _on_joint_changed(self, _i: int) -> None:
        if self._model is None:
            return
        if self._paused and self._data is not None:          # new joint, same frozen q
            try:
                cm = self._cache.get_cm(self._model)
                self._data = compute_matrices(self._model, self._data.q, None, cm,
                                              max(self._joint.currentIndex(), 0))
            except Exception:
                pass
            self._draw()
        else:
            self.refresh_now()

    def _on_pause(self, on: bool) -> None:
        self._paused = bool(on)
        self._note.setText("Updates paused." if on else "")
        if not on:
            self._dirty = True
            if self._active:
                self.refresh_now()

    def _on_jac_toggled(self, on: bool) -> None:
        self._jac.setVisible(bool(on))
        self._draw()

    # ------------------------------------------------------------------ drawing

    def _draw(self) -> None:
        d = self._data
        units = self._units.currentText()
        if d is None:
            for w in (self._tcp, self._tcp_world, self._tpc, self._tbl, self._qtext, self._jac):
                w.setPlainText("")
            self._tcp.setPlaceholderText("No robot.")
            self._lbl_world.setVisible(False)
            self._tcp_world.setVisible(False)
            self._sigma_text.setText("")
            self._sigma_bar.setValue(0)
            return
        self._tcp.setPlainText(format_matrix(pose_for_units(d.T_base_tcp, units)))
        wv = d.T_world_tcp is not None
        self._lbl_world.setVisible(wv)
        self._tcp_world.setVisible(wv)
        if wv:
            self._tcp_world.setPlainText(format_matrix(pose_for_units(d.T_world_tcp, units)))
        if d.T_parent_child is not None:
            self._tpc.setPlainText(format_matrix(pose_for_units(d.T_parent_child, units)))
            self._tbl.setPlainText(format_matrix(pose_for_units(d.T_base_link, units)))
        else:
            self._tpc.setPlainText("")
            self._tbl.setPlainText("")
        qv = q_row(d)
        lines = [f"{n:<14.14} {v:10.4f} {'deg' if t == 'revolute' else 'mm'}"
                 for n, v, t in zip(d.dof_names, qv, d.dof_types)]
        self._qtext.setPlainText("\n".join(lines))
        _fit_height(self._qtext, max(1, min(len(lines), 8)))
        if self._chk_jac.isChecked() and d.jacobian is not None:
            self._jac.setPlainText(format_matrix(d.jacobian))
            _fit_height(self._jac, 6)
        near = d.sigma_min < SINGULAR_WARN
        self._sigma_text.setText(
            f"w = {d.manipulability:.4g}    sigma_min = {d.sigma_min:.4f}"
            + ("    NEAR A SINGULARITY" if near else ""))
        self._sigma_text.setStyleSheet("color: #ff6b6b; font-weight: bold;" if near else "")
        self._sigma_bar.setValue(int(round(100 * min(max(d.sigma_min / SIGMA_FULL, 0.0), 1.0))))
        self._sigma_bar.setStyleSheet(
            "QProgressBar{border:1px solid #666;border-radius:2px;background:#222}"
            "QProgressBar::chunk{background:" + ("#e04040" if near else "#4caf50") + "}")

    # ------------------------------------------------------------------ copy

    def export(self, fmt: str = "text") -> str:
        """Every matrix of the panel in ``fmt`` (text, numpy, latex). Empty without a robot."""
        if fmt not in FORMATS:
            raise ValueError(f"fmt must be one of {FORMATS}")
        if self._data is None:
            return ""
        return export_text(self._data, fmt, self.units, 4, with_jacobian=True)

    def copy(self, fmt: str = "text") -> str:
        text = self.export(fmt)
        if text:
            cb = QGuiApplication.clipboard()
            if cb is not None:
                cb.setText(text)
            self._note.setText(f"Copied as {fmt}.")
        return text
