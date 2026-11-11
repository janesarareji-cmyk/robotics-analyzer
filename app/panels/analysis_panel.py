"""
app/panels/analysis_panel.py
R4.9: "Analysis" dock. Singularity indicators of the TCP Jacobian, inverse
dynamics torque table with torque limits, and energies at the current q.
Gravity is read from the project (SimulationSettings.gravity); never edited here.
"""

from __future__ import annotations

from typing import Any, Callable, Optional, Sequence

import core_bridge as cb
from PySide6.QtGui import QColor
from PySide6.QtWidgets import (
    QAbstractItemView, QComboBox, QGridLayout, QGroupBox, QHBoxLayout, QLabel,
    QPushButton, QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget,
)

from app.dyn_utils import check_torque_limits
from app.kin_utils import CompiledCache, current_q, singularity_at
from app.panels.num_field import NumField

TORQUE_HEADERS = ["DOF", "Joint", "Gravity", "Coriolis", "M*qdd", "Total", "Limit", "Ratio"]
MASS_WARNING = "mass not set"
_RED = QColor("#ff5555")
_OK_STYLE = "color: #55cc77;"
_BAD_STYLE = "color: #ff5555; font-weight: bold;"


def mass_not_set(model: Any) -> bool:
    """True when a non-root link under a non-fixed joint has mass 0."""
    try:
        pairs = cb.effective_links(model)
    except Exception:
        return False
    for j, (_parent, child) in zip(model.joints, pairs):
        if j.type == cb.JointType.Fixed:
            continue
        if 0 <= child < len(model.links) and model.links[child].inertia.mass <= 0.0:
            return True
    return False


class AnalysisPanel(QWidget):
    """Dock content: singularity, dynamics, energy."""

    def __init__(self, gravity_fn: Optional[Callable[[], Sequence[float]]] = None,
                 parent=None):
        super().__init__(parent)
        self._model: Any = None
        self._gravity_fn = gravity_fn
        self._cache = CompiledCache()
        self._n_dof = 0
        self._last_tau: list[float] = []

        root = QVBoxLayout(self)
        root.setContentsMargins(6, 6, 6, 6)
        root.setSpacing(6)

        # ---- singularity
        sg = QGroupBox("Jacobian / singularity")
        g = QGridLayout(sg)
        self._mode = QComboBox()
        self._mode.addItems(["full", "position", "angular"])
        self._mode.currentIndexChanged.connect(lambda _i: self.refresh())
        self._threshold = NumField(1e-3)
        self._threshold.value_changed.connect(lambda _v: self.refresh())
        g.addWidget(QLabel("Mode"), 0, 0)
        g.addWidget(self._mode, 0, 1)
        g.addWidget(QLabel("Threshold"), 0, 2)
        g.addWidget(self._threshold, 0, 3)
        self._lbl_smin = QLabel("-")
        self._lbl_smax = QLabel("-")
        self._lbl_cond = QLabel("-")
        self._lbl_manip = QLabel("-")
        for r, (name, lbl) in enumerate((("sigma_min", self._lbl_smin),
                                         ("sigma_max", self._lbl_smax),
                                         ("condition", self._lbl_cond),
                                         ("manipulability", self._lbl_manip)), start=1):
            g.addWidget(QLabel(name), r, 0)
            g.addWidget(lbl, r, 1)
        self._lbl_near = QLabel("-")
        g.addWidget(self._lbl_near, 1, 2, 1, 2)
        root.addWidget(sg)

        # ---- dynamics
        dg = QGroupBox("Dynamics")
        dv = QVBoxLayout(dg)
        self._lbl_gravity = QLabel("")
        dv.addWidget(self._lbl_gravity)
        self._mass_warning = QLabel(f"Warning: {MASS_WARNING}")
        self._mass_warning.setStyleSheet(_BAD_STYLE)
        self._mass_warning.setVisible(False)
        dv.addWidget(self._mass_warning)
        self._qd_box = QHBoxLayout()
        self._qd_fields: list[NumField] = []
        self._qdd_fields: list[NumField] = []
        self._qd_row = QWidget()
        self._qd_grid = QGridLayout(self._qd_row)
        self._qd_grid.setContentsMargins(0, 0, 0, 0)
        dv.addWidget(self._qd_row)
        self._table = QTableWidget(0, len(TORQUE_HEADERS))
        self._table.setHorizontalHeaderLabels(TORQUE_HEADERS)
        self._table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        dv.addWidget(self._table)
        self._btn_fd = QPushButton("Forward dynamics (qdd for zero torque)")
        self._btn_fd.clicked.connect(self.forward_dynamics)
        dv.addWidget(self._btn_fd)
        self._lbl_dyn_status = QLabel("")
        dv.addWidget(self._lbl_dyn_status)
        root.addWidget(dg, stretch=1)

        # ---- energy
        eg = QGroupBox("Energy")
        el = QGridLayout(eg)
        self._lbl_ke = QLabel("-")
        self._lbl_pe = QLabel("-")
        self._lbl_te = QLabel("-")
        for r, (name, lbl) in enumerate((("KE [J]", self._lbl_ke), ("PE [J]", self._lbl_pe),
                                         ("Total [J]", self._lbl_te))):
            el.addWidget(QLabel(name), r, 0)
            el.addWidget(lbl, r, 1)
        root.addWidget(eg)

        self._btn_fd.setEnabled(False)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    @property
    def cache(self) -> CompiledCache:
        return self._cache

    @property
    def mode_combo(self) -> QComboBox:
        return self._mode

    @property
    def threshold_field(self) -> NumField:
        return self._threshold

    @property
    def qd_fields(self) -> list[NumField]:
        return self._qd_fields

    @property
    def qdd_fields(self) -> list[NumField]:
        return self._qdd_fields

    @property
    def torque_table(self) -> QTableWidget:
        return self._table

    @property
    def mass_warning_visible(self) -> bool:
        return not self._mass_warning.isHidden()

    @property
    def mass_warning_text(self) -> str:
        return self._mass_warning.text()

    @property
    def forward_dynamics_enabled(self) -> bool:
        return self._btn_fd.isEnabled()

    @property
    def forward_dynamics_button(self) -> QPushButton:
        return self._btn_fd

    @property
    def near_singular_text(self) -> str:
        return self._lbl_near.text()

    @property
    def sigma_min_text(self) -> str:
        return self._lbl_smin.text()

    @property
    def condition_text(self) -> str:
        return self._lbl_cond.text()

    @property
    def manipulability_text(self) -> str:
        return self._lbl_manip.text()

    @property
    def energy_texts(self) -> tuple[str, str, str]:
        return self._lbl_ke.text(), self._lbl_pe.text(), self._lbl_te.text()

    @property
    def tau(self) -> list[float]:
        """Total torque per DOF shown in the table (empty when cleared)."""
        return list(self._last_tau)

    def gravity(self) -> list[float]:
        g = None
        if self._gravity_fn is not None:
            try:
                g = self._gravity_fn()
            except Exception:
                g = None
        if g is None or len(g) != 3:
            return [0.0, 0.0, -9.81]
        return [float(v) for v in g]

    def set_model(self, model: Any) -> None:
        """Robot replaced or edited (or None on close): invalidate caches, refresh."""
        self._cache.invalidate()
        usable = model is not None and len(model.joints) > 0 and model.dof() > 0
        self._model = model if usable else None
        n = self._model.dof() if self._model is not None else 0
        if n != self._n_dof or len(self._qd_fields) != n:
            self._rebuild_entries(n)
        self.refresh()

    def invalidate(self) -> None:
        """Model data changed without robot_changed (mass edit): rebuild on next refresh."""
        self._cache.invalidate()
        self.refresh()

    def refresh(self) -> None:
        """Recompute every group at the current q (cheap, UI thread)."""
        model = self._model
        if model is None:
            self._clear()
            return
        try:
            q = current_q(model)
        except Exception:
            self._clear()
            return
        self._refresh_singularity(model, q)
        self._refresh_dynamics(model, q)

    def forward_dynamics(self) -> None:
        """Fill qdd with the free acceleration (tau = 0) at the current q, qd."""
        model = self._model
        if model is None or mass_not_set(model):
            return
        try:
            dm = self._cache.get_dm(model)
            q = current_q(model)
            qd = [f.value() for f in self._qd_fields]
            qdd = cb.forward_dynamics(dm, q, qd, [0.0] * len(q), tuple(self.gravity()))
        except Exception as exc:
            self._lbl_dyn_status.setText(f"Forward dynamics failed: {exc}")
            return
        for f, v in zip(self._qdd_fields, qdd):
            f.set_value(float(v))

    # ------------------------------------------------------------------
    # Internals
    # ------------------------------------------------------------------

    def _rebuild_entries(self, n: int) -> None:
        while self._qd_grid.count():
            item = self._qd_grid.takeAt(0)
            w = item.widget()
            if w is not None:
                w.deleteLater()
        self._qd_fields = []
        self._qdd_fields = []
        self._n_dof = n
        if n:
            self._qd_grid.addWidget(QLabel("qd"), 0, 0)
            self._qd_grid.addWidget(QLabel("qdd"), 1, 0)
        for k in range(n):
            fd = NumField(0.0)
            fdd = NumField(0.0)
            fd.value_changed.connect(lambda _v: self.refresh())
            fdd.value_changed.connect(lambda _v: self.refresh())
            self._qd_fields.append(fd)
            self._qdd_fields.append(fdd)
            self._qd_grid.addWidget(fd, 0, k + 1)
            self._qd_grid.addWidget(fdd, 1, k + 1)

    def _clear(self) -> None:
        for lbl in (self._lbl_smin, self._lbl_smax, self._lbl_cond, self._lbl_manip,
                    self._lbl_near, self._lbl_ke, self._lbl_pe, self._lbl_te):
            lbl.setText("-")
        self._lbl_near.setStyleSheet("")
        self._table.setRowCount(0)
        self._last_tau = []
        self._mass_warning.setVisible(False)
        self._lbl_dyn_status.setText("")
        self._lbl_gravity.setText("")
        self._btn_fd.setEnabled(False)

    def _refresh_singularity(self, model: Any, q: list[float]) -> None:
        try:
            cm = self._cache.get_cm(model)
            info = singularity_at(cm, q, self._mode.currentText(), self._threshold.value())
        except Exception as exc:
            for lbl in (self._lbl_smin, self._lbl_smax, self._lbl_cond, self._lbl_manip):
                lbl.setText("-")
            self._lbl_near.setText(f"error: {exc}")
            self._lbl_near.setStyleSheet(_BAD_STYLE)
            return
        self._lbl_smin.setText(f"{info['sigma_min']:.4g}")
        self._lbl_smax.setText(f"{info['sigma_max']:.4g}")
        self._lbl_cond.setText(f"{info['condition']:.4g}")
        self._lbl_manip.setText(f"{info['manipulability']:.4g}")
        if info["near_singular"]:
            self._lbl_near.setText("NEAR SINGULAR")
            self._lbl_near.setStyleSheet(_BAD_STYLE)
        else:
            self._lbl_near.setText("ok")
            self._lbl_near.setStyleSheet(_OK_STYLE)

    def _refresh_dynamics(self, model: Any, q: list[float]) -> None:
        g = self.gravity()
        self._lbl_gravity.setText(f"Gravity [m/s^2]: {g[0]:g}, {g[1]:g}, {g[2]:g} (project)")
        warn = mass_not_set(model)
        self._mass_warning.setVisible(warn)
        self._btn_fd.setEnabled(not warn)
        n = len(q)
        try:
            dm = self._cache.get_dm(model)
            qd = [f.value() for f in self._qd_fields] or [0.0] * n
            qdd = [f.value() for f in self._qdd_fields] or [0.0] * n
            gt = cb.gravity_torque(dm, q, tuple(g))
            bias = cb.bias_torque(dm, q, qd, tuple(g))
            M = cb.mass_matrix(dm, q)
            tau = cb.inverse_dynamics(dm, q, qd, qdd, tuple(g))
            ke = cb.kinetic_energy(dm, q, qd)
            pe = cb.potential_energy(dm, q, tuple(g))
            te = cb.total_energy(dm, q, qd, tuple(g))
        except Exception as exc:
            self._table.setRowCount(0)
            self._last_tau = []
            self._lbl_dyn_status.setText(f"Dynamics failed: {exc}")
            for lbl in (self._lbl_ke, self._lbl_pe, self._lbl_te):
                lbl.setText("-")
            return
        self._lbl_dyn_status.setText("")
        tau = [float(v) for v in tau]
        self._last_tau = tau
        limits = check_torque_limits(model, tau)
        self._table.setRowCount(n)
        for k in range(n):
            mq = sum(float(M[k][c]) * qdd[c] for c in range(n))
            lim = limits[k]
            limit_txt = "none" if lim["no_limit"] else f"{lim['limit']:.4g}"
            ratio_txt = "-" if lim["no_limit"] else f"{lim['ratio']:.3g}"
            cells = [str(k), lim["joint_name"], f"{gt[k]:.5g}", f"{bias[k] - gt[k]:.5g}",
                     f"{mq:.5g}", f"{tau[k]:.5g}", limit_txt, ratio_txt]
            for c, text in enumerate(cells):
                item = QTableWidgetItem(text)
                if lim["exceeded"] and c in (5, 6, 7):
                    item.setForeground(_RED)
                self._table.setItem(k, c, item)
        self._lbl_ke.setText(f"{ke:.6g}")
        self._lbl_pe.setText(f"{pe:.6g}")
        self._lbl_te.setText(f"{te:.6g}")
