"""
app/panels/physical_panel.py
R3.7: "Physical" dock. One table row per link: mass, COM (link frame) and the
six inertia terms about the COM (link-frame axes, SI units). Edits go straight
into the URM (`Link.inertia`), so they are saved in `robot_model.json`.

Also: "Estimate from visual", "Scale all masses to total...", and a readout of
the total mass and the total COM in the world frame at the current q.
"""

from __future__ import annotations

from typing import Any, List, Optional

import numpy as np
from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QAbstractItemView, QHBoxLayout, QHeaderView, QInputDialog, QLabel,
    QPushButton, QStyle, QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget,
)

import core_bridge as cb

HEADERS = ["Link", "Mass [kg]", "COM x", "COM y", "COM z",
           "Ixx", "Ixy", "Ixz", "Iyy", "Iyz", "Izz", "Status"]
COL_NAME = 0
COL_MASS = 1
COL_COM = 2      # 2..4
COL_TENSOR = 5   # 5..10
COL_STATUS = 11


def _fmt(v: float) -> str:
    return f"{v:.9g}"


class PhysicalPropertiesPanel(QWidget):
    """Dock content: mass / COM / inertia of every link."""

    edited = Signal()              # a value changed (project is dirty)
    log_requested = Signal(str)    # message for the log panel

    def __init__(self, parent=None):
        super().__init__(parent)
        self._model: Any = None
        self._visuals: list = []
        self._mesh_cache: dict = {}
        self._filling = False

        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(4)

        self._table = QTableWidget(0, len(HEADERS))
        self._table.setHorizontalHeaderLabels(HEADERS)
        self._table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self._table.horizontalHeader().setSectionResizeMode(
            QHeaderView.ResizeMode.ResizeToContents
        )
        self._table.itemChanged.connect(self._on_item_changed)
        layout.addWidget(self._table)

        row = QHBoxLayout()
        self._btn_estimate = QPushButton("Estimate from visual")
        self._btn_estimate.setToolTip(
            "Estimate mass, COM and inertia of the selected links "
            "(all links when none is selected) from their visual"
        )
        self._btn_estimate.clicked.connect(self._on_estimate_clicked)
        row.addWidget(self._btn_estimate)
        self._btn_scale = QPushButton("Scale all masses to total...")
        self._btn_scale.clicked.connect(self._on_scale_clicked)
        row.addWidget(self._btn_scale)
        row.addStretch(1)
        layout.addLayout(row)

        self._readout = QLabel("")
        self._readout.setStyleSheet("color: #cccccc;")
        self._readout.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        layout.addWidget(self._readout)
        self._refresh_enabled()

    # ------------------------------------------------------------------
    # Model
    # ------------------------------------------------------------------

    def set_model(self, model: Any, visuals: Optional[list] = None) -> None:
        """Show the links of `model` (None clears). `visuals` = LinkVisual list."""
        self._model = model
        if visuals is not None:
            self._visuals = visuals
        if model is None:
            self._visuals = []
        self._mesh_cache.clear()
        self.rebuild()

    def set_visuals(self, visuals: Optional[list]) -> None:
        self._visuals = visuals if visuals is not None else []
        self._mesh_cache.clear()

    def rebuild(self) -> None:
        """Re-read every cell from the model."""
        self._filling = True
        try:
            self._table.setRowCount(0)
            model = self._model
            if model is not None:
                self._table.setRowCount(len(model.links))
                for i, lk in enumerate(model.links):
                    self._fill_row(i, lk)
        finally:
            self._filling = False
        self._refresh_status()
        self.update_readout()
        self._refresh_enabled()

    def _fill_row(self, i: int, lk: Any) -> None:
        name = QTableWidgetItem(lk.name or f"link{i}")
        name.setFlags(name.flags() & ~Qt.ItemFlag.ItemIsEditable)
        self._table.setItem(i, COL_NAME, name)
        inr = lk.inertia
        values = [inr.mass, *inr.com, *inr.inertia_tensor]
        for k, v in enumerate(values):
            self._table.setItem(i, COL_MASS + k, QTableWidgetItem(_fmt(v)))
        status = QTableWidgetItem("")
        status.setFlags(status.flags() & ~Qt.ItemFlag.ItemIsEditable)
        self._table.setItem(i, COL_STATUS, status)

    def _refresh_enabled(self) -> None:
        has = self._model is not None and len(self._model.links) > 0
        self._btn_estimate.setEnabled(has)
        self._btn_scale.setEnabled(has)

    def _refresh_status(self) -> None:
        """Status icon + tooltip per row from validate_physical()."""
        model = self._model
        if model is None:
            return
        msgs = list(model.validate_physical())
        style = self.style()
        ok_icon = style.standardIcon(QStyle.StandardPixmap.SP_DialogApplyButton)
        warn_icon = style.standardIcon(QStyle.StandardPixmap.SP_MessageBoxWarning)
        for i, lk in enumerate(model.links):
            item = self._table.item(i, COL_STATUS)
            if item is None:
                continue
            prefix = f"link '{lk.name}': "
            mine = [m[len(prefix):] for m in msgs if m.startswith(prefix)]
            if mine:
                item.setIcon(warn_icon)
                item.setText("; ".join(mine))
                item.setToolTip("\n".join(mine))
            else:
                item.setIcon(ok_icon)
                item.setText("OK")
                item.setToolTip("")

    # ------------------------------------------------------------------
    # Editing
    # ------------------------------------------------------------------

    def _on_item_changed(self, item: QTableWidgetItem) -> None:
        if self._filling or self._model is None:
            return
        row, col = item.row(), item.column()
        if col < COL_MASS or col > COL_TENSOR + 5:
            return
        try:
            value = float(item.text().strip())
        except ValueError:
            self._restore_cell(row, col)
            return
        self._set_value(row, col, value)
        self._after_edit()

    def _restore_cell(self, row: int, col: int) -> None:
        inr = self._model.links[row].inertia
        values = [inr.mass, *inr.com, *inr.inertia_tensor]
        self._filling = True
        try:
            self._table.item(row, col).setText(_fmt(values[col - COL_MASS]))
        finally:
            self._filling = False

    def _set_value(self, row: int, col: int, value: float) -> None:
        lk = self._model.links[row]
        old = lk.inertia
        new = cb.Inertia()
        new.mass = old.mass
        com = list(old.com)
        tensor = list(old.inertia_tensor)
        if col == COL_MASS:
            new.mass = value
        elif col < COL_TENSOR:
            com[col - COL_COM] = value
        else:
            tensor[col - COL_TENSOR] = value
        new.com = com
        new.inertia_tensor = tensor
        lk.inertia = new

    def _after_edit(self) -> None:
        self._refresh_status()
        self.update_readout()
        self.edited.emit()

    def set_value(self, row: int, col: int, text: str) -> None:
        """Programmatic cell edit (same path as typing in the table)."""
        self._table.item(row, col).setText(text)

    def set_link_inertia(self, row: int, mass: float, com, tensor) -> None:
        """Write a whole Inertia into link `row` and refresh the row."""
        new = cb.Inertia()
        new.mass = float(mass)
        new.com = [float(x) for x in com]
        new.inertia_tensor = [float(x) for x in tensor]
        self._model.links[row].inertia = new
        self._filling = True
        try:
            for k, v in enumerate([new.mass, *new.com, *new.inertia_tensor]):
                self._table.item(row, COL_MASS + k).setText(_fmt(v))
        finally:
            self._filling = False

    # ------------------------------------------------------------------
    # Totals
    # ------------------------------------------------------------------

    def total_mass(self) -> float:
        if self._model is None:
            return 0.0
        return float(sum(lk.inertia.mass for lk in self._model.links))

    def total_com(self) -> Optional[np.ndarray]:
        """Total COM in the world frame at the current q, or None (mass 0)."""
        model = self._model
        if model is None:
            return None
        total = self.total_mass()
        if not total > 0.0:
            return None
        frames = cb.fk(model)["frames"]
        acc = np.zeros(3)
        for k, lk in enumerate(model.links):
            m = lk.inertia.mass
            if m == 0.0:
                continue
            f = np.asarray(frames[k], dtype=float)
            acc += m * (f[:3, :3] @ np.asarray(lk.inertia.com, dtype=float) + f[:3, 3])
        return acc / total

    def update_readout(self) -> None:
        """Refresh the total mass / COM label (call after q changes)."""
        if self._model is None:
            self._readout.setText("")
            return
        total = self.total_mass()
        try:
            com = self.total_com()
        except Exception:
            com = None
        if com is None:
            self._readout.setText(f"Total mass: {total:.6g} kg    COM (world): -")
        else:
            self._readout.setText(
                f"Total mass: {total:.6g} kg    COM (world): "
                f"[{com[0]:.6g}, {com[1]:.6g}, {com[2]:.6g}] m"
            )

    @property
    def readout_text(self) -> str:
        return self._readout.text()

    # ------------------------------------------------------------------
    # Estimate / scale
    # ------------------------------------------------------------------

    def _visual_for_link(self, index: int):
        for v in self._visuals:
            if v.link_index == index:
                return v
        return None

    def selected_rows(self) -> List[int]:
        return sorted({i.row() for i in self._table.selectedIndexes()})

    def estimate_from_visuals(self, rows: Optional[List[int]] = None,
                              density: Optional[float] = None,
                              total_mass: Optional[float] = None) -> List[str]:
        """
        Estimate mass/COM/inertia from the visual of each row (None = all
        links). `density` [kg/m^3] applies per link. `total_mass` [kg] is the
        mass of one link when one row is estimated; for several rows it is the
        sum over those rows, shared by volume. Returns warnings (also logged).
        """
        from app.physics.estimate import visual_inertial

        model = self._model
        if model is None:
            return []
        if rows is None:
            rows = list(range(len(model.links)))
        warnings: List[str] = []
        results = {}
        use_density = density if density is not None else 1.0
        share = density is None and len(rows) > 1
        for r in rows:
            name = model.links[r].name
            v = self._visual_for_link(r)
            if v is None:
                warnings.append(f"link '{name}': no visual, skipped")
                continue
            try:
                if density is None and not share:
                    res = visual_inertial(v, total_mass=total_mass, cache=self._mesh_cache)
                else:
                    res = visual_inertial(v, density=use_density, cache=self._mesh_cache)
            except ValueError as exc:
                warnings.append(f"link '{name}': {exc}, skipped")
                continue
            warnings.extend(f"link '{name}': {w}" for w in res[3])
            results[r] = res
        if share and results:
            vol_mass = sum(res[0] for res in results.values())
            k = float(total_mass) / vol_mass if vol_mass > 0.0 else 0.0
            results = {
                r: (m * k, c, [x * k for x in t], w)
                for r, (m, c, t, w) in results.items()
            }
        for r, (m, c, t, _w) in results.items():
            self.set_link_inertia(r, m, c, t)
        if results:
            self._after_edit()
        for w in warnings:
            self.log_requested.emit(f"[WARNING] Physical: {w}")
        if results:
            self.log_requested.emit(
                f"[INFO] Physical: estimated {len(results)} link(s) from visuals."
            )
        return warnings

    def scale_masses_to(self, total: float) -> bool:
        """Scale every mass and inertia tensor so the total mass is `total`."""
        cur = self.total_mass()
        if self._model is None or not cur > 0.0 or not float(total) > 0.0:
            return False
        k = float(total) / cur
        for i, lk in enumerate(self._model.links):
            inr = lk.inertia
            self.set_link_inertia(i, inr.mass * k, inr.com,
                                  [x * k for x in inr.inertia_tensor])
        self._after_edit()
        return True

    def _on_estimate_clicked(self) -> None:
        rows = self.selected_rows() or None
        modes = ["Density [kg/m^3]", "Total mass [kg]"]
        mode, ok = QInputDialog.getItem(
            self, "Estimate from visual", "Known quantity:", modes, 0, False
        )
        if not ok:
            return
        if mode == modes[0]:
            val, ok = QInputDialog.getDouble(
                self, "Density", "Density [kg/m^3]:", 1000.0, 1e-6, 1e6, 3
            )
            if ok:
                self.estimate_from_visuals(rows, density=val)
        else:
            val, ok = QInputDialog.getDouble(
                self, "Total mass", "Total mass [kg]:", 1.0, 1e-9, 1e9, 6
            )
            if ok:
                self.estimate_from_visuals(rows, total_mass=val)

    def _on_scale_clicked(self) -> None:
        cur = self.total_mass()
        if not cur > 0.0:
            self.log_requested.emit("[WARNING] Physical: total mass is 0, cannot scale.")
            return
        val, ok = QInputDialog.getDouble(
            self, "Scale masses", "New total mass [kg]:", cur, 1e-9, 1e9, 6
        )
        if ok:
            self.scale_masses_to(val)

    # ------------------------------------------------------------------
    # Test / integration access
    # ------------------------------------------------------------------

    @property
    def table(self) -> QTableWidget:
        return self._table

    @property
    def row_count(self) -> int:
        return self._table.rowCount()

    def status_text(self, row: int) -> str:
        return self._table.item(row, COL_STATUS).text()
