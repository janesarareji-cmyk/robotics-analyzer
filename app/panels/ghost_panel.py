"""
app/panels/ghost_panel.py
R7.6: "Ghosts" dock. Named joint configurations (saved from the current q or from the IK
solution list), translucent ghosts of the robot at those configurations (max 8 shown, one colour
each), and a comparison table for two or more configurations (reference = first selected row).

The panel owns no 3D actors: it emits ``ghosts_changed([(name, q, color), ...])`` and the main
window draws them once through ``RobotView.add_ghost`` (no per-frame loop). Configurations live in
the project's ``ConfigStore``; ``set_model`` drops the ones with a wrong DOF count (log warning).
"""

from __future__ import annotations

import math
from typing import Any, Callable

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QAbstractItemView, QHBoxLayout, QLabel, QLineEdit, QPushButton, QTableWidget,
    QTableWidgetItem, QVBoxLayout, QWidget,
)

from app.configs.store import ConfigStore, compare_configs, joint_is_prismatic
from app.kin_utils import current_q

MAX_GHOSTS = 8
PALETTE = ("#4fc3f7", "#ff8a65", "#81c784", "#ba68c8", "#ffd54f", "#f06292", "#4db6ac", "#a1887f")
MAX_MESSAGE = f"Max {MAX_GHOSTS} ghosts shown: hide one first."


def _fmt(x: float) -> str:
    return f"{x:.5g}"


class GhostPanel(QWidget):
    """Dock content: configuration list, ghost toggles, comparison table."""

    log_requested = Signal(str)
    configs_changed = Signal()      # store edited (project becomes dirty)
    ghosts_changed = Signal(list)   # [(name, q, color)] of the shown ghosts

    def __init__(self, parent=None):
        super().__init__(parent)
        self._store = ConfigStore()
        self._model: Any = None
        self._cm: Any = None
        self._n_dof = 0
        self._shown: dict[str, str] = {}          # name -> colour, insertion order
        self._q_provider: Callable[[], list[float]] | None = None
        self._ik_source: Callable[[], list] | None = None
        self._rows: list[dict] = []
        self._loading = False

        layout = QVBoxLayout(self)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(4)

        row = QHBoxLayout()
        self._name = QLineEdit()
        self._name.setPlaceholderText("name")
        self._btn_save = QPushButton("Save current q")
        self._btn_save.clicked.connect(lambda: self.save_current())
        self._btn_ik = QPushButton("Add IK solutions")
        self._btn_ik.clicked.connect(lambda: self.add_ik_solutions(show=True))
        row.addWidget(self._name, 1)
        row.addWidget(self._btn_save)
        row.addWidget(self._btn_ik)
        layout.addLayout(row)

        self._table = QTableWidget(0, 3)
        self._table.setHorizontalHeaderLabels(["Show", "Name", "q"])
        self._table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self._table.setSelectionMode(QAbstractItemView.SelectionMode.ExtendedSelection)
        self._table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self._table.setMinimumHeight(110)
        self._table.itemChanged.connect(self._on_item_changed)
        layout.addWidget(self._table, 1)

        row2 = QHBoxLayout()
        self._btn_delete = QPushButton("Delete")
        self._btn_delete.clicked.connect(self.delete_selected)
        self._btn_hide = QPushButton("Hide all")
        self._btn_hide.clicked.connect(self.hide_all)
        self._btn_compare = QPushButton("Compare selected")
        self._btn_compare.clicked.connect(self.compare_selected)
        row2.addWidget(self._btn_delete)
        row2.addWidget(self._btn_hide)
        row2.addWidget(self._btn_compare)
        layout.addLayout(row2)

        self._status = QLabel("")
        self._status.setWordWrap(True)
        layout.addWidget(self._status)

        self._cmp = QTableWidget(0, 0)
        self._cmp.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self._cmp.setMinimumHeight(110)
        layout.addWidget(self._cmp, 1)
        self._update_buttons()

    # ------------------------------------------------------------------ facts
    @property
    def store(self) -> ConfigStore:
        return self._store

    @property
    def shown_names(self) -> list[str]:
        return list(self._shown)

    @property
    def status_text(self) -> str:
        return self._status.text()

    @property
    def table(self) -> QTableWidget:
        return self._table

    @property
    def comparison_table(self) -> QTableWidget:
        return self._cmp

    @property
    def comparison(self) -> list[dict]:
        return self._rows

    def ghost_color(self, name: str) -> str | None:
        return self._shown.get(name)

    # ------------------------------------------------------------------ setup
    def set_q_provider(self, fn: Callable[[], list[float]] | None) -> None:
        self._q_provider = fn

    def set_ik_source(self, fn: Callable[[], list] | None) -> None:
        """``fn()`` returns the IK solution q vectors (R4.9 list)."""
        self._ik_source = fn

    def set_store(self, store: ConfigStore | None) -> None:
        """Use the configurations of a project (None on close). Ghosts are hidden."""
        self._store = store if store is not None else ConfigStore()
        self._shown.clear()
        self._refresh_table()
        self.ghosts_changed.emit([])

    def set_model(self, model: Any) -> None:
        """Robot opened / replaced / edited (None on close): drop wrong-DOF configurations,
        hide the ghosts (they were drawn for the old robot), clear the comparison."""
        import core_bridge as cb

        usable = model is not None and len(model.joints) > 0 and model.dof() > 0
        self._model = model if usable else None
        self._n_dof = model.dof() if usable else 0
        self._cm = None
        if self._model is not None:
            try:
                self._cm = cb.compile(self._model)
            except Exception:
                self._cm = None
        dropped = self._store.validate_dof(self._n_dof) if model is not None else []
        if dropped:
            msg = (f"[WARNING] {len(dropped)} configuration(s) dropped, the robot has "
                   f"{self._n_dof} DOF: {', '.join(dropped)}")
            self.log_requested.emit(msg)
            self.configs_changed.emit()
        self._shown.clear()
        self._rows = []
        self._cmp.setRowCount(0)
        self._cmp.setColumnCount(0)
        self._status.setText("")
        self._refresh_table()
        self.ghosts_changed.emit([])

    # ------------------------------------------------------------------ store edit
    def _current_q(self) -> list[float] | None:
        if self._q_provider is not None:
            return [float(x) for x in self._q_provider()]
        if self._model is not None:
            return current_q(self._model)
        return None

    def save_current(self, name: str | None = None) -> str | None:
        """Store the current q under ``name`` (field text, else a generated one)."""
        q = self._current_q()
        if self._model is None or q is None or len(q) != self._n_dof:
            self._status.setText("No robot: nothing to save.")
            return None
        name = (name if name is not None else self._name.text()).strip()
        if not name:
            name = self._store.unique_name("config")
        self._store.set(name, q)
        self._name.clear()
        self._after_edit(f"Saved configuration '{name}'.")
        return name

    def add_configuration(self, name: str, q) -> str | None:
        """Store an explicit q (length must match the DOF count)."""
        if self._model is None or len(list(q)) != self._n_dof:
            self._status.setText(f"Configuration needs {self._n_dof} values.")
            return None
        self._store.set(name, q)
        self._after_edit(f"Saved configuration '{name}'.")
        return name

    def add_ik_solutions(self, show: bool = False) -> list[str]:
        """Add every IK solution as ``ik``, ``ik 2``, ...; optionally show them (up to the limit)."""
        sols = list(self._ik_source()) if self._ik_source is not None else []
        names = []
        for q in sols:
            q = [float(x) for x in q]
            if self._model is None or len(q) != self._n_dof:
                continue
            nm = self._store.unique_name("ik")
            self._store.set(nm, q)
            names.append(nm)
        if not names:
            self._status.setText("No IK solutions to add.")
            return []
        self._after_edit(f"Added {len(names)} IK solution(s).", emit_ghosts=False)
        if show:
            for nm in names:
                if not self.set_shown(nm, True):
                    break
        return names

    def delete(self, name: str) -> None:
        if name not in self._store:
            return
        self._store.remove(name)
        was = self._shown.pop(name, None) is not None
        self._after_edit(f"Deleted '{name}'.", emit_ghosts=was)

    def delete_selected(self) -> None:
        for name in self.selected_names():
            self.delete(name)

    def _after_edit(self, message: str, emit_ghosts: bool = False) -> None:
        self._refresh_table()
        self._status.setText(message)
        self.configs_changed.emit()
        if emit_ghosts:
            self._emit_ghosts()

    # ------------------------------------------------------------------ ghosts
    def set_shown(self, name: str, on: bool) -> bool:
        """Show / hide the ghost of ``name``. False when refused (unknown name or 8 shown)."""
        if name not in self._store:
            return False
        if on:
            if name in self._shown:
                return True
            if len(self._shown) >= MAX_GHOSTS:
                self._status.setText(MAX_MESSAGE)
                self.log_requested.emit(f"[WARNING] {MAX_MESSAGE}")
                self._sync_checks()
                return False
            used = set(self._shown.values())
            self._shown[name] = next((c for c in PALETTE if c not in used), PALETTE[0])
        else:
            if name not in self._shown:
                return True
            del self._shown[name]
        self._sync_checks()
        self._emit_ghosts()
        return True

    def hide_all(self) -> None:
        if self._shown:
            self._shown.clear()
            self._sync_checks()
            self._emit_ghosts()

    def _emit_ghosts(self) -> None:
        self.ghosts_changed.emit([(n, self._store.get(n), c) for n, c in self._shown.items()
                                  if n in self._store])

    # ------------------------------------------------------------------ table
    def selected_names(self) -> list[str]:
        rows = sorted({i.row() for i in self._table.selectedIndexes()})
        out = []
        for r in rows:
            it = self._table.item(r, 1)
            if it is not None:
                out.append(it.text())
        return out

    def select_names(self, names: list[str]) -> None:
        from PySide6.QtCore import QItemSelectionModel
        sm = self._table.selectionModel()
        sm.clearSelection()
        flags = QItemSelectionModel.SelectionFlag.Select | QItemSelectionModel.SelectionFlag.Rows
        for r in range(self._table.rowCount()):
            if self._table.item(r, 1).text() in names:
                sm.select(self._table.model().index(r, 0), flags)

    def _refresh_table(self) -> None:
        self._loading = True
        try:
            names = self._store.names
            self._table.setRowCount(len(names))
            for r, nm in enumerate(names):
                chk = QTableWidgetItem()
                chk.setFlags(Qt.ItemFlag.ItemIsUserCheckable | Qt.ItemFlag.ItemIsEnabled)
                chk.setCheckState(Qt.CheckState.Checked if nm in self._shown
                                  else Qt.CheckState.Unchecked)
                self._table.setItem(r, 0, chk)
                self._table.setItem(r, 1, QTableWidgetItem(nm))
                self._table.setItem(r, 2, QTableWidgetItem(
                    "[" + ", ".join(_fmt(x) for x in self._store.get(nm)) + "]"))
        finally:
            self._loading = False
        self._update_buttons()

    def _sync_checks(self) -> None:
        self._loading = True
        try:
            for r in range(self._table.rowCount()):
                nm = self._table.item(r, 1).text()
                self._table.item(r, 0).setCheckState(
                    Qt.CheckState.Checked if nm in self._shown else Qt.CheckState.Unchecked)
                col = self._shown.get(nm)
                self._table.item(r, 1).setBackground(
                    self._table.palette().base() if col is None else _qcolor(col))
        finally:
            self._loading = False

    def _on_item_changed(self, item: QTableWidgetItem) -> None:
        if self._loading or item.column() != 0:
            return
        nm = self._table.item(item.row(), 1).text()
        self.set_shown(nm, item.checkState() == Qt.CheckState.Checked)

    def _update_buttons(self) -> None:
        has = self._model is not None
        self._btn_save.setEnabled(has)
        self._btn_ik.setEnabled(has)
        self._btn_delete.setEnabled(len(self._store) > 0)
        self._btn_compare.setEnabled(len(self._store) >= 2)

    # ------------------------------------------------------------------ comparison
    def compare_selected(self) -> list[dict]:
        """Compare the selected rows (reference = first selected). Needs two or more."""
        return self.compare(self.selected_names())

    def compare(self, names: list[str]) -> list[dict]:
        names = [n for n in names if n in self._store]
        if self._model is None or len(names) < 2:
            self._status.setText("Select two or more configurations to compare.")
            self._rows = []
            self._cmp.setRowCount(0)
            return []
        try:
            rows = compare_configs(self._model, [(n, self._store.get(n)) for n in names], self._cm)
        except Exception as exc:
            self._status.setText(f"Comparison failed: {exc}")
            self._rows = []
            return []
        self._rows = rows
        self._fill_comparison(rows)
        self._status.setText(f"Comparison against '{names[0]}'.")
        return rows

    def _fill_comparison(self, rows: list[dict]) -> None:
        pris = joint_is_prismatic(self._model)
        heads = ["Name", "dx [m]", "dy [m]", "dz [m]", "TCP dist [m]", "Rot angle [deg]",
                 "Rot vec [deg]", "Manipulability", "dManip"]
        heads += [f"dq{i + 1} [{'m' if p else 'rad'}]" for i, p in enumerate(pris)]
        self._cmp.setColumnCount(len(heads))
        self._cmp.setHorizontalHeaderLabels(heads)
        self._cmp.setRowCount(len(rows))
        for r, d in enumerate(rows):
            rv = [math.degrees(x) for x in d["drot"]]
            cells = [d["name"], *(_fmt(x) for x in d["dp"]), _fmt(d["distance"]),
                     _fmt(math.degrees(d["angle"])), "(" + ", ".join(_fmt(x) for x in rv) + ")",
                     _fmt(d["manipulability"]), _fmt(d["dmanip"]), *(_fmt(x) for x in d["dq"])]
            for c, text in enumerate(cells):
                self._cmp.setItem(r, c, QTableWidgetItem(text))


def _qcolor(hex_color: str):
    from PySide6.QtGui import QColor
    c = QColor(hex_color)
    c.setAlpha(110)
    return c
