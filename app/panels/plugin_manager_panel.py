"""Plugin manager dock (R8.2): load results, enable/disable, permissions, reload."""
from __future__ import annotations

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (QAbstractItemView, QHBoxLayout, QLabel, QPushButton, QTableWidget,
                               QTableWidgetItem, QVBoxLayout, QWidget)

from app.plugins.loader import PluginManager

COLUMNS = ("Name", "Kind", "Version", "Status", "Permissions", "Message")


class PluginManagerPanel(QWidget):
    log_requested = Signal(str)

    def __init__(self, manager: PluginManager | None = None, parent=None):
        super().__init__(parent)
        self._manager: PluginManager | None = None
        lay = QVBoxLayout(self)
        self._folder = QLabel("")
        self._folder.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        lay.addWidget(self._folder)
        self._table = QTableWidget(0, len(COLUMNS))
        self._table.setHorizontalHeaderLabels(list(COLUMNS))
        self._table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self._table.setSelectionMode(QAbstractItemView.SelectionMode.SingleSelection)
        self._table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self._table.horizontalHeader().setStretchLastSection(True)
        self._table.itemSelectionChanged.connect(self._update_buttons)
        lay.addWidget(self._table, 1)
        row = QHBoxLayout()
        self._btn_toggle = QPushButton("Disable")
        self._btn_toggle.clicked.connect(self.toggle_selected)
        self._btn_revoke = QPushButton("Revoke permissions")
        self._btn_revoke.clicked.connect(self.revoke_selected)
        self._btn_reload = QPushButton("Reload all")
        self._btn_reload.clicked.connect(self.reload)
        for b in (self._btn_toggle, self._btn_revoke, self._btn_reload):
            row.addWidget(b)
        row.addStretch(1)
        lay.addLayout(row)
        note = QLabel("Plugins run inside the application. Permissions are a cooperative guard, "
                      "not a sandbox: install only plugins you trust.")
        note.setWordWrap(True)
        lay.addWidget(note)
        self.set_manager(manager)

    # ------------------------------------------------------------------
    def set_manager(self, manager: PluginManager | None) -> None:
        if self._manager is not None:
            try:
                self._manager.changed.disconnect(self.refresh)
            except (RuntimeError, TypeError):
                pass
        self._manager = manager
        if manager is not None:
            manager.changed.connect(self.refresh)
        self.refresh()

    def refresh(self) -> None:
        sel = self.selected_name()
        m = self._manager
        self._folder.setText(f"Folder: {m.folder}" if m is not None else "")
        results = m.results if m is not None else []
        self._table.setRowCount(len(results))
        for r, res in enumerate(results):
            perms = ""
            if m is not None and res.permissions:
                dec = m.permissions.decisions(res.name, res.permissions)
                perms = ", ".join(f"{k}:{v}" for k, v in dec.items())
            for c, text in enumerate((res.name, res.kind, res.version, res.status, perms, res.message)):
                it = QTableWidgetItem(text)
                it.setToolTip(res.message if c == 5 else text)
                self._table.setItem(r, c, it)
            if res.name == sel:
                self._table.selectRow(r)
        self._update_buttons()

    def selected_name(self) -> str | None:
        rows = self._table.selectionModel().selectedRows() if self._table.selectionModel() else []
        if not rows:
            return None
        it = self._table.item(rows[0].row(), 0)
        return it.text() if it is not None else None

    def _update_buttons(self) -> None:
        name = self.selected_name()
        res = self._manager.result(name) if (self._manager is not None and name) else None
        has = res is not None and res.status != "rejected"
        self._btn_toggle.setEnabled(has)
        self._btn_toggle.setText("Enable" if res is not None and res.status == "disabled" else "Disable")
        self._btn_revoke.setEnabled(has and bool(res.permissions))

    # ------------------------------------------------------------------
    def toggle_selected(self) -> None:
        name = self.selected_name()
        if self._manager is None or not name:
            return
        res = self._manager.result(name)
        if res is None:
            return
        self._manager.set_enabled(name, res.status == "disabled")

    def revoke_selected(self) -> None:
        name = self.selected_name()
        if self._manager is None or not name:
            return
        self._manager.permissions.revoke(name)
        self.log_requested.emit(f"[INFO] plugin {name}: permissions revoked (will ask again)")
        self.refresh()

    def reload(self) -> None:
        if self._manager is not None:
            self._manager.scan()

    @property
    def table(self) -> QTableWidget:
        return self._table
