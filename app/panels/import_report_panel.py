"""
app/panels/import_report_panel.py
R3.6: Import report dock. One row per issue, severity filters, row click
selects the named joint or link in the robot tree, "Save report..." writes
the full report as text.
"""

from __future__ import annotations

from typing import Any, Optional

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QCheckBox, QPushButton,
    QTableWidget, QTableWidgetItem, QHeaderView, QAbstractItemView,
    QFileDialog, QStyle,
)
from PySide6.QtCore import Signal
from PySide6.QtGui import QColor

from app.importers.import_report import (
    ImportIssue, ImportReport, SEV_ERROR, SEV_WARNING, SEV_INFO,
)

_SEVERITIES = (SEV_ERROR, SEV_WARNING, SEV_INFO)
_LABELS = {SEV_ERROR: "Errors", SEV_WARNING: "Warnings", SEV_INFO: "Info"}
_COLORS = {SEV_ERROR: "#ff6b6b", SEV_WARNING: "#e0b050", SEV_INFO: "#8fb8de"}
_HEADERS = ["Severity", "Code", "Message", "Element"]


def find_element(model: Any, name: Optional[str]) -> Optional[tuple[str, int]]:
    """
    Look an element name up in a model. Joints first, then links.
    Returns ("joint" | "link", index) or None (no name, no match, no model).
    """
    if model is None or not name:
        return None
    for i, joint in enumerate(model.joints):
        if joint.name == name:
            return ("joint", i)
    for i, link in enumerate(model.links):
        if link.name == name:
            return ("link", i)
    return None


class ImportReportPanel(QWidget):
    """Dock content: the issues of the last import."""

    element_clicked = Signal(str)  # element name of the clicked row

    def __init__(self, parent=None):
        super().__init__(parent)
        self._report = ImportReport()
        self._rows: list[ImportIssue] = []

        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(2)

        self._summary = QLabel("No import yet")
        self._summary.setStyleSheet("font-weight: bold; color: #aaaaaa;")
        layout.addWidget(self._summary)

        filter_row = QHBoxLayout()
        self._filters: dict[str, QCheckBox] = {}
        for sev in _SEVERITIES:
            box = QCheckBox(_LABELS[sev])
            box.setChecked(True)
            box.toggled.connect(self._refresh)
            self._filters[sev] = box
            filter_row.addWidget(box)
        filter_row.addStretch(1)
        layout.addLayout(filter_row)

        self._table = QTableWidget(0, len(_HEADERS))
        self._table.setHorizontalHeaderLabels(_HEADERS)
        self._table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self._table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self._table.setSelectionMode(QAbstractItemView.SelectionMode.SingleSelection)
        self._table.verticalHeader().setVisible(False)
        self._table.horizontalHeader().setSectionResizeMode(
            2, QHeaderView.ResizeMode.Stretch
        )
        self._table.setStyleSheet("background: #1e1e1e; color: #cccccc; border: none;")
        self._table.cellClicked.connect(self._on_cell_clicked)
        layout.addWidget(self._table)

        button_row = QHBoxLayout()
        button_row.addStretch(1)
        self._save_button = QPushButton("Save report...")
        self._save_button.clicked.connect(self.save_report_dialog)
        button_row.addWidget(self._save_button)
        layout.addLayout(button_row)

    # ------------------------------------------------------------------
    @property
    def report(self) -> ImportReport:
        return self._report

    @property
    def row_count(self) -> int:
        """Rows shown after the severity filters."""
        return self._table.rowCount()

    @property
    def table(self) -> QTableWidget:
        return self._table

    def filter_box(self, severity: str) -> QCheckBox:
        return self._filters[severity]

    def row_issue(self, row: int) -> Optional[ImportIssue]:
        if 0 <= row < len(self._rows):
            return self._rows[row]
        return None

    # ------------------------------------------------------------------
    def set_report(self, report: Optional[ImportReport]) -> None:
        """Show the issues of one import."""
        self._report = report if report is not None else ImportReport()
        self._refresh()

    def _refresh(self, *_args) -> None:
        shown = [
            i for i in self._report.issues if self._filters[i.severity].isChecked()
        ]
        self._rows = shown
        self._table.setRowCount(0)
        style = self.style()
        icons = {
            SEV_ERROR: style.standardIcon(QStyle.StandardPixmap.SP_MessageBoxCritical),
            SEV_WARNING: style.standardIcon(QStyle.StandardPixmap.SP_MessageBoxWarning),
            SEV_INFO: style.standardIcon(QStyle.StandardPixmap.SP_MessageBoxInformation),
        }
        for row, issue in enumerate(shown):
            self._table.insertRow(row)
            sev_item = QTableWidgetItem(icons[issue.severity], issue.severity.upper())
            sev_item.setForeground(QColor(_COLORS[issue.severity]))
            self._table.setItem(row, 0, sev_item)
            self._table.setItem(row, 1, QTableWidgetItem(issue.code))
            self._table.setItem(row, 2, QTableWidgetItem(issue.message))
            self._table.setItem(row, 3, QTableWidgetItem(issue.element or ""))
        self._table.resizeColumnsToContents()
        self._summary.setText(f"Import report: {self._report.summary()}")

    def _on_cell_clicked(self, row: int, _column: int) -> None:
        issue = self.row_issue(row)
        if issue is not None and issue.element:
            self.element_clicked.emit(issue.element)

    # ------------------------------------------------------------------
    def save_report_to(self, path: str) -> None:
        """Write the full report (all severities) as text."""
        with open(path, "w", encoding="utf-8") as fh:
            fh.write(self._report.to_text() + "\n")

    def save_report_dialog(self) -> Optional[str]:
        """Ask for a file name and save the full report. Returns the path or None."""
        path, _ = QFileDialog.getSaveFileName(
            self, "Save import report", "import_report.txt", "Text (*.txt)"
        )
        if not path:
            return None
        self.save_report_to(path)
        return path
