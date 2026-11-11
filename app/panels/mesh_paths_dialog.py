"""
app/panels/mesh_paths_dialog.py
R3.5: small dialog to edit the folders searched for package:// meshes.
"""

from __future__ import annotations

from PySide6.QtWidgets import (
    QDialog,
    QDialogButtonBox,
    QFileDialog,
    QHBoxLayout,
    QListWidget,
    QPushButton,
    QVBoxLayout,
)


class MeshPathsDialog(QDialog):
    def __init__(self, roots: list[str], parent=None):
        super().__init__(parent)
        self.setWindowTitle("Mesh search paths")
        self.resize(480, 300)
        self._list = QListWidget()
        self._list.addItems(list(roots))

        add = QPushButton("Add...")
        add.clicked.connect(self._on_add)
        rem = QPushButton("Remove")
        rem.clicked.connect(self._on_remove)
        row = QHBoxLayout()
        row.addWidget(add)
        row.addWidget(rem)
        row.addStretch(1)

        box = QDialogButtonBox(
            QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel
        )
        box.accepted.connect(self.accept)
        box.rejected.connect(self.reject)

        lay = QVBoxLayout(self)
        lay.addWidget(self._list)
        lay.addLayout(row)
        lay.addWidget(box)

    def add_root(self, path: str) -> None:
        if path and path not in self.roots():
            self._list.addItem(path)

    def _on_add(self) -> None:
        path = QFileDialog.getExistingDirectory(self, "Add mesh search folder")
        self.add_root(path)

    def _on_remove(self) -> None:
        for item in self._list.selectedItems():
            self._list.takeItem(self._list.row(item))

    def roots(self) -> list[str]:
        return [self._list.item(i).text() for i in range(self._list.count())]
