"""
app/panels/model_panel.py
Robot model tree panel.
R3.4: link/joint tree with active-chain highlighting and selection signal.
Also holds small topology helpers shared by the UI (joint_links, is_plain_chain).
"""

from __future__ import annotations

from typing import Any

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QLabel, QTreeWidget, QTreeWidgetItem, QHeaderView)
from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QBrush, QColor, QFont

import core_bridge as cb

ROLE_KIND = Qt.ItemDataRole.UserRole
ROLE_INDEX = Qt.ItemDataRole.UserRole + 1

_COLOR_ACTIVE = QColor("#e8e8e8")
_COLOR_INACTIVE = QColor("#777777")

_TYPE_NAMES = {
    cb.JointType.Revolute: "revolute",
    cb.JointType.Prismatic: "prismatic",
    cb.JointType.Fixed: "fixed",
}


def joint_type_name(joint: Any) -> str:
    return _TYPE_NAMES.get(joint.type, "revolute")


def joint_links(model: Any) -> list[tuple[int, int]]:
    """
    Effective (parent_link, child_link) per joint.
    Delegates to C++ kin::effective_links (single legacy rule, R4.1).
    """
    return [(int(p), int(c)) for p, c in cb.effective_links(model)]


def is_plain_chain(model: Any) -> bool:
    """True when for every joint i: parent_link == i and child_link == i + 1."""
    if model is None:
        return True
    return all(p == i and c == i + 1 for i, (p, c) in enumerate(joint_links(model)))


def resolved_tcp_link(model: Any) -> int:
    """tcp_link with -1 resolved to the last link."""
    n_links = len(model.links)
    t = model.tcp_link
    return n_links - 1 if t == -1 else t


def active_link_set(model: Any, active_joints: list[int]) -> set[int]:
    """Links on the active chain: link 0 plus the child link of each active joint."""
    links = joint_links(model)
    res = {0}
    for k in active_joints:
        if 0 <= k < len(links):
            res.add(links[k][1])
    return res


class ModelPanel(QWidget):
    """Robot model panel: displays the robot link/joint tree (read-only)."""

    selection_changed = Signal(str, int)  # kind ("link" | "joint"), index

    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(2)

        label = QLabel("Robot Model")
        label.setStyleSheet("font-weight: bold; color: #aaaaaa;")
        layout.addWidget(label)

        self._tree = QTreeWidget()
        self._tree.setHeaderLabel("No robot loaded")
        self._tree.setStyleSheet("background: #1e1e1e; color: #cccccc; border: none;")
        self._tree.setIndentation(12)
        self._tree.setTextElideMode(Qt.TextElideMode.ElideNone)
        self._tree.header().setStretchLastSection(False)
        self._tree.header().setSectionResizeMode(
            0, QHeaderView.ResizeMode.ResizeToContents)
        self._tree.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        self._tree.itemSelectionChanged.connect(self._on_selection)
        layout.addWidget(self._tree)

        self._link_items: dict[int, QTreeWidgetItem] = {}
        self._joint_items: dict[int, QTreeWidgetItem] = {}
        self._updating = False

    # ------------------------------------------------------------------
    @property
    def tree(self) -> QTreeWidget:
        return self._tree

    def link_item(self, index: int) -> QTreeWidgetItem | None:
        return self._link_items.get(index)

    def joint_item(self, index: int) -> QTreeWidgetItem | None:
        return self._joint_items.get(index)

    @property
    def link_item_count(self) -> int:
        return len(self._link_items)

    @staticmethod
    def is_active_item(item: QTreeWidgetItem) -> bool:
        """True when the item is drawn as part of the active chain (bold)."""
        return item.font(0).bold()

    # ------------------------------------------------------------------
    def set_model(self, model: Any, active_joints: list[int] | None = None) -> None:
        """Rebuild the tree. `active_joints` = joint indices of the active chain."""
        self._updating = True
        try:
            self._tree.clear()
            self._link_items.clear()
            self._joint_items.clear()
            if model is None or len(model.links) == 0:
                self._tree.setHeaderLabel("No robot loaded")
                return
            self._tree.setHeaderLabel(model.metadata.name or "Robot")

            pairs = joint_links(model)
            if active_joints is None:
                active_joints = list(range(len(model.joints)))
            active_j = set(active_joints)
            active_l = active_link_set(model, list(active_joints))

            children: dict[int, list[int]] = {}
            for k, (p, _c) in enumerate(pairs):
                children.setdefault(p, []).append(k)

            def style(item: QTreeWidgetItem, active: bool) -> None:
                f = QFont(item.font(0))
                f.setBold(active)
                item.setFont(0, f)
                item.setForeground(0, QBrush(_COLOR_ACTIVE if active else _COLOR_INACTIVE))

            def add_link(idx: int, parent_item: QTreeWidgetItem | None, seen: set[int]) -> None:
                if idx in seen or not (0 <= idx < len(model.links)):
                    return
                seen.add(idx)
                name = model.links[idx].name or f"link{idx}"
                item = QTreeWidgetItem([name])
                item.setData(0, ROLE_KIND, "link")
                item.setData(0, ROLE_INDEX, idx)
                style(item, idx in active_l)
                if parent_item is None:
                    self._tree.addTopLevelItem(item)
                else:
                    parent_item.addChild(item)
                self._link_items[idx] = item
                for k in children.get(idx, []):
                    j = model.joints[k]
                    label = f"{j.name} [{joint_type_name(j)}]"
                    if j.mimic is not None:
                        label += f" mimic→{j.mimic.joint}"
                    jitem = QTreeWidgetItem([label])
                    jitem.setData(0, ROLE_KIND, "joint")
                    jitem.setData(0, ROLE_INDEX, k)
                    style(jitem, k in active_j)
                    item.addChild(jitem)
                    self._joint_items[k] = jitem
                    add_link(pairs[k][1], jitem, seen)

            add_link(0, None, set())
            self._tree.expandAll()
        finally:
            self._updating = False

    def select(self, kind: str, index: int) -> None:
        """Select an item programmatically (emits selection_changed)."""
        item = (self._link_items if kind == "link" else self._joint_items).get(index)
        if item is not None:
            self._tree.setCurrentItem(item)

    def _on_selection(self) -> None:
        if self._updating:
            return
        items = self._tree.selectedItems()
        if not items:
            return
        item = items[0]
        kind = item.data(0, ROLE_KIND)
        idx = item.data(0, ROLE_INDEX)
        if kind is None or idx is None:
            return
        self.selection_changed.emit(str(kind), int(idx))
