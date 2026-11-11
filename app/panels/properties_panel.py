"""
app/panels/properties_panel.py
Properties dock panel.
R3.4: read-only properties of the link or joint selected in the robot tree.
"""

from __future__ import annotations

from typing import Any

from PySide6.QtWidgets import QWidget, QVBoxLayout, QLabel
from PySide6.QtCore import Qt

import core_bridge as cb
from app.panels.model_panel import joint_links, joint_type_name

_PLACEHOLDER = "Select an item to view its properties."


def _fmt(vals: Any) -> str:
    return "(" + ", ".join(f"{float(v):.6g}" for v in vals) + ")"


class PropertiesPanel(QWidget):
    """Properties panel: displays properties of the selected item."""

    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)

        label = QLabel("Properties")
        label.setStyleSheet("font-weight: bold; color: #aaaaaa;")
        layout.addWidget(label)

        self._body = QLabel(_PLACEHOLDER)
        self._body.setWordWrap(True)
        self._body.setAlignment(Qt.AlignmentFlag.AlignTop)
        self._body.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        self._body.setStyleSheet("color: #666666; padding: 8px;")
        layout.addWidget(self._body)
        layout.addStretch()

    @property
    def text(self) -> str:
        """Current body text (placeholder when nothing is selected)."""
        return self._body.text()

    def clear(self) -> None:
        self._body.setText(_PLACEHOLDER)
        self._body.setStyleSheet("color: #666666; padding: 8px;")

    def show_item(self, model: Any, kind: str, index: int) -> None:
        """Show read-only properties of a joint or link of `model`."""
        if model is None:
            self.clear()
            return
        lines: list[str] = []
        if kind == "joint" and 0 <= index < len(model.joints):
            j = model.joints[index]
            p, c = joint_links(model)[index]

            def lname(i: int) -> str:
                return model.links[i].name if 0 <= i < len(model.links) else "?"

            lines.append(f"Joint: {j.name}")
            lines.append(f"Index: {index}")
            lines.append(f"Type: {joint_type_name(j)}")
            is_oa = j.kinematics == cb.JointKinematics.OriginAxis
            lines.append(f"Kinematics: {'origin_axis' if is_oa else 'dh'}")
            lines.append(f"Parent link: {lname(p)}")
            lines.append(f"Child link: {lname(c)}")
            if is_oa:
                lines.append(f"Origin xyz: {_fmt(j.origin.xyz)}")
                lines.append(f"Origin rpy: {_fmt(j.origin.rpy)}")
                lines.append(f"Axis: {_fmt(j.axis)}")
            lines.append(f"Limits: lower {j.limits.lower:.6g}, upper {j.limits.upper:.6g}")
            if j.mimic is not None:
                m = j.mimic
                lines.append(
                    f"Mimic: {m.joint} (multiplier {m.multiplier:.6g}, offset {m.offset:.6g})"
                )
        elif kind == "link" and 0 <= index < len(model.links):
            lk = model.links[index]
            lines.append(f"Link: {lk.name}")
            lines.append(f"Index: {index}")
            lines.append(f"Mass: {lk.inertia.mass:.6g}")
        else:
            self.clear()
            return
        self._body.setText("\n".join(lines))
        self._body.setStyleSheet("color: #cccccc; padding: 8px;")
