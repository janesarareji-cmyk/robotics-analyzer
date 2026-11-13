"""Gripper detection (U2.4). No Qt dependency.

A DOF is a gripper DOF when its (master, non-mimic, non-fixed) joint name matches
``drive_joint|finger|gripper`` (case-insensitive). Open / closed values:
prismatic: open = upper limit, closed = lower limit; revolute: open = lower limit
(0 for the usual 0..x range), closed = upper limit.
"""
from __future__ import annotations

import re
from dataclasses import dataclass

NAME_RE = re.compile(r"drive_joint|finger|gripper", re.IGNORECASE)


@dataclass
class GripperDof:
    dof_index: int
    name: str
    open_value: float
    closed_value: float
    joint_index: int = -1

    def value_at(self, closed_fraction: float) -> float:
        """q for an opening slider position: 0 = open ... 1 = closed (linear)."""
        f = min(max(float(closed_fraction), 0.0), 1.0)
        return self.open_value + f * (self.closed_value - self.open_value)

    def fraction_of(self, q: float) -> float:
        span = self.closed_value - self.open_value
        if span == 0.0:
            return 0.0
        return min(max((float(q) - self.open_value) / span, 0.0), 1.0)


def gripper_dofs(model) -> list[GripperDof]:
    """Gripper DOFs of ``model`` in DOF order (empty for arm-only robots)."""
    out: list[GripperDof] = []
    for j_idx, q_idx in enumerate(model.q_index_map()):
        if q_idx < 0:
            continue
        j = model.joints[j_idx]
        if j.mimic is not None or not NAME_RE.search(j.name):
            continue
        lo, hi = float(j.limits.lower), float(j.limits.upper)
        if str(j.type).split(".")[-1].lower() == "prismatic":
            open_v, closed_v = hi, lo
        else:
            open_v, closed_v = lo, hi
        out.append(GripperDof(int(q_idx), str(j.name), open_v, closed_v, j_idx))
    out.sort(key=lambda g: g.dof_index)
    return out
