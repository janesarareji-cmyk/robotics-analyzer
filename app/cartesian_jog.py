"""U5.7: Cartesian jog. Move the tool by a small step along X/Y/Z or turn it about X/Y/Z.

The step is applied to the current tool pose in the base frame or the tool frame, solved with IK from
the current q (so the motion stays continuous) and passed through the collision guard. Pure logic, no
widgets: the panel is ``app/panels/jog_panel.py``.
"""
from __future__ import annotations

from typing import Any, Callable

import numpy as np
from PySide6.QtCore import QObject, Signal

import core_bridge as cb
from app.gripper import gripper_dofs
from app.kin_utils import CompiledCache, current_q, dof_limits, write_q

LIN_STEPS_MM = (1.0, 10.0, 50.0)
ANG_STEPS_DEG = (1.0, 5.0, 15.0)
AXES = ("x", "y", "z", "rx", "ry", "rz")
POS_TOL = 1e-4          # m
ROT_TOL = 1e-3          # rad
IK_OPTIONS = {"max_iter": 60}


def rot_axis(axis: int, angle: float) -> np.ndarray:
    c, s = np.cos(angle), np.sin(angle)
    if axis == 0:
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], float)
    if axis == 1:
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], float)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], float)


def jogged_pose(T: np.ndarray, axis: str, sign: int, lin_m: float, ang_rad: float, frame: str) -> np.ndarray:
    """New 4x4 tool pose: ``T`` moved one step. ``axis`` in x y z rx ry rz, ``sign`` +1 / -1."""
    T = np.asarray(T, float)
    out = T.copy()
    k = "xyz".index(axis[-1])
    if axis in ("x", "y", "z"):
        d = np.zeros(3)
        d[k] = sign * lin_m
        out[:3, 3] = T[:3, 3] + (T[:3, :3] @ d if frame == "tool" else d)
    else:
        Rd = rot_axis(k, sign * ang_rad)
        out[:3, :3] = T[:3, :3] @ Rd if frame == "tool" else Rd @ T[:3, :3]
    return out


class CartesianJog(QObject):
    q_changed = Signal()          # q written to the model (GUI refreshes)
    blocked = Signal(str)         # the collision guard held the robot back
    failed = Signal(str)          # IK found no pose (out of reach, singular, limit)

    def __init__(self, get_model: Callable[[], Any], parent: QObject | None = None):
        super().__init__(parent)
        self._get_model = get_model
        self._cache = CompiledCache()
        self._cache_model_id = None
        self._guard: Callable | None = None
        self.frame = "base"
        self.lin_mm = 10.0
        self.ang_deg = 5.0
        self.last_status = ""

    def set_guard(self, fn: Callable | None) -> None:
        """Collision guard ``fn(q_new, q_old=None) -> (q_final, blocked)``."""
        self._guard = fn

    def invalidate(self) -> None:
        self._cache.invalidate()

    def set_frame(self, frame: str) -> None:
        self.frame = "tool" if frame == "tool" else "base"

    def jog(self, axis: str, sign: int) -> bool:
        """One step. True when the robot moved (possibly stopped early by the guard)."""
        model = self._get_model()
        if model is None or axis not in AXES:
            return False
        if self._cache_model_id != id(model):
            self._cache.invalidate()
            self._cache_model_id = id(model)
        try:
            cm = self._cache.get_cm(model)
            q0 = current_q(model)
            T = np.asarray(cb.fk(model)["tcp"], float)
            goal = jogged_pose(T, axis, 1 if sign >= 0 else -1, self.lin_mm / 1000.0,
                               np.radians(self.ang_deg), self.frame)
            lo, hi = dof_limits(model)
            n_arm = len(q0)
            for g in gripper_dofs(model):
                lo[g.dof_index] = hi[g.dof_index] = q0[g.dof_index]
                n_arm -= 1
            res = cb.solve_ik(cm, lo, hi, goal, q0, dict(IK_OPTIONS))
            ok = bool(res["converged"]) and float(res["pos_error"]) <= POS_TOL
            if not ok and n_arm < 6 and axis in ("x", "y", "z"):      # arms with fewer than 6 DOF
                res = cb.solve_ik(cm, lo, hi, goal, q0, dict(IK_OPTIONS, position_only=True))
                ok = bool(res["converged"]) and float(res["pos_error"]) <= POS_TOL
        except Exception as exc:                                       # noqa: BLE001
            self._fail(f"Cannot move: {exc}")
            return False
        if not ok:
            self._fail("The tool cannot go further that way (out of reach, a joint limit or a straight arm).")
            return False
        q = list(res["q"])
        for g in gripper_dofs(model):
            q[g.dof_index] = q0[g.dof_index]
        blocked = False
        if self._guard is not None:
            q, blocked = self._guard(q, q0)
        write_q(model, q)
        self.last_status = ""
        self.q_changed.emit()
        if blocked:
            owner = getattr(self._guard, "__self__", None)
            text = getattr(owner, "text", "") or "Stopped: the robot would touch something."
            self.last_status = text
            self.blocked.emit(text)
        return True

    def _fail(self, text: str) -> None:
        self.last_status = text
        self.failed.emit(text)
