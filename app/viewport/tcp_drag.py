"""Drag the TCP in the 3D viewport (U2.6).

A PyVista sphere widget sits at the TCP. Dragging it solves IK on the GUI thread
(seed = current q, target = dragged position + current TCP orientation; position only
when the robot has fewer than 6 arm DOF or the orientation cannot be met). Events are
throttled to about 30 Hz; the last dropped event is solved on release.

``on_tcp_drag(xyz)`` and ``on_release()`` are plain methods: tests call them directly.
"""
from __future__ import annotations

import time
from typing import Any, Callable

import numpy as np
from PySide6.QtCore import QObject, Signal

import core_bridge as cb
from app.gripper import gripper_dofs
from app.kin_utils import CompiledCache, current_q, dof_limits, write_q

THROTTLE_S = 1.0 / 30.0
POS_TOL = 1e-4          # m: a solution further from the target than this is "not reachable"
COLOR_OK = "#4fc3f7"
COLOR_BAD = "#ff3030"
COLOR_SELECTED = "#ffd54f"
IK_OPTIONS = {"max_iter": 60}


class TcpDragController(QObject):
    """Interactive TCP target. Owns the sphere widget; the model q is written here."""

    q_changed = Signal()              # live q written (viewport + panels refresh, not committed)
    committed = Signal()              # mouse released: final q kept (project modified)
    reachable_changed = Signal(bool)  # False: "Target not reachable"

    def __init__(self, viewport: Any, get_model: Callable[[], Any], parent=None,
                 clock: Callable[[], float] = time.monotonic):
        super().__init__(parent)
        self._viewport = viewport
        self._get_model = get_model
        self._clock = clock
        self._cache = CompiledCache()
        self._cache_model_id: int | None = None
        self._enabled = False
        self._widget: Any = None
        self._widget_model_id: int | None = None
        self._dragging = False
        self._reachable = True
        self._ref_rot: np.ndarray | None = None
        self._last_solve_t = -1e9
        self._pending: np.ndarray | None = None
        self._dirty = False             # q written since the last commit
        self.last_ik_ms = 0.0
        self.solve_count = 0
        self.drop_count = 0

    # ------------------------------------------------------------------ state

    @property
    def enabled(self) -> bool:
        return self._enabled

    @property
    def reachable(self) -> bool:
        return self._reachable

    @property
    def dragging(self) -> bool:
        return self._dragging

    @property
    def widget(self) -> Any:
        return self._widget

    def set_enabled(self, on: bool) -> None:
        on = bool(on)
        if on == self._enabled:
            return
        self._enabled = on
        if on:
            self.rebuild()
        else:
            self._finish()
            self._remove_widget()

    def invalidate(self) -> None:
        """Robot replaced or edited: drop the compiled model and recreate the handle."""
        self._cache.invalidate()
        self.rebuild()

    def rebuild(self) -> None:
        """Remove and recreate the handle at the current TCP (when enabled and a robot exists)."""
        self._remove_widget()
        self._ref_rot = None
        self._pending = None
        self._dragging = False
        self._set_reachable(True)
        if self._enabled and self._get_model() is not None:
            self._create_widget()

    def refresh(self) -> None:
        """q changed from elsewhere (slider, IK panel, playback): move the handle to the TCP."""
        if not self._enabled or self._dragging:
            return
        model = self._get_model()
        if model is None:
            self._remove_widget()
            return
        if self._widget is None or self._widget_model_id != id(model):
            self.rebuild()
            return
        self._ref_rot = None
        self._move_widget(self._tcp(model)[:3, 3])

    def dispose(self) -> None:
        """Before the plotter closes."""
        self._enabled = False
        self._remove_widget()

    # ------------------------------------------------------------------ drag handlers

    def on_tcp_drag(self, target_xyz) -> bool:
        """Handle moved to ``target_xyz`` (world, m). Returns True when IK ran, False if dropped."""
        target = np.asarray(target_xyz, dtype=float).reshape(3)
        model = self._get_model()
        if model is None or not self._enabled:
            return False
        self._dragging = True
        now = self._clock()
        if now - self._last_solve_t < THROTTLE_S:
            self._pending = target
            self.drop_count += 1
            return False
        self._pending = None
        self._last_solve_t = now
        self._solve(model, target)
        return True

    def on_release(self) -> None:
        """Mouse released: solve the last dropped target, commit q, put the handle on the TCP."""
        model = self._get_model()
        if model is not None and self._enabled and self._pending is not None:
            self._solve(model, self._pending)
        self._pending = None
        self._finish()
        if model is not None and self._widget is not None:
            self._move_widget(self._tcp(model)[:3, 3])
        self._set_widget_color(COLOR_OK)

    # ------------------------------------------------------------------ internals

    def _finish(self) -> None:
        self._dragging = False
        self._ref_rot = None
        if self._dirty:
            self._dirty = False
            self.committed.emit()

    def _tcp(self, model: Any) -> np.ndarray:
        return np.asarray(cb.fk(model)["tcp"], dtype=float)

    def _get_cm(self, model: Any):
        if self._cache_model_id != id(model):
            self._cache.invalidate()
            self._cache_model_id = id(model)
        return self._cache.get_cm(model)

    def _solve(self, model: Any, target_xyz: np.ndarray) -> bool:
        t0 = time.perf_counter()
        ok = False
        try:
            cm = self._get_cm(model)
            q0 = current_q(model)
            lo, hi = dof_limits(model)
            n_arm = len(q0)
            for g in gripper_dofs(model):      # gripper DOFs stay where they are
                lo[g.dof_index] = hi[g.dof_index] = q0[g.dof_index]
                n_arm -= 1
            if self._ref_rot is None:
                self._ref_rot = self._tcp(model)[:3, :3].copy()
            pose = np.eye(4)
            pose[:3, :3] = self._ref_rot
            pose[:3, 3] = target_xyz
            res = None
            if n_arm >= 6:
                res = cb.solve_ik(cm, lo, hi, pose, q0, dict(IK_OPTIONS))
                ok = bool(res["converged"]) and float(res["pos_error"]) <= POS_TOL
            if not ok:
                opts = dict(IK_OPTIONS, position_only=True)
                res = cb.solve_ik(cm, lo, hi, pose, q0, opts)
                ok = bool(res["converged"]) and float(res["pos_error"]) <= POS_TOL
            if ok:
                q = list(res["q"])
                for g in gripper_dofs(model):
                    q[g.dof_index] = q0[g.dof_index]
                write_q(model, q)
                self._dirty = True
        except Exception:
            ok = False
        self.last_ik_ms = (time.perf_counter() - t0) * 1e3
        self.solve_count += 1
        self._set_reachable(ok)
        if ok:
            self.q_changed.emit()
        return ok

    def _set_reachable(self, ok: bool) -> None:
        changed = ok != self._reachable
        self._reachable = ok
        self._set_widget_color(COLOR_OK if ok else COLOR_BAD)
        if changed:
            self.reachable_changed.emit(ok)

    # ---- widget (VTK sphere widget; created only with a real plotter)

    def _plotter(self) -> Any:
        return getattr(self._viewport, "plotter", None)

    def _radius(self, model: Any) -> float:
        rv = self._viewport.robot_view
        return max(0.012, 0.5 * float(rv.triad_length))

    def _create_widget(self) -> None:
        model = self._get_model()
        plotter = self._plotter()
        if model is None or plotter is None:
            return
        try:
            center = self._tcp(model)[:3, 3]
            self._widget = plotter.add_sphere_widget(
                self._widget_moved, center=tuple(float(v) for v in center),
                radius=self._radius(model), color=COLOR_OK, selected_color=COLOR_SELECTED,
                interaction_event="always", test_callback=False)
            self._widget.AddObserver("EndInteractionEvent", lambda *_: self.on_release())
            self._widget_model_id = id(model)
        except Exception:
            self._widget = None
            self._widget_model_id = None

    def _widget_moved(self, point) -> None:
        self.on_tcp_drag(point)

    def _move_widget(self, xyz) -> None:
        if self._widget is None:
            return
        try:
            self._widget.SetCenter(*[float(v) for v in xyz])
            self._widget.SetRadius(self._radius(self._get_model()))
            self._widget.Modified()
        except Exception:
            pass

    def _set_widget_color(self, color: str) -> None:
        if self._widget is None:
            return
        try:
            from pyvista import Color
            self._widget.GetSphereProperty().SetColor(Color(color).float_rgb)
        except Exception:
            pass

    def _remove_widget(self) -> None:
        w, self._widget, self._widget_model_id = self._widget, None, None
        if w is None:
            return
        try:
            w.Off()
            w.RemoveAllObservers()
            plotter = self._plotter()
            if plotter is not None and w in plotter.widgets.sphere_widgets:
                plotter.widgets.sphere_widgets.remove(w)
        except Exception:
            pass
