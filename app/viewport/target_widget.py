"""Target frame in the 3D viewport (U4a.1).

``TargetController`` owns one target pose (4x4) and draws it: RGB triad plus a small cone along
the tool z axis (the cone colour shows the solve status: neutral, reachable, unreachable,
near singularity). A sphere widget at the origin drags the position. Rotation: three step
buttons per axis in the Target panel (and typed roll / pitch / yaw). Rings were left out: keyboard
and buttons are lighter and testable. "Place on surface": click mode picks the surface under the
mouse (VTK cell picker) and puts the target there with tool z = -normal.

Plain methods for tests: ``move_to(xyz)``, ``rotate(axis, deg)``, ``place_at(point, normal)``,
``pick_at_display(x, y)``.
"""
from __future__ import annotations

from typing import Any, Callable

import numpy as np
from PySide6.QtCore import QObject, Signal

from app.target import pose_on_surface, rotate_pose

COLOR_NEUTRAL = "#4fc3f7"
COLOR_OK = "#66bb6a"
COLOR_BAD = "#ff5252"
COLOR_WARN = "#ffa726"
COLOR_SELECTED = "#ffd54f"
STATUS_COLORS = {"neutral": COLOR_NEUTRAL, "ok": COLOR_OK, "bad": COLOR_BAD, "warn": COLOR_WARN}


class TargetController(QObject):
    """One draggable target frame. ``pose_changed`` fires on every change (also typed edits)."""

    pose_changed = Signal(object)        # 4x4 ndarray
    committed = Signal(object)           # drag released
    picked = Signal(object)              # click mode: 4x4 pose placed on the surface

    def __init__(self, viewport: Any, get_ref_rot: Callable[[], Any] | None = None, parent=None):
        super().__init__(parent)
        self._viewport = viewport
        self._get_ref_rot = get_ref_rot
        self._pose: np.ndarray | None = None
        self._visible = False
        self._status = "neutral"
        self._widget: Any = None
        self._actors: list[Any] = []
        self._cone_actor: Any = None
        self._pick_mode = False
        self._dragging = False

    # ------------------------------------------------------------------ state

    @property
    def pose(self) -> np.ndarray | None:
        return None if self._pose is None else self._pose.copy()

    @property
    def visible(self) -> bool:
        return self._visible

    @property
    def status(self) -> str:
        return self._status

    @property
    def pick_mode(self) -> bool:
        return self._pick_mode

    @property
    def widget(self) -> Any:
        return self._widget

    def _plotter(self) -> Any:
        return getattr(self._viewport, "plotter", None)

    def _rv(self) -> Any:
        return getattr(self._viewport, "robot_view", None)

    def _size(self) -> float:
        rv = self._rv()
        return max(0.02, 2.0 * float(getattr(rv, "triad_length", 0.05)))

    # ------------------------------------------------------------------ pose

    def set_pose(self, pose: Any, emit: bool = True) -> None:
        """Set the target pose (4x4) and redraw."""
        p = np.array(pose, float)
        if p.shape != (4, 4) or not np.all(np.isfinite(p)):
            raise ValueError("target pose must be a finite 4x4 matrix")
        self._pose = p
        self._redraw()
        if emit:
            self.pose_changed.emit(p.copy())

    def move_to(self, xyz: Any) -> None:
        if self._pose is None:
            return
        p = self._pose.copy()
        p[:3, 3] = np.asarray(xyz, float).reshape(3)
        self.set_pose(p)

    def rotate(self, axis: str, deg: float) -> None:
        """Rotate about the target's own x / y / z axis."""
        if self._pose is None:
            return
        self.set_pose(rotate_pose(self._pose, axis, deg, local=True))

    def place_at(self, point: Any, normal: Any) -> np.ndarray:
        """Target on a surface point, tool z = -normal. Keeps the old x direction where possible."""
        ref = None
        if self._pose is not None:
            ref = self._pose[:3, :3]
        elif self._get_ref_rot is not None:
            ref = self._get_ref_rot()
        p = pose_on_surface(point, normal, ref)
        self.set_pose(p)
        self.committed.emit(p.copy())
        return p

    def set_status(self, status: str) -> None:
        """``neutral`` / ``ok`` / ``bad`` / ``warn``: colour of the tool cone."""
        if status not in STATUS_COLORS:
            raise ValueError(f"unknown status {status!r}")
        self._status = status
        self._paint_cone()

    def show(self, on: bool = True) -> None:
        self._visible = bool(on)
        if self._visible:
            self._redraw()
        else:
            self._clear_actors()
            self._remove_widget()

    # ------------------------------------------------------------------ picking

    def pick_at_display(self, x: float, y: float):
        """Surface point and normal under display pixel ``(x, y)``: ``(point, normal)`` or None."""
        plotter = self._plotter()
        if plotter is None:
            return None
        try:
            import vtk
            picker = vtk.vtkCellPicker()
            picker.SetTolerance(0.001)
            for a in self._actors:                 # never pick the target itself
                picker.AddIgnoredProp(a) if hasattr(picker, "AddIgnoredProp") else None
            if not picker.Pick(float(x), float(y), 0.0, plotter.renderer):
                return None
            return (np.array(picker.GetPickPosition(), float),
                    np.array(picker.GetPickNormal(), float))
        except Exception:
            return None

    def set_pick_mode(self, on: bool) -> None:
        """Click on a surface places the target there (a click, not a camera drag)."""
        on = bool(on)
        if on == self._pick_mode:
            return
        self._pick_mode = on
        plotter = self._plotter()
        if plotter is None:
            return
        try:
            if on:
                plotter.track_click_position(self._on_click, side="left", double=False, viewport=False)
            else:
                plotter.untrack_click_position(side="left")
        except Exception:
            pass

    def _on_click(self, pos) -> None:
        if not self._pick_mode:
            return
        hit = self.pick_at_display(pos[0], pos[1])
        if hit is None:
            return
        p = self.place_at(*hit)
        self.picked.emit(p.copy())

    # ------------------------------------------------------------------ drawing

    def _mesh_triad(self):
        rv = self._rv()
        return rv._create_triad_mesh(self._size())

    def _redraw(self) -> None:
        if not self._visible or self._pose is None:
            return
        plotter = self._plotter()
        if plotter is None:
            return
        try:
            if not self._actors:
                import pyvista as pv
                tri = plotter.add_mesh(self._mesh_triad(), scalars="colors", rgb=True,
                                       line_width=5.0, reset_camera=False, pickable=False)
                s = self._size()
                # tool body: apex at the TCP origin, base behind it on -z (tool points along +z)
                cone = pv.Cone(center=(0, 0, -s * 0.45 * 0.5), direction=(0, 0, 1),
                               height=s * 0.45, radius=s * 0.16, resolution=24)
                self._cone_actor = plotter.add_mesh(cone, color=STATUS_COLORS[self._status],
                                                    opacity=0.85, reset_camera=False,
                                                    pickable=False)
                self._actors = [tri, self._cone_actor]
            for a in self._actors:
                a.user_matrix = self._pose
            self._move_widget()
            self._viewport.robot_view._render()
        except Exception:
            pass

    def _paint_cone(self) -> None:
        if self._cone_actor is None:
            return
        try:
            from pyvista import Color
            self._cone_actor.GetProperty().SetColor(Color(STATUS_COLORS[self._status]).float_rgb)
            self._viewport.robot_view._render()
        except Exception:
            pass

    def _clear_actors(self) -> None:
        plotter = self._plotter()
        for a in self._actors:
            try:
                plotter.remove_actor(a)
            except Exception:
                pass
        self._actors = []
        self._cone_actor = None

    # ---- sphere widget (position drag)

    def _move_widget(self) -> None:
        plotter = self._plotter()
        if plotter is None or self._pose is None:
            return
        if self._widget is None:
            try:
                self._widget = plotter.add_sphere_widget(
                    self._widget_moved, center=tuple(float(v) for v in self._pose[:3, 3]),
                    radius=max(0.012, 0.35 * self._size()), color=COLOR_NEUTRAL,
                    selected_color=COLOR_SELECTED, interaction_event="always",
                    test_callback=False)
                self._widget.AddObserver("EndInteractionEvent", lambda *_: self._on_release())
            except Exception:
                self._widget = None
            return
        try:
            self._widget.SetCenter(*[float(v) for v in self._pose[:3, 3]])
            self._widget.Modified()
        except Exception:
            pass

    def _widget_moved(self, point) -> None:
        if self._pose is None or not self._visible:
            return
        self._dragging = True
        p = self._pose.copy()
        p[:3, 3] = np.asarray(point, float)
        self._pose = p
        for a in self._actors:
            a.user_matrix = p
        self.pose_changed.emit(p.copy())

    def _on_release(self) -> None:
        if self._dragging and self._pose is not None:
            self._dragging = False
            self.committed.emit(self._pose.copy())

    def _remove_widget(self) -> None:
        w, self._widget = self._widget, None
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

    def dispose(self) -> None:
        """Before the plotter closes."""
        self._visible = False
        self.set_pick_mode(False)
        self._remove_widget()
        self._actors = []
        self._cone_actor = None
