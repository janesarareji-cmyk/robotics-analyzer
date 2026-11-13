"""Draggable via points of the curve (U4b.3).

One PyVista sphere widget per via point, same technique as the TCP drag and the target handle.
``points_changed`` fires on every drag event (the Curve panel debounces the recompute at 150 ms,
which is also the 30 Hz-or-slower throttle); ``committed`` when the mouse is released.
Plain methods for tests: ``set_points``, ``move_point``.
"""
from __future__ import annotations

from typing import Any

import numpy as np
from PySide6.QtCore import QObject, Signal

COLOR = "#ffd54f"
COLOR_SELECTED = "#ff9800"


class ViaHandles(QObject):
    point_moved = Signal(int, object)          # index, xyz (3,) ndarray, during a drag
    committed = Signal(int)                    # index, mouse released

    def __init__(self, viewport: Any, parent=None):
        super().__init__(parent)
        self._viewport = viewport
        self._points: list[np.ndarray] = []
        self._widgets: list[Any] = []
        self._visible = False
        self._radius = 0.012

    @property
    def points(self) -> list:
        return [p.copy() for p in self._points]

    @property
    def widgets(self) -> list:
        return list(self._widgets)

    @property
    def visible(self) -> bool:
        return self._visible

    def _plotter(self) -> Any:
        return getattr(self._viewport, "plotter", None)

    def set_radius(self, r: float) -> None:
        self._radius = max(0.004, float(r))

    def set_points(self, points: Any) -> None:
        """Replace the handles (positions in the world frame)."""
        self._points = [np.asarray(p, float).reshape(3).copy() for p in points]
        self._rebuild()

    def show(self, on: bool) -> None:
        on = bool(on)
        if on == self._visible:
            return
        self._visible = on
        self._rebuild()

    def _rebuild(self) -> None:
        self._remove_all()
        plotter = self._plotter()
        if not self._visible or plotter is None:
            return
        for i, p in enumerate(self._points):
            try:
                w = plotter.add_sphere_widget(
                    lambda pt, i=i: self._moved(i, pt), center=tuple(float(v) for v in p),
                    radius=self._radius, color=COLOR, selected_color=COLOR_SELECTED,
                    interaction_event="always", test_callback=False)
                w.AddObserver("EndInteractionEvent", lambda *_a, i=i: self.committed.emit(i))
                self._widgets.append(w)
            except Exception:
                pass

    def _moved(self, i: int, pt: Any) -> None:
        if not self._visible or not (0 <= i < len(self._points)):
            return
        self._points[i] = np.asarray(pt, float).reshape(3).copy()
        self.point_moved.emit(i, self._points[i].copy())

    def move_point(self, i: int, xyz: Any) -> None:
        """Move one handle as a drag would (tests): widget centre and ``point_moved``."""
        self._moved(i, xyz)
        if 0 <= i < len(self._widgets):
            try:
                self._widgets[i].SetCenter(*[float(v) for v in xyz])
            except Exception:
                pass

    def _remove_all(self) -> None:
        plotter = self._plotter()
        for w in self._widgets:
            try:
                w.Off()
                w.RemoveAllObservers()
                if plotter is not None and w in plotter.widgets.sphere_widgets:
                    plotter.widgets.sphere_widgets.remove(w)
            except Exception:
                pass
        self._widgets = []

    def dispose(self) -> None:
        self._visible = False
        self._remove_all()
        self._points = []
