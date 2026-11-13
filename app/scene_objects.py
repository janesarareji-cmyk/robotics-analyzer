"""Scene objects (U3.6): default placement and the drag widget of an environment object.

Objects are ``EnvShape`` items of ``project.collision.environment`` (saved in the project).
``default_object`` gives size and position of a new object: 0.1 m, on the floor, 0.4 m in front
of the robot base (along the base x axis). ``ObjectDragController`` shows a PyVista box widget
on the selected object; dragging it moves the object (position only; size and rotation are typed
in the Collision panel).
"""
from __future__ import annotations

import logging
from typing import Any, Callable

import numpy as np
from PySide6.QtCore import QObject, Signal

from app.planning.collision import FLOOR_CLEARANCE, EnvShape

log = logging.getLogger(__name__)

DEFAULT_SIZE = 0.1
FLOOR_LIFT = 0.0005          # m: new objects float 0.5 mm over the floor plane (no boundary contact)
FRONT_DISTANCE = 0.4


def default_params(kind: str, size: float = DEFAULT_SIZE) -> dict:
    """Default shape parameters: box ``size`` cube, cylinder radius ``size/2`` length ``size``."""
    if kind == "box":
        return {"size": [size, size, size]}
    if kind == "sphere":
        return {"radius": size / 2.0}
    if kind == "cylinder":
        return {"radius": size / 2.0, "length": size}
    raise ValueError(f"unknown object kind {kind!r}")


def half_height(kind: str, params: dict) -> float:
    if kind == "box":
        return 0.5 * float(params["size"][2])
    if kind == "sphere":
        return float(params["radius"])
    return 0.5 * float(params["length"])


def default_object(kind: str, model: Any) -> tuple:
    """``(params, xyz)`` of a new object of ``kind`` for ``model`` (no model: in front of origin)."""
    params = default_params(kind)
    base = np.eye(4)
    if model is not None and len(model.joints):
        try:
            import core_bridge as cb
            base = np.asarray(cb.fk(model)["frames"][0], dtype=float)
        except Exception as exc:                    # fall back to the origin
            log.warning("object placement without FK: %s", exc)
    fwd = base[:3, 0].copy()
    fwd[2] = 0.0
    n = float(np.linalg.norm(fwd))
    fwd = fwd / n if n > 1e-9 else np.array([1.0, 0.0, 0.0])
    xy = base[:2, 3] + FRONT_DISTANCE * fwd[:2]
    z = float(base[2, 3]) - FLOOR_CLEARANCE + FLOOR_LIFT + half_height(kind, params)
    return params, [float(xy[0]), float(xy[1]), z]


class ObjectDragController(QObject):
    """Box widget on the selected environment object; ``moved(index, xyz)`` on release."""

    moved = Signal(int, object)

    def __init__(self, viewport: Any, parent=None):
        super().__init__(parent)
        self._viewport = viewport
        self._widget: Any = None
        self._index = -1

    @property
    def index(self) -> int:
        return self._index

    @property
    def has_widget(self) -> bool:
        return self._widget is not None

    def select(self, index: int, shape: EnvShape | None) -> None:
        """Show the widget on ``shape`` (None or bad index: hide it)."""
        self._remove()
        self._index = int(index) if shape is not None else -1
        if shape is None:
            return
        try:
            b = self._bounds(shape)
            plotter = self._viewport.plotter
            self._widget = plotter.add_box_widget(
                self.on_box, bounds=b, factor=1.0, rotation_enabled=False, color="#ffd54f",
                interaction_event="end")
        except Exception as exc:                    # no interactor (tests, offscreen): typed edit only
            log.info("object drag widget not shown: %s", exc)
            self._widget = None

    def clear(self) -> None:
        self._remove()
        self._index = -1

    def _remove(self) -> None:
        if self._widget is not None:
            try:
                self._viewport.plotter.clear_box_widgets()
            except Exception:
                pass
            self._widget = None

    @staticmethod
    def _bounds(shape: EnvShape) -> tuple:
        s = shape.to_shape()
        r = float(s.bounding_radius())
        x, y, z = shape.xyz
        return (x - r, x + r, y - r, y + r, z - r, z + r)

    def on_box(self, poly: Any) -> None:
        """Widget callback: the box centre is the new object position."""
        if self._index < 0:
            return
        c = np.asarray(getattr(poly, "center", poly), dtype=float)
        self.moved.emit(self._index, [float(v) for v in c[:3]])
