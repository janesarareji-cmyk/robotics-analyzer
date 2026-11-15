"""All targets of the project as labelled markers in the 3D view (D1.2).

``TargetMarkers.set_entries([(name, pose, status)])`` draws one sphere, one short tool-direction cone
and a text label per target. ``status`` = ``"ok"`` (green, reachable), ``"bad"`` (red, unreachable),
``"warn"`` (orange, near singular), ``"neutral"`` (blue, not checked yet). ``set_current(name)``
rings the target the demo / user works on in yellow. The selected target of the Target panel keeps
its own draggable frame (``TargetController``); these markers are display only and not pickable.
"""
from __future__ import annotations

from typing import Any, Sequence

import numpy as np

COLORS = {"ok": "#66bb6a", "bad": "#ff5252", "warn": "#ffa726", "neutral": "#4fc3f7"}
CURRENT_COLOR = "#ffd54f"


class TargetMarkers:
    def __init__(self, viewport: Any):
        self._viewport = viewport
        self._entries: list[tuple[str, np.ndarray, str]] = []
        self._actors: dict[str, list[Any]] = {}
        self._label_actor: Any = None
        self._ring: Any = None
        self._current: str | None = None
        self._visible = True

    # ---------------------------------------------------------------- state
    @property
    def entries(self) -> list[tuple[str, np.ndarray, str]]:
        return [(n, p.copy(), s) for n, p, s in self._entries]

    @property
    def names(self) -> list[str]:
        return [n for n, _p, _s in self._entries]

    def status_of(self, name: str) -> str | None:
        for n, _p, s in self._entries:
            if n == name:
                return s
        return None

    @property
    def current(self) -> str | None:
        return self._current

    @property
    def actor_count(self) -> int:
        return sum(len(v) for v in self._actors.values()) + (1 if self._label_actor is not None else 0)

    @property
    def visible(self) -> bool:
        return self._visible

    # ---------------------------------------------------------------- drawing
    def _plotter(self) -> Any:
        return getattr(self._viewport, "plotter", None)

    def _radius(self) -> float:
        rv = getattr(self._viewport, "robot_view", None)
        return max(0.008, 0.9 * float(getattr(rv, "triad_length", 0.05)))

    def set_entries(self, entries: Sequence[tuple[str, Any, str]]) -> None:
        self._entries = [(str(n), np.array(p, float), str(s)) for n, p, s in entries]
        if self._current not in self.names:
            self._current = None
        self._redraw()

    def set_status(self, name: str, status: str) -> None:
        self._entries = [(n, p, status if n == name else s) for n, p, s in self._entries]
        self._redraw()

    def set_current(self, name: str | None) -> None:
        self._current = name if name in self.names else None
        self._redraw()

    def set_visible(self, on: bool) -> None:
        self._visible = bool(on)
        self._redraw()

    def clear(self) -> None:
        self._entries = []
        self._current = None
        self._redraw()

    def _remove(self) -> None:
        plotter = self._plotter()
        if plotter is None:
            return
        for acts in self._actors.values():
            for a in acts:
                try:
                    plotter.remove_actor(a)
                except Exception:
                    pass
        self._actors = {}
        for a in (self._label_actor, self._ring):
            if a is not None:
                try:
                    plotter.remove_actor(a)
                except Exception:
                    pass
        self._label_actor = None
        self._ring = None

    def _redraw(self) -> None:
        plotter = self._plotter()
        if plotter is None:
            return
        try:
            self._remove()
            if self._visible and self._entries:
                import pyvista as pv
                r = self._radius()
                pts, labels = [], []
                for name, pose, status in self._entries:
                    col = COLORS.get(status, COLORS["neutral"])
                    sph = pv.Sphere(radius=r, center=(0, 0, 0), theta_resolution=20, phi_resolution=20)
                    cone = pv.Cone(center=(0, 0, -r * 2.2), direction=(0, 0, 1), height=r * 2.4,
                                   radius=r * 0.7, resolution=20)
                    acts = []
                    for mesh, opacity in ((sph, 0.95), (cone, 0.6)):
                        a = plotter.add_mesh(mesh, color=col, opacity=opacity, reset_camera=False,
                                             pickable=False)
                        a.user_matrix = pose
                        acts.append(a)
                    self._actors[name] = acts
                    pts.append(pose[:3, 3] + np.array([0.0, 0.0, 2.2 * r]))
                    labels.append(name)
                self._label_actor = plotter.add_point_labels(
                    np.array(pts), labels, font_size=14, text_color="white", shape_color="#202030",
                    shape_opacity=0.75, point_size=0, always_visible=True, reset_camera=False,
                    pickable=False, render=False, show_points=False)
                if self._current is not None:
                    for name, pose, _s in self._entries:
                        if name == self._current:
                            ring = pv.Sphere(radius=r * 1.6, center=(0, 0, 0))
                            self._ring = plotter.add_mesh(ring, color=CURRENT_COLOR, style="wireframe",
                                                          opacity=0.9, reset_camera=False,
                                                          pickable=False)
                            self._ring.user_matrix = pose
            rv = getattr(self._viewport, "robot_view", None)
            if rv is not None:
                rv._render()
        except Exception:
            pass

    def dispose(self) -> None:
        self._entries = []
        self._remove()
