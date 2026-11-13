"""Ghost preview of ranked IK solutions (U4a.3).

``TargetPreview`` draws translucent robot copies through ``RobotView.add_ghost`` (keys
``("ik", k)``): green (#66bb6a) for a valid solution, red (#ff5252) for an invalid one. The
selected solution is drawn alone, or with "show all" up to 8 solutions where the selected one is
brighter (opacity 0.55 against 0.25). ``target_status`` gives the colour state of the target
triad: ``ok`` (a valid solution), ``bad`` (none), ``warn`` (best valid one is near singularity).
"""
from __future__ import annotations

from typing import Any, Sequence

import core_bridge as cb

GHOST_OK = "#66bb6a"
GHOST_BAD = "#ff5252"
OPACITY = 0.35
OPACITY_SELECTED = 0.55
OPACITY_OTHER = 0.22
MAX_ALL = 8


def target_status(solutions: Sequence[Any]) -> str:
    """``"bad"`` no valid solution, ``"warn"`` best valid one near singularity, else ``"ok"``."""
    best = next((s for s in solutions if s.valid), None)
    if best is None:
        return "bad"
    return "warn" if best.near_singular else "ok"


class TargetPreview:
    """Owns the ghost keys of the IK preview; other ghosts (Ghosts dock) are never touched."""

    def __init__(self, get_view, get_model):
        self._get_view = get_view
        self._get_model = get_model
        self._keys: list = []
        self.selected = -1
        self.show_all = False

    @property
    def keys(self) -> list:
        return list(self._keys)

    def clear(self) -> None:
        rv = self._get_view()
        if rv is not None:
            for k in self._keys:
                rv.remove_ghost(k, render=False)
            if self._keys:
                rv._render()
        self._keys = []

    def show(self, solutions: Sequence[Any], selected: int = -1, show_all: bool | None = None) -> None:
        """Draw the preview. ``selected`` indexes ``solutions`` (-1: nothing selected)."""
        if show_all is not None:
            self.show_all = bool(show_all)
        self.selected = int(selected)
        self.clear()
        rv, model = self._get_view(), self._get_model()
        if rv is None or model is None or not solutions:
            return
        if self.show_all:
            idx = list(range(min(len(solutions), MAX_ALL)))
            if 0 <= self.selected < len(solutions) and self.selected not in idx:
                idx[-1] = self.selected
        else:
            idx = [self.selected] if 0 <= self.selected < len(solutions) else []
        if not idx:
            return
        cm = cb.compile(model)
        for k in idx:
            s = solutions[k]
            key = ("ik", k)
            alone = len(idx) == 1
            op = OPACITY if alone else (OPACITY_SELECTED if k == self.selected else OPACITY_OTHER)
            rv.add_ghost(key, model, cb.fk_compiled(cm, list(s.q)),
                         GHOST_OK if s.valid else GHOST_BAD, op)
            self._keys.append(key)

    def redraw(self, solutions: Sequence[Any]) -> None:
        """Draw again with the same selection (after the Ghosts dock cleared all ghosts)."""
        self.show(solutions, self.selected)
