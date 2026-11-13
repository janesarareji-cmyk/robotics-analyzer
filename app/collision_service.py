"""Live collision service (U3.3).

One ``CollisionWorld`` for the current project: link hulls (``shapes_from_project``), the
allowed-collision matrix (user edits + auto), environment objects and the floor. The world
is rebuilt lazily after ``set_project`` / ``invalidate`` (robot, shapes, ACM or environment
changed).

``check(q)`` says whether ``q`` collides; ``first_free(q_from, q_to)`` returns the last free
``q`` on the straight joint-space segment. ``enabled = False`` ("Allow pass-through") makes
every check report free, the old pass-through behaviour.

Start inside a contact (the zero pose of a UR arm puts the wrist under the base plane):
``first_free`` then lets a move go on as long as it adds no contact pair that the start
pose did not already have, so the robot can always leave.
"""
from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import Any, Sequence

import numpy as np
from PySide6.QtCore import QObject, Signal

from app.planning.collision import (
    FLOOR_ID, CollisionConfig, CollisionWorld, shapes_from_project,
)

log = logging.getLogger(__name__)

FREE_TOL = 1e-3                 # rad (or m) accuracy of first_free


@dataclass
class CheckResult:
    """Outcome of one collision check."""

    ok: bool = True
    pairs: list = field(default_factory=list)       # (linkA, linkB, -depth); env -1-k, floor FLOOR_ID
    names: list = field(default_factory=list)       # (nameA, nameB) per pair

    @property
    def links(self) -> set:
        """Robot link indices that touch something."""
        return {a for a, b, _ in self.pairs if a >= 0} | {b for a, b, _ in self.pairs if b >= 0}

    @property
    def env_ids(self) -> set:
        """Environment object indices that are touched (floor excluded)."""
        return {-1 - b for _, b, _ in self.pairs if b < 0 and b != FLOOR_ID}

    def text(self) -> str:
        if self.ok or not self.names:
            return ""
        shown = "; ".join(f"{a} touches {b}" for a, b in self.names[:3])
        return shown + (f" (+{len(self.names) - 3})" if len(self.names) > 3 else "")

    def key(self) -> tuple:
        return tuple(sorted((a, b) for a, b, _ in self.pairs))


class CollisionService(QObject):
    """Collision checks of the current project, shared by every motion input."""

    collision_changed = Signal(object)          # CheckResult, when the colliding set changes

    def __init__(self, parent: QObject | None = None):
        super().__init__(parent)
        self._project: Any = None
        self._world: CollisionWorld | None = None
        self._dirty = True
        self._enabled = True
        self._last_key: tuple | None = ()
        self.last_result = CheckResult()
        self.last_block: CheckResult | None = None   # what stopped the last first_free
        self.error = ""

    # -- state ----------------------------------------------------------
    @property
    def enabled(self) -> bool:
        return self._enabled

    @enabled.setter
    def enabled(self, on: bool) -> None:
        on = bool(on)
        if on != self._enabled:
            self._enabled = on
            if not on:
                self._publish(CheckResult())

    def set_project(self, project: Any) -> None:
        self._project = project
        self.invalidate()
        self._last_key = ()
        self.last_result = CheckResult()

    def invalidate(self) -> None:
        """Rebuild the world at the next check (robot, shapes, ACM or environment changed)."""
        self._dirty = True

    @property
    def world(self) -> CollisionWorld | None:
        """The current world, rebuilt when needed. None without robot / shapes."""
        if self._dirty:
            self._rebuild()
        return self._world

    def _rebuild(self) -> None:
        self._dirty = False
        self._world = None
        self.error = ""
        p = self._project
        model = getattr(p, "robot_model", None)
        if p is None or model is None or len(model.joints) == 0:
            return
        try:
            shapes = [s for s in shapes_from_project(p) if s.link >= 0]
            cfg = getattr(p, "collision", None) or CollisionConfig()
            self._world = CollisionWorld(model, shapes, cfg)
        except Exception as exc:                       # missing fcl, bad shapes: pass-through
            self.error = str(exc)
            log.warning("collision world not built: %s", exc)

    # -- names ------------------------------------------------------------
    def link_name(self, i: int) -> str:
        """Display name of a pair id: robot link, environment object (-1-k) or the floor."""
        p = self._project
        if i == FLOOR_ID:
            return "floor"
        if i < 0:
            env = getattr(getattr(p, "collision", None), "environment", [])
            k = -1 - i
            return (env[k].name or f"object {k + 1}") if 0 <= k < len(env) else f"object {k + 1}"
        links = getattr(getattr(p, "robot_model", None), "links", [])
        return (links[i].name or f"link{i}") if 0 <= i < len(links) else f"link{i}"

    # -- checks ------------------------------------------------------------
    def check(self, q: Sequence[float], emit: bool = True) -> CheckResult:
        """Collision state at ``q``. Always ok when disabled or when there is no world."""
        w = self.world if self._enabled else None
        if w is None:
            res = CheckResult()
        else:
            w.set_q(q)
            if w.in_collision():
                pairs = w.pairs()
                res = CheckResult(False, pairs, [(self.link_name(a), self.link_name(b))
                                                 for a, b, _ in pairs])
            else:
                res = CheckResult()
        if emit:
            self._publish(res)
        return res

    def _publish(self, res: CheckResult) -> None:
        self.last_result = res
        key = res.key()
        if key != self._last_key:
            self._last_key = key
            self.collision_changed.emit(res)

    def is_free(self, q: Sequence[float]) -> bool:
        return self.check(q, emit=False).ok

    def first_free(self, q_from: Sequence[float], q_to: Sequence[float],
                   steps: int = 16, tol: float = FREE_TOL) -> list:
        """Last collision-free ``q`` on the segment ``q_from`` -> ``q_to`` (``q_to`` if all free).

        Samples ``steps`` points to find the first contact, then bisects to ``tol`` (largest
        joint change). ``last_block`` keeps the contact that stopped the move (None when free).
        A start that is already in contact may move on while no new pair appears.
        """
        a = np.asarray(q_from, dtype=float)
        b = np.asarray(q_to, dtype=float)
        self.last_block = None
        if not self._enabled or self.world is None:
            return b.tolist()
        allowed = set(self.check(a, emit=False).key())          # contacts the start already has

        def probe(t: float):
            r = self.check(a + t * (b - a), emit=False)
            return (r.ok or set(r.key()) <= allowed), r

        span = float(np.abs(b - a).max())
        lo, hi, hit = 0.0, 1.0, None
        for t in np.linspace(0.0, 1.0, max(int(steps), 2))[1:]:
            good, r = probe(float(t))
            if good:
                lo = float(t)
            else:
                hi, hit = float(t), r
                break
        if hit is None:
            return b.tolist()
        while (hi - lo) * span > tol:
            m = 0.5 * (lo + hi)
            good, r = probe(m)
            if good:
                lo = m
            else:
                hi, hit = m, r
        self.last_block = hit
        return (a + lo * (b - a)).tolist()
