"""Collision-free start pose of a freshly loaded robot (U3.4b).

Some sample robots start (all joints 0, or mid-range) with a link under the floor or inside
another link. ``find_free_start`` tries the original pose, the mid-range pose, 0.25-step blends
between the original and the mid-range pose, then seeded random samples inside the limits, and
returns the free pose closest (L2) to the original one.
"""
from __future__ import annotations

import logging
from typing import Any, Sequence

import numpy as np

from app.kin_utils import current_q, dof_limits
from app.planning.collision import (
    CollisionConfig, CollisionWorld, shapes_from_geometry,
)

log = logging.getLogger(__name__)

N_RANDOM = 400


def find_free_start(world: CollisionWorld, q0: Sequence[float], lo: Sequence[float],
                    hi: Sequence[float], seed: int = 0, n_random: int = N_RANDOM) -> list | None:
    """Free pose nearest to ``q0`` (``q0`` itself when free). None when nothing free was found."""
    q0 = np.asarray(q0, dtype=float)
    lo = np.asarray(lo, dtype=float)
    hi = np.asarray(hi, dtype=float)

    def free(q) -> bool:
        world.set_q(q)
        return not world.in_collision()

    if free(q0):
        return q0.tolist()
    mid = 0.5 * (lo + hi)
    cands = [mid] + [q0 + t * (mid - q0) for t in (0.25, 0.5, 0.75)]
    good = [c for c in cands if free(c)]
    if not good:
        rng = np.random.default_rng(seed)
        for _ in range(int(n_random)):
            c = lo + rng.random(len(lo)) * (hi - lo)
            if free(c):
                good.append(c)
                if len(good) >= 25:
                    break
    if not good:
        return None
    return min(good, key=lambda c: float(np.linalg.norm(c - q0))).tolist()


def ensure_free_start(model: Any, items: Sequence, config: CollisionConfig | None = None,
                      seed: int = 0, world: CollisionWorld | None = None) -> tuple:
    """Look for a collision-free start pose (floor on) when the model's current one collides.

    Returns ``(q, note)``: ``q`` is the pose to use (None = keep the current one), ``note`` a log
    line (empty when the pose was free already). Does not change ``model``.
    """
    if model is None or model.dof() == 0:
        return None, ""
    try:
        cfg = config if config is not None else CollisionConfig()
        if world is None:
            shapes = [s for s in shapes_from_geometry(list(items), cfg.fit_mode) if s.link >= 0]
            if not shapes:
                return None, ""
            world = CollisionWorld(model, shapes, cfg)
        q0 = current_q(model)
        lo, hi = dof_limits(model)
        q = find_free_start(world, q0, lo, hi, seed)
    except Exception as exc:                          # no fcl, bad shapes: keep the pose
        log.warning("start pose check skipped: %s", exc)
        return None, ""
    if q is None:
        return None, "Start pose collides and no free pose was found"
    if np.allclose(q, q0):
        return None, ""
    d = float(np.linalg.norm(np.asarray(q) - np.asarray(q0)))
    return q, f"The start pose touched something, so a nearby free pose is used (moved {d:.2f} rad)"
