"""Automatic allowed-collision matrix as a background task (U3.2).

``AutoAcmTask`` samples the joint space in a ``TaskRunner`` worker (progress, cancel) and returns
the ``CollisionWorld.auto_acm`` result; ``apply_auto_acm`` stores it in ``project.collision`` on
the GUI thread.
"""
from __future__ import annotations

from typing import Any

from app.kin_utils import current_q
from app.planning.collision import (
    DEFAULT_FIT_MODE, CollisionConfig, CollisionWorld, shapes_from_geometry,
)
from app.start_pose import ensure_free_start
from app.task_runner import Task


def project_items(project: Any) -> list:
    """Stored collision items of a project, else its visuals (same rule as ``shapes_from_project``)."""
    items = list(getattr(project, "collisions", []) or [])
    return items or list(getattr(project, "visuals", []) or [])


class AutoAcmTask(Task):
    """Worker: build the link shapes and run ``CollisionWorld.auto_acm`` (not applied)."""

    def __init__(self, project: Any, n_samples: int = 2000, seed: int = 0):
        super().__init__()
        cfg = getattr(project, "collision", None)
        self._model = project.robot_model
        self._items = project_items(project)
        self._fit = cfg.fit_mode if cfg is not None else DEFAULT_FIT_MODE
        self._edits = dict(cfg.edits) if cfg is not None else {}
        self._q0 = current_q(self._model)
        self.n_samples = int(n_samples)
        self.seed = int(seed)

    def execute(self) -> Any:
        self.report_progress(0, "Auto ACM: building shapes")
        shapes = [s for s in shapes_from_geometry(self._items, self._fit) if s.link >= 0]
        if not shapes:
            return {"pairs": {}, "counts": {}, "n_samples": 0, "seed": self.seed, "seconds": 0.0}
        world = CollisionWorld(self._model, shapes,
                               CollisionConfig(edits=dict(self._edits), fit_mode=self._fit))
        res = world.auto_acm(self.n_samples, self.seed, self._q0, apply=False,
                             progress=self.report_progress, cancel=lambda: self.is_cancelled)
        # U3.4b: start pose free with the new matrix and the floor, else a nearby free pose
        cfg = CollisionConfig(edits=dict(self._edits), fit_mode=self._fit)
        cfg.set_auto(res["pairs"], {})
        self.report_progress(99, "Checking start pose")
        q, note = ensure_free_start(self._model, self._items, cfg, self.seed)
        res["start_q"], res["start_note"] = q, note
        return res


def apply_auto_acm(project: Any, result: dict) -> bool:
    """Store an ``auto_acm`` result in ``project.collision`` (created when missing)."""
    if project is None or not result:
        return False
    cfg = getattr(project, "collision", None)
    if cfg is None:
        cfg = project.collision = CollisionConfig()
    cfg.set_auto(result["pairs"], {k: result[k] for k in ("n_samples", "seed", "seconds")
                                   if k in result})
    return True
