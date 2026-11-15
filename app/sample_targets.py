"""Sample targets of the bundled robots (D1.2).

Every ``robots/<name>/`` has a ``targets.json`` with 5 named targets: 3 reachable (valid, collision
free IK, free approach, free planned motion between them) and 2 unreachable ("Too far", "Too close").
The file is made by ``tools/gen_sample_targets.py`` and checked by ``tests/python/test_sample_targets.py``.

File format = ``TargetStore.to_dict`` plus a ``"kind"`` key per item (``"reachable"`` /
``"unreachable"``) that the store ignores.

``build_scene`` makes a collision-ready headless copy of a sample robot (project, auto allowed
collision matrix, free start pose, floor) so the generator and the tests use the same rules as the app.
"""
from __future__ import annotations

import json
import logging
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

import core_bridge as cb
from app.target import TargetStore

log = logging.getLogger("robotics_analyzer.sample_targets")

TARGETS_FILE = "targets.json"
APPROACH_DIST = 0.08          # m back along the tool z axis for the free-approach check
KIND_REACHABLE = "reachable"
KIND_UNREACHABLE = "unreachable"


def targets_path(urdf_path: str | Path) -> Path:
    return Path(urdf_path).parent / TARGETS_FILE


def _read(urdf_path: str | Path) -> dict | None:
    p = targets_path(urdf_path)
    try:
        d = json.loads(p.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None
    return d if isinstance(d, dict) else None


def load_sample_targets(urdf_path: str | Path) -> TargetStore | None:
    """``TargetStore`` shipped next to a sample robot URDF, None when there is none."""
    d = _read(urdf_path)
    if d is None:
        return None
    store = TargetStore.from_dict(d)
    return store if len(store) else None


def sample_kinds(urdf_path: str | Path) -> dict[str, str]:
    """``{target name: "reachable" | "unreachable"}`` as written by the generator."""
    d = _read(urdf_path) or {}
    return {str(t.get("name")): str(t.get("kind", "")) for t in d.get("targets", [])}


def write_sample_targets(urdf_path: str | Path, items: list[tuple[str, np.ndarray, str]]) -> Path:
    """Write ``[(name, pose, kind)]`` as ``targets.json`` next to the URDF."""
    d = TargetStore.to_dict(_store_of(items))
    for entry, (_n, _p, kind) in zip(d["targets"], items):
        entry["kind"] = kind
        entry["pose"] = [[round(float(v), 6) for v in row] for row in entry["pose"]]
    p = targets_path(urdf_path)
    p.write_text(json.dumps(d, indent=1) + "\n", encoding="utf-8")
    return p


def _store_of(items) -> TargetStore:
    s = TargetStore()
    for name, pose, _k in items:
        s.add(pose, name)
    return s


@dataclass
class SampleScene:
    """A sample robot ready for checks: model, project, shared collision service, one session."""
    name: str
    urdf: Path
    model: Any
    project: Any
    service: Any
    session: Any
    cm: Any
    q0: list

    def tcp(self, q=None) -> np.ndarray:
        if q is not None:
            from app.kin_utils import write_q
            write_q(self.model, list(q))
        return np.asarray(cb.fk(self.model)["tcp"], float)


def build_scene(urdf_path: str | Path, n_samples: int = 2000) -> SampleScene:
    """Import the sample, compute the auto ACM and the free start pose like the app does."""
    from app.auto_acm import AutoAcmTask, apply_auto_acm
    from app.collision_service import CollisionService
    from app.importers.urdf import import_urdf
    from app.kin_utils import current_q, write_q
    from app.project.project_data import ProjectData

    urdf_path = Path(urdf_path)
    res = import_urdf(str(urdf_path))
    if res.model is None:
        raise RuntimeError(f"import failed: {res.report.summary()}")
    project = ProjectData.new(urdf_path.parent.name)
    project.robot_model = res.model
    project.visuals = list(res.visuals or [])
    project.collisions = list(res.collisions or [])
    out = AutoAcmTask(project, n_samples=n_samples).execute()
    apply_auto_acm(project, out)
    if out.get("start_q"):
        write_q(res.model, out["start_q"])
    svc = CollisionService()
    svc.set_project(project)
    session = svc.session()
    return SampleScene(urdf_path.parent.name, urdf_path, res.model, project, svc, session,
                       cb.compile(res.model), list(current_q(res.model)))


@dataclass
class TargetCheck:
    reachable: bool
    q: list | None = None
    reason: str = ""
    near_singular: bool = False
    approach_free: bool = False


def check_target(scene: SampleScene, pose: Any, q_from: Any = None, approach: float = APPROACH_DIST,
                 n_starts: int = 60) -> TargetCheck:
    """Ranked IK of ``pose`` with the scene collision session. Reachable = a valid (limits, no
    collision incl. floor) solution exists AND the approach pose ``approach`` m back along the tool
    z axis has a valid solution joined to it by a collision-free straight joint segment."""
    from app.analysis.ik_rank import rank_solutions
    q_from = scene.q0 if q_from is None else q_from
    rr = rank_solutions(scene.model, np.asarray(pose, float), q_from, scene.session, cm=scene.cm,
                        n_starts=n_starts)
    best = rr.best()
    if best is None:
        why = rr.solutions[0].reason.split(":")[0] if rr.solutions else "no IK solution"
        return TargetCheck(False, None, why)
    out = TargetCheck(True, list(best.q), "", best.near_singular, True)
    if approach > 0.0:
        pre = np.array(pose, float)
        pre[:3, 3] = pre[:3, 3] - approach * pre[:3, 2]
        rp = rank_solutions(scene.model, pre, best.q, scene.session, cm=scene.cm, n_starts=n_starts)
        bp = rp.best()
        if bp is None:
            out.approach_free = False
            return out
        qa, qb = np.asarray(bp.q, float), np.asarray(best.q, float)
        qs = qa + np.linspace(0.0, 1.0, 21)[:, None] * (qb - qa)
        out.approach_free = not bool(np.asarray(scene.session.world.check_batch(qs)).any())
    return out
