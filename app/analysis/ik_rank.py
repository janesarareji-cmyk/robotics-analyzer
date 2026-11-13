"""Ranked IK solutions of a target pose (U4a.2). No Qt dependency.

``rank_solutions`` runs ``solve_ik_multi`` (all solutions, deduped), then scores every solution
with four parts, each normalised to 0..1 (1 = best):

* motion: ``1 - d / d_max`` with d = weighted joint distance to the current q (d_max over all solutions)
* limits: smallest distance to a joint limit over half the joint range
* manipulability: ``sigma_min / max sigma_min`` (sigma_min of the TCP Jacobian)
* clearance: collision clearance clipped at 0.10 m, divided by 0.10 (colliding = invalid)

Total = weighted sum (default 0.4, 0.2, 0.2, 0.2). Invalid solutions (collision, outside limits)
come last with a reason. Gripper DOFs stay where they are (locked). Robots with fewer than 6 arm
DOF are solved for the position only (``RankResult.note`` says so).
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any, Sequence

import numpy as np

import core_bridge as cb
from app.gripper import gripper_dofs
from app.kin_utils import SINGULAR_WARN, current_q, dof_limits

DEFAULT_WEIGHTS = (0.4, 0.2, 0.2, 0.2)          # motion, limits, manipulability, clearance
PART_NAMES = ("motion", "limits", "manip", "clearance")
CLEARANCE_CAP = 0.10                            # m
N_STARTS = 60
MAX_SOLUTIONS = 40


@dataclass
class RankedSolution:
    q: list
    total: float
    parts: dict                                  # motion / limits / manip / clearance in 0..1
    valid: bool
    reason: str = ""
    raw: dict = field(default_factory=dict)      # distance, margin, sigma_min, clearance (m)

    @property
    def near_singular(self) -> bool:
        return float(self.raw.get("sigma_min", 1.0)) < SINGULAR_WARN


@dataclass
class RankResult:
    solutions: list
    position_only: bool = False
    note: str = ""
    elapsed_ms: float = 0.0
    n_found: int = 0

    def best(self) -> RankedSolution | None:
        for s in self.solutions:
            if s.valid:
                return s
        return None

    def __len__(self) -> int:
        return len(self.solutions)

    def __iter__(self):
        return iter(self.solutions)

    def __getitem__(self, i):
        return self.solutions[i]


def normalise_weights(weights: Sequence[float] | None) -> np.ndarray:
    w = np.asarray(DEFAULT_WEIGHTS if weights is None else list(weights), float)
    if w.shape != (4,) or not np.all(np.isfinite(w)) or np.any(w < 0) or w.sum() <= 0:
        raise ValueError("weights need 4 non-negative numbers, not all zero")
    return w / w.sum()


def score(parts: dict, weights: Sequence[float] | None = None) -> float:
    """Weighted total of the four parts (weights normalised)."""
    w = normalise_weights(weights)
    return float(sum(wi * float(parts[k]) for wi, k in zip(w, PART_NAMES)))


def sigma_min_at(cm: Any, q: Sequence[float], position_only: bool = False) -> float:
    """Smallest singular value of the TCP Jacobian (position rows only when ``position_only``)."""
    J = np.asarray(cb.jacobian_tcp(cm, list(q)), float)
    if position_only:
        J = J[:3]
    sv = np.linalg.svd(J, compute_uv=False)
    k = min(J.shape)
    return float(sv[k - 1]) if len(sv) >= k and k > 0 else 0.0


def rank_solutions(model: Any, target: Any, q_current: Sequence[float] | None = None,
                   collision_service: Any = None, weights: Sequence[float] | None = None,
                   n_starts: int = N_STARTS, seed: int = 42, cm: Any = None,
                   joint_weights: Sequence[float] | None = None) -> RankResult:
    """Rank the IK solutions of ``target`` (4x4) from ``q_current`` (default: model q)."""
    t0 = time.perf_counter()
    w = normalise_weights(weights)
    q_cur = np.asarray(current_q(model) if q_current is None else q_current, float)
    n = model.dof()
    if q_cur.shape != (n,):
        raise ValueError(f"q_current needs {n} values")
    cm = cm if cm is not None else cb.compile(model)
    lo, hi = (np.array(v, float) for v in dof_limits(model))
    locked = np.zeros(n, bool)
    for g in gripper_dofs(model):
        lo[g.dof_index] = hi[g.dof_index] = q_cur[g.dof_index]
        locked[g.dof_index] = True
    n_arm = int(n - locked.sum())
    pos_only = n_arm < 6
    note = ""
    if pos_only:
        note = f"{n_arm} arm joints: position only, orientation ignored."
    sols = cb.solve_ik_multi(cm, list(lo), list(hi), np.asarray(target, float), list(q_cur),
                             {"position_only": pos_only}, int(n_starts), int(seed))
    jw = np.ones(n) if joint_weights is None else np.asarray(joint_weights, float)

    qs = [np.asarray(s["q"], float) for s in sols][:MAX_SOLUTIONS * 3]
    dist = np.array([float(np.sqrt(np.sum(jw * (q - q_cur) ** 2))) for q in qs])
    half = np.where(hi > lo, 0.5 * (hi - lo), 1.0)
    free = ~locked & (hi > lo)
    sig = np.array([sigma_min_at(cm, q, pos_only) for q in qs])
    d_max = float(dist.max()) if len(dist) else 0.0
    s_max = float(sig.max()) if len(sig) else 0.0

    out: list[RankedSolution] = []
    for q, d, sg in zip(qs, dist, sig):
        margin_n = 1.0
        if free.any():
            m = np.minimum(q - lo, hi - q)[free] / half[free]
            margin_n = float(np.clip(m.min(), 0.0, 1.0))
            outside = bool(np.any((q < lo - 1e-9) | (q > hi + 1e-9)))
        else:
            outside = False
        clear, reason = float("inf"), ""
        valid = True
        if outside:
            valid, reason = False, "outside joint limits"
        if collision_service is not None:
            res = collision_service.check(q, emit=False)
            if not res.ok:
                valid = False
                reason = reason or f"collision: {res.text()}"
                clear = 0.0
            else:
                clear = float(collision_service.clearance(q))
        parts = {
            "motion": 1.0 - (float(d) / d_max if d_max > 1e-12 else 0.0),
            "limits": margin_n,
            "manip": float(sg) / s_max if s_max > 1e-12 else 0.0,
            "clearance": 1.0 if not np.isfinite(clear) else float(np.clip(clear / CLEARANCE_CAP, 0, 1)),
        }
        out.append(RankedSolution(
            q=[float(v) for v in q], total=score(parts, w), parts=parts, valid=valid, reason=reason,
            raw={"distance": float(d), "margin": margin_n, "sigma_min": float(sg),
                 "clearance": clear}))
    out.sort(key=lambda s: (not s.valid, -s.total))
    return RankResult(out[:MAX_SOLUTIONS], pos_only, note, (time.perf_counter() - t0) * 1e3, len(qs))


def rerank(solutions: Sequence[RankedSolution], weights: Sequence[float] | None) -> list:
    """Same solutions with new totals for ``weights`` (no new IK), best valid first."""
    out = [RankedSolution(s.q, score(s.parts, weights), dict(s.parts), s.valid, s.reason, dict(s.raw))
           for s in solutions]
    out.sort(key=lambda s: (not s.valid, -s.total))
    return out
