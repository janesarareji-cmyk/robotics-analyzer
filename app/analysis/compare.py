"""Robot comparison (U2.7): reach, workspace volume, mass, gravity torque, payload.

Pure functions (no Qt). ``compare_robot`` runs the study for one model; the Compare dock calls
it per robot in a TaskRunner worker. Reuses ``analyze_workspace`` (CPU), ``cb.fk_batch``,
``cb.inverse_dynamics`` and ``cb.jacobian_tcp``.

Definitions (same samples, voxel size and seed for every robot):
- reach: max distance of the TCP from the base frame origin over the workspace samples (plus a
  small deterministic local refinement of the best samples, so the stretched pose is not left to
  chance). The TCP is the tool point of the model, so it includes the flange / tool offset.
- stretched pose ``q_stretch``: the sample (same refinement) with the largest HORIZONTAL TCP
  distance from the base origin (world frame, gravity along -z). The 3D maximum alone can be an
  arm pointing straight up, where gravity torque is zero and the payload meaningless.
- gravity torque per joint: static RNEA (qd = qdd = 0, gravity (0, 0, -9.81)) at ``q_stretch``.
- payload: largest point mass at the TCP with every |tau_i| <= effort_i at ``q_stretch``
  (tau_i = tau_gravity_i + m * a_i, a_i = -(J_v^T g)_i). 0 when the robot cannot even hold
  itself there; None ("n/a") when a joint that carries the payload has no effort limit.
"""
from __future__ import annotations

import csv
import math
from pathlib import Path
from typing import Any, Callable

import numpy as np

import core_bridge as cb
from app.kin_utils import dof_joint_types, dof_limits

DEFAULT_SAMPLES = 20_000
DEFAULT_VOXEL = 0.05
DEFAULT_SEED = 42
GRAVITY = (0.0, 0.0, -9.81)
REFINE_ROUNDS = 12


def dof_master_joints(model) -> list[int]:
    """Joint index of the master joint of every DOF."""
    out = [-1] * model.dof()
    for j_idx, q_idx in enumerate(model.q_index_map()):
        if q_idx >= 0:
            out[q_idx] = j_idx
    return out


def dof_names(model) -> list[str]:
    return [str(model.joints[j].name) for j in dof_master_joints(model)]


def dof_efforts(model) -> list[float]:
    """Effort limit per DOF (nan when missing: limit <= 0 or not finite)."""
    out = []
    for j in dof_master_joints(model):
        e = float(model.joints[j].limits.effort)
        out.append(e if (math.isfinite(e) and e > 0.0) else float("nan"))
    return out


def total_mass(model) -> float:
    return float(sum(float(l.inertia.mass) for l in model.links))


def payload_capacity(tau_g, efforts, a) -> float | None:
    """Largest m >= 0 with |tau_g_i + m * a_i| <= effort_i for every i (None = n/a).

    A joint with a_i == 0 (not moved by a load at the TCP) does not limit the payload and does not
    need an effort limit. A joint with a_i != 0 and no limit makes the answer n/a.
    """
    best = math.inf
    for t, e, ai in zip(tau_g, efforts, a):
        if abs(ai) < 1e-9:
            continue
        if not math.isfinite(e):
            return None
        if abs(t) > e:                       # cannot hold its own weight
            return 0.0
        m = (e - t) / ai if ai > 0 else (-e - t) / ai
        best = min(best, m)
    return 0.0 if best == math.inf else max(0.0, float(best))


def _positions(cm, q: np.ndarray, origin: np.ndarray) -> np.ndarray:
    """TCP world positions relative to the base origin (N, 3)."""
    return np.asarray(cb.fk_batch(cm, q, "world")["position"]) - origin


def _norm3(p: np.ndarray) -> np.ndarray:
    return np.linalg.norm(p, axis=1)


def _norm_xy(p: np.ndarray) -> np.ndarray:
    return np.linalg.norm(p[:, :2], axis=1)


def find_stretched_pose(cm, q: np.ndarray, lo: np.ndarray, hi: np.ndarray, seed: int,
                        origin: np.ndarray, score=_norm3, cancel=None) -> tuple[float, np.ndarray]:
    """(best score, q) over the samples, refined by a small deterministic local search.

    ``score(p)`` maps TCP positions (relative to the base origin) to a value to maximise:
    3D distance (reach) or horizontal distance (stretched pose for the payload).
    """
    r = score(_positions(cm, q, origin))
    order = np.argsort(r)[::-1][:8]
    best_q = q[order].copy()                              # (8, n)
    best_r = r[order].copy()
    rng = np.random.default_rng(seed + 1)
    sigma = 0.3
    for _ in range(REFINE_ROUNDS):
        if cancel is not None and cancel.is_cancelled():
            break
        cand = best_q[:, None, :] + rng.normal(0.0, sigma, (len(best_q), 48, best_q.shape[1]))
        cand = np.clip(cand, lo, hi).reshape(-1, best_q.shape[1])
        cr = score(_positions(cm, cand, origin))
        all_q = np.vstack([best_q, cand])
        all_r = np.concatenate([best_r, cr])
        top = np.argsort(all_r)[::-1][:8]
        best_q, best_r = all_q[top], all_r[top]
        sigma *= 0.6
    return float(best_r[0]), best_q[0].copy()


def compare_robot(model, name: str, n_samples: int = DEFAULT_SAMPLES,
                  voxel_size: float = DEFAULT_VOXEL, seed: int = DEFAULT_SEED,
                  cancel=None, progress: Callable[[int, str], Any] | None = None) -> dict:
    """Study one robot. ``cancel.is_cancelled()`` stops early (result has ``cancelled`` True)."""
    from app.analysis.workspace_analysis import analyze_workspace
    n = int(model.dof())
    cm = cb.compile(model)
    lo_l, hi_l = dof_limits(model)
    lo, hi = np.asarray(lo_l, float), np.asarray(hi_l, float)
    if not (np.all(np.isfinite(lo)) and np.all(np.isfinite(hi)) and np.all(hi >= lo)):
        raise ValueError(f"{name}: joint limits must be finite with lower <= upper")

    def step(pct: int, text: str) -> None:
        if progress is not None:
            progress(pct, f"{name}: {text}")

    res: dict = dict(name=name, n_dof=n, mass=total_mass(model), cancelled=False,
                     n_samples=int(n_samples), voxel_size=float(voxel_size),
                     dof_names=dof_names(model), dof_types=dof_joint_types(model),
                     lo=lo.tolist(), hi=hi.tolist(), effort=dof_efforts(model))
    step(0, "sampling")
    rng = np.random.default_rng(int(seed))
    q = lo + rng.random((int(n_samples), n)) * (hi - lo)
    origin = np.asarray(cb.fk_compiled(cm, [0.0] * n)["frames"][0], dtype=float)[:3, 3]
    reach, q_reach = find_stretched_pose(cm, q, lo, hi, int(seed), origin, _norm3, cancel)
    _h, q_st = find_stretched_pose(cm, q, lo, hi, int(seed), origin, _norm_xy, cancel)
    res.update(reach=reach, q_reach=q_reach.tolist(), q_stretch=q_st.tolist(), reach_xy=_h)
    if cancel is not None and cancel.is_cancelled():
        res["cancelled"] = True
        return res
    step(30, "workspace volume")
    ws = analyze_workspace(
        model, dict(q=q, voxel_size=float(voxel_size), want_metrics=False), cancel=cancel,
        progress=lambda d, t: step(30 + int(50 * d / max(t, 1)), "workspace volume"))
    if ws.get("cancelled"):
        res["cancelled"] = True
        return res
    res.update(volume=float(ws["volume"]), volume_err=float(ws["volume_err_bound"]))
    step(85, "gravity torque and payload")
    dm = cb.build_dyn_model(model, cm)
    zero = [0.0] * n
    tau_g = [float(v) for v in cb.inverse_dynamics(dm, q_st.tolist(), zero, zero, list(GRAVITY))]
    jv = np.asarray(cb.jacobian_tcp(cm, q_st.tolist()))[:3]
    a = (-(jv.T @ np.asarray(GRAVITY))).tolist()
    res.update(tau_gravity=tau_g, payload_a=a,
               payload=payload_capacity(tau_g, res["effort"], a))
    step(100, "done")
    return res


def load_robot(path: str | Path) -> tuple[str, Any]:
    """(display name, model) of a URDF file; the sample title for a bundled sample."""
    from app.importers.urdf import import_urdf
    from app.sample_robots import import_display_name
    r = import_urdf(str(path), [])
    if r.model is None:
        raise ValueError(f"{path}: URDF import failed: {r.report.summary()}")
    return import_display_name(str(path), r.model.metadata.name), r.model


# ------------------------------------------------------------------ table

HEADERS = ["Robot", "DOF", "Mass [kg]", "Reach [m]", "Workspace volume [m3]",
           "Gravity torque per joint [N m]", "Effort limit per joint [N m]",
           "Payload at stretched pose [kg]", "Joint limits"]


def _num(v, spec: str = ".4g") -> str:
    return "n/a" if v is None or not math.isfinite(v) else format(v, spec)


def _limit_text(res: dict) -> str:
    parts = []
    for nm, ty, lo, hi in zip(res["dof_names"], res["dof_types"], res["lo"], res["hi"]):
        if ty == "prismatic":
            parts.append(f"{nm} {lo:.3g}..{hi:.3g} m")
        else:
            parts.append(f"{nm} {math.degrees(lo):.0f}..{math.degrees(hi):.0f} deg")
    return "; ".join(parts)


def table_rows(results: list[dict]) -> list[list[str]]:
    rows = []
    for r in results:
        rows.append([
            r["name"], str(r["n_dof"]), _num(r["mass"]), _num(r.get("reach")),
            _num(r.get("volume")),
            "; ".join(_num(abs(t), ".3g") for t in r.get("tau_gravity", [])),
            "; ".join(_num(e, ".3g") for e in r["effort"]),
            "n/a" if r.get("payload") is None else _num(r["payload"], ".3g"),
            _limit_text(r),
        ])
    return rows


def export_csv(results: list[dict], path: str | Path) -> int:
    """Write the table as CSV (header row, one row per robot). Returns the row count."""
    rows = table_rows(results)
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(HEADERS)
        w.writerows(rows)
    return len(rows)
