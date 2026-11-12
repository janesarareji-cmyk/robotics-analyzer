"""Named joint configurations and their comparison (R7.6). No Qt dependency.

``ConfigStore`` keeps ``{name: q}`` (project data, ``configs.json``). ``compare_configs``
gives the TCP / joint / manipulability differences of two or more configurations against the
first one (the reference).
"""
from __future__ import annotations

import logging
import math
from typing import Any, Iterable, Sequence

import numpy as np

log = logging.getLogger("robotics_analyzer.configs")

CONFIGS_FORMAT = 1


class ConfigStore:
    """Ordered ``{name: q}`` dictionary with validation. Each q is a list of floats."""

    def __init__(self, configs: dict | None = None):
        self._d: dict[str, list[float]] = {}
        for k, v in (configs or {}).items():
            self.set(k, v)

    # -- access -------------------------------------------------------------
    def __len__(self) -> int:
        return len(self._d)

    def __contains__(self, name: object) -> bool:
        return name in self._d

    def __eq__(self, other: object) -> bool:
        return isinstance(other, ConfigStore) and self._d == other._d

    @property
    def names(self) -> list[str]:
        return list(self._d)

    def get(self, name: str) -> list[float]:
        return list(self._d[name])

    def as_dict(self) -> dict[str, list[float]]:
        return {k: list(v) for k, v in self._d.items()}

    # -- edit ---------------------------------------------------------------
    def set(self, name: str, q: Iterable[float]) -> None:
        name = str(name).strip()
        if not name:
            raise ValueError("configuration name is empty")
        vals = [float(x) for x in q]
        if not vals or not all(math.isfinite(x) for x in vals):
            raise ValueError("configuration needs finite numbers")
        self._d[name] = vals

    def remove(self, name: str) -> None:
        self._d.pop(name, None)

    def rename(self, old: str, new: str) -> None:
        new = str(new).strip()
        if not new:
            raise ValueError("configuration name is empty")
        if old not in self._d:
            raise KeyError(old)
        if new != old and new in self._d:
            raise ValueError(f"name {new!r} already used")
        self._d = {(new if k == old else k): v for k, v in self._d.items()}

    def clear(self) -> None:
        self._d.clear()

    def unique_name(self, base: str = "config") -> str:
        if base not in self._d:
            return base
        k = 2
        while f"{base} {k}" in self._d:
            k += 1
        return f"{base} {k}"

    def validate_dof(self, n_dof: int) -> list[str]:
        """Drop every configuration whose length differs from ``n_dof``. Returns the dropped names
        (a log warning is written when any)."""
        bad = [k for k, v in self._d.items() if len(v) != int(n_dof)]
        for k in bad:
            del self._d[k]
        if bad:
            log.warning("%d configuration(s) dropped: the robot has %d DOF (%s)",
                        len(bad), n_dof, ", ".join(bad))
        return bad

    # -- file ---------------------------------------------------------------
    def to_dict(self) -> dict:
        return {"format": CONFIGS_FORMAT,
                "configs": [{"name": k, "q": list(v)} for k, v in self._d.items()]}

    @classmethod
    def from_dict(cls, d: dict) -> "ConfigStore":
        s = cls()
        for item in d.get("configs", []):
            s.set(item["name"], item["q"])
        return s


# ---------------------------------------------------------------------------
# Comparison
# ---------------------------------------------------------------------------

def wrap_pi(x):
    """Wrap angles to [-pi, pi)."""
    return (np.asarray(x, float) + math.pi) % (2.0 * math.pi) - math.pi


def _rotvec(R: np.ndarray) -> np.ndarray:
    """Rotation vector (angle * axis) of a rotation matrix, any angle in [0, pi]."""
    c = float(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0))
    ang = math.acos(c)
    v = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    if ang < 1e-9:
        return 0.5 * v
    if math.pi - ang < 1e-6:                       # near pi: axis from the symmetric part
        B = (R + R.T) / 2.0 - c * np.eye(3)
        k = int(np.argmax(np.diag(B)))
        axis = B[:, k] / math.sqrt(max(B[k, k], 1e-300) * (1.0 - c))
        if np.dot(axis, v) < 0:
            axis = -axis
        return axis * ang
    return v * (ang / (2.0 * math.sin(ang)))


def joint_is_prismatic(model: Any) -> list[bool]:
    """Per DOF: True when the DOF drives a prismatic joint (master joint of ``q_index_map``)."""
    from app.kin_utils import dof_joint_types
    return [t == "prismatic" for t in dof_joint_types(model)]


def compare_configs(model: Any, named: Sequence[tuple[str, Sequence[float]]],
                    cm: Any = None) -> list[dict]:
    """Differences of every configuration to the first one.

    Returns one dict per input: ``name``, ``q``, ``tcp`` (4x4), ``dp`` (3, world TCP position
    minus the reference), ``distance`` (|dp|), ``drot`` (3, world rotation vector of
    ``R R_ref^T``), ``angle`` (rad), ``dq`` (per DOF, revolute wrapped to [-pi, pi), prismatic
    plain difference), ``manipulability`` and ``dmanip`` (minus the reference).
    """
    import core_bridge as cb
    from app.kin_utils import singularity_at

    if not named:
        return []
    cm = cm if cm is not None else cb.compile(model)
    n = model.dof()
    pris = joint_is_prismatic(model)
    rows = []
    for name, q in named:
        q = [float(x) for x in q]
        if len(q) != n:
            raise ValueError(f"configuration {name!r} has {len(q)} entries, robot has {n} DOF")
        tcp = np.asarray(cb.fk_compiled(cm, q)["tcp"], float)
        manip = float(singularity_at(cm, q)["manipulability"])
        rows.append({"name": name, "q": q, "tcp": tcp, "manipulability": manip})
    ref = rows[0]
    for r in rows:
        r["dp"] = r["tcp"][:3, 3] - ref["tcp"][:3, 3]
        r["distance"] = float(np.linalg.norm(r["dp"]))
        r["drot"] = _rotvec(r["tcp"][:3, :3] @ ref["tcp"][:3, :3].T)
        r["angle"] = float(np.linalg.norm(r["drot"]))
        d = np.asarray(r["q"]) - np.asarray(ref["q"])
        r["dq"] = [float(d[i]) if pris[i] else float(wrap_pi(d[i])) for i in range(n)]
        r["dmanip"] = r["manipulability"] - ref["manipulability"]
    return rows
