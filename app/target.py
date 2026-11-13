"""Target poses (U4a.1). No Qt dependency.

``Target(name, pose, frame)``: a wanted TCP pose, 4x4 in the same frame as ``cb.fk(...)["tcp"]``
(``frame`` is kept as a label, ``"base"`` by default). ``TargetStore`` is the ordered list kept in
the project (``targets.json``, optional member: old files simply have none).

Pose helpers: ``pose_from_xyz_rpy`` / ``xyz_rpy_from_pose`` (roll-pitch-yaw, R = Rz(yaw) Ry(pitch)
Rx(roll)), ``pose_on_surface(point, normal, ref_rot)`` (tool z axis = -normal).
"""
from __future__ import annotations

import logging
import math
from typing import Any, Iterable

import numpy as np

log = logging.getLogger("robotics_analyzer.target")

TARGETS_FORMAT = 1


def rot_x(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], float)


def rot_y(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], float)


def rot_z(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], float)


def pose_from_xyz_rpy(xyz: Iterable[float], rpy: Iterable[float]) -> np.ndarray:
    """4x4 pose from position (m) and roll, pitch, yaw (rad): R = Rz(yaw) Ry(pitch) Rx(roll)."""
    x = np.asarray(list(xyz), float).reshape(3)
    r, p, y = (float(v) for v in rpy)
    T = np.eye(4)
    T[:3, :3] = rot_z(y) @ rot_y(p) @ rot_x(r)
    T[:3, 3] = x
    return T


def xyz_rpy_from_pose(T: Any) -> tuple[np.ndarray, np.ndarray]:
    """``(xyz, rpy)`` of a 4x4 pose (inverse of ``pose_from_xyz_rpy``; pitch in [-pi/2, pi/2])."""
    T = np.asarray(T, float)
    R = T[:3, :3]
    sp = -R[2, 0]
    pitch = math.asin(max(-1.0, min(1.0, sp)))
    if abs(sp) < 1.0 - 1e-9:
        roll = math.atan2(R[2, 1], R[2, 2])
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:                                   # gimbal lock: yaw = 0
        roll = math.atan2(-R[1, 2], R[1, 1])
        yaw = 0.0
    return T[:3, 3].copy(), np.array([roll, pitch, yaw])


def rotate_pose(T: Any, axis: str, deg: float, local: bool = True) -> np.ndarray:
    """Rotate pose ``T`` by ``deg`` about its own (``local``) or the world x / y / z axis."""
    T = np.array(T, float)
    R = {"x": rot_x, "y": rot_y, "z": rot_z}[axis.lower()](math.radians(float(deg)))
    if local:
        T[:3, :3] = T[:3, :3] @ R
    else:
        T[:3, :3] = R @ T[:3, :3]
    return T


def pose_on_surface(point: Any, normal: Any, ref_rot: Any = None) -> np.ndarray:
    """Target on a surface: origin at ``point``, tool z axis = ``-normal`` (tool points in).

    The x axis keeps the direction of ``ref_rot``'s x axis as far as the new z allows.
    """
    n = np.asarray(normal, float).reshape(3)
    nn = float(np.linalg.norm(n))
    if nn < 1e-12:
        raise ValueError("surface normal has zero length")
    z = -n / nn
    ref = np.eye(3) if ref_rot is None else np.asarray(ref_rot, float)
    x = ref[:, 0] - z * float(ref[:, 0] @ z)
    if np.linalg.norm(x) < 1e-6:                   # ref x parallel to z: use ref y
        x = ref[:, 1] - z * float(ref[:, 1] @ z)
    if np.linalg.norm(x) < 1e-6:
        x = np.cross(z, [1.0, 0.0, 0.0]) if abs(z[0]) < 0.9 else np.cross(z, [0.0, 1.0, 0.0])
    x = x / np.linalg.norm(x)
    y = np.cross(z, x)
    T = np.eye(4)
    T[:3, :3] = np.column_stack([x, y, z])
    T[:3, 3] = np.asarray(point, float).reshape(3)
    return T


class Target:
    """One named target pose."""

    def __init__(self, name: str, pose: Any, frame: str = "base"):
        p = np.array(pose, float)
        if p.shape != (4, 4) or not np.all(np.isfinite(p)):
            raise ValueError("target pose must be a finite 4x4 matrix")
        self.name = str(name)
        self.pose = p
        self.frame = str(frame)

    def copy(self) -> "Target":
        return Target(self.name, self.pose.copy(), self.frame)

    def __eq__(self, other: object) -> bool:
        return (isinstance(other, Target) and self.name == other.name
                and self.frame == other.frame and np.array_equal(self.pose, other.pose))

    def to_dict(self) -> dict:
        return {"name": self.name, "frame": self.frame, "pose": self.pose.tolist()}

    @classmethod
    def from_dict(cls, d: dict) -> "Target":
        return cls(d["name"], d["pose"], d.get("frame", "base"))


class TargetStore:
    """Ordered list of targets with unique names (project data, ``targets.json``)."""

    def __init__(self, targets: Iterable[Target] | None = None):
        self._items: list[Target] = list(targets or [])

    def __len__(self) -> int:
        return len(self._items)

    def __iter__(self):
        return iter(self._items)

    def __getitem__(self, i: int) -> Target:
        return self._items[i]

    def __eq__(self, other: object) -> bool:
        return isinstance(other, TargetStore) and self._items == other._items

    def names(self) -> list[str]:
        return [t.name for t in self._items]

    def index(self, name: str) -> int:
        for i, t in enumerate(self._items):
            if t.name == name:
                return i
        return -1

    def unique_name(self, base: str = "Target") -> str:
        names = set(self.names())
        k = 1
        while f"{base} {k}" in names:
            k += 1
        return f"{base} {k}"

    def add(self, pose: Any, name: str | None = None, frame: str = "base") -> Target:
        nm = (name or "").strip() or self.unique_name()
        if nm in self.names():
            nm = self.unique_name(nm)
        t = Target(nm, pose, frame)
        self._items.append(t)
        return t

    def remove(self, i: int) -> None:
        del self._items[i]

    def rename(self, i: int, name: str) -> bool:
        nm = name.strip()
        if not nm or any(t.name == nm for k, t in enumerate(self._items) if k != i):
            return False
        self._items[i].name = nm
        return True

    def set_pose(self, i: int, pose: Any) -> None:
        self._items[i] = Target(self._items[i].name, pose, self._items[i].frame)

    def clear(self) -> None:
        self._items.clear()

    def to_dict(self) -> dict:
        return {"format": TARGETS_FORMAT, "targets": [t.to_dict() for t in self._items]}

    @classmethod
    def from_dict(cls, d: dict) -> "TargetStore":
        out = cls()
        for item in d.get("targets", []):
            try:
                t = Target.from_dict(item)
            except (KeyError, TypeError, ValueError) as exc:
                log.warning("Bad target skipped: %s", exc)
                continue
            if t.name in out.names():
                log.warning("Duplicate target %r skipped", t.name)
                continue
            out._items.append(t)
        return out
