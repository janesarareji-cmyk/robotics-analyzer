"""Argument checking and conversion helpers of the API (internal)."""
from __future__ import annotations

import functools
import math
from typing import Any

import numpy as np

from .errors import InvalidArgument, RoboticsApiError


def guard(error_cls):
    """Map any non-API exception of the wrapped function to ``error_cls`` (cause chained)."""
    def deco(fn):
        @functools.wraps(fn)
        def wrapper(*args, **kwargs):
            try:
                return fn(*args, **kwargs)
            except RoboticsApiError:
                raise
            except Exception as exc:  # noqa: BLE001 - mapping is the point
                raise error_cls(f"{fn.__name__}: {exc}") from exc
        return wrapper
    return deco


def vec(x: Any, n: int, name: str) -> np.ndarray:
    """1-D finite float array with exactly ``n`` entries, else ``InvalidArgument``."""
    if isinstance(x, (str, bytes)) or (isinstance(x, np.ndarray) and x.dtype.kind in "OUSV"):
        raise InvalidArgument(f"{name} must be a list of numbers")
    try:
        a = np.asarray(x, dtype=float)
    except (TypeError, ValueError) as exc:
        raise InvalidArgument(f"{name} must be a list of numbers") from exc
    if a.ndim != 1 or a.shape[0] != n:
        raise InvalidArgument(f"{name} needs {n} values (the robot has n_dof = {n}), "
                              f"got shape {a.shape}")
    if not np.all(np.isfinite(a)):
        raise InvalidArgument(f"{name} has a non-finite value")
    return a


def pose4(x: Any, name: str = "pose") -> np.ndarray:
    """4x4 homogeneous transform (finite, bottom row 0 0 0 1)."""
    try:
        a = np.asarray(x, dtype=float)
    except (TypeError, ValueError) as exc:
        raise InvalidArgument(f"{name} must be a 4x4 matrix") from exc
    if a.shape != (4, 4) or not np.all(np.isfinite(a)):
        raise InvalidArgument(f"{name} must be a finite 4x4 matrix, got shape {a.shape}")
    if not np.allclose(a[3], [0, 0, 0, 1]):
        raise InvalidArgument(f"{name}: last row must be [0, 0, 0, 1]")
    R = a[:3, :3]
    if not np.allclose(R.T @ R, np.eye(3), atol=1e-6) or abs(np.linalg.det(R) - 1.0) > 1e-6:
        raise InvalidArgument(f"{name}: the 3x3 rotation part is not a rotation matrix")
    return a


def gravity3(g: Any) -> tuple[float, float, float]:
    if g is None:
        return (0.0, 0.0, -9.81)
    return tuple(float(v) for v in vec(g, 3, "gravity"))


def rpy_from_matrix(R: np.ndarray) -> list[float]:
    """Roll, pitch, yaw (URDF convention R = Rz(yaw) Ry(pitch) Rx(roll)) of a rotation matrix."""
    sp = -R[2, 0]
    pitch = math.asin(max(-1.0, min(1.0, sp)))
    if abs(sp) < 1.0 - 1e-9:
        roll = math.atan2(R[2, 1], R[2, 2])
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:                                  # gimbal lock: yaw = 0
        roll = math.atan2(-R[1, 2], R[1, 1])
        yaw = 0.0
    return [roll, pitch, yaw]


def xyz_rpy(pose: Any) -> tuple[list[float], list[float]]:
    """``(xyz, rpy)`` from a 4x4 matrix, ``{"xyz", "rpy"}`` dict, or a list of 3 (xyz) / 6 values."""
    if isinstance(pose, dict):
        xyz = vec(pose.get("xyz", (0, 0, 0)), 3, "pose xyz")
        rpy = vec(pose.get("rpy", (0, 0, 0)), 3, "pose rpy")
        return xyz.tolist(), rpy.tolist()
    try:
        a = np.asarray(pose, dtype=float)
    except (TypeError, ValueError) as exc:
        raise InvalidArgument("pose must be a 4x4 matrix, {'xyz','rpy'}, or 3 or 6 numbers") from exc
    if a.shape == (4, 4):
        T = pose4(a)
        return T[:3, 3].tolist(), rpy_from_matrix(T[:3, :3])
    if a.shape == (3,):
        return vec(a, 3, "pose").tolist(), [0.0, 0.0, 0.0]
    if a.shape == (6,):
        v = vec(a, 6, "pose")
        return v[:3].tolist(), v[3:].tolist()
    raise InvalidArgument("pose must be a 4x4 matrix, {'xyz','rpy'}, or 3 or 6 numbers")


def shape_from_dict(d: dict, allowed: tuple[str, ...], link: int = -1):
    """Build an ``app.planning.collision.Shape`` from a plain dict.

    Keys: ``type`` (box, sphere, cylinder, capsule), ``pose`` (see ``xyz_rpy``), ``size``
    (box: [x, y, z] in m; sphere: radius; cylinder/capsule: [radius, length]) or the explicit
    ``radius`` / ``length``; ``link`` (robot shapes only).
    """
    from app.planning.collision import Shape, pose_matrix
    if not isinstance(d, dict):
        raise InvalidArgument("a shape must be a dict")
    kind = d.get("type", d.get("kind"))
    if kind not in allowed:
        raise InvalidArgument(f"shape type {kind!r} not allowed, use one of {allowed}")
    xyz, rpy = xyz_rpy(d.get("pose", {"xyz": (0, 0, 0)}))
    size = d.get("size")
    try:
        if kind == "box":
            if size is None or np.asarray(size, float).shape != (3,):
                raise InvalidArgument("box needs size [x, y, z]")
            params: dict = {"size": [float(v) for v in size]}
        elif kind == "sphere":
            r = d.get("radius", size)
            if r is None or np.ndim(r) != 0:
                raise InvalidArgument("sphere needs radius (size = radius)")
            params = {"radius": float(r)}
        else:
            r, h = d.get("radius"), d.get("length")
            if r is None and size is not None and np.asarray(size, float).shape == (2,):
                r, h = size
            if r is None or h is None:
                raise InvalidArgument(f"{kind} needs size [radius, length]")
            params = {"radius": float(r), "length": float(h)}
    except (TypeError, ValueError) as exc:
        raise InvalidArgument(f"bad shape size: {exc}") from exc
    if np.any(np.concatenate([np.ravel(v) for v in params.values()]) <= 0):
        raise InvalidArgument("shape sizes must be > 0")
    lk = int(d.get("link", link))
    return Shape(kind, params, lk, pose_matrix(xyz, rpy)), xyz, rpy, params
