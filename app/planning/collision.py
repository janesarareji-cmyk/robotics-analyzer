"""Collision geometry and checks (R7.2).

Library: python-fcl (FCL) behind ``CollisionWorld``; see ``docs/collision-decision.md``.

Shapes: ``sphere``, ``box``, ``capsule``, ``cylinder`` (FCL exact cylinder, no
capsule approximation) and ``convex`` (hull of a mesh, <= 64 vertices, U3.1). Mesh
collision geometry is never used as a raw mesh: it is fitted by a convex hull (default),
an oriented box, a sphere or a capsule (``fit_mesh``).

Link ids in ``pairs()``: link index >= 0 for robot links; an environment shape
number ``k`` is reported as ``-1 - k`` (``env_id``).
"""
from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import Any, Iterable, Sequence

import numpy as np

import core_bridge as cb

log = logging.getLogger(__name__)

try:                                    # optional at import time; needed by CollisionWorld
    import fcl
except ImportError:                     # pragma: no cover - depends on the environment
    fcl = None

SHAPE_KINDS = ("sphere", "box", "capsule", "cylinder", "convex")
FIT_MODES = ("obb", "sphere", "capsule", "hull")
DEFAULT_FIT_MODE = "hull"
COLLISION_FORMAT = 1


def env_id(k: int) -> int:
    """Pair id of environment shape number ``k`` (always negative)."""
    return -1 - int(k)


# ---------------------------------------------------------------------------
# Shapes
# ---------------------------------------------------------------------------

def pose_matrix(xyz: Sequence[float], rpy: Sequence[float]) -> np.ndarray:
    """4x4 pose from xyz and URDF rpy (same convention as ``cb.pose_to_iso``)."""
    p = cb.Pose()
    p.xyz = [float(v) for v in xyz]
    p.rpy = [float(v) for v in rpy]
    return np.array(cb.pose_to_iso(p), dtype=float)


@dataclass
class Shape:
    """One collision primitive.

    ``link`` is the robot link index (pose is in the link frame) or -1 for an
    environment shape (pose is in the world frame).
    params: sphere ``radius``; box ``size`` [x,y,z]; capsule/cylinder ``radius``,
    ``length`` (axis = local z; capsule length is the cylindrical part); convex ``vertices``
    (n,3) and ``faces`` (m,3) outward triangles.
    """

    kind: str
    params: dict
    link: int = -1
    pose: np.ndarray = field(default_factory=lambda: np.eye(4))

    def __post_init__(self) -> None:
        if self.kind not in SHAPE_KINDS:
            raise ValueError(f"unknown shape kind {self.kind!r}")
        self.pose = np.asarray(self.pose, dtype=float).reshape(4, 4)
        if self.kind == "box":
            if len(self.params.get("size", ())) != 3:
                raise ValueError("box needs size [x, y, z]")
        elif self.kind == "convex":
            v = np.asarray(self.params.get("vertices", ()), dtype=float)
            f = np.asarray(self.params.get("faces", ()), dtype=int)
            if v.ndim != 2 or v.shape[1] != 3 or len(v) < 4 or f.ndim != 2 or f.shape[1] != 3:
                raise ValueError("convex needs vertices (n,3) and faces (m,3)")
            self.params = {"vertices": v, "faces": f}
        elif self.kind == "sphere":
            if "radius" not in self.params:
                raise ValueError("sphere needs radius")
        elif "radius" not in self.params or "length" not in self.params:
            raise ValueError(f"{self.kind} needs radius and length")

    def bounding_radius(self) -> float:
        """Radius of a sphere around the shape origin that contains the shape."""
        p = self.params
        if self.kind == "sphere":
            return float(p["radius"])
        if self.kind == "box":
            return 0.5 * float(np.linalg.norm(p["size"]))
        if self.kind == "convex":
            return float(np.linalg.norm(p["vertices"], axis=1).max())
        h = 0.5 * float(p["length"])
        r = float(p["radius"])
        return h + r if self.kind == "capsule" else float(np.hypot(h, r))

    def to_fcl(self):
        p = self.params
        if self.kind == "sphere":
            return fcl.Sphere(float(p["radius"]))
        if self.kind == "box":
            return fcl.Box(*[float(v) for v in p["size"]])
        if self.kind == "capsule":
            return fcl.Capsule(float(p["radius"]), float(p["length"]))
        if self.kind == "convex":
            f = np.asarray(p["faces"], dtype=np.int64)
            flat = np.column_stack([np.full(len(f), 3, dtype=np.int64), f]).ravel()
            return fcl.Convex(np.asarray(p["vertices"], dtype=float), len(f), flat)
        return fcl.Cylinder(float(p["radius"]), float(p["length"]))


@dataclass
class EnvShape:
    """Static environment shape in world frame (saved in the project)."""

    kind: str                    # sphere | box | cylinder
    params: dict
    xyz: list = field(default_factory=lambda: [0.0, 0.0, 0.0])
    rpy: list = field(default_factory=lambda: [0.0, 0.0, 0.0])
    name: str = ""

    def __post_init__(self) -> None:
        if self.kind not in ("sphere", "box", "cylinder"):
            raise ValueError(f"environment shape kind {self.kind!r} not allowed")
        Shape(self.kind, self.params)           # validates params

    def to_shape(self) -> Shape:
        return Shape(self.kind, dict(self.params), -1, pose_matrix(self.xyz, self.rpy))

    def to_dict(self) -> dict:
        return {"name": self.name, "kind": self.kind, "params": dict(self.params),
                "xyz": [float(v) for v in self.xyz], "rpy": [float(v) for v in self.rpy]}

    @classmethod
    def from_dict(cls, d: dict) -> "EnvShape":
        return cls(kind=d["kind"], params=dict(d.get("params", {})),
                   xyz=[float(v) for v in d.get("xyz", (0, 0, 0))],
                   rpy=[float(v) for v in d.get("rpy", (0, 0, 0))],
                   name=d.get("name", ""))


# ---------------------------------------------------------------------------
# Allowed collision matrix + environment (project data)
# ---------------------------------------------------------------------------

def _key(a: int, b: int) -> tuple[int, int]:
    return (a, b) if a <= b else (b, a)


@dataclass
class CollisionConfig:
    """ACM edits and environment of a project (``collision.json``).

    Default: parent-child (adjacent) link pairs are not checked. ``edits`` maps a link
    pair to ``True`` (force check) or ``False`` (never check) and wins over the default.
    """

    edits: dict = field(default_factory=dict)          # (a, b) with a <= b -> bool check
    environment: list = field(default_factory=list)    # list[EnvShape]
    fit_mode: str = DEFAULT_FIT_MODE                   # mesh fit used by shapes_from_geometry

    def __post_init__(self) -> None:
        if self.fit_mode not in FIT_MODES:
            raise ValueError(f"fit_mode must be one of {FIT_MODES}")

    def is_default(self) -> bool:
        return not self.edits and not self.environment and self.fit_mode == DEFAULT_FIT_MODE

    def set_check(self, a: int, b: int, check: bool) -> None:
        if a == b:
            raise ValueError("a link pair needs two different links")
        self.edits[_key(int(a), int(b))] = bool(check)

    def clear_edit(self, a: int, b: int) -> None:
        self.edits.pop(_key(int(a), int(b)), None)

    def checked(self, a: int, b: int, adjacent: bool) -> bool:
        k = _key(a, b)
        if k in self.edits:
            return self.edits[k]
        return not adjacent

    def to_dict(self) -> dict:
        return {
            "format": COLLISION_FORMAT,
            "fit_mode": self.fit_mode,
            "acm_edits": [[a, b, bool(v)] for (a, b), v in sorted(self.edits.items())],
            "environment": [e.to_dict() for e in self.environment],
        }

    @classmethod
    def from_dict(cls, d: dict) -> "CollisionConfig":
        edits = {}
        for a, b, v in d.get("acm_edits", []):
            edits[_key(int(a), int(b))] = bool(v)
        return cls(edits=edits,
                   environment=[EnvShape.from_dict(e) for e in d.get("environment", [])],
                   fit_mode=d.get("fit_mode", "obb"))   # old file without the key


# ---------------------------------------------------------------------------
# Shapes from stored geometry (R3.5) and mesh fit
# ---------------------------------------------------------------------------

def fit_mesh(points: np.ndarray, mode: str = DEFAULT_FIT_MODE) -> tuple[str, dict, np.ndarray]:
    """Fit a primitive to mesh points (n, 3) in the mesh frame.

    Returns ``(kind, params, pose)``; pose is the primitive pose in the mesh frame.
    ``hull``: convex hull (<= 64 vertices, kind ``convex``, pose identity; a flat or
    degenerate mesh falls back to ``obb``); ``obb``: PCA oriented box; ``sphere``: bounding sphere of the AABB centre;
    ``capsule``: axis = principal direction, radius = max distance to the axis.
    All fits contain every point (conservative).
    """
    pts = np.asarray(points, dtype=float).reshape(-1, 3)
    if len(pts) == 0:
        raise ValueError("no points to fit")
    if mode not in FIT_MODES:
        raise ValueError(f"unknown fit mode {mode!r}")
    if mode == "hull":
        from app.planning.hull import convex_hull
        h = convex_hull(pts)
        if h is not None:
            return "convex", {"vertices": h[0], "faces": h[1]}, np.eye(4)
        mode = "obb"
    if mode == "sphere":
        c = 0.5 * (pts.min(0) + pts.max(0))
        r = float(np.linalg.norm(pts - c, axis=1).max())
        pose = np.eye(4)
        pose[:3, 3] = c
        return "sphere", {"radius": r}, pose
    mean = pts.mean(0)
    if len(pts) >= 3:
        w, v = np.linalg.eigh(np.cov((pts - mean).T))
        axes = v[:, ::-1]                       # largest variance first
    else:
        axes = np.eye(3)
    if np.linalg.det(axes) < 0:
        axes[:, 2] = -axes[:, 2]
    loc = (pts - mean) @ axes
    lo, hi = loc.min(0), loc.max(0)
    centre = mean + axes @ (0.5 * (lo + hi))
    pose = np.eye(4)
    pose[:3, :3] = axes
    pose[:3, 3] = centre
    if mode == "obb":
        return "box", {"size": [float(x) for x in (hi - lo)]}, pose
    # capsule along axis 0 -> rotate so the capsule z axis = principal axis
    ax = axes[:, 0]
    z = ax
    tmp = axes[:, 1]
    cap_axes = np.column_stack([tmp, np.cross(z, tmp), z])
    rel = pts - centre
    along = rel @ ax
    perp = np.linalg.norm(rel - np.outer(along, ax), axis=1)
    r = float(perp.max())
    half = float(np.abs(along).max())          # cylinder part covers all points (conservative)
    pose = np.eye(4)
    pose[:3, :3] = cap_axes
    pose[:3, 3] = centre
    return "capsule", {"radius": r, "length": 2.0 * half}, pose


def _mesh_points(v: Any) -> np.ndarray | None:
    """Vertices of a mesh LinkVisual with its URDF scale applied, or None."""
    path = getattr(v, "mesh_path", None)
    if not path or getattr(v, "fallback", False):
        return None
    try:
        from app.importers import visuals as vis
        mesh = vis.read_mesh_file(path)
        pts = np.asarray(mesh.points, dtype=float)
    except Exception as exc:                       # unreadable mesh: skip with a log
        log.warning("collision mesh '%s' unreadable: %s", path, exc)
        return None
    scale = np.asarray(v.params.get("scale", (1, 1, 1)), dtype=float)
    return pts * scale


def _fit_visual(v: Any, fit_mode: str):
    """``fit_mesh`` of a mesh LinkVisual, cached per mesh file + scale + mode."""
    from app.planning import hull as _hull

    def fit():
        pts = _mesh_points(v)
        return None if pts is None else fit_mesh(pts, fit_mode)

    path = getattr(v, "mesh_path", None)
    if not path:
        return fit()
    key = _hull.mesh_key(path, v.params.get("scale", (1, 1, 1))) + (fit_mode,)
    return _hull.cached(key, fit)


def shapes_from_geometry(items: Iterable[Any], fit_mode: str = DEFAULT_FIT_MODE) -> list[Shape]:
    """Build link shapes from stored ``LinkVisual`` items (collisions, or visuals).

    Mesh items are fitted (``fit_mode``); a mesh that cannot be read is skipped.
    """
    out: list[Shape] = []
    for v in items:
        pose = pose_matrix(v.origin_xyz, v.origin_rpy)
        if v.kind == "mesh":
            fitted = _fit_visual(v, fit_mode)
            if fitted is None:
                continue
            kind, params, fp = fitted
            out.append(Shape(kind, params, int(v.link_index), pose @ fp))
        elif v.kind in SHAPE_KINDS:
            out.append(Shape(v.kind, dict(v.params), int(v.link_index), pose))
    return out


def shapes_from_project(project: Any) -> list[Shape]:
    """Shapes of a project: stored collisions, else its visuals (R3.5)."""
    items = list(getattr(project, "collisions", []) or [])
    if not items:
        items = list(getattr(project, "visuals", []) or [])
    cfg = getattr(project, "collision", None)
    return shapes_from_geometry(items, cfg.fit_mode if cfg else DEFAULT_FIT_MODE)


# ---------------------------------------------------------------------------
# Collision world
# ---------------------------------------------------------------------------

class CollisionWorld:
    """Robot link shapes + static environment, queried at a joint vector ``q``.

    ``CollisionWorld(model, shapes, config=None)``. ``set_q(q)`` moves the robot;
    ``in_collision()``, ``pairs()``, ``min_distance()``, ``check_batch(qs)``.
    Needs python-fcl (``fcl``).
    """

    def __init__(self, model: Any, shapes: Iterable[Shape],
                 config: CollisionConfig | None = None):
        if fcl is None:
            raise RuntimeError("python-fcl is not installed (pip install python-fcl)")
        self.model = model
        self.config = config if config is not None else CollisionConfig()
        self._cm = cb.compile(model)
        self.n_links = len(model.links)
        self.adjacent = {_key(int(p), int(c)) for p, c in cb.effective_links(model)}
        self.robot_shapes = [s for s in shapes if s.link >= 0]
        for s in self.robot_shapes:
            if s.link >= self.n_links:
                raise ValueError(f"shape on link {s.link}, model has {self.n_links} links")
        env = [e.to_shape() for e in self.config.environment]
        self.env_shapes = env
        self._robot_objs = [fcl.CollisionObject(s.to_fcl(), fcl.Transform())
                            for s in self.robot_shapes]
        self._env_objs = []
        for s in env:
            R, t = s.pose[:3, :3], s.pose[:3, 3]
            self._env_objs.append(fcl.CollisionObject(s.to_fcl(), fcl.Transform(R, t)))
        self._link_of = np.array([s.link for s in self.robot_shapes], dtype=int)
        self._pose_loc = np.array([s.pose for s in self.robot_shapes]).reshape(-1, 4, 4)
        self._rad = np.array([s.bounding_radius() for s in self.robot_shapes])
        self._env_rad = np.array([s.bounding_radius() for s in env])
        self._env_c = np.array([s.pose[:3, 3] for s in env]).reshape(-1, 3)
        self._build_pairs()
        self._req = fcl.CollisionRequest(num_max_contacts=1, enable_contact=False)
        self._req_c = fcl.CollisionRequest(num_max_contacts=1, enable_contact=True)
        self._dreq = fcl.DistanceRequest()
        self._q = None
        self._wc = np.zeros((len(self.robot_shapes), 3))
        self.set_q(np.zeros(model.dof()))

    # -- setup ---------------------------------------------------------
    def _build_pairs(self) -> None:
        """Robot-robot pairs (ACM filtered) and robot-environment pairs as index lists."""
        n = len(self.robot_shapes)
        rr = []
        for i in range(n):
            for j in range(i + 1, n):
                a, b = int(self._link_of[i]), int(self._link_of[j])
                if a == b:
                    continue
                if self.config.checked(a, b, _key(a, b) in self.adjacent):
                    rr.append((i, j))
        self._rr = rr
        self._re = [(i, k) for i in range(n) for k in range(len(self.env_shapes))]
        self._rr_i = np.array([p[0] for p in rr], dtype=int)
        self._rr_j = np.array([p[1] for p in rr], dtype=int)
        self._re_i = np.array([p[0] for p in self._re], dtype=int)
        self._re_k = np.array([p[1] for p in self._re], dtype=int)

    # -- state ---------------------------------------------------------
    def set_q(self, q: Sequence[float]) -> None:
        q = [float(v) for v in q]
        if len(q) != self.model.dof():
            raise ValueError("q needs one entry per DOF")
        frames = np.asarray(cb.fk_compiled(self._cm, q)["frames"])
        self._q = q
        self._apply_frames(frames)

    def _apply_frames(self, frames: np.ndarray) -> None:
        if len(self._robot_objs) == 0:
            return
        T = np.matmul(frames[self._link_of], self._pose_loc)       # (n, 4, 4) world poses
        self._wc = T[:, :3, 3]
        for obj, M in zip(self._robot_objs, T):
            obj.setTransform(fcl.Transform(M[:3, :3], M[:3, 3]))

    # -- queries -------------------------------------------------------
    def _candidates(self):
        """Pairs whose bounding spheres overlap: lists of (i, j) robot-robot, (i, k) env."""
        rr, re = [], []
        if len(self._rr_i):
            d = np.linalg.norm(self._wc[self._rr_i] - self._wc[self._rr_j], axis=1)
            hit = d <= self._rad[self._rr_i] + self._rad[self._rr_j]
            rr = [self._rr[m] for m in np.nonzero(hit)[0]]
        if len(self._re_i):
            d = np.linalg.norm(self._wc[self._re_i] - self._env_c[self._re_k], axis=1)
            hit = d <= self._rad[self._re_i] + self._env_rad[self._re_k]
            re = [self._re[m] for m in np.nonzero(hit)[0]]
        return rr, re

    def in_collision(self) -> bool:
        rr, re = self._candidates()
        for i, j in rr:
            if fcl.collide(self._robot_objs[i], self._robot_objs[j], self._req,
                           fcl.CollisionResult()):
                return True
        for i, k in re:
            if fcl.collide(self._robot_objs[i], self._env_objs[k], self._req,
                           fcl.CollisionResult()):
                return True
        return False

    def _pair_ids(self, i: int, j: int | None, k: int | None):
        a = int(self._link_of[i])
        b = int(self._link_of[j]) if j is not None else env_id(k)
        return a, b

    def _signed(self, o1, o2) -> float:
        """Signed distance: >= 0 separation, < 0 minus penetration depth."""
        res = fcl.CollisionResult()
        if fcl.collide(o1, o2, self._req_c, res):
            depth = max((c.penetration_depth for c in res.contacts), default=0.0)
            return -float(depth)
        dres = fcl.DistanceResult()
        return float(fcl.distance(o1, o2, self._dreq, dres))

    def pairs(self) -> list[tuple[int, int, float]]:
        """Colliding pairs ``(linkA, linkB, -penetration_depth)``; environment id ``-1-k``."""
        out = []
        rr, re = self._candidates()
        for i, j in rr:
            if fcl.collide(self._robot_objs[i], self._robot_objs[j], self._req,
                           fcl.CollisionResult()):
                a, b = self._pair_ids(i, j, None)
                out.append((a, b, self._signed(self._robot_objs[i], self._robot_objs[j])))
        for i, k in re:
            if fcl.collide(self._robot_objs[i], self._env_objs[k], self._req,
                           fcl.CollisionResult()):
                a, b = self._pair_ids(i, None, k)
                out.append((a, b, self._signed(self._robot_objs[i], self._env_objs[k])))
        return out

    def min_distance(self) -> float:
        """Smallest signed distance over all checked pairs (negative: penetration).

        ``inf`` when there is nothing to check.
        """
        best = np.inf
        for i, j in self._rr:
            best = min(best, self._signed(self._robot_objs[i], self._robot_objs[j]))
        for i, k in self._re:
            best = min(best, self._signed(self._robot_objs[i], self._env_objs[k]))
        return float(best)

    def zero_pose_pairs(self, q: Sequence[float] | None = None) -> list[tuple[int, int]]:
        """Link pairs colliding at ``q`` (default zero pose), robot-robot only, ACM-checked."""
        q0 = self._q
        self.set_q(np.zeros(self.model.dof()) if q is None else q)
        out = sorted({_key(a, b) for a, b, _ in self.pairs() if a >= 0 and b >= 0})
        self.set_q(q0)
        return out

    def disable_always_colliding(self, q: Sequence[float] | None = None) -> list[tuple[int, int]]:
        """ACM helper: stop checking pairs that collide at ``q`` (default zero pose).

        Edits ``config`` and rebuilds the pair list. Returns the disabled pairs.
        """
        found = self.zero_pose_pairs(q)
        for a, b in found:
            self.config.set_check(a, b, False)
        self._build_pairs()
        return found

    def check_batch(self, q_array: np.ndarray) -> np.ndarray:
        """Collision flag for every row of ``q_array`` (N, dof). Leaves state at the last row."""
        qs = np.asarray(q_array, dtype=float)
        if qs.ndim != 2 or qs.shape[1] != self.model.dof():
            raise ValueError(f"q_array must have shape (N, {self.model.dof()})")
        out = np.zeros(len(qs), dtype=bool)
        cm = self._cm
        for n, q in enumerate(qs):
            self._apply_frames(np.asarray(cb.fk_compiled(cm, q.tolist())["frames"]))
            out[n] = self.in_collision()
        if len(qs):
            self._q = qs[-1].tolist()
        return out


# ---------------------------------------------------------------------------
# Shape-shape queries (closed-form testable)
# ---------------------------------------------------------------------------

def shape_distance(a: Shape, b: Shape) -> float:
    """Signed distance between two world-posed shapes (pose = world pose).

    >= 0 separation; < 0 minus penetration depth.
    """
    if fcl is None:
        raise RuntimeError("python-fcl is not installed")
    oa = fcl.CollisionObject(a.to_fcl(), fcl.Transform(a.pose[:3, :3], a.pose[:3, 3]))
    ob = fcl.CollisionObject(b.to_fcl(), fcl.Transform(b.pose[:3, :3], b.pose[:3, 3]))
    res = fcl.CollisionResult()
    if fcl.collide(oa, ob, fcl.CollisionRequest(num_max_contacts=1, enable_contact=True), res):
        return -float(max((c.penetration_depth for c in res.contacts), default=0.0))
    return float(fcl.distance(oa, ob, fcl.DistanceRequest(), fcl.DistanceResult()))
