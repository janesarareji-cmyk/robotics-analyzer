"""Convex hull of mesh points for collision shapes (U3.1).

No scipy: the hull comes from VTK through PyVista (``delaunay_3d`` + ``extract_surface``).
Large meshes are first cut to their support points in many directions; a hull with more
than ``MAX_HULL_VERTICES`` vertices is simplified by fewer directions and then inflated
about its centroid so that every original point stays inside (conservative).
"""
from __future__ import annotations

import logging
import os
import warnings

import numpy as np

log = logging.getLogger(__name__)

MAX_HULL_VERTICES = 64
_CAND_DIRECTIONS = 512
_cache: dict = {}


def clear_hull_cache() -> None:
    """Forget cached hulls (called on a robot / project change)."""
    _cache.clear()


def fibonacci_directions(n: int) -> np.ndarray:
    """``n`` roughly uniform unit vectors."""
    i = np.arange(n) + 0.5
    phi = np.arccos(1.0 - 2.0 * i / n)
    th = np.pi * (1.0 + 5.0 ** 0.5) * i
    return np.column_stack([np.cos(th) * np.sin(phi), np.sin(th) * np.sin(phi), np.cos(phi)])


def _support_points(pts: np.ndarray, n_dirs: int) -> np.ndarray:
    d = fibonacci_directions(n_dirs)
    proj = pts @ d.T                                    # (N, n_dirs)
    idx = np.unique(np.concatenate([proj.argmax(0), proj.argmin(0)]))
    return pts[idx]


def _raw_hull(pts: np.ndarray):
    """(vertices (n,3), faces (m,3)) from VTK, or None when the points are degenerate."""
    import pyvista as pv
    if len(pts) < 4:
        return None
    try:
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            grid = pv.PolyData(np.ascontiguousarray(pts, dtype=float)).delaunay_3d()
            if grid.n_cells == 0:
                return None
            surf = grid.extract_surface(algorithm="dataset_surface").triangulate()
    except Exception as exc:                            # degenerate input
        log.debug("hull failed: %s", exc)
        return None
    if surf.n_cells == 0:
        return None
    faces = np.asarray(surf.faces).reshape(-1, 4)[:, 1:]
    used = np.unique(faces)
    remap = -np.ones(surf.n_points, dtype=int)
    remap[used] = np.arange(len(used))
    return np.asarray(surf.points, dtype=float)[used], remap[faces]


def _orient(v: np.ndarray, f: np.ndarray) -> np.ndarray:
    """Make every triangle face outward (normal away from the centroid)."""
    c = v.mean(0)
    a, b, d = v[f[:, 0]], v[f[:, 1]], v[f[:, 2]]
    n = np.cross(b - a, d - a)
    out = f.copy()
    flip = np.einsum("ij,ij->i", n, (a + b + d) / 3.0 - c) < 0.0
    out[flip] = out[flip][:, [0, 2, 1]]
    return out


def _planes(v: np.ndarray, f: np.ndarray):
    a, b, d = v[f[:, 0]], v[f[:, 1]], v[f[:, 2]]
    n = np.cross(b - a, d - a)
    ln = np.linalg.norm(n, axis=1)
    ok = ln > 1e-14
    n = n[ok] / ln[ok, None]
    off = np.einsum("ij,ij->i", n, a[ok])
    return n, off


def convex_hull(points: np.ndarray, max_vertices: int = MAX_HULL_VERTICES):
    """Convex hull ``(vertices, faces)`` of ``points`` (n,3), at most ``max_vertices`` vertices.

    Returns None when the points are degenerate (flat, collinear or fewer than 4).
    Faces are outward triangles. The result contains every input point (tol 1e-9).
    """
    pts = np.unique(np.asarray(points, dtype=float).reshape(-1, 3), axis=0)
    cand = _support_points(pts, _CAND_DIRECTIONS) if len(pts) > 2000 else pts
    hull = _raw_hull(cand)
    if hull is None:
        return None
    v, f = hull
    n_dirs = 200
    while len(v) > max_vertices and n_dirs >= 8:
        nxt = _raw_hull(_support_points(v, n_dirs))
        if nxt is None:
            break
        v, f = nxt
        n_dirs = int(n_dirs * 0.7)
    if len(v) > max_vertices:                           # last resort: exact count by directions
        for n in (max_vertices, 48, 32, 24, 16, 12, 8):
            nxt = _raw_hull(_support_points(v, n))
            if nxt is not None:
                v, f = nxt
                if len(v) <= max_vertices:
                    break
    if len(v) > max_vertices:
        return None
    f = _orient(v, f)
    # conservative: push the faces outward until every original point is inside
    nrm, off = _planes(v, f)
    c = v.mean(0)
    out = (pts @ nrm.T - off).max()                     # worst point outside a face plane
    if out > 0.0:
        r_in = float((off - nrm @ c).min())
        if r_in > 1e-12:
            s = 1.0 + (out + 1e-12) / r_in
            v = c + (v - c) * s
        else:
            return None
    return v, f


def cached(key, fn):
    """Cached ``fn()`` result per ``key``; a None result is cached too."""
    if key not in _cache:
        _cache[key] = fn()
    return _cache[key]


def mesh_key(path: str, scale) -> tuple:
    try:
        st = os.stat(path)
        sig = (st.st_mtime_ns, st.st_size)
    except OSError:
        sig = (0, 0)
    return (str(path), tuple(float(s) for s in scale), sig)
