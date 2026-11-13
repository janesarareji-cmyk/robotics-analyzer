"""Convex hull of mesh points for collision shapes (U3.1).

No scipy: own incremental hull in numpy (``_raw_hull``).
Large meshes are first cut to their support points in many directions; a hull with more
than ``MAX_HULL_VERTICES`` vertices is rebuilt from the most important points (greedy farthest
point) and its vertices are pushed outward so that every original point stays inside.
"""
from __future__ import annotations

import logging
import os

import numpy as np

log = logging.getLogger(__name__)

MAX_HULL_VERTICES = 64
_CAND_DIRECTIONS = 300
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
    """Convex hull ``(vertices (n,3), faces (m,3))`` by incremental insertion, outward faces.

    None when the points are degenerate (fewer than 4, or flat / collinear). (VTK's
    ``delaunay_3d`` was tried first: on cylinder-like meshes it returns a surface that is not
    convex, so the hull is computed here.)
    """
    P = np.asarray(pts, dtype=float).reshape(-1, 3)
    if len(P) < 4:
        return None
    scale = float(np.ptp(P, axis=0).max())
    if scale <= 0.0:
        return None
    eps = 1e-10 * scale
    # initial tetrahedron: extreme points, then the points farthest from line / plane
    i0, i1 = int(P[:, 0].argmin()), int(P[:, 0].argmax())
    if i0 == i1:
        i0, i1 = int(P[:, 1].argmin()), int(P[:, 1].argmax())
    d = P[i1] - P[i0]
    ln = np.linalg.norm(d)
    if ln <= eps:
        return None
    d /= ln
    rel = P - P[i0]
    dist = np.linalg.norm(rel - np.outer(rel @ d, d), axis=1)
    i2 = int(dist.argmax())
    if dist[i2] <= eps * 1e3:
        return None
    n = np.cross(P[i1] - P[i0], P[i2] - P[i0])
    n /= np.linalg.norm(n)
    h = rel @ n
    i3 = int(np.abs(h).argmax())
    if abs(h[i3]) <= eps * 1e3:
        return None
    inner = P[[i0, i1, i2, i3]].mean(0)
    faces: list = []                 # (a, b, c, normal, offset)

    PL = P.tolist()
    inner_l = inner.tolist()

    def add(a, b, c):
        pa, pb, pc = PL[a], PL[b], PL[c]
        ux, uy, uz = pb[0] - pa[0], pb[1] - pa[1], pb[2] - pa[2]
        wx, wy, wz = pc[0] - pa[0], pc[1] - pa[1], pc[2] - pa[2]
        nx, ny, nz = uy * wz - uz * wy, uz * wx - ux * wz, ux * wy - uy * wx
        ln_ = (nx * nx + ny * ny + nz * nz) ** 0.5
        if ln_ <= 0.0:
            return
        nx, ny, nz = nx / ln_, ny / ln_, nz / ln_
        off = nx * pa[0] + ny * pa[1] + nz * pa[2]
        if nx * inner_l[0] + ny * inner_l[1] + nz * inner_l[2] - off > 0.0:
            b, c, nx, ny, nz, off = c, b, -nx, -ny, -nz, -off
        faces.append((a, b, c, (nx, ny, nz), off))

    for a, b, c in ((i0, i1, i2), (i0, i1, i3), (i0, i2, i3), (i1, i2, i3)):
        add(a, b, c)
    used = {i0, i1, i2, i3}
    order = np.argsort(-np.linalg.norm(P - inner, axis=1))
    for pi in order:
        pi = int(pi)
        if pi in used:
            continue
        N = np.array([f[3] for f in faces])
        O = np.array([f[4] for f in faces])
        vis = np.nonzero(N @ P[pi] - O > eps)[0]
        if len(vis) == 0:
            continue
        edges = {}
        for m in vis:
            a, b, c = faces[m][:3]
            for e in ((a, b), (b, c), (c, a)):
                edges[e] = edges.get(e, 0) + 1
        horizon = [e for e in edges if (e[1], e[0]) not in edges]
        keep = set(range(len(faces))) - set(int(m) for m in vis)
        faces = [faces[m] for m in sorted(keep)]
        for a, b in horizon:
            add(a, b, pi)
        used.add(pi)
    f = np.array([f[:3] for f in faces], dtype=int)
    verts = np.unique(f)
    remap = -np.ones(len(P), dtype=int)
    remap[verts] = np.arange(len(verts))
    return P[verts], remap[f]


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


def _outside(v: np.ndarray, f: np.ndarray, pts: np.ndarray) -> np.ndarray:
    """Per point: largest signed distance outside any face plane of the hull (<= 0 inside)."""
    nrm, off = _planes(v, _orient(v, f))
    return (pts @ nrm.T - off).max(axis=1)


def _greedy(cand: np.ndarray, max_vertices: int):
    """Hull of at most ``max_vertices`` points of ``cand``: add the farthest outside point."""
    i0, i1 = int(cand[:, 0].argmin()), int(cand[:, 0].argmax())
    d = cand[i1] - cand[i0]
    d /= max(np.linalg.norm(d), 1e-30)
    rel = cand - cand[i0]
    i2 = int(np.linalg.norm(rel - np.outer(rel @ d, d), axis=1).argmax())
    n = np.cross(cand[i1] - cand[i0], cand[i2] - cand[i0])
    n /= max(np.linalg.norm(n), 1e-30)
    i3 = int(np.abs(rel @ n).argmax())
    sel = [i0, i1, i2, i3]
    hull = _raw_hull(cand[sel])
    for _ in range(3 * max_vertices):
        if hull is None:
            break
        out = _outside(hull[0], hull[1], cand)
        out[sel] = -np.inf
        k = int(out.argmax())
        if out[k] <= 1e-12 or len(hull[0]) >= max_vertices:
            break
        nxt = _raw_hull(cand[sel + [k]])
        sel.append(k)
        if nxt is None or len(nxt[0]) > max_vertices:
            break
        hull = nxt
    return hull


def _inflate(v: np.ndarray, f: np.ndarray, pts: np.ndarray):
    """Move vertices outward along their normals until every point of ``pts`` is inside.

    Returns ``(vertices, faces)`` (hull of the moved vertices) or None.
    """
    for _ in range(25):
        f = _orient(v, f)
        viol = float(_outside(v, f, pts).max())
        if viol <= 1e-10:
            return v, f
        a, b, c = v[f[:, 0]], v[f[:, 1]], v[f[:, 2]]
        fn = np.cross(b - a, c - a)
        fn /= np.maximum(np.linalg.norm(fn, axis=1, keepdims=True), 1e-30)
        vn = np.zeros_like(v)
        for k in range(3):
            np.add.at(vn, f[:, k], fn)
        vn /= np.maximum(np.linalg.norm(vn, axis=1, keepdims=True), 1e-30)
        cmin = np.ones(len(v))              # smallest cos(vertex normal, adjacent face normal)
        for k in range(3):
            np.minimum.at(cmin, f[:, k], np.einsum("ij,ij->i", vn[f[:, k]], fn))
        cmin = np.clip(cmin, 0.2, 1.0)
        v = v + vn * (1.05 * viol + 1e-9) / cmin[:, None]
        hull = _raw_hull(v)
        if hull is None:
            return None
        v, f = hull
    return None


def convex_hull(points: np.ndarray, max_vertices: int = MAX_HULL_VERTICES):
    """Convex hull ``(vertices, faces)`` of ``points`` (n,3), at most ``max_vertices`` vertices.

    Returns None when the points are degenerate (flat, collinear or fewer than 4).
    Faces are outward triangles. The result contains every input point (tol 1e-9): a hull
    with more vertices than allowed is rebuilt from the ``max_vertices`` most important
    points (greedy farthest point) and its faces are pushed outward by the worst miss.
    """
    pts = np.unique(np.asarray(points, dtype=float).reshape(-1, 3), axis=0)
    cand = _support_points(pts, _CAND_DIRECTIONS) if len(pts) > 400 else pts
    hull = _raw_hull(cand)
    if hull is None:
        return None
    v, f = hull
    if len(v) > max_vertices:
        hull = _greedy(v, max_vertices)
        if hull is None:
            return None
        v, f = hull
    f = _orient(v, f)
    return _inflate(v, f, pts)


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
