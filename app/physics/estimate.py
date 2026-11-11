"""
app/physics/estimate.py
R3.7: Estimate mass, centre of mass and inertia of a link from its visual.

Conventions (same as urm::Inertia):
  * result is in the LINK frame,
  * tensor order (ixx, ixy, ixz, iyy, iyz, izz) about the COM, axes parallel
    to the link frame, kg m^2, with ixy = -integral(x y rho dV).
Uniform density. Closed triangle mesh: signed-tetrahedron method (Eberly,
"Polyhedral mass properties"). Open mesh: convex hull + warning.
"""

from __future__ import annotations

import math
from typing import Any, List, Optional, Tuple

import numpy as np

Result = Tuple[float, List[float], List[float], List[str]]


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _check_density_mass(density, total_mass) -> None:
    if (density is None) == (total_mass is None):
        raise ValueError("give exactly one of density or total_mass")
    val = density if density is not None else total_mass
    if not (math.isfinite(float(val)) and float(val) > 0.0):
        raise ValueError("density / total_mass must be finite and > 0")


def _origin_matrix(origin) -> np.ndarray:
    """None | 4x4 matrix | (xyz, rpy) -> 4x4 numpy matrix."""
    if origin is None:
        return np.eye(4)
    try:
        arr = np.asarray(origin, dtype=float)
    except (ValueError, TypeError):
        arr = None
    if arr is not None and arr.shape == (4, 4):
        return arr
    import core_bridge as cb

    xyz, rpy = origin
    p = cb.Pose()
    p.xyz = [float(x) for x in xyz]
    p.rpy = [float(x) for x in rpy]
    return np.asarray(cb.pose_to_iso(p), dtype=float)


def _pack(mass: float, com, tensor3: np.ndarray, warnings: List[str]) -> Result:
    t = tensor3
    return (
        float(mass),
        [float(c) for c in com],
        [float(t[0, 0]), float(t[0, 1]), float(t[0, 2]),
         float(t[1, 1]), float(t[1, 2]), float(t[2, 2])],
        warnings,
    )


# ---------------------------------------------------------------------------
# Triangle mesh (signed tetrahedra)
# ---------------------------------------------------------------------------

def _mass_properties(points: np.ndarray, tris: np.ndarray):
    """
    Unit-density volume, first moment and second-moment integrals over the
    solid bounded by the triangles. Returns (vol, first[3], P[3,3]) with
    first[i] = integral(x_i dV), P[i,j] = integral(x_i x_j dV), about the origin.
    Each triangle forms a tetrahedron with the origin (det = 6 * signed volume).
    """
    a = points[tris[:, 0]]
    b = points[tris[:, 1]]
    c = points[tris[:, 2]]
    det = np.einsum("ij,ij->i", a, np.cross(b, c))
    vol = det.sum() / 6.0
    first = (det[:, None] * (a + b + c)).sum(axis=0) / 24.0

    p = np.zeros((3, 3))
    for i in range(3):
        for j in range(i, 3):
            diag = a[:, i] * a[:, j] + b[:, i] * b[:, j] + c[:, i] * c[:, j]
            offd = (a[:, i] * (b[:, j] + c[:, j]) + b[:, i] * (a[:, j] + c[:, j])
                    + c[:, i] * (a[:, j] + b[:, j]))
            # tetra with a vertex at the origin: det/120 * (2*diag + offd)
            p[i, j] = p[j, i] = (det * (2.0 * diag + offd)).sum() / 120.0
    return vol, first, p


def _closed_mesh_inertial(mesh: Any, density, total_mass):
    pts = np.asarray(mesh.points, dtype=float)
    faces = np.asarray(mesh.faces).reshape(-1, 4)[:, 1:]
    # Shift to the mesh centroid first: keeps the cancellation error small
    # when the mesh sits far from the origin.
    shift = pts.mean(axis=0)
    vol, first, p = _mass_properties(pts - shift, faces)
    if vol < 0.0:                       # inward-facing normals
        vol, first, p = -vol, -first, -p
    if not vol > 1e-18:
        raise ValueError("mesh has zero volume")
    com_s = first / vol                 # COM relative to the shift
    mass = float(density) * vol if density is not None else float(total_mass)
    rho = mass / vol
    # inertia about the shift point, then parallel-axis to the COM
    t_shift = rho * (np.trace(p) * np.eye(3) - p)
    t_com = t_shift - mass * (np.dot(com_s, com_s) * np.eye(3) - np.outer(com_s, com_s))
    return mass, com_s + shift, t_com


def estimate_inertial(mesh: Any, density: Optional[float] = None,
                      total_mass: Optional[float] = None) -> Result:
    """
    Mass, COM, inertia tensor of a uniform-density solid given by a triangle
    mesh (already in the link frame). Exactly one of `density` (kg/m^3) or
    `total_mass` (kg). Returns (mass, com[3], tensor[6], warnings).
    """
    _check_density_mass(density, total_mass)
    warnings: List[str] = []
    m = mesh.triangulate().clean()
    if m.n_cells == 0:
        raise ValueError("mesh has no triangles")
    if m.n_open_edges > 0:
        warnings.append(
            "mesh is not watertight; the convex hull is used for the estimate"
        )
        try:
            hull = m.delaunay_3d().extract_surface(algorithm="dataset_surface").triangulate().clean()
        except Exception as exc:
            raise ValueError(f"convex hull failed: {exc}")
        if hull.n_cells == 0:
            raise ValueError("convex hull failed: empty surface")
        m = hull
    mass, com, t = _closed_mesh_inertial(m, density, total_mass)
    return _pack(mass, com, t, warnings)


# ---------------------------------------------------------------------------
# Primitives (analytic)
# ---------------------------------------------------------------------------

def primitive_inertial(kind: str, params: dict, origin=None,
                       density: Optional[float] = None,
                       total_mass: Optional[float] = None) -> Result:
    """
    Analytic mass properties of a box / cylinder (axis z) / sphere placed at
    `origin` (None | 4x4 | (xyz, rpy)) inside the link frame.
    params: box {"size": [x,y,z]}; cylinder {"radius", "length"}; sphere {"radius"}.
    """
    _check_density_mass(density, total_mass)
    if kind == "box":
        x, y, z = (float(s) for s in params["size"])
        vol = x * y * z
        coef = np.array([y * y + z * z, x * x + z * z, x * x + y * y]) / 12.0
    elif kind == "cylinder":
        r = float(params["radius"])
        h = float(params["length"])
        vol = math.pi * r * r * h
        ixx = (3.0 * r * r + h * h) / 12.0
        coef = np.array([ixx, ixx, r * r / 2.0])
    elif kind == "sphere":
        r = float(params["radius"])
        vol = 4.0 / 3.0 * math.pi * r ** 3
        coef = np.full(3, 0.4 * r * r)
    else:
        raise ValueError(f"unknown primitive '{kind}'")
    if not vol > 0.0:
        raise ValueError("primitive has zero volume")
    mass = float(density) * vol if density is not None else float(total_mass)
    iso = _origin_matrix(origin)
    rot = iso[:3, :3]
    return _pack(mass, iso[:3, 3], rot @ np.diag(coef * mass) @ rot.T, [])


# ---------------------------------------------------------------------------
# Visual -> inertial (used by the Physical panel)
# ---------------------------------------------------------------------------

def visual_inertial(visual: Any, density: Optional[float] = None,
                    total_mass: Optional[float] = None,
                    cache: Optional[dict] = None) -> Result:
    """
    Estimate from one `LinkVisual` (R3.5). Primitives are analytic; meshes use
    the mesh method after scale + visual origin, so the result is in the link
    frame. Fallback / unreadable / too-large meshes raise ValueError.
    """
    from app.importers import visuals as vis

    origin = (visual.origin_xyz, visual.origin_rpy)
    if visual.kind in ("box", "cylinder", "sphere"):
        return primitive_inertial(visual.kind, visual.params, origin,
                                  density, total_mass)
    if visual.kind != "mesh" or visual.fallback:
        raise ValueError("no usable mesh for this visual")
    mesh = vis.load_visual_mesh(visual, cache=cache)
    if mesh is None:
        raise ValueError("mesh too large")
    mesh = mesh.copy()
    iso = _origin_matrix(origin)
    pts = np.asarray(mesh.points, dtype=float)
    mesh.points = pts @ iso[:3, :3].T + iso[:3, 3]
    return estimate_inertial(mesh, density, total_mass)
