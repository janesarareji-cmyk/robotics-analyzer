"""
app/importers/visuals.py
R3.5: Link visual / collision geometry records, mesh path resolution and
mesh loading. Pure Python. The C++ URM does not store visuals.
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, List, Optional, Sequence
from urllib.parse import unquote, urlparse
from urllib.request import url2pathname

DEFAULT_RGBA = (0.7, 0.7, 0.7, 1.0)
MESH_FORMATS = (".stl", ".obj", ".ply")
MAX_TRIANGLES = 2_000_000

KINDS = ("box", "cylinder", "sphere", "mesh")


@dataclass
class LinkVisual:
    """One visual (or collision) shape attached to a link."""

    link_index: int
    link_name: str
    kind: str  # "box" | "cylinder" | "sphere" | "mesh"
    params: dict = field(default_factory=dict)
    origin_xyz: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    origin_rpy: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    rgba: List[float] = field(default_factory=lambda: list(DEFAULT_RGBA))
    mesh_path: Optional[str] = None
    fallback: bool = False

    def to_dict(self) -> dict:
        return {
            "link_index": self.link_index,
            "link_name": self.link_name,
            "kind": self.kind,
            "params": dict(self.params),
            "origin_xyz": list(self.origin_xyz),
            "origin_rpy": list(self.origin_rpy),
            "rgba": list(self.rgba),
            "mesh_path": self.mesh_path,
            "fallback": bool(self.fallback),
        }

    @classmethod
    def from_dict(cls, d: dict) -> "LinkVisual":
        return cls(
            link_index=int(d["link_index"]),
            link_name=d.get("link_name", ""),
            kind=d["kind"],
            params=dict(d.get("params", {})),
            origin_xyz=[float(x) for x in d.get("origin_xyz", (0.0, 0.0, 0.0))],
            origin_rpy=[float(x) for x in d.get("origin_rpy", (0.0, 0.0, 0.0))],
            rgba=[float(x) for x in d.get("rgba", DEFAULT_RGBA)],
            mesh_path=d.get("mesh_path"),
            fallback=bool(d.get("fallback", False)),
        )


def visual_origin_iso(v: LinkVisual):
    """4x4 numpy pose of the visual inside its link frame."""
    import core_bridge as cb

    p = cb.Pose()
    p.xyz = [float(x) for x in v.origin_xyz]
    p.rpy = [float(x) for x in v.origin_rpy]
    return cb.pose_to_iso(p)


# ---------------------------------------------------------------------------
# Path resolution
# ---------------------------------------------------------------------------

def _first_file(cands) -> Optional[str]:
    for c in cands:
        try:
            if c.is_file():
                return str(c.resolve())
        except OSError:
            continue
    return None


def resolve_mesh(
    filename: str, urdf_dir, search_roots: Optional[Sequence] = None
) -> Optional[str]:
    """Return the absolute path of a mesh file, or None when not found."""
    fn = (filename or "").strip()
    if not fn:
        return None
    roots = [Path(r) for r in (search_roots or []) if r]
    base = Path(urdf_dir)

    if fn.startswith("package://"):
        rest = fn[len("package://"):]
        pkg, _, rel = rest.partition("/")
        cands = []
        for r in roots:
            cands.append(r / pkg / rel)
            cands.append(r / rel)
        found = _first_file(cands)
        if found:
            return found
        d = base.resolve()
        cands = []
        for anc in [d, *d.parents]:
            if anc.name == pkg:
                cands.append(anc / rel)
            cands.append(anc / pkg / rel)
        return _first_file(cands)

    if fn.startswith("file://"):
        parsed = urlparse(fn)
        path = url2pathname(unquote(parsed.path))
        if parsed.netloc and parsed.netloc != "localhost":
            path = "//" + parsed.netloc + path
        return _first_file([Path(path)])

    p = Path(fn)
    if p.is_absolute():
        return _first_file([p])
    return _first_file([base / p])


# ---------------------------------------------------------------------------
# Mesh loading
# ---------------------------------------------------------------------------

def mesh_format_ok(path: str) -> bool:
    return Path(path).suffix.lower() in MESH_FORMATS


def read_mesh_file(path: str) -> Any:
    """Read a mesh file with PyVista. Raises on unreadable files."""
    import pyvista as pv

    mesh = pv.read(path)
    if not isinstance(mesh, pv.PolyData):
        mesh = mesh.extract_surface()
    return mesh


def triangle_count(mesh: Any) -> int:
    if getattr(mesh, "is_all_triangles", False):
        return int(mesh.n_cells)
    return int(mesh.triangulate().n_cells)


def fallback_cube(edge: float) -> Any:
    import pyvista as pv

    e = max(float(edge), 1e-6)
    return pv.Cube(center=(0.0, 0.0, 0.0), x_length=e, y_length=e, z_length=e)


def load_visual_mesh(v: LinkVisual, triad_length: float = 0.05,
                     cache: Optional[dict] = None) -> Any:
    """
    Build the PolyData of one visual, centred on its own origin (the origin
    pose is applied by the caller). Returns None when the mesh is too large
    to draw. Missing / unreadable / unsupported mesh -> fallback cube.
    """
    import numpy as np
    import pyvista as pv

    if v.kind == "box":
        x, y, z = (float(s) for s in v.params["size"])
        return pv.Cube(center=(0.0, 0.0, 0.0), x_length=x, y_length=y, z_length=z)
    if v.kind == "sphere":
        return pv.Sphere(radius=float(v.params["radius"]))
    if v.kind == "cylinder":
        return pv.Cylinder(
            center=(0.0, 0.0, 0.0),
            direction=(0.0, 0.0, 1.0),
            radius=float(v.params["radius"]),
            height=float(v.params["length"]),
        )
    if v.kind != "mesh":
        return fallback_cube(triad_length)

    if v.fallback or not v.mesh_path or not os.path.isfile(v.mesh_path) \
            or not mesh_format_ok(v.mesh_path):
        return fallback_cube(triad_length)

    scale = tuple(float(s) for s in v.params.get("scale", (1.0, 1.0, 1.0)))
    key = (v.mesh_path, scale)
    if cache is not None and key in cache:
        return cache[key]
    try:
        mesh = read_mesh_file(v.mesh_path)
    except Exception:
        return fallback_cube(triad_length)
    if triangle_count(mesh) > MAX_TRIANGLES:
        mesh = None
    else:
        mesh = mesh.copy()
        mesh.points = np.asarray(mesh.points, dtype=float) * np.array(scale)
    if cache is not None:
        cache[key] = mesh
    return mesh
