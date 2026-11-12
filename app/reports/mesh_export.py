"""R8.3: export the visual meshes of a robot as STL / OBJ files (PyVista)."""
from __future__ import annotations

import re
from pathlib import Path

import numpy as np

FORMATS = ("stl", "obj")


def _safe(name: str) -> str:
    return re.sub(r"[^A-Za-z0-9_.-]+", "_", str(name)).strip("_") or "link"


def export_meshes(visuals, folder, fmt: str = "stl", model=None, q=None) -> list[str]:
    """Write one mesh file per visual into ``folder``; returns the file paths.

    Each mesh is posed with its visual origin in the LINK frame, or, when ``model`` is given,
    in the WORLD frame at joint vector ``q`` (default zeros). Primitives (box, sphere,
    cylinder) become triangle meshes; a mesh whose file is missing becomes the placeholder cube.
    Files are named ``<link>_<n>.<fmt>``. Raises ``ValueError`` for an unknown format.
    """
    import pyvista as pv
    from app.importers.visuals import load_visual_mesh, visual_origin_iso

    fmt = fmt.lower().lstrip(".")
    if fmt not in FORMATS:
        raise ValueError(f"format must be one of {FORMATS}")
    frames = None
    if model is not None:
        import core_bridge as cb
        n = int(model.dof())
        qq = [0.0] * n if q is None else [float(x) for x in q]
        frames = [np.asarray(f, float) for f in cb.fk(model, qq)["frames"]]
    out_dir = Path(folder)
    out_dir.mkdir(parents=True, exist_ok=True)
    counts: dict = {}
    paths = []
    for v in visuals:
        mesh = load_visual_mesh(v)
        if mesh is None:                      # too large to load
            continue
        mesh = mesh.copy()
        T = np.asarray(visual_origin_iso(v), float)
        if frames is not None and 0 <= v.link_index < len(frames):
            T = frames[v.link_index] @ T
        mesh.transform(T, inplace=True)
        if not isinstance(mesh, pv.PolyData):
            mesh = mesh.extract_surface()
        k = counts.get(v.link_name, 0)
        counts[v.link_name] = k + 1
        p = out_dir / f"{_safe(v.link_name)}_{k}.{fmt}"
        mesh.triangulate().save(str(p))
        paths.append(str(p))
    return paths
