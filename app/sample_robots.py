"""
app/sample_robots.py
U1: locate the bundled sample robots (robots/<name>/<name>.urdf).
"""

from __future__ import annotations

import sys
from pathlib import Path


def sample_robots_dir() -> Path:
    """Folder that holds the sample robots (repo root, or next to the frozen exe)."""
    if getattr(sys, "frozen", False):
        meipass = getattr(sys, "_MEIPASS", None)
        if meipass and (Path(meipass) / "robots").is_dir():
            return Path(meipass) / "robots"
        return Path(sys.executable).resolve().parent / "robots"
    return Path(__file__).resolve().parents[1] / "robots"


def list_sample_robots(root: Path | str | None = None) -> list[tuple[str, Path]]:
    """Return (title, urdf path) for every robots/<name>/<name>.urdf, sorted by title.

    The title is the first line of SOURCE.txt (folder name when missing).
    """
    base = Path(root) if root is not None else sample_robots_dir()
    out: list[tuple[str, Path]] = []
    if not base.is_dir():
        return out
    for folder in sorted(p for p in base.iterdir() if p.is_dir()):
        urdf = folder / f"{folder.name}.urdf"
        if not urdf.is_file():
            continue
        title = folder.name
        try:
            lines = (folder / "SOURCE.txt").read_text(encoding="utf-8", errors="replace").splitlines()
            if lines and lines[0].strip():
                title = lines[0].strip()
        except OSError:
            pass
        out.append((title, urdf))
    out.sort(key=lambda t: t[0].lower())
    return out


_GENERIC_ROBOT_NAMES = {"", "uf_robot", "robot", "unnamed robot"}


def import_display_name(path: str, urdf_name: str | None) -> str:
    """Project name for an imported URDF (U2.1).

    Sample robot -> sample title. Generic URDF name (empty, ``robot``, ``UF_ROBOT``)
    -> file stem. Otherwise the URDF robot name.
    """
    p = Path(path)
    try:
        rp = p.resolve()
        for title, urdf in list_sample_robots():
            if urdf.resolve() == rp:
                return title
    except OSError:
        pass
    name = (urdf_name or "").strip()
    if name.lower() in _GENERIC_ROBOT_NAMES:
        return p.stem or "Imported Robot"
    return name
