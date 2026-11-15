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


# D1.1: human display data for the start screen (folder name -> name, maker, dof, kind).
# dof = joints the user moves (gripper fingers not counted); checked against the models in tests.
SAMPLE_INFO: dict[str, dict] = {
    "ur3": {"name": "UR3", "maker": "Universal Robots", "dof": 6, "kind": "6-axis arm"},
    "ur5": {"name": "UR5", "maker": "Universal Robots", "dof": 6, "kind": "6-axis arm"},
    "ur10": {"name": "UR10", "maker": "Universal Robots", "dof": 6, "kind": "6-axis arm"},
    "xarm5": {"name": "UFACTORY xArm 5", "maker": "UFACTORY", "dof": 5, "kind": "5-axis arm"},
    "xarm5_gripper": {"name": "UFACTORY xArm 5 + Gripper", "maker": "UFACTORY", "dof": 5,
                      "kind": "5-axis arm + gripper"},
    "xarm6": {"name": "UFACTORY xArm 6", "maker": "UFACTORY", "dof": 6, "kind": "6-axis arm"},
    "xarm7": {"name": "UFACTORY xArm 7", "maker": "UFACTORY", "dof": 7, "kind": "7-axis arm (redundant)"},
    "kuka_iiwa": {"name": "KUKA LBR iiwa 14", "maker": "KUKA", "dof": 7, "kind": "7-axis arm (redundant)"},
    "panda": {"name": "Franka Panda", "maker": "Franka Emika", "dof": 7, "kind": "7-axis arm + 2-finger hand"},
}


def sample_info(stem: str, fallback_title: str = "") -> dict:
    """Display data of the sample robot folder ``stem`` (fallback when unknown)."""
    info = SAMPLE_INFO.get(stem)
    if info:
        return dict(info)
    return {"name": fallback_title or stem, "maker": "", "dof": 0, "kind": ""}


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
