"""
app/project/serializer.py
Robotics Analyzer — .robotproj file format serialization.
R1.5: Save and load ProjectData as a ZIP archive of JSON files.

File format (.robotproj = ZIP):
  project_metadata.json      — ProjectMetadata fields
  robot_model.json           — Unified Robot Model (empty dict if not set)
  physical_properties.json   — Physical property data (empty dict if not set)
  simulation_settings.json   — SimulationSettings fields
"""

from __future__ import annotations

import json
import logging
import os
import shutil
import tempfile
import zipfile
import zlib
from pathlib import Path, PurePosixPath
from typing import Any

import core_bridge as cb

from app.project.migrations import migrate, is_newer, parse_version
from app.project.project_data import (
    ProjectData,
    ProjectMetadata,
    SimulationSettings,
    PROJECT_FORMAT_VERSION,
)


ROBOTPROJ_EXTENSION = ".robotproj"
ENCODING = "utf-8"
INDENT = 2

# Archive member names
_META_FILE = "project_metadata.json"
_ROBOT_FILE = "robot_model.json"
_PHYSICS_FILE = "physical_properties.json"
_SIM_FILE = "simulation_settings.json"
_VISUALS_FILE = "visuals.json"
_TRAJ_FILE = "trajectory.json"
_COLLISION_FILE = "collision.json"
_CONFIGS_FILE = "configs.json"
_ASSET_DIR = "assets"

log = logging.getLogger("robotics_analyzer.project")


class ProjectSerializationError(Exception):
    """Raised when a .robotproj file cannot be saved or loaded."""


class ProjectFormatError(ProjectSerializationError):
    """Raised when a .robotproj file has an unrecognized or incompatible format."""


def save(project: ProjectData, path: str | Path) -> None:
    """
    Save a ProjectData to a .robotproj file at the given path.
    Atomic (R8.4): written to <name>.tmp, fsynced, then moved over the target.
    The previous file stays as <name>.prev until the new one is in place.
    Updates project.metadata.modified_at and project.file_path.

    Raises ProjectSerializationError on I/O failure.
    """
    path = Path(path)
    if path.suffix != ROBOTPROJ_EXTENSION:
        raise ProjectSerializationError(
            f"Path must have extension '{ROBOTPROJ_EXTENSION}', got '{path.suffix}'"
        )

    # Update timestamp before saving
    project.touch()
    project.metadata.format_version = PROJECT_FORMAT_VERSION

    write_atomic(project, path)

    project.file_path = str(path)


def prev_path(path: str | Path) -> Path:
    """<name>.robotproj.prev: the previous file while a save is in progress."""
    p = Path(path)
    return p.with_name(p.name + ".prev")


def _write_archive(project: ProjectData, target: Path) -> None:
    """Write the project ZIP to ``target`` and fsync it."""
    with zipfile.ZipFile(target, mode="w", compression=zipfile.ZIP_DEFLATED) as zf:
        _write_json(zf, _META_FILE, project.metadata.to_dict())
        if project.robot_model is not None and hasattr(project.robot_model, "to_json"):
            zf.writestr(_ROBOT_FILE, project.robot_model.to_json().encode(ENCODING))
        elif isinstance(project.robot_model, dict):
            _write_json(zf, _ROBOT_FILE, project.robot_model)
        else:
            _write_json(zf, _ROBOT_FILE, {})
        _write_json(zf, _PHYSICS_FILE, project.physical_properties or {})
        _write_json(zf, _SIM_FILE, project.simulation_settings.to_dict())
        _write_visuals(zf, project)
        if getattr(project, "trajectory", None) is not None:
            _write_json(zf, _TRAJ_FILE, project.trajectory.to_dict())
        coll = getattr(project, "collision", None)
        if coll is not None and not coll.is_default():
            _write_json(zf, _COLLISION_FILE, coll.to_dict())
        configs = getattr(project, "configs", None)
        if configs is not None and len(configs) > 0:
            _write_json(zf, _CONFIGS_FILE, configs.to_dict())
    with open(target, "rb+") as fh:
        os.fsync(fh.fileno())


def write_atomic(project: ProjectData, path: str | Path, keep_prev: bool = True) -> None:
    """
    Atomic write of the project archive to ``path`` (does not touch project state).
    write <name>.tmp + fsync -> old file COPIED to <name>.prev -> tmp replaces
    target (one atomic os.replace) -> .prev removed. The target is never moved
    away, so a kill at any point leaves the old file or the new file (plus maybe
    a .prev copy), never a missing or truncated target.
    Raises ProjectSerializationError; the tmp file is always removed on failure.
    """
    path = Path(path)
    tmp = path.with_name(path.name + ".tmp")
    prev = prev_path(path)
    try:
        _write_archive(project, tmp)
    except (OSError, zipfile.BadZipFile) as exc:
        tmp.unlink(missing_ok=True)
        raise ProjectSerializationError(f"Failed to save project: {exc}") from exc
    except BaseException:
        tmp.unlink(missing_ok=True)
        raise
    copied = False
    try:
        # never overwrite a good .prev with a damaged target
        if keep_prev and path.exists() and zipfile.is_zipfile(path):
            shutil.copy2(path, prev)
            copied = True
        os.replace(tmp, path)
    except OSError as exc:
        tmp.unlink(missing_ok=True)
        if copied:                      # target is untouched: the .prev copy is redundant
            prev.unlink(missing_ok=True)
        raise ProjectSerializationError(f"Failed to save project: {exc}") from exc
    if copied:
        prev.unlink(missing_ok=True)


def load(path: str | Path) -> ProjectData:
    """
    Load a ProjectData from a .robotproj file.
    Returns a fully populated ProjectData with file_path set.

    Raises ProjectFormatError if the file is not a valid .robotproj archive.
    Raises ProjectSerializationError on I/O failure.
    """
    path = Path(path)

    if not path.exists():
        raise ProjectSerializationError(f"File not found: {path}")

    try:
        with zipfile.ZipFile(path, mode="r") as zf:
            names = zf.namelist()

            if _META_FILE not in names:
                raise ProjectFormatError(
                    f"Invalid .robotproj: missing '{_META_FILE}'"
                )

            meta_dict = _read_json(zf, _META_FILE)
            metadata = ProjectMetadata.from_dict(meta_dict)

            _check_format_version(metadata.format_version)
            file_version = metadata.format_version

            if _ROBOT_FILE in names:
                raw_robot = zf.read(_ROBOT_FILE).decode(ENCODING).strip()
                if not raw_robot or raw_robot == "{}":
                    robot_model = cb.RobotModel()
                else:
                    try:
                        robot_model = cb.RobotModel.from_json(raw_robot)
                    except Exception as exc:
                        raise ProjectFormatError(
                            f"Invalid robot model in project: {exc}"
                        ) from exc
            else:
                robot_model = cb.RobotModel()

            physical_properties = (
                _read_json(zf, _PHYSICS_FILE) if _PHYSICS_FILE in names else {}
            )
            sim_dict = _read_json(zf, _SIM_FILE) if _SIM_FILE in names else {}
            # R8.4: chained in-memory migration (file is never rewritten here)
            docs = {_META_FILE: meta_dict, _SIM_FILE: sim_dict,
                    _PHYSICS_FILE: physical_properties}
            steps = migrate(docs, file_version)
            sim_dict, physical_properties = docs[_SIM_FILE], docs[_PHYSICS_FILE]
            simulation_settings = SimulationSettings.from_dict(sim_dict)
            visuals, collisions, asset_dir = _read_visuals(zf, names)
            trajectory = _read_trajectory(zf, names)
            collision = _read_collision(zf, names)
            configs = _read_configs(zf, names)

    except zipfile.BadZipFile as exc:
        raise ProjectSerializationError(
            f"File is not a valid .robotproj archive: {exc}"
        ) from exc
    except json.JSONDecodeError as exc:
        raise ProjectSerializationError(
            f"Corrupt JSON in .robotproj archive: {exc}"
        ) from exc
    except OSError as exc:
        raise ProjectSerializationError(f"I/O error loading project: {exc}") from exc
    except ProjectSerializationError:
        raise
    except (KeyError, TypeError, ValueError, AttributeError, EOFError,
            zlib.error, zipfile.LargeZipFile) as exc:
        # A1.2 (audit H2): any other damage is a format error, so .prev fallback and
        # the "Cannot Open Project" dialog work.
        raise ProjectFormatError(
            f"Invalid project content ({type(exc).__name__}: {exc})"
        ) from exc

    return ProjectData(
        metadata=metadata,
        simulation_settings=simulation_settings,
        robot_model=robot_model,
        physical_properties=physical_properties if physical_properties else None,
        file_path=str(path),
        visuals=visuals,
        collisions=collisions,
        asset_dir=asset_dir,
        trajectory=trajectory,
        collision=collision,
        configs=configs,
        migrated_from=file_version if steps else None,
    )


# ------------------------------------------------------------------
# Internal helpers
# ------------------------------------------------------------------

def _write_json(zf: zipfile.ZipFile, name: str, data: Any) -> None:
    """Write a dict as JSON to a zip archive member."""
    text = json.dumps(data, indent=INDENT, ensure_ascii=False)
    zf.writestr(name, text.encode(ENCODING))


def _write_visuals(zf: zipfile.ZipFile, project: ProjectData) -> None:
    """Write visuals.json and copy each distinct mesh file into assets/."""
    name_of: dict[str, str] = {}     # source abs path -> archive name
    used: set[str] = set()

    def archive_name(src: str) -> str:
        if src in name_of:
            return name_of[src]
        base = Path(src).name
        stem, suffix = Path(base).stem, Path(base).suffix
        cand, n = base, 0
        while cand in used:
            n += 1
            cand = f"{stem}_{n}{suffix}"
        used.add(cand)
        zf.write(src, f"{_ASSET_DIR}/{cand}")
        name_of[src] = cand
        return cand

    def dump(items: list[Any]) -> list[dict[str, Any]]:
        out = []
        for v in items:
            d = v.to_dict()
            src = d.get("mesh_path")
            if src and not d.get("fallback") and Path(src).is_file():
                d["mesh_path"] = f"{_ASSET_DIR}/{archive_name(src)}"
            out.append(d)
        return out

    _write_json(zf, _VISUALS_FILE, {
        "visuals": dump(project.visuals),
        "collisions": dump(project.collisions),
    })


def _read_visuals(zf: zipfile.ZipFile, names: list[str]):
    """Return (visuals, collisions, asset_dir). Old 1.0.0 files: empty lists."""
    from app.importers.visuals import LinkVisual

    if _VISUALS_FILE not in names:
        return [], [], None
    data = _read_json(zf, _VISUALS_FILE)
    asset_names = [n for n in names
                   if n.startswith(_ASSET_DIR + "/") and not n.endswith("/")]
    asset_dir = None
    if asset_names:
        asset_dir = tempfile.mkdtemp(prefix="robotproj_assets_")
        for n in asset_names:
            target = Path(asset_dir) / PurePosixPath(n).name   # no zip-slip
            target.write_bytes(zf.read(n))

    def load(items: list[dict[str, Any]]) -> list[Any]:
        out = []
        for d in items:
            v = LinkVisual.from_dict(d)
            mp = v.mesh_path
            if mp and mp.startswith(_ASSET_DIR + "/"):
                real = Path(asset_dir) / PurePosixPath(mp).name if asset_dir else None
                if real is not None and real.is_file():
                    v.mesh_path = str(real)
                else:
                    log.warning("Missing asset '%s' in project archive; "
                                "placeholder cube is used", mp)
                    v.mesh_path = None
                    v.fallback = True
            out.append(v)
        return out

    return (load(data.get("visuals", [])), load(data.get("collisions", [])),
            asset_dir)


def _read_trajectory(zf: zipfile.ZipFile, names: list[str]) -> Any:
    """trajectory.json -> Trajectory. Absent (1.0/1.1 files) or invalid: None."""
    if _TRAJ_FILE not in names:
        return None
    from app.sim.trajectory import Trajectory
    try:
        return Trajectory.from_dict(_read_json(zf, _TRAJ_FILE))
    except (KeyError, TypeError, ValueError) as exc:
        log.warning("Invalid trajectory in project archive ignored: %s", exc)
        return None


def _read_collision(zf: zipfile.ZipFile, names: list[str]) -> Any:
    """collision.json -> CollisionConfig. Absent (pre 1.3.0 files) or invalid: None."""
    if _COLLISION_FILE not in names:
        return None
    from app.planning.collision import CollisionConfig
    try:
        return CollisionConfig.from_dict(_read_json(zf, _COLLISION_FILE))
    except (KeyError, TypeError, ValueError) as exc:
        log.warning("Invalid collision data in project archive ignored: %s", exc)
        return None


def _read_configs(zf: zipfile.ZipFile, names: list[str]) -> Any:
    """configs.json -> ConfigStore. Absent (pre 1.3.0 files) or invalid: empty store."""
    from app.configs.store import ConfigStore
    if _CONFIGS_FILE not in names:
        return ConfigStore()
    try:
        return ConfigStore.from_dict(_read_json(zf, _CONFIGS_FILE))
    except (KeyError, TypeError, ValueError) as exc:
        log.warning("Invalid configurations in project archive ignored: %s", exc)
        return ConfigStore()


def _read_json(zf: zipfile.ZipFile, name: str) -> Any:
    """Read and parse a JSON archive member."""
    raw = zf.read(name)
    try:
        text = raw.decode(ENCODING)
    except UnicodeDecodeError as exc:
        raise ProjectFormatError(f"'{name}' is not valid UTF-8: {exc}") from exc
    return json.loads(text)


def _check_format_version(version: str) -> None:
    """
    Raise ProjectFormatError for a version this app cannot read:
    unknown/garbled, other major, or NEWER than PROJECT_FORMAT_VERSION
    (refused, nothing is written). Older 1.x.x versions are migrated.
    """
    if parse_version(version) is None:
        raise ProjectFormatError(f"Unknown .robotproj format version: {version!r}.")
    if is_newer(version):
        raise ProjectFormatError(
            f"This project was saved by a newer version of the application "
            f"(format {version}); this application reads up to "
            f"{PROJECT_FORMAT_VERSION}. Update the application. The file was not changed."
        )
    major = version.split(".")[0]
    current_major = PROJECT_FORMAT_VERSION.split(".")[0]
    if major != current_major:
        raise ProjectFormatError(
            f"Incompatible .robotproj format version: {version!r}. "
            f"This application reads version {current_major}.x.x."
        )
    from app.project.migrations import MIGRATIONS
    if version != PROJECT_FORMAT_VERSION and version not in MIGRATIONS:
        raise ProjectFormatError(f"Unknown .robotproj format version: {version!r}.")
