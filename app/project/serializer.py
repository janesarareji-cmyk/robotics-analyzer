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
import zipfile
from pathlib import Path
from typing import Any

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


class ProjectSerializationError(Exception):
    """Raised when a .robotproj file cannot be saved or loaded."""


class ProjectFormatError(ProjectSerializationError):
    """Raised when a .robotproj file has an unrecognized or incompatible format."""


def save(project: ProjectData, path: str | Path) -> None:
    """
    Save a ProjectData to a .robotproj file at the given path.
    Creates the file. Overwrites if it already exists.
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

    try:
        with zipfile.ZipFile(path, mode="w", compression=zipfile.ZIP_DEFLATED) as zf:
            _write_json(zf, _META_FILE, project.metadata.to_dict())
            _write_json(zf, _ROBOT_FILE, project.robot_model or {})
            _write_json(zf, _PHYSICS_FILE, project.physical_properties or {})
            _write_json(zf, _SIM_FILE, project.simulation_settings.to_dict())
    except (OSError, zipfile.BadZipFile) as exc:
        raise ProjectSerializationError(f"Failed to save project: {exc}") from exc

    project.file_path = str(path)


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

            robot_model = _read_json(zf, _ROBOT_FILE) if _ROBOT_FILE in names else {}
            physical_properties = (
                _read_json(zf, _PHYSICS_FILE) if _PHYSICS_FILE in names else {}
            )
            sim_dict = _read_json(zf, _SIM_FILE) if _SIM_FILE in names else {}
            simulation_settings = SimulationSettings.from_dict(sim_dict)

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

    return ProjectData(
        metadata=metadata,
        simulation_settings=simulation_settings,
        robot_model=robot_model if robot_model else None,
        physical_properties=physical_properties if physical_properties else None,
        file_path=str(path),
    )


# ------------------------------------------------------------------
# Internal helpers
# ------------------------------------------------------------------

def _write_json(zf: zipfile.ZipFile, name: str, data: Any) -> None:
    """Write a dict as JSON to a zip archive member."""
    text = json.dumps(data, indent=INDENT, ensure_ascii=False)
    zf.writestr(name, text.encode(ENCODING))


def _read_json(zf: zipfile.ZipFile, name: str) -> Any:
    """Read and parse a JSON archive member."""
    raw = zf.read(name)
    return json.loads(raw.decode(ENCODING))


def _check_format_version(version: str) -> None:
    """
    Raise ProjectFormatError if the format version is incompatible.
    Currently accepts any 1.x.x version.
    """
    major = version.split(".")[0] if version else "0"
    current_major = PROJECT_FORMAT_VERSION.split(".")[0]
    if major != current_major:
        raise ProjectFormatError(
            f"Incompatible .robotproj format version: {version!r}. "
            f"This application reads version {current_major}.x.x."
        )
