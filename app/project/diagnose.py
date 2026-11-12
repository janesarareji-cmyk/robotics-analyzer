"""
app/project/diagnose.py
R8.4: diagnose a damaged .robotproj and salvage what can be loaded.

diagnose(path) never raises; it returns a DiagnosisReport with human-readable
messages and a suggested action per issue. salvage(path) loads what it can.
"""

from __future__ import annotations

import json
import logging
import zipfile
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import core_bridge as cb

from app.project import serializer as ser
from app.project.migrations import is_newer, parse_version
from app.project.project_data import (
    PROJECT_FORMAT_VERSION,
    ProjectData,
    ProjectMetadata,
    SimulationSettings,
)

log = logging.getLogger("robotics_analyzer.project")

# issue codes
NOT_FOUND = "not_found"
NOT_A_ZIP = "not_a_zip"
BAD_CRC = "bad_crc"
MISSING_METADATA = "missing_metadata"
INVALID_JSON = "invalid_json"
WRONG_SCHEMA = "wrong_schema"
MISSING_ASSET = "missing_asset"
UNKNOWN_VERSION = "unknown_version"
NEWER_VERSION = "newer_version"

_MAJOR = PROJECT_FORMAT_VERSION.split(".")[0]


@dataclass
class Issue:
    code: str
    severity: str            # "error" | "warning"
    message: str
    action: str              # suggested action
    member: str | None = None


@dataclass
class DiagnosisReport:
    path: str
    issues: list[Issue] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return not self.issues

    @property
    def codes(self) -> list[str]:
        return [i.code for i in self.issues]

    def has(self, code: str) -> bool:
        return code in self.codes

    def text(self) -> str:
        if self.ok:
            return f"{self.path}: no problems found."
        return "\n".join(f"[{i.severity}] {i.message} Suggested action: {i.action}"
                         for i in self.issues)

    def add(self, code, severity, message, action, member=None) -> None:
        self.issues.append(Issue(code, severity, message, action, member))


_JSON_MEMBERS = (ser._META_FILE, ser._ROBOT_FILE, ser._PHYSICS_FILE, ser._SIM_FILE,
                 ser._VISUALS_FILE, ser._TRAJ_FILE, ser._COLLISION_FILE, ser._CONFIGS_FILE)

_BACKUP_HINT = "Open <name>.robotproj.prev, the autosave or a .bak copy instead."


def diagnose(path: str | Path) -> DiagnosisReport:
    """Inspect a .robotproj file without loading it into the application."""
    p = Path(path)
    rep = DiagnosisReport(str(p))
    if not p.is_file():
        rep.add(NOT_FOUND, "error", f"File not found: {p}.",
                "Check the path; look for <name>.robotproj.prev or <name>.autosave.robotproj.")
        return rep
    if not zipfile.is_zipfile(p):
        rep.add(NOT_A_ZIP, "error",
                "The file is not a ZIP archive (empty, truncated or not a .robotproj).",
                _BACKUP_HINT)
        return rep
    try:
        zf = zipfile.ZipFile(p)
    except zipfile.BadZipFile as exc:
        rep.add(NOT_A_ZIP, "error", f"The ZIP structure is damaged ({exc}).", _BACKUP_HINT)
        return rep
    with zf:
        names = zf.namelist()
        bad: set[str] = set()
        for info in zf.infolist():
            try:
                zf.read(info)
            except (zipfile.BadZipFile, OSError, RuntimeError) as exc:
                bad.add(info.filename)
                rep.add(BAD_CRC, "error",
                        f"Archive member '{info.filename}' is damaged ({exc}).",
                        "Salvage the rest (partial load) or use a backup copy.",
                        info.filename)

        docs: dict[str, Any] = {}
        for m in _JSON_MEMBERS:
            if m not in names or m in bad:
                continue
            try:
                docs[m] = json.loads(zf.read(m).decode(ser.ENCODING))
            except json.JSONDecodeError as exc:
                rep.add(INVALID_JSON, "error",
                        f"'{m}' is not valid JSON (line {exc.lineno}, column {exc.colno}: "
                        f"{exc.msg}).",
                        "Fix the text at that position, or open the backup / .prev / "
                        "autosave copy.", m)
            except UnicodeDecodeError as exc:
                rep.add(INVALID_JSON, "error", f"'{m}' is not valid UTF-8 ({exc}).",
                        "Use a backup copy.", m)

        if ser._META_FILE not in names:
            rep.add(MISSING_METADATA, "error",
                    f"The archive has no '{ser._META_FILE}' (project name and version).",
                    "Salvage the rest (partial load) or use a backup copy.",
                    ser._META_FILE)
        elif ser._META_FILE in docs:
            _check_meta(rep, docs[ser._META_FILE])

        _check_schema(rep, docs, names)

        vis = docs.get(ser._VISUALS_FILE)
        if isinstance(vis, dict):
            for key in ("visuals", "collisions"):
                for d in vis.get(key, []) or []:
                    mp = d.get("mesh_path") if isinstance(d, dict) else None
                    if (isinstance(mp, str) and mp.startswith(ser._ASSET_DIR + "/")
                            and not d.get("fallback") and mp not in names):
                        rep.add(MISSING_ASSET, "warning",
                                f"Mesh '{mp}' is listed in {ser._VISUALS_FILE} but missing "
                                f"in the archive; a placeholder cube is used.",
                                "Re-import the robot or restore the mesh file.", mp)
    return rep


def _check_meta(rep: DiagnosisReport, meta: Any) -> None:
    m = ser._META_FILE
    if not isinstance(meta, dict) or not isinstance(meta.get("name"), str):
        rep.add(WRONG_SCHEMA, "error",
                f"'{m}': field 'name' is missing or not text.",
                "Salvage the rest (partial load); the name is replaced by the file name.", m)
        return
    v = meta.get("format_version", PROJECT_FORMAT_VERSION)
    if parse_version(v) is None or str(v).split(".")[0] != _MAJOR:
        rep.add(UNKNOWN_VERSION, "error",
                f"Unknown format version {v!r}; this application reads {_MAJOR}.x.x.",
                "Use the application version that wrote the file.", m)
    elif is_newer(v):
        rep.add(NEWER_VERSION, "error",
                f"Saved by a newer application (format {v}); this one reads "
                f"up to {PROJECT_FORMAT_VERSION}.",
                "Update the application. The file is not changed.", m)


def _check_schema(rep: DiagnosisReport, docs: dict[str, Any], names: list[str]) -> None:
    sim = docs.get(ser._SIM_FILE)
    if sim is not None:
        try:
            SimulationSettings.from_dict(sim)
        except (KeyError, TypeError, ValueError, AttributeError) as exc:
            rep.add(WRONG_SCHEMA, "error",
                    f"'{ser._SIM_FILE}': invalid field ({exc}).",
                    "Salvage the rest (partial load); default settings are used.",
                    ser._SIM_FILE)
    if ser._ROBOT_FILE in docs:
        raw = docs[ser._ROBOT_FILE]
        if raw not in ({}, None):
            try:
                cb.RobotModel.from_json(json.dumps(raw))
            except Exception as exc:  # noqa: BLE001 - bridge raises generic errors
                rep.add(WRONG_SCHEMA, "error",
                        f"'{ser._ROBOT_FILE}': not a valid robot model ({exc}).",
                        "Salvage the rest (partial load); an empty robot is used.",
                        ser._ROBOT_FILE)


def salvage(path: str | Path) -> tuple[ProjectData | None, list[str]]:
    """
    Load what can be loaded; every skipped part becomes a warning.
    Returns (project or None, warnings). None only if the file is not a ZIP at all.
    """
    p = Path(path)
    warnings: list[str] = []
    if not p.is_file() or not zipfile.is_zipfile(p):
        return None, [f"{p}: not a readable ZIP archive; nothing to salvage."]
    try:
        zf = zipfile.ZipFile(p)
    except zipfile.BadZipFile as exc:
        return None, [f"{p}: damaged ZIP ({exc}); nothing to salvage."]
    with zf:
        names = zf.namelist()

        def get(member: str, default: Any, what: str) -> Any:
            if member not in names:
                return default
            try:
                return json.loads(zf.read(member).decode(ser.ENCODING))
            except (json.JSONDecodeError, UnicodeDecodeError, zipfile.BadZipFile,
                    OSError) as exc:
                warnings.append(f"{what} ('{member}') skipped: {exc}")
                return default

        meta_d = get(ser._META_FILE, None, "Project metadata")
        try:
            meta = ProjectMetadata.from_dict(meta_d)
            if parse_version(meta.format_version) is None or is_newer(meta.format_version):
                raise ValueError(f"unusable format version {meta.format_version!r}")
        except (KeyError, TypeError, ValueError, AttributeError) as exc:
            warnings.append(f"Project metadata unusable ({exc}); name taken from file name.")
            meta = ProjectMetadata(name=p.stem)

        model = cb.RobotModel()
        if ser._ROBOT_FILE in names:
            try:
                raw = zf.read(ser._ROBOT_FILE).decode(ser.ENCODING).strip()
                if raw and raw != "{}":
                    model = cb.RobotModel.from_json(raw)
            except Exception as exc:  # noqa: BLE001
                warnings.append(f"Robot model skipped ({exc}); empty robot used.")
                model = cb.RobotModel()

        phys = get(ser._PHYSICS_FILE, {}, "Physical properties")
        sim_d = get(ser._SIM_FILE, {}, "Simulation settings")
        try:
            sim = SimulationSettings.from_dict(sim_d if isinstance(sim_d, dict) else {})
        except (KeyError, TypeError, ValueError) as exc:
            warnings.append(f"Simulation settings invalid ({exc}); defaults used.")
            sim = SimulationSettings()
        try:
            visuals, collisions, asset_dir = ser._read_visuals(zf, names)
        except Exception as exc:  # noqa: BLE001
            warnings.append(f"Visuals skipped ({exc}).")
            visuals, collisions, asset_dir = [], [], None
        try:
            trajectory = ser._read_trajectory(zf, names)
            collision = ser._read_collision(zf, names)
            configs = ser._read_configs(zf, names)
        except Exception as exc:  # noqa: BLE001
            warnings.append(f"Trajectory/collision/configurations skipped ({exc}).")
            from app.configs.store import ConfigStore
            trajectory, collision, configs = None, None, ConfigStore()

    for w in warnings:
        log.warning("salvage %s: %s", p, w)
    return ProjectData(
        metadata=meta, simulation_settings=sim, robot_model=model,
        physical_properties=phys if isinstance(phys, dict) and phys else None,
        file_path=None, visuals=visuals, collisions=collisions, asset_dir=asset_dir,
        trajectory=trajectory, collision=collision, configs=configs,
    ), warnings
