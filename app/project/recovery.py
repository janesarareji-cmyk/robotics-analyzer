"""
app/project/recovery.py
R8.4: autosave and crash-recovery file names and helpers (see docs/project-recovery.md).

  <name>.robotproj                 the project
  <name>.autosave.robotproj        autosave (only while modified; removed on clean save/close)
  <name>.robotproj.prev            previous file during a save (removed after success)
  <name>.robotproj.tmp             file being written (removed after success/failure)
  <name>.v<old>.bak.robotproj      backup made once when an old-format file is opened
"""

from __future__ import annotations

import dataclasses
import logging
from pathlib import Path

from app.project.project_data import PROJECT_FORMAT_VERSION, ProjectData
from app.project.serializer import (
    ProjectSerializationError,
    load,
    prev_path,
    write_atomic,
)

log = logging.getLogger("robotics_analyzer.project")


def autosave_path(path: str | Path) -> Path:
    """<name>.autosave.robotproj next to the project."""
    p = Path(path)
    return p.with_name(f"{p.stem}.autosave{p.suffix}")


def write_autosave(project: ProjectData, path: str | Path) -> Path:
    """Write the autosave atomically. Does not change the project (path, timestamp)."""
    target = autosave_path(path)
    meta = dataclasses.replace(project.metadata, format_version=PROJECT_FORMAT_VERSION)
    snapshot = dataclasses.replace(project, metadata=meta)
    write_atomic(snapshot, target, keep_prev=False)
    return target


def remove_autosave(path: str | Path) -> None:
    """Delete the autosave (and a stale tmp of it) if present."""
    a = autosave_path(path)
    for f in (a, a.with_name(a.name + ".tmp")):
        try:
            f.unlink(missing_ok=True)
        except OSError as exc:
            log.warning("Cannot remove %s: %s", f, exc)


def newer_autosave(path: str | Path) -> Path | None:
    """The autosave if it exists and is newer than the project (or the project is gone)."""
    p, a = Path(path), autosave_path(path)
    if not a.is_file():
        return None
    if not p.is_file():
        return a
    return a if a.stat().st_mtime > p.stat().st_mtime else None


def load_best(path: str | Path) -> tuple[ProjectData, Path]:
    """
    Open ``path``; if that fails and <name>.prev exists, open the .prev.
    Returns (project, file actually opened). Raises the first error if both fail.
    """
    p = Path(path)
    try:
        return load(p), p
    except ProjectSerializationError as first:
        prev = prev_path(p)
        if prev.is_file():
            try:
                proj = load(prev)
            except ProjectSerializationError:
                raise first
            log.warning("Project %s unreadable (%s); opened %s", p, first, prev)
            return proj, prev
        raise
