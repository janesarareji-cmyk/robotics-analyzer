"""
app/project/migrations.py
R8.4: project format migrations.

Every PROJECT_FORMAT_VERSION step has a function in MIGRATIONS. A migration
works IN MEMORY on the parsed JSON members of the archive (``docs``: member
name -> parsed JSON) and returns nothing; it never touches the file. The file
is rewritten only when the user saves.

History (all steps so far only ADDED optional archive members, so the
functions are no-ops that document the step):
  1.0.0 -> 1.1.0   + visuals.json, assets/
  1.1.0 -> 1.2.0   + trajectory.json
  1.2.0 -> 1.3.0   + collision.json, configs.json
"""

from __future__ import annotations

import logging
import shutil
from pathlib import Path
from typing import Any, Callable

from app.project.project_data import PROJECT_FORMAT_VERSION

log = logging.getLogger("robotics_analyzer.project")

Docs = dict[str, Any]

# Ordered list of every format version ever written.
VERSIONS = ("1.0.0", "1.1.0", "1.2.0", "1.3.0")


def parse_version(text: Any) -> tuple[int, int, int] | None:
    """'1.2.0' -> (1, 2, 0); anything else -> None."""
    if not isinstance(text, str):
        return None
    parts = text.strip().split(".")
    if len(parts) != 3 or not all(p.isdigit() for p in parts):
        return None
    return int(parts[0]), int(parts[1]), int(parts[2])


def is_newer(version: str) -> bool:
    """True if ``version`` is newer than this application's format."""
    v, cur = parse_version(version), parse_version(PROJECT_FORMAT_VERSION)
    return v is not None and cur is not None and v > cur


def is_older(version: str) -> bool:
    v, cur = parse_version(version), parse_version(PROJECT_FORMAT_VERSION)
    return v is not None and cur is not None and v < cur


def _m_1_0_to_1_1(docs: Docs) -> None:
    """Added visuals.json / assets/ (optional). Nothing to convert."""


def _m_1_1_to_1_2(docs: Docs) -> None:
    """Added trajectory.json (optional). Nothing to convert."""


def _m_1_2_to_1_3(docs: Docs) -> None:
    """Added collision.json and configs.json (optional). Nothing to convert."""


MIGRATIONS: dict[str, Callable[[Docs], None]] = {
    "1.0.0": _m_1_0_to_1_1,
    "1.1.0": _m_1_1_to_1_2,
    "1.2.0": _m_1_2_to_1_3,
}

_NEXT = {"1.0.0": "1.1.0", "1.1.0": "1.2.0", "1.2.0": "1.3.0"}


def migrate(docs: Docs, from_version: str) -> list[str]:
    """
    Run the chained migrations from ``from_version`` up to the current format.
    Returns the list of versions the steps started from (e.g. ['1.0.0', '1.1.0']).
    Unknown or newer versions are not handled here (the loader refuses them).
    """
    steps: list[str] = []
    v = from_version
    while v != PROJECT_FORMAT_VERSION:
        fn = MIGRATIONS.get(v)
        if fn is None:
            raise KeyError(f"no migration from format version {v!r}")
        fn(docs)
        steps.append(v)
        v = _NEXT[v]
    return steps


def backup_path(path: str | Path, old_version: str) -> Path:
    """<name>.v<old>.bak.robotproj next to the file."""
    p = Path(path)
    return p.with_name(f"{p.stem}.v{old_version}.bak{p.suffix}")


def make_backup(path: str | Path, old_version: str) -> Path | None:
    """
    Copy an old-format file to its backup name BEFORE it is migrated.
    Created once: an existing backup is never overwritten. Returns the backup
    path when one was created now, else None.
    """
    src, dst = Path(path), backup_path(path, old_version)
    if dst.exists():
        return None
    try:
        shutil.copy2(src, dst)
    except OSError as exc:
        log.warning("Cannot create backup %s: %s", dst, exc)
        return None
    log.info("Backup of old-format project created: %s", dst)
    return dst
