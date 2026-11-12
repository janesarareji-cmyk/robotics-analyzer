"""Plugin manifest ``plugin.json`` (R8.2). Pure data validation: no plugin code runs here."""
from __future__ import annotations

import json
import re
from dataclasses import dataclass
from pathlib import Path

import robotics_api

KINDS = ("importer", "planner", "analysis")
PERMISSIONS = ("file_read", "file_write", "network", "gpu")
REQUIRED_KEYS = ("name", "version", "api_version", "entry_point", "kind")
OPTIONAL_KEYS = ("permissions", "description")
_NAME_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,63}$")
_VER_RE = re.compile(r"^\d+\.\d+(\.\d+)?$")
_API_RE = re.compile(r"^(\d+)\.(\d+)$")
_ENTRY_RE = re.compile(r"^([A-Za-z_]\w*(?:\.[A-Za-z_]\w*)*):([A-Za-z_]\w*)$")


class ManifestError(ValueError):
    """The manifest is invalid; the message says why."""


@dataclass(frozen=True)
class Manifest:
    name: str
    version: str
    api_version: str
    entry_point: str
    kind: str
    permissions: tuple = ()
    description: str = ""

    @property
    def module(self) -> str:
        return self.entry_point.split(":")[0]

    @property
    def function(self) -> str:
        return self.entry_point.split(":")[1]


def api_compatible(api_version: str, current: str | None = None) -> bool:
    """Same MAJOR as the running ``robotics_api`` and MINOR not newer than it."""
    m = _API_RE.match(str(api_version))
    c = _API_RE.match(str(current or robotics_api.__api_version__))
    if not m or not c:
        return False
    return int(m.group(1)) == int(c.group(1)) and int(m.group(2)) <= int(c.group(2))


def parse_manifest(data) -> Manifest:
    """Validate a manifest dict. Raises ``ManifestError`` with a clear message."""
    if not isinstance(data, dict):
        raise ManifestError("manifest must be a JSON object")
    unknown = sorted(set(data) - set(REQUIRED_KEYS) - set(OPTIONAL_KEYS))
    if unknown:
        raise ManifestError(f"unknown key(s): {', '.join(map(str, unknown))}")
    missing = [k for k in REQUIRED_KEYS if k not in data]
    if missing:
        raise ManifestError(f"missing key(s): {', '.join(missing)}")
    for k in REQUIRED_KEYS:
        if not isinstance(data[k], str) or not data[k].strip():
            raise ManifestError(f"'{k}' must be a non-empty string")
    if not _NAME_RE.match(data["name"]):
        raise ManifestError("'name' may use letters, digits, '_', '.', '-' (max 64)")
    if not _VER_RE.match(data["version"]):
        raise ManifestError("'version' must look like 1.2 or 1.2.3")
    if not _API_RE.match(data["api_version"]):
        raise ManifestError("'api_version' must look like 1.0")
    if not api_compatible(data["api_version"]):
        raise ManifestError(
            f"api_version {data['api_version']} is not compatible with robotics_api "
            f"{robotics_api.__api_version__}")
    if not _ENTRY_RE.match(data["entry_point"]):
        raise ManifestError("'entry_point' must be 'module:function'")
    if data["kind"] not in KINDS:
        raise ManifestError(f"unknown kind {data['kind']!r}, use one of {KINDS}")
    perms = data.get("permissions", [])
    if not isinstance(perms, list) or not all(isinstance(p, str) for p in perms):
        raise ManifestError("'permissions' must be a list of strings")
    bad = [p for p in perms if p not in PERMISSIONS]
    if bad:
        raise ManifestError(f"unknown permission(s): {', '.join(bad)}; use {PERMISSIONS}")
    if len(set(perms)) != len(perms):
        raise ManifestError("'permissions' has duplicates")
    desc = data.get("description", "")
    if not isinstance(desc, str):
        raise ManifestError("'description' must be a string")
    return Manifest(data["name"], data["version"], data["api_version"], data["entry_point"],
                    data["kind"], tuple(perms), desc)


def load_manifest(path) -> Manifest:
    """Read and validate a ``plugin.json`` file."""
    try:
        text = Path(path).read_text(encoding="utf-8")
    except OSError as exc:
        raise ManifestError(f"cannot read manifest: {exc}") from exc
    try:
        data = json.loads(text)
    except ValueError as exc:
        raise ManifestError(f"manifest is not valid JSON: {exc}") from exc
    return parse_manifest(data)
