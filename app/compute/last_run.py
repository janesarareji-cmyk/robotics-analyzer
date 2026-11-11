"""R5.6: small holder for the last compute run info (fed by the Workspace panel)."""
from __future__ import annotations

from typing import Any

_last: dict[str, Any] | None = None


def set_last_run(backend: str, options: dict | None, info: dict | None) -> None:
    global _last
    _last = {"backend": str(backend), "options": dict(options or {}), "info": dict(info or {})}


def get_last_run() -> dict[str, Any] | None:
    if _last is None:
        return None
    return {"backend": _last["backend"], "options": dict(_last["options"]),
            "info": dict(_last["info"])}


def clear_last_run() -> None:
    global _last
    _last = None
