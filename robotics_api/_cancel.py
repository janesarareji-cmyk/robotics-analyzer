"""Cooperative cancel for long API calls (R8.1).

The script runner installs a ``CancelToken`` for its worker thread (``cancel_scope``). Long API
calls (``simulate``, ``plan``, ``workspace``) call ``check_cancel()`` often; it raises
``ScriptCancelled`` (or ``ScriptTimeLimit``) when the token is set. The token is thread-local:
code running in another thread started by a script is not cancelled.
"""
from __future__ import annotations

import threading
import time
from contextlib import contextmanager

from .errors import ScriptCancelled, ScriptTimeLimit

_local = threading.local()


class CancelToken:
    """Cancel flag with an optional time limit (seconds, ``None`` or 0 = no limit)."""

    def __init__(self, time_limit: float | None = None):
        self._event = threading.Event()
        self._deadline = (time.monotonic() + float(time_limit)) if time_limit else None
        self.timed_out = False

    def cancel(self) -> None:
        self._event.set()

    def is_cancelled(self) -> bool:
        if self._event.is_set():
            return True
        if self._deadline is not None and time.monotonic() >= self._deadline:
            self.timed_out = True
            self._event.set()
            return True
        return False

    __call__ = is_cancelled


def current_token() -> CancelToken | None:
    return getattr(_local, "token", None)


@contextmanager
def cancel_scope(token: CancelToken | None):
    """Install ``token`` for the current thread inside the ``with`` block."""
    prev = current_token()
    _local.token = token
    try:
        yield token
    finally:
        _local.token = prev


def check_cancel() -> None:
    """Raise ``ScriptCancelled`` when the running script was cancelled (no-op outside a runner)."""
    tok = current_token()
    if tok is not None and tok.is_cancelled():
        raise (ScriptTimeLimit("script time limit reached") if tok.timed_out
               else ScriptCancelled("script cancelled"))


def is_cancelled() -> bool:
    """True when a cancel was requested (does not raise)."""
    tok = current_token()
    return bool(tok is not None and tok.is_cancelled())
