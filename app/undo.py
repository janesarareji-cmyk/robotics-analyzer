"""U5.9: undo / redo (Ctrl+Z, Ctrl+Y) for joint values, targets and scene objects.

One ``QUndoStack``. Each "domain" (q, targets, objects) gives a snapshot function and a restore
function. The window calls ``record(domain)`` after an edit; when the snapshot differs from the last
one a command is pushed. Fast repeats of the same domain (slider drag, jog hold) merge into one step.
"""
from __future__ import annotations

import copy
import time
from typing import Any, Callable

import numpy as np
from PySide6.QtCore import QObject, Signal
from PySide6.QtGui import QUndoCommand, QUndoStack

MERGE_WINDOW_S = 0.6
LIMIT = 100
TEXT = {"q": "Move joints", "targets": "Edit target", "objects": "Edit object"}
_IDS = {"q": 1, "targets": 2, "objects": 3}


def _same(a: Any, b: Any) -> bool:
    if isinstance(a, (list, tuple)) and a and all(isinstance(x, (int, float, np.floating)) for x in a) \
            and isinstance(b, (list, tuple)) and len(a) == len(b):
        return bool(np.allclose(a, b, atol=1e-12, rtol=0))
    return a == b


class _Command(QUndoCommand):
    def __init__(self, mgr: "UndoManager", domain: str, before: Any, after: Any):
        super().__init__(TEXT.get(domain, domain))
        self._mgr = mgr
        self.domain = domain
        self.before = before
        self.after = after
        self.stamp = time.monotonic()
        self._first = True

    def id(self) -> int:
        return _IDS.get(self.domain, 0)

    def undo(self) -> None:
        self._mgr._apply(self.domain, self.before)

    def redo(self) -> None:
        if self._first:                 # the edit is already done when the command is pushed
            self._first = False
            return
        self._mgr._apply(self.domain, self.after)

    def mergeWith(self, other: QUndoCommand) -> bool:
        if not isinstance(other, _Command) or other.domain != self.domain:
            return False
        if other.stamp - self.stamp > MERGE_WINDOW_S or other.stamp - self.stamp < 0:
            return False
        self.after = other.after
        self.stamp = other.stamp
        return True


class UndoManager(QObject):
    applied = Signal(str)               # domain restored by undo / redo

    def __init__(self, parent: QObject | None = None):
        super().__init__(parent)
        self.stack = QUndoStack(self)
        self.stack.setUndoLimit(LIMIT)
        self._domains: dict[str, tuple[Callable, Callable]] = {}
        self._last: dict[str, Any] = {}
        self._applying = False

    def register(self, domain: str, snapshot: Callable[[], Any], restore: Callable[[Any], None]) -> None:
        self._domains[domain] = (snapshot, restore)
        self._last[domain] = self._safe_snapshot(domain)

    def _safe_snapshot(self, domain: str) -> Any:
        try:
            return copy.deepcopy(self._domains[domain][0]())
        except Exception:
            return None

    def reset(self) -> None:
        """New project or robot: forget the history, take new baselines."""
        self.stack.clear()
        for d in self._domains:
            self._last[d] = self._safe_snapshot(d)

    def rebase(self, domain: str) -> None:
        """Take a new baseline of one domain without a history entry (change not caused by the user)."""
        if domain in self._domains:
            self._last[domain] = self._safe_snapshot(domain)

    def record(self, domain: str) -> bool:
        """Call after an edit. Pushes a command when the domain changed. True when pushed."""
        if self._applying or domain not in self._domains:
            return False
        cur = self._safe_snapshot(domain)
        old = self._last.get(domain)
        if cur is None or old is None or _same(cur, old):
            self._last[domain] = cur if cur is not None else old
            return False
        self._last[domain] = cur
        self.stack.push(_Command(self, domain, old, cur))
        return True

    def _apply(self, domain: str, state: Any) -> None:
        self._applying = True
        try:
            self._domains[domain][1](copy.deepcopy(state))
            self._last[domain] = self._safe_snapshot(domain)
        finally:
            self._applying = False
        self.applied.emit(domain)

    def undo(self) -> None:
        self.stack.undo()

    def redo(self) -> None:
        self.stack.redo()

    @property
    def can_undo(self) -> bool:
        return self.stack.canUndo()

    @property
    def can_redo(self) -> bool:
        return self.stack.canRedo()
