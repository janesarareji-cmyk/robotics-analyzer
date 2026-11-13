"""Plugin permissions (R8.2): declared, asked once, remembered per plugin in settings.

Cooperative guard: it stops honest plugins from overstepping, it does NOT stop hostile code
(plugins run in this process). The dialog text says so.
"""
from __future__ import annotations

import threading
import time
from typing import Callable

from PySide6.QtCore import QCoreApplication, QObject, QThread, Signal, Slot
from PySide6.QtWidgets import QMessageBox

from .manifest import PERMISSIONS

DIALOG_NOTE = ("This is a cooperative guard, not a sandbox: plugins run inside the application "
               "and hostile code can bypass it. Grant only to plugins you trust.")
_MEANING = {
    "file_read": "read files you picked and files inside its own folder",
    "file_write": "write files inside its own data folder",
    "network": "access the network",
    "gpu": "query the GPU / compute backend",
}


ASK_TIMEOUT = 120.0          # seconds an unanswered dialog is waited for (A1.3, audit M9)

# Time spent inside permission dialogs per asking thread (A1.3, audit M9). ``guarded_call``
# subtracts it from the call timeout.
_dialog_acc: dict = {}
_dialog_open: dict = {}
_dialog_lock = threading.Lock()


def clock() -> float:
    return time.monotonic()


def dialog_time(ident) -> float:
    """Seconds thread ``ident`` has spent (so far) waiting in permission dialogs."""
    with _dialog_lock:
        total = _dialog_acc.get(ident, 0.0)
        if ident in _dialog_open:
            total += clock() - _dialog_open[ident]
        return total


def forget_dialog_time(ident) -> None:
    with _dialog_lock:
        _dialog_acc.pop(ident, None)
        _dialog_open.pop(ident, None)


def permission_text(plugin: str, perm: str) -> str:
    return f"Plugin '{plugin}' wants to {_MEANING.get(perm, perm)} ({perm}).\n\n{DIALOG_NOTE}"


def _key(plugin: str, perm: str) -> str:
    return f"plugins/perm/{plugin}/{perm}"


class _GuiAsker(QObject):
    """Runs the dialog on the GUI thread even when asked from a worker thread."""

    request = Signal(object)

    def __init__(self):
        super().__init__()
        self.request.connect(self._run)
        self._lock = threading.Lock()
        self._pending: list = []
        self._closing = False

    def shutdown(self) -> None:
        """Release every worker waiting for an answer (A1.1, audit M3). The answer is
        ``None`` = denied for this call only (not remembered)."""
        with self._lock:
            self._closing = True
            pending, self._pending = self._pending, []
        for box in pending:
            box["answer"] = None
            box["event"].set()

    @Slot(object)
    def _run(self, box):
        if self._closing:
            box["answer"] = None
            box["event"].set()
            return
        try:
            yes = QMessageBox.StandardButton.Yes
            r = QMessageBox.question(None, "Plugin permission", box["text"],
                                     yes | QMessageBox.StandardButton.No,
                                     QMessageBox.StandardButton.No)
            box["answer"] = (r == yes)
        except Exception:  # noqa: BLE001 - never leave the asker waiting
            box["answer"] = False
        box["event"].set()

    def ask(self, text: str):
        """True / False, or ``None`` when the application is closing (do not remember)."""
        box = {"text": text, "event": threading.Event()}
        app = QCoreApplication.instance()
        if app is None or QThread.currentThread() == app.thread():
            self._run(box)
            return box["answer"]
        with self._lock:
            if self._closing:
                return None
            self._pending.append(box)
        self.request.emit(box)
        if not box["event"].wait(ASK_TIMEOUT):
            with self._lock:
                if box in self._pending:
                    self._pending.remove(box)
            box["answer"] = None          # unanswered: denied for this call only, not remembered
            return None
        return box.get("answer")


class PermissionManager:
    """Decides every permission request: declared? remembered? else ask."""

    def __init__(self, settings, ask: Callable[[str, str, str], bool] | None = None):
        self._settings = settings
        self._ask = ask
        self._asker = _GuiAsker() if ask is None else None   # lives in the creating (GUI) thread

    def shutdown(self) -> None:
        """Release workers waiting in the dialog (answer: denied, not saved)."""
        if self._asker is not None:
            self._asker.shutdown()

    def _ask_user(self, plugin: str, perm: str):
        text = permission_text(plugin, perm)
        ident = threading.get_ident()
        with _dialog_lock:
            _dialog_open[ident] = clock()
        try:
            if self._ask is not None:
                return bool(self._ask(plugin, perm, text))
            return self._asker.ask(text)
        finally:
            with _dialog_lock:
                t0 = _dialog_open.pop(ident, None)
                if t0 is not None:
                    _dialog_acc[ident] = _dialog_acc.get(ident, 0.0) + (clock() - t0)

    def state(self, plugin: str, perm: str) -> str:
        """``granted`` | ``denied`` | ``ask`` (nothing remembered)."""
        v = self._settings.get(_key(plugin, perm), "")
        return v if v in ("granted", "denied") else "ask"

    def check(self, plugin: str, declared, perm: str) -> None:
        """Raise ``PermissionError`` unless ``perm`` is declared and (remembered or just) granted."""
        if perm not in PERMISSIONS:
            raise PermissionError(f"unknown permission {perm!r}")
        if perm not in declared:
            raise PermissionError(f"plugin '{plugin}' did not declare permission '{perm}'")
        st = self.state(plugin, perm)
        if st == "granted":
            return
        if st == "denied":
            raise PermissionError(f"permission '{perm}' was denied for plugin '{plugin}'")
        answer = self._ask_user(plugin, perm)
        if answer is None:        # closing: denied for this call only
            raise PermissionError(f"permission '{perm}' was not granted (no answer or application closing)")
        if answer:
            self._settings.set(_key(plugin, perm), "granted")
        else:
            self._settings.set(_key(plugin, perm), "denied")
            raise PermissionError(f"permission '{perm}' was denied for plugin '{plugin}'")

    def revoke(self, plugin: str, perm: str | None = None) -> None:
        """Forget the remembered decision(s); the next use asks again."""
        for p in ([perm] if perm else PERMISSIONS):
            self._settings.set(_key(plugin, p), "")

    def decisions(self, plugin: str, declared) -> dict:
        return {p: self.state(plugin, p) for p in declared}
