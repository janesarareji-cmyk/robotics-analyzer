"""Host object handed to a plugin (R8.2) and the guarded call used for every plugin call.

The host exposes only the capabilities the manifest declared. It is a cooperative guard, not a
sandbox (see docs/plugin-guide.md).
"""
from __future__ import annotations

import os
import threading
import time
import traceback
from typing import Callable

from PySide6.QtCore import QCoreApplication, QThread

from . import permissions as _perm
from .permissions import PermissionManager


class PluginCallError(RuntimeError):
    """A plugin call failed. ``reason``: raised | timeout | garbage | permission."""

    def __init__(self, reason: str, message: str, plugin: str = ""):
        super().__init__(message)
        self.reason = reason
        self.plugin = plugin


def _on_gui_thread() -> bool:
    app = QCoreApplication.instance()
    return app is not None and QThread.currentThread() == app.thread()


def guarded_call(fn: Callable, args=(), kwargs=None, timeout: float = 30.0, cancel=None,
                 grace: float = 2.0):
    """Run ``fn`` in a daemon thread, wait at most ``timeout`` seconds.

    Returns the value. Raises ``PluginCallError`` (raised / timeout / permission). A hung thread
    cannot be killed: on timeout it is abandoned. On the GUI thread the event loop keeps running
    while waiting. ``cancel``: ``threading.Event``; once set the call gets ``grace`` seconds.
    """
    box: dict = {}

    def run():
        try:
            box["v"] = fn(*args, **(kwargs or {}))
        except BaseException as exc:  # noqa: BLE001 - foreign code, even SystemExit
            box["e"] = exc
            box["tb"] = traceback.format_exc()

    t = threading.Thread(target=run, daemon=True, name="plugin-call")
    t.start()
    t0 = _perm.clock()
    cancel_at = None
    gui = _on_gui_thread()
    while t.is_alive():
        t.join(0.02)
        if gui:
            QCoreApplication.processEvents()
        now = _perm.clock()
        if cancel is not None and cancel.is_set() and cancel_at is None:
            cancel_at = now
        if cancel_at is not None and now - cancel_at > grace:
            raise PluginCallError("timeout", f"did not stop {grace:g} s after cancel")
        if now - t0 - _perm.dialog_time(t.ident) > timeout:
            _perm.forget_dialog_time(t.ident)
            raise PluginCallError("timeout", f"no answer after {timeout:g} s (thread abandoned)")
    _perm.forget_dialog_time(t.ident)
    if "e" in box:
        exc = box["e"]
        if isinstance(exc, PermissionError):
            raise PluginCallError("permission", str(exc)) from exc
        last = box["tb"].strip().splitlines()[-1]
        raise PluginCallError("raised", last)
    return box.get("v")


class PluginHost:
    """What ``register(host)`` and the plugin functions receive."""

    def __init__(self, name: str, kind: str, declared, plugin_dir: str, perms: PermissionManager,
                 log: Callable[[str], None] | None = None, version: str = ""):
        self.name = name
        self.kind = kind
        self.permissions = tuple(declared)
        self.version = version
        self._dir = os.path.realpath(plugin_dir)
        self._perms = perms
        self._log = log or (lambda m: None)
        self._allowed_read: list[str] = []
        self._registrations: list = []
        if "network" in self.permissions:
            self.http_get = self._http_get          # absent (AttributeError) when not declared
        if "gpu" in self.permissions:
            self.gpu_info = self._gpu_info

    # ---------------------------------------------------------------- plugin facing
    def log(self, message: str) -> None:
        self._log(f"[plugin {self.name}] {message}")

    def register_importer(self, label: str, file_filter: str, import_file: Callable) -> None:
        """Importer plugin: ``file_filter`` like ``"My format (*.xyz)"``; ``import_file(path)``."""
        self._register("importer", label=str(label), file_filter=str(file_filter), fn=import_file)

    def register_planner(self, name: str, plan_fn: Callable) -> None:
        """Planner plugin: ``plan_fn`` has the ``app.planning.planner.plan`` signature."""
        self._register("planner", label=str(name), fn=plan_fn)

    def register_analysis_panel(self, title: str, factory: Callable) -> None:
        """Analysis plugin: ``factory(project_view) -> QWidget``."""
        self._register("analysis", label=str(title), fn=factory)

    def _register(self, kind: str, **kw) -> None:
        if kind != self.kind:
            raise TypeError(f"plugin '{self.name}' is of kind '{self.kind}', cannot register a {kind}")
        if not callable(kw.get("fn")):
            raise TypeError("registered function must be callable")
        if not kw["label"].strip():
            raise ValueError("label must not be empty")
        self._registrations.append(dict(kind=kind, **kw))

    # ---- files (scope: own folder + files the user picked; write: own data folder)
    def _in(self, path: str, roots) -> bool:
        p = os.path.realpath(path)
        for r in roots:
            try:
                if os.path.commonpath([p, r]) == r:
                    return True
            except ValueError:
                continue
        return False

    @property
    def data_dir(self) -> str:
        """Folder a plugin may write into (needs ``file_write``)."""
        return os.path.join(self._dir, "data")

    def _read_roots(self):
        return [self._dir] + list(self._allowed_read)

    def read_bytes(self, path) -> bytes:
        if not self._in(str(path), self._read_roots()):
            raise PermissionError(f"read outside the allowed scope: {path}")
        self._perms.check(self.name, self.permissions, "file_read")
        with open(path, "rb") as f:
            return f.read()

    def read_text(self, path, encoding: str = "utf-8") -> str:
        return self.read_bytes(path).decode(encoding)

    def write_bytes(self, path, data: bytes) -> None:
        root = os.path.realpath(self.data_dir)
        if not self._in(str(path), [root]):
            raise PermissionError(f"write outside the allowed scope ({root}): {path}")
        self._perms.check(self.name, self.permissions, "file_write")
        os.makedirs(os.path.dirname(os.path.realpath(path)), exist_ok=True)
        with open(path, "wb") as f:
            f.write(data)

    def write_text(self, path, text: str, encoding: str = "utf-8") -> None:
        self.write_bytes(path, text.encode(encoding))

    # ---- network / gpu (only bound when declared)
    def _http_get(self, url: str, timeout: float = 10.0) -> bytes:
        self._perms.check(self.name, self.permissions, "network")
        import urllib.request
        with urllib.request.urlopen(url, timeout=timeout) as r:  # noqa: S310
            return r.read()

    def _gpu_info(self) -> dict:
        self._perms.check(self.name, self.permissions, "gpu")
        from app.compute.backend import get_backend
        b = get_backend("auto")
        try:
            return dict(b.device_info())
        except Exception:  # noqa: BLE001
            return {"available": False, "name": getattr(b, "name", "CPU")}

    # ---------------------------------------------------------------- app facing
    def _allow_read(self, path) -> None:
        """App only: the user picked this file; allow it and its folder for reading."""
        p = os.path.realpath(str(path))
        self._allowed_read.extend([p, os.path.dirname(p)])

    def _clear_allowed(self) -> None:
        self._allowed_read.clear()


class ProjectView:
    """Read-only live view of the project for analysis plugins (fresh ``AppHandle`` per access)."""

    def __init__(self, provider: Callable, version: str = ""):
        object.__setattr__(self, "_provider", provider)
        object.__setattr__(self, "_version", version)

    def snapshot(self):
        from app.script_runner import AppHandle
        return AppHandle(object.__getattribute__(self, "_provider")(),
                         object.__getattribute__(self, "_version"))

    def __getattr__(self, name: str):
        if name.startswith("_"):
            raise AttributeError(name)
        return getattr(self.snapshot(), name)

    def __setattr__(self, name, value):
        raise AttributeError("project view is read-only")
