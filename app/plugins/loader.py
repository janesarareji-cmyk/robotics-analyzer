"""Plugin loader and manager (R8.2).

Scans a plugins folder, validates manifests WITHOUT running plugin code, loads entry points with
``importlib`` behind a guarded call, and keeps a result list (name, status, message). One failing
plugin never blocks the others or the app.
"""
from __future__ import annotations

import importlib
import importlib.util
import os
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

from PySide6.QtCore import QObject, Signal

from .host import PluginCallError, PluginHost, ProjectView, guarded_call
from .kinds import PluginAnalysis, PluginImporter, PluginPlanner
from .manifest import Manifest, ManifestError, load_manifest
from .permissions import PermissionManager

STATUSES = ("ok", "rejected", "error", "disabled")
MANIFEST_NAME = "plugin.json"


def default_plugins_folder() -> str:
    base = os.environ.get("APPDATA") or os.path.expanduser("~")
    return os.path.join(base, "RoboticsAnalyzer", "plugins")


@dataclass
class LoadResult:
    name: str
    status: str            # ok | rejected | error | disabled
    message: str = ""
    kind: str = ""
    version: str = ""
    permissions: tuple = ()
    path: str = ""
    description: str = ""


class LoadedPlugin:
    """A plugin with a manifest, a host and its registrations."""

    def __init__(self, manifest: Manifest, path: str, host: PluginHost, manager: "PluginManager"):
        self.manifest, self.path, self.host, self._manager = manifest, path, host, manager
        self.module = None
        self.importers: list[PluginImporter] = []
        self.planners: list[PluginPlanner] = []
        self.analysis: list[PluginAnalysis] = []
        self.session_error: str | None = None

    @property
    def name(self) -> str:
        return self.manifest.name

    @property
    def active(self) -> bool:
        return self.session_error is None

    def fail(self, reason: str, message: str) -> None:
        """Disable for this session (raised / timeout / garbage)."""
        if self.session_error is None:
            self.session_error = f"{reason}: {message}"
            self._manager._plugin_failed(self, reason, message)

    def call(self, fn: Callable, args=(), kwargs=None, timeout: float | None = None, cancel=None,
             validate: Callable | None = None):
        """Guarded plugin call: exceptions, timeout and garbage results are reported and the
        plugin is disabled for the session (a PermissionError only fails the call)."""
        if not self.active:
            raise PluginCallError("disabled", f"plugin '{self.name}' is disabled for this session",
                                  self.name)
        timeout = self._manager.call_timeout if timeout is None else timeout
        try:
            value = guarded_call(fn, args, kwargs, timeout=timeout, cancel=cancel)
            if validate is not None:
                try:
                    value = validate(value)
                except PluginCallError:
                    raise
                except Exception as exc:  # noqa: BLE001 - a weird object can break validation
                    raise PluginCallError("garbage", f"unusable result ({exc})") from exc
            return value
        except PluginCallError as exc:
            exc.plugin = self.name
            if exc.reason == "permission":
                self._manager.log_message.emit(f"[WARNING] plugin {self.name}: {exc}")
            elif exc.reason != "disabled":
                self.fail(exc.reason, str(exc))
            raise


class PluginManager(QObject):
    """Owns the plugin list. ``changed`` fires after a scan, enable/disable or a session failure."""

    changed = Signal()
    log_message = Signal(str)

    def __init__(self, settings, folder: str | None = None,
                 ask: Callable[[str, str, str], bool] | None = None,
                 project_provider: Callable | None = None, app_version: str = "",
                 load_timeout: float = 10.0, call_timeout: float = 60.0, parent=None):
        super().__init__(parent)
        self._settings = settings
        self._folder = folder
        self.permissions = PermissionManager(settings, ask)
        self._project_provider = project_provider or (lambda: None)
        self.app_version = app_version
        self.load_timeout = load_timeout
        self.call_timeout = call_timeout
        self.results: list[LoadResult] = []
        self._plugins: dict[str, LoadedPlugin] = {}
        self._dirs: dict[str, str] = {}

    # ------------------------------------------------------------------ folder / settings
    @property
    def folder(self) -> str:
        if self._folder:
            return self._folder
        f = getattr(self._settings, "plugins_folder", None)
        return f() if callable(f) else default_plugins_folder()

    def is_enabled(self, name: str) -> bool:
        return str(self._settings.get(f"plugins/enabled/{name}", "1")) != "0"

    def project_view(self) -> ProjectView:
        return ProjectView(self._project_provider, self.app_version)

    # ------------------------------------------------------------------ scan / load
    def scan(self) -> list[LoadResult]:
        """(Re)load every plugin of the folder."""
        for p in list(self._plugins.values()):
            self._unload(p)
        self.results, self._plugins, self._dirs = [], {}, {}
        root = Path(self.folder)
        dirs = []
        try:
            if root.is_dir():
                dirs = sorted(d for d in root.iterdir() if d.is_dir() and (d / MANIFEST_NAME).is_file())
        except OSError as exc:
            self.log_message.emit(f"[ERROR] plugins folder unreadable: {exc}")
        for d in dirs:
            self.results.append(self._process_dir(d))
        for r in self.results:
            self.log_message.emit(f"[{'INFO' if r.status in ('ok', 'disabled') else 'WARNING'}] "
                                  f"plugin {r.name}: {r.status}" + (f" - {r.message}" if r.message else ""))
        self.changed.emit()
        return list(self.results)

    def _process_dir(self, d: Path) -> LoadResult:
        try:
            man = load_manifest(d / MANIFEST_NAME)
        except ManifestError as exc:
            return LoadResult(d.name, "rejected", str(exc), path=str(d))
        base = dict(name=man.name, kind=man.kind, version=man.version, permissions=man.permissions,
                    path=str(d), description=man.description)
        if man.name in self._dirs:
            return LoadResult(**{**base, "status": "rejected",
                                 "message": f"duplicate plugin name (also in {self._dirs[man.name]})"})
        self._dirs[man.name] = str(d)
        if not self.is_enabled(man.name):
            return LoadResult(**{**base, "status": "disabled", "message": "disabled in settings"})
        host = PluginHost(man.name, man.kind, man.permissions, str(d), self.permissions,
                          log=self.log_message.emit, version=man.version)
        plugin = LoadedPlugin(man, str(d), host, self)
        try:
            guarded_call(self._import_and_register, (plugin,), timeout=self.load_timeout)
            n = len(host._registrations)
            if n == 0:
                raise PluginCallError("raised", f"register() did not register any {man.kind}")
            for reg in host._registrations:
                if reg["kind"] == "importer":
                    plugin.importers.append(PluginImporter(plugin, reg["label"], reg["file_filter"],
                                                           reg["fn"]))
                elif reg["kind"] == "planner":
                    plugin.planners.append(PluginPlanner(plugin, reg["label"], reg["fn"]))
                else:
                    plugin.analysis.append(PluginAnalysis(plugin, reg["label"], reg["fn"]))
        except PluginCallError as exc:
            self._drop_modules(man.name)
            msg = "load timeout: " + str(exc) if exc.reason == "timeout" else str(exc)
            return LoadResult(**{**base, "status": "error", "message": msg})
        self._plugins[man.name] = plugin
        return LoadResult(**{**base, "status": "ok", "message": ""})

    @staticmethod
    def _uniq(name: str) -> str:
        return "ra_plugin_" + re.sub(r"\W", "_", name)

    def _import_and_register(self, plugin: LoadedPlugin) -> None:
        man, root = plugin.manifest, os.path.realpath(plugin.path)
        parts = man.module.split(".")
        base = os.path.realpath(os.path.join(root, *parts))
        if os.path.commonpath([base, root]) != root:
            raise ImportError("entry point module is outside the plugin folder")
        top = os.path.realpath(os.path.join(root, parts[0]))
        uniq = self._uniq(man.name)
        if os.path.isfile(top + ".py"):
            spec = importlib.util.spec_from_file_location(uniq, top + ".py")
        elif os.path.isfile(os.path.join(top, "__init__.py")):
            spec = importlib.util.spec_from_file_location(
                uniq, os.path.join(top, "__init__.py"), submodule_search_locations=[top])
        else:
            raise ImportError(f"entry point module '{man.module}' not found in plugin folder")
        mod = importlib.util.module_from_spec(spec)
        sys.modules[uniq] = mod
        spec.loader.exec_module(mod)
        if len(parts) > 1:
            mod = importlib.import_module(uniq + "." + ".".join(parts[1:]))
        fn = getattr(mod, man.function, None)
        if not callable(fn):
            raise ImportError(f"entry point function '{man.function}' not found in '{man.module}'")
        plugin.module = mod
        fn(plugin.host)

    def _drop_modules(self, name: str) -> None:
        uniq = self._uniq(name)
        for k in [k for k in sys.modules if k == uniq or k.startswith(uniq + ".")]:
            sys.modules.pop(k, None)

    def _unload(self, plugin: LoadedPlugin) -> None:
        plugin.importers, plugin.planners, plugin.analysis = [], [], []
        self._drop_modules(plugin.name)

    # ------------------------------------------------------------------ enable / disable
    def set_enabled(self, name: str, enabled: bool) -> bool:
        """Persist the choice and (un)load the plugin. False if the plugin is unknown."""
        d = self._dirs.get(name)
        idx = next((i for i, r in enumerate(self.results) if r.name == name), -1)
        if d is None or idx < 0:
            return False
        self._settings.set(f"plugins/enabled/{name}", "1" if enabled else "0")
        old = self._plugins.pop(name, None)
        if old is not None:
            self._unload(old)
        self._dirs.pop(name, None)
        self.results[idx] = self._process_dir(Path(d))
        self.changed.emit()
        return True

    def result(self, name: str) -> LoadResult | None:
        return next((r for r in self.results if r.name == name), None)

    def plugin(self, name: str) -> LoadedPlugin | None:
        return self._plugins.get(name)

    def _plugin_failed(self, plugin: LoadedPlugin, reason: str, message: str) -> None:
        r = self.result(plugin.name)
        text = f"disabled for this session ({reason}): {message}"
        if r is not None:
            r.status, r.message = "disabled", text
        self.log_message.emit(f"[ERROR] plugin {plugin.name} {text}")
        self.changed.emit()

    # ------------------------------------------------------------------ registries
    def _active(self):
        return [p for p in self._plugins.values() if p.active]

    def importers(self) -> list[PluginImporter]:
        return [i for p in self._active() for i in p.importers]

    def planners(self) -> list[PluginPlanner]:
        return [i for p in self._active() for i in p.planners]

    def analysis_panels(self) -> list[PluginAnalysis]:
        return [i for p in self._active() for i in p.analysis]
