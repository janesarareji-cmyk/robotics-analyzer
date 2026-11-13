"""U6.1: analysis overlays in the 3D view (reach map, singularity zones, joint load).

``Overlay`` is the base class: ``compute(model, opts, cancel, progress)`` makes a data dict in a worker
(no Qt, no 3D), ``show(view, data, opts, base, opacity)`` draws it through ``RobotView`` on the GUI
thread, ``hide(view)`` removes it, ``legend(opts)`` gives the colour bar stops.
``OverlayManager`` owns the toggle, opacity, options and the cache of every overlay. Data is cached
per robot structure + tool (hash of ``cb.compile_arrays``) + an optional extra hash (collision
matrix), so toggling an overlay off and on again does not recompute. Compute runs in the
``TaskRunner`` with progress and cancel; a robot change or a newer job drops the stale result.
Live overlays (``live = True``, the joint load bars) have no compute: they are fed by ``update_live``.
"""
from __future__ import annotations

import hashlib
import itertools
import traceback
from typing import Any, Callable

import numpy as np
from PySide6.QtCore import QObject, Qt, Signal

import core_bridge as cb

# colour bar stops: list of (position 0..1, (r, g, b) 0..255)
Legend = list


def robot_hash(model: Any, extra: str = "") -> str:
    """Hash of the robot structure and tool: changes when the kinematics (not q) change."""
    h = hashlib.sha1()
    try:
        arrays = cb.compile_arrays(model)
        for k in sorted(arrays):
            v = np.ascontiguousarray(np.asarray(arrays[k]))
            h.update(k.encode())
            h.update(str(v.shape).encode())
            h.update(v.tobytes())
    except Exception:
        h.update(repr(getattr(model, "name", "")).encode())
    h.update(extra.encode())
    return h.hexdigest()


class Overlay:
    """Base class. Subclasses set ``key``, ``title``, ``blurb`` and override the hooks."""

    key = "overlay"
    title = "Overlay"
    blurb = ""
    live = False                     # True: no compute step, fed by OverlayManager.update_live
    default_opacity = 0.35
    recompute_options: tuple[str, ...] = ()   # option names that need a new compute

    def default_options(self) -> dict:
        return {}

    def compute(self, model: Any, opts: dict, cancel: Any, progress: Callable[[int, int], Any]) -> dict:
        """Worker thread. Return the data dict (must not touch Qt or the 3D view)."""
        return {}

    def show(self, view: Any, data: dict | None, opts: dict, base: np.ndarray, opacity: float) -> int:
        """GUI thread. Draw ``data`` (None = nothing yet). Returns the number of drawn items."""
        return 0

    def hide(self, view: Any) -> None:
        view.clear_overlay(self.key)

    def legend(self, opts: dict) -> Legend:
        return []

    def legend_labels(self, opts: dict) -> tuple[str, str]:
        """Text under the left and the right end of the colour bar."""
        return ("", "")

    def summary(self, data: dict | None, opts: dict) -> str:
        return ""

    def make_controls(self, manager: "OverlayManager") -> Any:
        """Optional extra widget for the Overlays panel (sliders, check boxes). None = none."""
        return None

    def update_live(self, model: Any, view: Any, q: Any, qd: Any, qdd: Any, base: np.ndarray,
                    force: bool = False) -> None:
        """Live overlays only: new robot state (``force``: draw now, ignore the refresh limit)."""

    def reset(self) -> None:
        """The robot changed: drop what was built for the old one."""

    @property
    def has_pending(self) -> bool:
        """Live overlays: a newer state is waiting for ``flush``."""
        return False

    def flush(self) -> None:
        """Live overlays: draw the state the refresh limit held back."""


class OverlayManager(QObject):
    """Toggles, opacity, options, cache and compute jobs of all overlays."""

    changed = Signal(str)                 # overlay key: drawing or data changed
    status_changed = Signal(str, str)     # key, text
    progress_changed = Signal(str, int)   # key, percent (0..100, -1 = idle)
    computing_changed = Signal(str, bool)
    enabled_changed = Signal(str, bool)
    _progress = Signal(str, int, int)     # from the worker thread
    _done = Signal(object)                # (job, data, error)

    def __init__(self, task_runner: Any, get_view: Callable[[], Any], parent: QObject | None = None):
        super().__init__(parent)
        self._runner = task_runner
        self._get_view = get_view
        self._overlays: dict[str, Overlay] = {}
        self._enabled: dict[str, bool] = {}
        self._opacity: dict[str, float] = {}
        self._opts: dict[str, dict] = {}
        self._cache: dict[str, tuple[str, dict]] = {}     # key -> (robot hash, data)
        self._jobs: dict[str, dict] = {}                  # key -> running job
        self._ids = itertools.count(1)
        self._model: Any = None
        self._extra_fn: Callable[[], str] | None = None
        self._base = np.eye(4)
        self._generation = 0
        self._status: dict[str, str] = {}
        self._live_source: Callable[[], tuple] | None = None
        from PySide6.QtCore import QTimer
        self._flush_timer = QTimer(self)
        self._flush_timer.setSingleShot(True)
        self._flush_timer.setInterval(40)
        self._flush_timer.timeout.connect(self._flush_live)
        self.compute_count: dict[str, int] = {}
        self._progress.connect(self._on_progress, Qt.ConnectionType.QueuedConnection)
        self._done.connect(self._on_done, Qt.ConnectionType.QueuedConnection)

    # ------------------------------------------------------------------ registry
    def register(self, overlay: Overlay) -> None:
        k = overlay.key
        self._overlays[k] = overlay
        self._enabled[k] = False
        self._opacity[k] = float(overlay.default_opacity)
        self._opts[k] = overlay.default_options()
        self.compute_count[k] = 0

    @property
    def keys(self) -> list[str]:
        return list(self._overlays)

    def overlay(self, key: str) -> Overlay:
        return self._overlays[key]

    def enabled(self, key: str) -> bool:
        return bool(self._enabled.get(key, False))

    def opacity(self, key: str) -> float:
        return float(self._opacity[key])

    def options(self, key: str) -> dict:
        return dict(self._opts[key])

    def data(self, key: str) -> dict | None:
        """Cached data valid for the current robot, else None."""
        ent = self._cache.get(key)
        if ent is None or ent[0] != self._current_hash():
            return None
        return ent[1]

    def status(self, key: str) -> str:
        return self._status.get(key, "")

    def is_computing(self, key: str) -> bool:
        return key in self._jobs

    # ------------------------------------------------------------------ model
    def set_extra_hash(self, fn: Callable[[], str] | None) -> None:
        """``fn() -> str`` folded into the cache key (e.g. the collision matrix)."""
        self._extra_fn = fn

    def _current_hash(self) -> str:
        if self._model is None:
            return ""
        return robot_hash(self._model, self._extra_fn() if self._extra_fn else "")

    def set_model(self, model: Any) -> None:
        """Robot changed or closed: cancel jobs, drop every drawing; cache stays keyed by the hash."""
        self._generation += 1
        for k in list(self._jobs):
            self.cancel(k)
        self._model = model
        view = self._get_view()
        for ov in self._overlays.values():
            ov.reset()
        if view is not None:
            for ov in self._overlays.values():
                ov.hide(view)
        if model is None:
            self._cache.clear()
            return
        try:
            self._base = np.asarray(cb.compile_arrays(model)["base"], dtype=float).reshape(4, 4)
        except Exception:
            self._base = np.eye(4)
        for k in self._overlays:
            if self._enabled[k]:
                self._refresh(k)

    @property
    def base(self) -> np.ndarray:
        return self._base

    # ------------------------------------------------------------------ user actions
    def set_enabled(self, key: str, on: bool) -> None:
        on = bool(on)
        if self._enabled[key] == on:
            return
        self._enabled[key] = on
        if on:
            self._refresh(key)
        else:
            self.cancel(key)
            view = self._get_view()
            if view is not None:
                self._overlays[key].hide(view)
        self.enabled_changed.emit(key, on)
        self.changed.emit(key)

    def set_opacity(self, key: str, value: float) -> None:
        self._opacity[key] = min(max(float(value), 0.05), 1.0)
        view = self._get_view()
        if view is not None and self._enabled[key]:
            ov = self._overlays[key]
            if hasattr(view, "set_overlay_opacity") and view.overlay_actor(key) is not None:
                view.set_overlay_opacity(key, self._opacity[key])
        self.changed.emit(key)

    def set_option(self, key: str, name: str, value: Any) -> None:
        """Set an option. Options named in ``recompute_options`` start a new compute, others only redraw."""
        ov = self._overlays[key]
        if self._opts[key].get(name) == value:
            return
        self._opts[key][name] = value
        if name in ov.recompute_options:
            self._cache.pop(key, None)
        if self._enabled[key]:
            self._refresh(key)
        self.changed.emit(key)

    def recompute(self, key: str) -> None:
        """Drop the cache and compute again (only if enabled; else it computes at the next toggle)."""
        self._cache.pop(key, None)
        if self._enabled[key]:
            self.cancel(key)
            self._refresh(key)

    # ------------------------------------------------------------------ draw / compute
    def _refresh(self, key: str) -> None:
        ov = self._overlays[key]
        view = self._get_view()
        if self._model is None or view is None:
            return
        if ov.live:
            self.update_live(force=True)
            self.changed.emit(key)
            return
        data = self.data(key)
        if data is not None:
            self._status[key] = ov.summary(data, self._opts[key])
            ov.show(view, data, self._opts[key], self._base, self._opacity[key])
            self.status_changed.emit(key, self._status[key])
            self.changed.emit(key)
            return
        if key not in self._jobs:
            self._start(key)

    def redraw(self, key: str) -> None:
        """Draw the cached data again (an option changed that only affects the drawing)."""
        if self._enabled.get(key):
            self._refresh(key)

    def _start(self, key: str) -> None:
        ov = self._overlays[key]
        model, opts = self._model, dict(self._opts[key])
        chash = self._current_hash()
        job = dict(id=next(self._ids), key=key, gen=self._generation, hash=chash,
                   token=cb.CancelToken())
        self._jobs[key] = job
        self._status[key] = "Computing..."
        self.status_changed.emit(key, "Computing...")
        self.computing_changed.emit(key, True)
        self.progress_changed.emit(key, 0)

        def progress(done, total):
            self._progress.emit(key, int(done), int(total))

        def work():
            try:
                return job, ov.compute(model, opts, job["token"], progress), ""
            except Exception:
                return job, None, traceback.format_exc()

        from app.task_runner import FunctionTask
        task = FunctionTask(work)
        task.signals.result.connect(self._done, Qt.ConnectionType.QueuedConnection)
        job["task"] = task
        self._runner.submit(task)

    def cancel(self, key: str) -> None:
        job = self._jobs.pop(key, None)
        if job is not None:
            job["token"].cancel()
            self.computing_changed.emit(key, False)
            self.progress_changed.emit(key, -1)

    def _on_progress(self, key: str, done: int, total: int) -> None:
        if key in self._jobs and total > 0:
            self.progress_changed.emit(key, int(100.0 * done / total))

    def _on_done(self, payload: Any) -> None:
        job, data, err = payload
        key = job["key"]
        if self._jobs.get(key) is not job:        # cancelled, replaced or robot changed
            return
        del self._jobs[key]
        self.computing_changed.emit(key, False)
        self.progress_changed.emit(key, -1)
        ov = self._overlays[key]
        if err:
            last = err.strip().splitlines()[-1] if err.strip() else "error"
            self._status[key] = f"Failed: {last}"
            self.status_changed.emit(key, self._status[key])
            return
        if data is None or data.get("cancelled"):
            self._status[key] = "Cancelled."
            self.status_changed.emit(key, self._status[key])
            return
        self.compute_count[key] += 1
        self._cache[key] = (job["hash"], data)
        if self._enabled[key] and job["gen"] == self._generation:
            self._refresh(key)
        self.changed.emit(key)

    # ------------------------------------------------------------------ live overlays
    def set_live_source(self, fn: Callable[[], tuple] | None) -> None:
        """``fn() -> (q, qd, qdd)``: the state a live overlay draws when it is switched on."""
        self._live_source = fn

    def update_live(self, q: Any = None, qd: Any = None, qdd: Any = None, force: bool = False) -> None:
        """Feed the live overlays (joint load) with the current state (at most 30 Hz, latest wins)."""
        if self._model is None:
            return
        if q is None:
            if self._live_source is None:
                return
            q, qd, qdd = self._live_source()
        pending = False
        for key, ov in self._overlays.items():
            if ov.live and self._enabled[key]:
                ov.update_live(self._model, self._get_view(), q, qd, qdd, self._base, force)
                pending = pending or ov.has_pending
        if pending:
            self._flush_timer.start()

    def _flush_live(self) -> None:
        pending = False
        for key, ov in self._overlays.items():
            if ov.live and self._enabled[key]:
                ov.flush()
                pending = pending or ov.has_pending
        if pending:
            self._flush_timer.start()

    # ------------------------------------------------------------------ probe
    def probe(self, point_world: Any) -> dict:
        """Tell the overlays that have a probe (reach map) where the target is. key -> result."""
        out: dict = {}
        view = self._get_view()
        if view is None or self._model is None:
            return out
        for key, ov in self._overlays.items():
            if self._enabled[key] and hasattr(ov, "probe"):
                out[key] = ov.probe(view, point_world, self._base)
        return out

    # ------------------------------------------------------------------ misc
    def dispose(self) -> None:
        for k in list(self._jobs):
            self.cancel(k)
        view = self._get_view()
        if view is not None:
            for ov in self._overlays.values():
                try:
                    ov.hide(view)
                except Exception:
                    pass


def default_overlays() -> list[Overlay]:
    """The overlays shipped with the app (modules imported here so a missing one does not break the others)."""
    out: list[Overlay] = []
    for mod, cls in (("app.analysis.reach_overlay", "ReachOverlay"),
                     ("app.analysis.singularity_overlay", "SingularityOverlay"),
                     ("app.analysis.load_overlay", "LoadOverlay")):
        try:
            m = __import__(mod, fromlist=[cls])
            out.append(getattr(m, cls)())
        except ImportError:
            continue
    return out
