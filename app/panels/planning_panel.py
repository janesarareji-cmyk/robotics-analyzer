"""
app/panels/planning_panel.py
R7.4: "Planning" panel. Start/goal joint vectors (typed, from the current q, or from a named
configuration), planner choice, time limit, Plan (worker via TaskRunner) / Cancel, result
summary and "Send to trajectory" (emits `trajectory_ready` with a timed R6.4 `Trajectory`).

Named configurations (R7.6) hook: call `set_named_configs({name: q})`; the "Named" combos
then fill the Start / Goal column. Collision: `set_collision_source(fn)` where `fn()` returns a
``(shapes, config)`` pair or ``None`` (joint limits only); `set_project(project)` sets the default
source (`project.collisions`/`visuals` shapes + `project.collision or CollisionConfig()`).
The world is built inside the worker.
"""

from __future__ import annotations

import math
import threading
from typing import Any, Callable

import numpy as np
from PySide6.QtCore import Signal
from PySide6.QtWidgets import (
    QCheckBox, QComboBox, QDoubleSpinBox, QFormLayout, QHBoxLayout, QLabel, QPushButton,
    QSpinBox, QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget,
)

from app.kin_utils import current_q, dof_limits
from app.planning.planner import DEFAULT_RESOLUTION, PLANNERS, PlanResult, plan
from app.planning.shortcut import TimedPath, time_parametrize
from app.task_runner import FunctionTask

_TYPED = "(typed)"
_PLANNER_LABEL = {"rrt_connect": "RRT-Connect", "rrt_star": "RRT*", "prm": "PRM"}


def _fmt(x: float) -> str:
    return f"{x:.6g}"


class PlanningPanel(QWidget):
    """Dock content for path planning."""

    log_requested = Signal(str)
    trajectory_ready = Signal(object)     # Trajectory (Send to trajectory)
    plan_finished = Signal(object)        # PlanResult (also on failure / cancel)
    _progress = Signal(int)
    _done = Signal(int, object)           # (generation, (PlanResult, TimedPath)), from the worker
    _failed = Signal(int, str)

    def __init__(self, task_runner: Any, parent=None):
        super().__init__(parent)
        self._runner = task_runner
        self._model: Any = None
        self._project: Any = None
        self._collision_source: Callable[[], Any] | None = None
        self._q_provider: Callable[[], list[float]] | None = None
        self._named: dict[str, list[float]] = {}
        self._generation = 0
        self._running = False
        self._cancel_evt: threading.Event | None = None
        self._task: Any = None
        self._result: PlanResult | None = None
        self._timed: TimedPath | None = None
        self._n_dof = 0
        self._loading = False

        layout = QVBoxLayout(self)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(4)

        self._table = QTableWidget(0, 2)
        self._table.setHorizontalHeaderLabels(["Start [rad|m]", "Goal [rad|m]"])
        self._table.setMinimumHeight(110)
        layout.addWidget(self._table)

        row = QHBoxLayout()
        self._btn_start_cur = QPushButton("Start = current q")
        self._btn_goal_cur = QPushButton("Goal = current q")
        self._btn_start_cur.clicked.connect(lambda: self.capture_current("start"))
        self._btn_goal_cur.clicked.connect(lambda: self.capture_current("goal"))
        row.addWidget(self._btn_start_cur)
        row.addWidget(self._btn_goal_cur)
        layout.addLayout(row)

        nrow = QHBoxLayout()
        self._named_start = QComboBox()
        self._named_goal = QComboBox()
        self._named_start.activated.connect(lambda _i: self._apply_named("start"))
        self._named_goal.activated.connect(lambda _i: self._apply_named("goal"))
        nrow.addWidget(QLabel("Named start"))
        nrow.addWidget(self._named_start, 1)
        nrow.addWidget(QLabel("goal"))
        nrow.addWidget(self._named_goal, 1)
        layout.addLayout(nrow)

        form = QFormLayout()
        self._plugin_planners: dict = {}
        self._planner = QComboBox()
        for p in PLANNERS:
            self._planner.addItem(_PLANNER_LABEL[p], p)
        form.addRow("Planner", self._planner)
        self._time_limit = QDoubleSpinBox()
        self._time_limit.setRange(0.05, 600.0)
        self._time_limit.setDecimals(2)
        self._time_limit.setValue(5.0)
        self._time_limit.setSuffix(" s")
        form.addRow("Time limit", self._time_limit)
        self._seed = QSpinBox()
        self._seed.setRange(0, 2_000_000_000)
        self._seed.setValue(0)
        form.addRow("Seed", self._seed)
        self._resolution = QDoubleSpinBox()
        self._resolution.setRange(0.001, 0.5)
        self._resolution.setDecimals(3)
        self._resolution.setSingleStep(0.005)
        self._resolution.setValue(DEFAULT_RESOLUTION)
        form.addRow("Edge resolution", self._resolution)
        self._chk_shortcut = QCheckBox("Shortcut path")
        self._chk_shortcut.setChecked(True)
        form.addRow("", self._chk_shortcut)
        layout.addLayout(form)

        brow = QHBoxLayout()
        self._btn_plan = QPushButton("Plan")
        self._btn_plan.clicked.connect(self.plan)
        self._btn_cancel = QPushButton("Cancel")
        self._btn_cancel.clicked.connect(self.cancel)
        self._btn_send = QPushButton("Send to trajectory")
        self._btn_send.clicked.connect(self.send_to_trajectory)
        brow.addWidget(self._btn_plan)
        brow.addWidget(self._btn_cancel)
        brow.addWidget(self._btn_send)
        layout.addLayout(brow)

        self._status = QLabel("")
        self._status.setWordWrap(True)
        layout.addWidget(self._status)
        self._summary = QLabel("")
        self._summary.setWordWrap(True)
        layout.addWidget(self._summary)
        layout.addStretch(1)

        self._progress.connect(self._on_progress)
        self._done.connect(self._on_result)
        self._failed.connect(self._on_error)
        self._set_named_combos()
        self._update_buttons()

    # ------------------------------------------------------------------ facts
    @property
    def is_running(self) -> bool:
        return self._running

    @property
    def result(self) -> PlanResult | None:
        return self._result

    @property
    def timed(self) -> TimedPath | None:
        return self._timed

    @property
    def status_text(self) -> str:
        return self._status.text()

    @property
    def summary_text(self) -> str:
        return self._summary.text()

    def button_enabled(self) -> dict[str, bool]:
        return {"plan": self._btn_plan.isEnabled(), "cancel": self._btn_cancel.isEnabled(),
                "send": self._btn_send.isEnabled()}

    # ------------------------------------------------------------------ setup
    def set_q_provider(self, fn: Callable[[], list[float]] | None) -> None:
        self._q_provider = fn

    def set_collision_source(self, fn: Callable[[], Any] | None) -> None:
        """``fn() -> (shapes, CollisionConfig)`` or None (joint limits only)."""
        self._collision_source = fn

    def set_project(self, project: Any) -> None:
        """Default collision source from the project (stored shapes + config)."""
        self._project = project
        self._collision_source = self._project_source if project is not None else None

    def _project_source(self):
        from app.planning.collision import CollisionConfig, shapes_from_project
        project = self._project
        if project is None:
            return None
        cfg = getattr(project, "collision", None) or CollisionConfig()
        shapes = [s for s in shapes_from_project(project) if s.link >= 0]
        if not shapes and not cfg.environment:
            return None
        return shapes, cfg

    def set_named_configs(self, configs: dict[str, list[float]] | None) -> None:
        """R7.6 hook: ``{name: q}``. Entries with a wrong DOF count are ignored."""
        self._named = {k: list(v) for k, v in (configs or {}).items()
                       if len(v) == self._n_dof}
        self._set_named_combos()

    def _set_named_combos(self) -> None:
        for box in (self._named_start, self._named_goal):
            box.clear()
            box.addItem(_TYPED)
            box.addItems(sorted(self._named))

    def set_model(self, model: Any) -> None:
        """Robot replaced or edited (None on close): cancel, clear results, reset the table."""
        self._generation += 1
        if self._cancel_evt is not None:
            self._cancel_evt.set()
        self._running = False
        self._cancel_evt = None
        self._result, self._timed = None, None
        self._status.setText("")
        self._summary.setText("")
        usable = model is not None and len(model.joints) > 0 and model.dof() > 0
        self._model = model if usable else None
        self._n_dof = model.dof() if usable else 0
        self._named = {k: v for k, v in self._named.items() if len(v) == self._n_dof}
        self._set_named_combos()
        self._loading = True
        try:
            self._table.setRowCount(self._n_dof)
            for i in range(self._n_dof):
                self._table.setVerticalHeaderItem(i, QTableWidgetItem(f"q{i + 1}"))
                for c in (0, 1):
                    self._table.setItem(i, c, QTableWidgetItem("0"))
        finally:
            self._loading = False
        self._update_buttons()

    # ------------------------------------------------------------------ start / goal
    def _col(self, name: str) -> int:
        return 0 if name == "start" else 1

    def vector(self, name: str) -> list[float] | None:
        """Start or goal vector from the table; None when a cell is not a finite number."""
        c = self._col(name)
        out = []
        for i in range(self._n_dof):
            it = self._table.item(i, c)
            try:
                v = float(it.text()) if it is not None else math.nan
            except ValueError:
                return None
            if not math.isfinite(v):
                return None
            out.append(v)
        return out

    def set_vector(self, name: str, q) -> None:
        c = self._col(name)
        for i, v in enumerate(q):
            if i < self._n_dof:
                self._table.item(i, c).setText(_fmt(float(v)))

    def _current_q(self) -> list[float] | None:
        if self._q_provider is not None:
            return [float(x) for x in self._q_provider()]
        if self._model is not None:
            return current_q(self._model)
        return None

    def capture_current(self, name: str) -> None:
        q = self._current_q()
        if q is not None and len(q) == self._n_dof:
            self.set_vector(name, q)

    def _apply_named(self, name: str) -> None:
        box = self._named_start if name == "start" else self._named_goal
        key = box.currentText()
        if key in self._named:
            self.set_vector(name, self._named[key])

    # ------------------------------------------------------------------ run
    def _update_buttons(self) -> None:
        self._btn_plan.setEnabled(self._model is not None and not self._running)
        self._btn_cancel.setEnabled(self._running)
        ok = self._result is not None and self._result.success and self._timed is not None
        self._btn_send.setEnabled(bool(ok) and not self._running)

    def set_plugin_planners(self, planners) -> None:
        """R8.2: replace the plugin entries of the planner combo.

        ``planners``: objects with ``key``, ``display`` and ``__call__`` (plan signature).
        """
        cur = self._planner.currentData()
        for i in range(self._planner.count() - 1, -1, -1):
            if str(self._planner.itemData(i)).startswith("plugin:"):
                self._planner.removeItem(i)
        self._plugin_planners = {}
        for p in planners:
            self._planner.addItem(p.display, p.key)
            self._plugin_planners[p.key] = p
        idx = self._planner.findData(cur)
        self._planner.setCurrentIndex(idx if idx >= 0 else 0)

    def options(self) -> dict:
        return dict(planner=self._planner.currentData(), time_limit=float(self._time_limit.value()),
                    seed=int(self._seed.value()), resolution=float(self._resolution.value()),
                    do_shortcut=self._chk_shortcut.isChecked())

    def plan(self) -> None:
        """Start planning in a worker. Returns at once."""
        if self._model is None or self._running:
            return
        start, goal = self.vector("start"), self.vector("goal")
        if start is None or goal is None:
            self._status.setText("Start / goal need finite numbers.")
            return
        model = self._model
        try:
            src = self._collision_source() if self._collision_source is not None else None
        except Exception as exc:
            self._status.setText(f"Collision setup failed: {exc}")
            return
        opts = self.options()
        lo, hi = dof_limits(model)
        gen = self._generation
        evt = threading.Event()
        self._cancel_evt = evt
        self._running = True
        self._result, self._timed = None, None
        self._summary.setText("")
        self._status.setText(f"Planning with {self._planner.currentText()}...")
        self.log_requested.emit(
            f"[INFO] Planning: {opts['planner']}, limit {opts['time_limit']:.2f} s, seed {opts['seed']}.")

        def progress(it, _name):
            if gen == self._generation:
                self._progress.emit(int(it))

        plugin_planners = dict(self._plugin_planners)

        def work():
            world = None
            if src is not None:
                from app.planning.collision import CollisionWorld
                shapes, cfg = src
                world = CollisionWorld(model, shapes, cfg)
            plugin_fn = plugin_planners.get(opts["planner"])
            if plugin_fn is not None:      # R8.2: guarded + validated plugin planner
                res = plugin_fn(model, start, goal, collision=world, limits=(lo, hi), cancel=evt,
                                progress=progress, **opts)
            else:
                res = plan(model, start, goal, world, (lo, hi), cancel=evt, progress=progress, **opts)
            timed = None
            if res.success:
                timed = time_parametrize(res.path, model, collision=world)
            return res, timed

        task = FunctionTask(work)       # connect before submit: a fast task must not lose its result
        task.signals.result.connect(lambda r, g=gen: self._done.emit(g, r))
        task.signals.error.connect(lambda m, g=gen: self._failed.emit(g, m))
        self._task = task               # keep the signal object alive until the result arrives
        self._runner.submit(task)
        self._update_buttons()

    def cancel(self) -> None:
        if self._cancel_evt is not None and self._running:
            self._cancel_evt.set()
            self._status.setText("Cancelling...")

    def _on_progress(self, it: int) -> None:
        if self._running:
            self._status.setText(f"Planning... {it} iterations")

    def _on_error(self, gen: int, message: str) -> None:
        if gen != self._generation:
            return
        self._running = False
        self._cancel_evt = None
        last = message.strip().splitlines()[-1] if message.strip() else "error"
        self._status.setText(f"Failed: {last}")
        self.log_requested.emit(f"[ERROR] Planning failed: {last}")
        self._update_buttons()

    def _on_result(self, gen: int, payload) -> None:
        if gen != self._generation:      # robot changed or project closed meanwhile
            return
        res, timed = payload
        self._running = False
        self._cancel_evt = None
        self._result, self._timed = res, timed
        if res.success:
            self._status.setText("Path found.")
            traj = timed.trajectory
            text = (f"{len(res.path)} waypoints, length {res.length:.3f} rad "
                    f"(raw {res.raw_length:.3f}), {res.iterations} iterations, "
                    f"{res.time * 1000:.0f} ms; trajectory {traj.duration:.2f} s "
                    f"({timed.interpolation}); limits: {timed.report.summary().splitlines()[0]}")
            self._summary.setText(text)
            self.log_requested.emit(f"[INFO] Planning done: {text}")
        else:
            self._status.setText(f"No path: {res.reason}.")
            self._summary.setText(f"{res.iterations} iterations, {res.time * 1000:.0f} ms")
            self.log_requested.emit(f"[WARNING] Planning failed: {res.reason}")
        self._update_buttons()
        self.plan_finished.emit(res)

    def send_to_trajectory(self) -> None:
        """Hand the timed trajectory of the last successful plan to the trajectory editor."""
        if self._timed is None or self._result is None or not self._result.success:
            return
        self.trajectory_ready.emit(self._timed.trajectory)
        self.log_requested.emit("[INFO] Planned path sent to the trajectory editor.")

    def shutdown(self) -> None:
        self._generation += 1
        if self._cancel_evt is not None:
            self._cancel_evt.set()
