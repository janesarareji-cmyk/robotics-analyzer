"""Demo tour (D1.3): visit the targets of the project one after the other.

``DemoRunner`` takes the targets in store order and for each one:

* reachable (a valid, collision-free IK solution exists)  -> plans a collision-free motion from the
  current pose (``plan_target_motion``, no approach), hands the trajectory to the window for
  playback (``play_requested``), and after the playback finishes waits ``pause_ms`` ("reached");
* unreachable -> status line ``Target "Too far": unreachable, skipped (reason)`` and on.

After the last target the robot returns to the first reachable one (A -> B -> C -> A), when at
least two were reachable. Progress is ``Step k of N``. ``cancel()`` stops at once. Planning runs in
a worker (``TaskRunner``); the runner itself has no Qt widget dependency (signals + a QTimer), so it
works with the Simulation dock hidden.

``classify_targets`` is the same reachability test used for the green / red 3D markers.
"""
from __future__ import annotations

import logging
import threading
from dataclasses import dataclass, field
from typing import Any, Callable

import numpy as np
from PySide6.QtCore import QObject, QTimer, Signal

import core_bridge as cb
from app.analysis.ik_rank import rank_solutions
from app.analysis.target_motion import plan_target_motion
from app.kin_utils import current_q
from app.target_preview import target_status
from app.task_runner import FunctionTask

log = logging.getLogger("robotics_analyzer.demo")

PAUSE_MS = 1200            # wait at each reached target
SKIP_MS = 1500             # a skipped (unreachable) target stays on screen this long


def classify_targets(model: Any, targets: Any, q: Any, session: Any, cm: Any = None) -> list[dict]:
    """``[{"name", "status", "reason", "q"}]`` per target: status ``ok`` / ``warn`` / ``bad``.

    ``bad`` = no valid solution (reason: ``out of reach`` when IK finds nothing, else the reason of
    the best invalid solution, e.g. ``collision`` or ``outside joint limits``).
    """
    out = []
    for t in targets:
        rr = rank_solutions(model, t.pose, q, session, cm=cm, n_starts=40)
        st = target_status(rr.solutions)
        best = rr.best()
        if best is not None:
            reason, qs = "", list(best.q)
        else:
            qs = None
            if rr.solutions:
                reason = (rr.solutions[0].reason or "invalid").split(":")[0]
            else:
                reason = "out of reach"
        out.append({"name": t.name, "status": st, "reason": reason, "q": qs})
    return out


@dataclass
class DemoSummary:
    visited: list = field(default_factory=list)
    skipped: list = field(default_factory=list)
    failed: list = field(default_factory=list)
    cancelled: bool = False

    def text(self) -> str:
        if self.cancelled:
            return "Demo cancelled."
        return (f"Demo finished: {len(self.visited)} visited, {len(self.skipped)} skipped"
                + (f", {len(self.failed)} failed." if self.failed else "."))


class DemoRunner(QObject):
    status_changed = Signal(str)         # one-line status ("Step 2 of 6: moving to ...")
    progress_changed = Signal(int, int)  # step, total (0, 0 when idle)
    step_started = Signal(str, str)      # target name, "move" | "skip" | "return"
    play_requested = Signal(object, dict)    # Trajectory, info (name, step, total)
    finished = Signal(object)            # DemoSummary
    _planned = Signal(int, object)
    _classified = Signal(int, object)
    _failed = Signal(int, str)

    def __init__(self, task_runner: Any, parent: QObject | None = None):
        super().__init__(parent)
        self._runner = task_runner
        self.pause_ms = PAUSE_MS
        self.skip_ms = SKIP_MS
        self._get_model: Callable[[], Any] = lambda: None
        self._get_store: Callable[[], Any] = lambda: []
        self._get_session: Callable[[], Any] = lambda: None
        self._get_cm: Callable[[Any], Any] = lambda m: None
        self._running = False
        self._gen = 0
        self._cancel: threading.Event | None = None
        self._steps: list[tuple[str, str]] = []      # (target name, "move" | "skip" | "return")
        self._poses: dict[str, np.ndarray] = {}
        self._index = -1
        self._waiting_playback = False
        self._summary = DemoSummary()
        self._timer = QTimer(self)
        self._timer.setSingleShot(True)
        self._timer.timeout.connect(self._next)
        self._classified.connect(self._on_classified)
        self._planned.connect(self._on_planned)
        self._failed.connect(self._on_failed)
        self._reasons: dict = {}
        self._task: Any = None
        self._pending_name = ""
        self.last_status = ""
        self.last_summary: DemoSummary | None = None
        self.step = 0
        self.total = 0

    # ------------------------------------------------------------------ setup
    def configure(self, get_model: Callable, get_store: Callable, get_session: Callable,
                  get_cm: Callable) -> None:
        self._get_model, self._get_store = get_model, get_store
        self._get_session, self._get_cm = get_session, get_cm

    @property
    def running(self) -> bool:
        return self._running

    @property
    def waiting_playback(self) -> bool:
        return self._waiting_playback

    @property
    def current_name(self) -> str | None:
        return self._steps[self._index][0] if 0 <= self._index < len(self._steps) else None

    def _say(self, text: str) -> None:
        self.last_status = text
        self.status_changed.emit(text)

    # ------------------------------------------------------------------ control
    def start(self) -> bool:
        """Begin the tour from the current pose. False when it cannot start (no robot / no targets)."""
        if self._running:
            return False
        model, store = self._get_model(), self._get_store()
        if model is None or len(list(store)) == 0:
            self._say("Demo: load a robot with targets first (File > Open Sample Robot).")
            return False
        self._running = True
        self._gen += 1
        gen = self._gen
        self._cancel = threading.Event()
        self._summary = DemoSummary()
        self._steps, self._index, self._waiting_playback = [], -1, False
        self.step, self.total = 0, 0
        targets = [t.copy() for t in store]
        self._poses = {t.name: t.pose.copy() for t in targets}
        session, q0 = self._get_session(), current_q(model)
        cm = self._get_cm(model)
        self._say("Demo: checking the targets ...")
        self.progress_changed.emit(0, len(targets) + 1)

        def work():
            return classify_targets(model, targets, q0, session, cm)

        task = FunctionTask(work)
        task.signals.result.connect(lambda r, g=gen: self._classified.emit(g, r))
        task.signals.error.connect(lambda m, g=gen: self._failed.emit(g, m))
        self._task = task
        self._runner.submit(task)
        return True

    def cancel(self) -> None:
        if not self._running:
            return
        self._gen += 1
        self._timer.stop()
        if self._cancel is not None:
            self._cancel.set()
        self._summary.cancelled = True
        self._finish()

    def _finish(self) -> None:
        self._running = False
        self._waiting_playback = False
        self._timer.stop()
        self.last_summary = self._summary
        self._say(self._summary.text())
        self.progress_changed.emit(0, 0)
        self.finished.emit(self._summary)

    # ------------------------------------------------------------------ steps
    def _on_classified(self, gen: int, res: list) -> None:
        if gen != self._gen or not self._running:
            return
        kinds = {r["name"]: r for r in res}
        order = [t for t in self._poses]                 # store order
        steps = [(n, "move" if kinds[n]["q"] is not None else "skip") for n in order]
        reach = [n for n, k in steps if k == "move"]
        if len(reach) >= 2:
            steps.append((reach[0], "return"))
        self._steps = steps
        self._reasons = {n: kinds[n]["reason"] for n in order}
        self.total = len(steps)
        self._index = -1
        self._next()

    def _on_failed(self, gen: int, message: str) -> None:
        if gen != self._gen or not self._running:
            return
        last = message.strip().splitlines()[-1] if message.strip() else "error"
        self._summary.failed.append(last)
        self._say(f"Demo stopped: {last}")
        self._finish()

    def _next(self) -> None:
        if not self._running:
            return
        self._index += 1
        if self._index >= len(self._steps):
            self._finish()
            return
        name, kind = self._steps[self._index]
        self.step = self._index + 1
        self.progress_changed.emit(self.step, self.total)
        self.step_started.emit(name, kind)
        head = f"Step {self.step} of {self.total}: "
        if kind == "skip":
            why = self._reasons.get(name, "")
            self._summary.skipped.append(name)
            self._say(head + f'Target "{name}": unreachable, skipped' + (f" ({why})" if why else ""))
            self._timer.start(int(self.skip_ms))
            return
        verb = "returning to" if kind == "return" else "moving to"
        self._say(head + f'{verb} "{name}" ...')
        self._plan(name, kind)

    def _plan(self, name: str, kind: str) -> None:
        model, session = self._get_model(), self._get_session()
        T = self._poses[name]
        q_start = current_q(model)
        cm = self._get_cm(model)
        evt = self._cancel
        gen = self._gen

        def work():
            rr = rank_solutions(model, T, q_start, session, cm=cm, n_starts=40)
            best = rr.best()
            if best is None:
                return {"ok": False, "message": "no valid solution from here"}
            world = session.world if session is not None else None
            res = plan_target_motion(model, q_start, best.q, T, world, cm=cm, cancel=evt)
            return {"ok": res.ok, "message": res.message, "result": res, "q": list(best.q)}

        task = FunctionTask(work)
        task.signals.result.connect(lambda r, g=gen, n=name, k=kind: self._planned.emit(g, (n, k, r)))
        task.signals.error.connect(lambda m, g=gen: self._failed.emit(g, m))
        self._task = task
        self._runner.submit(task)

    def _on_planned(self, gen: int, payload: Any) -> None:
        if gen != self._gen or not self._running:
            return
        name, kind, r = payload
        head = f"Step {self.step} of {self.total}: "
        if not r["ok"]:
            msg = r["message"]
            if "already at the target" in msg:
                self._summary.visited.append(name)
                self._say(head + f'already at "{name}"')
                self._timer.start(int(self.pause_ms))
                return
            self._summary.failed.append(name)
            self._say(head + f'no motion to "{name}" ({msg}), skipped')
            self._timer.start(int(self.skip_ms))
            return
        res = r["result"]
        self._waiting_playback = True
        self._pending_name = name
        self.play_requested.emit(res.trajectory, {"name": name, "step": self.step,
                                                  "total": self.total, "kind": kind,
                                                  "q_goal": r["q"]})

    def on_playback_finished(self) -> None:
        """Window: the trajectory handed out by ``play_requested`` reached its end."""
        if not self._running or not self._waiting_playback:
            return
        self._waiting_playback = False
        name = getattr(self, "_pending_name", "")
        self._summary.visited.append(name)
        self._say(f'Step {self.step} of {self.total}: reached "{name}"')
        self._timer.start(int(self.pause_ms))

    def on_play_failed(self, why: str = "") -> None:
        """Window could not play the trajectory (collision stop, does not fit the robot)."""
        if not self._running or not self._waiting_playback:
            return
        self._waiting_playback = False
        name = getattr(self, "_pending_name", "")
        self._summary.failed.append(name)
        self._say(f'Step {self.step} of {self.total}: "{name}" not reached ({why or "playback stopped"})')
        self._timer.start(int(self.skip_ms))
