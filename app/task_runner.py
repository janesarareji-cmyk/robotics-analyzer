"""
app/task_runner.py
Robotics Analyzer — async task execution framework.
R1.4: QThreadPool-based task runner with typed signals.

Design:
- Task subclasses QRunnable. It has a WorkerSignals companion object.
- TaskRunner wraps QThreadPool and submits Task instances.
- UI code connects to task.signals.result / .progress / .error / .finished.
- No task may call Qt UI methods directly — only emit signals.
"""

from __future__ import annotations

import traceback
import threading
from typing import Any, Callable

from PySide6.QtCore import QObject, QRunnable, QThreadPool, Signal, Slot


class WorkerSignals(QObject):
    """
    Signals for a Task.
    Defined in a separate QObject because QRunnable cannot inherit QObject
    in PySide6 without ABI conflicts.

    Signals:
        result(object)       — Emitted with the task return value on success.
        progress(int, str)   — Emitted with (percent 0-100, description).
        error(str)           — Emitted with a formatted traceback on exception.
        finished()           — Emitted when the task ends (success or failure).
        cancelled()          — Emitted if the task exits due to cancellation.
    """

    result = Signal(object)
    progress = Signal(int, str)
    error = Signal(str)
    finished = Signal()
    cancelled = Signal()


class Task(QRunnable):
    """
    Base class for all background tasks.

    Subclass this and override execute(). Emit progress via self.report_progress().
    Check self.is_cancelled periodically in long loops and return early if True.

    Example:
        class MyTask(Task):
            def execute(self):
                for i in range(100):
                    if self.is_cancelled:
                        return None
                    self.report_progress(i, f"Step {i}")
                return "done"
    """

    def __init__(self):
        super().__init__()
        self.setAutoDelete(True)
        self.signals = WorkerSignals()
        self._cancelled = False
        self._cancel_lock = threading.Lock()

    # ------------------------------------------------------------------
    # Public API — called from UI thread
    # ------------------------------------------------------------------

    def cancel(self) -> None:
        """Request cancellation. Thread-safe."""
        with self._cancel_lock:
            self._cancelled = True

    @property
    def is_cancelled(self) -> bool:
        """Return True if cancellation was requested. Thread-safe."""
        with self._cancel_lock:
            return self._cancelled

    # ------------------------------------------------------------------
    # Task implementation — override in subclass
    # ------------------------------------------------------------------

    def execute(self) -> Any:
        """
        Override this method to implement the task body.
        Return the result value. Raise an exception on error.
        Return None or check self.is_cancelled to handle cancellation.
        Do NOT call any Qt UI methods here. Use self.report_progress() only.
        """
        raise NotImplementedError("Subclasses must implement execute()")

    # ------------------------------------------------------------------
    # Progress reporting — safe to call from execute()
    # ------------------------------------------------------------------

    def report_progress(self, percent: int, message: str = "") -> None:
        """Emit a progress update. percent must be 0–100."""
        self.signals.progress.emit(max(0, min(100, percent)), message)

    # ------------------------------------------------------------------
    # QRunnable entry point — do not override
    # ------------------------------------------------------------------

    def run(self) -> None:
        """QRunnable.run() — dispatches to execute() with error handling."""
        if self.is_cancelled:
            self.signals.cancelled.emit()
            self.signals.finished.emit()
            return
        try:
            result = self.execute()
            if self.is_cancelled:
                self.signals.cancelled.emit()
            else:
                self.signals.result.emit(result)
        except Exception:
            error_msg = traceback.format_exc()
            self.signals.error.emit(error_msg)
        finally:
            self.signals.finished.emit()


class FunctionTask(Task):
    """
    Convenience Task that wraps a plain Python callable.
    Use this for one-off tasks without subclassing Task.

    Example:
        task = FunctionTask(lambda: expensive_computation(data))
        task.signals.result.connect(on_result)
        runner.submit(task)
    """

    def __init__(self, fn: Callable[[], Any]):
        super().__init__()
        self._fn = fn

    def execute(self) -> Any:
        return self._fn()


class TaskRunner(QObject):
    """
    Application task runner.
    Wraps QThreadPool. Provides submit(), wait_all(), and thread counts.
    One TaskRunner instance is shared across the application.
    """

    def __init__(self, parent=None, max_threads: int | None = None):
        super().__init__(parent)
        self._pool = QThreadPool()
        if max_threads is not None:
            self._pool.setMaxThreadCount(max_threads)

    def submit(self, task: Task) -> None:
        """Submit a task for async execution. Returns immediately."""
        self._pool.start(task)

    def submit_fn(self, fn: Callable[[], Any]) -> FunctionTask:
        """
        Wrap a callable in a FunctionTask and submit it.
        Returns the FunctionTask so callers can connect to its signals.
        """
        task = FunctionTask(fn)
        self._pool.start(task)
        return task

    def wait_all(self, timeout_ms: int = -1) -> bool:
        """
        Block until all active tasks complete.
        Returns True if all tasks finished, False if timeout expired.
        timeout_ms=-1 means wait indefinitely.
        """
        return self._pool.waitForDone(timeout_ms)

    def shutdown(self) -> None:
        """Wait for all tasks and release thread pool resources."""
        self._pool.waitForDone()
        self._pool.clear()

    @property
    def active_count(self) -> int:
        """Return the number of currently running tasks."""
        return self._pool.activeThreadCount()

    @property
    def max_threads(self) -> int:
        """Return the maximum number of threads in the pool."""
        return self._pool.maxThreadCount()
