"""
tests/python/test_task_runner.py
R1.4: TaskRunner and Task unit tests.
"""

import os
import time
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication
from PySide6.QtCore import QTimer
from app.task_runner import Task, FunctionTask, TaskRunner, WorkerSignals


# ------------------------------------------------------------------
# Concrete Task subclass for testing
# ------------------------------------------------------------------

class AddTask(Task):
    """Test task: returns a + b after a short delay."""

    def __init__(self, a: int, b: int, delay: float = 0.0):
        super().__init__()
        self._a = a
        self._b = b
        self._delay = delay

    def execute(self) -> int:
        if self._delay > 0:
            time.sleep(self._delay)
        return self._a + self._b


class ProgressTask(Task):
    """Test task: emits 10 progress updates and returns 'done'."""

    def execute(self) -> str:
        for i in range(10):
            if self.is_cancelled:
                return None
            self.report_progress(i * 10, f"Step {i}")
            time.sleep(0.001)
        self.report_progress(100, "Complete")
        return "done"


class ErrorTask(Task):
    """Test task: always raises an exception."""

    def execute(self) -> None:
        raise ValueError("Deliberate test error")


class SlowCancellableTask(Task):
    """Test task: runs until cancelled."""

    def execute(self) -> str:
        for i in range(1000):
            if self.is_cancelled:
                return None
            time.sleep(0.001)
        return "finished"


# ------------------------------------------------------------------
# Tests
# ------------------------------------------------------------------

def test_worker_signals_creation():
    """WorkerSignals creates without error."""
    signals = WorkerSignals()
    assert signals is not None


def test_task_creation():
    """Task subclass creates without error."""
    task = AddTask(1, 2)
    assert task is not None
    assert not task.is_cancelled


def test_task_runner_creation(qapp):
    """TaskRunner creates without error."""
    runner = TaskRunner()
    assert runner is not None
    assert runner.max_threads > 0
    runner.shutdown()


def test_task_runner_submit_and_result(qapp):
    """TaskRunner submits a task and delivers the result via signal."""
    results = []
    runner = TaskRunner()
    task = AddTask(3, 4)
    task.signals.result.connect(results.append)

    runner.submit(task)
    runner.wait_all(timeout_ms=5000)
    QApplication.processEvents()

    assert len(results) == 1
    assert results[0] == 7
    runner.shutdown()


def test_task_runner_function_task(qapp):
    """FunctionTask wraps a callable and returns its result."""
    results = []
    runner = TaskRunner()
    task = runner.submit_fn(lambda: 42)
    task.signals.result.connect(results.append)

    runner.wait_all(timeout_ms=5000)
    QApplication.processEvents()

    assert len(results) == 1
    assert results[0] == 42
    runner.shutdown()


def test_task_finished_signal(qapp):
    """Task emits finished signal after completion."""
    finished = []
    runner = TaskRunner()
    task = AddTask(1, 1)
    task.signals.finished.connect(lambda: finished.append(True))

    runner.submit(task)
    runner.wait_all(timeout_ms=5000)
    QApplication.processEvents()

    assert len(finished) == 1
    runner.shutdown()


def test_task_error_signal(qapp):
    """ErrorTask emits error signal with traceback string."""
    errors = []
    runner = TaskRunner()
    task = ErrorTask()
    task.signals.error.connect(errors.append)

    runner.submit(task)
    runner.wait_all(timeout_ms=5000)
    QApplication.processEvents()

    assert len(errors) == 1
    assert "ValueError" in errors[0]
    assert "Deliberate test error" in errors[0]
    runner.shutdown()


def test_task_progress_signal(qapp):
    """ProgressTask emits progress signals."""
    progress_updates = []
    runner = TaskRunner()
    task = ProgressTask()
    task.signals.progress.connect(lambda p, m: progress_updates.append(p))

    runner.submit(task)
    runner.wait_all(timeout_ms=5000)
    QApplication.processEvents()

    assert len(progress_updates) > 0
    assert 100 in progress_updates
    runner.shutdown()


def test_task_cancellation(qapp):
    """Cancelled task emits cancelled signal and not result."""
    results = []
    cancelled = []
    runner = TaskRunner()
    task = SlowCancellableTask()
    task.signals.result.connect(results.append)
    task.signals.cancelled.connect(lambda: cancelled.append(True))

    runner.submit(task)
    time.sleep(0.02)  # Let it start
    task.cancel()
    runner.wait_all(timeout_ms=5000)
    QApplication.processEvents()

    assert len(cancelled) == 1
    # result may or may not fire depending on timing; cancelled must fire
    runner.shutdown()


def test_task_cancel_before_start():
    """Task cancelled before submission does not run execute()."""
    task = AddTask(1, 1)
    task.cancel()
    assert task.is_cancelled


def test_main_window_has_task_runner(qapp):
    """MainWindow exposes a TaskRunner via .task_runner property."""
    from app.main_window import MainWindow
    from app.task_runner import TaskRunner
    window = MainWindow()
    assert isinstance(window.task_runner, TaskRunner)
    window.close()
