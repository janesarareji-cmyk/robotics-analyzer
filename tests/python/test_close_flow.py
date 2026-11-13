"""A1.1: close flow (audit H1, M3, LOW temp dirs / stale autosave). Offscreen Qt."""
import os
import tempfile
import threading
import time
import unittest.mock as mock
from pathlib import Path

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtCore import QSettings
from PySide6.QtWidgets import QApplication, QMessageBox

from app.main_window import MainWindow
from app.project.project_data import ProjectData
from app.project.recovery import autosave_path, write_autosave
from app.settings import AppSettings

TEST_ORG = "RoboticsAnalyzer_test_a11close"
SB = QMessageBox.StandardButton


@pytest.fixture
def win(qapp, monkeypatch, tmp_path):
    monkeypatch.setattr(AppSettings, "_ORGANIZATION", TEST_ORG)
    monkeypatch.setattr(AppSettings, "_APPLICATION", TEST_ORG)
    s = QSettings(TEST_ORG, TEST_ORG)
    s.clear()
    s.sync()
    w = MainWindow()
    w.show()
    yield w
    w.project_controller._is_modified = False
    w.task_runner.wait_all(20000)
    QApplication.processEvents()
    w.close()
    s.clear()
    s.sync()


def _open_modified(win, tmp_path, name="p"):
    """Saved project on disk, then modified, with an autosave file and a temp asset dir."""
    pc = win.project_controller
    pc._set_project(ProjectData.new(name), modified=True)
    path = str(tmp_path / f"{name}.robotproj")
    assert pc.save_project_as(win, path)
    pc.mark_modified()
    asset = tempfile.mkdtemp(prefix="robotproj_assets_")
    pc.current_project.asset_dir = asset
    auto = pc.autosave_now()
    assert auto is not None and auto.is_file()
    return path, auto, asset


def test_close_modified_prompts_and_cancel_keeps_window(win, tmp_path, monkeypatch):
    """Audit H1: closing a modified project asks first; Cancel keeps window and project."""
    path, auto, asset = _open_modified(win, tmp_path)
    q = mock.Mock(return_value=SB.Cancel)
    monkeypatch.setattr(QMessageBox, "question", q)
    assert win.close() is False
    assert q.called
    assert win.isVisible()
    assert win.project_controller.has_project
    assert auto.is_file() and Path(asset).is_dir()


def test_close_discard_removes_autosave_and_temp(win, tmp_path, monkeypatch):
    """Audit H1 + LOW: Discard closes; autosave and temp asset dir are gone."""
    path, auto, asset = _open_modified(win, tmp_path)
    monkeypatch.setattr(QMessageBox, "question", mock.Mock(return_value=SB.Discard))
    assert win.close() is True
    assert not win.isVisible()
    assert not win.project_controller.has_project
    assert not auto.exists()
    assert not Path(asset).exists()


def test_close_save_writes_file(win, tmp_path, monkeypatch):
    """Audit H1: Save writes the project before closing."""
    path, auto, asset = _open_modified(win, tmp_path)
    win.project_controller.current_project.metadata.description = "edited-before-close"
    monkeypatch.setattr(QMessageBox, "question", mock.Mock(return_value=SB.Save))
    assert win.close() is True
    from app.project.serializer import load
    assert load(path).metadata.description == "edited-before-close"
    assert not auto.exists()


def test_close_with_hung_task_returns_fast_with_warning(win, caplog):
    """Audit M3: a task that never ends does not block close."""
    ev = threading.Event()
    names = []
    win.task_runner.submit_fn(lambda: ev.wait(30))
    time.sleep(0.2)
    win.task_runner._shutdown_timeout_ms = 1000   # not required to exist; harmless
    t0 = time.time()
    with caplog.at_level("WARNING"):
        win.close()
    dt = time.time() - t0
    ev.set()
    assert dt < 6.0
    assert any("still running" in r.getMessage() for r in caplog.records)


def test_task_runner_shutdown_timeout():
    """Audit M3: TaskRunner.shutdown(timeout_ms) returns False after the timeout."""
    from app.task_runner import TaskRunner
    tr = TaskRunner()
    ev = threading.Event()
    tr.submit_fn(lambda: ev.wait(30))
    time.sleep(0.2)
    t0 = time.time()
    assert tr.shutdown(timeout_ms=500) is False
    assert time.time() - t0 < 3.0
    ev.set()
    tr.wait_all(5000)


def test_pending_permission_ask_released_on_close(win):
    """Audit M3: a worker waiting in the permission dialog is released, denied, NOT saved."""
    pm = win._plugin_manager.permissions
    result = {}

    def worker():
        try:
            pm.check("pl", ["network"], "network")
            result["r"] = "granted"
        except PermissionError as exc:
            result["r"] = "denied"

    t = threading.Thread(target=worker, daemon=True)
    t0 = time.time()
    t.start()
    time.sleep(0.5)          # worker is blocked: GUI thread did not process events
    assert t.is_alive()
    win.close()
    t.join(5)
    assert not t.is_alive()
    assert time.time() - t0 < 10
    assert result["r"] == "denied"
    assert pm.state("pl", "network") == "ask"


def test_open_other_project_removes_old_autosave_and_temp(win, tmp_path):
    """Audit LOW: opening B after A removes A's autosave and temp dir."""
    pc = win.project_controller
    pathB = str(tmp_path / "b.robotproj")
    pc._set_project(ProjectData.new("b"), modified=True)
    assert pc.save_project_as(win, pathB)
    pathA, auto, asset = _open_modified(win, tmp_path, "a")
    pc._is_modified = False
    assert pc.open_project(win, pathB)
    assert not auto.exists()
    assert not Path(asset).exists()
