"""R5.5: Workspace dock, backend wiring, point cloud / voxel actors (offscreen)."""
import os
import time
import unittest.mock as mock
from pathlib import Path

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtCore import QSettings
from PySide6.QtWidgets import QApplication

from app.compute import backend as be
from app.compute.backend import GpuBackend
from app.main_window import MainWindow
from app.settings import AppSettings

FX = Path(__file__).parent / "fixtures" / "urdf"
TEST_ORG = "RoboticsAnalyzer_test_r55ui"


@pytest.fixture
def win(qapp, monkeypatch):
    monkeypatch.setattr(AppSettings, "_ORGANIZATION", TEST_ORG)
    monkeypatch.setattr(AppSettings, "_APPLICATION", TEST_ORG)
    monkeypatch.delenv(be.ENV_FORCE_CPU, raising=False)
    s = QSettings(TEST_ORG, TEST_ORG)
    s.clear()
    s.sync()
    w = MainWindow()
    w.show()
    yield w
    w.project_controller._is_modified = False
    w.workspace_panel.cancel()
    w.task_runner.wait_all(20000)
    w.workspace_dock.hide()
    w.close()
    s.clear()
    s.sync()


def _pump(win, ms=20000):
    assert win.task_runner.wait_all(ms), "task did not finish"
    QApplication.processEvents()


def _load(win, name="planar_2r.urdf"):
    with mock.patch("app.main_window.QMessageBox.warning"):
        win.import_urdf_file(str(FX / name))
        _pump(win)
    win.project_controller._is_modified = False
    return win.project_controller.robot


def _cpu_panel(win, n=200_000):
    p = win.workspace_panel
    p._backend.setCurrentText("cpu")
    p._samples.setValue(n)
    return p


def test_dock_hidden_at_start_and_view_toggle(win):
    assert not win.workspace_dock.isVisible()
    win._action_view_workspace.trigger()
    assert win.workspace_dock.isVisible()
    win._action_view_workspace.trigger()
    assert not win.workspace_dock.isVisible()
    assert win.workspace_dock.windowTitle() == "Workspace"


def test_defaults_and_run_disabled_without_robot(win):
    p = win.workspace_panel
    assert p._samples.value() == 1_000_000
    assert p._seed.value() == 42
    assert p._backend.currentText() == "auto"
    assert p._chk_points.isChecked()
    assert not p._btn_run.isEnabled() and not p._btn_cancel.isEnabled()
    p.run()                       # no robot: no-op
    assert not p.is_running


def test_voxel_default_from_reach(win):
    model = _load(win)
    p = win.workspace_panel
    reach = be.estimate_reach(model)
    assert p._voxel.value() == pytest.approx(reach / 100.0, rel=1e-6)
    assert p._btn_run.isEnabled()


def test_run_2r_progress_and_cloud_actor(win):
    _load(win)
    p = _cpu_panel(win, 200_000)
    rv = win.viewport.robot_view
    assert rv.workspace_points_actor is None
    t0 = time.perf_counter()
    p.run()
    assert time.perf_counter() - t0 < 1.0     # the handler returned: UI thread not blocked
    assert p.is_running and not p._btn_run.isEnabled() and p._btn_cancel.isEnabled()
    assert p.result is None                   # result arrives later
    _pump(win)
    assert not p.is_running
    assert p.progress_value == 100
    assert p.result["n_samples_done"] == 200_000
    assert rv.workspace_points_actor is not None
    assert 0 < rv.workspace_points_count <= 2_000_000
    assert rv.workspace_voxel_actor is None   # "Show voxels" off
    assert "200000 samples" in p.status_text and "cpu" in p.status_text
    assert p._btn_run.isEnabled() and not p._btn_cancel.isEnabled()
    assert p.last_info["backend"] == "cpu"
    assert any("Workspace done" in line for line in win.log_panel.text().splitlines()) \
        if hasattr(win.log_panel, "text") else True


def test_show_checkboxes_toggle_actors(win):
    _load(win)
    p = _cpu_panel(win, 100_000)
    rv = win.viewport.robot_view
    p.run()
    _pump(win)
    assert rv.workspace_points_actor is not None
    p._chk_voxels.setChecked(True)
    assert rv.workspace_voxel_actor is not None and rv.workspace_voxel_count > 0
    p._chk_points.setChecked(False)
    assert rv.workspace_points_actor is None
    p._chk_voxels.setChecked(False)
    assert rv.workspace_voxel_actor is None


def test_cancel_stops_run(win):
    _load(win)
    p = _cpu_panel(win, 1_000_000_000)        # far too long to finish on its own
    p.run()
    assert p.is_running
    end = time.perf_counter() + 0.4
    while time.perf_counter() < end:
        QApplication.processEvents()
    p.cancel()
    _pump(win, 30000)
    assert not p.is_running
    assert p.result["cancelled"] is True
    assert p.result["n_samples_done"] < 1_000_000_000
    assert "Cancelled" in p.status_text


def test_clear_on_project_close(win):
    _load(win)
    p = _cpu_panel(win, 100_000)
    p.run()
    _pump(win)
    rv = win.viewport.robot_view
    assert p.result is not None and rv.workspace_points_actor is not None
    assert win.project_controller.close_project(None)
    QApplication.processEvents()
    assert p.result is None
    assert rv.workspace_points_actor is None and rv.workspace_voxel_actor is None
    assert not p._btn_run.isEnabled()


def test_clear_on_robot_change_and_stale_result_dropped(win):
    model = _load(win)
    p = _cpu_panel(win, 100_000)
    rv = win.viewport.robot_view
    p.run()
    _pump(win)
    assert rv.workspace_points_actor is not None
    win.project_controller.set_robot(model)          # robot replaced
    QApplication.processEvents()
    assert p.result is None and rv.workspace_points_actor is None
    # result arriving after a robot change is dropped
    p._samples.setValue(300_000)
    p.run()
    win.project_controller.set_robot(model)
    _pump(win)
    assert p.result is None and rv.workspace_points_actor is None


def test_fallback_reason_shown(win):
    _load(win)
    p = win.workspace_panel
    p._samples.setValue(50_000)

    class Broken:
        def device_info(self):
            return dict(available=True, name="fake", error="")

        def is_dead(self):
            return False

        def sample_workspace(self, *a, **k):
            raise RuntimeError("CUDA error: fake sticky error")

    p._backend_factory = lambda pref: GpuBackend(Broken(), "test")
    p.run()
    _pump(win)
    assert p.last_info["fallback"] is True
    assert "FALLBACK" in p.status_text and "fake sticky error" in p.status_text
    assert win.viewport.robot_view.workspace_points_actor is not None


def test_error_is_reported_not_raised(win):
    _load(win)
    p = _cpu_panel(win, 50_000)
    p._voxel.set_value(1e-6)                           # grid too large -> ValueError in worker
    p.run()
    _pump(win)
    assert not p.is_running
    assert p.status_text.startswith("Failed")
    assert p.result is None


def test_compute_settings_persist(win):
    p = win.workspace_panel
    p._backend.setCurrentText("cpu")
    p._vram.setValue(3000)
    p._vram.editingFinished.emit()
    s = AppSettings()
    assert s.compute_backend() == "cpu"
    assert s.vram_budget_mb() == 3000
    assert p.options()["budget_bytes"] == 3000 * 1024 * 1024


def test_robot_view_caps(win):
    rv = win.viewport.robot_view
    rng = np.random.default_rng(0)
    rv.set_workspace_points(rng.random((2_300_000, 3), dtype=np.float32))
    assert rv.workspace_points_count <= 2_000_000
    rv.set_workspace_points(None)
    assert rv.workspace_points_actor is None and rv.workspace_points_count == 0
    rv.set_workspace_voxels(rng.random((230_000, 3)), 0.01)
    assert rv.workspace_voxel_count <= 200_000
    assert "cap" in rv.workspace_voxel_note and "230000" in rv.workspace_voxel_note
    rv.set_workspace_voxels(rng.random((50, 3)), 0.01)
    assert rv.workspace_voxel_count == 50 and rv.workspace_voxel_note == ""
    rv.clear_workspace()
    assert rv.workspace_voxel_actor is None
