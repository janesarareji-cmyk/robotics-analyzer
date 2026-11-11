"""R5.6: performance dashboard (benchmark function, JSON, panel, timer, last-run wiring)."""
import json
import unittest.mock as mock
from pathlib import Path

import pytest

pytest.importorskip("PySide6")

from PySide6.QtCore import QSettings
from PySide6.QtWidgets import QApplication

from app.compute import backend as be
from app.compute import last_run
from app.compute.backend import CpuBackend, GpuBackend, get_backend
from app.compute.benchmark import BENCH_KEYS, ROW_KEYS, run_benchmark, write_benchmark
from app.main_window import MainWindow
from app.settings import AppSettings

FX = Path(__file__).parent / "fixtures" / "urdf"
TEST_ORG = "RoboticsAnalyzer_test_r56perf"
VERSION_KEYS = ("gpu", "sm", "driver_version", "toolkit", "runtime_version", "numpy",
                "pyside6", "python", "cpu", "cpu_threads", "git_commit")


def _gpu_backend():
    gb, why = be._gpu_bridge_or_reason()
    if gb is None:
        pytest.skip("no CUDA device")
    return GpuBackend(gb, "test")


@pytest.fixture
def win(qapp, monkeypatch):
    monkeypatch.setattr(AppSettings, "_ORGANIZATION", TEST_ORG)
    monkeypatch.setattr(AppSettings, "_APPLICATION", TEST_ORG)
    monkeypatch.delenv(be.ENV_FORCE_CPU, raising=False)
    s = QSettings(TEST_ORG, TEST_ORG)
    s.clear()
    s.sync()
    last_run.clear_last_run()
    w = MainWindow()
    w.show()
    yield w
    w.project_controller._is_modified = False
    w.workspace_panel.cancel()
    w.task_runner.wait_all(30000)
    w.performance_dock.hide()
    w.close()
    last_run.clear_last_run()
    s.clear()
    s.sync()


def _pump(win, ms=60000):
    assert win.task_runner.wait_all(ms), "task did not finish"
    QApplication.processEvents()


def _load(win, name="planar_2r.urdf"):
    with mock.patch("app.main_window.QMessageBox.warning"):
        win.import_urdf_file(str(FX / name))
        _pump(win)
    win.project_controller._is_modified = False
    return win.project_controller.robot


def _model():
    from .robots import fk_test_robots
    return fk_test_robots()["six_dof_fixture"]


def test_benchmark_cpu_keys_and_rows():
    res = run_benchmark(CpuBackend("t"), _model(), n=20_000, n_ik=500)
    for k in BENCH_KEYS:
        assert k in res
    assert res["backend"] == "cpu" and res["n"] == 20_000 and res["n_ik"] == 500
    for k in ("fk", "workspace", "ik"):
        assert res[k]["backend"] is None and res[k]["speedup"] is None
        for rk in ROW_KEYS:
            assert rk in res[k]["cpu"]
        assert res[k]["cpu"]["samples_per_s"] > 0 and res[k]["cpu"]["ms"] > 0


def test_benchmark_rejects_no_robot():
    with pytest.raises(ValueError):
        run_benchmark(CpuBackend("t"), None, n=100)


def test_benchmark_json_has_versions(tmp_path):
    res = run_benchmark(CpuBackend("t"), _model(), n=5_000, n_ik=200)
    p = write_benchmark(res, tmp_path / "sub" / "bench.json")
    data = json.loads(p.read_text())
    for k in VERSION_KEYS:
        assert k in data["versions"]
    assert data["versions"]["numpy"] and data["versions"]["cpu_threads"] >= 1
    assert data["versions"]["gpu"] is None   # CPU backend: no GPU fields
    assert "fk" in data and "workspace" in data and "ik" in data


def test_benchmark_gpu_rows_and_versions():
    gbk = _gpu_backend()
    res = run_benchmark(gbk, _model(), n=50_000, n_ik=500)
    assert res["backend"] == "cuda"
    for k in ("fk", "workspace"):
        assert res[k]["backend"]["samples_per_s"] > 0
        assert res[k]["speedup"] and res[k]["speedup"] > 0
    assert res["versions"]["gpu"] and res["versions"]["sm"] and res["versions"]["driver_version"]
    assert res["versions"]["toolkit"] is not None


def test_last_run_holder():
    last_run.clear_last_run()
    assert last_run.get_last_run() is None
    last_run.set_last_run("cpu", {"n_samples": 5}, {"chunk": 7})
    lr = last_run.get_last_run()
    assert lr["backend"] == "cpu" and lr["options"]["n_samples"] == 5 and lr["info"]["chunk"] == 7
    last_run.clear_last_run()
    assert last_run.get_last_run() is None


def test_dock_hidden_at_start_and_view_toggle(win):
    assert not win.performance_dock.isVisible()
    assert not win.performance_panel.timer_active
    win._action_view_performance.trigger()
    assert win.performance_dock.isVisible()
    win._action_view_performance.trigger()
    assert not win.performance_dock.isVisible()
    assert win.performance_dock.windowTitle() == "Performance"


def test_timer_only_while_visible(win):
    p = win.performance_panel
    assert not p.timer_active
    win.performance_dock.show()
    QApplication.processEvents()
    assert p.timer_active
    win.performance_dock.hide()
    QApplication.processEvents()
    assert not p.timer_active


def test_panel_cpu_summary_without_gpu(win):
    p = win.performance_panel
    p._backend_factory = lambda: get_backend("cpu", win.settings if hasattr(win, "settings") else None)
    p.refresh()
    assert "No GPU" in p.device_text
    assert p.vram_text == "n/a"
    assert p.last_settings_text == "no run yet"


def test_panel_benchmark_cpu_path_and_save(win, tmp_path):
    _load(win)
    p = win.performance_panel
    p._backend_factory = lambda: CpuBackend("test")
    p.bench_samples = 20_000
    assert p._btn_run.isEnabled()
    p.run_benchmark()
    assert p.is_running and not p._btn_run.isEnabled()
    _pump(win)
    QApplication.processEvents()
    assert not p.is_running, p._bench_status.text()
    assert p.last_result is not None and p.last_result["n"] == 20_000
    assert "CPU" in p.fk_text and "M/s" in p.fk_text
    out = tmp_path / "b.json"
    p.save_benchmark(str(out))
    assert "versions" in json.loads(out.read_text())


def test_run_disabled_without_robot(win):
    assert not win.performance_panel._btn_run.isEnabled()


def test_workspace_run_feeds_last_run_and_panel(win):
    _load(win)
    wp = win.workspace_panel
    wp._backend.setCurrentText("cpu")
    wp._samples.setValue(50_000)
    wp.run()
    _pump(win)
    lr = last_run.get_last_run()
    assert lr is not None and lr["backend"] == "cpu"
    assert lr["options"]["n_samples"] == 50_000
    win.performance_panel.refresh()
    assert "50000 samples" in win.performance_panel.last_settings_text
    assert "MiB" in win.performance_panel.last_peak_text


def test_panel_gpu_vram_line(win):
    _gpu_backend()
    p = win.performance_panel
    p._backend_factory = lambda: _gpu_backend()
    p.refresh()
    assert "MiB" in p.vram_text and "/" in p.vram_text
