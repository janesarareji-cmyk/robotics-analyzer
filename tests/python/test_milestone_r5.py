"""
tests/python/test_milestone_r5.py
R5 stage milestone: GPU workspace study and FK batch on the 6-DOF fixture, driven through
an offscreen MainWindow with the real core_bridge and gpu_bridge.

GPU parts skip with a clear message when no CUDA device is present; the CPU parts
(item 1, CPU half of 5, 7) still run. fp32 tolerances: docs/gpu-precision-decision.md
(R5 Decisions 12).
"""

import json
import os
import time
import unittest.mock as mock
from pathlib import Path

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtCore import QSettings
from PySide6.QtWidgets import QApplication

import core_bridge as cb
from app.compute import backend as be
from app.compute import last_run
from app.compute.backend import CpuBackend, GpuBackend, get_backend
from app.compute.benchmark import BENCH_KEYS, run_benchmark, write_benchmark
from app.kin_utils import dof_limits
from app.main_window import MainWindow
from app.settings import AppSettings

from .test_gpu_fk import _rot_angle, _tolerances

FX = Path(__file__).parent / "fixtures" / "urdf"
URDF = "six_dof_with_gripper.urdf"
TEST_ORG = "RoboticsAnalyzer_test_r5milestone"
N_WS = 1_000_000
N_SUB = 10_000
EIGHT_GIB = 8 * 1024 ** 3
MAX_POINTS = 2_000_000
VERSION_KEYS = ("gpu", "sm", "driver_version", "toolkit", "runtime_version", "numpy",
                "pyside6", "python", "cpu", "cpu_threads", "git_commit")


def _gpu_or_skip():
    gb, why = be._gpu_bridge_or_reason()
    if gb is None:
        pytest.skip(f"no CUDA device ({why})")
    return gb


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
    w.task_runner.wait_all(60000)
    w.workspace_dock.hide()
    w.performance_dock.hide()
    w.close()
    last_run.clear_last_run()
    s.clear()
    s.sync()


def _pump(win, ms=120000):
    assert win.task_runner.wait_all(ms), "task did not finish"
    QApplication.processEvents()


def _load(win):
    """Item 1: import the fixture and set the TCP link to l6."""
    with mock.patch("app.main_window.QMessageBox.warning") as w:
        win.import_urdf_file(str(FX / URDF))
        _pump(win)
    assert w.call_count == 0
    model = win.project_controller.robot
    names = [lk.name for lk in model.links]
    win.set_tcp_link(names.index("l6"))
    win.project_controller._is_modified = False
    assert names[model.tcp_link] == "l6"
    return model


def _voxel_set(r):
    return set(map(tuple, np.asarray(r["occupied_voxels"]).tolist()))


def _tol_pos(model):
    return _tolerances(model)[1]


# ---------------------------------------------------------------------------
# Item 1, 2, 4: GPU workspace through the panel
# ---------------------------------------------------------------------------

def test_items_1_2_4_gpu_workspace_panel(win):
    _gpu_or_skip()
    model = _load(win)
    assert model.dof() == 7
    p = win.workspace_panel
    p._backend.setCurrentText("cuda")
    p._samples.setValue(N_WS)
    rv = win.viewport.robot_view
    assert rv.workspace_points_actor is None
    p.run()
    assert p.is_running
    _pump(win)
    assert not p.is_running
    info = p.last_info
    # item 2: 1e6 samples on the GPU, grid + capped cloud in the viewport
    assert info["backend"] == "cuda" and not info["fallback"], info
    assert p.result["n_samples_done"] == N_WS
    assert len(p.result["occupied_voxels"]) > 0
    assert rv.workspace_points_actor is not None
    assert 0 < rv.workspace_points_count <= MAX_POINTS
    p._chk_voxels.setChecked(True)
    assert rv.workspace_voxel_actor is not None and rv.workspace_voxel_count > 0
    # item 4: VRAM peak below the configured budget and below 8 GB; log has chunk + peak
    budget = int(info["budget_bytes"])
    assert budget == p.options()["budget_bytes"]
    assert 0 < info["vram_peak_bytes"] <= budget < EIGHT_GIB
    assert info["vram_peak_bytes"] < EIGHT_GIB
    log = win.log_panel.toPlainText()
    assert "Workspace done" in log
    assert f"chunk {info['chunk']}" in log and "VRAM peak" in log
    print(f"\n[ws gpu] {info['total_ms']:.1f} ms, peak {info['vram_peak_bytes'] / 2**20:.1f} MiB "
          f"of budget {budget / 2**20:.0f} MiB, chunk {info['chunk']}, chunks {info['chunks']}, "
          f"voxels {len(p.result['occupied_voxels'])}")


# ---------------------------------------------------------------------------
# Item 3: 1e4-sample GPU subset equals the CPU result
# ---------------------------------------------------------------------------

def test_item_3_subset_gpu_equals_cpu(win):
    _gpu_or_skip()
    model = _load(win)
    gbk = get_backend("cuda")
    assert isinstance(gbk, GpuBackend)
    voxel = be.estimate_reach(model) / 100.0
    opt = dict(mode="random", n_samples=N_SUB, seed=42, voxel_size=voxel,
               want_counts=False, want_points=True, max_points=N_SUB)
    rg = gbk.sample_workspace(model, opt)
    rc = CpuBackend("ref").sample_workspace(model, opt)
    assert rg["info"]["backend"] == "cuda" and rc["info"]["backend"] == "cpu"
    assert rg["n_samples_done"] == rc["n_samples_done"] == N_SUB
    assert rg["grid_dims"] == rc["grid_dims"]
    # positions of the first 1e4 samples: Decisions 12 tolerance
    pg, pc = rg["points"], rc["points"]
    assert pg.shape == pc.shape == (N_SUB, 3)
    tol = _tol_pos(model)
    dmax = float(np.abs(pg - pc).max())
    print(f"\n[subset] points max diff {dmax:.3e} (tol {tol:.3e})")
    assert dmax <= tol + 1e-6
    # grid: at most 0.1 % of voxels differ
    sg, sc = _voxel_set(rg), _voxel_set(rc)
    frac = len(sg ^ sc) / max(1, len(sc))
    print(f"[subset] occupied cpu {len(sc)} gpu {len(sg)}, differing fraction {frac:.2e}")
    assert frac <= 0.001


# ---------------------------------------------------------------------------
# Item 5: cancel, CPU backend by setting, fallback
# ---------------------------------------------------------------------------

def test_item_5a_cancel_mid_run_gpu(win):
    _gpu_or_skip()
    _load(win)
    p = win.workspace_panel
    p._backend.setCurrentText("cuda")
    p._samples.setValue(2_000_000_000)           # far too long to finish by itself
    p.run()
    assert p.is_running
    end = time.perf_counter() + 0.4
    while time.perf_counter() < end:
        QApplication.processEvents()
    t0 = time.perf_counter()
    p.cancel()
    _pump(win, 60000)
    assert time.perf_counter() - t0 < 30.0
    assert not p.is_running
    assert p.result["cancelled"] is True
    assert p.result["n_samples_done"] < 2_000_000_000
    assert "Cancelled" in p.status_text


def test_item_5b_cpu_by_setting_matches_gpu_subset(win):
    gb = _gpu_or_skip()
    model = _load(win)
    p = win.workspace_panel
    p._samples.setValue(100_000)
    # CPU forced by the setting (backend combo is persisted into AppSettings)
    p._backend.setCurrentText("cpu")
    assert AppSettings().compute_backend() == "cpu"
    p.run()
    _pump(win)
    assert p.last_info["backend"] == "cpu" and not p.last_info["fallback"]
    cpu_res = p.result
    assert cpu_res["n_samples_done"] == 100_000
    # same workspace on the GPU (same options)
    opts = p.last_options
    gpu_res = get_backend("cuda").sample_workspace(model, opts)
    assert gpu_res["info"]["backend"] == "cuda"
    sc, sg = _voxel_set(cpu_res), _voxel_set(gpu_res)
    frac = len(sc ^ sg) / max(1, len(sc))
    print(f"\n[1e5 subset] occupied cpu {len(sc)} gpu {len(sg)}, differing fraction {frac:.2e}")
    assert cpu_res["grid_dims"] == gpu_res["grid_dims"]
    assert frac <= 0.001
    assert gb is not None


def test_item_5c_fake_oom_falls_back_and_reports(win):
    gb = _gpu_or_skip()
    _load(win)
    p = win.workspace_panel
    p._backend.setCurrentText("cuda")
    p._samples.setValue(300_000)
    gb.set_fake_oom(1)
    try:
        p.run()
        _pump(win)
    finally:
        gb.set_fake_oom(0)
    assert p.last_info["backend"] == "cpu"
    assert p.last_info["fallback"] is True
    assert "out of GPU memory" in p.last_info["fallback_reason"]
    assert "FALLBACK" in p.status_text
    assert p.result["n_samples_done"] == 300_000
    assert win.viewport.robot_view.workspace_points_actor is not None


def test_item_5d_force_cpu_env(win, monkeypatch):
    """CPU-only path: no GPU needed. Env override wins over the 'cuda' preference."""
    _load(win)
    monkeypatch.setenv(be.ENV_FORCE_CPU, "1")
    p = win.workspace_panel
    p._backend.setCurrentText("cuda")
    p._samples.setValue(20_000)
    p.run()
    _pump(win)
    assert p.last_info["backend"] == "cpu"
    assert p.result["n_samples_done"] == 20_000


# ---------------------------------------------------------------------------
# Item 6: FK batch 1e6 fp32 within the Decisions 12 tolerances
# ---------------------------------------------------------------------------

def test_item_6_fk_batch_1e6_fp32(win):
    _gpu_or_skip()
    model = _load(win)
    gbk = get_backend("cuda")
    lo, hi = dof_limits(model)
    rng = np.random.default_rng(42)
    q = rng.uniform(lo, hi, size=(N_WS, model.dof()))
    g = gbk.fk_batch(model, q, precision="fp32")
    assert g["info"]["backend"] == "cuda" and g["info"]["precision"] == "fp32"
    ref = cb.fk_batch(cb.compile(model), q.astype(np.float32).astype(np.float64))
    n, tol_pos, tol_rot = _tolerances(model)
    eps = 1.19e-7
    dp = np.abs(g["position"] - ref["position"])
    dr = _rot_angle(g["rotation"], ref["rotation"])
    RtR = np.einsum("nba,nbc->nac", g["rotation"], g["rotation"])
    ortho = np.sqrt(((RtR - np.eye(3)) ** 2).sum(axis=(1, 2)))
    print(f"\n[fk 1e6] n={n} pos max {dp.max():.3e} (tol {tol_pos:.3e}); "
          f"rot max {dr.max():.3e} (tol {tol_rot:.3e}); ortho max {ortho.max():.3e} "
          f"(tol {4 * n * eps:.3e})")
    assert dp.max() <= tol_pos
    assert dr.max() <= tol_rot
    assert np.sqrt((dp ** 2).mean()) <= tol_pos / 10
    assert np.sqrt((dr ** 2).mean()) <= tol_rot / 10
    assert ortho.max() <= 4 * n * eps


# ---------------------------------------------------------------------------
# Item 7: benchmark JSON with version fields
# ---------------------------------------------------------------------------

def test_item_7_benchmark_json_written(win, tmp_path):
    model = _load(win)
    perf = win.performance_panel
    perf.bench_samples = 50_000
    perf.set_model(model)
    perf.run_benchmark()
    _pump(win)
    res = perf.last_result
    assert res is not None, perf._bench_status.text()
    out = tmp_path / "bench" / "benchmark.json"
    perf.save_benchmark(str(out))
    data = json.loads(out.read_text())
    for k in BENCH_KEYS:
        assert k in data
    for k in VERSION_KEYS:
        assert k in data["versions"]
    assert data["versions"]["numpy"] and data["versions"]["cpu_threads"] >= 1
    assert data["n"] == 50_000
    for k in ("fk", "workspace", "ik"):
        assert data[k]["cpu"]["samples_per_s"] > 0
    if data["backend"] == "cuda":
        assert data["versions"]["gpu"] and data["versions"]["sm"]
        assert data["versions"]["driver_version"]
        assert data[k]["backend"] is not None


def test_item_7_benchmark_function_cpu_versions(tmp_path):
    from .robots import fk_test_robots
    res = run_benchmark(CpuBackend("t"), fk_test_robots()["six_dof_fixture"], n=5_000, n_ik=200)
    p = write_benchmark(res, tmp_path / "b.json")
    data = json.loads(p.read_text())
    for k in VERSION_KEYS:
        assert k in data["versions"]
