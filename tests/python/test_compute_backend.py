"""R5.5: Python compute backend (CPU / CUDA), automatic fallback, budget, settings.

GPU parts skip with a clear message when no CUDA device is present. The fallback logic
is also tested without a GPU through a fake `gpu_bridge` object.
"""
import logging
import sys

import numpy as np
import pytest
from PySide6.QtCore import QSettings

import core_bridge as cb

from app.compute import backend as be
from app.compute import budget
from app.compute.backend import CpuBackend, GpuBackend, get_backend
from app.kin_utils import dof_limits
from app.settings import AppSettings

from .numpy_ref import fk_numpy, spec_from_model
from .robots import fk_test_robots, random_q, random_robot

EPS32 = 1.19e-7
TEST_ORG = "RoboticsAnalyzer_test_r55"


@pytest.fixture(autouse=True)
def isolated_settings(monkeypatch):
    """AppSettings (also inside MainWindow) writes to a throw-away organisation."""
    monkeypatch.setattr(AppSettings, "_ORGANIZATION", TEST_ORG)
    monkeypatch.setattr(AppSettings, "_APPLICATION", TEST_ORG)
    monkeypatch.delenv(be.ENV_FORCE_CPU, raising=False)
    s = QSettings(TEST_ORG, TEST_ORG)
    s.clear()
    s.sync()
    yield
    s.clear()
    s.sync()


@pytest.fixture
def robot():
    return fk_test_robots()["dh_6r"]


def _q(model, n, seed=0):
    rng = np.random.default_rng(seed)
    return np.array([random_q(model, rng) for _ in range(n)])


def _gpu_or_skip():
    gb = be._import_gpu_bridge()
    if gb is None:
        pytest.skip("gpu_bridge not importable")
    info = gb.device_info()
    if not info["available"]:
        pytest.skip(f"no CUDA device ({info['error']})")
    return gb


class FakeGb:
    """Stand-in for gpu_bridge: every compute call raises (a sticky CUDA error)."""

    def __init__(self, dead=False, exc=RuntimeError("CUDA error: illegal memory access")):
        self.dead = dead
        self.exc = exc
        self.calls = []

    def device_info(self):
        return dict(available=True, name="fake", error="")

    def mem_info(self):
        return dict(ok=True, free_bytes=4 << 30, total_bytes=8 << 30, error="")

    def is_dead(self):
        return self.dead

    def fk_batch(self, *a, **k):
        self.calls.append("fk_batch")
        raise self.exc

    def sample_workspace(self, *a, **k):
        self.calls.append("sample_workspace")
        raise self.exc

    def solve_ik_batch(self, *a, **k):
        self.calls.append("solve_ik_batch")
        raise AssertionError("GPU IK must not be used (R5.4 no-go)")


# ---------------------------------------------------------------------------
# selection
# ---------------------------------------------------------------------------


def test_cpu_preference():
    b = get_backend("cpu")
    assert b.name == "cpu" and b.reason
    d = b.device_info()
    assert d["backend"] == "cpu" and d["available"] is False


def test_auto_falls_back_when_gpu_bridge_import_fails(monkeypatch):
    monkeypatch.setitem(sys.modules, "gpu_bridge", None)   # import raises ImportError
    b = get_backend("auto")
    assert b.name == "cpu"
    assert "not importable" in b.reason
    b2 = get_backend("cuda")
    assert b2.name == "cpu" and "not importable" in b2.reason


def test_env_force_cpu_always_wins(monkeypatch):
    monkeypatch.setenv(be.ENV_FORCE_CPU, "1")
    for pref in ("auto", "cuda", "cpu", None):
        b = get_backend(pref)
        assert b.name == "cpu"
        assert "env" in b.reason and be.ENV_FORCE_CPU in b.reason


def test_env_zero_does_not_force(monkeypatch):
    monkeypatch.setenv(be.ENV_FORCE_CPU, "0")
    assert "env" not in get_backend("cpu").reason


def test_auto_without_device_or_dead_gpu(monkeypatch):
    class NoDev(FakeGb):
        def device_info(self):
            return dict(available=False, error="no device")

    monkeypatch.setattr(be, "_import_gpu_bridge", lambda: NoDev())
    b = get_backend("auto")
    assert b.name == "cpu" and "no device" in b.reason
    monkeypatch.setattr(be, "_import_gpu_bridge", lambda: FakeGb(dead=True))
    b = get_backend("auto")
    assert b.name == "cpu" and "dead" in b.reason


def test_auto_picks_gpu_when_available(monkeypatch):
    monkeypatch.setattr(be, "_import_gpu_bridge", lambda: FakeGb())
    b = get_backend("auto")
    assert b.name == "cuda" and b.reason
    assert b.device_info()["backend"] == "cuda"


def test_preference_from_settings():
    s = AppSettings()
    s.set_compute_backend("cpu")
    assert get_backend(None, s).name == "cpu"
    s.set(be.KEY_BACKEND, "bogus")
    assert get_backend(None, s).name in ("cpu", "cuda")     # unknown value -> auto


def test_settings_persist():
    s = AppSettings()
    assert s.compute_backend() == "auto" and s.vram_budget_mb() == 6144
    s.set_compute_backend("cpu")
    s.set_vram_budget_mb(2048)
    s.sync()
    s2 = AppSettings()
    assert s2.compute_backend() == "cpu"
    assert s2.vram_budget_mb() == 2048
    assert s2.get("compute/backend") == "cpu"
    assert int(s2.get("compute/vram_budget_mb")) == 2048


# ---------------------------------------------------------------------------
# budget
# ---------------------------------------------------------------------------


def test_budget_default_and_clamp():
    s = AppSettings()
    assert budget.vram_budget_mb(s) == 6144
    assert budget.vram_budget_bytes(s) == 6144 * 1024 * 1024
    s.set(budget.KEY_VRAM_BUDGET_MB, 99999)
    assert budget.vram_budget_mb(s) == 7168
    s.set(budget.KEY_VRAM_BUDGET_MB, 1)
    assert budget.vram_budget_mb(s) == 256
    s.set(budget.KEY_VRAM_BUDGET_MB, "junk")
    assert budget.vram_budget_mb(s) == 6144
    assert budget.set_vram_budget_mb(s, 100000) == 7168


def test_max_batch_cpu_is_fixed(monkeypatch):
    monkeypatch.setattr(be, "_import_gpu_bridge", lambda: None)
    assert budget.max_batch(100) == 1_000_000


def test_max_batch_uses_live_free_memory(monkeypatch):
    s = AppSettings()
    monkeypatch.setattr(be, "_import_gpu_bridge", lambda: FakeGb())
    n = budget.max_batch(100, s)
    expect = cb.chunk_size(4 << 30, budget.vram_budget_bytes(s), 0, 100)
    assert n == expect and n >= 65536
    s.set_vram_budget_mb(256)
    assert budget.max_batch(100, s) in (0, 65536) or budget.max_batch(100, s) < n


def test_max_batch_real_gpu():
    _gpu_or_skip()
    s = AppSettings()
    n = budget.max_batch(100, s)
    assert n == 0 or 65536 <= n <= 4_194_304


# ---------------------------------------------------------------------------
# CPU backend results
# ---------------------------------------------------------------------------


def test_cpu_fk_batch_matches_numpy_and_info(robot):
    q = _q(robot, 300)
    r = CpuBackend().fk_batch(robot, q, "fp32", "world")
    ref = cb.fk_batch(cb.compile(robot), q)
    assert np.array_equal(r["position"], ref["position"])
    p0 = fk_numpy(spec_from_model(robot), q[7])[1][:3, 3]
    assert np.allclose(r["position"][7], p0, atol=1e-9)
    info = r["info"]
    assert info["backend"] == "cpu" and info["precision"] == "fp64"
    assert info["fallback"] is False and info["fallback_reason"] == ""


def test_cpu_fk_bad_precision_and_arrays(robot):
    with pytest.raises(ValueError):
        CpuBackend().fk_batch(robot, _q(robot, 4), "fp16")
    with pytest.raises(ValueError):
        CpuBackend().fk_batch(cb.compile_arrays(robot), _q(robot, 4))


def test_cpu_workspace_info_keys(robot):
    opt = dict(n_samples=20_000, voxel_size=0.1, want_counts=False)
    r = CpuBackend().sample_workspace(robot, opt)
    assert r["info"]["backend"] == "cpu" and r["info"]["fallback"] is False
    assert r["n_samples_done"] == 20_000
    lo, hi = dof_limits(robot)
    ref = cb.sample_workspace(cb.compile(robot), lo, hi, opt)
    assert np.array_equal(np.asarray(r["occupied"]), np.asarray(ref["occupied"]))


def test_solve_ik_batch_is_cpu_and_equals_core(robot):
    rng = np.random.default_rng(1)
    qt = _q(robot, 8, seed=3)
    spec = spec_from_model(robot)
    targets = np.array([fk_numpy(spec, q)[1] for q in qt])
    q0 = qt + rng.uniform(-0.2, 0.2, qt.shape)
    lo, hi = dof_limits(robot)
    ref = cb.solve_ik_batch(cb.compile(robot), lo, hi, targets, q0)
    r = CpuBackend().solve_ik_batch(robot, targets, q0)
    assert np.array_equal(r["q"], ref["q"])
    assert r["info"]["backend"] == "cpu"
    # GpuBackend must route to the CPU (GPU IK is no-go) and never call gpu_bridge IK
    fake = FakeGb()
    g = GpuBackend(fake).solve_ik_batch(robot, targets, q0)
    assert np.array_equal(g["q"], ref["q"])
    assert "solve_ik_batch" not in fake.calls
    assert g["info"]["backend"] == "cpu" and g["info"]["fallback"] is False
    assert be.USE_GPU_IK is False


def test_estimate_reach_positive(robot):
    assert be.estimate_reach(robot) > 0
    assert be.estimate_reach(cb.compile_arrays(robot)) == pytest.approx(be.estimate_reach(robot))


# ---------------------------------------------------------------------------
# fallback (no GPU needed: fake gpu_bridge)
# ---------------------------------------------------------------------------


def test_fake_sticky_error_falls_back_fk(robot, caplog):
    q = _q(robot, 500)
    g = GpuBackend(FakeGb())
    with caplog.at_level(logging.WARNING, logger="robotics_analyzer"):
        r = g.fk_batch(robot, q)
    assert r["info"]["fallback"] is True
    assert "illegal memory access" in r["info"]["fallback_reason"]
    assert r["info"]["backend"] == "cpu"
    ref = cb.fk_batch(cb.compile(robot), q)
    assert np.array_equal(r["position"], ref["position"])
    assert any("re-running on the CPU" in m for m in caplog.messages)


def test_fake_sticky_error_falls_back_workspace(robot):
    opt = dict(n_samples=30_000, voxel_size=0.1)
    r = GpuBackend(FakeGb()).sample_workspace(robot, opt)
    assert r["info"]["fallback"] is True and r["info"]["backend"] == "cpu"
    ref = CpuBackend().sample_workspace(robot, opt)
    assert np.array_equal(np.asarray(r["counts"]), np.asarray(ref["counts"]))


def test_dead_gpu_goes_straight_to_cpu(robot):
    fake = FakeGb(dead=True)
    r = GpuBackend(fake).fk_batch(robot, _q(robot, 10))
    assert r["info"]["fallback"] is True and "dead" in r["info"]["fallback_reason"]
    assert fake.calls == []          # no retry on a dead GPU


def test_value_error_is_not_a_fallback(robot):
    g = GpuBackend(FakeGb(exc=ValueError("bad input")))
    with pytest.raises(ValueError, match="bad input"):
        g.fk_batch(robot, _q(robot, 10))


# ---------------------------------------------------------------------------
# real GPU (skipped without a device)
# ---------------------------------------------------------------------------


def test_gpu_vs_cpu_fk_batch(robot):
    _gpu_or_skip()
    b = get_backend("cuda")
    assert b.name == "cuda"
    n = 20_000
    q = _q(robot, n, seed=5)
    g = b.fk_batch(robot, q, "fp32")
    assert g["info"]["backend"] == "cuda" and g["info"]["fallback"] is False
    qf = q.astype(np.float32).astype(np.float64)       # CPU reference sees float q
    c = CpuBackend().fk_batch(robot, qf, "fp64")
    nj = len(robot.joints)
    reach = be.estimate_reach(robot)
    tol_pos = 8 * nj * EPS32 * reach
    assert np.abs(g["position"] - c["position"]).max() <= tol_pos
    assert np.abs(g["rotation"] - c["rotation"]).max() <= 8 * nj * EPS32 * 3


def test_gpu_vs_cpu_workspace(robot):
    _gpu_or_skip()
    opt = dict(n_samples=200_000, voxel_size=0.1, want_counts=False, want_points=True,
               max_points=1000)
    g = get_backend("cuda").sample_workspace(robot, opt)
    c = CpuBackend().sample_workspace(robot, opt)
    assert g["info"]["backend"] == "cuda"
    assert g["n_samples_done"] == c["n_samples_done"] == 200_000
    assert g["grid_dims"] == c["grid_dims"]
    vg = {tuple(v) for v in np.asarray(g["occupied_voxels"]).tolist()}
    vc = {tuple(v) for v in np.asarray(c["occupied_voxels"]).tolist()}
    assert len(vg ^ vc) <= 0.001 * len(vc) + 2
    assert np.abs(np.asarray(g["points"]) - np.asarray(c["points"])).max() < 1e-3


def test_gpu_fake_oom_sets_fallback_flag(robot):
    gb = _gpu_or_skip()
    b = get_backend("cuda")
    q = _q(robot, 400_000, seed=2)
    gb.release_buffers()
    gb.set_fake_oom(50)
    try:
        r = b.fk_batch(robot, q, "fp32")
    finally:
        gb.set_fake_oom(0)
    assert r["info"]["fallback"] is True
    assert r["info"]["backend"] == "cpu"
    assert "memory" in r["info"]["fallback_reason"]
    ref = cb.fk_batch(cb.compile(robot), q)
    assert np.abs(r["position"] - ref["position"]).max() <= 1e-12
    assert not gb.is_dead()
    gb.release_buffers()
    # workspace path too
    opt = dict(n_samples=300_000, voxel_size=0.1)
    gb.set_fake_oom(1)
    try:
        w = b.sample_workspace(robot, opt)
    finally:
        gb.set_fake_oom(0)
    assert w["info"]["fallback"] is True and w["info"]["backend"] == "cpu"
    assert "memory" in w["info"]["fallback_reason"]


def test_force_cpu_env_gives_cpu_results(monkeypatch, robot):
    monkeypatch.setenv(be.ENV_FORCE_CPU, "1")
    b = get_backend("cuda")
    r = b.sample_workspace(robot, dict(n_samples=10_000, voxel_size=0.1))
    assert r["info"]["backend"] == "cpu" and r["info"]["fallback"] is False
