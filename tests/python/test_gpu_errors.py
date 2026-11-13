"""A1.5: CUDA error paths (audit LOW C++/CUDA items).

Fault injection uses the debug functions of gpu_bridge (_debug_set_fault,
_debug_pool_handle_count, _debug_reset_pool). Skips without a CUDA device.
"""
import numpy as np
import pytest

import core_bridge as cb

from .robots import fk_test_robots
from .test_workspace import _gpu_or_skip, limits, run_cpu, run_gpu, voxel_set


@pytest.fixture(scope="module")
def gb():
    return _gpu_or_skip()


@pytest.fixture(autouse=True)
def _clean(gb):
    yield
    gb._debug_set_fault("")
    gb.set_fake_oom(0)


def test_ws_count_bits_oom_takes_host_path(gb):
    """Audit LOW ws_count_bits: OOM while counting -> host bitset_to_voxels, same voxels."""
    m = fk_test_robots()["dh_6r"]
    opt = dict(n_samples=300_000, voxel_size=0.1)
    ref = run_gpu(gb, m, opt)
    assert ref["info"]["gpu_compaction"] is True
    gb._debug_set_fault("ws_count_oom")
    r = run_gpu(gb, m, opt)
    gb._debug_set_fault("")
    assert r["info"]["backend"] == "cuda"
    assert r["info"]["gpu_compaction"] is False
    assert voxel_set(r) == voxel_set(ref)
    assert len(voxel_set(r) ^ voxel_set(run_cpu(m, opt))) <= 0.001 * len(voxel_set(ref))


def test_sample_q_oom_is_memory_error(gb):
    """Audit LOW sample_q_gpu: device OOM -> MemoryError (was an unmapped non-std exception)."""
    gb.set_fake_oom(1)
    with pytest.raises(MemoryError):
        gb.sample_q([-1.0, -1.0], [1.0, 1.0], dict(seed=1), 0, 1000)
    gb.set_fake_oom(0)
    assert gb.sample_q([-1.0, -1.0], [1.0, 1.0], dict(seed=1), 0, 1000).shape == (1000, 2)


def test_sample_q_over_budget_is_value_error(gb):
    """Audit LOW sample_q_gpu: n * n_dof * 4 above device memory -> ValueError."""
    with pytest.raises(ValueError):
        gb.sample_q([-1.0, -1.0], [1.0, 1.0], dict(seed=1), 0, 10**11)


def test_pool_init_failure_leaks_no_handles(gb):
    """Audit LOW ensure_pool: init failure destroys streams/events/buffers; retries do not grow."""
    gb._debug_reset_pool()
    assert gb._debug_pool_handle_count() == 0
    arr = cb.compile_arrays(fk_test_robots()["dh_6r"])
    q = np.zeros((1000, 6))
    gb._debug_set_fault("pool_init")
    for _ in range(2):
        with pytest.raises(RuntimeError):
            gb.fk_batch(arr, q)
        assert gb._debug_pool_handle_count() == 0
    gb._debug_set_fault("")
    r = gb.fk_batch(arr, q)
    assert r["info"]["backend"] == "cuda"
    n_ok = gb._debug_pool_handle_count()
    assert n_ok == 8            # 2 streams + 4 events + 2 device tables
    gb.fk_batch(arr, q)
    assert gb._debug_pool_handle_count() == n_ok


def test_unknown_fault_name_rejected(gb):
    with pytest.raises(ValueError):
        gb._debug_set_fault("nope")
