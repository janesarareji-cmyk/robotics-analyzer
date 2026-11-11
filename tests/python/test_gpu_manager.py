"""R5.1: GPU manager, chunk_size rule, gpu_bridge skeleton."""
import pytest

import core_bridge as cb

MiB = 1024 * 1024
GiB = 1024 * MiB


def _gpu():
    try:
        import gpu_bridge
    except ImportError as e:  # pragma: no cover
        pytest.skip(f"gpu_bridge not importable: {e}")
    return gpu_bridge


def _device_or_skip():
    g = _gpu()
    info = g.device_info()
    if not info["available"]:
        pytest.skip(f"no CUDA device ({info['error']})")
    return g, info


# ---- chunk_size (core_bridge, no GPU needed) ----

def test_chunk_size_plenty_is_max():
    assert cb.chunk_size(8 * GiB, 8 * GiB, 0, 100) == 4194304


def test_chunk_size_tiny_is_zero():
    assert cb.chunk_size(100 * MiB, 8 * GiB, 0, 100) == 0


def test_chunk_size_multiple_of_256_and_formula():
    c = cb.chunk_size(2 * GiB, 8 * GiB, 0, 1000)
    expect = int((0.6 * 2 * GiB - 256 * MiB) / 2000)
    expect -= expect % 256
    assert c == expect and c % 256 == 0


def test_chunk_size_persistent_and_headroom():
    a = cb.chunk_size(2 * GiB, 8 * GiB, 0, 1000)
    assert cb.chunk_size(2 * GiB, 8 * GiB, 500 * MiB, 1000) < a
    assert cb.chunk_size(2 * GiB, 8 * GiB, 0, 1000, headroom=0.5) < a
    assert cb.chunk_size(2 * GiB, 8 * GiB, 2 * GiB, 1000) == 0


def test_chunk_size_user_budget_caps():
    assert cb.chunk_size(8 * GiB, 64 * MiB, 0, 1000) == 0


@pytest.mark.parametrize("args", [
    (GiB, GiB, 0, 0), (GiB, GiB, 0, -1), (-1, GiB, 0, 10), (GiB, 0, 0, 10),
    (GiB, GiB, -1, 10),
])
def test_chunk_size_invalid(args):
    with pytest.raises(ValueError):
        cb.chunk_size(*args)


def test_chunk_size_invalid_headroom():
    with pytest.raises(ValueError):
        cb.chunk_size(GiB, GiB, 0, 10, headroom=0.0)


# ---- gpu_bridge ----

def test_gpu_bridge_imports_and_reports():
    g = _gpu()
    info = g.device_info()
    for k in ("available", "name", "sm_major", "sm_minor", "total_bytes",
              "free_bytes", "driver_version", "runtime_version", "error"):
        assert k in info
    assert isinstance(g.is_dead(), bool)


def test_device_info_is_rtx_5060_sm120():
    g, info = _device_or_skip()
    assert "5060" in info["name"]
    assert (info["sm_major"], info["sm_minor"]) == (12, 0)
    assert 7 * GiB < info["total_bytes"] < 9 * GiB
    assert info["driver_version"] > 0 and info["runtime_version"] > 0


def test_mem_info_free_le_total():
    g, _ = _device_or_skip()
    m = g.mem_info()
    assert m["ok"]
    assert 0 < m["free_bytes"] <= m["total_bytes"]


def test_fake_oom_hook_toggles():
    g = _gpu()
    g.set_fake_oom(2)
    g.set_fake_oom(0)
    assert g.is_dead() is False
