"""R5.2: GPU batch FK (gpu_bridge.fk_batch) vs the CPU double engine.

GPU tests skip with a clear message when no CUDA device is present.
fp32 tolerances: docs/gpu-precision-decision.md (R5 Decisions 12).
"""
import numpy as np
import pytest

import core_bridge as cb

from .robots import fk_test_robots, random_robot, sample_q

EPS32 = 1.19e-7
N_BIG = 1_000_000


def _gpu_or_skip():
    try:
        import gpu_bridge
    except ImportError as e:  # pragma: no cover
        pytest.skip(f"gpu_bridge not importable: {e}")
    info = gpu_bridge.device_info()
    if not info["available"]:
        pytest.skip(f"no CUDA device ({info['error']})")
    return gpu_bridge


@pytest.fixture(scope="module")
def gb():
    return _gpu_or_skip()


@pytest.fixture(scope="module")
def robots():
    return fk_test_robots()


def _tolerances(model):
    """(n, tol_pos, tol_rot) from the active path (Decisions 12)."""
    arr = cb.compile_arrays(model)
    chain = set(cb.active_chain(model))
    n = len(chain)
    reach = float(np.linalg.norm(arr["tool"][:3, 3]))
    for k, ji in enumerate(arr["joint_index"]):
        if int(ji) in chain:
            reach += float(np.linalg.norm(arr["pre"][k][:3, 3]) + np.linalg.norm(arr["post"][k][:3, 3]))
    return n, 8 * n * EPS32 * reach, 8 * n * EPS32


def _rot_angle(Ra, Rb):
    fro = np.sqrt(((Ra - Rb) ** 2).sum(axis=(1, 2)))
    return 2.0 * np.arcsin(np.minimum(1.0, fro / (2.0 * np.sqrt(2.0))))


# ---------------------------------------------------------------------------
# Accuracy at 1e6 samples
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("name", ["six_dof_fixture", "dh_6r", "origin_axis_12r", "tree_mimic_fixed"])
def test_fp32_within_tolerance_1e6(gb, robots, name):
    m = robots[name]
    cm = cb.compile(m)
    arr = cb.compile_arrays(m)
    q = sample_q(m, N_BIG, seed=42)
    g = gb.fk_batch(arr, q, precision="fp32")
    assert g["info"]["backend"] == "cuda" and g["info"]["precision"] == "fp32"
    ref = cb.fk_batch(cm, q.astype(np.float32).astype(np.float64))  # (double)(float)q
    n, tol_pos, tol_rot = _tolerances(m)
    dp = np.abs(g["position"] - ref["position"])
    dr = _rot_angle(g["rotation"], ref["rotation"])
    RtR = np.einsum("nba,nbc->nac", g["rotation"], g["rotation"])
    ortho = np.sqrt(((RtR - np.eye(3)) ** 2).sum(axis=(1, 2)))
    print(f"\n[{name}] n={n} pos max {dp.max():.3e} (tol {tol_pos:.3e}) rms {np.sqrt((dp**2).mean()):.3e}; "
          f"rot max {dr.max():.3e} (tol {tol_rot:.3e}) rms {np.sqrt((dr**2).mean()):.3e}; "
          f"ortho max {ortho.max():.3e} (tol {4 * n * EPS32:.3e})")
    assert dp.max() <= tol_pos
    assert dr.max() <= tol_rot
    assert np.sqrt((dp ** 2).mean()) <= tol_pos / 10
    assert np.sqrt((dr ** 2).mean()) <= tol_rot / 10
    assert ortho.max() <= 4 * n * EPS32


@pytest.mark.parametrize("name", ["six_dof_fixture", "dh_6r", "origin_axis_12r", "tree_mimic_fixed"])
def test_fp64_matches_cpu_1e6(gb, robots, name):
    m = robots[name]
    cm = cb.compile(m)
    arr = cb.compile_arrays(m)
    q = sample_q(m, N_BIG, seed=42)
    g = gb.fk_batch(arr, q, precision="fp64")
    assert g["info"]["backend"] == "cuda" and g["info"]["precision"] == "fp64"
    ref = cb.fk_batch(cm, q)
    dp = np.abs(g["position"] - ref["position"]).max()
    dr = np.abs(g["rotation"] - ref["rotation"]).max()
    print(f"\n[{name}] fp64 pos {dp:.3e} rot {dr:.3e}")
    assert dp <= 1e-12
    assert dr <= 1e-12


def test_base_frame_matches_cpu(gb, robots):
    m = robots["origin_axis_12r"]
    cm = cb.compile(m)
    arr = cb.compile_arrays(m)
    q = sample_q(m, 5000, seed=9)
    g = gb.fk_batch(arr, q, precision="fp64", frame="base")
    ref = cb.fk_batch(cm, q, frame="base")
    assert np.abs(g["position"] - ref["position"]).max() <= 1e-12
    assert np.abs(g["rotation"] - ref["rotation"]).max() <= 1e-12


def test_default_precision_follows_decision(gb, robots):
    m = robots["dh_6r"]
    arr = cb.compile_arrays(m)
    assert gb.DEFAULT_FK_PRECISION in ("fp32", "fp64")
    g = gb.fk_batch(arr, sample_q(m, 1000))
    assert g["info"]["precision"] == gb.DEFAULT_FK_PRECISION


# ---------------------------------------------------------------------------
# Edge cases
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("n", [1, 255, 256, 257, 1000])
@pytest.mark.parametrize("precision", ["fp32", "fp64"])
def test_small_and_odd_sizes(gb, robots, n, precision):
    m = robots["tree_mimic_fixed"]
    cm = cb.compile(m)
    arr = cb.compile_arrays(m)
    q = sample_q(m, n, seed=n)
    g = gb.fk_batch(arr, q, precision=precision)
    ref = cb.fk_batch(cm, q.astype(np.float32).astype(np.float64) if precision == "fp32" else q)
    tol = 1e-12 if precision == "fp64" else 1e-4
    assert g["position"].shape == (n, 3) and g["rotation"].shape == (n, 3, 3)
    assert np.abs(g["position"] - ref["position"]).max() <= tol
    assert np.abs(g["rotation"] - ref["rotation"]).max() <= tol


def test_empty_array(gb, robots):
    m = robots["dh_6r"]
    arr = cb.compile_arrays(m)
    g = gb.fk_batch(arr, np.zeros((0, 6)))
    assert g["position"].shape == (0, 3)
    assert g["rotation"].shape == (0, 3, 3)
    assert g["info"]["chunks"] == 0


def test_wrong_q_width_raises(gb, robots):
    arr = cb.compile_arrays(robots["dh_6r"])
    with pytest.raises(ValueError):
        gb.fk_batch(arr, np.zeros((4, 5)))
    with pytest.raises(ValueError):
        gb.fk_batch(arr, np.zeros(6))


def test_bad_options_raise(gb, robots):
    arr = cb.compile_arrays(robots["dh_6r"])
    q = np.zeros((4, 6))
    with pytest.raises(ValueError):
        gb.fk_batch(arr, q, precision="fp16")
    with pytest.raises(ValueError):
        gb.fk_batch(arr, q, frame="tool")
    with pytest.raises(ValueError):
        gb.fk_batch(arr, q, chunk=1000)  # not a multiple of 256


def test_more_than_64_joints_raises(gb):
    m = random_robot(77, 65)
    arr = cb.compile_arrays(m)
    with pytest.raises(ValueError):
        gb.fk_batch(arr, np.zeros((3, 65)))


def test_63_joints_ok(gb):
    """63 chain joints = 64 links: the largest supported model."""
    m = random_robot(78, 63)
    cm = cb.compile(m)
    arr = cb.compile_arrays(m)
    q = sample_q(m, 2000, seed=1)
    g = gb.fk_batch(arr, q, precision="fp64")
    ref = cb.fk_batch(cm, q)
    assert np.abs(g["position"] - ref["position"]).max() <= 1e-10


def test_zero_dof_robot(gb):
    m = random_robot(21, 3)
    for j in m.joints:
        j.type = cb.JointType.Fixed
    arr = cb.compile_arrays(m)
    cm = cb.compile(m)
    g = gb.fk_batch(arr, np.zeros((300, 0)), precision="fp64")
    ref = cb.fk_batch(cm, np.zeros((300, 0)))
    assert np.abs(g["position"] - ref["position"]).max() <= 1e-12
    assert np.abs(g["rotation"] - ref["rotation"]).max() <= 1e-12


# ---------------------------------------------------------------------------
# Chunking, fake OOM
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("precision", ["fp32", "fp64"])
def test_chunk_size_does_not_change_result(gb, robots, precision):
    m = robots["origin_axis_12r"]
    arr = cb.compile_arrays(m)
    q = sample_q(m, 200_003, seed=7)  # not a multiple of any chunk
    base = gb.fk_batch(arr, q, precision=precision)
    small = gb.fk_batch(arr, q, precision=precision, chunk=65536)
    tiny = gb.fk_batch(arr, q, precision=precision, chunk=4096)
    assert small["info"]["chunks"] == 4  # ceil(200003 / 65536)
    assert tiny["info"]["chunks"] == 49
    assert np.array_equal(base["position"], small["position"])
    assert np.array_equal(base["rotation"], small["rotation"])
    assert np.array_equal(base["position"], tiny["position"])
    assert np.array_equal(base["rotation"], tiny["rotation"])


def test_fake_oom_halves_chunk_then_succeeds(gb, robots):
    m = robots["dh_6r"]
    arr = cb.compile_arrays(m)
    q = sample_q(m, 400_000, seed=2)
    ref = gb.fk_batch(arr, q, precision="fp32")
    gb.release_buffers()
    gb.set_fake_oom(1)
    try:
        g = gb.fk_batch(arr, q, precision="fp32")
    finally:
        gb.set_fake_oom(0)
    assert g["info"]["backend"] == "cuda"
    assert g["info"]["chunk"] < ref["info"]["chunk"]
    assert np.array_equal(g["position"], ref["position"])


def test_fake_oom_below_minimum_falls_back_to_cpu(gb, robots):
    m = robots["dh_6r"]
    cm = cb.compile(m)
    arr = cb.compile_arrays(m)
    q = sample_q(m, 400_000, seed=2)
    gb.release_buffers()
    gb.set_fake_oom(50)
    try:
        g = gb.fk_batch(arr, q, precision="fp32")
    finally:
        gb.set_fake_oom(0)
    assert g["info"]["backend"] == "cpu"
    assert g["info"]["precision"] == "fp64"
    assert "memory" in g["info"]["fallback_reason"]
    ref = cb.fk_batch(cm, q)
    assert np.abs(g["position"] - ref["position"]).max() <= 1e-12
    assert not gb.is_dead()
    # GPU still usable afterwards
    assert gb.fk_batch(arr, q[:1000])["info"]["backend"] == "cuda"
