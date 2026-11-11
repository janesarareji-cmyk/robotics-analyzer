"""R5.3: workspace sampling (CPU reference core_bridge.sample_workspace and GPU
gpu_bridge.sample_workspace).

GPU tests skip with a clear message when no CUDA device is present.

Philox4x32-10 is checked two ways: the published Random123 known-answer vectors
(they were also reproduced by an independent pure-Python implementation written
from the algorithm), and that pure-Python implementation against the C++ one on
many random counters and keys.

Note on the annulus test: counting occupied voxels over-estimates the area of a
region by about (perimeter * 2h/pi) / area (voxels that merely touch the
boundary); for the 2R annulus at h = 0.02 that is about 2.4 %, not below 1 %.
The test therefore compares with the exact number of voxels that intersect the
annulus (computed independently with numpy) within 1 %, and bounds the raw area
error to 3.5 %.
"""
import time

import numpy as np
import pytest

import core_bridge as cb

from .robots import fk_test_robots, pose, random_robot

EPS32 = 1.19e-7

# --------------------------------------------------------------------------
# independent pure-Python Philox4x32-10 (from the algorithm, no C++ shared)
# --------------------------------------------------------------------------
_M0, _M1 = 0xD2511F53, 0xCD9E8D57
_W0, _W1 = 0x9E3779B9, 0xBB67AE85
_MASK = 0xFFFFFFFF


def philox_py(ctr, key):
    c = list(ctr)
    k = list(key)
    for _ in range(10):
        p0 = _M0 * c[0]
        p1 = _M1 * c[2]
        c = [(p1 >> 32) ^ c[1] ^ k[0], p1 & _MASK, (p0 >> 32) ^ c[3] ^ k[1], p0 & _MASK]
        k = [(k[0] + _W0) & _MASK, (k[1] + _W1) & _MASK]
    return c


def philox_np(ctr, key):
    """Vectorised version: ctr (n,4), key (n,2) uint32 -> (n,4) uint32."""
    c = [ctr[:, i].astype(np.uint64) for i in range(4)]
    k = [key[:, i].astype(np.uint64) for i in range(2)]
    m = np.uint64(_MASK)
    for _ in range(10):
        p0 = np.uint64(_M0) * c[0]
        p1 = np.uint64(_M1) * c[2]
        c = [(p1 >> np.uint64(32)) ^ c[1] ^ k[0], p1 & m, (p0 >> np.uint64(32)) ^ c[3] ^ k[1], p0 & m]
        k = [(k[0] + np.uint64(_W0)) & m, (k[1] + np.uint64(_W1)) & m]
    return np.stack(c, axis=1).astype(np.uint32)


def q_numpy(lo, hi, seed, start, n):
    """Float32 q of the random sampler computed with numpy (rounding: double then float)."""
    lo32 = np.asarray(lo, dtype=np.float32)
    rng32 = np.asarray(hi, dtype=np.float32) - lo32
    nd = len(lo)
    idx = np.arange(start, start + n, dtype=np.uint64)
    q = np.empty((n, nd), dtype=np.float32)
    key = np.tile(np.array([seed & _MASK, (seed >> 32) & _MASK], dtype=np.uint32), (n, 1))
    for blk in range((nd + 3) // 4):
        ctr = np.stack([(idx & np.uint64(_MASK)).astype(np.uint32),
                        (idx >> np.uint64(32)).astype(np.uint32),
                        np.full(n, blk, dtype=np.uint32), np.zeros(n, dtype=np.uint32)], axis=1)
        out = philox_np(ctr, key)
        for lane in range(4):
            d = blk * 4 + lane
            if d < nd:
                u = (out[:, lane] >> np.uint32(8)).astype(np.float32) * np.float32(2.0 ** -24)
                q[:, d] = (u.astype(np.float64) * float(rng32[d]) + float(lo32[d])).astype(np.float32)
    return q


# --------------------------------------------------------------------------
# helpers
# --------------------------------------------------------------------------

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


def planar_2r(l1=1.0, l2=0.5):
    """Planar 2R: joints about z, link lengths l1, l2 (tool offset), limits +-pi, identity base."""
    m = cb.RobotModel()
    for i in range(3):
        lk = cb.Link()
        lk.name = f"l{i}"
        m.links.append(lk)
    for i, x in enumerate([0.0, l1]):
        j = cb.Joint()
        j.name = f"j{i}"
        j.parent_link, j.child_link = i, i + 1
        j.type = cb.JointType.Revolute
        j.kinematics = cb.JointKinematics.OriginAxis
        j.origin = pose([x, 0, 0], [0, 0, 0])
        j.axis = [0.0, 0.0, 1.0]
        lim = cb.JointLimits()
        lim.lower, lim.upper = -np.pi, np.pi
        lim.vel, lim.effort = 1.0, 10.0
        j.limits = lim
        m.joints.append(j)
    m.tool = pose([l2, 0, 0], [0, 0, 0])
    m.tcp_link = 2
    return m


def limits(m):
    from app.kin_utils import dof_limits
    return dof_limits(m)


def run_cpu(m, opt, **kw):
    lo, hi = limits(m)
    return cb.sample_workspace(cb.compile(m), lo, hi, opt, **kw)


def run_gpu(gb, m, opt, **kw):
    lo, hi = limits(m)
    return gb.sample_workspace(cb.compile_arrays(m), lo, hi, opt, **kw)


def bits(r):
    return np.asarray(r["occupied"])


def voxel_set(r):
    return set(map(tuple, np.asarray(r["occupied_voxels"]).tolist()))


def annulus_voxel_count(h, bounds_lo, dims, r_in=0.5, r_out=1.5):
    """Exact number of grid cells (one z layer) that intersect the annulus (independent oracle)."""
    ix = np.arange(dims[0])
    iy = np.arange(dims[1])
    x0 = bounds_lo[0] + ix * h
    y0 = bounds_lo[1] + iy * h
    X0, Y0 = np.meshgrid(x0, y0, indexing="ij")
    X1, Y1 = X0 + h, Y0 + h
    nx = np.where((X0 <= 0) & (X1 >= 0), 0.0, np.minimum(np.abs(X0), np.abs(X1)))
    ny = np.where((Y0 <= 0) & (Y1 >= 0), 0.0, np.minimum(np.abs(Y0), np.abs(Y1)))
    dmin = np.hypot(nx, ny)
    dmax = np.hypot(np.maximum(np.abs(X0), np.abs(X1)), np.maximum(np.abs(Y0), np.abs(Y1)))
    return int(((dmin <= r_out) & (dmax >= r_in)).sum())


ANNULUS_OPT = dict(n_samples=2_000_000, voxel_size=0.02,
                   bounds_lo=[-1.52, -1.52, -0.01], bounds_hi=[1.52, 1.52, 0.01])


# --------------------------------------------------------------------------
# Philox
# --------------------------------------------------------------------------

KAT = [
    ([0, 0, 0, 0], [0, 0], [0x6627E8D5, 0xE169C58D, 0xBC57AC4C, 0x9B00DBD8]),
    ([0xFFFFFFFF] * 4, [0xFFFFFFFF] * 2, [0x408F276D, 0x41C83B0E, 0xA20BC7C6, 0x6D5451FD]),
    ([0x243F6A88, 0x85A308D3, 0x13198A2E, 0x03707344], [0xA4093822, 0x299F31D0],
     [0xD16CFE09, 0x94FDCCEB, 0x5001E420, 0x24126EA1]),
]


@pytest.mark.parametrize("ctr,key,expected", KAT)
def test_philox_known_answers(ctr, key, expected):
    assert list(cb.philox4x32_10(ctr, key)) == expected
    assert philox_py(ctr, key) == expected  # the independent implementation agrees too


def test_philox_matches_pure_python_on_random_inputs():
    rng = np.random.default_rng(1)
    for _ in range(3000):
        ctr = [int(v) for v in rng.integers(0, 2 ** 32, 4, dtype=np.uint64)]
        key = [int(v) for v in rng.integers(0, 2 ** 32, 2, dtype=np.uint64)]
        assert list(cb.philox4x32_10(ctr, key)) == philox_py(ctr, key)


def test_sample_q_matches_independent_numpy_philox():
    m = random_robot(5, 7)
    lo, hi = limits(m)
    for seed, start in ((42, 0), (7, 123456), (2 ** 40 + 3, 2 ** 33 + 5)):
        q = cb.sample_q(lo, hi, dict(seed=seed), start, 500)
        ref = q_numpy(lo, hi, seed, start, 500)
        ulp = np.spacing(np.abs(ref).astype(np.float32))
        assert np.all(np.abs(q - ref) <= ulp)
        rng32 = (np.asarray(hi, np.float32) - np.asarray(lo, np.float32))
        assert np.all(q >= np.asarray(lo, np.float32) - 1e-6)
        assert np.all(q <= np.asarray(hi, np.float32) + 1e-6)
        assert np.all(rng32 > 0)


def test_sample_q_independent_of_chunking_and_seed_matters():
    lo, hi = [-3.0] * 6, [3.0] * 6
    full = cb.sample_q(lo, hi, dict(seed=42), 0, 3000)
    parts = np.concatenate([cb.sample_q(lo, hi, dict(seed=42), s, n)
                            for s, n in ((0, 1000), (1000, 1), (1001, 1999))])
    assert np.array_equal(full, parts)
    other = cb.sample_q(lo, hi, dict(seed=43), 0, 3000)
    assert not np.array_equal(full, other)


def test_cpu_gpu_identical_q(gb):
    lo, hi = [-3.1, -2.0, -1.0, 0.0, 0.5, -3.14159, 1.0], [3.1, 2.0, 1.0, 0.5, 2.5, 3.14159, 1.0]
    for start, n in ((0, 4096), (2 ** 32 - 10, 100), (2 ** 33 + 77, 1000)):
        c = cb.sample_q(lo, hi, dict(seed=99), start, n)
        g = gb.sample_q(lo, hi, dict(seed=99), start, n)
        assert c.dtype == g.dtype == np.float32
        assert np.array_equal(c, g)  # bit identical
    div = dict(mode="grid", divisions=[3, 1, 4, 2, 5, 7, 2])
    c = cb.sample_q(lo, hi, div, 5, 800)
    g = gb.sample_q(lo, hi, div, 5, 800)
    assert np.array_equal(c, g)


# --------------------------------------------------------------------------
# 2R annulus
# --------------------------------------------------------------------------

def _check_annulus(r):
    h = 0.02
    dims = r["grid_dims"]
    assert dims[2] == 1
    nv = len(r["occupied_voxels"])
    exact_cover = annulus_voxel_count(h, [-1.52, -1.52], dims)
    area = nv * h * h
    raw_err = abs(area / (2 * np.pi) - 1)
    cover_err = abs(nv / exact_cover - 1)
    print(f"\n[annulus {r['info']['backend']}] voxels {nv}, area {area:.4f} vs 2pi {2 * np.pi:.4f} "
          f"(raw err {raw_err * 100:.2f} %), exact cover {exact_cover} (err {cover_err * 100:.3f} %)")
    assert cover_err < 0.01
    assert raw_err < 0.035
    assert r["n_outside"] == 0
    assert int(np.asarray(r["counts"]).sum()) == ANNULUS_OPT["n_samples"]
    assert not r["cancelled"] and r["n_samples_done"] == ANNULUS_OPT["n_samples"]
    # every occupied voxel lies in the annulus band (distance of the cell to the origin)
    v = np.asarray(r["occupied_voxels"])
    cx = -1.52 + (v[:, 0] + 0.5) * h
    cy = -1.52 + (v[:, 1] + 0.5) * h
    dist = np.hypot(cx, cy)
    assert dist.min() > 0.5 - h and dist.max() < 1.5 + h


def test_planar_2r_annulus_cpu():
    _check_annulus(run_cpu(planar_2r(1.0, 0.5), ANNULUS_OPT))


def test_planar_2r_annulus_gpu(gb):
    r = run_gpu(gb, planar_2r(1.0, 0.5), ANNULUS_OPT)
    assert r["info"]["backend"] == "cuda" and r["info"]["gpu_compaction"]
    _check_annulus(r)


def test_planar_2r_gpu_vs_cpu_grid(gb):
    m = planar_2r(1.0, 0.5)
    rc = run_cpu(m, ANNULUS_OPT)
    rg = run_gpu(gb, m, ANNULUS_OPT)
    diff = voxel_set(rc) ^ voxel_set(rg)
    assert len(diff) <= 0.001 * len(voxel_set(rc))
    assert np.array_equal(np.asarray(rc["occupied_voxels"]).shape[1:], [3])


def test_default_bounds_use_reach():
    r = run_cpu(planar_2r(1.0, 0.5), dict(n_samples=1000, voxel_size=0.1))
    # reach 1.5, margin one voxel: [-1.6, 1.6]^3 -> 32 cells per axis
    assert tuple(r["grid_dims"]) in ((32, 32, 32), (33, 33, 33))
    assert np.allclose(r["origin"], [-1.6, -1.6, -1.6])
    assert r["n_outside"] == 0


# --------------------------------------------------------------------------
# GPU vs CPU on general robots, chunk independence, memory
# --------------------------------------------------------------------------

@pytest.mark.parametrize("name", ["six_dof_fixture", "origin_axis_12r", "tree_mimic_fixed"])
def test_gpu_vs_cpu_grid_general_robots(gb, name):
    m = fk_test_robots()[name]
    opt = dict(n_samples=1_000_000, voxel_size=0.05)
    rc = run_cpu(m, opt)
    rg = run_gpu(gb, m, opt)
    assert rg["info"]["backend"] == "cuda"
    assert rc["grid_dims"] == rg["grid_dims"]
    sc, sg = voxel_set(rc), voxel_set(rg)
    frac = len(sc ^ sg) / max(1, len(sc))
    print(f"\n[{name}] occupied cpu {len(sc)} gpu {len(sg)}, differing fraction {frac:.2e}")
    assert frac <= 0.001
    assert rc["n_samples_done"] == rg["n_samples_done"] == opt["n_samples"]
    assert abs(rc["n_outside"] - rg["n_outside"]) <= 5
    cc, cg = np.asarray(rc["counts"]), np.asarray(rg["counts"])
    assert int(cc.sum()) + rc["n_outside"] == opt["n_samples"]
    assert int(cg.sum()) + rg["n_outside"] == opt["n_samples"]
    assert np.abs(cc.astype(np.int64) - cg).sum() <= 0.002 * opt["n_samples"]


def test_chunk_size_does_not_change_the_grid_gpu(gb):
    m = fk_test_robots()["origin_axis_12r"]
    base = dict(n_samples=700_000, voxel_size=0.08)
    ref = run_gpu(gb, m, base)
    for chunk in (65536, 100_352, 262_144):
        r = run_gpu(gb, m, dict(base, chunk=chunk))
        assert r["info"]["chunks"] > 1 and r["info"]["chunk"] == chunk
        assert np.array_equal(np.asarray(r["counts"]), np.asarray(ref["counts"]))
        assert np.array_equal(bits(r), bits(ref))
        assert r["n_outside"] == ref["n_outside"]
    print(f"\nchunk independence: identical counts for 3 forced chunk sizes; default chunks {ref['info']['chunks']}")


def test_chunk_size_does_not_change_the_grid_cpu():
    m = fk_test_robots()["origin_axis_12r"]
    base = dict(n_samples=300_000, voxel_size=0.08)
    ref = run_cpu(m, base)
    r = run_cpu(m, dict(base, chunk=65536))
    assert r["info"]["chunks"] > 1
    assert np.array_equal(np.asarray(r["counts"]), np.asarray(ref["counts"]))


def test_cpu_threads_do_not_change_the_grid():
    m = fk_test_robots()["dh_6r"]
    base = dict(n_samples=200_000, voxel_size=0.1)
    a = run_cpu(m, dict(base, n_threads=1))
    b = run_cpu(m, dict(base, n_threads=8))
    assert np.array_equal(np.asarray(a["counts"]), np.asarray(b["counts"]))


def _persistent_estimate(r):
    nv = int(np.prod(r["grid_dims"]))
    return (nv + 31) // 32 * 4 + nv * 4


def test_tiny_budget_forces_many_chunks_same_grid_gpu(gb):
    m = fk_test_robots()["origin_axis_12r"]
    base = dict(n_samples=2_000_000, voxel_size=0.08)
    ref = run_gpu(gb, m, base)
    budget = _persistent_estimate(ref) + 4096 + 600_000
    r = run_gpu(gb, m, dict(base, budget_bytes=budget))
    info = r["info"]
    print(f"\n[budget] budget {budget} peak {info['vram_peak_bytes']} chunks {info['chunks']} chunk {info['chunk']}")
    assert info["backend"] == "cuda"
    assert info["budget_bytes"] == budget
    assert info["vram_peak_bytes"] <= budget
    assert info["chunks"] >= 20
    assert np.array_equal(np.asarray(r["counts"]), np.asarray(ref["counts"]))
    assert np.array_equal(bits(r), bits(ref))


def test_budget_peak_with_points_gpu(gb):
    m = fk_test_robots()["origin_axis_12r"]
    base = dict(n_samples=1_000_000, voxel_size=0.1, want_points=True, max_points=300_000)
    ref = run_gpu(gb, m, base)
    budget = _persistent_estimate(ref) + 4096 + 2 * 12 * 65536 + 400_000
    r = run_gpu(gb, m, dict(base, budget_bytes=budget))
    assert r["info"]["vram_peak_bytes"] <= budget
    assert r["info"]["chunks"] >= 10
    assert np.array_equal(np.asarray(r["points"]), np.asarray(ref["points"]))
    assert np.array_equal(np.asarray(r["counts"]), np.asarray(ref["counts"]))


def test_budget_too_small_for_grid_is_value_error(gb):
    m = fk_test_robots()["origin_axis_12r"]
    with pytest.raises(ValueError, match="budget_bytes"):
        run_gpu(gb, m, dict(n_samples=1000, voxel_size=0.05, budget_bytes=1024))


def test_fake_oom_falls_back_to_cpu_with_same_grid(gb):
    m = fk_test_robots()["dh_6r"]
    opt = dict(n_samples=300_000, voxel_size=0.1)
    ref = run_gpu(gb, m, opt)
    gb.set_fake_oom(1)
    try:
        r = run_gpu(gb, m, opt)
    finally:
        gb.set_fake_oom(0)
    assert r["info"]["backend"] == "cpu"
    assert "out of GPU memory" in r["info"]["fallback_reason"]
    cpu = run_cpu(m, opt)
    assert np.array_equal(np.asarray(r["counts"]), np.asarray(cpu["counts"]))
    assert len(voxel_set(r) ^ voxel_set(ref)) <= 0.001 * len(voxel_set(ref))


# --------------------------------------------------------------------------
# cancel and progress
# --------------------------------------------------------------------------

def _cancel_case(run, m):
    tok = cb.CancelToken()
    assert not tok.is_cancelled()
    calls = []

    def progress(done, total):
        calls.append((done, total))
        if len(calls) == 1:
            tok.cancel()

    t0 = time.perf_counter()
    r = run(m, dict(n_samples=20_000_000, voxel_size=0.1, chunk=65536), cancel=tok, progress=progress)
    dt = time.perf_counter() - t0
    assert tok.is_cancelled()
    assert r["cancelled"] is True
    assert 0 < r["n_samples_done"] < 20_000_000
    assert r["n_samples_done"] <= 65536 * 2
    assert len(calls) == 1 and calls[0][1] == 20_000_000
    assert dt < 5.0  # returned promptly, did not run the 20e6 samples
    assert int(np.asarray(r["counts"]).sum()) + r["n_outside"] == r["n_samples_done"]


def test_cancel_from_progress_cpu():
    _cancel_case(run_cpu, fk_test_robots()["dh_6r"])


def test_cancel_from_progress_gpu(gb):
    _cancel_case(lambda m, o, **kw: run_gpu(gb, m, o, **kw), fk_test_robots()["dh_6r"])


def test_precancelled_token_does_no_work(gb):
    m = fk_test_robots()["dh_6r"]
    tok = cb.CancelToken()
    tok.cancel()
    for r in (run_cpu(m, dict(n_samples=100_000, voxel_size=0.1), cancel=tok),
              run_gpu(gb, m, dict(n_samples=100_000, voxel_size=0.1), cancel=tok)):
        assert r["cancelled"] and r["n_samples_done"] == 0
        assert len(r["occupied_voxels"]) == 0
    tok.reset()
    assert not tok.is_cancelled()


def test_progress_is_monotonic_and_complete_cpu():
    calls = []
    run_cpu(fk_test_robots()["dh_6r"], dict(n_samples=500_000, voxel_size=0.1, chunk=65536),
            progress=lambda d, t: calls.append((d, t)))
    assert len(calls) >= 7
    assert [c[0] for c in calls] == sorted(c[0] for c in calls)
    assert calls[-1] == (500_000, 500_000)


def test_progress_is_complete_gpu(gb):
    calls = []
    run_gpu(gb, fk_test_robots()["dh_6r"], dict(n_samples=500_000, voxel_size=0.1, chunk=65536),
            progress=lambda d, t: calls.append((d, t)))
    assert len(calls) >= 7
    assert [c[0] for c in calls] == sorted(c[0] for c in calls)
    assert calls[-1] == (500_000, 500_000)


def test_progress_exception_is_reported():
    def bad(d, t):
        raise KeyError("boom")

    with pytest.raises(RuntimeError, match="progress callback raised"):
        run_cpu(fk_test_robots()["dh_6r"], dict(n_samples=200_000, voxel_size=0.1, chunk=65536),
                progress=bad)


# --------------------------------------------------------------------------
# points
# --------------------------------------------------------------------------

def _tol_pos(m):
    arr = cb.compile_arrays(m)
    chain = set(cb.active_chain(m))
    n = len(chain)
    reach = float(np.linalg.norm(arr["tool"][:3, 3]))
    for k, ji in enumerate(arr["joint_index"]):
        if int(ji) in chain:
            reach += float(np.linalg.norm(arr["pre"][k][:3, 3]) + np.linalg.norm(arr["post"][k][:3, 3]))
    return 8 * n * EPS32 * reach


@pytest.mark.parametrize("name", ["six_dof_fixture", "origin_axis_12r", "tree_mimic_fixed"])
def test_points_equal_cpu_tcp_of_first_samples(gb, name):
    m = fk_test_robots()[name]
    lo, hi = limits(m)
    opt = dict(n_samples=400_000, voxel_size=0.1, want_points=True, max_points=100_000, seed=5)
    q = cb.sample_q(lo, hi, dict(seed=5), 0, 100_000).astype(np.float64)
    ref = cb.fk_batch(cb.compile(m), q, frame="base")["position"]
    tol = _tol_pos(m)
    rc = run_cpu(m, opt)
    rg = run_gpu(gb, m, dict(opt, chunk=65536))  # points cross several chunk borders
    assert rc["points"].shape == rg["points"].shape == (100_000, 3)
    assert rc["points"].dtype == rg["points"].dtype == np.float32
    assert np.abs(rc["points"] - ref).max() < 1e-6 * (1 + np.abs(ref).max())
    assert np.abs(rg["points"] - ref).max() <= tol
    assert np.abs(rg["points"] - rc["points"]).max() <= tol + 1e-6


def test_points_count_is_capped_and_optional():
    m = fk_test_robots()["dh_6r"]
    r = run_cpu(m, dict(n_samples=1000, voxel_size=0.1, want_points=True, max_points=10_000))
    assert r["points"].shape == (1000, 3)  # capped by the number of samples
    r = run_cpu(m, dict(n_samples=1000, voxel_size=0.1, want_points=True, max_points=10))
    assert r["points"].shape == (10, 3)
    r = run_cpu(m, dict(n_samples=1000, voxel_size=0.1))
    assert r["points"] is None


def test_counts_are_optional(gb):
    m = fk_test_robots()["dh_6r"]
    opt = dict(n_samples=50_000, voxel_size=0.1, want_counts=False)
    for r in (run_cpu(m, opt), run_gpu(gb, m, opt)):
        assert r["counts"] is None
        assert len(r["occupied_voxels"]) > 0


def test_same_seed_same_result_different_seed_differs(gb):
    m = fk_test_robots()["dh_6r"]
    opt = dict(n_samples=100_000, voxel_size=0.05)
    a, b = run_gpu(gb, m, opt), run_gpu(gb, m, opt)
    assert np.array_equal(np.asarray(a["counts"]), np.asarray(b["counts"]))
    c = run_gpu(gb, m, dict(opt, seed=43))
    assert not np.array_equal(np.asarray(a["counts"]), np.asarray(c["counts"]))


# --------------------------------------------------------------------------
# grid mode
# --------------------------------------------------------------------------

def test_grid_mode_mixed_radix_q():
    lo, hi = [-1.0, 0.0, 2.0], [1.0, 3.0, 2.0]
    div = [3, 4, 2]
    q = cb.sample_q(lo, hi, dict(mode="grid", divisions=div), 0, 24)
    assert q.shape == (24, 3)
    for idx in range(24):  # dof 0 varies fastest
        k0, k1, k2 = idx % 3, (idx // 3) % 4, idx // 12
        e0 = lo[0] + k0 * (hi[0] - lo[0]) / 2
        e1 = lo[1] + k1 * (hi[1] - lo[1]) / 3
        e2 = 2.0  # lo == hi
        assert np.allclose(q[idx], [e0, e1, e2], atol=1e-6)
    q1 = cb.sample_q([0.0], [4.0], dict(mode="grid", divisions=[1]), 0, 1)
    assert q1[0, 0] == 2.0  # a single division is the center


def test_grid_mode_counts_expected_samples(gb):
    m = planar_2r(1.0, 0.5)
    opt = dict(mode="grid", divisions=[100, 50], n_samples=5000, voxel_size=0.05,
               bounds_lo=[-1.6, -1.6, -0.05], bounds_hi=[1.6, 1.6, 0.05])
    rc = run_cpu(m, opt)
    rg = run_gpu(gb, m, opt)
    for r in (rc, rg):
        assert r["n_samples"] == 5000 and r["n_samples_done"] == 5000
        assert int(np.asarray(r["counts"]).sum()) + r["n_outside"] == 5000
        assert r["n_outside"] == 0
    assert voxel_set(rc) == voxel_set(rg) or len(voxel_set(rc) ^ voxel_set(rg)) <= 0.001 * len(voxel_set(rc)) + 2
    # deterministic: a second GPU run gives the same counts
    rg2 = run_gpu(gb, m, opt)
    assert np.array_equal(np.asarray(rg["counts"]), np.asarray(rg2["counts"]))


def test_grid_mode_product_over_cap_is_value_error():
    m = planar_2r()
    with pytest.raises(ValueError, match="cap"):
        run_cpu(m, dict(mode="grid", divisions=[100, 100], n_samples=9999, voxel_size=0.1))
    with pytest.raises(ValueError):
        run_cpu(m, dict(mode="grid", divisions=[100], n_samples=9999, voxel_size=0.1))
    with pytest.raises(ValueError):
        run_cpu(m, dict(mode="grid", divisions=[0, 5], n_samples=9999, voxel_size=0.1))


# --------------------------------------------------------------------------
# wrong inputs (both backends)
# --------------------------------------------------------------------------

BAD_OPTIONS = [
    ("voxel_size zero", dict(n_samples=10, voxel_size=0.0)),
    ("voxel_size negative", dict(n_samples=10, voxel_size=-0.1)),
    ("voxel_size missing", dict(n_samples=10)),
    ("n_samples zero", dict(n_samples=0, voxel_size=0.1)),
    ("n_samples negative", dict(n_samples=-5, voxel_size=0.1)),
    ("bad mode", dict(n_samples=10, voxel_size=0.1, mode="spiral")),
    ("unknown key", dict(n_samples=10, voxel_size=0.1, voxelsize=0.1)),
    ("bounds inverted", dict(n_samples=10, voxel_size=0.1, bounds_lo=[1, 1, 1], bounds_hi=[0, 0, 0])),
    ("bounds only lo", dict(n_samples=10, voxel_size=0.1, bounds_lo=[0, 0, 0])),
    ("bounds wrong size", dict(n_samples=10, voxel_size=0.1, bounds_lo=[0, 0], bounds_hi=[1, 1])),
    ("chunk not multiple", dict(n_samples=10, voxel_size=0.1, chunk=100)),
    ("grid without divisions", dict(n_samples=10, voxel_size=0.1, mode="grid")),
    ("max_points zero", dict(n_samples=10, voxel_size=0.1, want_points=True, max_points=0)),
    ("occupied cap 1024^3", dict(n_samples=10, voxel_size=0.001)),
    ("counts cap 256 MB", dict(n_samples=10, voxel_size=0.1, bounds_lo=[0, 0, 0], bounds_hi=[42.1, 42.1, 42.1])),
    ("not a dict", [1, 2, 3]),
]


@pytest.mark.parametrize("label,opt", BAD_OPTIONS, ids=[b[0] for b in BAD_OPTIONS])
def test_bad_inputs_are_value_error_cpu(label, opt):
    with pytest.raises(ValueError):
        run_cpu(planar_2r(), opt)


@pytest.mark.parametrize("label,opt", BAD_OPTIONS, ids=[b[0] for b in BAD_OPTIONS])
def test_bad_inputs_are_value_error_gpu(gb, label, opt):
    with pytest.raises(ValueError):
        run_gpu(gb, planar_2r(), opt)


def test_cap_message_names_the_maximum():
    with pytest.raises(ValueError, match="maximum allowed"):
        run_cpu(planar_2r(), dict(n_samples=10, voxel_size=0.001))
    with pytest.raises(ValueError, match="maximum allowed"):
        run_cpu(planar_2r(), dict(n_samples=10, voxel_size=0.1, bounds_lo=[0, 0, 0],
                                  bounds_hi=[42.1, 42.1, 42.1]))


def test_counts_cap_can_be_avoided_with_want_counts_false(gb):
    big = dict(n_samples=20_000, voxel_size=0.1, bounds_lo=[-21, -21, -21], bounds_hi=[21.1, 21.1, 21.1],
               want_counts=False)
    m = planar_2r()
    for r in (run_cpu(m, big), run_gpu(gb, m, big)):
        assert r["counts"] is None
        assert len(r["occupied_voxels"]) > 0
        assert r["n_samples_done"] == 20_000


def test_bad_limits_and_cancel_type(gb):
    m = planar_2r()
    cm = cb.compile(m)
    arr = cb.compile_arrays(m)
    opt = dict(n_samples=10, voxel_size=0.1)
    for lo, hi in (([0.0], [1.0]), ([0.0, 0.0], [1.0]), ([1.0, 0.0], [0.0, 1.0]),
                   ([float("nan"), 0.0], [1.0, 1.0])):
        with pytest.raises(ValueError):
            cb.sample_workspace(cm, lo, hi, opt)
        with pytest.raises(ValueError):
            gb.sample_workspace(arr, lo, hi, opt)
    with pytest.raises(ValueError):
        cb.sample_workspace(cm, [-1.0, -1.0], [1.0, 1.0], opt, cancel="stop")
    with pytest.raises(ValueError):
        gb.sample_workspace(arr, [-1.0, -1.0], [1.0, 1.0], opt, cancel=object())


def test_cancel_token_is_shared_between_modules(gb):
    assert gb.CancelToken is cb.CancelToken or isinstance(gb.CancelToken(), cb.CancelToken)
    tok = gb.CancelToken()
    tok.cancel()
    r = run_cpu(planar_2r(), dict(n_samples=1000, voxel_size=0.1), cancel=tok)
    assert r["cancelled"]


def test_gpu_workspace_rejects_more_than_64_dof(gb):
    m = random_robot(3, 65)
    with pytest.raises(ValueError):
        gb.sample_workspace(cb.compile_arrays(m), [-1.0] * 65, [1.0] * 65, dict(n_samples=10, voxel_size=0.1))
