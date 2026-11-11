"""R5.4: batch IK. CPU part (core_bridge.solve_ik_batch). GPU part: see marked place at the end."""
import numpy as np
import pytest

import core_bridge as cb

from app.kin_utils import dof_limits

from .numpy_ref import fk_numpy, so3_log, spec_from_model
from .robots import fk_test_robots, random_q, random_robot

N_CASES = 100
TOL = 1e-6


def _robots():
    six = fk_test_robots()["six_dof_fixture"]
    return {
        "six_dof_fixture": six,
        "random_6r_dh": random_robot(11, 6, kinds="dh"),
        "random_6r_origin_axis": random_robot(12, 6, kinds="origin_axis"),
    }


ROBOTS = _robots()


def _setup(name, n=N_CASES, seed=0):
    model = ROBOTS[name]
    spec = spec_from_model(model)
    cm = cb.compile(model)
    lo, hi = dof_limits(model)
    rng = np.random.default_rng(seed)
    dof = model.dof()
    qt = np.array([random_q(model, rng) for _ in range(n)])
    targets = np.array([fk_numpy(spec, q)[1] for q in qt])
    q0 = np.clip(qt + rng.uniform(-0.5, 0.5, qt.shape) / np.sqrt(dof), lo, hi)
    return model, spec, cm, lo, hi, qt, targets, q0


@pytest.mark.parametrize("name", list(ROBOTS))
def test_equals_per_item_solve_ik(name):
    _, _, cm, lo, hi, _, targets, q0 = _setup(name, n=40, seed=1)
    r = cb.solve_ik_batch(cm, lo, hi, targets, q0)
    for i in range(len(targets)):
        s = cb.solve_ik(cm, lo, hi, targets[i], list(q0[i]), None)
        assert np.array_equal(r["q"][i], np.array(s["q"])), (name, i)
        assert r["pos_error"][i] == s["pos_error"]
        assert r["rot_error"][i] == s["rot_error"]
        assert r["iterations"][i] == s["iterations"]
        assert bool(r["converged"][i]) == s["converged"]
        assert (r["status"][i] == cb.IK_CONVERGED) == s["converged"]


def test_options_forwarded():
    _, _, cm, lo, hi, _, targets, q0 = _setup("random_6r_origin_axis", n=10, seed=2)
    opt = {"max_iter": 3, "damping": 1e-2}
    r = cb.solve_ik_batch(cm, lo, hi, targets, q0, opt)
    for i in range(10):
        s = cb.solve_ik(cm, lo, hi, targets[i], list(q0[i]), opt)
        assert np.array_equal(r["q"][i], np.array(s["q"]))


@pytest.mark.parametrize("name", list(ROBOTS))
def test_thread_count_independent(name):
    _, _, cm, lo, hi, _, targets, q0 = _setup(name, n=200, seed=3)
    ref = cb.solve_ik_batch(cm, lo, hi, targets, q0, None, 1)
    for nt in (2, 0):
        r = cb.solve_ik_batch(cm, lo, hi, targets, q0, None, nt)
        for k in ("q", "status", "pos_error", "rot_error", "iterations", "converged"):
            assert np.array_equal(r[k], ref[k]), (name, nt, k)


@pytest.mark.parametrize("name", list(ROBOTS))
def test_reachable_convergence(name):
    model, spec, cm, lo, hi, qt, targets, q0 = _setup(name)
    r = cb.solve_ik_batch(cm, lo, hi, targets, q0)
    conv = r["status"] == cb.IK_CONVERGED
    assert conv.mean() >= 0.9, f"{name}: rate {conv.mean():.2f}"
    for i in np.nonzero(conv)[0]:
        T = fk_numpy(spec, r["q"][i])[1]
        ep = np.linalg.norm(targets[i][:3, 3] - T[:3, 3])
        er = np.linalg.norm(so3_log(targets[i][:3, :3] @ T[:3, :3].T))
        assert ep < TOL and er < TOL
    assert np.all(r["q"] >= np.array(lo) - 1e-12) and np.all(r["q"] <= np.array(hi) + 1e-12)


def test_statuses_present_and_shapes():
    _, _, cm, lo, hi, _, targets, q0 = _setup("six_dof_fixture", n=30, seed=4)
    r = cb.solve_ik_batch(cm, lo, hi, targets, q0)
    assert set(r) == {"q", "status", "converged", "pos_error", "rot_error", "iterations"}
    assert r["q"].shape == (30, len(lo))
    assert r["status"].dtype == np.int32 and r["converged"].dtype == bool
    assert {cb.IK_CONVERGED, cb.IK_MAX_ITER, cb.IK_STALLED, cb.IK_NAN} == {0, 1, 2, 3}
    assert set(np.unique(r["status"])) <= {0, 1, 2, 3}
    assert cb.IK_CONVERGED in set(r["status"])
    # broadcast q0 works
    rb = cb.solve_ik_batch(cm, lo, hi, targets, np.zeros(len(lo)))
    assert rb["q"].shape == (30, len(lo))
    # empty batch
    re_ = cb.solve_ik_batch(cm, lo, hi, np.zeros((0, 4, 4)), np.zeros((0, len(lo))))
    assert re_["q"].shape == (0, len(lo))


def test_nan_input_gives_status_nan():
    _, _, cm, lo, hi, _, targets, q0 = _setup("six_dof_fixture", n=4, seed=5)
    targets = targets.copy()
    targets[1, 0, 3] = np.nan
    q0 = q0.copy()
    q0[2, 0] = np.inf
    r = cb.solve_ik_batch(cm, lo, hi, targets, q0)
    assert r["status"][1] == cb.IK_NAN and r["status"][2] == cb.IK_NAN
    assert r["status"][0] != cb.IK_NAN and r["status"][3] != cb.IK_NAN


def test_wrong_shapes_value_error():
    _, _, cm, lo, hi, _, targets, q0 = _setup("six_dof_fixture", n=5, seed=6)
    dof = len(lo)
    with pytest.raises(ValueError):
        cb.solve_ik_batch(cm, lo, hi, targets[:, :3, :], q0)
    with pytest.raises(ValueError):
        cb.solve_ik_batch(cm, lo, hi, targets.reshape(5, 16), q0)
    with pytest.raises(ValueError):
        cb.solve_ik_batch(cm, lo, hi, targets, q0[:3])
    with pytest.raises(ValueError):
        cb.solve_ik_batch(cm, lo, hi, targets, np.zeros((5, dof + 1)))
    with pytest.raises(ValueError):
        cb.solve_ik_batch(cm, lo[:-1], hi, targets, q0)
    with pytest.raises(ValueError):
        cb.solve_ik_batch(cm, lo, hi, targets, q0, {"bogus": 1})


def test_unreachable_target_no_crash():
    _, _, cm, lo, hi, _, targets, q0 = _setup("six_dof_fixture", n=1, seed=7)
    t = targets[:1].copy()
    t[0, :3, 3] = [50.0, 50.0, 50.0]
    r = cb.solve_ik_batch(cm, lo, hi, t, q0[:1])
    assert r["status"][0] in (cb.IK_MAX_ITER, cb.IK_STALLED)
    assert not r["converged"][0]
    assert np.all(np.isfinite(r["q"]))


# ---------------------------------------------------------------------------
# GPU TESTS (R5.4 part B, prototype gpu_bridge.solve_ik_batch = GPU fp32 + CPU polish).
# Skip without a CUDA device.
# ---------------------------------------------------------------------------


def _gpu_or_skip():
    try:
        import gpu_bridge
    except Exception as e:  # pragma: no cover
        pytest.skip(f"gpu_bridge not importable: {e}")
    info = gpu_bridge.device_info()
    if not info["available"]:
        pytest.skip(f"no CUDA device ({info['error']})")
    return gpu_bridge


def _gpu_run(gb, model, cm, lo, hi, targets, q0, **kw):
    return gb.solve_ik_batch(cb.compile_arrays(model), lo, hi, targets, q0, **kw)


@pytest.mark.parametrize("name", list(ROBOTS))
def test_gpu_converged_items_fk_error(name):
    gb = _gpu_or_skip()
    model, spec, cm, lo, hi, qt, targets, q0 = _setup(name, n=200, seed=21)
    r = _gpu_run(gb, model, cm, lo, hi, targets, q0)
    assert r["info"]["backend"] == "cuda", r["info"]
    conv = r["status"] == cb.IK_CONVERGED
    assert conv.mean() >= 0.9, f"{name}: rate {conv.mean():.2f}"
    for i in np.nonzero(conv)[0]:
        T = fk_numpy(spec, r["q"][i])[1]
        ep = np.linalg.norm(targets[i][:3, 3] - T[:3, 3])
        er = np.linalg.norm(so3_log(targets[i][:3, :3] @ T[:3, :3].T))
        assert ep < TOL and er < TOL, (name, i, ep, er)
    assert np.all(r["q"] >= np.array(lo) - 1e-12) and np.all(r["q"] <= np.array(hi) + 1e-12)
    assert set(r) == {"q", "status", "converged", "pos_error", "rot_error", "iterations", "info"}


def test_gpu_rate_within_one_point_of_cpu():
    gb = _gpu_or_skip()
    model, _, cm, lo, hi, _, targets, q0 = _setup("six_dof_fixture", n=10000, seed=22)
    c = cb.solve_ik_batch(cm, lo, hi, targets, q0)
    g = _gpu_run(gb, model, cm, lo, hi, targets, q0)
    rate_c = float((c["status"] == cb.IK_CONVERGED).mean())
    rate_g = float((g["status"] == cb.IK_CONVERGED).mean())
    assert rate_g >= rate_c - 0.01, (rate_c, rate_g)


def test_gpu_nan_input_gives_status_nan():
    gb = _gpu_or_skip()
    model, _, cm, lo, hi, _, targets, q0 = _setup("six_dof_fixture", n=4, seed=23)
    targets = targets.copy()
    targets[1, 0, 3] = np.nan
    q0 = q0.copy()
    q0[2, 0] = np.inf
    r = _gpu_run(gb, model, cm, lo, hi, targets, q0)
    assert r["status"][1] == cb.IK_NAN and r["status"][2] == cb.IK_NAN
    assert r["status"][0] != cb.IK_NAN and r["status"][3] != cb.IK_NAN


def test_gpu_small_forced_chunks_same_results():
    gb = _gpu_or_skip()
    model, _, cm, lo, hi, _, targets, q0 = _setup("six_dof_fixture", n=1000, seed=24)
    ref = _gpu_run(gb, model, cm, lo, hi, targets, q0)
    small = _gpu_run(gb, model, cm, lo, hi, targets, q0, chunk=256)
    assert small["info"]["chunks"] > 1
    for k in ("q", "status", "pos_error", "rot_error", "iterations"):
        assert np.array_equal(small[k], ref[k]), k


def test_gpu_info_counts_consistent():
    gb = _gpu_or_skip()
    model, _, cm, lo, hi, _, targets, q0 = _setup("random_6r_origin_axis", n=500, seed=25)
    r = _gpu_run(gb, model, cm, lo, hi, targets, q0)
    info = r["info"]
    assert info["n_polish_converged"] == int(r["converged"].sum())
    assert info["n_polish_converged"] == int((r["status"] == cb.IK_CONVERGED).sum())
    assert 0 <= info["n_gpu_converged"] <= len(targets)
    assert info["gpu_ms"] > 0 and info["polish_ms"] >= 0 and info["kernel_ms"] > 0
    assert info["fallback_reason"] == ""


def test_gpu_wrong_input_value_error():
    gb = _gpu_or_skip()
    model, _, cm, lo, hi, _, targets, q0 = _setup("six_dof_fixture", n=5, seed=26)
    arrays = cb.compile_arrays(model)
    with pytest.raises(ValueError):
        gb.solve_ik_batch(arrays, lo, hi, targets[:, :3, :], q0)
    with pytest.raises(ValueError):
        gb.solve_ik_batch(arrays, lo, hi, targets, q0, {"null_gain": 0.1})
    with pytest.raises(ValueError):
        gb.solve_ik_batch(arrays, lo, hi, targets, q0, None, 100)  # chunk not a multiple of 128


def test_gpu_experimental_flag_exposed():
    gb = _gpu_or_skip()
    assert gb.GPU_IK_EXPERIMENTAL is True
