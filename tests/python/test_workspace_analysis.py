"""R7.5: volume, manipulability map, singularity regions, slices (independent closed forms)."""
import os
import unittest.mock as mock
from pathlib import Path

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import core_bridge as cb
from app.analysis import workspace_analysis as wa
from app.kin_utils import dof_limits
from tests.python.robots import pose
from tests.python.test_workspace import planar_2r

L1, L2 = 1.0, 0.5
FX = Path(__file__).parent / "fixtures" / "urdf"


def planar_nr(lengths, tool=0.3):
    """Planar RR..R about z; joint i origin at x = lengths[i-1]; tool offset along x."""
    n = len(lengths)
    m = cb.RobotModel()
    for i in range(n + 1):
        lk = cb.Link()
        lk.name = f"l{i}"
        m.links.append(lk)
    xs = [0.0] + list(lengths[:-1])
    for i in range(n):
        j = cb.Joint()
        j.name = f"j{i}"
        j.parent_link, j.child_link = i, i + 1
        j.type = cb.JointType.Revolute
        j.kinematics = cb.JointKinematics.OriginAxis
        j.origin = pose([xs[i], 0, 0], [0, 0, 0])
        j.axis = [0.0, 0.0, 1.0]
        lim = cb.JointLimits()
        lim.lower, lim.upper = -np.pi, np.pi
        lim.vel, lim.effort = 1.0, 10.0
        j.limits = lim
        m.joints.append(j)
    m.tool = pose([tool, 0, 0], [0, 0, 0])
    m.tcp_link = n
    return m


def tcp_2r(q):
    c1, s1, c12, s12 = np.cos(q[:, 0]), np.sin(q[:, 0]), np.cos(q.sum(1)), np.sin(q.sum(1))
    return np.stack([L1 * c1 + L2 * c12, L1 * s1 + L2 * s12, np.zeros(len(q))], axis=1)


def jac_2r_xy(q):
    c1, s1, c12, s12 = np.cos(q[:, 0]), np.sin(q[:, 0]), np.cos(q.sum(1)), np.sin(q.sum(1))
    J = np.empty((len(q), 2, 2))
    J[:, 0, 0] = -L1 * s1 - L2 * s12
    J[:, 0, 1] = -L2 * s12
    J[:, 1, 0] = L1 * c1 + L2 * c12
    J[:, 1, 1] = L2 * c12
    return J


# ---------------------------------------------------------------- per-sample metrics

def test_2r_w_and_sigma_min_vs_closed_form():
    m = planar_2r(L1, L2)
    cm = cb.compile(m)
    q = np.random.default_rng(1).uniform(-np.pi, np.pi, (3000, 2))
    smin, w = wa.sample_metrics(cm, q, "position")
    np.testing.assert_allclose(w, L1 * L2 * np.abs(np.sin(q[:, 1])), atol=1e-12)
    sv = np.linalg.svd(jac_2r_xy(q), compute_uv=False)
    np.testing.assert_allclose(smin, sv[:, -1], atol=1e-10)


def test_2r_singular_exactly_at_q2_zero_and_pi():
    cm = cb.compile(planar_2r(L1, L2))
    q = np.array([[0.3, 0.0], [0.3, np.pi], [-1.2, -np.pi], [0.3, np.pi / 2]])
    smin, w = wa.sample_metrics(cm, q, "position")
    assert np.all(smin[:3] < 1e-9) and np.all(w[:3] < 1e-9)
    assert smin[3] > 0.3 and w[3] == pytest.approx(L1 * L2, rel=1e-12)


def test_3r_planar_full_mode_vs_numpy():
    ls = [1.0, 0.7, 0.4]
    m = planar_nr(ls, tool=0.3)
    cm = cb.compile(m)
    q = np.random.default_rng(2).uniform(-np.pi, np.pi, (800, 3))
    smin, w = wa.sample_metrics(cm, q, "full")
    ref_s, ref_w = np.empty(len(q)), np.empty(len(q))
    for k, qq in enumerate(q):
        p = np.zeros(2)
        a = 0.0
        pj = []
        for i in range(3):
            if i > 0:
                p = p + np.array([ls[i - 1] * np.cos(a), ls[i - 1] * np.sin(a)])
            pj.append(p.copy())
            a += qq[i]
        tcp = p + 0.3 * np.array([np.cos(a), np.sin(a)])
        J = np.array([[-(tcp[1] - pj[i][1]) for i in range(3)],
                      [(tcp[0] - pj[i][0]) for i in range(3)],
                      [1.0, 1.0, 1.0]])
        sv = np.linalg.svd(J, compute_uv=False)
        ref_s[k], ref_w[k] = sv[-1], abs(np.linalg.det(J))
    np.testing.assert_allclose(smin, ref_s, atol=1e-9)
    np.testing.assert_allclose(w, ref_w, atol=1e-9)
    # planar 3R det = l1' * l2' * |sin q2| (first two link lengths of the chain)
    np.testing.assert_allclose(w, ls[0] * ls[1] * np.abs(np.sin(q[:, 1])), atol=1e-9)


# ---------------------------------------------------------------- volume

def test_2r_volume_vs_annulus_area_1_percent():
    m = planar_2r(L1, L2)
    h = 0.005
    n = 3000
    g = np.linspace(-np.pi, np.pi, n)
    q = np.stack(np.meshgrid(g, g, indexing="ij"), axis=-1).reshape(-1, 2)
    res = wa.analyze_workspace(m, dict(q=q, voxel_size=h, want_metrics=False,
                                       bounds_lo=[-1.6, -1.6, -h / 2], bounds_hi=[1.6, 1.6, h / 2]))
    assert res["dims"][2] == 1
    area = res["volume"] / h
    exact = np.pi * (1.5 ** 2 - 0.5 ** 2)
    assert abs(area - exact) / exact < 0.01
    assert res["volume_err_bound"] > 0
    assert not res["message"] and not res["cancelled"]


def test_volume_helper_matches_r5_occupancy():
    m = planar_2r(L1, L2)
    lo, hi = dof_limits(m)
    r5 = cb.sample_workspace(cb.compile(m), lo, hi, dict(n_samples=20000, voxel_size=0.05,
                                                         want_counts=False))
    assert wa.volume_from_occupancy(r5) == pytest.approx(len(r5["occupied_voxels"]) * 0.05 ** 3)
    assert wa.volume_from_occupancy(r5) > 0


def test_boundary_bound_on_solid_block():
    nx = ny = nz = 6
    ix, iy, iz = np.meshgrid([1, 2, 3], [1, 2, 3], [1, 2, 3], indexing="ij")
    keys = np.sort(((ix * ny + iy) * nz + iz).ravel())
    assert wa._boundary_count(keys, (nx, ny, nz)) == 26   # 27 voxels, only the center is interior


# ---------------------------------------------------------------- map == independent oracle

def _oracle(n, seed, h, blo):
    rng = np.random.default_rng(seed)
    acc = {}
    done = 0
    while done < n:
        m = min(wa.CHUNK, n - done)
        q = -np.pi + rng.random((m, 2)) * 2 * np.pi
        p = tcp_2r(q)
        idx = np.floor((p - blo) / h).astype(int)
        sv = np.linalg.svd(jac_2r_xy(q), compute_uv=False)
        for i in range(m):
            k = tuple(idx[i])
            a = acc.setdefault(k, [0, 0.0, np.inf, 0.0, np.inf])
            a[0] += 1
            a[1] += sv[i, -1]
            a[2] = min(a[2], sv[i, -1])
            w = L1 * L2 * abs(np.sin(q[i, 1]))
            a[3] += w
            a[4] = min(a[4], w)
        done += m
    return acc


@pytest.fixture(scope="module")
def study():
    m = planar_2r(L1, L2)
    h = 0.05
    n = 50_000
    res = wa.analyze_workspace(m, dict(n_samples=n, seed=7, voxel_size=h, threshold=0.05,
                                       bounds_lo=[-1.6, -1.6, -h / 2], bounds_hi=[1.6, 1.6, h / 2]))
    return m, res, n, h


def test_voxel_stats_match_independent_oracle(study):
    _m, res, n, h = study
    acc = _oracle(n, 7, h, np.array([-1.6, -1.6, -h / 2]))
    assert res["filled"] == len(acc) and res["n_samples_done"] == n
    ijk = wa.voxel_index(res)
    for r in range(res["filled"]):
        a = acc[tuple(ijk[r])]
        assert res["count"][r] == a[0]
        assert res["mean_smin"][r] == pytest.approx(a[1] / a[0], abs=1e-9)
        assert res["min_smin"][r] == pytest.approx(a[2], abs=1e-9)
        assert res["mean_w"][r] == pytest.approx(a[3] / a[0], abs=1e-9)
        assert res["min_w"][r] == pytest.approx(a[4], abs=1e-9)


def test_singular_voxels_lie_on_the_workspace_boundary(study):
    _m, res, _n, h = study
    c = wa.voxel_centers(res)
    r = np.hypot(c[:, 0], c[:, 1])
    sing = res["singular_mask"]
    assert sing.sum() > 0 and res["n_singular"] == sing.sum()
    d = np.minimum(np.abs(r[sing] - 0.5), np.abs(r[sing] - 1.5))
    assert np.all(d < h * 1.5 + 0.05)         # singular set = q2 near 0 or pi = outer / inner circle
    mid = (r > 0.9) & (r < 1.1) & (res["count"] > 5)
    assert mid.any() and not sing[mid].any()
    # w near 0 on the boundary, about l1 l2 sin q2 (<= l1 l2) in the middle
    assert res["mean_w"][mid].max() <= L1 * L2 + 1e-9


def test_singular_stat_mean_is_stricter_than_min(study):
    m, res, n, h = study
    res2 = wa.analyze_workspace(m, dict(n_samples=n, seed=7, voxel_size=h, threshold=0.05,
                                        singular_stat="mean",
                                        bounds_lo=[-1.6, -1.6, -h / 2], bounds_hi=[1.6, 1.6, h / 2]))
    assert res2["n_singular"] <= res["n_singular"]
    assert np.all(res["singular_mask"][res2["singular_mask"]])


# ---------------------------------------------------------------- slices

@pytest.mark.parametrize("name", ["mean_smin", "min_w", "count", "singular"])
@pytest.mark.parametrize("axis", [0, 1, 2])
def test_slice_equals_direct_voxel_lookup(study, name, axis):
    _m, res, _n, h = study
    off = float(res["origin"][axis] + (res["dims"][axis] // 2 + 0.5) * h) if axis != 2 else 0.0
    sl = wa.slice_image(res, name, axis, off)
    assert sl["valid"]
    img = sl["image"]
    a, b = sl["axes"]
    assert img.shape == (res["dims"][a], res["dims"][b])
    rng = np.random.default_rng(3)
    cells = list(zip(rng.integers(0, img.shape[0], 400), rng.integers(0, img.shape[1], 400)))
    ijk = wa.voxel_index(res)
    cells += [(int(ijk[k, a]), int(ijk[k, b])) for k in np.where(ijk[:, axis] == sl["layer"])[0]]
    assert len(cells) > 400
    for ia, ib in cells:
        idx = [0, 0, 0]
        idx[axis], idx[a], idx[b] = sl["layer"], ia, ib
        want = wa.voxel_value(res, *idx, name)
        got = img[ia, ib]
        assert (np.isnan(want) and np.isnan(got)) or got == want
    assert sl["extent"][1] - sl["extent"][0] == pytest.approx(res["dims"][a] * h)


def test_slice_outside_grid_is_empty_and_flagged(study):
    _m, res, _n, _h = study
    sl = wa.slice_image(res, "count", 2, 10.0)
    assert not sl["valid"] and np.all(np.isnan(sl["image"]))
    with pytest.raises(ValueError):
        wa.slice_image(res, "count", 3, 0.0)
    with pytest.raises(ValueError):
        wa.field(res, "nope")


# ---------------------------------------------------------------- empty, cancel, options

def test_empty_workspace_gives_clear_message():
    m = planar_2r(L1, L2)
    res = wa.analyze_workspace(m, dict(n_samples=2000, voxel_size=0.05,
                                       bounds_lo=[5, 5, 5], bounds_hi=[6, 6, 6]))
    assert res["filled"] == 0 and res["volume"] == 0.0 and res["volume_err_bound"] == 0.0
    assert "empty" in res["message"].lower()
    assert res["n_outside"] == 2000
    sl = wa.slice_image(res, "mean_w", 2, 5.5)
    assert np.all(np.isnan(sl["image"]))


def test_cancel_and_progress():
    m = planar_2r(L1, L2)
    tok = cb.CancelToken()
    tok.cancel()
    res = wa.analyze_workspace(m, dict(n_samples=100_000, voxel_size=0.05), tok)
    assert res["cancelled"] and res["n_samples_done"] == 0 and res["message"] == ""
    seen = []
    tok2 = cb.CancelToken()

    def prog(d, t):
        seen.append((d, t))
        if d >= 2 * wa.CHUNK:
            tok2.cancel()

    res = wa.analyze_workspace(m, dict(n_samples=200_000, voxel_size=0.05), tok2, prog)
    assert res["cancelled"] and res["n_samples_done"] == 2 * wa.CHUNK
    assert [d for d, _ in seen] == sorted(d for d, _ in seen)
    assert res["filled"] > 0


def test_bad_options():
    m = planar_2r(L1, L2)
    with pytest.raises(ValueError):
        wa.analyze_workspace(m, dict(bogus=1))
    with pytest.raises(ValueError):
        wa.analyze_workspace(m, dict(metric_mode="x"))
    with pytest.raises(ValueError):
        wa.analyze_workspace(m, dict(bounds_lo=[0, 0, 0], bounds_hi=[0, 1, 1]))


def test_batch_metrics_speed_is_usable():
    import time
    cm = cb.compile(planar_2r(L1, L2))
    q = np.random.default_rng(0).uniform(-3, 3, (20000, 2))
    t = time.perf_counter()
    wa.sample_metrics(cm, q)
    per = (time.perf_counter() - t) / len(q)
    assert per < 50e-6      # measured ~6 us; 1e5 samples well under a few seconds


# ---------------------------------------------------------------- panel (offscreen)

TEST_ORG = "RoboticsAnalyzer_test_r75ui"


@pytest.fixture
def win(qapp, monkeypatch):
    from PySide6.QtCore import QSettings
    from app.main_window import MainWindow
    from app.settings import AppSettings
    from app.compute import backend as be
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
    w.close()
    s.clear()
    s.sync()


def _pump(win):
    from PySide6.QtWidgets import QApplication
    assert win.task_runner.wait_all(30000)
    QApplication.processEvents()


def test_panel_analysis_flow(win):
    with mock.patch("app.main_window.QMessageBox.warning"):
        win.import_urdf_file(str(FX / "planar_2r.urdf"))
        _pump(win)
    win.project_controller._is_modified = False
    p = win.workspace_panel
    got = []
    p.display_changed.connect(got.append)
    p._a_samples.setValue(20_000)
    p.analyze()
    assert p._a_running and not p._btn_analyze.isEnabled() and p._btn_cancel.isEnabled()
    _pump(win)
    assert not p._a_running
    res = p.analysis
    assert res is not None and res["filled"] > 0 and res["n_samples_done"] == 20_000
    assert "volume" in p.analysis_status_text
    assert p.slice_info is not None and p.slice_info["image"].shape == tuple(
        res["dims"][a] for a in p.slice_info["axes"])
    d = got[-1]
    assert d is not None and d["scalars"] is not None and len(d["scalars"]) == len(d["points"]) == res["filled"]
    p._a_field.setCurrentText("mean_w")
    p._a_axis.setCurrentText("x")
    assert p.slice_info["axes"] == (1, 2)
    assert p._slice.levels()[0] <= p._slice.levels()[1]
    # empty result -> message, no crash
    empty = wa.analyze_workspace(p._model, dict(n_samples=1000, voxel_size=0.05,
                                                bounds_lo=[9, 9, 9], bounds_hi=[10, 10, 10]))
    p._on_analysis(p._generation, empty)
    assert "empty" in p.analysis_status_text.lower()
    assert got[-1] is None


def test_panel_analysis_cancel(win):
    with mock.patch("app.main_window.QMessageBox.warning"):
        win.import_urdf_file(str(FX / "planar_2r.urdf"))
        _pump(win)
    win.project_controller._is_modified = False
    p = win.workspace_panel
    p._a_samples.setValue(20_000_000)
    p.analyze()
    p.cancel()
    _pump(win)
    assert not p._a_running
    assert p.analysis is not None and p.analysis["cancelled"]
    assert "Cancelled" in p.analysis_status_text
