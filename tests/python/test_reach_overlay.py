"""U6.2: reach map overlay and the batch Jacobian behind it."""
import os
import time

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
pytest.importorskip("fcl")

import core_bridge as cb
from app.analysis import workspace_analysis as wa
from app.analysis.batch_jacobian import (batch_pose_jacobian, fast_sample_metrics, metrics_batch,
                                         sigma_min_batch)
from app.analysis.reach_overlay import QUALITY, ReachOverlay
from app.compute.backend import estimate_reach
from app.importers.urdf import import_urdf
from app.kin_utils import dof_limits
from app.main_window import MainWindow
from app.settings import AppSettings

from .test_curve_editor import pump_until
from .test_live_blocking import SAMPLES, load

ORG = "RoboticsAnalyzer_test_u6reach"


@pytest.fixture
def win(qapp, monkeypatch):
    monkeypatch.setattr(AppSettings, "_ORGANIZATION", ORG)
    monkeypatch.setattr(AppSettings, "_APPLICATION", ORG)
    AppSettings()._settings.clear()
    w = MainWindow()
    w.settings._settings.clear()
    w.show()
    yield w
    w.project_controller._is_modified = False
    w.sim_dock.hide()
    w.close()


def _model(name):
    return import_urdf(str(SAMPLES[name][1])).model


def _samples(model, n, seed=0):
    lo, hi = (np.asarray(v) for v in dof_limits(model))
    return lo + np.random.default_rng(seed).random((n, len(lo))) * (hi - lo)


@pytest.mark.parametrize("name", ["ur5", "panda", "xarm5"])
def test_batch_jacobian_matches_cb(name):
    model = _model(name)
    cm = cb.compile(model)
    q = _samples(model, 60, 1)
    J, p, R = batch_pose_jacobian(cm, q)
    smin, w = metrics_batch(J, "full")
    for i in range(len(q)):
        Jr = np.asarray(cb.jacobian_tcp(cm, q[i].tolist()))
        sv = np.linalg.svd(Jr, compute_uv=False)
        k = min(Jr.shape)
        assert abs(sv[k - 1] - smin[i]) < 1e-5
        assert np.allclose(np.linalg.svd(J[i], compute_uv=False), sv, atol=1e-5)


def test_fast_metrics_equal_sample_metrics_position():
    model = _model("ur5")
    cm = cb.compile(model)
    q = _samples(model, 80, 2)
    a, wa_ = wa.sample_metrics(cm, q, "position")
    b, wb = fast_sample_metrics(cm, q, "position")
    assert np.allclose(a, b, atol=1e-5) and np.allclose(wa_, wb, atol=1e-5)


def test_sigma_min_batch_planar_2r():
    # planar 2R, position mode: sigma_min = 0 at q2 = 0 and pi, w = l1 l2 |sin q2|
    model = import_urdf(os.path.join(os.path.dirname(__file__), "..", "..", "examples",
                                     "planar_2r.urdf")).model
    cm = cb.compile(model)
    q = np.array([[0.3, 0.0], [0.3, np.pi], [0.3, 0.5], [-1.0, 1.1], [0.2, 2.0]])
    J, _, _ = batch_pose_jacobian(cm, q)
    smin, w = metrics_batch(J, "position")
    assert smin[0] < 1e-5 and smin[1] < 1e-5 and smin[2] > 0.05
    k = w[2:] / np.abs(np.sin(q[2:, 1]))
    assert np.allclose(k, k[0], rtol=1e-5)          # w = (l1 l2) |sin q2|


def test_overlay_voxel_count_equals_analyze_workspace():
    model = _model("ur5")
    ov = ReachOverlay()
    data = ov.compute(model, {"quality": "Fast", "seed": 42}, cb.CancelToken(), lambda d, t: None)
    samples, cells = QUALITY["Fast"]
    h = max(estimate_reach(model) / cells, 1e-3)
    ref = wa.analyze_workspace(model, dict(n_samples=samples, seed=42, voxel_size=h,
                                           metric_mode="position"))
    assert data["filled"] == ref["filled"] > 0
    assert np.array_equal(data["keys"], ref["keys"])
    assert np.allclose(data["mean_w"], ref["mean_w"], atol=1e-4)


def test_overlay_in_window_slice_and_probe(win):
    model = load(win, "ur5")
    m = win.overlays
    rv = win.viewport.robot_view
    m.set_option("reach", "quality", "Fast")
    m.set_enabled("reach", True)
    assert pump_until(lambda: rv.overlay_actor("reach") is not None, 120)
    data = m.data("reach")
    full = rv.overlay_voxel_count("reach")
    assert full == min(data["filled"], rv.MAX_OVERLAY_VOXELS)
    assert "voxels" in m.status("reach")
    # slice: one layer only
    m.set_option("reach", "slice", True)
    m.set_option("reach", "height", 0.5)
    layer = rv.overlay_voxel_count("reach")
    mask = ReachOverlay.layer_mask(data, 0.5)
    assert layer == int(mask.sum()) and 0 < layer < full
    # probe: tcp of the start pose is reachable (green), a point far away is not (red)
    tcp = np.asarray(cb.fk(model)["tcp"], float)[:3, 3]
    res = m.probe(tcp)
    assert res["reach"] is True
    assert rv.overlay_actor("reach_probe") is not None
    far = tcp + np.array([5.0, 0, 0])
    assert m.probe(far)["reach"] is False
    # off: everything gone
    m.set_enabled("reach", False)
    assert rv.overlay_actor("reach") is None and rv.overlay_actor("reach_probe") is None


def test_target_drag_moves_probe(win):
    load(win, "ur5")
    m = win.overlays
    rv = win.viewport.robot_view
    m.set_option("reach", "quality", "Fast")
    m.set_enabled("reach", True)
    assert pump_until(lambda: m.data("reach") is not None, 120)
    win.set_mode("Target")
    pose = win.target_panel.current_pose()
    assert pose is not None
    win.target_panel.controller.pose_changed.emit(pose)
    assert rv.overlay_actor("reach_probe") is not None
    p1 = np.array(rv.overlay_actor("reach_probe").GetCenter())
    moved = pose.copy()
    moved[0, 3] += 0.2
    win.target_panel.controller.pose_changed.emit(moved)
    p2 = np.array(rv.overlay_actor("reach_probe").GetCenter())
    assert np.linalg.norm(p2 - p1) > 0.05
    m.set_enabled("reach", False)


def test_reach_speed_print(win):
    model = _model("ur5")
    ov = ReachOverlay()
    t0 = time.perf_counter()
    d = ov.compute(model, {"quality": "Normal", "seed": 1}, cb.CancelToken(), lambda a, b: None)
    dt = time.perf_counter() - t0
    print(f"reach map ur5 Normal: {d['filled']} voxels, {dt:.2f} s")
    assert dt < 30
