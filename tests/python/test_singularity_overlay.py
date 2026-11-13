"""U6.3: singularity zones overlay (kinds, threshold, planar 2R circles, CSV, window)."""
import csv
import os
import time

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
pytest.importorskip("fcl")

import core_bridge as cb
from app.analysis import workspace_analysis as wa
from app.analysis.batch_jacobian import batch_pose_jacobian, sigma_min_batch
from app.analysis.singularity_overlay import (DEFAULT_KINDS, KINDS, SingularityOverlay, arm_info,
                                              classify, export_csv, study, zones)
from app.importers.urdf import import_urdf
from app.kin_utils import SINGULAR_WARN, dof_limits
from app.main_window import MainWindow
from app.settings import AppSettings

from .test_curve_editor import pump_until
from .test_live_blocking import SAMPLES, load

ORG = "RoboticsAnalyzer_test_u6sing"
PLANAR = os.path.join(os.path.dirname(__file__), "..", "..", "examples", "planar_2r.urdf")


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


def _poses(model, n, fix=None, seed=0):
    lo, hi = (np.asarray(v) for v in dof_limits(model))
    q = lo + np.random.default_rng(seed).random((n, len(lo))) * (hi - lo)
    for i, v in (fix or {}).items():
        q[:, i] = v
    return q


def test_arm_info_wrist_kinds():
    info = {}
    for name in ("ur5", "kuka_iiwa", "xarm5", "panda"):
        m = _model(name)
        info[name] = arm_info(m, cb.compile(m))
    assert info["ur5"].has_kinds and info["ur5"].wrist == "offset"
    assert info["kuka_iiwa"].has_kinds and info["kuka_iiwa"].wrist == "spherical"
    assert not info["xarm5"].has_kinds            # 5 DOF: general only
    assert not info["panda"].has_kinds            # gripper DOFs after the arm: last three are not the wrist


def test_ur5_wrist_set_contains_q5_zero():
    m = _model("ur5")
    cm = cb.compile(m)
    info = arm_info(m, cm)
    q = _poses(m, 3000, {4: 0.0}, seed=1)
    J, p, _ = batch_pose_jacobian(cm, q)
    assert np.all(sigma_min_batch(J) < 1e-6)               # q5 = 0 is singular
    assert np.all(classify(info, J, p) == 0)               # kind wrist
    d = study(m, q_samples=q)
    z = zones(d, SINGULAR_WARN, ("wrist",))
    assert z["red"].all() and d["filled"] > 100            # every voxel of those poses is a wrist zone
    # q5 away from 0 and pi: no wrist zone
    q2 = _poses(m, 3000, {4: 1.2}, seed=2)
    z2 = zones(study(m, q_samples=q2), SINGULAR_WARN, ("wrist",))
    assert not z2["red"].any()


def test_ur5_elbow_set_contains_stretched_poses():
    m = _model("ur5")
    cm = cb.compile(m)
    info = arm_info(m, cm)
    q = _poses(m, 3000, {2: 0.0, 4: 1.0}, seed=3)           # elbow stretched, wrist bent
    J, p, _ = batch_pose_jacobian(cm, q)
    assert np.all(sigma_min_batch(J) < 1e-6)
    kind = classify(info, J, p)
    assert np.all(kind == 1)
    z = zones(study(m, q_samples=q), SINGULAR_WARN, ("elbow",))
    assert z["red"].all()
    assert not zones(study(m, q_samples=q), SINGULAR_WARN, ("wrist",))["red"].any()


def test_shoulder_samples_are_mostly_singular():
    m = _model("ur5")
    cm = cb.compile(m)
    info = arm_info(m, cm)
    q = _poses(m, 200_000, seed=4)
    J, p, _ = batch_pose_jacobian(cm, q)
    s = sigma_min_batch(J)
    kind = classify(info, J, p)
    sh = kind == 2
    assert sh.sum() > 100
    assert np.median(s[sh]) < 0.5 * np.median(s)


def test_planar_2r_singular_circles(tmp_path):
    src = open(PLANAR, encoding="utf-8").read()
    f = tmp_path / "p.urdf"
    f.write_text(src.replace('<origin xyz="1 0 0" rpy="0 0 0"/>\n    <axis', '<origin xyz="1 0 0" rpy="0 0 0"/>\n    <axis')
                 .replace('<joint name="j_tip" type="fixed">\n    <parent link="l2"/>\n    <child link="tip"/>\n    <origin xyz="1 0 0"',
                          '<joint name="j_tip" type="fixed">\n    <parent link="l2"/>\n    <child link="tip"/>\n    <origin xyz="0.6 0 0"'),
                 encoding="utf-8")
    m = import_urdf(str(f)).model
    h = 0.02
    d = study(m, n_samples=300_000, voxel_size=h, metric="position", seed=5)
    z = zones(d, 0.02, KINDS)
    c = wa.voxel_centers(d)
    r = np.linalg.norm(c[:, :2], axis=1)
    red = z["red"]
    assert red.sum() > 20
    # outer circle l1 + l2 = 1.6, inner circle |l1 - l2| = 0.4: every red voxel is near one of them
    near = np.minimum(np.abs(r[red] - 1.6), np.abs(r[red] - 0.4))
    assert near.max() < 1.5 * h + 0.5 * h * np.sqrt(3)
    assert r[red].max() > 1.6 - 2 * h and r[red].max() < 1.6 + 2 * h
    inner = r[red][r[red] < 1.0]
    assert len(inner) > 0 and abs(inner.mean() - 0.4) < 2 * h
    assert not d["has_kinds"] and set(np.unique(z["kind"][red])) == {3}     # general only


def test_zones_threshold_kinds_and_csv(tmp_path):
    m = _model("ur5")
    d = study(m, "Fast", n_samples=60_000)
    lo = zones(d, 0.02)
    hi = zones(d, 0.1)
    assert hi["red"].sum() >= lo["red"].sum() > 0
    assert np.all(hi["red"][lo["red"]])                       # red at 0.02 stays red at 0.1
    only = zones(d, 0.05, ("wrist",))
    allk = zones(d, 0.05, KINDS)
    assert np.all(allk["red"][only["red"]])
    assert not zones(d, 0.05, ())["red"].any()
    path = tmp_path / "s.csv"
    base = np.eye(4)
    base[:3, 3] = [1.0, 2.0, 3.0]
    n = export_csv(str(path), d, 0.05, DEFAULT_KINDS, base)
    z = zones(d, 0.05, DEFAULT_KINDS)
    assert n == int((z["red"] | z["orange"]).sum()) > 0
    rows = list(csv.reader(open(path, encoding="utf-8")))
    assert rows[0] == ["x", "y", "z", "sigma_min", "kind"] and len(rows) == n + 1
    assert all(r[4] in KINDS and float(r[3]) < 0.1 for r in rows[1:])
    c0 = wa.voxel_centers(d)[(z["red"] | z["orange"])][0]
    assert np.allclose([float(v) for v in rows[1][:3]], c0 + [1.0, 2.0, 3.0], atol=1e-5)


def test_study_1e6_speed_print():
    m = _model("ur5")
    t0 = time.perf_counter()
    d = study(m, n_samples=1_000_000)
    dt = time.perf_counter() - t0
    print(f"singularity study ur5, 1e6 samples (CPU, batch Jacobian): {dt:.2f} s, {d['filled']} voxels")
    assert dt < 20


def test_overlay_in_window(win, tmp_path):
    load(win, "ur5")
    m = win.overlays
    rv = win.viewport.robot_view
    m.set_option("singularity", "quality", "Fast")
    m.set_enabled("singularity", True)
    assert pump_until(lambda: rv.overlay_actor("singularity") is not None, 120)
    ov = m.overlay("singularity")
    n_default = rv.overlay_voxel_count("singularity")
    assert n_default > 0 and "red" in m.status("singularity")
    # threshold: redraw only, no recompute
    cnt = m.compute_count["singularity"]
    m.set_option("singularity", "threshold", 0.01)
    assert rv.overlay_voxel_count("singularity") < n_default
    assert m.compute_count["singularity"] == cnt
    m.set_option("singularity", "threshold", SINGULAR_WARN)
    # kind boxes
    m.set_option("singularity", "kinds", KINDS)
    assert rv.overlay_voxel_count("singularity") >= n_default
    m.set_option("singularity", "kinds", ())
    assert rv.overlay_actor("singularity") is None
    m.set_option("singularity", "kinds", DEFAULT_KINDS)
    # export
    path = tmp_path / "win.csv"
    assert ov.export(str(path)) > 0 and path.exists()
    # the legend names the threshold
    left, right = ov.legend_labels(m.options("singularity"))
    assert f"{SINGULAR_WARN:.3f}" in left
    m.set_enabled("singularity", False)
    assert rv.overlay_actor("singularity") is None
