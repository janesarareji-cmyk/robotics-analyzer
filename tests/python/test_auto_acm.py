"""U3.2: automatic allowed-collision matrix."""
import os
import time

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
pytest.importorskip("fcl")

from PySide6.QtCore import QCoreApplication

from app.auto_acm import AutoAcmTask, apply_auto_acm
from app.importers.urdf import import_urdf
from app.kin_utils import current_q
from app.main_window import MainWindow
from app.planning.collision import FLOOR_ID, CollisionConfig, CollisionWorld, shapes_from_geometry
from app.sample_robots import list_sample_robots
from app.settings import AppSettings

SAMPLES = {p.parent.name: (t, p) for t, p in list_sample_robots()}


def _world(name, cfg=None):
    res = import_urdf(str(SAMPLES[name][1]))
    shapes = shapes_from_geometry(list(res.collisions) or list(res.visuals))
    return res, CollisionWorld(res.model, shapes, cfg or CollisionConfig())


@pytest.mark.parametrize("name", sorted(SAMPLES))
def test_start_pose_free_and_time(name):
    res, w = _world(name)
    q0 = current_q(res.model)
    t0 = time.perf_counter()
    out = w.auto_acm(2000, 0, q0)
    dt = time.perf_counter() - t0
    print(f"auto ACM {name}: {dt:.2f} s, {len(out['pairs'])} pairs disabled")
    assert dt < 10.0
    w.set_q(q0)
    assert [p for p in w.pairs() if p[1] != FLOOR_ID] == []      # no self collision at q0
    assert w.config.auto == out["pairs"]
    assert set(out["pairs"].values()) <= {"always", "never", "start"}


def test_deterministic_and_idempotent():
    res, w = _world("ur5")
    a = w.auto_acm(300, 1, current_q(res.model))
    b = w.auto_acm(300, 1, current_q(res.model))
    assert a["pairs"] == b["pairs"] and a["counts"] == b["counts"]


def test_gripper_fingers_not_flagged():
    res, w = _world("xarm5_gripper")
    names = [l.name for l in res.model.links]
    w.auto_acm(2000, 0, current_q(res.model))
    grip = {i for i, n in enumerate(names) if "gripper" in n or "knuckle" in n or "finger" in n}
    base = names.index("xarm_gripper_base_link")
    w.set_q(current_q(res.model))
    assert not [p for p in w.pairs() if p[0] in grip and p[1] in grip]
    # over random poses the gripper base never reports a hit against its own fingers/knuckles
    # (finger vs finger stays checked: that contact stops a closing gripper)
    from app.kin_utils import dof_limits
    lo, hi = (np.asarray(v) for v in dof_limits(w.model))
    rng = np.random.default_rng(5)
    for _ in range(100):
        w.set_q(lo + rng.random(len(lo)) * (hi - lo))
        assert not [p for p in w.pairs()
                    if base in p[:2] and p[0] in grip and p[1] in grip]


def test_folded_ur5_is_flagged():
    res, w = _world("ur5")
    q0 = current_q(res.model)
    w.auto_acm(2000, 0, q0)
    q = list(q0)
    q[1], q[2] = 0.79, -3.14                      # upper arm up, elbow folded back over the base
    w.set_q(q)
    hits = [p for p in w.pairs() if p[1] != FLOOR_ID]
    assert hits, "folded pose must collide"
    names = [l.name for l in res.model.links]
    print("folded ur5:", [(names[a], names[b]) for a, b, _ in hits])


def test_user_edit_wins():
    res, w = _world("ur5")
    out = w.auto_acm(500, 0, current_q(res.model))
    a, b = next(iter(out["pairs"]))
    assert w.config.source(a, b) == "auto" and not w.config.checked(a, b, False)
    w.config.set_check(a, b, True)
    assert w.config.source(a, b) == "user" and w.config.checked(a, b, False)
    w._build_pairs()
    out2 = w.auto_acm(500, 0, current_q(res.model))
    assert (a, b) not in out2["pairs"]            # explicit edits are left alone
    assert w.config.edits[(min(a, b), max(a, b))] is True


def test_cancel_changes_nothing():
    res, w = _world("ur5")
    n = [0]

    def cancel():
        n[0] += 1
        return n[0] > 5
    assert w.auto_acm(2000, 0, cancel=cancel) is None
    assert w.config.auto == {} and not w.config.auto_info


def test_config_roundtrip_and_default():
    cfg = CollisionConfig()
    assert cfg.is_default()
    cfg.set_auto({(3, 1): "never", (0, 2): "start"}, {"n_samples": 10, "seed": 0, "seconds": 0.1})
    assert not cfg.is_default()
    back = CollisionConfig.from_dict(cfg.to_dict())
    assert back.auto == {(1, 3): "never", (0, 2): "start"} and back.auto_info["n_samples"] == 10
    assert back.source(1, 3) == "auto" and not back.checked(1, 3, False)
    assert CollisionConfig.from_dict({"acm_edits": []}).auto == {}      # old file


def test_project_save_reopen(tmp_path):
    from app.project.project_data import ProjectData
    from app.project.serializer import load, save
    res = import_urdf(str(SAMPLES["ur5"][1]))
    p = ProjectData.new("t")
    p.robot_model = res.model
    p.visuals, p.collisions = list(res.visuals), list(res.collisions)
    task = AutoAcmTask(p, n_samples=200)
    assert apply_auto_acm(p, task.execute())
    f = tmp_path / "a.robotproj"
    save(p, str(f))
    q = load(str(f))
    assert q.collision.auto == p.collision.auto and q.collision.auto_info == p.collision.auto_info


@pytest.fixture
def win(qapp, monkeypatch):
    monkeypatch.setattr(AppSettings, "_ORGANIZATION", "RoboticsAnalyzer_test_acm")
    monkeypatch.setattr(AppSettings, "_APPLICATION", "RoboticsAnalyzer_test_acm")
    w = MainWindow()
    w.settings._settings.clear()
    yield w
    w.project_controller._is_modified = False
    w.close()


def test_import_runs_auto_acm_in_worker(win):
    res = import_urdf(str(SAMPLES["panda"][1]))
    win._import_path = str(SAMPLES["panda"][1])
    win._on_urdf_import_result(res)
    assert win.auto_acm_task is not None
    assert win.task_runner.wait_all(30000)
    QCoreApplication.processEvents()
    proj = win.project_controller.current_project
    assert proj.collision is not None and proj.collision.auto
    tbl = win.collision_panel.acm_table
    assert tbl.rowCount() == len(proj.collision.auto)
    assert tbl.item(0, 3).text().startswith("auto")
    assert win.project_controller.is_modified


def test_second_import_cancels_first(win):
    for name in ("ur5", "ur3"):
        res = import_urdf(str(SAMPLES[name][1]))
        win._import_path = str(SAMPLES[name][1])
        win._on_urdf_import_result(res)
    assert win.task_runner.wait_all(30000)
    QCoreApplication.processEvents()
    proj = win.project_controller.current_project
    assert proj.collision is not None and proj.collision.auto
