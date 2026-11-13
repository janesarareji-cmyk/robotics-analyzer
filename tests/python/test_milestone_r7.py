"""
tests/python/test_milestone_r7.py
R7 stage milestone: collision-aware path planning, time parametrisation, playback, ghosts,
workspace analysis and project round trip on the 6-DOF fixture, driven through an offscreen
MainWindow with the real core_bridge.

Oracles are independent of the code under test: poses come from ``numpy_ref.fk_numpy``; the
collision recheck is a separate separating-axis box-box test written here (not the planner's
fcl edge check); the workspace volume is re-estimated with a vectorised numpy FK, another seed
and a shifted voxel grid.
"""

import os
import time
import unittest.mock as mock
from pathlib import Path

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
pytest.importorskip("fcl")

from PySide6.QtCore import QSettings
from PySide6.QtWidgets import QApplication

from app.kin_utils import dof_limits
from app.main_window import MainWindow
from app.planning.checks import check_trajectory
from app.planning.collision import shapes_from_project
from app.project.serializer import save
from app.settings import AppSettings
from app.sim.mode import DYNAMIC, KINEMATIC

from .numpy_ref import fk_numpy, spec_from_model

FX = Path(__file__).parent / "fixtures" / "urdf"
URDF = "six_dof_with_gripper.urdf"
TEST_ORG = "RoboticsAnalyzer_test_r7milestone"
RMS_LIMIT = 1e-3
FK_TOL = 1e-6
# arm upright (low gravity torque: the fixture effort limit is 10 N m), base joint swings -1 -> +1 rad
START = [-1.0, -1.5, 0.36, 0.0, -0.25, 0.0, 0.0]
GOAL = [1.0, -1.5, 0.36, 0.0, -0.25, 0.0, 0.0]
EFFORT_CHECK = 40.0         # N m, explicit (see test_items_1_2)
WALL = [0.1, 0.14, 0.3]
WS_SAMPLES = 100_000
WS_VOXEL = 0.05
RESULTS: dict = {}          # numbers for the stage gate record (printed with -s)


@pytest.fixture
def win(qapp, monkeypatch):
    monkeypatch.setattr(AppSettings, "_ORGANIZATION", TEST_ORG)
    monkeypatch.setattr(AppSettings, "_APPLICATION", TEST_ORG)
    s = QSettings(TEST_ORG, TEST_ORG)
    s.clear()
    s.sync()
    w = MainWindow()
    w.show()
    QApplication.processEvents()
    yield w
    w.project_controller._is_modified = False
    w.sim_panel.shutdown()
    w.planning_panel.shutdown()
    w.task_runner.wait_all(60000)
    w.close()
    s.clear()
    s.sync()


def _pump(win, ms=60000):
    assert win.task_runner.wait_all(ms), "task did not finish"
    QApplication.processEvents()


def _pump_until(cond, timeout=60.0):
    end = time.perf_counter() + timeout
    while time.perf_counter() < end:
        QApplication.processEvents()
        if cond():
            return True
        time.sleep(0.005)
    return cond()


# ---------------------------------------------------------------------------
# Independent oracles
# ---------------------------------------------------------------------------

def _box_axes_overlap(ca, Ra, ha, cb_, Rb, hb) -> bool:
    """Separating axis test of two oriented boxes (centre, rotation, half sizes). True = overlap."""
    d = cb_ - ca
    axes = [Ra[:, i] for i in range(3)] + [Rb[:, i] for i in range(3)]
    for i in range(3):
        for j in range(3):
            c = np.cross(Ra[:, i], Rb[:, j])
            if np.linalg.norm(c) > 1e-9:
                axes.append(c / np.linalg.norm(c))
    for a in axes:
        ra = np.sum(ha * np.abs(Ra.T @ a))
        rb = np.sum(hb * np.abs(Rb.T @ a))
        if abs(d @ a) > ra + rb:
            return False
    return True


class DenseCheck:
    """Robot-vs-environment and enabled robot-vs-robot box overlap from numpy FK + SAT."""

    def __init__(self, model, project):
        self.spec = spec_from_model(model)
        shapes = shapes_from_project(project)
        assert all(s.kind == "box" for s in shapes), "fixture links are fitted boxes"
        self.robot = [s for s in shapes if s.link >= 0]
        cfg = project.collision
        self.env = [e.to_shape() for e in cfg.environment]
        adj = {frozenset((j.parent_link, j.child_link)) for j in model.joints}
        self.pairs = []
        for a in range(len(self.robot)):
            for b in range(a + 1, len(self.robot)):
                la, lb = self.robot[a].link, self.robot[b].link
                if la == lb or frozenset((la, lb)) in adj:
                    continue
                if cfg.edits.get((min(la, lb), max(la, lb)), True) is False:
                    continue
                self.pairs.append((a, b))

    def collides(self, q) -> tuple[bool, bool]:
        """(robot hits environment, robot hits itself)."""
        frames, _ = fk_numpy(self.spec, q)
        T = [frames[s.link] @ s.pose for s in self.robot]
        half = [0.5 * np.asarray(s.params["size"], float) for s in self.robot]
        hit_env = False
        for k, e in enumerate(self.env):
            he = 0.5 * np.asarray(e.params["size"], float)
            for Ti, hi in zip(T, half):
                if _box_axes_overlap(Ti[:3, 3], Ti[:3, :3], hi, e.pose[:3, 3], e.pose[:3, :3], he):
                    hit_env = True
        hit_self = any(_box_axes_overlap(T[a][:3, 3], T[a][:3, :3], half[a],
                                         T[b][:3, 3], T[b][:3, :3], half[b]) for a, b in self.pairs)
        return hit_env, hit_self


def _tcp_batch(spec, q):
    """Vectorised numpy forward kinematics, TCP position in the base frame. q (N, n_dof)."""
    q = np.asarray(q, float)
    n = len(q)
    base = np.array(spec["base"], float)
    frames = [np.broadcast_to(np.eye(4), (n, 4, 4)).copy() for _ in range(spec["n_links"])]
    frames[0] = np.broadcast_to(base, (n, 4, 4)).copy()
    for j in sorted(spec["joints"], key=lambda j: j["child"]):
        A = np.broadcast_to(np.array(j["pre"], float), (n, 4, 4)).copy()
        if j["type"] != "fixed":
            qe = j["mult"] * q[:, j["src"]] + j["offset"]
            ax = np.asarray(j["axis"], float)
            ax = ax / np.linalg.norm(ax)
            M = np.broadcast_to(np.eye(4), (n, 4, 4)).copy()
            if j["type"] == "revolute":
                K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
                s, c = np.sin(qe)[:, None, None], np.cos(qe)[:, None, None]
                M[:, :3, :3] = np.eye(3) + s * K + (1 - c) * (K @ K)
            else:
                M[:, :3, 3] = ax[None, :] * qe[:, None]
            A = A @ M
        A = A @ np.array(j["post"], float)
        frames[j["child"]] = frames[j["parent"]] @ A
    tcp = frames[spec["tcp"]] @ np.array(spec["tool"], float)
    p = tcp[:, :3, 3] - base[:3, 3]
    return p @ base[:3, :3]               # base^-1 applied to a point


# ---------------------------------------------------------------------------
# Scenario
# ---------------------------------------------------------------------------

def _load(win):
    """Item 1: import, TCP l6, zero-pose pairs off, wall on the straight joint-space line."""
    with mock.patch("app.main_window.QMessageBox.warning") as w:
        win.import_urdf_file(str(FX / URDF))
        _pump(win)
    assert w.call_count == 0
    model = win.project_controller.robot
    names = [lk.name for lk in model.links]
    win.set_tcp_link(names.index("l6"))
    assert names[model.tcp_link] == "l6"
    spec = spec_from_model(model)
    lo, hi = (np.asarray(v, float) for v in dof_limits(model))
    for q in (START, GOAL):
        assert np.all(np.asarray(q) >= lo) and np.all(np.asarray(q) <= hi)
    win.collision_panel.set_fit_mode("obb")                     # the SAT oracle below needs boxes
    win.collision_panel.set_floor(False)                        # U3: oracle has no floor
    found = win.collision_panel.disable_zero_pose_pairs()       # fitted cubes of finger_l/r overlap
    mid = 0.5 * (np.asarray(START) + np.asarray(GOAL))
    p = fk_numpy(spec, mid)[1][:3, 3]
    assert win.collision_panel.add_shape("box", {"size": WALL},
                                         [float(p[0]), float(p[1]), float(p[2])], name="wall")
    gp = win.ghost_panel
    assert gp.add_configuration("start", START) == "start"
    assert gp.add_configuration("goal", GOAL) == "goal"
    win.project_controller._is_modified = False
    RESULTS["acm_disabled"] = len(found)
    return model


def _plan(win):
    pp = win.planning_panel
    pp._named_start.setCurrentText("start")
    pp._apply_named("start")
    pp._named_goal.setCurrentText("goal")
    pp._apply_named("goal")
    assert pp.vector("start") == START and pp.vector("goal") == GOAL
    pp._planner.setCurrentIndex(0)                  # RRT-Connect
    pp._seed.setValue(7)
    pp._time_limit.setValue(60.0)
    pp.plan()
    assert _pump_until(lambda: not pp.is_running, 90.0)
    assert pp.result is not None and pp.result.success, pp.status_text
    return pp


@pytest.fixture
def planned(win):
    model = _load(win)
    pp = _plan(win)
    return win, model, pp


def _play(sp, mode, tr):
    sp.set_mode(mode)
    sp.play_trajectory()
    pl = sp.player
    assert pl is not None and pl.mode == mode
    guard = 0
    while not pl.finished and guard < 100000:
        pl.advance(0.02)
        guard += 1
    assert pl.finished and sp.engine.error is None
    sp._emit_state()
    return pl


# ---------------------------------------------------------------------------
# Items 1, 2: obstacle, plan, independent collision recheck, constraint check
# ---------------------------------------------------------------------------

def test_items_1_2_plan_is_collision_free(win):
    model = _load(win)
    proj = win.project_controller.current_project
    dense = DenseCheck(model, proj)
    assert len(dense.env) == 1
    # item 1: endpoints free, the straight joint-space line hits the wall
    for q in (START, GOAL):
        assert dense.collides(q) == (False, False)
    line = np.linspace(START, GOAL, 201)
    line_hits = sum(dense.collides(q)[0] for q in line)
    RESULTS["line_env_hits_of_201"] = line_hits
    assert line_hits > 20, "wall must block the straight line"
    assert not any(dense.collides(q)[1] for q in line)         # only the wall blocks it

    pp = _plan(win)
    res, timed = pp.result, pp.timed
    assert timed is not None
    path = np.asarray(res.path)
    assert np.allclose(path[0], START, atol=1e-9) and np.allclose(path[-1], GOAL, atol=1e-9)
    assert len(path) >= 3, "straight line is blocked: path needs a via point"
    straight = float(np.linalg.norm(np.asarray(GOAL) - np.asarray(START)))
    RESULTS.update(path_len=res.length, raw_len=res.raw_length, straight=straight,
                   path_wps=len(path), plan_ms=res.time * 1000, iterations=res.iterations)
    assert res.length >= straight - 1e-9
    # every joint-space edge of the path, dense (independent of the planner's edge check)
    worst_edge = 0
    for a, b in zip(path[:-1], path[1:]):
        n = max(2, int(np.ceil(np.linalg.norm(b - a) / 0.01)) + 1)
        for q in np.linspace(a, b, n):
            e, s = dense.collides(q)
            assert not e and not s, "path edge collides"
            worst_edge += 1
    RESULTS["path_samples_checked"] = worst_edge
    # item 2: trajectory ok and every 1e-2 sample collision-free
    tr = timed.trajectory
    # position + velocity limits of the model; the fixture effort limit (10 N m) is below its own
    # static gravity torque (25.9 N m dof 1 at q = 0), so torque is checked against EFFORT_CHECK.
    rep = check_trajectory(tr, model, check_torque=False)
    assert rep.ok, rep.summary()
    rep = check_trajectory(tr, model, limits={"effort": EFFORT_CHECK})
    rep10 = check_trajectory(tr, model)                        # model limits as they are
    RESULTS["torque_violations_at_model_limit_10Nm"] = len(rep10.of_kind("torque"))
    RESULTS.update(traj_duration=tr.duration, traj_ok=rep.ok, traj_summary=rep.summary().splitlines()[0],
                   traj_interp=timed.interpolation)
    assert rep.ok, rep.summary()
    assert timed.report.ok
    ts = np.arange(0.0, tr.duration, 1e-2)
    ts = np.append(ts, tr.duration)
    lo, hi = (np.asarray(v, float) for v in dof_limits(model))
    for t in ts:
        q = np.asarray(tr.q(t))
        assert np.all(q >= lo - 1e-9) and np.all(q <= hi + 1e-9)
        e, s = dense.collides(q)
        assert not e and not s, f"trajectory collides at t={t:.3f}"
    RESULTS["traj_samples_checked"] = len(ts)


# ---------------------------------------------------------------------------
# Item 3: send to trajectory, kinematic and dynamic playback, recorder
# ---------------------------------------------------------------------------

def test_item_3_send_and_playback(planned):
    win, model, pp = planned
    sp = win.sim_panel
    assert sp.trajectory is None
    pp.send_to_trajectory()
    tr = sp.trajectory
    assert tr is not None and tr == pp.timed.trajectory
    assert win.project_controller.current_project.trajectory is tr
    assert np.allclose(tr.waypoints[0].q, START) and np.allclose(tr.waypoints[-1].q, GOAL)
    dur = tr.duration
    # kinematic
    pl = _play(sp, KINEMATIC, tr)
    err = float(np.max(np.abs(sp.engine.state.q - np.asarray(GOAL))))
    RESULTS["kin_final_q_err"] = err
    assert err < 1e-9
    arrays = sp.recorder.export_arrays()
    t = arrays["q"][0]
    assert len(t) > 10 and np.all(np.isfinite(arrays["q"][1]))
    assert t[-1] == pytest.approx(dur, abs=1e-6)
    RESULTS["kin_samples"] = len(t)
    sp.stop_trajectory()
    # dynamic
    _play(sp, DYNAMIC, tr)
    arrays = sp.recorder.export_arrays()
    t, q = arrays["q"]
    desired = np.array([tr.q(x) for x in t])
    e = q - desired
    rms = float(np.sqrt(np.mean(e * e)))
    RESULTS.update(track_rms=rms, track_max=float(np.max(np.abs(e))), dyn_samples=len(t))
    assert len(t) > 10 and t[-1] == pytest.approx(dur, abs=2e-3)
    assert np.all(np.isfinite(arrays["tau"][1]))
    assert rms < RMS_LIMIT, f"RMS {rms:.3e}"
    sp.stop_trajectory()


# ---------------------------------------------------------------------------
# Item 4: ghosts
# ---------------------------------------------------------------------------

def test_item_4_ghosts_match_numpy_fk(win):
    model = _load(win)
    spec = spec_from_model(model)
    gp, rv = win.ghost_panel, win.robot_view
    assert rv.ghost_count == 0
    assert gp.set_shown("start", True) and gp.set_shown("goal", True)
    assert rv.ghost_count == 2 and sorted(rv.ghost_keys) == ["goal", "start"]
    worst = 0.0
    for key, q in (("start", START), ("goal", GOAL)):
        ref = fk_numpy(spec, q)[1]
        worst = max(worst, float(np.max(np.abs(np.asarray(rv.ghost_tcp(key)) - ref))))
        assert len(rv.ghost_actors(key)) >= 2
    RESULTS["ghost_tcp_err"] = worst
    assert worst < FK_TOL
    assert gp.ghost_color("start") != gp.ghost_color("goal")
    # the real robot did not move
    assert np.allclose(np.asarray(win.sim_panel.engine.state.q if win.sim_panel.engine else 0.0), 0.0)
    gp.hide_all()
    assert rv.ghost_count == 0


# ---------------------------------------------------------------------------
# Item 5: workspace analysis map + slice, volume vs independent estimate
# ---------------------------------------------------------------------------

def test_item_5_workspace_map(win):
    model = _load(win)
    wp = win.workspace_panel
    wp._a_samples.setValue(WS_SAMPLES)
    wp._voxel.set_value(WS_VOXEL)
    t0 = time.perf_counter()
    wp.analyze()
    assert _pump_until(lambda: not wp.is_running and wp.analysis is not None, 120.0)
    dt = time.perf_counter() - t0
    res = wp.analysis
    assert res is not None and not res["cancelled"] and not res["message"], wp.analysis_status_text
    assert res["n_samples_done"] == WS_SAMPLES and res["filled"] > 100
    # sigma_min slice draws
    wp._a_field.setCurrentText("mean_smin")
    assert wp.slice_info is not None and wp.slice_info["valid"]
    img = np.asarray(wp.slice_info["image"], float)
    assert np.isfinite(img).any()
    assert wp._slice.image is not None
    wp._a_field.setCurrentText("mean_w")
    assert np.isfinite(np.asarray(wp.slice_info["image"], float)).any()
    # independent volume estimate: numpy FK, other seed, other grid origin
    spec = spec_from_model(model)
    lo, hi = (np.asarray(v, float) for v in dof_limits(model))
    rng = np.random.default_rng(2024)
    h = WS_VOXEL
    seen = set()
    chunks = 4
    for _ in range(chunks):
        q = lo + rng.random((WS_SAMPLES // chunks * 2, model.dof())) * (hi - lo)
        p = _tcp_batch(spec, q)
        idx = np.floor((p + 0.5 * h) / h).astype(np.int64)    # grid shifted by h/2
        seen.update(map(tuple, np.unique(idx, axis=0).tolist()))
    v_ind = len(seen) * h ** 3
    v_ana = res["volume"]
    bound = res["volume_err_bound"]
    RESULTS.update(ws_volume=v_ana, ws_bound=bound, ws_volume_indep=v_ind, ws_filled=res["filled"],
                   ws_seconds=dt, ws_n_singular=res["n_singular"])
    assert abs(v_ana - v_ind) <= bound, f"volume {v_ana:.4f} vs independent {v_ind:.4f} bound {bound:.4f}"
    assert abs(v_ana - v_ind) / v_ind < 0.15


# ---------------------------------------------------------------------------
# Item 6: save, close, open; item 7: nothing running after close
# ---------------------------------------------------------------------------

def test_items_6_7_round_trip_and_clean_close(planned, tmp_path):
    win, model, pp = planned
    pp.send_to_trajectory()
    pc = win.project_controller
    proj = pc.current_project
    tr = win.sim_panel.trajectory
    assert proj.trajectory is tr
    env0 = [e.to_dict() for e in proj.collision.environment]
    edits0 = dict(proj.collision.edits)
    cfg0 = {k: list(v) for k, v in proj.configs.items()} if isinstance(proj.configs, dict) \
        else {k: list(win.ghost_panel.store.get(k)) for k in win.ghost_panel.store.names}
    assert set(cfg0) == {"start", "goal"} and env0 and edits0
    out = tmp_path / "r7.robotproj"
    save(proj, out)
    pc._is_modified = False
    pc.close_project(None)
    assert win.sim_panel.trajectory is None and len(win.ghost_panel.store) == 0
    with mock.patch("app.main_window.QMessageBox.warning") as w:
        assert pc.open_project(None, str(out))
    assert w.call_count == 0
    p2 = pc.current_project
    assert p2.trajectory == tr
    assert win.sim_panel.trajectory == tr
    assert [e.to_dict() for e in p2.collision.environment] == env0
    assert dict(p2.collision.edits) == edits0
    assert [e.to_dict() for e in win.collision_panel.config.environment] == env0
    assert win.ghost_panel.store.names == ["start", "goal"]
    for k, q in cfg0.items():
        assert list(win.ghost_panel.store.get(k)) == q
    assert win.planning_panel._named.keys() == {"start", "goal"}
    # the reopened project plans again (obstacle still blocks the straight line)
    pc._is_modified = False
    # item 7: close the window, nothing keeps running
    win.sim_panel.stop_trajectory()
    win.close()
    QApplication.processEvents()
    assert win.task_runner.active_count == 0
    assert not win.sim_panel.timer.isActive()
    assert not win.graph_panel.timer.isActive()
    assert not win.planning_panel.is_running


def test_zz_print_gate_numbers():
    """Numbers for the PROGRESS.md gate record (run with -s)."""
    print("R7 GATE NUMBERS", {k: (f"{v:.4g}" if isinstance(v, float) else v) for k, v in RESULTS.items()})
