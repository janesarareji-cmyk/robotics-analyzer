"""R7.6 UI: ghosts, comparison table, collision settings, planning wiring in MainWindow (offscreen)."""
import os
import time
import unittest.mock as mock
from pathlib import Path

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtCore import QSettings, Qt
from PySide6.QtWidgets import QApplication

import core_bridge as cb
from app.configs.store import ConfigStore
from app.kin_utils import current_q, write_q
from app.main_window import MainWindow
from app.panels.collision_panel import CollisionPanel
from app.panels.ghost_panel import MAX_GHOSTS, GhostPanel
from app.planning.collision import CollisionConfig
from app.project.serializer import load, save
from app.settings import AppSettings

from .numpy_ref import fk_numpy, spec_from_model
from .robots import random_q, random_robot
from .test_cartesian_traj import planar_2r

FX = Path(__file__).parent / "fixtures" / "urdf"
TEST_ORG = "RoboticsAnalyzer_test_r76ui"


# ------------------------------------------------------------------ ghost panel alone
@pytest.fixture
def gp(qapp):
    p = GhostPanel()
    p.set_store(ConfigStore())
    p.set_model(planar_2r())
    p.show()
    return p


def test_panel_save_show_compare(gp):
    shown = []
    gp.ghosts_changed.connect(shown.append)
    changed = []
    gp.configs_changed.connect(lambda: changed.append(1))
    gp.set_q_provider(lambda: [0.3, -0.2])
    assert gp.save_current("a") == "a" and gp.store.get("a") == [0.3, -0.2] and changed
    gp.add_configuration("b", [1.0, 0.5])
    assert gp.table.rowCount() == 2
    assert gp.add_configuration("bad", [1.0]) is None          # wrong DOF
    assert gp.set_shown("a", True) and gp.set_shown("b", True)
    names = [e[0] for e in shown[-1]]
    assert names == ["a", "b"] and gp.ghost_color("a") != gp.ghost_color("b")
    assert gp.table.item(0, 0).checkState() == Qt.CheckState.Checked
    # table check box toggles a ghost
    gp.table.item(0, 0).setCheckState(Qt.CheckState.Unchecked)
    assert gp.shown_names == ["b"] and [e[0] for e in shown[-1]] == ["b"]
    # comparison
    assert gp.compare(["a"]) == []                              # needs two
    gp.select_names(["a", "b"])
    rows = gp.compare_selected()
    assert [r["name"] for r in rows] == ["a", "b"]
    assert gp.comparison_table.rowCount() == 2
    heads = [gp.comparison_table.horizontalHeaderItem(c).text()
             for c in range(gp.comparison_table.columnCount())]
    assert "TCP dist [m]" in heads and "dq2 [rad]" in heads and "dManip" in heads
    # delete removes the ghost too
    gp.delete("b")
    assert gp.shown_names == [] and gp.store.names == ["a"] and shown[-1] == []


def test_max_eight_ghosts_message(gp):
    for i in range(MAX_GHOSTS + 2):
        gp.add_configuration(f"c{i}", [0.1 * i, 0.0])
    ok = [gp.set_shown(f"c{i}", True) for i in range(MAX_GHOSTS + 2)]
    assert ok == [True] * MAX_GHOSTS + [False, False]
    assert len(gp.shown_names) == MAX_GHOSTS
    assert "Max 8" in gp.status_text
    assert len({gp.ghost_color(n) for n in gp.shown_names}) == MAX_GHOSTS   # one colour each
    # box of the refused row is unchecked again
    assert gp.table.item(MAX_GHOSTS, 0).checkState() == Qt.CheckState.Unchecked
    gp.hide_all()
    assert gp.shown_names == []


def test_ik_solutions_added_and_shown(gp):
    gp.set_ik_source(lambda: [[0.1, 0.2], [0.3, 0.4], [1.0]])    # last one: wrong DOF, skipped
    names = gp.add_ik_solutions(show=True)
    assert names == ["ik", "ik 2"] and gp.shown_names == names


def test_model_change_drops_configs(qapp):
    p = GhostPanel()
    logs = []
    p.log_requested.connect(logs.append)
    store = ConfigStore({"two": [0.0, 0.1], "three": [0.0, 0.0, 0.0]})
    p.set_store(store)
    p.set_model(planar_2r())
    assert store.names == ["two"]
    assert any("dropped" in m and "three" in m for m in logs)
    p.set_model(random_robot(3, 4))
    assert store.names == []
    p.set_model(None)                                           # closed: nothing to drop, no error


# ------------------------------------------------------------------ collision panel alone
def test_collision_panel_edit_list(qapp):
    from app.project.project_data import ProjectData
    proj = ProjectData.new("c")
    proj.robot_model = planar_2r()
    p = CollisionPanel()
    got = []
    p.changed.connect(lambda: got.append(1))
    p.set_project(proj)
    p.set_model(proj.robot_model)
    assert proj.collision is None
    assert p.add_shape("box", {"size": [0.3, 0.3, 0.4]}, [0, 1.2, 0], name="wall")
    assert p.add_shape("sphere", {"radius": 0.2}, [1, 1, 1])
    assert p.add_shape("cylinder", {"radius": 0.1, "length": 0.5})
    assert not p.add_shape("capsule", {"radius": 0.1, "length": 0.5})      # not allowed
    assert not p.add_shape("box", {"size": [0.3, -1, 0.4]})                # bad size
    cfg = proj.collision
    assert [e.kind for e in cfg.environment] == ["box", "sphere", "cylinder"]
    assert cfg.environment[0].name == "wall" and cfg.environment[0].xyz == [0.0, 1.2, 0.0]
    assert p.env_table.rowCount() == 3 and p.env_table.item(0, 1).text() == "box"
    assert p.remove_shape(1) and [e.kind for e in cfg.environment] == ["box", "cylinder"]
    assert not p.remove_shape(9)
    # ACM edit list
    assert p.set_acm_edit(0, 2, True) and cfg.edits == {(0, 2): True}
    assert p.set_acm_edit(2, 1, False) and cfg.edits[(1, 2)] is False
    assert not p.set_acm_edit(1, 1, True) and not p.set_acm_edit(0, 9, True)
    assert p.acm_table.rowCount() == 2
    assert p.remove_acm_edit(2, 0) and (0, 2) not in cfg.edits
    assert len(got) == 7
    p.set_project(None)
    assert p.env_table.rowCount() == 0 and not p.add_shape("box", {"size": [1, 1, 1]})


def test_collision_zero_pose_button(qapp):
    pytest.importorskip("fcl")
    from app.importers.visuals import LinkVisual
    from app.project.project_data import ProjectData
    proj = ProjectData.new("c")
    proj.robot_model = planar_2r()
    # link 0 and link 2 boxes overlap at q = 0 (not adjacent) -> always colliding at zero pose
    def box(link, x):
        return LinkVisual(link_index=link, link_name=f"l{link}", kind="box", params={"size": [1.0, 0.1, 0.1]},
                          origin_xyz=[x, 0.0, 0.0], origin_rpy=[0.0, 0.0, 0.0])
    proj.collisions = [box(0, 0.5), box(2, 0.5)]
    p = CollisionPanel()
    p.set_project(proj)
    p.set_model(proj.robot_model)
    got = []
    p.changed.connect(lambda: got.append(1))
    found = p.disable_zero_pose_pairs()
    assert found == [(0, 2)] and proj.collision.edits == {(0, 2): False} and got
    assert p.disable_zero_pose_pairs() == []                    # second time: nothing left


# ------------------------------------------------------------------ main window wiring
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
    w.task_runner.wait_all(20000)
    w.close()
    s.clear()
    s.sync()


def _pump(win, ms=20000):
    assert win.task_runner.wait_all(ms), "task did not finish"
    QApplication.processEvents()


def _pump_until(cond, timeout=30.0):
    end = time.perf_counter() + timeout
    while time.perf_counter() < end:
        QApplication.processEvents()
        if cond():
            return True
        time.sleep(0.005)
    return False


def _clean(win):
    """Forget unsaved changes: close/open would ask in a modal dialog (hangs offscreen)."""
    win.project_controller._is_modified = False


def _load(win, name="planar_2r.urdf"):
    with mock.patch("app.main_window.QMessageBox.warning"):
        win.import_urdf_file(str(FX / name))
        _pump(win)
    win.project_controller._is_modified = False
    return win.project_controller.robot


def test_docks_exist_hidden_and_toggle(qapp, monkeypatch):
    monkeypatch.setattr(AppSettings, "_ORGANIZATION", TEST_ORG)
    monkeypatch.setattr(AppSettings, "_APPLICATION", TEST_ORG)
    win = MainWindow()                       # not shown: dock visibility signals are immediate
    for dock, act in ((win.planning_dock, win._action_view_planning),
                      (win.ghost_dock, win._action_view_ghosts),
                      (win.collision_dock, win._action_view_collision)):
        assert dock.isHidden() and not act.isChecked()
        act.setChecked(True)
        assert not dock.isHidden()
        act.setChecked(False)
        assert dock.isHidden()
    assert win.tabifiedDockWidgets(win.joint_dock)
    win.close()


def test_ghost_count_and_tcp_vs_numpy(win):
    model = _load(win, "six_dof_with_gripper.urdf")
    model.tcp_link = [i for i, lk in enumerate(model.links) if lk.name == "l6"][0]
    win.project_controller.set_robot(model)
    win.project_controller._is_modified = False
    spec = spec_from_model(model)
    rng = np.random.default_rng(5)
    qs = {f"p{i}": list(np.array(random_q(model, rng)) * 0.5) for i in range(3)}
    gp, rv = win.ghost_panel, win.robot_view
    for k, q in qs.items():
        assert gp.add_configuration(k, q) == k
    assert win.project_controller.is_modified                       # configs mark the project
    assert rv.ghost_count == 0
    for n, k in enumerate(qs, 1):
        gp.set_shown(k, True)
        assert rv.ghost_count == n == len(gp.shown_names)
    for k, q in qs.items():
        assert np.allclose(rv.ghost_tcp(k), fk_numpy(spec, q)[1], atol=1e-6)
        assert len(rv.ghost_actors(k)) >= 2
    gp.set_shown("p1", False)
    assert rv.ghost_count == 2 and sorted(rv.ghost_keys) == ["p0", "p2"]
    # the real robot is not moved by ghosts
    assert np.allclose(current_q(win.project_controller.robot), 0.0)
    # robot replaced with the same DOF: ghosts hidden (drawn for the old robot), configs stay
    win.project_controller.set_robot(win.project_controller.robot)
    assert rv.ghost_count == 0 and len(gp.store) == 3
    gp.hide_all()
    _clean(win)
    win.project_controller.close_project(win)
    assert rv.ghost_count == 0 and len(gp.store) == 0


def test_ik_solution_list_feeds_ghosts(win):
    model = _load(win)
    win.ik_panel._solutions = [{"q": [0.1, 0.2]}, {"q": [0.5, -0.5]}]
    names = win.ghost_panel.add_ik_solutions(show=True)
    assert win.robot_view.ghost_count == 2 and names == ["ik", "ik 2"]


def test_project_round_trip_and_dof_change(win, tmp_path):
    _load(win)
    gp = win.ghost_panel
    gp.add_configuration("start", [0.0, 0.0])
    gp.add_configuration("goal", [2.2, 0.3])
    win.collision_panel.add_shape("box", {"size": [0.3, 0.3, 0.4]}, [0, 1.2, 0], name="b")
    path = tmp_path / "g.robotproj"
    save(win.project_controller.current_project, path)
    _clean(win)
    win.project_controller.close_project(win)
    with mock.patch("app.main_window.QMessageBox.warning"):
        assert win.project_controller.open_project(win, str(path))
    assert gp.store.names == ["start", "goal"]
    assert win.planning_panel._named.keys() == {"start", "goal"}      # fed to the planning panel
    cfg = win.project_controller.current_project.collision
    assert [e.name for e in cfg.environment] == ["b"]
    assert [e.name for e in win.collision_panel.config.environment] == ["b"]
    # same file, robot with another DOF count: configs dropped with a warning in the log
    _clean(win)
    win.project_controller.close_project(win)
    p = load(path)
    p.robot_model = random_robot(3, 3)
    save(p, path)
    _clean(win)
    win.project_controller.close_project(win)
    with mock.patch("app.main_window.QMessageBox.warning"):
        assert win.project_controller.open_project(win, str(path))
    assert len(gp.store) == 0 and len(win.project_controller.current_project.configs) == 0


def test_planning_wiring_plan_and_send(win):
    pytest.importorskip("fcl")
    _load(win)
    gp, pp = win.ghost_panel, win.planning_panel
    gp.add_configuration("start", [0.0, 0.0])
    gp.add_configuration("goal", [2.2, 0.3])
    win.collision_panel.add_shape("box", {"size": [0.3, 0.3, 0.4]}, [5.0, 5.0, 0.0], name="far")
    # collision source = project shapes + project config with the environment obstacle
    shapes, cfg = pp._collision_source()
    assert [e.name for e in cfg.environment] == ["far"]
    pp._named_start.setCurrentText("start")
    pp._apply_named("start")
    pp._named_goal.setCurrentText("goal")
    pp._apply_named("goal")
    assert pp.vector("start") == [0.0, 0.0] and pp.vector("goal") == [2.2, 0.3]
    pp.capture_current("start")                                    # q provider = model q (zeros)
    assert pp.vector("start") == [0.0, 0.0]
    pp._time_limit.setValue(20.0)
    pp.plan()
    assert _pump_until(lambda: not pp.is_running)
    assert pp.result is not None and pp.result.success, pp.status_text
    assert win.sim_panel.trajectory is None
    pp.send_to_trajectory()
    traj = win.sim_panel.trajectory
    assert traj is not None and traj.n_dof == 2
    assert np.allclose(traj.waypoints[0].q, [0.0, 0.0]) and np.allclose(traj.waypoints[-1].q, [2.2, 0.3])
    assert win.project_controller.current_project.trajectory is traj       # stored in the project
    assert win.project_controller.is_modified


# ------------------------------------------------------------------ shown set saved (F1.4)
def _reopen(win, path):
    _clean(win)
    win.project_controller.close_project(win)
    with mock.patch("app.main_window.QMessageBox.warning"):
        assert win.project_controller.open_project(win, str(path))


def test_shown_set_round_trip_in_main_window(win, tmp_path):
    _load(win)
    gp, rv, pc = win.ghost_panel, win.robot_view, win.project_controller
    for k, q in (("a", [0.1, 0.2]), ("b", [0.3, -0.2]), ("c", [-0.5, 0.4]), ("d", [1.0, 1.0])):
        gp.add_configuration(k, q)
    pc._is_modified = False
    for k in ("c", "a", "b"):
        gp.set_shown(k, True)
    assert pc.is_modified                                  # show/hide marks the project
    colours = [gp.ghost_color(k) for k in ("c", "a", "b")]
    assert len(set(colours)) == 3
    path = tmp_path / "shown.robotproj"

    save(pc.current_project, path)
    assert load(path).configs.shown == list(zip(("c", "a", "b"), colours))
    _reopen(win, path)
    assert gp.shown_names == ["c", "a", "b"]
    assert [gp.ghost_color(k) for k in ("c", "a", "b")] == colours
    assert rv.ghost_count == 3 and sorted(rv.ghost_keys) == ["a", "b", "c"]
    assert not pc.is_modified
    # delete a shown config: gone from shown after save
    gp.delete("a")
    assert gp.shown_names == ["c", "b"] and rv.ghost_count == 2
    save(pc.current_project, path)
    assert [n for n, _ in load(path).configs.shown] == ["c", "b"]
    # rename (store level) keeps the entry
    gp.store.rename("b", "b2")
    save(pc.current_project, path)
    assert [n for n, _ in load(path).configs.shown] == ["c", "b2"]
    _reopen(win, path)
    assert gp.shown_names == ["c", "b2"] and rv.ghost_count == 2
    gp.hide_all()
    save(pc.current_project, path)
    _reopen(win, path)
    assert gp.shown_names == [] and rv.ghost_count == 0


def test_panel_set_store_restores_and_emits(qapp):
    st = ConfigStore({"x": [0.0, 0.0], "y": [0.1, 0.1]})
    st.set_shown([("y", "#123456"), ("x", "#654321")])
    p = GhostPanel()
    seen = []
    p.ghosts_changed.connect(lambda e: seen.append([(n, c) for n, _, c in e]))
    p.set_store(st)
    assert p.shown_names == ["y", "x"] and seen[-1] == [("y", "#123456"), ("x", "#654321")]
    p.set_model(planar_2r())                               # project open order: kept
    assert p.shown_names == ["y", "x"] and seen[-1] == [("y", "#123456"), ("x", "#654321")]
    p.set_model(planar_2r())                               # robot replaced later: hidden
    assert p.shown_names == [] and seen[-1] == [] and st.shown == []
