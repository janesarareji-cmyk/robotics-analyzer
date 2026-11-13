"""U4a.3 / U4a.4: ghost preview, target panel, play to target."""
import os

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
pytest.importorskip("fcl")

from PySide6.QtCore import QCoreApplication

import core_bridge as cb
from app.analysis.ik_rank import RankedSolution, rank_solutions
from app.importers.urdf import import_urdf
from app.kin_utils import SINGULAR_WARN, current_q
from app.main_window import MainWindow
from app.sample_robots import list_sample_robots
from app.settings import AppSettings
from app.target_preview import GHOST_BAD, GHOST_OK, TargetPreview, target_status

SAMPLES = {p.parent.name: (t, p) for t, p in list_sample_robots()}


@pytest.fixture
def win(qapp, monkeypatch):
    monkeypatch.setattr(AppSettings, "_ORGANIZATION", "RoboticsAnalyzer_test_u4a4")
    monkeypatch.setattr(AppSettings, "_APPLICATION", "RoboticsAnalyzer_test_u4a4")
    w = MainWindow()
    w.settings._settings.clear()
    w.collision_action.setChecked(True)
    w.show()
    yield w
    w.project_controller._is_modified = False
    w.sim_dock.hide()
    w.close()


def _load(win, name):
    path = SAMPLES[name][1]
    win._import_path = str(path)
    win._on_urdf_import_result(import_urdf(str(path)))
    win.task_runner.wait_all(30000)
    QCoreApplication.processEvents()
    return win.project_controller.robot


def _sol(q, valid=True, sigma=0.5, reason=""):
    return RankedSolution(list(q), 0.5, {}, valid, reason, {"sigma_min": sigma})


# ---------------------------------------------------------------- U4a.3 preview

def test_target_status_colours():
    assert target_status([]) == "bad"
    assert target_status([_sol([0], valid=False)]) == "bad"
    assert target_status([_sol([0], sigma=0.5)]) == "ok"
    assert target_status([_sol([0], sigma=SINGULAR_WARN / 2)]) == "warn"
    # invalid first, then a good valid one: status follows the best VALID one
    assert target_status([_sol([0], valid=False), _sol([1], sigma=0.5)]) == "ok"


def test_preview_ghost_colours_and_show_all(win):
    model = _load(win, "ur5")
    rv = win.viewport.robot_view
    pv_ = TargetPreview(lambda: rv, lambda: win.project_controller.robot)
    q0 = current_q(model)
    sols = [_sol(q0), _sol(np.add(q0, 0.3), valid=False, reason="collision")] + \
           [_sol(np.add(q0, 0.1 * k)) for k in range(2, 11)]
    pv_.show(sols, selected=1, show_all=False)
    assert rv.ghost_keys == [("ik", 1)]
    actor = rv.ghost_actors(("ik", 1))[1]
    col = tuple(round(c, 2) for c in actor.GetProperty().GetColor())
    from pyvista import Color
    assert col == tuple(round(c, 2) for c in Color(GHOST_BAD).float_rgb)
    pv_.show(sols, selected=0)
    col = tuple(round(c, 2) for c in rv.ghost_actors(("ik", 0))[1].GetProperty().GetColor())
    assert col == tuple(round(c, 2) for c in Color(GHOST_OK).float_rgb)
    pv_.show(sols, selected=9, show_all=True)           # more than 8: selected one is kept
    assert len(rv.ghost_keys) == 8 and ("ik", 9) in rv.ghost_keys
    sel_op = rv.ghost_actors(("ik", 9))[1].GetProperty().GetOpacity()
    oth_op = rv.ghost_actors(("ik", 0))[1].GetProperty().GetOpacity()
    assert sel_op > oth_op
    # the Ghosts-dock ghosts are not touched by clear()
    rv.add_ghost("named", model, cb.fk(model))
    pv_.clear()
    assert rv.ghost_keys == ["named"]
    rv.clear_ghosts()


def test_preview_ghost_at_solution_pose(win):
    model = _load(win, "ur5")
    rv = win.viewport.robot_view
    cm = cb.compile(model)
    q = np.array([0.3, -1.2, 1.4, -0.9, 1.1, 0.4])
    T = np.asarray(cb.fk_compiled(cm, list(q))["tcp"])
    res = rank_solutions(model, T, current_q(model), win.collision_service, cm=cm)
    pv_ = TargetPreview(lambda: rv, lambda: model)
    pv_.show(res.solutions, selected=0)
    tcp = rv.ghost_tcp(("ik", 0))
    assert np.allclose(tcp[:3, 3], T[:3, 3], atol=1e-4)
    pv_.clear()
    assert rv.ghost_count == 0
