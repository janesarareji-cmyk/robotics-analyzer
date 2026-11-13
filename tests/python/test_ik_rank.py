"""U4a.2: ranked IK solutions (scores, weights, collision, speed)."""
import os

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
pytest.importorskip("fcl")

from PySide6.QtCore import QCoreApplication

import core_bridge as cb
from app.analysis.ik_rank import DEFAULT_WEIGHTS, RankedSolution, normalise_weights, rank_solutions, score
from app.importers.urdf import import_urdf
from app.kin_utils import SINGULAR_WARN, current_q, dof_limits, write_q
from app.main_window import MainWindow
from app.sample_robots import list_sample_robots
from app.settings import AppSettings

SAMPLES = {p.parent.name: (t, p) for t, p in list_sample_robots()}


@pytest.fixture
def win(qapp, monkeypatch):
    monkeypatch.setattr(AppSettings, "_ORGANIZATION", "RoboticsAnalyzer_test_u4a2")
    monkeypatch.setattr(AppSettings, "_APPLICATION", "RoboticsAnalyzer_test_u4a2")
    w = MainWindow()
    w.settings._settings.clear()
    w.collision_action.setChecked(True)
    w.show()
    yield w
    w.project_controller._is_modified = False
    w.close()


def _load(win, name):
    path = SAMPLES[name][1]
    win._import_path = str(path)
    win._on_urdf_import_result(import_urdf(str(path)))
    win.task_runner.wait_all(30000)
    QCoreApplication.processEvents()
    return win.project_controller.robot


def _ur5_case(model):
    lo, hi = dof_limits(model)
    q = np.array([0.3, -1.2, 1.4, -0.9, 1.1, 0.4])
    cm = cb.compile(model)
    T = np.asarray(cb.fk_compiled(cm, list(q))["tcp"])
    return cm, q, T


def test_weights_validation_and_score():
    assert np.isclose(normalise_weights(None).sum(), 1.0)
    assert np.allclose(normalise_weights([2, 2, 2, 2]), 0.25)
    for bad in ([1, 2, 3], [-1, 1, 1, 1], [0, 0, 0, 0], [np.nan, 1, 1, 1]):
        with pytest.raises(ValueError):
            normalise_weights(bad)
    parts = {"motion": 1.0, "limits": 0.0, "manip": 0.5, "clearance": 0.0}
    assert score(parts) == pytest.approx(0.4 + 0.1)


def test_ur5_elbow_up_and_down_found_sorted(win):
    model = _load(win, "ur5")
    cm, q, T = _ur5_case(model)
    res = rank_solutions(model, T, q + 0.05, win.collision_service, cm=cm)
    assert len(res) >= 4 and not res.position_only
    elbows = {int(np.sign(s.q[2])) for s in res if s.valid}
    assert elbows == {1, -1}, "elbow up and elbow down both expected"
    valid = [s for s in res if s.valid]
    assert [s.total for s in valid] == sorted([s.total for s in valid], reverse=True)
    # every solution reaches the target, parts in 0..1
    for s in res:
        Tq = np.asarray(cb.fk_compiled(cm, s.q)["tcp"])
        assert np.linalg.norm(Tq[:3, 3] - T[:3, 3]) < 1e-4
        assert all(0.0 <= v <= 1.0 for v in s.parts.values())
    # the solution nearest to the seed has the best motion part
    nearest = min(res, key=lambda s: s.raw["distance"])
    assert nearest.parts["motion"] == pytest.approx(max(s.parts["motion"] for s in res))
    assert min(s.parts["motion"] for s in res) == pytest.approx(0.0, abs=1e-9)


def test_order_changes_with_weights(win):
    model = _load(win, "ur5")
    cm, q, T = _ur5_case(model)
    a = rank_solutions(model, T, q + 0.05, win.collision_service, weights=(1, 0, 0, 0), cm=cm)
    b = rank_solutions(model, T, q + 0.05, win.collision_service, weights=(0, 0, 1, 0), cm=cm)
    qa = [tuple(np.round(s.q, 6)) for s in a]
    qb = [tuple(np.round(s.q, 6)) for s in b]
    assert qa != qb
    # motion-only: first one is the nearest; manip-only: first one has the largest sigma_min
    av = [s for s in a if s.valid]
    assert av[0].raw["distance"] == pytest.approx(min(s.raw["distance"] for s in av))
    bv = [s for s in b if s.valid]
    assert bv[0].raw["sigma_min"] == pytest.approx(max(s.raw["sigma_min"] for s in bv))


def test_colliding_solution_marked_invalid(win):
    model = _load(win, "ur5")
    cm, q, T = _ur5_case(model)
    base = rank_solutions(model, T, q + 0.05, win.collision_service, cm=cm)
    n0 = sum(s.valid for s in base)
    # box at the elbow of the best solution
    best, second = base[0], base[1]
    assert best.valid and second.valid
    fa = cb.fk_compiled(cm, best.q)["frames"]
    fb = cb.fk_compiled(cm, second.q)["frames"]
    # the link frame where the two solutions differ most: a box there blocks only the best one
    k = max(range(4, 9), key=lambda i: np.linalg.norm(np.asarray(fa[i])[:3, 3] - np.asarray(fb[i])[:3, 3]))
    spot = np.asarray(fa[k])[:3, 3]
    win.collision_panel.add_shape("box", {"size": [0.03, 0.03, 0.03]}, list(spot), name="blocker")
    QCoreApplication.processEvents()
    win._sync_env_objects()
    res = rank_solutions(model, T, q + 0.05, win.collision_service, cm=cm)
    bad = [s for s in res if not s.valid]
    assert bad and all(s.reason.startswith("collision") for s in bad)
    assert any(np.allclose(s.q, best.q) for s in bad)
    assert [s.valid for s in res] == sorted([s.valid for s in res], reverse=True)   # invalid last
    assert sum(s.valid for s in res) < n0
    assert res.best() is not None and np.allclose(res.best().q, second.q)


def test_position_only_note_for_5dof(win):
    model = _load(win, "xarm5")
    cm = cb.compile(model)
    q = np.array([0.2, 0.4, -0.6, 0.5, 0.1])
    T = np.asarray(cb.fk_compiled(cm, list(q))["tcp"])
    res = rank_solutions(model, T, q, win.collision_service, cm=cm)
    assert res.position_only and "position only" in res.note
    assert len(res) >= 1
    for s in res:
        Tq = np.asarray(cb.fk_compiled(cm, s.q)["tcp"])
        assert np.linalg.norm(Tq[:3, 3] - T[:3, 3]) < 1e-3


def test_speed_7dof(win):
    model = _load(win, "panda")
    cm = cb.compile(model)
    lo, hi = dof_limits(model)
    q = np.array([(a + b) / 2 for a, b in zip(lo, hi)])
    q[0], q[3] = 0.3, -1.7
    T = np.asarray(cb.fk_compiled(cm, list(q))["tcp"])
    rank_solutions(model, T, q, win.collision_service, cm=cm)             # warm up
    times = []
    for _ in range(5):
        res = rank_solutions(model, T, q + 0.05, win.collision_service, cm=cm)
        times.append(res.elapsed_ms)
    med = float(np.median(times))
    print(f"\nU4a.2 rank_solutions panda ({model.dof()} dof): {res.n_found} found, median {med:.0f} ms, "
          f"max {max(times):.0f} ms")
    assert len(res) >= 1
    assert med < 300.0


def test_singular_warn_constant():
    assert 0 < SINGULAR_WARN < 1
    s = RankedSolution([0.0], 0.5, {}, True, raw={"sigma_min": SINGULAR_WARN / 2})
    assert s.near_singular
    assert DEFAULT_WEIGHTS == (0.4, 0.2, 0.2, 0.2)


def test_settings_weights(win):
    st = win.settings
    assert st.ik_rank_weights() == (0.4, 0.2, 0.2, 0.2)
    st.set_ik_rank_weights((1, 2, 3, 4))
    assert st.ik_rank_weights() == (1.0, 2.0, 3.0, 4.0)
    st._settings.setValue(st.KEY_IK_RANK_WEIGHTS, "junk")
    assert st.ik_rank_weights() == (0.4, 0.2, 0.2, 0.2)
