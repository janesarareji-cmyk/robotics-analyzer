"""D1.2: five sample targets per bundled robot (3 reachable, 2 unreachable)."""
import os

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
pytest.importorskip("fcl")

import core_bridge as cb
from app.analysis.ik_rank import rank_solutions
from app.analysis.target_motion import plan_target_motion
from app.gripper import gripper_dofs
from app.sample_robots import SAMPLE_INFO, list_sample_robots
from app.sample_targets import (KIND_REACHABLE, KIND_UNREACHABLE, build_scene, check_target,
                                load_sample_targets, sample_kinds)

SAMPLES = {p.parent.name: (t, p) for t, p in list_sample_robots()}
NAMES = sorted(SAMPLES)


def test_nine_robots_all_have_info_and_targets():
    assert len(NAMES) == 9 and set(NAMES) == set(SAMPLE_INFO)
    for n in NAMES:
        store = load_sample_targets(SAMPLES[n][1])
        assert store is not None and len(store) == 5, n


@pytest.mark.parametrize("name", NAMES)
def test_targets_reachable_unreachable_and_linked(name):
    urdf = SAMPLES[name][1]
    store = load_sample_targets(urdf)
    kinds = sample_kinds(urdf)
    assert store.names() == ["Pick A", "Too far", "Place B", "Home-side C", "Too close"]
    assert sorted(kinds.values()) == [KIND_REACHABLE] * 3 + [KIND_UNREACHABLE] * 2
    scene = build_scene(urdf)
    # sample info DOF = arm joints (gripper DOFs not counted)
    assert scene.model.dof() - len(gripper_dofs(scene.model)) == SAMPLE_INFO[name]["dof"]
    reach, unreach = [], []
    for t in store:
        c = check_target(scene, t.pose)
        if kinds[t.name] == KIND_REACHABLE:
            assert c.reachable and c.approach_free, (name, t.name, c)
            reach.append(t)
        else:
            assert not c.reachable, (name, t.name)
            unreach.append(t)
    assert [t.name for t in reach] == ["Pick A", "Place B", "Home-side C"]
    assert [t.name for t in unreach] == ["Too far", "Too close"]
    # the three are far apart (distinct, visible moves)
    pts = np.array([t.pose[:3, 3] for t in reach])
    for i in range(3):
        for j in range(i + 1, 3):
            assert np.linalg.norm(pts[i] - pts[j]) > 0.25, (name, i, j)
    # chain q0 -> A -> B -> C -> A with the planner, every sample collision free
    q = list(scene.q0)
    world = scene.session.world
    cm = scene.cm
    for T in [reach[0].pose, reach[1].pose, reach[2].pose, reach[0].pose]:
        best = rank_solutions(scene.model, T, q, scene.session, cm=cm).best()
        assert best is not None
        res = plan_target_motion(scene.model, q, best.q, T, world, cm=cm)
        assert res.ok, (name, res.message)
        traj = res.trajectory
        ts = np.append(np.arange(0.0, traj.duration, 0.02), traj.duration)
        qs = np.array([traj.q(t) for t in ts])
        assert not np.asarray(world.check_batch(qs)).any(), name
        tcp = np.asarray(cb.fk_compiled(cm, list(traj.q(traj.duration)))["tcp"])
        assert np.linalg.norm(tcp[:3, 3] - T[:3, 3]) < 1e-3
        q = list(best.q)
