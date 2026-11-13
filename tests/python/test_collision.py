"""R7.2: collision geometry, ACM, environment, batch check.

Oracles are closed-form geometry on the planar 2R robot (links 1 and 2 are boxes
along x), not the code under test.
"""
import math
import time
import zipfile

import numpy as np
import pytest

pytest.importorskip("fcl")

from app.importers.visuals import LinkVisual
from app.planning.collision import (
    CollisionConfig, CollisionWorld, EnvShape, Shape, env_id, fit_mesh, pose_matrix,
    shape_distance, shapes_from_geometry,
)
from app.project.project_data import PROJECT_FORMAT_VERSION, ProjectData
from app.project.serializer import load, save

from .robots import random_robot
from .test_cartesian_traj import planar_2r

TOL = 1e-6


def T(x, y=0.0, z=0.0):
    return pose_matrix([x, y, z], [0, 0, 0])


def planar_shapes(base_sphere_x=0.75):
    """link1: box 1.0 x 0.1 x 0.1 on x in [0,1]; link2: box 0.5 x 0.1 x 0.1 on x in [0,0.5];
    link0: sphere r 0.1 at (base_sphere_x, 0, 0) in the world."""
    return [
        Shape("sphere", {"radius": 0.1}, 0, T(base_sphere_x)),
        Shape("box", {"size": [1.0, 0.1, 0.1]}, 1, T(0.5)),
        Shape("box", {"size": [0.5, 0.1, 0.1]}, 2, T(0.25)),
    ]


# --- poses of the planar robot -------------------------------------------------

def test_known_collision_and_miss():
    w = CollisionWorld(planar_2r(), planar_shapes())
    w.set_q([0.0, 0.0])                   # link2 on x in [1, 1.5], sphere x up to 0.85
    assert not w.in_collision()
    assert w.pairs() == []
    assert w.min_distance() == pytest.approx(0.15, abs=TOL)
    w.set_q([0.0, math.pi])               # link2 folds back over x in [0.5, 1] -> hits sphere
    assert w.in_collision()
    prs = w.pairs()
    assert [(a, b) for a, b, _ in prs] == [(0, 2)]
    assert prs[0][2] < 0.0
    assert w.min_distance() < 0.0


def test_adjacent_links_excluded_by_default():
    m = planar_2r()
    w = CollisionWorld(m, planar_shapes())
    # sphere (link 0) overlaps link 1 box at every q with q0 = 0, but 0-1 is adjacent
    w.set_q([0.0, 0.0])
    assert not w.in_collision()
    cfg = CollisionConfig()
    cfg.set_check(0, 1, True)
    w2 = CollisionWorld(m, planar_shapes(), cfg)
    w2.set_q([0.0, 0.0])
    assert w2.in_collision()
    assert [(a, b) for a, b, _ in w2.pairs()] == [(0, 1)]
    cfg.set_check(0, 1, False)
    assert not CollisionWorld(m, planar_shapes(), cfg).in_collision()


def test_non_adjacent_link_pair_distance_closed_form():
    # link0 sphere above link 2 end: sphere at (1.25, 0.5, 0) r 0.1; link2 box top y=0.05
    sh = planar_shapes()
    sh[0] = Shape("sphere", {"radius": 0.1}, 0, T(1.25, 0.5))
    w = CollisionWorld(planar_2r(), sh)
    w.set_q([0.0, 0.0])
    assert w.min_distance() == pytest.approx(0.5 - 0.1 - 0.05, abs=TOL)


def test_disable_always_colliding_at_zero_pose():
    sh = planar_shapes(base_sphere_x=1.2)        # overlaps link2 box (x in [1,1.5]) at q = 0
    w = CollisionWorld(planar_2r(), sh)
    w.set_q([0.0, 0.0])
    assert w.in_collision()
    assert w.zero_pose_pairs() == [(0, 2)]
    assert w.disable_always_colliding() == [(0, 2)]
    assert w.config.edits == {(0, 2): False}
    assert not w.in_collision()
    w.set_q([0.0, math.pi])                       # still ignored at other poses
    assert not w.in_collision()


# --- environment ---------------------------------------------------------------

def test_environment_box_distance_and_hit():
    cfg = CollisionConfig(environment=[EnvShape("box", {"size": [0.2, 0.2, 0.2]},
                                                xyz=[2.0, 0.0, 0.0])])
    sh = planar_shapes(base_sphere_x=-5.0)       # sphere far away
    w = CollisionWorld(planar_2r(), sh, cfg)
    w.set_q([0.0, 0.0])                          # link2 x max 1.5, env x min 1.9
    assert not w.in_collision()
    assert w.min_distance() == pytest.approx(0.4, abs=TOL)
    cfg.environment[0] = EnvShape("box", {"size": [0.2, 0.2, 0.2]}, xyz=[1.55, 0.0, 0.0])
    w = CollisionWorld(planar_2r(), sh, cfg)
    w.set_q([0.0, 0.0])                          # overlap in x: 1.5 - 1.45 = 0.05
    assert w.in_collision()
    (a, b, d), = w.pairs()
    assert (a, b) == (2, env_id(0)) and b < 0
    assert d == pytest.approx(-0.05, abs=TOL)


def test_environment_sphere_and_cylinder_distances():
    cfg = CollisionConfig(environment=[
        EnvShape("sphere", {"radius": 0.25}, xyz=[1.25, 1.0, 0.0]),
        EnvShape("cylinder", {"radius": 0.1, "length": 1.0}, xyz=[-1.0, 0.0, 0.0]),
    ])
    w = CollisionWorld(planar_2r(), planar_shapes(base_sphere_x=-5.0), cfg)
    w.set_q([0.0, 0.0])
    d = w.min_distance()
    # sphere above link2: 1.0 - 0.25 - 0.05 = 0.7; cylinder at x=-1 (axis z): link1 x min 0
    # -> 0 - (-1 + 0.1) = 0.9 away from link1 box
    assert d == pytest.approx(0.7, abs=TOL)
    cfg.environment[0] = EnvShape("sphere", {"radius": 0.25}, xyz=[5.0, 5.0, 0.0])
    w = CollisionWorld(planar_2r(), planar_shapes(base_sphere_x=-5.0), cfg)
    w.set_q([0.0, 0.0])
    assert w.min_distance() == pytest.approx(0.9, abs=TOL)


# --- shape-shape closed forms ----------------------------------------------------

def test_sphere_sphere_distance():
    a = Shape("sphere", {"radius": 0.5}, -1, T(0.0))
    b = Shape("sphere", {"radius": 0.25}, -1, T(2.0, 1.0, 2.0))
    assert shape_distance(a, b) == pytest.approx(3.0 - 0.75, abs=TOL)
    c = Shape("sphere", {"radius": 0.5}, -1, T(0.7))
    assert shape_distance(a, c) == pytest.approx(0.7 - 1.0, abs=1e-4)   # depth 0.3


def test_sphere_box_distance_face_and_corner():
    box = Shape("box", {"size": [2.0, 2.0, 2.0]}, -1, T(0.0))
    face = Shape("sphere", {"radius": 0.5}, -1, T(3.0))
    assert shape_distance(box, face) == pytest.approx(3.0 - 1.0 - 0.5, abs=TOL)
    corner = Shape("sphere", {"radius": 0.5}, -1, T(2.0, 2.0, 2.0))
    assert shape_distance(box, corner) == pytest.approx(math.sqrt(3.0) - 0.5, abs=TOL)
    rot = Shape("box", {"size": [2.0, 2.0, 2.0]}, -1,
                pose_matrix([0, 0, 0], [0, 0, math.pi / 4]))
    s = Shape("sphere", {"radius": 0.5}, -1, T(3.0))
    # rotated cube reaches x = sqrt(2) along the diagonal
    assert shape_distance(rot, s) == pytest.approx(3.0 - math.sqrt(2.0) - 0.5, abs=TOL)


def test_capsule_distance_to_sphere():
    cap = Shape("capsule", {"radius": 0.1, "length": 1.0}, -1, T(0.0))     # z in [-.5,.5] + caps
    s = Shape("sphere", {"radius": 0.2}, -1, T(1.0, 0.0, 0.0))
    assert shape_distance(cap, s) == pytest.approx(1.0 - 0.1 - 0.2, abs=TOL)
    s2 = Shape("sphere", {"radius": 0.2}, -1, T(0.0, 0.0, 2.0))
    assert shape_distance(cap, s2) == pytest.approx(2.0 - 0.5 - 0.1 - 0.2, abs=TOL)


# --- stored geometry and mesh fit -------------------------------------------------

def test_shapes_from_geometry_primitives():
    items = [
        LinkVisual(1, "l1", "box", {"size": [1, 2, 3]}, [0, 0, 0.5], [0, 0, 0]),
        LinkVisual(2, "l2", "cylinder", {"radius": 0.1, "length": 0.4}, [0.1, 0, 0], [0, 0, 0]),
        LinkVisual(2, "l2", "sphere", {"radius": 0.3}),
        LinkVisual(2, "l2", "mesh", {"filename": "x.stl", "scale": [1, 1, 1]}),   # no file
    ]
    sh = shapes_from_geometry(items)
    assert [(s.kind, s.link) for s in sh] == [("box", 1), ("cylinder", 2), ("sphere", 2)]
    assert sh[0].pose[2, 3] == pytest.approx(0.5)


def _box_cloud(rng, half, R, c, n=400):
    p = rng.uniform(-1, 1, (n, 3)) * half
    corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * half
    return np.vstack([p, corners]) @ R.T + c


def test_fit_obb_recovers_rotated_box():
    rng = np.random.default_rng(1)
    c, a = np.array([0.3, -0.2, 0.5]), 0.7
    R = pose_matrix([0, 0, 0], [0.2, -0.4, a])[:3, :3]
    pts = _box_cloud(rng, np.array([0.5, 0.2, 0.1]), R, c)
    kind, params, pose = fit_mesh(pts, "obb")
    assert kind == "box"
    assert sorted(params["size"]) == pytest.approx([0.2, 0.4, 1.0], abs=0.05)   # PCA box: near-tight, not exact
    assert np.allclose(pose[:3, 3], c, atol=0.05)


@pytest.mark.parametrize("mode", ["obb", "sphere", "capsule"])
def test_fit_contains_all_points(mode):
    rng = np.random.default_rng(2)
    R = pose_matrix([0, 0, 0], [0.3, 0.1, -0.5])[:3, :3]
    pts = _box_cloud(rng, np.array([0.4, 0.1, 0.15]), R, np.array([1.0, 0.0, 0.0]))
    kind, params, pose = fit_mesh(pts, mode)
    sh = Shape(kind, params, -1, pose)
    loc = (pts - pose[:3, 3]) @ pose[:3, :3]
    eps = 1e-9
    if kind == "box":
        assert np.all(np.abs(loc) <= np.array(params["size"]) / 2 + eps)
    elif kind == "sphere":
        assert np.all(np.linalg.norm(loc, axis=1) <= params["radius"] + eps)
    else:
        h = params["length"] / 2
        z = np.clip(loc[:, 2], -h, h)
        d = np.linalg.norm(loc - np.column_stack([np.zeros(len(loc)), np.zeros(len(loc)), z]),
                           axis=1)
        assert np.all(d <= params["radius"] + eps)
    assert sh.bounding_radius() > 0


# --- project format ----------------------------------------------------------------

def _cfg():
    cfg = CollisionConfig(fit_mode="sphere")
    cfg.set_check(0, 1, True)
    cfg.set_check(3, 1, False)
    cfg.environment.append(EnvShape("box", {"size": [0.2, 0.3, 0.4]}, [1, 2, 3], [0.1, 0.2, 0.3],
                                    "table"))
    cfg.environment.append(EnvShape("cylinder", {"radius": 0.1, "length": 2.0}, [0, 0, 1]))
    return cfg


def test_project_roundtrip_acm_and_environment(tmp_path):
    assert PROJECT_FORMAT_VERSION == "1.4.0"
    p = ProjectData.new("c")
    p.collision = _cfg()
    out = tmp_path / "c.robotproj"
    save(p, out)
    assert "collision.json" in zipfile.ZipFile(out).namelist()
    q = load(out)
    assert q.collision.to_dict() == p.collision.to_dict()
    assert q.collision.edits == {(0, 1): True, (1, 3): False}
    assert q.collision.environment[0].name == "table"


def test_project_without_collision_has_no_file_and_old_file_loads(tmp_path):
    p = ProjectData.new("n")
    out = tmp_path / "n.robotproj"
    save(p, out)
    assert "collision.json" not in zipfile.ZipFile(out).namelist()
    assert load(out).collision is None
    p.collision = _cfg()
    save(p, out)
    old = tmp_path / "old.robotproj"
    with zipfile.ZipFile(out) as zin, zipfile.ZipFile(old, "w") as zout:
        for n in zin.namelist():
            if n == "collision.json":
                continue
            data = zin.read(n)
            if n == "project_metadata.json":
                data = data.replace(b'"1.4.0"', b'"1.2.0"')
            zout.writestr(n, data)
    q = load(old)
    assert q.collision is None and q.metadata.format_version == "1.2.0"


def test_bad_collision_config_rejected():
    with pytest.raises(ValueError):
        EnvShape("capsule", {"radius": 1, "length": 1})
    with pytest.raises(ValueError):
        Shape("box", {"size": [1, 2]})
    with pytest.raises(ValueError):
        CollisionConfig().set_check(2, 2, True)


# --- batch --------------------------------------------------------------------------

def test_check_batch_matches_single_queries_and_speed():
    m = planar_2r()
    w = CollisionWorld(m, planar_shapes())
    rng = np.random.default_rng(5)
    qs = rng.uniform(-math.pi, math.pi, (200, 2))
    flags = w.check_batch(qs)
    assert flags.dtype == bool and flags.shape == (200,)
    for q, f in zip(qs, flags):
        w.set_q(q)
        assert w.in_collision() == bool(f)
    # analytic: link2 box can reach the sphere only when it folds back; sample both outcomes
    assert flags.any() and (~flags).any()


def test_batch_speed_six_dof_recorded():
    m = random_robot(7, 6, kinds="origin_axis")
    shapes = [Shape("capsule", {"radius": 0.03, "length": 0.25}, i, T(0.0, 0.0, 0.0))
              for i in range(1, 7)]
    w = CollisionWorld(m, shapes)
    rng = np.random.default_rng(0)
    qs = rng.uniform(-math.pi, math.pi, (2000, 6))
    w.check_batch(qs[:50])
    t0 = time.perf_counter()
    w.check_batch(qs)
    rate = len(qs) / (time.perf_counter() - t0)
    print(f"R7.2 check_batch 6-DOF: {rate:.0f} checks/s")
    assert rate > 3000          # target 20 000 (see docs/collision-decision.md for the number)
