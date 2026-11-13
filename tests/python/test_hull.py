"""U3.1: convex hull link shapes (app/planning/hull.py, fit_mode "hull")."""
import time

import numpy as np
import pytest

pytest.importorskip("fcl")

from app.importers.urdf import import_urdf
from app.panels.collision_panel import CollisionPanel
from app.planning import hull as H
from app.planning.collision import (
    DEFAULT_FIT_MODE, CollisionConfig, CollisionWorld, Shape, fit_mesh, pose_matrix,
    shape_distance, shapes_from_geometry,
)
from app.sample_robots import list_sample_robots

ROBOTS = list_sample_robots()


def cube_points(h=0.5):
    return np.array([[x, y, z] for x in (-h, h) for y in (-h, h) for z in (-h, h)], float)


def inside(v, f, pts, tol=1e-9):
    a, b, c = v[f[:, 0]], v[f[:, 1]], v[f[:, 2]]
    n = np.cross(b - a, c - a)
    n /= np.linalg.norm(n, axis=1, keepdims=True)
    off = np.einsum("ij,ij->i", n, a)
    return float((pts @ n.T - off).max()) <= tol


def test_default_fit_mode_is_hull():
    assert DEFAULT_FIT_MODE == "hull"
    assert CollisionConfig().fit_mode == "hull"
    assert CollisionConfig().is_default()
    assert CollisionConfig.from_dict({"acm_edits": []}).fit_mode == "obb"   # old file
    assert CollisionConfig.from_dict({"fit_mode": "sphere"}).fit_mode == "sphere"


def test_hull_cube_and_distance():
    v, f = H.convex_hull(cube_points())
    assert len(v) == 8 and inside(v, f, cube_points())
    kind, params, pose = fit_mesh(cube_points(), "hull")
    assert kind == "convex" and np.allclose(pose, np.eye(4))
    cube = Shape(kind, params, 0, np.eye(4))
    sph = Shape("sphere", {"radius": 0.1}, -1, pose_matrix([1.0, 0, 0], [0, 0, 0]))
    assert shape_distance(cube, sph) == pytest.approx(0.4, abs=1e-6)         # 1 - 0.5 - 0.1
    corner = Shape("sphere", {"radius": 0.1}, -1, pose_matrix([1.0, 1.0, 1.0], [0, 0, 0]))
    assert shape_distance(cube, corner) == pytest.approx(3 ** 0.5 * 0.5 - 0.1, abs=1e-6)
    hit = Shape("sphere", {"radius": 0.2}, -1, pose_matrix([0.6, 0, 0], [0, 0, 0]))
    assert shape_distance(cube, hit) < 0.0


def test_hull_contains_all_points_and_vertex_cap():
    rng = np.random.default_rng(3)
    for pts in (rng.normal(size=(5000, 3)),                       # many hull vertices
                rng.normal(size=(300, 3)) * [1.0, 0.2, 0.05]):
        v, f = H.convex_hull(pts)
        assert len(v) <= H.MAX_HULL_VERTICES
        assert inside(v, f, pts)
        assert H.convex_hull(pts, 20)[0].shape[0] <= 20


def test_degenerate_falls_back_to_box():
    flat = np.column_stack([np.random.default_rng(0).random((50, 2)), np.zeros(50)])
    assert H.convex_hull(flat) is None
    kind, _, _ = fit_mesh(flat, "hull")
    assert kind == "box"


def test_hull_cache(tmp_path):
    H.clear_hull_cache()
    calls = []
    p = tmp_path / "m.stl"
    p.write_bytes(b"x")
    key = H.mesh_key(str(p), (1, 1, 1))
    H.cached(key, lambda: calls.append(1) or "a")
    H.cached(key, lambda: calls.append(1) or "b")
    assert calls == [1]
    H.clear_hull_cache()
    H.cached(key, lambda: calls.append(1) or "a")
    assert calls == [1, 1]


@pytest.mark.parametrize("title,path", ROBOTS, ids=[p.parent.name for _, p in ROBOTS])
def test_sample_robot_hull_world(title, path):
    H.clear_hull_cache()
    res = import_urdf(str(path))
    items = list(res.collisions) or list(res.visuals)
    t0 = time.perf_counter()
    shapes = shapes_from_geometry(items, "hull")
    dt = time.perf_counter() - t0
    assert shapes
    n_conv = sum(s.kind == "convex" for s in shapes)
    assert n_conv >= 1
    for s in shapes:
        if s.kind == "convex":
            assert len(s.params["vertices"]) <= H.MAX_HULL_VERTICES
    world = CollisionWorld(res.model, shapes, CollisionConfig())
    world.set_q(np.zeros(res.model.dof()))
    world.pairs()                                   # runs without error
    t0 = time.perf_counter()
    shapes_from_geometry(items, "hull")             # cached
    assert time.perf_counter() - t0 < max(dt, 0.05)
    print(f"{path.parent.name}: {len(shapes)} shapes, {n_conv} hulls, build {dt:.2f} s")


def test_panel_fit_combo(qapp):
    from types import SimpleNamespace
    panel = CollisionPanel()
    assert panel.fit_mode == "hull"
    proj = SimpleNamespace(collision=None, robot_model=None)
    panel.set_project(proj)
    got = []
    panel.changed.connect(lambda: got.append(1))
    assert panel.set_fit_mode("obb")
    assert proj.collision.fit_mode == "obb" and got
    assert panel.fit_mode == "obb"
    assert not panel.set_fit_mode("nope")
