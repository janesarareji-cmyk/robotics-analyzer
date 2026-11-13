"""U3.3: floor collision and the live CollisionService."""
import time
from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip("fcl")

from app.collision_service import CollisionService
from app.importers.urdf import import_urdf
from app.kin_utils import current_q, dof_limits
from app.planning.collision import (
    FLOOR_ID, CollisionConfig, CollisionWorld, EnvShape, shapes_from_geometry,
)
from app.sample_robots import list_sample_robots

SAMPLES = {p.parent.name: p for _, p in list_sample_robots()}


def make_project(name, auto=True):
    res = import_urdf(str(SAMPLES[name]))
    proj = SimpleNamespace(robot_model=res.model, visuals=list(res.visuals),
                           collisions=list(res.collisions), collision=CollisionConfig())
    if auto:
        w = CollisionWorld(res.model, shapes_from_geometry(proj.collisions or proj.visuals),
                           proj.collision)
        w.auto_acm(1000, 0, current_q(res.model))
    return proj


def service(name, auto=True):
    s = CollisionService()
    p = make_project(name, auto)
    s.set_project(p)
    return s, p


def random_q(model, rng, n):
    lo, hi = (np.asarray(v) for v in dof_limits(model))
    return lo + rng.random((n, len(lo))) * (hi - lo)


# -- floor ------------------------------------------------------------------

def test_floor_world_basics():
    res = import_urdf(str(SAMPLES["panda"]))
    shapes = shapes_from_geometry(res.collisions or res.visuals)
    w = CollisionWorld(res.model, shapes, CollisionConfig())
    assert w.floor_z == pytest.approx(-0.001)
    assert 0 not in {shapes[i].link for i in w._floor_idx}       # base link is static: skipped
    off = CollisionWorld(res.model, shapes, CollisionConfig(floor=False))
    assert off.floor_z is None
    rng = np.random.default_rng(0)
    hits = 0
    for q in random_q(res.model, rng, 200):
        w.set_q(q)
        off.set_q(q)
        fl = [p for p in w.pairs() if p[1] == FLOOR_ID]
        hits += bool(fl)
        assert not [p for p in off.pairs() if p[1] == FLOOR_ID]
        assert w.in_collision() or not fl
    assert hits > 0, "some random panda poses must touch the floor"


def test_floor_config_roundtrip():
    cfg = CollisionConfig(floor=False)
    assert not cfg.is_default()
    assert CollisionConfig.from_dict(cfg.to_dict()).floor is False
    assert CollisionConfig.from_dict({"acm_edits": []}).floor is True      # old file: floor on
    assert CollisionConfig().is_default() and CollisionConfig().floor


# -- service -----------------------------------------------------------------

def test_check_disabled_and_no_project():
    s = CollisionService()
    assert s.check([0.0] * 3).ok and s.world is None
    s, p = service("panda")
    rng = np.random.default_rng(1)
    qs = random_q(p.robot_model, rng, 100)
    bad = [q for q in qs if not s.check(q, emit=False).ok]
    assert bad
    s.enabled = False
    assert all(s.check(q).ok for q in bad)
    assert s.first_free(qs[0], bad[0]) == list(map(float, bad[0]))


def test_signal_only_on_change():
    s, p = service("panda")
    got = []
    s.collision_changed.connect(got.append)
    rng = np.random.default_rng(2)
    qs = random_q(p.robot_model, rng, 100)
    bad = next(q for q in qs if not s.check(q, emit=False).ok)
    good = next(q for q in qs if s.check(q, emit=False).ok)
    s.check(good)
    assert got == []
    s.check(bad)
    s.check(bad)
    assert len(got) == 1 and not got[0].ok and got[0].text()
    s.check(good)
    assert len(got) == 2 and got[1].ok


@pytest.mark.parametrize("name", ["xarm6", "panda"])
def test_check_speed(name):
    s, p = service(name)
    rng = np.random.default_rng(3)
    qs = random_q(p.robot_model, rng, 300)
    s.check(qs[0], emit=False)
    ts = []
    for q in qs:
        t0 = time.perf_counter()
        s.check(q, emit=False)
        ts.append((time.perf_counter() - t0) * 1e3)
    free = [q for q in qs if s.is_free(q)]
    tf = []
    for q in free:
        t0 = time.perf_counter()
        s.check(q, emit=False)
        tf.append((time.perf_counter() - t0) * 1e3)
    print(f"{name}: check all median {np.median(ts):.3f} p95 {np.percentile(ts, 95):.3f} ms; "
          f"free-only median {np.median(tf):.3f} p95 {np.percentile(tf, 95):.3f} ms; "
          f"{len(free)}/{len(qs)} free")
    assert np.median(ts) < 2.0 and np.median(tf) < 2.0


def test_first_free_stops_at_contact():
    s, p = service("panda")
    rng = np.random.default_rng(4)
    a = np.asarray(current_q(p.robot_model))
    assert s.is_free(a)
    qs = random_q(p.robot_model, rng, 400)
    n_checked = 0
    for b in qs:
        if s.is_free(b):
            continue
        q = np.asarray(s.first_free(a, b))
        assert s.is_free(q)
        assert s.last_block is not None and not s.last_block.ok
        # the contact is right behind q: a fine scan along the move finds it within 2e-3 rad
        span = float(np.abs(b - a).max())
        t_q = float(np.abs(q - a).max() / span)
        ts = np.linspace(t_q, 1.0, 2000)
        t_hit = next(t for t in ts if not s.is_free(a + t * (b - a)))
        assert (t_hit - t_q) * span < 2e-3 + span / 2000
        n_checked += 1
        if n_checked >= 6:
            break
    assert n_checked >= 3


def test_free_move_returns_target():
    s, p = service("ur5")
    a = np.asarray(current_q(p.robot_model))
    a[1] -= 0.4                                  # wrist above the floor
    b = a + 0.01
    assert s.is_free(a) and s.is_free(b)
    assert s.first_free(a, b) == pytest.approx(list(b))
    assert s.last_block is None


def test_start_in_contact_can_leave():
    """ur5 zero pose has the wrist under the base plane; moving away must stay possible."""
    s, p = service("ur5")
    a = np.asarray(current_q(p.robot_model))
    r = s.check(a, emit=False)
    assert not r.ok and any(b == FLOOR_ID for _, b, _ in r.pairs)
    b = a.copy()
    b[1] -= 0.4                                  # shoulder lifts: wrist goes up, away from the floor
    out = s.first_free(a, b)
    assert out == pytest.approx(list(b))


def test_environment_object_blocks():
    s, p = service("ur5")
    a = np.asarray(current_q(p.robot_model))
    b = a.copy()
    b[1] -= 0.4
    assert s.first_free(a, b) == pytest.approx(list(b))
    from app.planning.collision import pose_matrix  # noqa: F401
    w = s.world
    w.set_q(b)
    # a 0.2 m box exactly at the wrist_3 link origin at pose b
    import core_bridge as cb
    fr = np.asarray(cb.fk_compiled(w._cm, list(b))["frames"])
    pos = fr[-1][:3, 3]
    p.collision.environment.append(EnvShape("box", {"size": [0.2, 0.2, 0.2]}, [float(x) for x in pos],
                                            [0, 0, 0], "block"))
    s.invalidate()
    r = s.check(b, emit=False)
    assert not r.ok and any("block" in n for pair in r.names for n in pair)
    assert r.env_ids == {0}
    out = np.asarray(s.first_free(a, b))
    assert s.is_free(out) and np.abs(out - a).max() < np.abs(b - a).max()
    p.collision.environment.clear()
    s.invalidate()
    assert s.is_free(b)


def test_floor_toggle_changes_result():
    s, p = service("panda")
    rng = np.random.default_rng(5)
    q = next(q for q in random_q(p.robot_model, rng, 300)
             if any(b == FLOOR_ID for _, b, _ in s.check(q, emit=False).pairs))
    p.collision.floor = False
    s.invalidate()
    assert not any(b == FLOOR_ID for _, b, _ in s.check(q, emit=False).pairs)
