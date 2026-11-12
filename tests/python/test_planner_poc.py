"""R7.3 PoC: 2-DOF RRT-Connect around a circle obstacle (evidence for own-planner decision).

OMPL has no wheel for this Python (see docs/planner-decision.md); the OMPL PoC is
skipped when OMPL is missing.
"""
import importlib.util
import math
import time

import numpy as np
import pytest

CENTER = np.array([0.0, 0.0])
RADIUS = 0.8
LO, HI = -2.0, 2.0
STEP = 0.2


def _free(q):
    return float(np.linalg.norm(q - CENTER)) > RADIUS


def _edge_free(a, b):
    n = max(2, int(np.linalg.norm(b - a) / 0.02))
    return all(_free(a + (b - a) * t) for t in np.linspace(0.0, 1.0, n + 1))


def _extend(tree, parent, q_target):
    i = _nearest(tree, q_target)
    near = tree[i]
    d = q_target - near
    dist = float(np.linalg.norm(d))
    q_new = q_target if dist <= STEP else near + d / dist * STEP
    if not _edge_free(near, q_new):
        return None, False
    tree.append(q_new)
    parent.append(i)
    return len(tree) - 1, bool(np.allclose(q_new, q_target))


def _nearest(tree, q):
    return int(np.argmin([np.linalg.norm(t - q) for t in tree]))


def _path(tree, parent, i):
    out = []
    while i != -1:
        out.append(tree[i])
        i = parent[i]
    return out[::-1]


def rrt_connect(start, goal, seed=0, max_iter=5000):
    rng = np.random.default_rng(seed)
    ta, pa = [start], [-1]
    tb, pb = [goal], [-1]
    swapped = False
    for _ in range(max_iter):
        q = rng.uniform(LO, HI, 2)
        ia, _ = _extend(ta, pa, q)
        if ia is not None:
            while True:
                ib, reached = _extend(tb, pb, ta[ia])
                if ib is None or reached:
                    break
            if ib is not None and reached:
                pa_path = _path(ta, pa, ia)
                pb_path = _path(tb, pb, ib)[::-1]
                path = pa_path + pb_path[1:] if np.allclose(pa_path[-1], pb_path[0]) else pa_path + pb_path
                return path[::-1] if swapped else path
        ta, pa, tb, pb = tb, pb, ta, pa
        swapped = not swapped
    return None


def test_rrt_connect_2dof_circle():
    start = np.array([-1.5, 0.0])
    goal = np.array([1.5, 0.0])
    t0 = time.perf_counter()
    path = rrt_connect(start, goal)
    dt = time.perf_counter() - t0
    assert path is not None
    assert np.allclose(path[0], start) and np.allclose(path[-1], goal)
    assert all(_free(q) for q in path)
    assert all(_edge_free(a, b) for a, b in zip(path[:-1], path[1:]))
    length = sum(float(np.linalg.norm(b - a)) for a, b in zip(path[:-1], path[1:]))
    assert length >= 3.0 - 1e-9
    print(f"RRT-Connect PoC: {dt*1e3:.1f} ms, {len(path)} waypoints, length {length:.3f}")


@pytest.mark.skipif(importlib.util.find_spec("ompl") is None, reason="OMPL not installed")
def test_ompl_rrt_connect_2dof():
    from ompl import base as ob, geometric as og  # pragma: no cover

    space = ob.RealVectorStateSpace(2)  # pragma: no cover
    b = ob.RealVectorBounds(2)
    b.setLow(LO)
    b.setHigh(HI)
    space.setBounds(b)
    si = ob.SpaceInformation(space)
    si.setStateValidityChecker(ob.StateValidityCheckerFn(lambda s: _free(np.array([s[0], s[1]]))))
    si.setup()
    pdef = ob.ProblemDefinition(si)
    s, g = ob.State(space), ob.State(space)
    s[0], s[1], g[0], g[1] = -1.5, 0.0, 1.5, 0.0
    pdef.setStartAndGoalStates(s, g)
    p = og.RRTConnect(si)
    p.setProblemDefinition(pdef)
    p.setup()
    assert p.solve(2.0)
