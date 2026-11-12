"""R7.4: joint-space path planners (own code, OMPL is not installable, see docs/planner-decision.md).

``plan(model, start_q, goal_q, collision, limits, planner, time_limit, seed) -> PlanResult``.
Planners: ``rrt_connect``, ``rrt_star``, ``prm``. Same seed -> same path (RRT* and PRM stop
on iteration counts, only ``time_limit`` can cut a run short). Planning is inside the joint
limits, there is no revolute wrap. ``collision`` is a ``CollisionWorld`` or ``None``
(joint limits only). A goal may be a joint vector or a 4x4 TCP pose (IK solutions are the goals).
"""
from __future__ import annotations

import heapq
import math
import time
from dataclasses import dataclass, field
from typing import Any, Callable

import numpy as np

import core_bridge as cb
from app.kin_utils import dof_limits
from app.planning.shortcut import path_length, shortcut

PLANNERS = ("rrt_connect", "rrt_star", "prm")
DEFAULT_RESOLUTION = 0.02       # rad, max joint-space step of an edge check
DEFAULT_STEP = 0.3              # rad, extension step of the tree planners
PROGRESS_EVERY = 16             # iterations between progress callbacks


@dataclass
class PlanResult:
    path: np.ndarray | None          # (n, dof) waypoints, start first, goal last; None on failure
    success: bool
    time: float
    iterations: int
    reason: str = ""
    planner: str = ""
    length: float = 0.0
    raw_length: float = 0.0          # before shortcutting
    goal_index: int = -1
    info: dict = field(default_factory=dict)


class _Cancelled(Exception):
    pass


class _Timeout(Exception):
    pass


def _cancel_fn(cancel) -> Callable[[], bool]:
    if cancel is None:
        return lambda: False
    if callable(cancel):
        return lambda: bool(cancel())
    if hasattr(cancel, "is_set"):
        return lambda: bool(cancel.is_set())
    if hasattr(cancel, "is_cancelled"):
        return lambda: bool(cancel.is_cancelled)
    raise TypeError("cancel must be a callable, an Event or have is_cancelled")


class _Tree:
    """Growing node array with nearest / radius queries."""

    def __init__(self, dof: int):
        self.q = np.empty((64, dof))
        self.parent: list[int] = []
        self.n = 0

    def add(self, q, parent: int = -1) -> int:
        if self.n == len(self.q):
            self.q = np.concatenate([self.q, np.empty_like(self.q)])
        self.q[self.n] = q
        self.parent.append(parent)
        self.n += 1
        return self.n - 1

    def nearest(self, q) -> int:
        d = self.q[:self.n] - q
        return int(np.argmin(np.einsum("ij,ij->i", d, d)))

    def dists(self, q) -> np.ndarray:
        return np.linalg.norm(self.q[:self.n] - q, axis=1)

    def path_to_root(self, i: int) -> list[np.ndarray]:
        out = []
        while i != -1:
            out.append(self.q[i].copy())
            i = self.parent[i]
        return out


class _Space:
    """Limits, edge validity (dense interpolation) and run control."""

    def __init__(self, model, collision, lo, hi, resolution, cancel, time_limit, t0):
        self.model, self.collision = model, collision
        self.lo, self.hi = np.asarray(lo, float), np.asarray(hi, float)
        self.dof = len(self.lo)
        self.res = float(resolution)
        self._cancel = cancel
        self.deadline = t0 + float(time_limit)
        self.checks = 0

    def check(self) -> None:
        if self._cancel():
            raise _Cancelled
        if time.perf_counter() > self.deadline:
            raise _Timeout

    def in_limits(self, q) -> bool:
        return bool(np.all(q >= self.lo - 1e-12) and np.all(q <= self.hi + 1e-12))

    def state_free(self, q) -> bool:
        if not self.in_limits(q):
            return False
        if self.collision is None:
            return True
        self.checks += 1
        return not bool(self.collision.check_batch(np.asarray(q, float)[None, :])[0])

    def edge_free(self, a, b) -> bool:
        """Both ends and every point at spacing <= resolution are in limits and collision free."""
        a, b = np.asarray(a, float), np.asarray(b, float)
        if not (self.in_limits(a) and self.in_limits(b)):
            return False
        if self.collision is None:
            return True
        n = max(1, int(math.ceil(float(np.linalg.norm(b - a)) / self.res)))
        ts = np.linspace(0.0, 1.0, n + 1)
        # far end first, then the rest in two chunks (early exit on the usual blocked edge)
        order = [ts[-1:], ts[:-1:2], ts[1:-1:2]]
        for chunk in order:
            if len(chunk) == 0:
                continue
            pts = a + (b - a) * chunk[:, None]
            self.checks += len(pts)
            if bool(np.any(self.collision.check_batch(pts))):
                return False
        return True

    def sample(self, rng) -> np.ndarray:
        return rng.uniform(self.lo, self.hi)

    def steer(self, a, b, step):
        d = b - a
        dist = float(np.linalg.norm(d))
        if dist <= step:
            return b.copy(), True
        return a + d / dist * step, False


# ---------------------------------------------------------------------------
# planners; each returns (path | None, goal_index, iterations, reason)
# ---------------------------------------------------------------------------

def _rrt_connect(sp: _Space, start, goals, rng, step, progress):
    ts_, tg = _Tree(sp.dof), _Tree(sp.dof)
    ts_.add(start)
    root_of_goal = {}
    for k, g in enumerate(goals):
        root_of_goal[tg.add(g)] = k
    it = 0
    a_is_start = True
    ta, tb = ts_, tg
    while True:
        sp.check()
        it += 1
        if progress and it % PROGRESS_EVERY == 0:
            progress(it, "rrt_connect")
        q = sp.sample(rng)
        i = ta.nearest(q)
        q_new, _ = sp.steer(ta.q[i], q, step)
        if sp.edge_free(ta.q[i], q_new):
            ia = ta.add(q_new, i)
            # greedy connect of the other tree
            ib = -1
            reached = False
            while True:
                sp.check()
                j = tb.nearest(ta.q[ia])
                q2, reached = sp.steer(tb.q[j], ta.q[ia], step)
                if not sp.edge_free(tb.q[j], q2):
                    break
                ib = tb.add(q2, j)
                if reached:
                    break
            if reached and ib >= 0:
                pa = ta.path_to_root(ia)[::-1]           # root(a) ... joint
                pb = tb.path_to_root(ib)                 # joint ... root(b)
                # joint nodes coincide (reached): drop the duplicate
                full = pa + pb[1:]
                root_b = tb.path_to_root(ib)[-1]
                if a_is_start:
                    path, gk = full, _goal_of(root_b, goals)
                else:
                    path, gk = full[::-1], _goal_of(pa[0], goals)
                return np.array(path), gk, it, ""
        ta, tb = tb, ta
        a_is_start = not a_is_start


def _goal_of(q, goals) -> int:
    d = [float(np.linalg.norm(q - g)) for g in goals]
    return int(np.argmin(d))


def _rrt_star(sp: _Space, start, goals, rng, step, progress, extra_iters=300, max_neighbors=12):
    d = sp.dof
    span = float(np.max(sp.hi - sp.lo))
    t = _Tree(d)
    t.add(start)
    cost = [0.0]
    children: list[list[int]] = [[]]
    goal_links: dict[int, int] = {}        # tree node -> goal index it connects to
    it = 0
    first = None

    def set_parent(i, p, c):
        old = t.parent[i]
        if old >= 0:
            children[old].remove(i)
        t.parent[i] = p
        children[p].append(i)
        delta = c - cost[i]
        stack = [i]
        while stack:
            k = stack.pop()
            cost[k] += delta if k != i else delta
            stack.extend(children[k])

    def best():
        bestc, bi, bg = math.inf, -1, -1
        for node, gk in goal_links.items():
            c = cost[node] + float(np.linalg.norm(goals[gk] - t.q[node]))
            if c < bestc:
                bestc, bi, bg = c, node, gk
        return bestc, bi, bg

    while True:
        try:
            sp.check()
        except _Timeout:
            if first is None:
                raise
            break
        it += 1
        if progress and it % PROGRESS_EVERY == 0:
            progress(it, "rrt_star")
        if first is not None and it - first >= extra_iters:
            break
        if rng.random() < 0.1:
            q = goals[int(rng.integers(len(goals)))]
        else:
            q = sp.sample(rng)
        i = t.nearest(q)
        q_new, _ = sp.steer(t.q[i], q, step)
        if float(np.linalg.norm(q_new - t.q[i])) < 1e-9:
            continue
        if not sp.edge_free(t.q[i], q_new):
            continue
        n = t.n
        radius = float(np.clip(1.5 * span * (math.log(n + 1) / (n + 1)) ** (1.0 / d),
                               step, 3.0 * step))
        dd = t.dists(q_new)
        near = np.where(dd <= radius)[0]
        near = near[np.argsort(dd[near])][:max_neighbors]
        best_p, best_c = i, cost[i] + float(np.linalg.norm(q_new - t.q[i]))
        for k in near:
            k = int(k)
            c = cost[k] + float(dd[k])
            if c < best_c - 1e-12 and sp.edge_free(t.q[k], q_new):
                best_p, best_c = k, c
        new = t.add(q_new, best_p)
        cost.append(best_c)
        children.append([])
        children[best_p].append(new)
        for k in near:                                   # rewire
            k = int(k)
            if k == best_p or k == new:
                continue
            c = best_c + float(dd[k])
            if c < cost[k] - 1e-12 and sp.edge_free(q_new, t.q[k]):
                # k may not be an ancestor of new (new's ancestors have lower cost)
                set_parent(k, new, c)
        for gk, g in enumerate(goals):                   # connect to goals
            if float(np.linalg.norm(g - q_new)) <= 2.0 * step and sp.edge_free(q_new, g):
                goal_links[new] = gk
                if first is None:
                    first = it
        if goal_links and first is None:
            first = it
    c, node, gk = best()
    if node < 0:
        return None, -1, it, "no path found"
    path = t.path_to_root(node)[::-1] + [goals[gk].copy()]
    return np.array(path), gk, it, ""


def _prm(sp: _Space, start, goals, rng, step, progress, batch=40, k_nn=6, max_nodes=20000):
    d = sp.dof
    t = _Tree(d)
    t.add(start)
    gidx = {}
    for k, g in enumerate(goals):
        gidx[t.add(g)] = k
    adj: list[dict[int, float]] = [dict() for _ in range(t.n)]
    it = 0
    tried: set = set()

    def connect(i, upto):
        dd = t.dists(t.q[i])[:upto]
        order = np.argsort(dd)
        made = 0
        for j in order:
            j = int(j)
            if j == i or j in adj[i] or (min(i, j), max(i, j)) in tried:
                continue
            if made >= k_nn:
                break
            if dd[j] > 4.0 * step:
                break
            tried.add((min(i, j), max(i, j)))
            if sp.edge_free(t.q[i], t.q[j]):
                adj[i][j] = adj[j][i] = float(dd[j])
            made += 1

    def route():
        dist = {0: 0.0}
        prev = {}
        heap = [(0.0, 0)]
        while heap:
            c, u = heapq.heappop(heap)
            if c > dist.get(u, math.inf):
                continue
            if u in gidx:
                path = [u]
                while path[-1] != 0:
                    path.append(prev[path[-1]])
                return path[::-1], gidx[u]
            for v, w in adj[u].items():
                nc = c + w
                if nc < dist.get(v, math.inf):
                    dist[v], prev[v] = nc, u
                    heapq.heappush(heap, (nc, v))
        return None, -1

    # start and goals connect to each other first (adjacency grows as nodes arrive)
    for i in range(1, t.n):
        connect(i, t.n)
    while True:
        sp.check()
        added = 0
        while added < batch:
            it += 1
            if progress and it % PROGRESS_EVERY == 0:
                progress(it, "prm")
            q = sp.sample(rng)
            if not sp.state_free(q):
                if it > 50 * max_nodes:
                    return None, -1, it, "no free sample"
                continue
            i = t.add(q)
            adj.append({})
            connect(i, t.n)
            added += 1
            if t.n >= max_nodes:
                break
        # re-connect start / goal nodes to the new nodes
        for i in range(0, 1 + len(goals)):
            connect(i, t.n)
        nodes, gk = route()
        if nodes is not None:
            return np.array([t.q[i] for i in nodes]), gk, it, ""
        if t.n >= max_nodes:
            return None, -1, it, "roadmap full, no path"


_RUN = {"rrt_connect": _rrt_connect, "rrt_star": _rrt_star, "prm": _prm}


# ---------------------------------------------------------------------------
# public API
# ---------------------------------------------------------------------------

def _limits_arrays(model, limits):
    if limits is None:
        lo, hi = dof_limits(model)
    elif isinstance(limits, dict):
        lo, hi = limits.get("pos_lo"), limits.get("pos_hi")
    else:
        lo, hi = limits
    lo, hi = np.array(lo, float), np.array(hi, float)
    bad = ~(lo < hi)            # no limit (or lo == hi == 0 default): sample in +-pi
    lo[bad], hi[bad] = -math.pi, math.pi
    return lo, hi


def ik_goals(model, pose, lo, hi, q_ref, n_starts: int = 30, seed: int = 42,
             position_only: bool = False) -> list[np.ndarray]:
    """IK solutions of a 4x4 TCP ``pose`` (distinct, inside the limits), nearest to ``q_ref`` first."""
    cm = cb.compile(model)
    sols = cb.solve_ik_multi(cm, list(lo), list(hi), np.asarray(pose, float), list(q_ref),
                             {"position_only": bool(position_only)}, int(n_starts), int(seed))
    return [np.asarray(s["q"], float) for s in sols]


def plan(model, start_q, goal_q, collision=None, limits=None, planner: str = "rrt_connect",
         time_limit: float = 5.0, seed: int = 0, resolution: float = DEFAULT_RESOLUTION,
         step: float = DEFAULT_STEP, cancel=None, progress=None, do_shortcut: bool = True,
         shortcut_iterations: int = 100, ik_starts: int = 30) -> PlanResult:
    """Plan a collision free joint-space path ``start_q -> goal_q``.

    ``goal_q``: joint vector, or a 4x4 TCP pose (IK solutions become the goals; the first
    collision free one reached wins). ``limits``: ``(lo, hi)``, a dict with ``pos_lo``/``pos_hi``
    or None (URM joint limits; missing limit -> +-pi sampling range). ``cancel``: callable,
    ``threading.Event`` or object with ``is_cancelled``. ``progress(iteration, planner_name)`` is
    called every 16 iterations. Never raises for planning failures: ``success=False`` + ``reason``.
    """
    t0 = time.perf_counter()
    name = str(planner)
    if name not in _RUN:
        raise ValueError(f"unknown planner {planner!r}, use one of {PLANNERS}")
    if resolution <= 0 or step <= 0 or time_limit <= 0:
        raise ValueError("resolution, step and time_limit must be positive")
    n = model.dof()
    lo, hi = _limits_arrays(model, limits)
    cancel_fn = _cancel_fn(cancel)

    def fail(reason, it=0):
        return PlanResult(None, False, time.perf_counter() - t0, it, reason, name)

    start = np.asarray(start_q, float)
    if start.shape != (n,):
        raise ValueError(f"start_q needs {n} values")
    sp = _Space(model, collision, lo, hi, resolution, cancel_fn, time_limit, t0)
    if not sp.in_limits(start):
        return fail("start outside joint limits")
    if not sp.state_free(start):
        return fail("start in collision")

    g = np.asarray(goal_q, float)
    if g.shape == (4, 4):
        try:
            cands = ik_goals(model, g, lo, hi, start, ik_starts, seed)
        except Exception as e:                                   # bad pose / IK error
            return fail(f"IK failed: {e}")
        if not cands:
            return fail("no IK solution for the goal pose")
        goals = [c for c in cands if sp.state_free(c)]
        if not goals:
            return fail("all IK solutions are in collision or outside the limits")
    elif g.shape == (n,):
        if not sp.in_limits(g):
            return fail("goal outside joint limits")
        if not sp.state_free(g):
            return fail("goal in collision")
        goals = [g]
    else:
        raise ValueError(f"goal_q needs {n} values or a 4x4 pose")

    rng = np.random.default_rng(int(seed))
    prog = (lambda it, nm: progress(it, nm)) if progress else None
    iterations = 0
    path = None
    reason = ""
    gk = -1
    try:
        # trivial case: straight line to the nearest goal
        order = sorted(range(len(goals)), key=lambda k: float(np.linalg.norm(goals[k] - start)))
        for k in order:
            sp.check()
            if sp.edge_free(start, goals[k]):
                path, gk = np.array([start, goals[k]]), k
                break
        if path is None:
            path, gk, iterations, reason = _RUN[name](sp, start, goals, rng, step, prog)
    except _Cancelled:
        return fail("cancelled", iterations)
    except _Timeout:
        return fail("time limit reached", iterations)
    if path is None:
        return fail(reason or "no path found", iterations)
    path[0], path[-1] = start, goals[gk]
    raw_len = path_length(path)
    if do_shortcut and len(path) > 2:
        path = shortcut(path, sp.edge_free, rng=rng, iterations=shortcut_iterations,
                        cancel=cancel_fn)
    return PlanResult(path, True, time.perf_counter() - t0, iterations, "", name,
                      path_length(path), raw_len, gk, {"collision_checks": sp.checks})
