"""F1.3: time-optimal path parametrisation with torque limits (TOPP-RA). No Qt.

Method: reachability analysis (Pham & Pham 2018, "A New Approach to Time-Optimal Path
Parameterization based on Reachability Analysis"). The path is ``q(s)`` on a grid
``s_0..s_N``. With ``x = sdot^2`` and ``u = sddot`` every limit is linear in ``(u, x)``::

    velocity      q'_j^2 x <= vmax_j^2
    acceleration  |q'_j u + q''_j x| <= amax_j
    torque        |a_j u + b_j x + c_j| <= effort_j     (RNEA, friction OFF)

with ``c = RNEA(q,0,0)``, ``a = RNEA(q,0,q') - c``, ``b = RNEA(q,q',q'') - c`` (exact: the
Coriolis term is quadratic in ``qd``). Stages are linked by ``x_{i+1} = x_i + 2 ds_i u_i``.
A backward pass finds the controllable intervals ``K_i`` (``K_N = {0}``), a forward greedy pass
takes the largest ``u`` that keeps ``x_{i+1}`` in ``K_{i+1}``. Each step is a 2-variable LP, solved
by ``_lp2d`` (vertex enumeration, no scipy). Friction is not linear in ``x`` and is left out;
``time_parametrize`` re-checks the result with friction (``check_trajectory``).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

import core_bridge as cb

from app.planning.checks import GRAVITY
from app.sim.trajectory import Trajectory, Waypoint

X_BIG = 1.0e4          # (rad/s)^2 cap of x = sdot^2 (no limit given)
U_BIG = 1.0e6
_TOL = 1e-9
_ACTIVE_TOL = 1e-7


def _infeasible(msg: str):
    from robotics_api.errors import PlanningError      # lazy: package import is heavy
    return PlanningError(msg)


# ---------------------------------------------------------------------------- 2-variable LP
def _lp2d(c, A, b, tol: float = _TOL):
    """Maximise ``c . z`` s.t. ``A z <= b`` (z in R^2). Exact vertex enumeration.

    Returns ``(z, value)`` or ``None`` when no vertex is feasible (empty region). The region
    must be bounded (callers add box rows); rows are normalised internally.
    """
    c = np.asarray(c, float)
    A = np.asarray(A, float).reshape(-1, 2)
    b = np.asarray(b, float).reshape(-1)
    nrm = np.hypot(A[:, 0], A[:, 1])
    keep = nrm > 1e-14
    if np.any(b[~keep] < -tol):
        return None
    A, b, nrm = A[keep], b[keep], nrm[keep]
    A = A / nrm[:, None]
    b = b / nrm
    m = len(b)
    if m < 2:
        return None
    ii, jj = np.triu_indices(m, 1)
    det = A[ii, 0] * A[jj, 1] - A[ii, 1] * A[jj, 0]
    ok = np.abs(det) > 1e-12
    ii, jj, det = ii[ok], jj[ok], det[ok]
    if len(ii) == 0:
        return None
    z0 = (b[ii] * A[jj, 1] - A[ii, 1] * b[jj]) / det
    z1 = (A[ii, 0] * b[jj] - b[ii] * A[jj, 0]) / det
    Z = np.stack([z0, z1], axis=1)
    feas = np.all(Z @ A.T <= b[None, :] + tol * (1.0 + np.abs(b))[None, :], axis=1)
    if not np.any(feas):
        return None
    Z = Z[feas]
    val = Z @ c
    k = int(np.argmax(val))
    return Z[k], float(val[k])


# ---------------------------------------------------------------------------- result
@dataclass
class ToppSolution:
    s: np.ndarray               # grid (all segments concatenated, local s per segment)
    q: np.ndarray               # (N+1, n) path points
    x: np.ndarray               # sdot^2 per grid point
    u: np.ndarray               # sddot per stage (N)
    t: np.ndarray               # time at grid points
    active: list = field(default_factory=list)     # per stage: names of binding constraints
    segments: list = field(default_factory=list)   # (i0, i1) index ranges, rest at their ends
    dq: np.ndarray | None = None  # (N+1, n) dq/ds at the grid points (U2.9: qd = dq * sqrt(x))

    @property
    def duration(self) -> float:
        return float(self.t[-1])


# ---------------------------------------------------------------------------- one rest-to-rest segment
def _norm_row(alpha, gamma, beta):
    n = math.hypot(alpha, gamma)
    return (alpha / n, gamma / n, beta / n) if n > 1e-14 else None


def _solve_segment(dm, s, Q, dQ, ddQ, vmax, amax, eff, gravity, margin):
    N = len(s) - 1
    n = Q.shape[1]
    ds = np.diff(s)
    g = tuple(float(v) for v in gravity)
    f = 1.0 - margin
    raw: list[list[tuple]] = []           # point -> [(alpha, gamma, beta, name)] unnormalised
    cs = []
    for i in range(N + 1):
        q, d1, d2 = list(Q[i]), dQ[i], ddQ[i]
        c = np.asarray(cb.inverse_dynamics(dm, q, [0.0] * n, [0.0] * n, g), float)
        cs.append(c)
        a = np.asarray(cb.inverse_dynamics(dm, q, [0.0] * n, list(d1), g), float) - c
        b = np.asarray(cb.inverse_dynamics(dm, q, list(d1), list(d2), g), float) - c
        r = []
        for j in range(n):
            if math.isfinite(vmax[j]) and d1[j] != 0.0:
                r.append((0.0, d1[j] ** 2, (f * vmax[j]) ** 2, f"vel:{j}"))
            if math.isfinite(amax[j]):
                r.append((d1[j], d2[j], f * amax[j], f"acc+:{j}"))
                r.append((-d1[j], -d2[j], f * amax[j], f"acc-:{j}"))
            if math.isfinite(eff[j]):
                r.append((a[j], b[j], f * eff[j] - c[j], f"tau+:{j}"))
                r.append((-a[j], -b[j], f * eff[j] + c[j], f"tau-:{j}"))
        raw.append(r)
    rows: list[list[tuple]] = []          # stage -> [(alpha, gamma, beta, name)] normalised
    for i in range(N):
        # limits hold at BOTH ends of a stage (U2.9): the rows of point i+1 are written in
        # (u_i, x_i) with x_{i+1} = x_i + 2 ds_i u_i, so sddot constant on a stage cannot
        # overshoot a limit at the next grid point
        kk = 2.0 * ds[i]
        cand = list(raw[i]) + [(al + ga * kk, ga, be, nm) for al, ga, be, nm in raw[i + 1]]
        cand.append((0.0, -1.0, 0.0, "x>=0"))
        cand.append((0.0, 1.0, X_BIG, "x<=big"))
        out = []
        for al, ga, be, nm in cand:
            nr = _norm_row(al, ga, be)
            if nr is None:
                if be < -_TOL:
                    raise _infeasible(f"torque/acceleration limit cannot be met at stage {i} "
                                      f"({nm}) of the path")
                continue
            out.append((*nr, nm))
        rows.append(out)
    # rest at both ends needs the holding torque inside the limit
    for i in (0, N):
        over = [j for j in range(n) if math.isfinite(eff[j]) and abs(cs[i][j]) > f * eff[j] + 1e-9]
        if over:
            raise _infeasible("torque limits cannot be met on this path at rest-to-rest: holding "
                              f"torque of dof {over[0]} is {abs(cs[i][over[0]]):.4g} > limit "
                              f"{eff[over[0]]:.4g} at grid point {i}")

    lo = np.zeros(N + 1)
    hi = np.zeros(N + 1)
    uc = np.array([0.0, 1.0])
    for i in range(N - 1, -1, -1):
        k = 2.0 * ds[i]
        extra = [_norm_row(k, 1.0, hi[i + 1]), _norm_row(-k, -1.0, -lo[i + 1]),
                 (1.0, 0.0, U_BIG), (-1.0, 0.0, U_BIG)]
        A = [(r[0], r[1]) for r in rows[i]] + [(e[0], e[1]) for e in extra]
        bb = [r[2] for r in rows[i]] + [e[2] for e in extra]
        top = _lp2d(uc, A, bb)
        if top is None:
            raise _infeasible(f"torque limits cannot be met on this path at rest-to-rest "
                              f"(no admissible speed at grid point {i} of {N})")
        bot = _lp2d(-uc, A, bb)
        hi[i], lo[i] = top[0][1], bot[0][1]
        lo[i] = max(lo[i], 0.0)
    if lo[0] > 1e-9:
        raise _infeasible("torque limits cannot be met on this path at rest-to-rest "
                          "(cannot start from rest)")

    x = np.zeros(N + 1)
    u = np.zeros(N)
    active = []
    for i in range(N):
        k = 2.0 * ds[i]
        ext = [_norm_row(k, 1.0, hi[i + 1]) + ("K+",), _norm_row(-k, -1.0, -lo[i + 1]) + ("K-",)]
        allr = rows[i] + ext
        u_hi, u_lo = math.inf, -math.inf
        for al, ga, be, _nm in allr:
            if al > 1e-12:
                u_hi = min(u_hi, (be - ga * x[i]) / al)
            elif al < -1e-12:
                u_lo = max(u_lo, (be - ga * x[i]) / al)
        ui = u_hi if u_lo <= u_hi else 0.5 * (u_lo + u_hi)
        xn = min(max(x[i] + k * ui, lo[i + 1]), hi[i + 1])
        if i == N - 1:
            xn = 0.0
        u[i] = (xn - x[i]) / k
        x[i + 1] = xn
        act = [nm for al, ga, be, nm in allr
               if be - (al * u[i] + ga * x[i]) <= _ACTIVE_TOL and nm not in ("x>=0", "x<=big")]
        active.append(act)
    # times
    t = np.zeros(N + 1)
    for i in range(N):
        den = math.sqrt(x[i]) + math.sqrt(x[i + 1])
        if den <= 1e-12:
            raise _infeasible(f"torque limits cannot be met on this path at rest-to-rest "
                              f"(no motion at stage {i})")
        t[i + 1] = t[i] + 2.0 * ds[i] / den
    return x, u, t, active


# ---------------------------------------------------------------------------- geometry
def _spline_geometry(P, n_grid):
    """C2 spline through the waypoints ``P`` with s in [0, 1] (chord length). Natural ends."""
    seg = np.linalg.norm(np.diff(P, axis=0), axis=1)
    cum = np.concatenate([[0.0], np.cumsum(seg)])
    s_knots = cum / cum[-1]
    sp = Trajectory([Waypoint(float(sk), tuple(q)) for sk, q in zip(s_knots, P)],
                    "cubic", clamped=False)
    s = np.linspace(0.0, 1.0, int(n_grid) + 1)
    Q = np.array([sp.q(v) for v in s])
    dQ = np.array([sp.qd(v) for v in s])
    ddQ = np.array([sp.qdd(v) for v in s])
    return sp, s, Q, dQ, ddQ


def _dedupe(path):
    P = [np.asarray(path[0], float)]
    for q in path[1:]:
        q = np.asarray(q, float)
        if np.linalg.norm(q - P[-1]) > 1e-9:
            P.append(q)
    return np.array(P)


def solve_topp(model, path, vmax, amax, effort, gravity=GRAVITY, n_grid: int = 100,
               interpolation: str = "cubic", collision=None, pos_lo=None, pos_hi=None,
               margin: float = 0.0, dm=None):
    """Time-optimal rest-to-rest parametrisation of ``path`` (waypoints ``(m, n)``).

    ``interpolation="cubic"``: C2 spline geometry (used only when a dense sample stays inside
    ``pos_lo/pos_hi`` and is collision free, else polyline). Polyline: every segment is solved
    rest-to-rest. Returns ``(ToppSolution, kind, notes)``; ``kind`` is "cubic" or "linear".
    Raises ``PlanningError`` when the limits cannot be met.
    """
    P = _dedupe(np.asarray(path, float))
    if len(P) < 2:
        raise ValueError("path needs two different waypoints")
    n = P.shape[1]
    vmax, amax, eff = (np.broadcast_to(np.asarray(v, float), (n,)) for v in (vmax, amax, effort))
    if dm is None:
        dm = cb.build_dyn_model(model, cb.compile(model))
    notes: list[str] = []
    kind = interpolation
    geo = None
    if kind == "cubic" and len(P) >= 3:
        sp, s, Q, dQ, ddQ = _spline_geometry(P, n_grid)
        dense = np.array([sp.q(v) for v in np.linspace(0.0, 1.0, 4 * int(n_grid) + 1)])
        bad = False
        if pos_lo is not None and (np.any(dense < pos_lo - 1e-9) or np.any(dense > pos_hi + 1e-9)):
            bad = True
        if not bad and collision is not None and np.any(collision.check_batch(dense)):
            bad = True
        if bad:
            notes.append("cubic spline left the free space: linear path used")
            kind = "linear"
        else:
            geo = [(s, Q, dQ, ddQ)]
    elif kind == "cubic":                       # 2 waypoints: the spline is the line
        kind = "linear"
    if geo is None:
        kind = "linear"
        nseg = len(P) - 1
        per = max(10, int(math.ceil(n_grid / nseg)))
        geo = []
        for a, b in zip(P[:-1], P[1:]):
            s = np.linspace(0.0, 1.0, per + 1)
            Q = a[None, :] + s[:, None] * (b - a)[None, :]
            geo.append((s, Q, np.tile(b - a, (per + 1, 1)), np.zeros((per + 1, n))))
    S, QQ, DQ, X, U, T, ACT, segs = [], [], [], [], [], [], [], []
    t_off, i_off = 0.0, 0
    for (s, Q, dQ, ddQ) in geo:
        x, u, t, act = _solve_segment(dm, s, Q, dQ, ddQ, vmax, amax, eff, gravity, margin)
        first = 0 if not S else 1               # junction point shared with the previous segment
        S.append(s[first:])
        QQ.append(Q[first:])
        DQ.append(dQ[first:])
        X.append(x[first:])
        T.append(t[first:] + t_off)
        U.append(u)
        ACT.extend(act)
        segs.append((i_off, i_off + len(s) - 1 - 0))
        i_off += len(s) - 1
        t_off += t[-1]
    sol = ToppSolution(np.concatenate(S), np.vstack(QQ), np.concatenate(X), np.concatenate(U),
                       np.concatenate(T), ACT, segs, np.vstack(DQ))
    return sol, kind, notes


def solution_trajectory(sol: ToppSolution, scale: float = 1.0, hermite: bool = True) -> Trajectory:
    """Cubic ``Trajectory`` through ``(q(s_i), t_i * scale)``, zero velocity at both ends.

    U2.9: with ``hermite=True`` (default) each knot also carries the exact velocity
    ``dq/ds * sqrt(x_i) / scale`` of the solution, so the trajectory is a piecewise cubic Hermite
    (exact for constant ``sddot`` on a straight segment) and does not ring at acceleration jumps.
    ``hermite=False`` (or a solution without ``dq``) gives the old spline without velocities.
    """
    use = hermite and sol.dq is not None
    vel = sol.dq * np.sqrt(np.maximum(sol.x, 0.0))[:, None] / scale if use else None
    return Trajectory([Waypoint(float(t * scale), tuple(q), tuple(vel[i]) if use else None)
                       for i, (t, q) in enumerate(zip(sol.t, sol.q))],
                      "cubic", clamped=True)
