"""Check a Cartesian path before it is played (U4b.2). No Qt dependency.

``check_path(model, cpath, collision)`` looks at every sample of a ``CartesianPath``:

* reachable: IK solved the sample and the solved TCP is within ``REACH_TOL`` (1e-4 m) of it
* colliding: the robot touches itself, the floor or an object (contacts that exist at the first
  sample and do not get deeper are allowed, as in the live guard)
* sigma_min: smallest singular value of the TCP Jacobian (position rows only when the arm has
  fewer than 6 joints)
* clearance: smallest collision distance (m)
* joint_jump: the solution jumps more than ``max_joint_step`` from the previous sample (branch flip)

Sample status: ``bad`` (not reachable, colliding, joint jump), ``warn`` (sigma_min below
``SINGULAR_WARN`` or clearance below ``CLEARANCE_WARN``, or half the start clearance when the
robot already starts tighter than that), else ``ok``. Samples behind the first
unreachable one are bad too: the arm cannot get there.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np

import core_bridge as cb
from app.gripper import gripper_dofs
from app.kin_utils import SINGULAR_WARN

REACH_TOL = 1e-4               # m
CLEARANCE_WARN = 0.010         # m
OK, WARN, BAD = 0, 1, 2
STATUS_NAMES = ("ok", "warn", "bad")


@dataclass
class PathCheck:
    t: np.ndarray
    status: np.ndarray                  # (n,) int OK / WARN / BAD
    reachable: np.ndarray               # (n,) bool
    colliding: np.ndarray               # (n,) bool
    joint_jump: np.ndarray              # (n,) bool
    sigma_min: np.ndarray               # (n,) nan where unreachable
    clearance: np.ndarray               # (n,) m, inf = nothing to check, nan where unreachable
    reasons: list = field(default_factory=list)      # text per sample ("" = fine)
    message: str = ""                   # path failure text (IK) or ""

    @property
    def n(self) -> int:
        return len(self.status)

    @property
    def ok(self) -> bool:
        """No bad sample (warnings allowed)."""
        return not bool(np.any(self.status == BAD))

    @property
    def n_warn(self) -> int:
        return int(np.sum(self.status == WARN))

    @property
    def n_bad(self) -> int:
        return int(np.sum(self.status == BAD))

    @property
    def first_bad(self) -> int | None:
        idx = np.flatnonzero(self.status == BAD)
        return int(idx[0]) if len(idx) else None

    def segments(self, status: int) -> list[tuple[int, int]]:
        """Runs ``(first, last)`` sample index of consecutive samples with ``status``."""
        runs, start = [], None
        for k, s in enumerate(self.status):
            if s == status and start is None:
                start = k
            elif s != status and start is not None:
                runs.append((start, k - 1))
                start = None
        if start is not None:
            runs.append((start, self.n - 1))
        return runs

    def problem_text(self) -> str:
        """Why Play is blocked ("" when it is not): sample index, time and reason."""
        k = self.first_bad
        if k is None:
            return ""
        return f"Sample {k} (t = {self.t[k]:.2f} s): {self.reasons[k]}"

    def summary(self) -> str:
        if self.n == 0:
            return "No path."
        if not self.ok:
            return f"{self.n_bad} of {self.n} samples bad. " + self.problem_text()
        if self.n_warn:
            return f"Path OK, {self.n_warn} of {self.n} samples need care (near singularity or tight)."
        return "Path OK."


def check_path(model: Any, cpath: Any, collision: Any = None, cm: Any = None,
               max_joint_step: float = 0.5) -> PathCheck:
    """Per-sample check of ``cpath`` (see the module text). ``collision`` = ``CollisionService`` or
    ``CollisionSession`` (anything with ``check(q, emit=False)`` and ``clearance(q)``), or None."""
    n = len(cpath.t)
    cm = cm if cm is not None else cb.compile(model)
    n_arm = model.dof() - len(gripper_dofs(model))
    pos_only = n_arm < 6
    k_ok = len(cpath.q)
    status = np.zeros(n, int)
    reach = np.zeros(n, bool)
    coll = np.zeros(n, bool)
    jump = np.zeros(n, bool)
    sig = np.full(n, np.nan)
    clr = np.full(n, np.nan)
    reasons = [""] * n
    allowed: dict = {}
    warn_clr = CLEARANCE_WARN
    for k in range(n):
        if k >= k_ok:
            status[k] = BAD
            if k == cpath.failed_at and cpath.message.startswith("joint step"):
                jump[k] = reach[k] = True
                reasons[k] = "joint jump (branch flip): " + cpath.message
            elif k == cpath.failed_at and cpath.message:
                reasons[k] = "not reachable: " + cpath.message
            else:
                reasons[k] = "not reachable (the path stops before this sample)"
            continue
        q = np.asarray(cpath.q[k], float)
        tcp = np.asarray(cb.fk_compiled(cm, list(q))["tcp"], float)
        err = float(np.linalg.norm(tcp[:3, 3] - np.asarray(cpath.targets[k])[:3, 3]))
        reach[k] = err < REACH_TOL
        if not reach[k]:
            reasons[k] = f"not reachable (position error {err * 1000:.1f} mm)"
            status[k] = BAD
            continue
        J = np.asarray(cb.jacobian_tcp(cm, list(q)), float)
        if pos_only:
            J = J[:3]
        sv = np.linalg.svd(J, compute_uv=False)
        sig[k] = float(sv[min(J.shape) - 1])
        if k > 0:
            step = float(np.max(np.abs(q - np.asarray(cpath.q[k - 1], float)))) if q.size else 0.0
            jump[k] = step > max_joint_step
        warn = sig[k] < SINGULAR_WARN
        why = ""
        clr[k] = np.inf
        if collision is not None:
            res = collision.check(q, emit=False)
            if k == 0:
                allowed = res.depths()
            if res.ok:
                clr[k] = float(collision.clearance(q))
                if k == 0 and np.isfinite(clr[k]):         # a pair that is tight at the start
                    warn_clr = min(CLEARANCE_WARN, 0.5 * max(clr[k], 0.0))   # is not news
                warn = warn or clr[k] < warn_clr
            else:
                clr[k] = float(min(res.depths().values(), default=0.0))
                if res.worse_than(allowed):
                    coll[k] = True
                    why = "collides: " + res.text()
        if jump[k]:
            why = (why + "; " if why else "") + "joint jump (branch flip)"
        if coll[k] or jump[k]:
            status[k] = BAD
            reasons[k] = why
        elif warn:
            status[k] = WARN
            reasons[k] = "near a singularity" if sig[k] < SINGULAR_WARN else "tight clearance"
    return PathCheck(np.asarray(cpath.t, float), status, reach, coll, jump, sig, clr, reasons,
                     cpath.message)


# ---------------------------------------------------------------------------- tube colours

COLOR_OK = (0.40, 0.73, 0.42)
COLOR_WARN = (1.00, 0.65, 0.15)
COLOR_BAD = (1.00, 0.32, 0.32)
TUBE_MODES = ("status", "speed", "clearance", "singularity")


def _ramp(x, stops):
    """Piecewise linear colour ramp: ``stops`` = [(value, (r, g, b)), ...] ascending."""
    xs = [s[0] for s in stops]
    out = np.zeros((len(x), 3))
    for c in range(3):
        out[:, c] = np.interp(x, xs, [s[1][c] for s in stops])
    return out


def tube_colors(check: PathCheck | None, mode: str = "status", speed: Any = None,
                n: int | None = None) -> np.ndarray:
    """(n, 3) float RGB per sample for ``mode`` (status, speed, clearance, singularity).

    ``speed`` = TCP speed per sample (m/s), needed for ``"speed"``. Samples without a value
    (unreachable) are red. ``check`` may be None for ``"speed"``.
    """
    if mode not in TUBE_MODES:
        raise ValueError(f"unknown tube mode {mode!r}")
    if mode == "speed":
        v = np.asarray(speed if speed is not None else np.zeros(n or 0), float)
        top = float(np.max(v)) if len(v) and np.max(v) > 0 else 1.0
        return _ramp(v / top, [(0.0, (0.25, 0.35, 0.95)), (0.5, (0.15, 0.80, 0.85)),
                               (1.0, (1.0, 0.92, 0.25))])
    if check is None:
        raise ValueError("check is needed for this mode")
    if mode == "status":
        lut = np.array([COLOR_OK, COLOR_WARN, COLOR_BAD])
        return lut[check.status]
    if mode == "clearance":
        v = np.nan_to_num(np.where(np.isinf(check.clearance), 0.10, check.clearance), nan=-1.0)
        col = _ramp(np.clip(v, -0.01, 0.10), [(-0.01, COLOR_BAD), (0.0, COLOR_BAD),
                                              (CLEARANCE_WARN, COLOR_WARN), (0.10, COLOR_OK)])
    else:
        v = np.nan_to_num(check.sigma_min, nan=-1.0)
        col = _ramp(np.clip(v, -0.01, 0.30), [(-0.01, COLOR_BAD), (0.0, COLOR_BAD),
                                              (SINGULAR_WARN, COLOR_WARN), (0.30, COLOR_OK)])
    col[~check.reachable] = COLOR_BAD
    return col


def tcp_speed(targets: Any, t: Any) -> np.ndarray:
    """TCP speed (m/s) per sample from the sample positions and times."""
    p = np.asarray(targets, float)[:, :3, 3]
    t = np.asarray(t, float)
    if len(p) < 2:
        return np.zeros(len(p))
    return np.linalg.norm(np.gradient(p, t, axis=0), axis=1)
