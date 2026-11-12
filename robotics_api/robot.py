"""``Robot``: kinematics and dynamics of one robot model (R8.1).

Units: lengths in metres, angles in radians, time in seconds, forces in newtons, torques in
newton metres, masses in kilograms. Poses are 4x4 homogeneous transforms in the WORLD frame
(base pose included). Joint vectors have one entry per DOF (``n_dof``); a prismatic joint entry
is in metres, a revolute entry in radians. Mimic joints and fixed joints are not DOFs.
"""
from __future__ import annotations

import math
from typing import Any

import numpy as np

from ._util import gravity3, guard, pose4, shape_from_dict, vec
from .errors import ComputationError, InvalidArgument


class Robot:
    """A robot model with kinematic and dynamic queries.

    Create it with ``import_urdf(path)``, ``Project.robot`` or ``Robot.from_json``. The object
    owns a snapshot of the model: later edits in the app do not change it.
    """

    def __init__(self, model: Any, name: str = "", visuals: list | None = None,
                 collisions: list | None = None):
        from app.kin_utils import CompiledCache
        self._model = model
        self.name = str(name)
        self._visuals = list(visuals or [])
        self._collisions = list(collisions or [])
        self._shapes: list | None = None        # explicit collision shapes (set_collision_shapes)
        self._cache = CompiledCache()
        self.import_report = ""

    # ------------------------------------------------------------ construction
    @classmethod
    @guard(InvalidArgument)
    def from_json(cls, text: str, name: str = "") -> "Robot":
        """Robot from a Unified Robot Model JSON string (``Robot.to_json``)."""
        import core_bridge as cb
        return cls(cb.urm_from_json(text), name)

    def to_json(self) -> str:
        """The Unified Robot Model of the robot as a JSON string."""
        return self._model.to_json()

    def copy(self) -> "Robot":
        """Independent copy (new model snapshot, same collision geometry)."""
        import core_bridge as cb
        r = Robot(cb.urm_from_json(self._model.to_json()), self.name, self._visuals,
                  self._collisions)
        r._shapes = None if self._shapes is None else list(self._shapes)
        r.import_report = self.import_report
        return r

    # ------------------------------------------------------------------ facts
    @property
    def n_dof(self) -> int:
        """Number of degrees of freedom (non-fixed, non-mimic joints)."""
        return int(self._model.dof())

    @property
    def joint_names(self) -> list[str]:
        """Joint name per DOF, in DOF order (the order of every q vector)."""
        names = [""] * self.n_dof
        for j_idx, q_idx in enumerate(self._model.q_index_map()):
            if q_idx >= 0:
                names[q_idx] = str(self._model.joints[j_idx].name)
        return names

    @property
    def joint_types(self) -> list[str]:
        """``"revolute"`` or ``"prismatic"`` per DOF."""
        from app.kin_utils import dof_joint_types
        return list(dof_joint_types(self._model))

    @property
    def limits(self) -> np.ndarray:
        """Position limits, array of shape ``(n_dof, 2)`` = ``[lower, upper]`` per DOF (rad or m)."""
        from app.kin_utils import dof_limits
        lo, hi = dof_limits(self._model)
        return np.column_stack([np.asarray(lo, float), np.asarray(hi, float)]).reshape(-1, 2)

    def _ik_limits(self) -> tuple[list[float], list[float]]:
        lim = self.limits
        lo, hi = lim[:, 0].copy(), lim[:, 1].copy()
        bad = ~(lo < hi)               # no limit set: search in +-pi
        lo[bad], hi[bad] = -math.pi, math.pi
        return lo.tolist(), hi.tolist()

    @property
    def collision_shapes(self) -> list[dict]:
        """Collision geometry as plain dicts (``type``, ``link``, ``pose``, size keys)."""
        return [dict(type=s.kind, link=int(s.link), pose=np.array(s.pose), **s.params)
                for s in self._link_shapes()]

    def set_collision_shapes(self, shapes: list[dict]) -> None:
        """Replace the robot collision geometry.

        Each dict: ``type`` (box / sphere / cylinder / capsule), ``link`` (link index; the
        shape pose is in that link frame), ``pose`` (4x4, ``{"xyz","rpy"}`` or 3/6 numbers, m and
        rad), ``size`` (box [x, y, z] m; sphere radius m; cylinder, capsule [radius, length] m).
        """
        out = [shape_from_dict(d, ("box", "sphere", "cylinder", "capsule"))[0] for d in shapes]
        for s in out:
            if not (0 <= s.link < len(self._model.links)):
                raise InvalidArgument(f"shape link {s.link} out of range")
        self._shapes = out

    def _link_shapes(self) -> list:
        if self._shapes is not None:
            return self._shapes
        from app.planning.collision import shapes_from_geometry
        items = self._collisions or self._visuals
        return shapes_from_geometry(items) if items else []

    # --------------------------------------------------------------- internals
    def _cm(self):
        return self._cache.get_cm(self._model)

    def _dm(self):
        return self._cache.get_dm(self._model)

    def _q(self, q: Any, name: str = "q") -> list[float]:
        return vec(q, self.n_dof, name).tolist()

    # -------------------------------------------------------------- kinematics
    @guard(ComputationError)
    def fk(self, q: Any) -> np.ndarray:
        """Forward kinematics. ``q``: n_dof values. Returns the 4x4 world pose of the TCP (m)."""
        import core_bridge as cb
        return np.array(cb.fk(self._model, self._q(q))["tcp"], dtype=float)

    @guard(ComputationError)
    def jacobian(self, q: Any) -> np.ndarray:
        """Geometric Jacobian of the TCP in the world frame, shape ``(6, n_dof)``.

        Rows 0-2: linear velocity (m/s per unit joint speed), rows 3-5: angular velocity (rad/s).
        """
        import core_bridge as cb
        return np.array(cb.jacobian_tcp(self._cm(), self._q(q)), dtype=float)

    @guard(ComputationError)
    def ik(self, pose: Any, seed: Any = None, all_solutions: bool = False,
           position_only: bool = False, n_starts: int = 30, random_seed: int = 42):
        """Inverse kinematics of a 4x4 world TCP ``pose`` (m).

        ``seed``: start joint vector (default zeros); solutions are sorted by distance to it.
        Solutions respect the joint limits. Returns the nearest solution (array of n_dof) or
        ``None`` when there is none; with ``all_solutions=True`` a list of arrays (maybe empty).
        ``position_only`` ignores the orientation. ``n_starts`` random restarts, ``random_seed``
        for them.
        """
        import core_bridge as cb
        target = pose4(pose)
        q_seed = np.zeros(self.n_dof) if seed is None else vec(seed, self.n_dof, "seed")
        if int(n_starts) < 1:
            raise InvalidArgument("n_starts must be >= 1")
        lo, hi = self._ik_limits()
        sols = cb.solve_ik_multi(self._cm(), lo, hi, target, q_seed.tolist(),
                                 {"position_only": bool(position_only)}, int(n_starts),
                                 int(random_seed))
        out = [np.asarray(s["q"], dtype=float) for s in sols]
        if all_solutions:
            return out
        return out[0] if out else None

    # ---------------------------------------------------------------- dynamics
    @guard(ComputationError)
    def inverse_dynamics(self, q: Any, qd: Any, qdd: Any, gravity: Any = None) -> np.ndarray:
        """Joint torques (N.m, N for prismatic) that produce ``qdd`` at ``(q, qd)``.

        ``gravity``: world gravity vector in m/s^2 (default [0, 0, -9.81]).
        """
        import core_bridge as cb
        return np.asarray(cb.inverse_dynamics(self._dm(), self._q(q), self._q(qd, "qd"),
                                              self._q(qdd, "qdd"), gravity3(gravity)), float)

    @guard(ComputationError)
    def forward_dynamics(self, q: Any, qd: Any, tau: Any, gravity: Any = None) -> np.ndarray:
        """Joint accelerations (rad/s^2, m/s^2) for the applied torques ``tau`` (N.m, N)."""
        import core_bridge as cb
        return np.asarray(cb.forward_dynamics(self._dm(), self._q(q), self._q(qd, "qd"),
                                              self._q(tau, "tau"), gravity3(gravity)), float)

    @guard(ComputationError)
    def mass_matrix(self, q: Any) -> np.ndarray:
        """Joint space mass matrix M(q), shape ``(n_dof, n_dof)``, symmetric positive definite."""
        import core_bridge as cb
        return np.array(cb.mass_matrix(self._dm(), self._q(q)), dtype=float)

    @guard(ComputationError)
    def energy(self, q: Any, qd: Any, gravity: Any = None) -> dict:
        """Mechanical energy in joules: ``{"ke", "pe", "total"}``.

        ``pe = -sum m g.p_com`` for the world gravity vector (default [0, 0, -9.81]).
        """
        import core_bridge as cb
        qq, vv, g = self._q(q), self._q(qd, "qd"), gravity3(gravity)
        ke = float(cb.kinetic_energy(self._dm(), qq, vv))
        pe = float(cb.potential_energy(self._dm(), qq, g))
        return {"ke": ke, "pe": pe, "total": ke + pe}

    @guard(ComputationError)
    def manipulability(self, q: Any) -> float:
        """Yoshikawa manipulability of the full 6-row TCP Jacobian: product of its singular
        values (mixed units: m and rad). 0 at a singular pose."""
        import core_bridge as cb
        res = cb.analyze_singularity(cb.jacobian_tcp(self._cm(), self._q(q)), mode="full")
        return float(res["manipulability"])

    def __repr__(self) -> str:
        return f"Robot(name={self.name!r}, n_dof={self.n_dof})"
