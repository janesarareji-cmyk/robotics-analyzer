"""Make robots/<name>/targets.json for the bundled sample robots (D1.2).

    python tools/gen_sample_targets.py [name ...]      (default: all 9)

Per robot: 3 reachable targets ("Pick A", "Place B", "Home-side C") and 2 unreachable ones
("Too far", "Too close"), written in the order A, Too far, B, C, Too close. Reachable poses are
forward kinematics of random collision-free joint vectors (so the orientation is one the robot can
really hold), chosen near wanted azimuth / radius / height slots, tool pointing down when possible.
Each reachable target is verified with ``check_target`` (valid IK, free approach) and the chain
q0 -> A -> B -> C -> A is verified with ``plan_target_motion``. Seeded, deterministic.
"""
from __future__ import annotations

import itertools
import math
import os
import sys
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import numpy as np                                        # noqa: E402

import core_bridge as cb                                  # noqa: E402
from app.analysis.target_motion import plan_target_motion  # noqa: E402
from app.kin_utils import dof_limits, write_q              # noqa: E402
from app.gripper import gripper_dofs                       # noqa: E402
from app.sample_robots import list_sample_robots           # noqa: E402
from app.sample_targets import (KIND_REACHABLE, KIND_UNREACHABLE, SampleScene, build_scene,  # noqa: E402
                                check_target, write_sample_targets)
from app.target import pose_from_xyz_rpy                   # noqa: E402

N_SAMPLES = 8000
SLOTS = [  # name, azimuth (deg), radius / reach, height / reach
    ("Pick A", -45.0, 0.55, 0.28),
    ("Place B", 45.0, 0.72, 0.30),
    ("Home-side C", 140.0, 0.50, 0.50),
]
TOP_K = 6


def _samples(scene: SampleScene, seed: int = 7):
    """Collision-free random q, with TCP pose; also the max horizontal reach seen."""
    rng = np.random.default_rng(seed)
    lo, hi = (np.array(v, float) for v in dof_limits(scene.model))
    q_ref = np.array(scene.q0, float)
    for g in gripper_dofs(scene.model):
        lo[g.dof_index] = hi[g.dof_index] = q_ref[g.dof_index]
    qs = lo + rng.random((N_SAMPLES, len(lo))) * (hi - lo)
    free = ~np.asarray(scene.session.world.check_batch(qs))
    qs = qs[free]
    poses = np.array([np.asarray(cb.fk_compiled(scene.cm, list(q))["tcp"], float) for q in qs])
    return qs, poses


def _az(p) -> float:
    return math.degrees(math.atan2(p[1], p[0]))


def _angdiff(a: float, b: float) -> float:
    return abs((a - b + 180.0) % 360.0 - 180.0)


def make_robot(name: str, urdf: Path) -> list | None:
    scene = build_scene(urdf)
    qs, poses = _samples(scene)
    r_xy = np.hypot(poses[:, 0, 3], poses[:, 1, 3])
    r_max = float(np.percentile(r_xy, 99.5))
    z = poses[:, 2, 3]
    down = -poses[:, 2, 2]                              # tool z axis pointing down
    az = np.array([_az(p[:3, 3]) for p in poses])
    print(f"{name}: {len(qs)} free samples, reach {r_max:.3f} m, start TCP {scene.tcp(scene.q0)[:3, 3].round(3)}",
          flush=True)

    cand_per_slot = []
    for sname, az_t, r_t, z_t in SLOTS:
        cost = (np.array([_angdiff(a, az_t) for a in az]) / 25.0
                + np.abs(r_xy - r_t * r_max) / (0.08 * r_max)
                + np.abs(z - z_t * r_max) / (0.08 * r_max)
                + np.clip(0.6 - down, 0.0, None) * 4.0
                + (z < 0.06) * 10.0)
        order = np.argsort(cost)
        ok = []
        for i in order[:120]:
            if len(ok) >= TOP_K:
                break
            c = check_target(scene, poses[i])
            if c.reachable and c.approach_free and not c.near_singular:
                ok.append((poses[i], c))
        if not ok:
            print(f"  no verified candidate for {sname}")
            return None
        cand_per_slot.append(ok)

    chosen = None
    for combo in itertools.product(*cand_per_slot):
        if _chain_ok(scene, [c[0] for c in combo]):
            chosen = combo
            break
    if chosen is None:
        print("  no chain of candidates plans")
        return None

    far = pose_from_xyz_rpy((1.45 * r_max, 0.0, 0.30 * r_max), (math.pi, 0.0, 0.0))
    if check_target(scene, far).reachable:
        print("  'Too far' is reachable?!")
        return None
    close = None
    for xyz in ((0.03, 0.0, 0.35 * r_max), (0.0, 0.0, 0.5 * r_max), (-0.03, 0.0, 0.2 * r_max),
                (0.0, 0.0, 0.1 * r_max)):
        T = pose_from_xyz_rpy(xyz, (math.pi, 0.0, 0.0))
        if not check_target(scene, T).reachable:
            close = T
            break
    if close is None:
        print("  no unreachable 'Too close' pose found")
        return None
    items = [("Pick A", chosen[0][0], KIND_REACHABLE),
             ("Too far", far, KIND_UNREACHABLE),
             ("Place B", chosen[1][0], KIND_REACHABLE),
             ("Home-side C", chosen[2][0], KIND_REACHABLE),
             ("Too close", close, KIND_UNREACHABLE)]
    return items


def _chain_ok(scene: SampleScene, poses) -> bool:
    """q0 -> A -> B -> C -> A planned without collision (approach 0, like the demo)."""
    q = list(scene.q0)
    seq = [poses[0], poses[1], poses[2], poses[0]]
    from app.analysis.ik_rank import rank_solutions
    for T in seq:
        rr = rank_solutions(scene.model, T, q, scene.session, cm=scene.cm)
        best = rr.best()
        if best is None:
            return False
        res = plan_target_motion(scene.model, q, best.q, T, scene.session.world, cm=scene.cm)
        if not res.ok and "already" not in res.message:
            return False
        q = list(best.q)
    return True


def main() -> int:
    from PySide6.QtCore import QCoreApplication
    QCoreApplication.instance() or QCoreApplication([])
    only = set(sys.argv[1:])
    rc = 0
    for title, urdf in list_sample_robots():
        name = urdf.parent.name
        if only and name not in only:
            continue
        items = make_robot(name, urdf)
        if items is None:
            rc = 1
            continue
        p = write_sample_targets(urdf, items)
        print("  wrote", p, flush=True)
    return rc


if __name__ == "__main__":
    sys.exit(main())
