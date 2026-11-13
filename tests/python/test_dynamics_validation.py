"""U2.8: app kinematics + dynamics vs MuJoCo (independent engine)."""
import sys
from pathlib import Path

import pytest

mujoco = pytest.importorskip("mujoco", reason="mujoco not installed (dev dependency)")

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
import validate_dynamics as vd  # noqa: E402


@pytest.mark.parametrize("robot", ["ur5", "panda", "kuka_iiwa"])
def test_robot_matches_mujoco(robot):
    res, ver = vd.validate_robot(ROOT / "robots" / robot / f"{robot}.urdf",
                                 "mujoco", n=20, seed=0)
    assert "MuJoCo" in ver
    assert res["fk"][0] <= vd.TOL["fk"], res
    for k in ("jac", "M", "tau", "g"):
        assert res[k][1] <= vd.TOL[k], (k, res[k])
    assert vd.passes(res)
