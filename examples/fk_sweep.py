"""Example 1: forward kinematics sweep of a planar 2R robot.

Run it from File -> Run script... or in the script console. Units: metres, radians.
"""
from pathlib import Path

import numpy as np

import robotics_api as api

HERE = Path(globals().get("__file__", "examples/fk_sweep.py")).resolve().parent

robot = api.import_urdf(str(HERE / "planar_2r.urdf"))
print(f"robot {robot.name}: {robot.n_dof} DOF, joints {robot.joint_names}")

# Sweep joint 1 over its range with joint 2 fixed at -0.5 rad.
lo, hi = robot.limits[0]
xs = []
for q1 in np.linspace(lo, hi, 9):
    T = robot.fk([q1, -0.5])
    xs.append(T[:2, 3])
    api.check_cancel()                       # lets Cancel stop a long loop
    print(f"q1 = {q1:+.3f} rad  tcp = ({T[0, 3]:+.4f}, {T[1, 3]:+.4f}) m")

radius = [float(np.hypot(*p)) for p in xs]
print(f"tcp distance from the base: min {min(radius):.4f} m, max {max(radius):.4f} m "
      "(constant for a pure rotation of joint 1)")
