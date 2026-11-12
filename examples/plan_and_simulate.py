"""Example 2: plan around a box obstacle, time-parametrize, then simulate the tracking.

Units: metres, radians, seconds. Collision geometry of the URDF-less planar robot is set by hand.
"""
from pathlib import Path

import numpy as np

import robotics_api as api

HERE = Path(globals().get("__file__", "examples/plan_and_simulate.py")).resolve().parent

robot = api.import_urdf(str(HERE / "planar_2r.urdf"))
# one thin box per link (the pose is in the link frame; link 1 and link 2 are 1 m long)
robot.set_collision_shapes([
    {"type": "box", "link": 1, "pose": [0.5, 0, 0], "size": [1.0, 0.1, 0.1]},
    {"type": "box", "link": 2, "pose": [0.5, 0, 0], "size": [1.0, 0.1, 0.1]},
])
obstacles = [{"type": "box", "pose": {"xyz": [0.0, 1.3, 0.0]}, "size": [0.3, 0.3, 0.4]}]

start, goal = [0.0, 0.0], [2.2, 0.3]
path = api.plan(robot, start, goal, obstacles=obstacles, planner="rrt_connect", seed=1)
print(f"path with {len(path)} waypoints")

traj = api.time_parametrize(path, robot)
print(f"trajectory {traj.duration:.2f} s, within limits: {traj.ok}")

res = api.simulate(robot, traj, dt=0.002, record_every=5)
err = np.abs(res["q"] - np.array([traj.q(t) for t in res["t"]])).max()
print(f"simulated {len(res['t'])} samples, max tracking error {err:.2e} rad")
print(f"final tcp = {np.round(res['tcp_pos'][-1], 4)} m")
