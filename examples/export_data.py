"""Example 3: simulate a free swing of the planar 2R robot and export the data.

Writes swing.csv and swing.npz into a temporary folder and prints the paths.
"""
import tempfile
from pathlib import Path

import robotics_api as api

HERE = Path(globals().get("__file__", "examples/export_data.py")).resolve().parent

robot = api.import_urdf(str(HERE / "planar_2r.urdf"))
# released from rest at 45 degrees, gravity along -y so the arm swings in its plane
res = api.simulate(robot, None, duration=1.0, dt=0.001, q0=[0.8, 0.0], integrator="rk4",
                   gravity=[0.0, -9.81, 0.0], record_every=10)
e = res["total"][:, 0]
print(f"{len(res['t'])} samples, total energy drift {abs(e[-1] - e[0]):.2e} J")

out = Path(tempfile.mkdtemp(prefix="robotics_api_"))
rows = api.export_csv(res, str(out / "swing.csv"), names=["q", "qd", "tcp_pos", "total"],
                      meta={"robot": robot.name})
api.export_npz(res, str(out / "swing.npz"), meta={"robot": robot.name})
print(f"wrote {rows} rows to {out}")
