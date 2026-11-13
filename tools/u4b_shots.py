"""U4b real-window screenshots: python tools/u4b_shots.py [outdir]

Launches the real MainWindow (native platform, not offscreen) and saves PNGs of the Curve and
Matrices docks: a red line through a box, a clear spline with via handles, playback with marker and
end ghost, the Matrices dock while playing, a line near the wrist singularity. Settings are isolated.
"""
from __future__ import annotations

import os
import sys
import time
from pathlib import Path

os.environ["QT_QPA_PLATFORM"] = "windows"      # real platform (test helpers default to offscreen)
ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import numpy as np                                        # noqa: E402
from PySide6.QtCore import QCoreApplication, QEvent       # noqa: E402
from PySide6.QtWidgets import QApplication                # noqa: E402

from app.settings import AppSettings                      # noqa: E402

AppSettings._ORGANIZATION = "RoboticsAnalyzer_shots_u4b"
AppSettings._APPLICATION = "RoboticsAnalyzer_shots_u4b"

from app.main_window import MainWindow                    # noqa: E402
from tests.python.test_live_blocking import load, put_q   # noqa: E402
from tests.python.test_curve_editor import UR5_Q, _fk     # noqa: E402

SIDE = np.array([0.0, -0.34, 0.0])


def pump(sec: float = 0.6) -> None:
    t0 = time.time()
    while time.time() - t0 < sec:
        QCoreApplication.processEvents()
        QCoreApplication.sendPostedEvents(None, QEvent.DeferredDelete)
        time.sleep(0.01)


def pump_until(cond, timeout=30.0):
    t0 = time.time()
    while time.time() - t0 < timeout and not cond():
        pump(0.02)


def shot(win, out: Path, name: str) -> None:
    pump(0.8)
    win.viewport.plotter.render()
    pump(0.4)
    path = out / f"{name}.png"
    win.grab().save(str(path))
    print("saved", path, flush=True)


def camera(win, focal, pos):
    pl = win.viewport.plotter
    pl.camera.focal_point = tuple(focal)
    pl.camera.position = tuple(pos)
    pl.render()


def main() -> int:
    out = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / "docs" / "screens" / "u4b"
    out.mkdir(parents=True, exist_ok=True)
    QApplication.instance() or QApplication([])
    AppSettings()._settings.clear()
    win = MainWindow()
    win.settings._settings.clear()
    win.resize(1700, 1000)
    win.show()
    pump(1.0)
    win.collision_action.setChecked(True)
    model = load(win, "ur5")
    put_q(win, UR5_Q)
    p0 = _fk(model, UR5_Q)[:3, 3]
    mid = p0 + 0.5 * SIDE
    win.collision_panel.add_shape("box", {"size": [0.08, 0.08, 0.08]}, list(mid), name="block")
    win._sync_env_objects()
    win.collision_service.invalidate()
    win._action_view_curve.setChecked(True)
    pump(0.8)
    tp, cp, mp = win.target_panel, win.curve_panel, win.matrix_panel
    tp.auto_solve = False
    tp.add_from_tcp()
    tp.move_to(p0 + SIDE)
    camera(win, p0 + 0.5 * SIDE, p0 + [1.0, 0.45, 0.55])
    pump_until(lambda: not cp.busy)
    shot(win, out, "01_line_red")

    cp.set_kind("spline")
    cp.add_via()
    cp.add_via()
    for i, f in enumerate((0.25, 0.7)):
        cp.set_via_xyz(i, (p0 + f * SIDE + [0.0, 0.0, 0.2]) * 1000.0)
    pump_until(lambda: not cp.busy)
    shot(win, out, "02_spline_clear")

    cp.set_timing("topp")
    got = []
    cp.trajectory_ready.connect(lambda t, i: got.append(t))
    cp.generate(play=False)
    pump_until(lambda: got and not cp.busy)
    win.sim_panel.seek_trajectory(0.5 * got[0].duration)      # half way, paused: marker + ghost
    shot(win, out, "03_playing")

    win._action_view_matrices.setChecked(True)
    mp.show_jacobian(True)
    win.sim_panel.seek_trajectory(0.8 * got[0].duration)
    pump(0.3)
    shot(win, out, "04_matrices")

    win._action_view_curve.setChecked(True)
    win.sim_panel.stop_trajectory()
    pump(0.3)
    win.collision_panel.remove_shape(0)
    win._sync_env_objects()
    win.collision_service.invalidate()
    cp.set_kind("line")
    qa = UR5_Q.copy()
    qa[4] = 0.5
    put_q(win, qa)
    win._on_joint_q_changed()
    qb = UR5_Q.copy()
    qb[4] = 0.03
    from app.target import xyz_rpy_from_pose
    xyz, rpy = xyz_rpy_from_pose(_fk(model, qb))
    tp.set_pose(xyz_mm=xyz * 1000.0, rpy_deg=np.degrees(rpy))
    cp.set_color_mode("singularity")
    pump_until(lambda: not cp.busy)
    camera(win, _fk(model, qa)[:3, 3], _fk(model, qa)[:3, 3] + [0.35, 0.2, 0.15])
    shot(win, out, "05_singular")
    win.project_controller._is_modified = False
    win.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
