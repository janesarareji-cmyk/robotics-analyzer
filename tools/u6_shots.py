"""U6 real-window screenshots: python tools/u6_shots.py <outdir>

Real platform (not offscreen). Settings isolated. Saves the overlay screens used by the UI review
and one frame of a real video export.
"""
from __future__ import annotations

import os
import sys
import time
from pathlib import Path

os.environ["QT_QPA_PLATFORM"] = "windows"
ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import numpy as np                                        # noqa: E402
from PySide6.QtCore import QCoreApplication, QEvent       # noqa: E402
from PySide6.QtWidgets import QApplication                # noqa: E402

from app.settings import AppSettings                      # noqa: E402

AppSettings._ORGANIZATION = "RoboticsAnalyzer_shots_u6"
AppSettings._APPLICATION = "RoboticsAnalyzer_shots_u6"

from app.kin_utils import current_q                       # noqa: E402
from app.main_window import MainWindow                    # noqa: E402
from app.sim.trajectory import Trajectory, Waypoint       # noqa: E402
from tests.python.test_live_blocking import load, put_q   # noqa: E402


def pump(sec: float = 0.6) -> None:
    t0 = time.time()
    while time.time() - t0 < sec:
        QCoreApplication.processEvents()
        QCoreApplication.sendPostedEvents(None, QEvent.DeferredDelete)
        time.sleep(0.01)


def pump_until(cond, timeout=120.0) -> bool:
    t0 = time.time()
    while time.time() - t0 < timeout:
        pump(0.05)
        if cond():
            return True
    return cond()


def shot(win, out: Path, name: str) -> None:
    pump(0.8)
    win.viewport.plotter.render()
    pump(0.4)
    path = out / f"{name}.png"
    win.grab().save(str(path))
    print("saved", path, flush=True)


def main() -> int:
    out = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / "docs" / "screens" / "u6"
    out.mkdir(parents=True, exist_ok=True)
    QApplication.instance() or QApplication([])
    AppSettings()._settings.clear()
    win = MainWindow()
    win.settings._settings.clear()
    win.resize(1600, 950)
    win.show()
    pump(1.0)
    win._tour._end(True)
    model = load(win, "ur5")
    put_q(win, [0.3, -1.2, 1.4, -0.5, 0.8, 0.2])
    win.viewport.reset_camera()
    m = win.overlays
    rv = win.viewport.robot_view

    win.set_mode("Analyze")
    pump(0.8)
    shot(win, out, "01_overlays_dock")
    m.set_enabled("reach", True)
    assert pump_until(lambda: m.data("reach") is not None)
    shot(win, out, "02_reach")
    import core_bridge as cb
    d = m.data("reach")
    z_tcp = float(np.asarray(cb.fk(model)["tcp"])[2, 3])
    frac = (z_tcp - d["origin"][2]) / (d["dims"][2] * d["voxel_size"])
    card = win.overlay_panel.cards["reach"].extra
    card.slice_check.setChecked(True)
    card.slice_slider.setValue(int(round(frac * 100)))
    pump(0.5)
    win.set_mode("Target")
    win.target_panel.add_from_tcp()
    tcp = np.asarray(win.target_panel.current_pose())
    win.target_panel.controller.pose_changed.emit(tcp)
    shot(win, out, "03_reach_slice_probe")
    win.set_mode("Analyze")
    m.set_enabled("reach", False)
    card.slice_check.setChecked(False)

    m.set_enabled("singularity", True)
    assert pump_until(lambda: m.data("singularity") is not None)
    shot(win, out, "04_singularity")
    m.set_option("singularity", "stat", "any")
    pump(0.5)
    shot(win, out, "05_singularity_any_orientation")
    m.set_enabled("singularity", False)

    win.set_mode("Model")
    pump(0.8)
    shot(win, out, "11_model_tree")
    win.set_mode("Move")
    m.set_enabled("load", True)
    put_q(win, [0.3, -0.3, 0.4, -0.5, 0.8, 0.2])
    pump(0.3)
    put_q(win, [0.3, -0.3, 0.4, -0.5, 0.8, 0.2])
    shot(win, out, "06_joint_load")
    # a motion: playback bar + load bars with speed
    q0 = np.array(current_q(model))
    q1 = q0 + np.array([0.7, -0.5, 0.5, 0.4, -0.3, 0.2])
    traj = Trajectory([Waypoint(0.0, q0.tolist()), Waypoint(1.2, q1.tolist())], interpolation="cubic")
    win.sim_panel.load_trajectory(traj)
    win.sim_panel.set_mode("kinematic")
    win.sim_panel.seek_trajectory(0.5)
    pump(0.6)
    shot(win, out, "07_playback_bar")
    win.resize(1400, 900)
    pump(0.8)
    shot(win, out, "08_toolbar_1400")
    win.resize(1600, 950)
    pump(0.6)
    dlg = win.export_video(modal=False)
    pump(0.6)
    dlg.fps.setValue(10)
    dlg.format.setCurrentIndex(dlg.format.findData("png"))
    dlg.path.setText(str(out / "video_frames"))
    dlg.grab().save(str(out / "09_video_dialog.png"))
    print("saved video dialog", flush=True)
    dlg.start()
    assert pump_until(lambda: dlg.result_path != "", 120)
    frames = sorted(Path(dlg.result_path).glob("frame_*.png"))
    print("frames", len(frames), flush=True)
    import shutil
    shutil.copy(frames[len(frames) // 2], out / "10_video_frame.png")
    shutil.rmtree(dlg.result_path, ignore_errors=True)
    dlg.reject()
    win.project_controller._is_modified = False
    win.sim_dock.hide()
    win.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
