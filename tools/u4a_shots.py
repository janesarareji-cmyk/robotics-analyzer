"""U4a real-window screenshots: python tools/u4a_shots.py [outdir]

Launches the real MainWindow (native platform, not offscreen) and saves PNGs of the Target dock
scenes: target frame on a box top, ranked solutions with ghost, all ghosts, unreachable target,
approach motion in progress. Settings are isolated (test organisation name).
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

AppSettings._ORGANIZATION = "RoboticsAnalyzer_shots_u4a"
AppSettings._APPLICATION = "RoboticsAnalyzer_shots_u4a"

from app.main_window import MainWindow                    # noqa: E402
from tests.python.test_live_blocking import load          # noqa: E402


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
    out = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / "docs" / "screens" / "u4a"
    out.mkdir(parents=True, exist_ok=True)
    QApplication.instance() or QApplication([])
    AppSettings()._settings.clear()
    win = MainWindow()
    win.settings._settings.clear()
    win.resize(1700, 1000)
    win.show()
    pump(1.0)
    win.collision_action.setChecked(True)
    load(win, "panda")
    win.collision_panel.add_shape("box", {"size": [0.2, 0.2, 0.3]}, [0.5, 0.0, 0.15], name="block")
    win._sync_env_objects()
    win._action_view_target.setChecked(True)
    pump(0.8)
    tp = win.target_panel
    tp.auto_solve = False
    camera(win, (0.45, 0.0, 0.3), (1.35, -0.9, 1.0))
    pump(0.5)
    import vtk
    pl = win.viewport.plotter
    c = vtk.vtkCoordinate()
    c.SetCoordinateSystemToWorld()
    c.SetValue(0.5, 0.0, 0.3)
    x, y = c.GetComputedDoubleDisplayValue(pl.renderer)
    tp._btn_click.setChecked(True)
    tp.controller._on_click((x, y))
    tp._btn_click.setChecked(False)
    shot(win, out, "01_target_on_box")

    tp.solve()
    pump_until(lambda: not tp.is_solving)
    shot(win, out, "02_ranked_ghost")
    tp.select_row(3)
    tp._chk_all.setChecked(True)
    shot(win, out, "03_all_ghosts")
    tp._chk_all.setChecked(False)

    tp.select_row(1)
    tp.set_approach(True, 100, retreat=True)
    tp.play_to_target()
    pump_until(lambda: not tp.is_planning)
    sp = win.sim_panel
    pl_ = sp.player
    if pl_ is not None:
        for _ in range(60):                      # about 1.2 s into the move
            pl_.advance(0.02)
        sp._emit_state()
    shot(win, out, "04_playing")
    if pl_ is not None:
        while not pl_.finished:
            pl_.advance(0.05)
        sp._emit_state()

    tp.set_pose(xyz_mm=[120, 0, 150])
    tp.solve()
    pump_until(lambda: not tp.is_solving)
    shot(win, out, "05_unreachable")
    win.project_controller._is_modified = False
    win.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
