"""U3 real-window screenshots: python tools/u3_shots.py [outdir]

Launches the real MainWindow (native platform, not offscreen), builds four scenes and saves a PNG
of the screen region of the window for each: red highlight, blocked status, object in the scene,
grasp. Uses the test helpers for scene setup. Settings are isolated (test organisation name).
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
from PySide6.QtCore import QCoreApplication, QEvent               # noqa: E402
from PySide6.QtWidgets import QApplication                # noqa: E402

from app.settings import AppSettings                      # noqa: E402

AppSettings._ORGANIZATION = "RoboticsAnalyzer_shots_u3"
AppSettings._APPLICATION = "RoboticsAnalyzer_shots_u3"

from app.kin_utils import current_q                       # noqa: E402
from app.main_window import MainWindow                    # noqa: E402
from tests.python.test_grasp import place_cube            # noqa: E402
from tests.python.test_live_blocking import (             # noqa: E402
    find_blocked_move, load, put_q, random_free_q, row_of_dof,
)


def pump(sec: float = 0.6) -> None:
    t0 = time.time()
    while time.time() - t0 < sec:
        QCoreApplication.processEvents()
        QCoreApplication.sendPostedEvents(None, QEvent.DeferredDelete)
        time.sleep(0.01)


def shot(win, out: Path, name: str) -> None:
    pump(0.8)
    win.viewport.plotter.render() if hasattr(win.viewport, "plotter") else None
    pump(0.4)
    pix = win.grab()          # screen grab of the native window renders tofu text here
    path = out / f"{name}.png"
    pix.save(str(path))
    print("saved", path, pix.width(), "x", pix.height())


def main() -> int:
    out = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / "docs" / "screens" / "u3"
    out.mkdir(parents=True, exist_ok=True)
    app = QApplication.instance() or QApplication([])
    win = MainWindow()
    win.settings._settings.clear()
    win.resize(1700, 1000)
    win.show()
    pump(1.0)
    win.collision_action.setChecked(True)

    # 1 + 2: red highlight, blocked status (ur5, jog into the floor/self)
    load(win, "ur5")
    a = random_free_q(win, seed=0)
    put_q(win, a)
    dof, v = find_blocked_move(win, a)
    b = a.copy()
    b[dof] = v
    win.collision_service.check(b)                       # shows the red highlight + Collision: text
    shot(win, out, "01_red_highlight")
    put_q(win, a)
    row = row_of_dof(win, dof)
    row.spinbox.setValue(v * row.to_display)             # blocked slider move
    shot(win, out, "02_blocked_status")

    # 3: an object in the scene
    win.collision_panel.add_shape("box", {"size": [0.1] * 3}, [0.4, 0.0, 0.05], name="box")
    win.collision_dock.show() if hasattr(win, "collision_dock") else None
    shot(win, out, "03_object_in_scene")

    # 4: grasp
    win.project_controller._is_modified = False     # no Save/Discard dialog
    load(win, "xarm5_gripper")
    print("loaded xarm", flush=True)
    place_cube(win)
    win.joint_panel.set_gripper(1.0)
    print("placed cube", flush=True)
    import core_bridge as cb
    tcp = np.asarray(cb.fk(win.project_controller.robot)["tcp"])[:3, 3]
    pl = win.viewport.plotter
    pl.camera.focal_point = tuple(tcp)
    pl.camera.position = tuple(tcp + np.array([0.45, -0.45, 0.3]))
    pl.render()
    shot(win, out, "04_grasp")
    win.project_controller._is_modified = False
    win.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
