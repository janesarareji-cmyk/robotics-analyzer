"""U5 real-window screenshots: python tools/u5_shots.py <outdir>

Real platform (not offscreen). Settings isolated. Saves the key screens used by the UI review.
"""
from __future__ import annotations

import os
import sys
import time
from pathlib import Path

os.environ["QT_QPA_PLATFORM"] = "windows"
ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from PySide6.QtCore import QCoreApplication, QEvent       # noqa: E402
from PySide6.QtWidgets import QApplication                # noqa: E402

from app.settings import AppSettings                      # noqa: E402

AppSettings._ORGANIZATION = "RoboticsAnalyzer_shots_u5"
AppSettings._APPLICATION = "RoboticsAnalyzer_shots_u5"

from app.main_window import MainWindow                    # noqa: E402
from tests.python.test_live_blocking import load          # noqa: E402


def pump(sec: float = 0.6) -> None:
    t0 = time.time()
    while time.time() - t0 < sec:
        QCoreApplication.processEvents()
        QCoreApplication.sendPostedEvents(None, QEvent.DeferredDelete)
        time.sleep(0.01)


def shot(win, out: Path, name: str) -> None:
    pump(0.8)
    win.viewport.plotter.render()
    pump(0.4)
    path = out / f"{name}.png"
    win.grab().save(str(path))
    print("saved", path, flush=True)


def main() -> int:
    out = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / "docs" / "screens" / "u5" / "after"
    out.mkdir(parents=True, exist_ok=True)
    QApplication.instance() or QApplication([])
    AppSettings()._settings.clear()
    win = MainWindow()
    win.settings._settings.clear()
    win.resize(1600, 950)
    win.show()
    pump(1.0)
    shot(win, out, "01_empty")
    win.start_tour()
    shot(win, out, "02_tour")
    win._tour._end(True)
    load(win, "ur5")
    win.collision_action.setChecked(True)
    pump(1.0)
    shot(win, out, "03_move")
    win.resize(1400, 900)
    pump(0.8)
    shot(win, out, "04_move_1400")
    win.resize(1600, 950)
    win.set_mode("Model")
    pump(0.8)
    win.model_panel.select("link", 3)
    shot(win, out, "05_model")
    win.set_mode("Target")
    pump(0.8)
    shot(win, out, "06_target")
    win.add_scene_object("box")
    win.set_mode("Plan")
    pump(0.8)
    shot(win, out, "07_plan")
    win.set_mode("Simulate")
    pump(0.8)
    shot(win, out, "08_simulate")
    win.set_mode("Analyze")
    pump(0.8)
    shot(win, out, "09_analyze")
    win._action_view_compare.setChecked(True)
    pump(0.8)
    shot(win, out, "10_compare")
    win.set_mode("Move")
    pump(0.5)
    row = win.joint_panel.rows[2]
    row.spinbox.setValue(row.upper_disp - 1.0)          # near a limit: red marker
    pump(0.5)
    win.jog_panel.step("z", 1)
    shot(win, out, "11_limit_and_jog")
    win.show_help()
    pump(0.8)
    win._help_dialog.grab().save(str(out / "12_help.png"))
    print("saved help", flush=True)
    win._help_dialog.close()
    win.project_controller._is_modified = False
    win.sim_dock.hide()
    win.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
