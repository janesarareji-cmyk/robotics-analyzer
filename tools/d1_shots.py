"""D1 real-window screenshots: python tools/d1_shots.py <outdir> [start|targets|demo]"""
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

AppSettings._ORGANIZATION = "RoboticsAnalyzer_shots_d1"
AppSettings._APPLICATION = "RoboticsAnalyzer_shots_d1"

from app.main_window import MainWindow                    # noqa: E402


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
    out = Path(sys.argv[1])
    what = sys.argv[2] if len(sys.argv) > 2 else "start"
    out.mkdir(parents=True, exist_ok=True)
    QApplication.instance() or QApplication([])
    win = MainWindow()
    win.settings._settings.clear()
    win.resize(1600, 950)
    win.show()
    pump(1.0)
    if what == "start":
        shot(win, out, "start_1600")
        win.resize(1400, 900)
        shot(win, out, "start_1400")
        win.resize(1100, 800)
        shot(win, out, "start_1100")
    else:
        from tests.python.test_live_blocking import load
        name = sys.argv[3] if len(sys.argv) > 3 else "ur5"
        load(win, name)
        pump(1.0)
        win.set_mode("Target")
        shot(win, out, f"{name}_targets")
        if what == "demo":
            win.start_demo()
            for i in range(6):
                pump(1.5)
                shot(win, out, f"{name}_demo_{i}")
    win.project_controller._is_modified = False
    win.sim_dock.hide()
    win.close()
    pump(0.5)
    return 0


if __name__ == "__main__":
    sys.exit(main())
