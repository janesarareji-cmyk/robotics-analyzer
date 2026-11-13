"""U5.5: render robots/<name>/thumb.png (260x170) for the start screen. Real Qt platform.

python tools/make_thumbs.py
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
from PySide6.QtCore import QCoreApplication, QEvent, Qt   # noqa: E402
from PySide6.QtGui import QImage                          # noqa: E402
from PySide6.QtWidgets import QApplication                # noqa: E402

from app.settings import AppSettings                      # noqa: E402

AppSettings._ORGANIZATION = "RoboticsAnalyzer_thumbs"
AppSettings._APPLICATION = "RoboticsAnalyzer_thumbs"

from app.main_window import MainWindow                    # noqa: E402
from app.sample_robots import list_sample_robots          # noqa: E402
from tests.python.test_live_blocking import load         # noqa: E402


def pump(sec):
    t0 = time.time()
    while time.time() - t0 < sec:
        QCoreApplication.processEvents()
        QCoreApplication.sendPostedEvents(None, QEvent.DeferredDelete)
        time.sleep(0.01)


def main() -> int:
    QApplication.instance() or QApplication([])
    AppSettings()._settings.clear()
    win = MainWindow()
    win.settings._settings.clear()
    win.resize(1400, 900)
    win.show()
    pump(1.0)
    win.set_mode("Move")
    win._action_show_floor.setChecked(False)
    win._action_show_frames.setChecked(False)
    for _title, path in list_sample_robots():
        win.project_controller._is_modified = False
        load(win, path.parent.name)
        pl = win.viewport.plotter
        pl.view_isometric()
        pl.reset_camera()
        pl.camera.zoom(1.25)
        pump(0.8)
        pl.render()
        pump(0.3)
        img = pl.screenshot(return_img=True)
        h, w, _c = img.shape
        qi = QImage(np.ascontiguousarray(img[:, :, :3]).data, w, h, 3 * w, QImage.Format.Format_RGB888)
        qi = qi.scaled(260, 170, Qt.AspectRatioMode.KeepAspectRatioByExpanding,
                       Qt.TransformationMode.SmoothTransformation)
        x, y = max(0, (qi.width() - 260) // 2), max(0, (qi.height() - 170) // 2)
        qi.copy(x, y, 260, 170).save(str(path.parent / "thumb.png"))
        print("thumb", path.parent.name, flush=True)
    win.project_controller._is_modified = False
    win.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
