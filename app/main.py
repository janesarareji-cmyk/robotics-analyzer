"""
app/main.py
Robotics Analyzer — application entry point.
R1.3: Initializes AppLogger before creating the MainWindow.
"""

import os
import sys

if not getattr(sys, "frozen", False):
    # `python app/main.py` puts app/ on sys.path, not the repo root.
    _ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    if _ROOT not in sys.path:
        sys.path.insert(0, _ROOT)

from PySide6.QtCore import QEvent
from PySide6.QtWidgets import QApplication
from app.main_window import MainWindow
from app.logger import AppLogger, log


from app import __version__  # noqa: E402


def create_app(log_level: str = "INFO"):
    """
    Create the QApplication, AppLogger, and MainWindow.
    Returns (app, window, logger).
    """
    app = QApplication.instance() or QApplication(sys.argv)
    app.setApplicationName("RoboticsAnalyzer")
    app.setApplicationVersion(__version__)
    app.setOrganizationName("RoboticsAnalyzer")

    # Initialize logging before creating MainWindow
    logger = AppLogger(level=log_level)

    # Create main window
    window = MainWindow()

    # Wire logger → LogPanel
    logger.wire_to_panel(window.log_panel)

    log.info("Main window created. Application ready.")
    return app, window, logger


def _destroy_window(app, window) -> None:
    """Delete the window through the event loop so Qt, not the GC, destroys it (A1.8)."""
    window.deleteLater()
    app.sendPostedEvents(None, QEvent.Type.DeferredDelete)
    app.processEvents()


def _sample_urdf() -> str:
    """Path of the bundled sample robot (frozen: next to the exe; source: examples/)."""
    import os
    base = getattr(sys, "_MEIPASS", None) or os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    return os.path.join(base, "examples", "planar_2r.urdf")


def smoke() -> int:
    """R8.5 smoke start: MainWindow + sample robot + one FK. One result line, exit code 0/1.

    No event loop is left running. Set QT_QPA_PLATFORM=offscreen for headless use.
    """
    import os
    import platform
    try:
        os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
        import PySide6
        import vtk
        import core_bridge as cb
        from app.compute.backend import get_backend
        from app.importers import import_urdf
        app, window, _logger = create_app()
        res = import_urdf(_sample_urdf())
        if res.model is None:
            raise RuntimeError("sample import failed: " + res.report.to_text())
        import numpy as np
        ndof = int(res.model.dof())
        T = np.array(cb.fk(res.model, [0.0] * ndof)["tcp"], dtype=float)
        pos = [float(x) for x in T[:3, 3]]
        be = get_backend()
        di = be.device_info()
        window.close()
        _destroy_window(app, window)
        del window, _logger
        print("SMOKE OK python={} qt={} vtk={} fk_tcp={} backend={} reason={}".format(
            platform.python_version(), PySide6.__version__, vtk.vtkVersion.GetVTKVersion(),
            [round(x, 6) for x in pos], di.get("backend", be.name), be.reason))
        return 0
    except Exception as e:  # noqa: BLE001
        print("SMOKE FAIL {}: {}".format(type(e).__name__, e))
        return 1


if __name__ == "__main__" and "--smoke" in sys.argv:
    sys.exit(smoke())

if __name__ == "__main__":
    app, window, logger = create_app()
    window.show()
    window.project_controller.check_startup_recovery(window)   # R8.4
    window.maybe_start_tour()                                   # U5.9: first run only
    rc = app.exec()
    _destroy_window(app, window)
    del window, logger
    sys.exit(rc)
