"""
app/main.py
Robotics Analyzer — application entry point.
R1.3: Initializes AppLogger before creating the MainWindow.
"""

import sys
from PySide6.QtWidgets import QApplication
from app.main_window import MainWindow
from app.logger import AppLogger, log


def create_app(log_level: str = "INFO"):
    """
    Create the QApplication, AppLogger, and MainWindow.
    Returns (app, window, logger).
    """
    app = QApplication.instance() or QApplication(sys.argv)
    app.setApplicationName("RoboticsAnalyzer")
    app.setApplicationVersion("0.1.0")
    app.setOrganizationName("RoboticsAnalyzer")

    # Initialize logging before creating MainWindow
    logger = AppLogger(level=log_level)

    # Create main window
    window = MainWindow()

    # Wire logger → LogPanel
    logger.wire_to_panel(window.log_panel)

    log.info("Main window created. Application ready.")
    return app, window, logger


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
    sys.exit(app.exec())
