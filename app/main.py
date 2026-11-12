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


if __name__ == "__main__":
    app, window, logger = create_app()
    window.show()
    window.project_controller.check_startup_recovery(window)   # R8.4
    sys.exit(app.exec())
