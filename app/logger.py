"""
app/logger.py
Robotics Analyzer — application logging framework.
R1.3: Python logging → file + Qt LogPanel signal routing.
"""

import logging
import logging.handlers
import os
from pathlib import Path
from datetime import datetime

from PySide6.QtCore import QObject, Signal


# Module-level application logger
log = logging.getLogger("robotics_analyzer")

# Handlers added by AppLogger instances; removed again when a new AppLogger is set up (A1.7).
_added_handlers: list = []


class QtLogHandler(QObject, logging.Handler):
    """
    Custom logging.Handler that emits a Qt signal for each log record.
    Connect the `message_logged` signal to any slot that accepts a str.
    Thread-safe: Qt will marshal the signal to the main thread automatically.
    """

    message_logged = Signal(str)

    def __init__(self, parent=None):
        QObject.__init__(self, parent)
        logging.Handler.__init__(self)
        formatter = logging.Formatter("[%(levelname)s] %(message)s")
        self.setFormatter(formatter)

    def emit(self, record: logging.LogRecord) -> None:
        """Format the record and emit the signal."""
        try:
            msg = self.format(record)
            self.message_logged.emit(msg)
        except Exception:
            self.handleError(record)


class AppLogger:
    """
    Application logging manager.
    Configures Python logging with:
    - A file handler (rotating, max 5 MB, 3 backups)
    - A Qt handler (routes to LogPanel via signal)
    - A console handler (debug builds)
    """

    LOG_DIR_NAME = "RoboticsAnalyzer"
    LOG_FILE_NAME = "app.log"
    MAX_BYTES = 5 * 1024 * 1024   # 5 MB
    BACKUP_COUNT = 3

    def __init__(self, level: str = "INFO"):
        self._logger = logging.getLogger("robotics_analyzer")
        self._qt_handler: QtLogHandler | None = None
        self._file_handler: logging.handlers.RotatingFileHandler | None = None
        self._log_path: Path | None = None
        self._setup(level)

    def _setup(self, level: str) -> None:
        """Configure all log handlers."""
        self._logger.setLevel(logging.DEBUG)  # Root level: allow all; handlers filter

        numeric_level = getattr(logging, level.upper(), logging.INFO)

        # A new AppLogger replaces the handlers of the previous one (no duplicates)
        for h in _added_handlers:
            self._logger.removeHandler(h)
            try:
                h.close()
            except Exception:  # noqa: BLE001
                pass
        _added_handlers.clear()

        # --- File handler (fallback: %TEMP%, then console only) ---
        import tempfile
        candidates = [self._get_log_dir(),
                      Path(tempfile.gettempdir()) / self.LOG_DIR_NAME / "logs"]
        for log_dir in candidates:
            try:
                log_dir.mkdir(parents=True, exist_ok=True)
                path = log_dir / self.LOG_FILE_NAME
                self._file_handler = logging.handlers.RotatingFileHandler(
                    path,
                    maxBytes=self.MAX_BYTES,
                    backupCount=self.BACKUP_COUNT,
                    encoding="utf-8"
                )
                self._log_path = path
                break
            except OSError:
                self._file_handler = None
        if self._file_handler is not None:
            self._file_handler.setLevel(logging.DEBUG)
            file_fmt = logging.Formatter(
                "%(asctime)s [%(levelname)-8s] %(name)s: %(message)s",
                datefmt="%Y-%m-%d %H:%M:%S"
            )
            self._file_handler.setFormatter(file_fmt)
            self._logger.addHandler(self._file_handler)
            _added_handlers.append(self._file_handler)
        else:
            console = logging.StreamHandler()
            console.setLevel(logging.DEBUG)
            self._logger.addHandler(console)
            _added_handlers.append(console)

        # --- Qt handler (connects to LogPanel in wire_to_panel) ---
        self._qt_handler = QtLogHandler()
        self._qt_handler.setLevel(numeric_level)
        self._logger.addHandler(self._qt_handler)
        _added_handlers.append(self._qt_handler)

        # Mark startup
        self._logger.info(
            "Robotics Analyzer logging initialized. "
            f"Level={level}. Log file={self._log_path}"
        )

    def wire_to_panel(self, panel) -> None:
        """
        Connect the Qt log handler to a LogPanel.
        Call this after the MainWindow and LogPanel are created.
        """
        if self._qt_handler is not None:
            self._qt_handler.message_logged.connect(panel.append)

    def set_level(self, level: str) -> None:
        """Change the log level for the Qt handler at runtime."""
        if self._qt_handler is not None:
            numeric = getattr(logging, level.upper(), logging.INFO)
            self._qt_handler.setLevel(numeric)

    @property
    def log_path(self) -> Path | None:
        """Return the path to the current log file."""
        return self._log_path

    @property
    def qt_handler(self) -> QtLogHandler | None:
        """Return the Qt log handler instance."""
        return self._qt_handler

    @staticmethod
    def _get_log_dir() -> Path:
        """Return the platform log directory for the application."""
        app_data = os.environ.get("APPDATA") or os.path.expanduser("~")
        return Path(app_data) / AppLogger.LOG_DIR_NAME / "logs"

    def shutdown(self) -> None:
        """Flush and close all handlers. Call on application exit."""
        logging.shutdown()
