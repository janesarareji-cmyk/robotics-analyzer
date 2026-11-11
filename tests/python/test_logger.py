"""
tests/python/test_logger.py
R1.3: AppLogger and QtLogHandler unit tests.
"""

import os
import logging
import tempfile
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from app.logger import AppLogger, QtLogHandler, log


@pytest.fixture
def temp_logger(tmp_path, monkeypatch):
    """Create an AppLogger that writes to a temp directory."""
    # Redirect log directory to tmp_path
    monkeypatch.setenv("APPDATA", str(tmp_path))
    logger = AppLogger(level="DEBUG")
    yield logger
    logger.shutdown()


def test_app_logger_creation(temp_logger):
    """AppLogger creates without error."""
    assert temp_logger is not None


def test_app_logger_creates_log_file(temp_logger):
    """AppLogger creates a log file on disk."""
    assert temp_logger.log_path is not None
    assert temp_logger.log_path.exists()


def test_app_logger_writes_to_file(temp_logger):
    """Log messages appear in the log file."""
    test_msg = "R1.3 test log message unique 99887766"
    log.info(test_msg)

    # Flush the file handler
    for handler in logging.getLogger("robotics_analyzer").handlers:
        handler.flush()

    content = temp_logger.log_path.read_text(encoding="utf-8")
    assert test_msg in content


def test_qt_handler_creation():
    """QtLogHandler creates without error."""
    handler = QtLogHandler()
    assert handler is not None
    assert handler.message_logged is not None


def test_qt_handler_emits_signal(qapp):
    """QtLogHandler emits message_logged signal when a record is handled."""
    received = []
    handler = QtLogHandler()
    handler.message_logged.connect(received.append)

    record = logging.LogRecord(
        name="test",
        level=logging.INFO,
        pathname="",
        lineno=0,
        msg="Hello from test",
        args=(),
        exc_info=None
    )
    handler.emit(record)

    assert len(received) == 1
    assert "Hello from test" in received[0]


def test_qt_handler_formats_level(qapp):
    """QtLogHandler includes the log level in the emitted message."""
    received = []
    handler = QtLogHandler()
    handler.message_logged.connect(received.append)

    record = logging.LogRecord(
        name="test",
        level=logging.WARNING,
        pathname="",
        lineno=0,
        msg="Warning test",
        args=(),
        exc_info=None
    )
    handler.emit(record)

    assert len(received) == 1
    assert "WARNING" in received[0]


def test_logger_wire_to_panel(qapp, temp_logger):
    """wire_to_panel connects logger to a mock panel."""
    received = []

    class MockPanel:
        def append(self, msg):
            received.append(msg)

    panel = MockPanel()
    temp_logger.wire_to_panel(panel)

    # Allow Qt to process events before checking
    from PySide6.QtWidgets import QApplication
    log.info("Wire test message 112233")
    QApplication.processEvents()

    assert any("Wire test message 112233" in m for m in received)


def test_logger_set_level(temp_logger):
    """set_level changes the Qt handler log level."""
    temp_logger.set_level("WARNING")
    assert temp_logger.qt_handler.level == logging.WARNING


def test_logger_log_path_is_in_appdata(temp_logger, tmp_path):
    """Log file path is inside the redirected APPDATA directory."""
    assert str(tmp_path) in str(temp_logger.log_path)
