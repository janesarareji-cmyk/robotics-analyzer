"""
tests/python/conftest.py
pytest configuration — adds build paths so core_bridge can be imported.
"""
import os
import sys
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

# Add bindings/ directory to sys.path so core_bridge.pyd can be imported
bindings_dir = Path(__file__).parent.parent.parent / "bindings"
if str(bindings_dir) not in sys.path:
    sys.path.insert(0, str(bindings_dir))

# Add the repo root so the top-level packages `app` and `robotics_api` import under any pytest invocation
repo_root = Path(__file__).parent.parent.parent
if str(repo_root) not in sys.path:
    sys.path.insert(0, str(repo_root))


import pytest  # noqa: E402


@pytest.fixture(autouse=True)
def _no_modal_unsaved_prompt(monkeypatch):
    """A1.1: MainWindow.closeEvent now asks about unsaved changes. A test window that is
    closed while modified must not block on a real modal box (offscreen: hangs). Tests
    that care patch QMessageBox.question themselves (their patch wins)."""
    try:
        from PySide6.QtWidgets import QMessageBox
    except Exception:
        return
    monkeypatch.setattr(
        QMessageBox, "question",
        staticmethod(lambda *a, **k: QMessageBox.StandardButton.Discard))


def _delete_leftover_widgets():
    """Delete parentless top-level widgets (windows, panels a test left behind) through the
    Qt event loop. Widgets with a Python-side lambda cycle otherwise die inside the cyclic
    GC / at interpreter exit, where PySide6 destruction order is random (access violation
    or abort at process exit, A1.8). pyqtgraph-owned popups stay with their owners."""
    import gc
    from PySide6.QtCore import QEvent
    from PySide6.QtWidgets import QApplication
    app = QApplication.instance()
    if app is None:
        return
    for w in list(app.topLevelWidgets()):
        try:
            if w.parent() is not None or type(w).__module__.startswith("pyqtgraph"):
                continue
            if type(w).__name__ in ("QMenu", "QFrame", "QWidget") and type(w).__module__ == "PySide6.QtWidgets":
                continue   # popups / helper windows owned by other objects
            w.close()
            w.deleteLater()
        except RuntimeError:
            pass
    app.sendPostedEvents(None, QEvent.DeferredDelete)
    app.processEvents()
    gc.collect()
    app.sendPostedEvents(None, QEvent.DeferredDelete)


@pytest.fixture(scope="session", autouse=True)
def _qt_session_cleanup():
    """A1.8: delete leftover widgets before interpreter exit."""
    yield
    _delete_leftover_widgets()
