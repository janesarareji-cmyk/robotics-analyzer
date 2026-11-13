"""Standalone: create MainWindow, show, close, delete, exit. Exit code 0 expected."""
import os, sys
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
from pathlib import Path
root = Path(__file__).resolve().parents[2]
for p in (root, root / "bindings"):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))
from PySide6.QtWidgets import QApplication
from PySide6.QtCore import QEvent
from app.main_window import MainWindow

app = QApplication.instance() or QApplication([])
win = MainWindow()
win.show()
app.processEvents()
win.close()
win.deleteLater()
app.sendPostedEvents(None, QEvent.DeferredDelete)
app.processEvents()
del win
sys.exit(0)
