"""U2.3 floor grid + drag-and-drop."""

import os

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtCore import QCoreApplication, QMimeData, QPoint, QPointF, Qt, QUrl
from PySide6.QtGui import QDropEvent

from app.importers.urdf import import_urdf
from app.main_window import MainWindow
from app.sample_robots import list_sample_robots
from app.settings import AppSettings
from app.viewport.robot_view import RobotView

SAMPLES = {p.parent.name: (t, p) for t, p in list_sample_robots()}


@pytest.fixture
def win(qapp, monkeypatch):
    monkeypatch.setattr(AppSettings, "_ORGANIZATION", "RoboticsAnalyzer_test_u2f")
    monkeypatch.setattr(AppSettings, "_APPLICATION", "RoboticsAnalyzer_test_u2f")
    w = MainWindow()
    w.settings._settings.clear()
    yield w
    w.project_controller._is_modified = False
    w.settings._settings.clear()
    w.close()


def _load(win, name):
    path = SAMPLES[name][1]
    res = import_urdf(str(path))
    win._import_path = str(path)
    win._on_urdf_import_result(res)
    QCoreApplication.processEvents()


def _drop(win, path):
    md = QMimeData()
    md.setUrls([QUrl.fromLocalFile(str(path))])
    ev = QDropEvent(QPointF(10, 10), Qt.DropAction.CopyAction, md,
                    Qt.MouseButton.LeftButton, Qt.KeyboardModifier.NoModifier)
    win.dropEvent(ev)
    return ev


def test_half_size_rounding():
    f = [np.eye(4), np.eye(4)]
    f[1][:3, 3] = [0.1, 0, 0.4]
    assert RobotView.floor_half_size(f, f[1]) == 0.5
    f[1][:3, 3] = [0.6, -0.2, 0]
    assert RobotView.floor_half_size(f, f[1]) == 1.0
    f[1][:3, 3] = [1.0, 0, 0]
    assert RobotView.floor_half_size(f, f[1]) == 1.0


def test_floor_exists_toggle_rebuild(win):
    _load(win, "ur5")
    rv = win._viewport.robot_view
    assert rv.show_floor and len(rv.floor_actors) == 2
    z, half = rv.floor_info
    assert z == pytest.approx(-0.001)
    assert half >= 1.0
    assert all(a.visibility for a in rv.floor_actors)
    win._action_show_floor.setChecked(False)
    assert not any(a.visibility for a in rv.floor_actors)
    assert win.settings.show_floor() is False
    first = rv.floor_actors
    _load(win, "xarm7")
    assert rv.floor_actors and rv.floor_actors[0] is not first[0]
    assert not any(a.visibility for a in rv.floor_actors)    # toggle persists
    win._action_show_floor.setChecked(True)
    assert all(a.visibility for a in rv.floor_actors)
    assert win.settings.show_floor() is True


def test_floor_default_on(win):
    assert win._action_show_floor.isChecked()
    assert win._viewport.robot_view.show_floor


def test_drop_urdf(win, monkeypatch):
    got = []
    monkeypatch.setattr(win, "import_urdf_file", lambda p: got.append(p))
    p = SAMPLES["ur3"][1]
    ev = _drop(win, p)
    assert ev.isAccepted()
    assert got == [str(p)] or [os.path.normcase(g) for g in got] == [os.path.normcase(str(p))]


def test_drop_robotproj(win, monkeypatch, tmp_path):
    got = []
    monkeypatch.setattr(win.project_controller, "open_project",
                        lambda parent, path=None: got.append(path) or True)
    p = tmp_path / "a.robotproj"
    p.write_bytes(b"x")
    _drop(win, p)
    assert len(got) == 1 and os.path.basename(got[0]) == "a.robotproj"


def test_drop_other_ignored(win, monkeypatch, tmp_path):
    called = []
    monkeypatch.setattr(win, "import_urdf_file", lambda p: called.append(p))
    monkeypatch.setattr(win.project_controller, "open_project",
                        lambda *a, **k: called.append(a))
    p = tmp_path / "a.txt"
    p.write_text("x")
    _drop(win, p)
    assert called == []
    assert "ignored" in win._status_label.text()
