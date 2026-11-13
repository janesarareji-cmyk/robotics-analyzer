"""A1.7: small app/API fixes from the audit of 2026-11-13 (LOW items)."""
import logging
import os
import threading
import time

import numpy as np
import pytest

import robotics_api as api
from robotics_api import ExportError, InvalidArgument, Robot
from robotics_api._util import pose4, vec, xyz_rpy

from .test_cartesian_traj import planar_2r
from .test_ui_workspace import _load, win  # noqa: F401  (fixture)


class SyncRunner:
    """Runs a task at once on the calling thread: a result is lost if nobody listens yet."""

    def __init__(self, real=None):
        self.tasks = []
        self._real = real

    def __getattr__(self, name):            # shutdown, wait_all, ... of the real runner
        return getattr(self._real, name)

    def submit_fn(self, fn):
        from app.task_runner import FunctionTask
        t = FunctionTask(fn)
        self.tasks.append(t)
        t.run()
        return t

    def submit(self, task):
        self.tasks.append(task)
        task.run()



# ------------------------------------------------------------------ 1 connect before submit
def test_ik_panel_connects_before_submit(win):
    """Audit LOW: a fast IK task must not lose its result."""
    _load(win)
    p = win.ik_panel
    p._runner = SyncRunner(p._runner)
    p.solve()
    assert p._running is False and len(p._solutions) > 0


def test_workspace_panel_connects_before_submit(win):
    """Audit LOW: a fast workspace task must not lose its result."""
    _load(win)
    p = win.workspace_panel
    p._backend.setCurrentText("cpu")
    p._samples.setValue(2000)
    p._runner = SyncRunner(p._runner)
    p.run()
    assert p._running is False and p._result is not None


def test_main_window_import_connects_before_submit(win, tmp_path):
    """Audit LOW: URDF import and export tasks connect before the task starts."""
    from tests.python.test_ui_workspace import FX
    got = []
    real = win._task_runner
    win._task_runner = SyncRunner(real)
    win._on_urdf_import_result = lambda r: got.append(r)
    try:
        win.import_urdf_file(str(FX / "planar_2r.urdf"))
    finally:
        win._task_runner = real
    assert len(got) == 1


# ------------------------------------------------------------------ 2 logger
def test_logger_handlers_not_duplicated_and_dir_fallback(tmp_path, monkeypatch, qapp):
    """Audit LOW: AppLogger removes its old handlers; unwritable log dir falls back."""
    from app.logger import AppLogger
    monkeypatch.setenv("APPDATA", str(tmp_path))
    lg = logging.getLogger("robotics_analyzer")
    before = len(lg.handlers)
    a = AppLogger()
    b = AppLogger()
    assert len(lg.handlers) == before + 2          # not +4
    blocker = tmp_path / "blocked"
    blocker.write_text("x")                        # a file where a dir is needed
    monkeypatch.setenv("APPDATA", str(blocker))
    monkeypatch.setenv("TEMP", str(tmp_path / "tmp"))
    monkeypatch.setenv("TMP", str(tmp_path / "tmp"))
    c = AppLogger()                                # must not raise
    assert c.log_path is None or c.log_path.parent.exists()
    for h in list(lg.handlers)[before:]:
        lg.removeHandler(h)
    del a, b


# ------------------------------------------------------------------ 3 mesh export names
def test_mesh_export_unique_by_safe_name(tmp_path):
    """Audit LOW: links 'a b' and 'a/b' share the safe name: files must not overwrite."""
    from app.importers.visuals import LinkVisual
    from app.reports.mesh_export import export_meshes
    vs = [LinkVisual(0, "a b", "box", {"size": [1, 1, 1]}),
          LinkVisual(1, "a/b", "box", {"size": [1, 1, 1]})]
    paths = export_meshes(vs, tmp_path, "stl")
    assert len(paths) == 2 and len(set(paths)) == 2 and all(os.path.exists(p) for p in paths)


# ------------------------------------------------------------------ 4 csv export
def test_csv_meta_newlines_and_duplicate_timestamps(tmp_path):
    """Audit LOW: meta cannot break the header; duplicate timestamps keep their rows."""
    from app.sim.export import export_csv
    arrays = {"a": (np.array([0.0, 1.0, 1.0, 2.0]), np.arange(4.0)),
              "b": (np.array([0.0, 1.0, 2.0]), np.arange(3.0))}
    p = tmp_path / "x.csv"
    n = export_csv(arrays, p, meta={"note\r\nx": "line1\nline2\r"})
    lines = p.read_text(encoding="utf-8").split("\n")
    assert lines[0] == "# note  x: line1 line2 " and lines[1].startswith("# columns:")
    assert lines[2].startswith("t [s]")
    assert n == 4                                  # 4 samples of 'a', one is a duplicate time
    rows = [ln for ln in lines if ln and not ln.startswith("#") and not ln.startswith("t [s]")]
    assert len(rows) == 4
    same = tmp_path / "y.csv"
    assert export_csv({"a": arrays["a"]}, same) == 4        # aligned path: same row count


# ------------------------------------------------------------------ 5 api util
def test_api_util_strict():
    """Audit LOW: vec refuses strings/objects; pose4 checks rotation; xyz_rpy in guard."""
    with pytest.raises(InvalidArgument):
        vec("abc", 3, "q")
    with pytest.raises(InvalidArgument):
        vec(np.array([1, 2, 3], dtype=object), 3, "q")
    bad = np.eye(4)
    bad[:3, :3] = 2.0 * np.eye(3)
    with pytest.raises(InvalidArgument):
        pose4(bad)
    refl = np.eye(4)
    refl[2, 2] = -1.0
    with pytest.raises(InvalidArgument):
        pose4(refl)
    pose4(np.eye(4))
    with pytest.raises(InvalidArgument):
        xyz_rpy(np.eye(4) * np.array([[1, 1, 1, 1]] * 4) * 2)   # not a valid pose


# ------------------------------------------------------------------ 6 pdf thread
def test_pdf_report_runs_on_gui_thread(tmp_path, qapp, monkeypatch):
    """Audit LOW: PDF written from a script worker thread runs on the GUI thread."""
    import app.reports.pdf_report as pr
    from PySide6.QtCore import QThread
    where = []
    real = pr.write_pdf

    def spy(d, path):
        where.append(QThread.currentThread())
        return real(d, path)

    monkeypatch.setattr(pr, "write_pdf", spy)
    robot = Robot(planar_2r(), "planar")
    out = {}

    def work():
        try:
            out["r"] = api.generate_report(robot, str(tmp_path / "r.pdf"))
        except BaseException as exc:  # noqa: BLE001
            out["e"] = exc

    t = threading.Thread(target=work, daemon=True)
    t.start()
    t0 = time.monotonic()
    while t.is_alive() and time.monotonic() - t0 < 30:
        qapp.processEvents()
        t.join(0.01)
    assert "e" not in out, out.get("e")
    assert where == [qapp.thread()] and (tmp_path / "r.pdf").stat().st_size > 0


def test_pdf_report_without_app_off_main_thread_raises(tmp_path, monkeypatch):
    """Audit LOW: no QApplication and not the main thread -> ExportError, no app created."""
    from PySide6.QtGui import QGuiApplication
    monkeypatch.setattr(QGuiApplication, "instance", staticmethod(lambda: None))
    robot = Robot(planar_2r(), "planar")
    out = {}

    def work():
        try:
            api.generate_report(robot, str(tmp_path / "r.pdf"))
        except BaseException as exc:  # noqa: BLE001
            out["e"] = exc

    t = threading.Thread(target=work, daemon=True)
    t.start()
    t.join(20)
    assert isinstance(out.get("e"), ExportError) and "GUI thread" in str(out["e"])
