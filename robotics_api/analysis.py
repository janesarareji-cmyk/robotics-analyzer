"""Workspace analysis and data export of the API (R8.1)."""
from __future__ import annotations

from typing import Any

import numpy as np

from ._cancel import check_cancel, current_token
from ._util import guard
from .errors import AnalysisError, ExportError, InvalidArgument, ScriptCancelled
from .robot import Robot


@guard(AnalysisError)
def workspace(robot: Robot, n_samples: int = 200_000, voxel: float | None = None,
              seed: int = 42, metrics: bool = False) -> dict:
    """Monte Carlo workspace of the TCP position (CPU).

    ``n_samples`` random joint vectors inside the joint limits (all limits must be finite);
    ``voxel``: voxel edge in m (default reach / 100); ``seed``: random seed; ``metrics=True``
    adds per-voxel singular-value and manipulability statistics. Returns a dict of numpy arrays
    and numbers: ``volume`` (m^3, voxel count x voxel^3), ``volume_err_bound`` (m^3), ``filled``
    (voxel count), ``voxel_size`` (m), ``origin`` (m, grid corner), ``dims`` (grid size),
    ``n_samples``, ``keys`` (linear voxel index), ``count`` (samples per filled voxel) and
    ``centers`` (filled, 3) voxel centres in the base frame (m). Cancel-aware.
    """
    from app.analysis.workspace_analysis import analyze_workspace, voxel_centers
    n = int(n_samples)
    if n < 1:
        raise InvalidArgument("n_samples must be >= 1")
    opts: dict = {"n_samples": n, "seed": int(seed), "want_metrics": bool(metrics)}
    if voxel is not None:
        if not (float(voxel) > 0):
            raise InvalidArgument("voxel must be > 0")
        opts["voxel_size"] = float(voxel)
    check_cancel()
    res = analyze_workspace(robot._model, opts, cancel=current_token())
    if res.get("cancelled"):
        check_cancel()
        raise ScriptCancelled("workspace cancelled")
    res["centers"] = voxel_centers(res)
    return res


def _arrays(result: dict, names) -> dict:
    """Recorder-style ``{name: (t, data)}`` from a ``simulate`` result dict."""
    if not isinstance(result, dict) or "t" not in result:
        raise InvalidArgument("result must be the dict returned by simulate (has key 't')")
    t = np.asarray(result["t"], dtype=float)
    keys = [k for k in result if k != "t"] if names is None else list(names)
    out = {}
    for k in keys:
        if k not in result:
            raise InvalidArgument(f"unknown signal {k!r}")
        d = np.asarray(result[k], dtype=float)
        if d.ndim == 1:
            d = d.reshape(-1, 1)
        if d.ndim != 2 or len(d) != len(t):
            raise InvalidArgument(f"signal {k!r} must have one row per time sample")
        out[k] = (t, d)
    return out


def _meta(meta: dict | None) -> dict:
    from app.sim.export import build_meta
    m = build_meta()
    m.update(dict(meta or {}))
    return m


@guard(ExportError)
def export_csv(result: dict, path: str, names: list[str] | None = None,
               meta: dict | None = None) -> int:
    """Write the ``simulate`` result to a CSV file (time column ``t [s]`` plus one column per
    signal entry, units in the header, metadata in ``#`` comment lines). Returns the row count."""
    from app.sim.export import export_csv as _csv
    return int(_csv(_arrays(result, names), str(path), None, _meta(meta)))


@guard(ExportError)
def export_npz(result: dict, path: str, names: list[str] | None = None,
               meta: dict | None = None) -> int:
    """Write the ``simulate`` result to a compressed NPZ file (``<name>_t``, ``<name>``, JSON
    ``meta``). Returns the row count."""
    from app.sim.export import export_npz as _npz
    return int(_npz(_arrays(result, names), str(path), _meta(meta), None))


@guard(ExportError)
def export_urdf(robot: Robot, path: str, copy_meshes: bool = True) -> dict:
    """Write the robot as a URDF file (R8.3). DH joints are converted to origin/axis; link
    visuals and collision shapes are written; mesh files found on disk are copied to a
    ``meshes`` folder next to ``path`` and referenced by relative paths.

    Returns ``{"path": str, "meshes": [relative mesh paths], "warnings": [str]}``.
    """
    from app.reports.urdf_export import export_urdf as _exp
    res = _exp(robot._model, str(path), robot._visuals, robot._collisions, copy_meshes)
    return {"path": res.path, "meshes": list(res.meshes), "warnings": list(res.warnings)}


@guard(ExportError)
def generate_report(robot: Robot, path: str, result: dict | None = None,
                    workspace_result: dict | None = None, plan: Any = None,
                    fmt: str | None = None) -> dict:
    """Write a robot report (R8.3) as HTML or PDF; ``fmt`` ("html" / "pdf") defaults to the
    file extension of ``path``.

    Sections: summary, joint table, DH or origin/axis table, mass and COM; optional:
    ``workspace_result`` (the dict returned by ``workspace``), ``result`` (the dict returned by
    ``simulate``: graphs), ``plan`` (a path array from ``plan``: length). A missing optional
    section is left out and named in the returned ``missing`` list.

    Returns ``{"path": str, "format": str, "pages": int | None, "missing": [str]}``.
    """
    import numpy as _np
    from app.reports.report import build_report_data
    ext = (fmt or str(path).rsplit(".", 1)[-1]).lower()
    if ext not in ("html", "pdf"):
        raise InvalidArgument("fmt must be 'html' or 'pdf'")
    arrays = _arrays(result, None) if result is not None else None
    data = build_report_data(robot._model, robot.name, workspace=workspace_result, arrays=arrays)
    if plan is not None:
        pts = _np.asarray(plan, dtype=float)
        if pts.ndim == 2 and len(pts) > 1:
            data.plan = dict(planner="(api)", success=True, time=0.0, iterations=0,
                             reason="", n_waypoints=len(pts), violations=None,
                             length=float(_np.linalg.norm(_np.diff(pts, axis=0), axis=1).sum()))
            data.missing = [m for m in data.missing if not m.startswith("plan")]
    pages = None
    if ext == "html":
        from app.reports.html_report import write_html
        write_html(data, path)
    else:
        pages = _write_pdf_on_gui_thread(data, path)
    return {"path": str(path), "format": ext, "pages": pages, "missing": list(data.missing)}


_QAPP = None
_INVOKER = None


def _write_pdf_on_gui_thread(data, path) -> int:
    """Write the PDF on the GUI thread (QTextDocument / QPdfWriter are GUI-thread classes).

    From a worker thread the call is blocking-queued onto the GUI thread (the GUI event loop
    must be running, as it is while a script runs). With no QApplication: created only on the
    main thread, else ``ExportError``.
    """
    import threading
    from PySide6.QtCore import QCoreApplication, QMetaObject, QObject, Qt, QThread, Slot
    from PySide6.QtGui import QGuiApplication
    from app.reports.pdf_report import write_pdf
    global _QAPP, _INVOKER
    app = QGuiApplication.instance()
    if app is None:
        if threading.current_thread() is not threading.main_thread():
            raise ExportError("PDF needs the GUI thread")
        _QAPP = app = QGuiApplication([])
    if QThread.currentThread() == app.thread():
        return write_pdf(data, path)

    box: dict = {}

    class _Invoker(QObject):
        @Slot()
        def run(self):
            try:
                box["v"] = write_pdf(data, path)
            except BaseException as exc:  # noqa: BLE001 - re-raised on the calling thread
                box["e"] = exc

    inv = _Invoker()
    inv.moveToThread(app.thread())
    _INVOKER = inv
    QMetaObject.invokeMethod(inv, "run", Qt.ConnectionType.BlockingQueuedConnection)
    _INVOKER = None
    if "e" in box:
        raise box["e"]
    return box["v"]
