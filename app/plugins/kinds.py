"""Plugin kinds (R8.2): importer, planner, analysis panel, and result validation.

A result that fails validation is "garbage": the plugin is reported and disabled for the session.
"""
from __future__ import annotations

import numpy as np

from app.importers.import_report import ImportReport, ImportResult
from app.kin_utils import dof_limits
from app.planning.planner import PlanResult

from .host import PluginCallError

TOL = 1e-9


def _garbage(msg: str) -> PluginCallError:
    return PluginCallError("garbage", msg)


# ----------------------------------------------------------------------------- importer
def validate_import_result(res) -> ImportResult:
    """Plugin ``import_file`` result -> ``ImportResult``. A bare ``RobotModel`` is accepted.

    A result with ``model is None`` is a legal failure (needs a report). A model must be a URM
    ``RobotModel`` and pass ``validate()`` with no error.
    """
    import core_bridge as cb
    if isinstance(res, cb.RobotModel):
        res = ImportResult(res)
    if not hasattr(res, "model"):
        raise _garbage(f"importer returned {type(res).__name__}, expected ImportResult or RobotModel")
    model = res.model
    report = getattr(res, "report", None)
    if report is None:
        report = ImportReport()
    elif not isinstance(report, ImportReport):
        raise _garbage("importer report must be an ImportReport")
    visuals, collisions = getattr(res, "visuals", []), getattr(res, "collisions", [])
    if not isinstance(visuals, list) or not isinstance(collisions, list):
        raise _garbage("importer visuals / collisions must be lists")
    if model is not None:
        if not isinstance(model, cb.RobotModel):
            raise _garbage(f"importer model is {type(model).__name__}, expected RobotModel")
        errors = model.validate()
        if errors:
            raise _garbage("importer model failed validate(): " + "; ".join(map(str, errors[:3])))
    return ImportResult(model, report, list(visuals), list(collisions))


# ----------------------------------------------------------------------------- planner
def validate_plan_result(res, model, start_q, goal_q, limits=None, planner: str = "") -> PlanResult:
    """Plugin plan result -> ``PlanResult``. Success needs a finite ``(n, dof)`` path that starts
    at ``start_q``, ends at ``goal_q`` (joint-vector goals) and stays in the joint limits."""
    if not hasattr(res, "success") or not hasattr(res, "path"):
        raise _garbage(f"planner returned {type(res).__name__}, expected a PlanResult")
    success = res.success
    if not isinstance(success, (bool, np.bool_)):
        raise _garbage("planner 'success' must be a bool")
    time_s = float(getattr(res, "time", 0.0) or 0.0)
    its = int(getattr(res, "iterations", 0) or 0)
    reason = str(getattr(res, "reason", "") or "")
    if not success:
        return PlanResult(None, False, time_s, its, reason or "plugin reported failure", planner)
    n = model.dof()
    try:
        path = np.array(res.path, dtype=float)
    except Exception:  # noqa: BLE001
        raise _garbage("planner path is not a numeric array") from None
    if path.ndim != 2 or path.shape[1] != n or path.shape[0] < 1:
        raise _garbage(f"planner path has shape {path.shape}, expected (n, {n})")
    if not np.all(np.isfinite(path)):
        raise _garbage("planner path has NaN or inf")
    start = np.asarray(start_q, float)
    if np.max(np.abs(path[0] - start)) > TOL:
        raise _garbage("planner path does not start at start_q")
    goal = np.asarray(goal_q, float)
    if goal.shape == (n,) and np.max(np.abs(path[-1] - goal)) > TOL:
        raise _garbage("planner path does not end at goal_q")
    lo, hi = limits if limits is not None else dof_limits(model)
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    if np.any(path < lo - TOL) or np.any(path > hi + TOL):
        raise _garbage("planner path leaves the joint limits")
    length = float(np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1))) if len(path) > 1 else 0.0
    return PlanResult(path, True, time_s, its, reason, planner, length, length,
                      len(path) - 1, dict(getattr(res, "info", None) or {}))


class PluginPlanner:
    """Callable with the ``plan`` signature: runs the plugin guarded and validates the result."""

    def __init__(self, plugin, label: str, fn, timeout: float | None = None):
        self.plugin, self.label, self.fn, self.timeout = plugin, label, fn, timeout

    @property
    def key(self) -> str:
        return f"plugin:{self.plugin.name}:{self.label}"

    @property
    def display(self) -> str:
        return f"{self.label} (plugin)"

    def __call__(self, model, start_q, goal_q, collision=None, limits=None, cancel=None, **kw):
        kw.pop("planner", None)
        tl = float(kw.get("time_limit", 5.0))
        timeout = self.timeout if self.timeout is not None else tl * 2 + 10
        cancel_evt = cancel if hasattr(cancel, "is_set") else None
        lim = None
        if limits is not None:
            lim = (list(limits[0]), list(limits[1]))
        return self.plugin.call(
            self.fn, (model, start_q, goal_q), dict(collision=collision, limits=lim, cancel=cancel, **kw),
            timeout=timeout, cancel=cancel_evt,
            validate=lambda r: validate_plan_result(r, model, start_q, goal_q, lim, self.label))


class PluginImporter:
    """Importer registered by a plugin: file filter + guarded ``import_file``."""

    def __init__(self, plugin, label: str, file_filter: str, fn, timeout: float | None = None):
        self.plugin, self.label, self.file_filter, self.fn, self.timeout = (
            plugin, label, file_filter, fn, timeout)

    def import_file(self, path: str) -> ImportResult:
        self.plugin.host._allow_read(path)
        try:
            return self.plugin.call(self.fn, (str(path),), {}, timeout=self.timeout,
                                    validate=validate_import_result)
        finally:
            self.plugin.host._clear_allowed()


class PluginAnalysis:
    """Analysis panel registered by a plugin. ``build`` runs on the GUI thread (no timeout)."""

    def __init__(self, plugin, title: str, factory):
        self.plugin, self.title, self.factory = plugin, title, factory

    def build(self, view):
        from PySide6.QtWidgets import QWidget
        try:
            w = self.factory(view)
            if not isinstance(w, QWidget):
                raise _garbage(f"analysis factory returned {type(w).__name__}, expected a QWidget")
            return w
        except PluginCallError as exc:
            self.plugin.fail(exc.reason, str(exc))
            raise
        except PermissionError as exc:
            raise PluginCallError("permission", str(exc), self.plugin.name) from exc
        except BaseException as exc:  # noqa: BLE001 - foreign code
            if isinstance(exc, KeyboardInterrupt):
                raise
            msg = f"{type(exc).__name__}: {exc}"
            self.plugin.fail("raised", msg)
            raise PluginCallError("raised", msg, self.plugin.name) from exc
