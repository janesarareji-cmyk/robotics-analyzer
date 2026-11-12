"""
app/script_runner.py
R8.1: run a Python script (file or editor text) in a worker thread.

* The script sees ``robotics_api`` (also as ``api``) and ``app``, a read-only snapshot of the
  open project (``AppHandle``).
* stdout / stderr of the worker thread are routed to ``output(text, is_err)`` line by line
  (thread-aware streams: output of other threads, and of the app itself, is NOT hijacked).
* Cancel is cooperative: ``cancel()`` sets a thread-local ``CancelToken``; ``robotics_api.simulate``,
  ``plan``, ``workspace`` (and scripts calling ``robotics_api.check_cancel()``) raise
  ``ScriptCancelled``. A script stuck in its own pure Python loop cannot be stopped.
* A script exception never leaves the runner: the traceback goes to ``output`` and the status is
  ``"error"``. Time limit (``time_limit`` seconds, None = none) is enforced the same cooperative way.

Other tools (R8.2 plugins) can reuse it: call ``run_string`` / ``run_file`` and connect
``output`` and ``finished``; see ``docs/scripting.md``.
"""

from __future__ import annotations

import os
import sys
import threading
import traceback
from typing import Any, Callable

from PySide6.QtCore import QObject, Signal

import robotics_api
from robotics_api import CancelToken, ScriptCancelled, ScriptTimeLimit, cancel_scope

from app.task_runner import FunctionTask, TaskRunner

STATUS_OK, STATUS_ERROR, STATUS_CANCELLED, STATUS_TIMEOUT = "ok", "error", "cancelled", "timeout"


# --------------------------------------------------------------------------------- streams
_sinks: dict[int, Callable[[str, bool], None]] = {}
_buffers: dict[tuple[int, bool], str] = {}
_lock = threading.Lock()
_install_count = 0
_saved: dict[str, Any] = {}


class _ThreadStream:
    """Text stream that routes ``write`` by thread id; other threads go to the original stream."""

    def __init__(self, original: Any, is_err: bool):
        self._orig = original
        self._is_err = is_err

    def write(self, text: str) -> int:
        tid = threading.get_ident()
        sink = _sinks.get(tid)
        if sink is None:
            return self._orig.write(text)
        key = (tid, self._is_err)
        buf = _buffers.get(key, "") + str(text)
        *lines, rest = buf.split("\n")
        _buffers[key] = rest
        for line in lines:
            sink(line, self._is_err)
        return len(text)

    def flush(self) -> None:
        if _sinks.get(threading.get_ident()) is None:
            try:
                self._orig.flush()
            except Exception:  # noqa: BLE001
                pass

    def isatty(self) -> bool:
        return False

    def __getattr__(self, name: str) -> Any:        # encoding, buffer, fileno, ...
        return getattr(self._orig, name)


def _flush_thread(tid: int) -> None:
    sink = _sinks.get(tid)
    for is_err in (False, True):
        rest = _buffers.pop((tid, is_err), "")
        if rest and sink is not None:
            sink(rest, is_err)


def _install() -> None:
    global _install_count
    with _lock:
        if _install_count == 0:
            _saved["out"], _saved["err"] = sys.stdout, sys.stderr
            sys.stdout = _ThreadStream(sys.stdout, False)
            sys.stderr = _ThreadStream(sys.stderr, True)
        _install_count += 1


def _uninstall() -> None:
    global _install_count
    with _lock:
        _install_count = max(0, _install_count - 1)
        if _install_count == 0 and _saved:
            # restore unconditionally: a hostile script may have replaced a stream
            sys.stdout, sys.stderr = _saved["out"], _saved["err"]
            _saved.clear()


# ----------------------------------------------------------------------------- app handle
class AppHandle:
    """Read-only view of the open project, handed to scripts as ``app``.

    A snapshot taken when the script starts: ``has_project``, ``project_name``,
    ``project_path``, ``robot`` (a ``robotics_api.Robot`` copy or None), ``q`` (current joint
    vector of the project robot, list or None), ``version`` (app version).
    """

    __slots__ = ("_d",)

    def __init__(self, project: Any = None, version: str = ""):
        d: dict[str, Any] = {"has_project": project is not None, "project_name": None,
                             "project_path": None, "robot": None, "q": None, "version": version}
        if project is not None:
            d["project_name"] = str(project.name)
            d["project_path"] = project.file_path
            try:
                import core_bridge as cb
                from app.kin_utils import current_q
                model = project.robot_model
                d["robot"] = robotics_api.Robot(cb.urm_from_json(model.to_json()), project.name,
                                                project.visuals, project.collisions)
                d["q"] = current_q(model)
            except Exception:  # noqa: BLE001 - a half built project is still a project
                pass
        object.__setattr__(self, "_d", d)

    def __getattr__(self, name: str) -> Any:
        d = object.__getattribute__(self, "_d")
        if name in d:
            return d[name]
        raise AttributeError(name)

    def __setattr__(self, name: str, value: Any) -> None:
        raise AttributeError("app is a read-only view of the project")

    def __dir__(self):
        return sorted(object.__getattribute__(self, "_d"))

    def __repr__(self) -> str:
        d = object.__getattribute__(self, "_d")
        return f"<app project={d['project_name']!r}>"


# ------------------------------------------------------------------------------- runner
class ScriptRunner(QObject):
    """Runs one script at a time in a worker thread of the ``TaskRunner``.

    Signals: ``started()``, ``output(text, is_err)`` (one line per call), ``finished(status,
    message)`` with status ``ok`` / ``error`` / ``cancelled`` / ``timeout`` (message = traceback
    for ``error``). Attributes: ``last_status``, ``last_message``, ``time_limit`` (s or None).
    """

    started = Signal()
    output = Signal(str, bool)
    finished = Signal(str, str)
    _done = Signal(str, str)           # worker -> GUI thread

    def __init__(self, task_runner: TaskRunner, project_provider: Callable[[], Any] | None = None,
                 time_limit: float | None = None, app_version: str = "", parent: QObject | None = None):
        super().__init__(parent)
        self._runner = task_runner
        self._provider = project_provider
        self._app_version = app_version
        self.time_limit = time_limit
        self.last_status: str | None = None
        self.last_message = ""
        self._running = False
        self._token: CancelToken | None = None
        self._task: FunctionTask | None = None      # keep a reference (R7.4 bug)
        self._path_added: str | None = None
        self._done.connect(self._on_done)

    @property
    def is_running(self) -> bool:
        return self._running

    # ------------------------------------------------------------------ control
    def run_string(self, code: str, name: str = "<editor>") -> bool:
        """Run ``code``. Returns False (and logs) when a script is already running."""
        return self._start(str(code), name, None)

    def run_file(self, path: str) -> bool:
        """Run a script file (UTF-8). A script folder import works (its folder is on sys.path)."""
        try:
            with open(path, encoding="utf-8") as f:
                code = f.read()
        except (OSError, UnicodeDecodeError) as exc:
            self.last_status, self.last_message = STATUS_ERROR, f"cannot read {path}: {exc}"
            self.output.emit(self.last_message, True)
            self.finished.emit(STATUS_ERROR, self.last_message)
            return False
        return self._start(code, str(path), str(path))

    def cancel(self) -> None:
        """Ask the running script to stop (cooperative)."""
        if self._token is not None:
            self._token.cancel()

    # ------------------------------------------------------------------ internals
    def _start(self, code: str, name: str, file_path: str | None) -> bool:
        if self._running:
            self.output.emit("a script is already running", True)
            return False
        project = None
        try:
            project = self._provider() if self._provider is not None else None
        except Exception:  # noqa: BLE001
            project = None
        ns: dict[str, Any] = {
            "__name__": "__main__", "__builtins__": __builtins__,
            "robotics_api": robotics_api, "api": robotics_api,
            "app": AppHandle(project, self._app_version),
        }
        if file_path is not None:
            ns["__file__"] = os.path.abspath(file_path)
            folder = os.path.dirname(os.path.abspath(file_path))
            if folder not in sys.path:
                sys.path.insert(0, folder)
                self._path_added = folder
        token = CancelToken(self.time_limit)
        self._token = token
        self._running = True
        self.last_status, self.last_message = None, ""
        _install()

        def sink(text: str, is_err: bool) -> None:
            self.output.emit(text, is_err)

        def work() -> None:
            tid = threading.get_ident()
            _sinks[tid] = sink
            status, message = STATUS_OK, ""
            try:
                with cancel_scope(token):
                    exec(compile(code, name, "exec"), ns)        # noqa: S102 - the point
            except ScriptCancelled as exc:
                status = STATUS_TIMEOUT if isinstance(exc, ScriptTimeLimit) else STATUS_CANCELLED
                message = str(exc)
            except SystemExit as exc:
                message = f"script exited ({exc.code})"
            except BaseException as exc:  # noqa: BLE001 - never leave the worker
                status = STATUS_ERROR
                tb = exc.__traceback__
                if isinstance(exc, SyntaxError) or tb is None:
                    message = "".join(traceback.format_exception_only(type(exc), exc))
                else:
                    message = "".join(traceback.format_exception(type(exc), exc, tb.tb_next or tb))
            finally:
                _flush_thread(tid)
                if status == STATUS_ERROR:
                    for line in message.rstrip("\n").split("\n"):
                        sink(line, True)
                _sinks.pop(tid, None)
                self._done.emit(status, message)

        task = FunctionTask(work)
        self._task = task
        self.started.emit()
        self._runner.submit(task)
        return True

    def _on_done(self, status: str, message: str) -> None:
        self._running = False
        self._token = None
        self._task = None
        if self._path_added is not None:
            try:
                sys.path.remove(self._path_added)
            except ValueError:
                pass
            self._path_added = None
        _uninstall()
        self.last_status, self.last_message = status, message
        self.finished.emit(status, message)
