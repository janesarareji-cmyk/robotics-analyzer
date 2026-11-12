"""R8.1: ScriptRunner (worker thread, output routing, cancel, time limit, hostile scripts)."""
import os
import sys
import time
from pathlib import Path

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication

from app.project.project_data import ProjectData
from app.script_runner import AppHandle, ScriptRunner
from app.task_runner import TaskRunner


EX = Path(__file__).parent.parent.parent / "examples"
URDF = str(EX / "planar_2r.urdf")
LOAD = "import robotics_api as api\nrobot = api.import_urdf(r'" + URDF + "')\n"


@pytest.fixture
def runner(qapp):
    tr = TaskRunner()
    r = ScriptRunner(tr, project_provider=lambda: None, app_version="test")
    r.lines = []
    r.done = []
    r.output.connect(lambda t, e: r.lines.append((t, e)))
    r.finished.connect(lambda s, m: r.done.append((s, m)))
    yield r
    r.cancel()
    tr.wait_all(10000)
    QApplication.processEvents()
    tr.shutdown()


def wait(runner, timeout=20.0):
    t0 = time.monotonic()
    while runner.is_running and time.monotonic() - t0 < timeout:
        QApplication.processEvents()
        time.sleep(0.002)
    QApplication.processEvents()
    assert not runner.is_running, "script did not finish"


def texts(runner, err=None):
    return [t for t, e in runner.lines if err is None or e == err]


def test_print_and_stderr_routed_line_by_line(runner):
    stdout_before = sys.stdout
    assert runner.run_string("import sys\nprint('hello')\nprint('a', 'b')\n"
                             "sys.stderr.write('oops\\n')\nprint('no newline', end='')")
    wait(runner)
    assert texts(runner, False) == ["hello", "a b", "no newline"]
    assert texts(runner, True) == ["oops"]
    assert runner.done == [("ok", "")] and runner.last_status == "ok"
    assert sys.stdout is stdout_before                       # streams restored


def test_main_thread_output_is_not_hijacked(runner, capsys):
    runner.run_string("import time\nprint('script line')\ntime.sleep(0.3)\nprint('end')")
    t0 = time.monotonic()
    wrote = False
    while runner.is_running and time.monotonic() - t0 < 10:
        QApplication.processEvents()
        if not wrote:
            print("MAIN THREAD LINE")                          # while the script runs
            wrote = True
        time.sleep(0.005)
    wait(runner)
    assert wrote
    assert "MAIN THREAD LINE" not in texts(runner)
    assert "MAIN THREAD LINE" in capsys.readouterr().out
    assert texts(runner) == ["script line", "end"]


def test_script_exception_reported_not_raised(runner):
    runner.run_string("x = 1\nprint('before')\n1 / 0\nprint('never')", "<my script>")
    wait(runner)
    status, msg = runner.done[0]
    assert status == "error" and "ZeroDivisionError" in msg
    assert "<my script>" in msg and "line 3" in msg                      # script frame, not runner
    err_lines = texts(runner, True)
    assert any("ZeroDivisionError" in t for t in err_lines)
    assert "before" in texts(runner, False) and "never" not in texts(runner)
    # the runner is usable again
    assert runner.run_string("print('again')")
    wait(runner)
    assert runner.last_status == "ok" and texts(runner)[-1] == "again"


def test_api_error_syntax_error_and_missing_file(runner, tmp_path):
    runner.run_string("import robotics_api as a\na.import_urdf('nope.urdf')")
    wait(runner)
    assert runner.last_status == "error" and "ImportFailed" in runner.last_message
    runner.run_string("def broken(:\n  pass")
    wait(runner)
    assert runner.last_status == "error" and "SyntaxError" in runner.last_message
    assert runner.run_file(str(tmp_path / "missing.py")) is False
    assert runner.last_status == "error" and runner.done[-1][0] == "error"


def test_hostile_scripts_never_crash_runner(runner):
    for code in ("import sys\nsys.exit(3)", "raise SystemExit", "raise KeyboardInterrupt",
                 "import sys\nsys.stdout = None", "import sys, io\nsys.stderr = io.StringIO()",
                 "raise BaseException('base')", "def f():\n    f()\nf()",
                 "class E(Exception):\n    def __str__(self):\n        raise RuntimeError('bad str')\nraise E()",
                 "import threading\nthreading.Thread(target=lambda: print('from thread')).start()"):
        before = (sys.stdout, sys.stderr)
        assert runner.run_string(code)
        wait(runner)
        assert runner.last_status in ("ok", "error")
        assert (sys.stdout, sys.stderr) == before, code        # streams are always restored
    assert runner.run_string("print('alive')")
    wait(runner)
    assert texts(runner)[-1] == "alive"


def test_second_run_while_running_is_rejected(runner):
    runner.run_string("import time\ntime.sleep(0.3)")
    assert runner.is_running
    assert runner.run_string("print('x')") is False
    assert "already running" in texts(runner)[-1]
    wait(runner)
    assert runner.done == [("ok", "")]


def test_cancel_stops_long_simulate_within_one_second(runner):
    code = ("import robotics_api as api, sys\n"
            "sys.path.insert(0, r'%s')\n"
            "from tests.python.test_api import mass_robot\n"
            "r = mass_robot()\n"
            "print('start')\n"
            "api.simulate(r, None, 1000.0, 1e-3, record_every=1000)\n"
            "print('never')" % os.getcwd())
    runner.run_string(code)
    t0 = time.monotonic()
    while "start" not in texts(runner) and time.monotonic() - t0 < 20:
        QApplication.processEvents()
        time.sleep(0.005)
    assert "start" in texts(runner)
    time.sleep(0.3)                          # the simulation is running now
    assert runner.is_running
    t_cancel = time.monotonic()
    runner.cancel()
    wait(runner, 5)
    assert time.monotonic() - t_cancel < 1.0
    assert runner.last_status == "cancelled" and "never" not in texts(runner)


def test_cancel_stops_plan_and_workspace(runner):
    for body in ("api.workspace(robot, n_samples=10**9)",
                 "while True:\n    api.check_cancel()\n"):
        runner.run_string("import robotics_api as api\n"
                          "from tests.python.test_api import mass_robot\n"
                          "robot = mass_robot()\nprint('go')\n" + body)
        t0 = time.monotonic()
        while texts(runner)[-1:] != ["go"] and time.monotonic() - t0 < 20:
            QApplication.processEvents()
            time.sleep(0.005)
        time.sleep(0.15)
        runner.cancel()
        t1 = time.monotonic()
        wait(runner, 5)
        assert time.monotonic() - t1 < 1.0 and runner.last_status == "cancelled"
        runner.lines.clear()


def test_time_limit_setting(runner):
    runner.time_limit = 0.3
    runner.run_string("import robotics_api as api\nfrom tests.python.test_api import mass_robot\n"
                      "api.simulate(mass_robot(), None, 1000.0, 1e-3, record_every=1000)")
    t0 = time.monotonic()
    wait(runner, 10)
    assert runner.last_status == "timeout" and time.monotonic() - t0 < 3.0
    runner.time_limit = None
    runner.run_string("print('fine')")
    wait(runner)
    assert runner.last_status == "ok"


def test_ui_event_loop_stays_responsive_during_run(runner):
    ticks = []
    timer = QTimer()
    timer.timeout.connect(lambda: ticks.append(time.monotonic()))
    timer.start(10)
    runner.run_string("import time\nfor _ in range(60):\n    time.sleep(0.01)\nprint('done')")
    t0 = time.monotonic()
    n_events = 0
    while runner.is_running and time.monotonic() - t0 < 10:
        QApplication.processEvents()
        n_events += 1
        time.sleep(0.002)
    timer.stop()
    wait(runner)
    assert texts(runner) == ["done"]
    assert len(ticks) >= 10 and n_events >= 10
    assert max(b - a for a, b in zip(ticks, ticks[1:])) < 0.3     # no long freeze of the GUI thread


def test_app_handle_read_only_snapshot(runner):
    proj = ProjectData.new("snap")
    runner._provider = lambda: proj
    runner.run_string("print(app.has_project, app.project_name, app.version)\n"
                      "print(type(app.robot).__name__, app.q)\n"
                      "print(api.__api_version__ == robotics_api.__api_version__)")
    wait(runner)
    assert texts(runner)[0] == "True snap test"
    assert texts(runner)[1] == "Robot []"
    assert texts(runner)[2] == "True"
    runner.run_string("app.project_name = 'x'")
    wait(runner)
    assert runner.last_status == "error" and "read-only" in runner.last_message
    runner._provider = lambda: None
    runner.run_string("print(app.has_project, app.robot, app.project_name)")
    wait(runner)
    assert texts(runner)[-1] == "False None None"
    h = AppHandle(None)
    with pytest.raises(AttributeError):
        h.anything = 1
    with pytest.raises(AttributeError):
        h.missing


def test_run_file_sets_dunder_file_and_script_folder_on_path(runner, tmp_path):
    (tmp_path / "helper_mod_r81.py").write_text("VALUE = 41\n", encoding="utf-8")
    f = tmp_path / "s.py"
    f.write_text("import os, helper_mod_r81\nprint(os.path.basename(__file__), helper_mod_r81.VALUE + 1)\n",
                 encoding="utf-8")
    assert runner.run_file(str(f))
    wait(runner)
    assert texts(runner) == ["s.py 42"] and runner.last_status == "ok"
    assert str(tmp_path) not in sys.path
    sys.modules.pop("helper_mod_r81", None)


def test_example_scripts_run_through_runner(runner):
    ex = EX
    for name in ("fk_sweep.py", "plan_and_simulate.py", "export_data.py"):
        runner.lines.clear()
        assert runner.run_file(str(ex / name))
        wait(runner, 60)
        assert runner.last_status == "ok", (name, runner.last_message)
        assert texts(runner, False)
