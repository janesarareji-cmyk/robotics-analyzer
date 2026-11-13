"""R8.1: script console dock and File -> Run script... (offscreen Qt)."""
import os
import time
import unittest.mock as mock
from pathlib import Path

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtCore import QSettings
from PySide6.QtWidgets import QApplication

from app.main_window import MainWindow
from app.settings import AppSettings

EX = Path(__file__).parent.parent.parent / "examples"
TEST_ORG = "RoboticsAnalyzer_test_r81ui"


@pytest.fixture
def win(qapp, monkeypatch):
    monkeypatch.setattr(AppSettings, "_ORGANIZATION", TEST_ORG)
    monkeypatch.setattr(AppSettings, "_APPLICATION", TEST_ORG)
    s = QSettings(TEST_ORG, TEST_ORG)
    s.clear()
    s.sync()
    w = MainWindow()
    w.show()
    yield w
    w.project_controller._is_modified = False
    w.script_runner.cancel()
    w.task_runner.wait_all(20000)
    QApplication.processEvents()
    w.close()
    s.clear()
    s.sync()


def wait(win, timeout=30.0):
    t0 = time.monotonic()
    while win.script_runner.is_running and time.monotonic() - t0 < timeout:
        QApplication.processEvents()
        time.sleep(0.003)
    QApplication.processEvents()
    assert not win.script_runner.is_running


def test_dock_hidden_at_start_and_view_toggle(win):
    assert not win.script_dock.isVisible()
    assert not win._action_view_script.isChecked()
    win._action_view_script.trigger()
    assert win.script_dock.isVisible()
    win._action_view_script.trigger()
    assert not win.script_dock.isVisible()
    assert win.script_dock.windowTitle() == "Script console"
    assert win.script_panel.cancel_button.isEnabled() is False


def test_run_editor_text_output_in_panel_and_log(win):
    p = win.script_panel
    p.editor.setPlainText("print('from the editor')\nprint(api.__api_version__)\n"
                          "import sys\nsys.stderr.write('warn\\n')")
    p.run_button.click()
    assert p.status_text() == "Running..."
    assert p.run_button.isEnabled() is False and p.cancel_button.isEnabled() is True
    wait(win)
    out = p.output_text()
    assert "from the editor" in out and "1.2" in out and "[err] warn" in out
    log = win.log_panel.toPlainText()
    assert "[script] from the editor" in log and "[script stderr] warn" in log
    assert p.status_text() == "Finished"
    assert p.run_button.isEnabled() is True and p.cancel_button.isEnabled() is False


def test_exception_shown_in_log_and_app_survives(win):
    p = win.script_panel
    p.editor.setPlainText("def boom():\n    return 1 / 0\nboom()")
    p.run_button.click()
    wait(win)
    log = win.log_panel.toPlainText()
    assert "ZeroDivisionError" in log and "boom" in log and "Traceback" in log
    assert "Error" in p.status_text()
    p.editor.setPlainText("print('still alive')")
    p.run_button.click()
    wait(win)
    assert "still alive" in win.log_panel.toPlainText()


def test_cancel_button_stops_long_script(win):
    p = win.script_panel
    p.editor.setPlainText(
        "import robotics_api as api\n"
        f"robot = api.import_urdf(r'{EX / 'planar_2r.urdf'}')\n"
        "print('started')\n"
        "api.simulate(robot, None, 1000.0, 1e-3, record_every=1000)\n")
    p.run_button.click()
    t0 = time.monotonic()
    while "started" not in p.output_text() and time.monotonic() - t0 < 20:
        QApplication.processEvents()
        time.sleep(0.005)
    time.sleep(0.2)
    t1 = time.monotonic()
    p.cancel_button.click()
    wait(win, 5)
    assert time.monotonic() - t1 < 1.0
    assert p.status_text() == "Cancelled" and "[cancelled]" in p.output_text()


def test_file_menu_run_script_uses_file_dialog(win, tmp_path):
    f = tmp_path / "hello.py"
    f.write_text("print('hello from file', app.has_project)\n", encoding="utf-8")
    assert win._action_run_script.text().replace("&", "") == "Run script..."
    with mock.patch("app.main_window.QFileDialog.getOpenFileName", return_value=(str(f), "")):
        win._action_run_script.trigger()
    wait(win)
    assert win.script_dock.isVisible()
    assert "hello from file False" in win.log_panel.toPlainText()
    # cancelled dialog: nothing runs
    before = win.log_panel.toPlainText()
    with mock.patch("app.main_window.QFileDialog.getOpenFileName", return_value=("", "")):
        win._action_run_script.trigger()
    assert win.log_panel.toPlainText() == before and not win.script_runner.is_running


def test_script_sees_open_project_through_app_handle(win):
    with mock.patch("app.main_window.QMessageBox.warning"):
        win.import_urdf_file(str(EX / "planar_2r.urdf"))
        assert win.task_runner.wait_all(20000)
        QApplication.processEvents()
    win.project_controller._is_modified = False
    p = win.script_panel
    p.editor.setPlainText("print(app.has_project, app.robot.n_dof, len(app.q))\n"
                          "print(app.robot.fk(app.q)[0, 3] > 1.0)")
    p.run_button.click()
    wait(win)
    out = p.output_text()
    assert "True 2 2" in out and "True" in out.splitlines()[-1]


def test_gui_event_loop_runs_while_script_runs(win):
    p = win.script_panel
    p.editor.setPlainText("import time\nfor i in range(40):\n    time.sleep(0.01)\nprint('slept')")
    p.run_button.click()
    n = 0
    while win.script_runner.is_running and n < 10000:
        QApplication.processEvents()
        n += 1
        time.sleep(0.002)
    wait(win)
    assert n >= 10 and "slept" in p.output_text()


def test_close_with_running_script_cancels_it(win):
    p = win.script_panel
    p.editor.setPlainText("import robotics_api as api\nwhile True:\n    api.check_cancel()\n    "
                          "import time; time.sleep(0.001)")
    p.run_button.click()
    time.sleep(0.1)
    assert win.script_runner.is_running
    win.close()                              # closeEvent cancels the script and joins the pool
    wait(win, 5)
    assert win.script_runner.last_status == "cancelled"
