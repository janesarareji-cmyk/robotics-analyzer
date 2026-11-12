"""R8.2: plugin manager panel and main window integration (offscreen Qt, QMessageBox patched)."""
import json
import os
import shutil
import unittest.mock as mock
from pathlib import Path

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtCore import QSettings
from PySide6.QtWidgets import QApplication

import core_bridge as cb

from app.main_window import MainWindow
from app.panels.plugin_manager_panel import PluginManagerPanel
from app.plugins.loader import PluginManager
from app.settings import AppSettings

from .test_cartesian_traj import planar_2r
from .test_plugins import PLAN_OK, DictSettings, manifest, mk

EX = Path(__file__).parent.parent.parent / "plugins_examples"
TEST_ORG = "RoboticsAnalyzer_test_r82ui"


def row_of(panel, name):
    t = panel.table
    for r in range(t.rowCount()):
        if t.item(r, 0).text() == name:
            return r
    raise AssertionError(name)


def cell(panel, name, col):
    return panel.table.item(row_of(panel, name), col).text()


def test_manager_panel_lists_statuses_and_toggles(tmp_path, qapp):
    root = tmp_path / "pl"
    mk(root, "good", PLAN_OK, man=manifest("good"))
    mk(root, "boom", "raise RuntimeError('import boom')\n", man=manifest("boom"))
    mk(root, "oldapi", PLAN_OK, man=manifest("oldapi", api_version="2.0"))
    s = DictSettings()
    m = PluginManager(s, folder=str(root), ask=lambda *a: True, load_timeout=2.0)
    p = PluginManagerPanel(m)
    p.show()
    m.scan()
    assert str(root) in p._folder.text()
    assert [cell(p, n, 3) for n in ("good", "boom", "oldapi")] == ["ok", "error", "rejected"]
    assert "import boom" in cell(p, "boom", 5) and "not compatible" in cell(p, "oldapi", 5)
    assert cell(p, "good", 1) == "planner"
    # rejected rows cannot be toggled
    p.table.selectRow(row_of(p, "oldapi"))
    assert not p._btn_toggle.isEnabled()
    # disable / enable via the button, persisted in settings
    p.table.selectRow(row_of(p, "good"))
    assert p._btn_toggle.isEnabled() and p._btn_toggle.text() == "Disable"
    p._btn_toggle.click()
    assert cell(p, "good", 3) == "disabled" and s.get("plugins/enabled/good") == "0"
    assert p.selected_name() == "good" and p._btn_toggle.text() == "Enable"
    p._btn_toggle.click()
    assert cell(p, "good", 3) == "ok" and s.get("plugins/enabled/good") == "1"
    # reload re-reads the folder
    mk(root, "late", PLAN_OK, man=manifest("late"))
    p._btn_reload.click()
    assert cell(p, "late", 3) == "ok"


def test_manager_panel_permissions_and_revoke(tmp_path, qapp):
    root = tmp_path / "pl"
    mk(root, "p", "def register(host):\n    host.register_planner('P', lambda *a, **k: None)\n",
       perms=["file_read", "network"])
    m = PluginManager(DictSettings(), folder=str(root), ask=lambda *a: True)
    panel = PluginManagerPanel(m)
    m.scan()
    assert cell(panel, "p", 4) == "file_read:ask, network:ask"
    f = root / "p" / "a.txt"
    f.write_text("x")
    m.plugin("p").host.read_text(f)
    panel.refresh()
    assert cell(panel, "p", 4) == "file_read:granted, network:ask"
    panel.table.selectRow(0)
    assert panel._btn_revoke.isEnabled()
    panel._btn_revoke.click()
    assert cell(panel, "p", 4) == "file_read:ask, network:ask"


def test_manager_panel_session_failure_shown(tmp_path, qapp):
    root = tmp_path / "pl"
    mk(root, "p", PLAN_OK.replace("success=True", "success='maybe'"), man=manifest("p"))
    m = PluginManager(DictSettings(), folder=str(root))
    panel = PluginManagerPanel(m)
    m.scan()
    with pytest.raises(Exception):
        m.plugin("p").planners[0](planar_2r(), [0, 0], [1, 1], limits=([-3, -3], [3, 3]))
    assert cell(panel, "p", 3) == "disabled" and "session" in cell(panel, "p", 5)


# ----------------------------------------------------------------------------- main window
@pytest.fixture
def win(qapp, monkeypatch, tmp_path):
    monkeypatch.setattr(AppSettings, "_ORGANIZATION", TEST_ORG)
    monkeypatch.setattr(AppSettings, "_APPLICATION", TEST_ORG)
    s = QSettings(TEST_ORG, TEST_ORG)
    s.clear()
    s.sync()
    root = tmp_path / "plugins"
    for d in EX.iterdir():
        if d.is_dir():
            shutil.copytree(d, root / d.name, ignore=shutil.ignore_patterns("__pycache__"))
    mk(root, "broken", "raise RuntimeError('broken at import')\n", man=manifest("broken"))
    AppSettings().set_plugins_folder(str(root))
    w = MainWindow()
    w.show()
    yield w
    w.project_controller._is_modified = False
    w.task_runner.wait_all(20000)
    QApplication.processEvents()
    w.close()
    s.clear()
    s.sync()


def test_window_loads_plugins_and_survives_a_broken_one(win):
    pm = win.plugin_manager
    assert {r.name: r.status for r in pm.results} == {
        "broken": "error", "joint_info_panel": "ok", "linear_planner": "ok", "urm_json_importer": "ok"}
    assert not win._plugin_dock.isVisible()
    win._action_view_plugins.trigger()
    assert win._plugin_dock.isVisible()
    p = win._plugin_panel
    assert cell(p, "broken", 3) == "error"


def test_planner_combo_import_menu_and_analysis_dock(win):
    combo = win.planning_panel._planner
    labels = [combo.itemText(i) for i in range(combo.count())]
    assert labels == ["RRT-Connect", "RRT*", "PRM", "Linear (plugin)"]
    assert combo.itemData(3) == "plugin:linear_planner:Linear"
    acts = [a.text() for a in win._menu_import.actions()]
    assert "URM JSON (plugin)..." in acts
    assert [a.text() for a in win._menu_plugin_views.actions()] == ["Joint info (joint_info_panel)"]
    (dock, act), = win._plugin_docks.values()
    assert dock.windowTitle() == "Joint info (plugin)" and not dock.isVisible()
    # disable the plugin in the manager: entries vanish everywhere, enable: back
    win.plugin_manager.set_enabled("linear_planner", False)
    win.plugin_manager.set_enabled("joint_info_panel", False)
    win.plugin_manager.set_enabled("urm_json_importer", False)
    QApplication.processEvents()
    assert combo.count() == 3 and not win._plugin_docks
    assert "URM JSON (plugin)..." not in [a.text() for a in win._menu_import.actions()]
    assert win._menu_plugin_views.actions() == []
    win.plugin_manager.set_enabled("linear_planner", True)
    assert combo.count() == 4


def test_plugin_planner_runs_through_planning_panel(win):
    panel = win._planning_panel
    with mock.patch("app.project.project_controller.QInputDialog.getText", return_value=("P", True)):
        win.project_controller.new_project(win)
    model = planar_2r()
    win.project_controller.set_robot(model)
    QApplication.processEvents()
    combo = panel._planner
    combo.setCurrentIndex(combo.findData("plugin:linear_planner:Linear"))
    assert combo.currentText() == "Linear (plugin)"
    assert panel.vector("start") is not None
    panel.set_vector("start", [0.0, 0.0])
    panel.set_vector("goal", [0.4, -0.3])
    results = []
    panel.plan_finished.connect(results.append)
    panel.plan()
    win.task_runner.wait_all(10000)
    for _ in range(200):
        QApplication.processEvents()
        if results:
            break
    assert results and results[0].planner == "Linear"


def test_plugin_import_end_to_end_and_failure_message(win, tmp_path):
    pm = win.plugin_manager
    pm.permissions._ask = lambda *a: True
    f = tmp_path / "robot.json"
    f.write_text(planar_2r(0.9, 0.3).to_json())
    imp = pm.importers()[0]
    with mock.patch("app.main_window.QMessageBox.warning") as warn:
        win.import_plugin_file(imp, str(f))
        win.task_runner.wait_all(10000)
        QApplication.processEvents()
    assert warn.call_count == 0
    assert win.project_controller.has_project and win.project_controller.robot.dof() == 2
    # a failing plugin call shows a message, never raises, plugin is disabled
    bad = tmp_path / "bad.json"
    bad.write_text("not urm json")
    with mock.patch("app.main_window.QMessageBox.warning") as warn:
        win.import_plugin_file(imp, str(bad))
        win.task_runner.wait_all(10000)
        QApplication.processEvents()
    assert warn.call_count == 1
    assert pm.result("urm_json_importer").status == "disabled"
    assert pm.importers() == []
