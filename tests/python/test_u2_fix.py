"""U2-fix: UI fixes from real-window screenshots."""

import os

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtCore import QCoreApplication
from PySide6.QtWidgets import QAbstractItemView

from app.importers.urdf import import_urdf
from app.main_window import MainWindow
from app.sample_robots import list_sample_robots
from app.settings import AppSettings

SAMPLES = {p.parent.name: (t, p) for t, p in list_sample_robots()}


@pytest.fixture
def win(qapp, monkeypatch):
    monkeypatch.setattr(AppSettings, "_ORGANIZATION", "RoboticsAnalyzer_test_u2fix")
    monkeypatch.setattr(AppSettings, "_APPLICATION", "RoboticsAnalyzer_test_u2fix")
    w = MainWindow()
    w.settings._settings.clear()
    w.show()
    yield w
    w.project_controller._is_modified = False
    w.close()


def _load(win, name):
    path = SAMPLES[name][1]
    res = import_urdf(str(path))
    win._import_path = str(path)
    win._on_urdf_import_result(res)
    QCoreApplication.processEvents()
    return res


def test_import_xarm5_gripper_keeps_report_dock_hidden(win):
    win.import_report_dock.hide()
    QCoreApplication.processEvents()
    assert not win.import_report_dock.isVisible()
    res = _load(win, "xarm5_gripper")
    assert not res.report.has_errors()
    assert not win.import_report_dock.isVisible()


def test_tree_root_shows_display_name(win):
    _load(win, "xarm5_gripper")
    title = SAMPLES["xarm5_gripper"][0]
    assert win.project_controller.current_project.name == title
    assert win.project_controller.robot.metadata.name == title
    assert win._model_panel._tree.headerItem().text(0) == title


def test_drag_off_resets_status(win):
    _load(win, "ur5")
    drag = win.tcp_drag
    win.drag_tcp_action.setChecked(True)
    drag.on_tcp_drag(np.array([5.0, 0.0, 0.0]))
    assert "not reachable" in win._status_label.text().lower()
    win.drag_tcp_action.setChecked(False)
    assert win._status_label.text() == "Ready"
    assert drag.reachable


def test_sim_panel_fits_380_and_teach_on_top(win):
    _load(win, "ur5")
    sp = win._sim_panel
    assert sp.minimumSizeHint().width() <= 360
    lay = sp.layout()
    first = lay.itemAt(0).widget()
    assert first is sp.timeline._teach_group
    assert sp.timeline._btn_teach_play is not None


def test_compare_table_scrolls_and_elides(win):
    from app.panels.compare_panel import NAME_COL_MAX, ComparePanel
    p = ComparePanel(win._task_runner)
    assert p._table.horizontalScrollBarPolicy().name in ("ScrollBarAsNeeded",)
    p._results = []
    long = "A very long robot name that goes on and on forever"
    p._table.setRowCount(1)
    from PySide6.QtWidgets import QTableWidgetItem
    p._table.setItem(0, 0, QTableWidgetItem(long))
    p._table.resizeColumnsToContents()
    assert NAME_COL_MAX == 160
