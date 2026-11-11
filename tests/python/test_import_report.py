"""
tests/python/test_import_report.py
R3.6: Import report dock, filters, row click, to_text() format, save, docs table.
"""

import os
import re
import unittest.mock as mock
from pathlib import Path

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtCore import Qt
from PySide6.QtTest import QTest
from PySide6.QtWidgets import QApplication

import core_bridge as cb
from app.importers import import_report as ir
from app.importers import import_urdf
from app.main_window import MainWindow
from app.panels.import_report_panel import find_element
from app.panels.model_panel import ROLE_INDEX, ROLE_KIND

ROOT = Path(__file__).resolve().parents[2]
FX = Path(__file__).parent / "fixtures"
URDF = FX / "urdf"
GOLDEN = FX / "import_report_golden.txt"
DOC = ROOT / "docs" / "import-report.md"


@pytest.fixture
def win(qapp):
    w = MainWindow()
    w.show()
    yield w
    w.project_controller._is_modified = False
    w.close()


def _import(win, name):
    """Import a URDF fixture through the main window. Returns the warning-box mock."""
    with mock.patch("app.main_window.QMessageBox.warning") as w:
        win.import_urdf_file(str(URDF / name))
        win.task_runner.wait_all(5000)
        QApplication.processEvents()
    return w


def _row_severities(panel):
    return [panel.table.item(r, 0).text() for r in range(panel.row_count)]


# ---------------------------------------------------------------------------
# Dock rows per fixture
# ---------------------------------------------------------------------------

@pytest.mark.parametrize(
    "name, errors, warnings, info",
    [
        ("planar_2r.urdf", 0, 0, 1),
        ("bad_xml.urdf", 1, 0, 0),
        ("missing_limit.urdf", 0, 2, 1),
        ("continuous_joint.urdf", 0, 1, 1),
        ("gripper_mimic.urdf", 0, 0, 1),
        ("primitives.urdf", 0, 0, 2),
        ("serial_6dof.urdf", 0, 0, 1),
        ("cycle.urdf", 1, 0, 0),
        ("two_roots.urdf", 1, 0, 0),
        ("xacro.urdf", 1, 0, 0),
        ("zero_axis.urdf", 1, 0, 0),
    ],
)
def test_dock_rows_per_fixture(win, name, errors, warnings, info):
    _import(win, name)
    panel = win.import_report_panel
    total = errors + warnings + info
    assert panel.row_count == total
    sev = _row_severities(panel)
    assert sev.count("ERROR") == errors
    assert sev.count("WARNING") == warnings
    assert sev.count("INFO") == info
    assert panel.report.summary() == f"{errors} errors, {warnings} warnings, {info} info"


def test_no_popup_for_any_report(win):
    w_bad = _import(win, "bad_xml.urdf")
    w_warn = _import(win, "missing_limit.urdf")
    assert not w_bad.called
    assert not w_warn.called


def test_errors_raise_dock_info_only_does_not(win):
    dock = win.import_report_dock
    dock.hide()
    _import(win, "planar_2r.urdf")
    assert not dock.isVisible()
    _import(win, "bad_xml.urdf")
    assert dock.isVisible()


def test_log_line_per_import(win):
    _import(win, "missing_limit.urdf")
    lines = [ln for ln in win.log_panel.text().splitlines() if "missing_limit.urdf" in ln]
    assert len(lines) == 1
    assert "0 errors, 2 warnings, 1 info" in lines[0]


# ---------------------------------------------------------------------------
# Filters
# ---------------------------------------------------------------------------

def test_severity_filters(win):
    _import(win, "missing_limit.urdf")
    panel = win.import_report_panel
    assert panel.row_count == 3
    panel.filter_box(ir.SEV_WARNING).setChecked(False)
    assert panel.row_count == 1
    assert _row_severities(panel) == ["INFO"]
    panel.filter_box(ir.SEV_INFO).setChecked(False)
    assert panel.row_count == 0
    panel.filter_box(ir.SEV_WARNING).setChecked(True)
    assert panel.row_count == 2
    panel.filter_box(ir.SEV_INFO).setChecked(True)
    assert panel.row_count == 3


def test_filters_do_not_change_saved_report(tmp_path, win):
    _import(win, "missing_limit.urdf")
    panel = win.import_report_panel
    panel.filter_box(ir.SEV_WARNING).setChecked(False)
    out = tmp_path / "all.txt"
    with mock.patch("app.panels.import_report_panel.QFileDialog.getSaveFileName",
                    return_value=(str(out), "")):
        panel.save_report_dialog()
    assert out.read_text(encoding="utf-8") == panel.report.to_text() + "\n"
    assert out.read_text(encoding="utf-8").count("WARNING") == 2


# ---------------------------------------------------------------------------
# Row click -> tree selection
# ---------------------------------------------------------------------------

def _click_row(panel, row):
    rect = panel.table.visualRect(panel.table.model().index(row, 0))
    QTest.mouseClick(panel.table.viewport(), Qt.MouseButton.LeftButton, pos=rect.center())
    QApplication.processEvents()


def test_row_click_selects_joint(win):
    _import(win, "missing_limit.urdf")
    panel = win.import_report_panel
    _click_row(panel, 1)  # warning on joint j2
    item = win.model_panel.tree.currentItem()
    assert item is not None
    assert item.data(0, ROLE_KIND) == "joint"
    assert item.text(0).startswith("j2 ")


def test_row_click_selects_link(win):
    _import(win, "missing_limit.urdf")
    panel = win.import_report_panel
    _click_row(panel, 2)  # TCP info names link l2
    item = win.model_panel.tree.currentItem()
    assert item.data(0, ROLE_KIND) == "link"
    assert item.text(0) == "l2"
    link_names = [lk.name for lk in win.project_controller.robot.links]
    assert item.data(0, ROLE_INDEX) == link_names.index("l2")


def test_row_click_without_element_does_nothing(win):
    _import(win, "bad_xml.urdf")  # error row has no element
    before = win.model_panel.tree.currentItem()
    _click_row(win.import_report_panel, 0)
    assert win.model_panel.tree.currentItem() is before


def test_find_element_joints_first_then_links():
    m = cb.RobotModel()
    for name in ("base", "x"):
        lk = cb.Link()
        lk.name = name
        m.links.append(lk)
    j = cb.Joint()
    j.name = "x"
    j.parent_link = 0
    j.child_link = 1
    m.joints.append(j)
    assert find_element(m, "x") == ("joint", 0)
    assert find_element(m, "base") == ("link", 0)
    assert find_element(m, "nope") is None
    assert find_element(m, None) is None
    assert find_element(None, "x") is None


# ---------------------------------------------------------------------------
# to_text()
# ---------------------------------------------------------------------------

def test_to_text_golden():
    rep = ir.ImportReport()
    rep.add(ir.SEV_ERROR, ir.URDF_CYCLE, "loop found", "j1")
    rep.add(ir.SEV_WARNING, ir.URDF_MISSING_LIMIT, "no limit", "j2")
    rep.add(ir.SEV_INFO, ir.URDF_TCP_CHOSEN, "tcp chosen: l2")
    assert rep.to_text() + "\n" == GOLDEN.read_text(encoding="utf-8")


def test_to_text_empty_report_is_summary_only():
    assert ir.ImportReport().to_text() == "0 errors, 0 warnings, 0 info"


# ---------------------------------------------------------------------------
# Save, and keep-model rules
# ---------------------------------------------------------------------------

def test_save_report_writes_full_text(tmp_path, win):
    _import(win, "missing_limit.urdf")
    out = tmp_path / "report.txt"
    with mock.patch("app.panels.import_report_panel.QFileDialog.getSaveFileName",
                    return_value=(str(out), "")) as dlg:
        assert win.import_report_panel.save_report_dialog() == str(out)
    assert dlg.called
    text = out.read_text(encoding="utf-8")
    assert text.splitlines()[-1] == "0 errors, 2 warnings, 1 info"
    assert "[WARNING] URDF_MISSING_LIMIT: " in text


def test_save_report_cancel_writes_nothing(tmp_path, win):
    _import(win, "missing_limit.urdf")
    with mock.patch("app.panels.import_report_panel.QFileDialog.getSaveFileName",
                    return_value=("", "")):
        assert win.import_report_panel.save_report_dialog() is None
    assert list(tmp_path.iterdir()) == []


def test_warnings_keep_model_loaded(win):
    _import(win, "missing_limit.urdf")
    assert win.project_controller.has_project
    assert win.project_controller.robot is not None
    assert len(win.project_controller.robot.joints) == 2


def test_errors_keep_old_project(win):
    with mock.patch("app.project.project_controller.QInputDialog.getText",
                    return_value=("Keep", True)):
        win.project_controller.new_project(win)
    proj = win.project_controller.current_project
    _import(win, "bad_xml.urdf")
    assert win.project_controller.current_project is proj
    assert win.project_controller.current_project.name == "Keep"
    assert win.import_report_panel.report.has_errors()


# ---------------------------------------------------------------------------
# docs/import-report.md table == code constants in app/importers/
# ---------------------------------------------------------------------------

def _code_constants():
    names = set()
    for py in (ROOT / "app" / "importers").glob("*.py"):
        text = py.read_text(encoding="utf-8")
        for m in re.finditer(r'^([A-Z][A-Z_]+) = "([A-Z][A-Z_]+)"', text, re.M):
            if m.group(1) == m.group(2) and m.group(1).startswith("URDF_"):
                names.add(m.group(1))
    return names


def _doc_codes():
    text = DOC.read_text(encoding="utf-8")
    return re.findall(r"^\| `(URDF_[A-Z_]+)` \| (?:error|warning|info) \|", text, re.M)


def test_docs_code_table_matches_constants():
    codes = _doc_codes()
    assert len(codes) == len(set(codes)), "duplicate code row in docs"
    assert set(codes) == _code_constants()
