"""R8.3: report data, HTML, PDF, API wrapper and File -> Export wiring (offscreen Qt)."""
import os
import re
import time
import unittest.mock as mock
from pathlib import Path

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import robotics_api as api
from app.planning.planner import PlanResult
from app.reports.html_report import render_html, write_html
from app.reports.pdf_report import write_pdf
from app.reports.report import ReportData, build_report_data

from .robots import random_inertias, random_robot

URDF = Path(__file__).parent / "fixtures" / "urdf"


def dh_robot(seed=7):
    m = random_robot(seed, 3, kinds="dh")
    random_inertias(m, np.random.default_rng(seed))
    m.metadata.name = "dh_demo"
    return m


def sim_arrays(model, t_end=0.05):
    r = api.Robot(model, "dh_demo")
    res = api.simulate(r, None, t_end, 1e-3)
    return r, res, {k: (np.asarray(res["t"]), np.asarray(v).reshape(len(res["t"]), -1))
                    for k, v in res.items() if k != "t"}


# ------------------------------------------------------------------ report data + HTML
def test_html_has_key_numbers_and_sections():
    m = dh_robot()
    d = build_report_data(m, "dh_demo", app_version="9.9.9")
    page = render_html(d)
    for sec in ("Robot summary", "DH table", "Mass and COM", "dh_demo", "Robotics Analyzer v9.9.9"):
        assert sec in page
    for j in m.joints:
        assert j.name in page
        assert f"{j.limits.lower:.6g}" in page and f"{j.limits.upper:.6g}" in page
        assert f"{j.dh.a:.6g}" in page and f"{j.dh.alpha:.6g}" in page
        assert f"{j.dh.d:.6g}" in page and f"{j.dh.theta:.6g}" in page
    total = sum(lk.inertia.mass for lk in m.links)
    assert f"{total:.6g} kg" in page
    assert d.n_dof == 3 and d.mass["total"] == pytest.approx(total)
    assert "Origin / axis table" not in page          # DH robot: no origin table


def test_origin_axis_table_for_origin_axis_robot():
    m = random_robot(8, 3)
    page = render_html(build_report_data(m, "oa"))
    assert "Origin / axis table" in page and "DH table" not in page
    assert f"{m.joints[0].origin.xyz[0]:.6g}" in page


def test_missing_sections_render_with_note(tmp_path):
    m = random_robot(9, 2)                              # no masses, no workspace/plan/graphs
    d = build_report_data(m, "bare")
    assert d.mass is None and d.workspace is None and d.plan is None and d.graphs == []
    page = render_html(d)
    assert "Omitted sections" in page
    for word in ("mass and COM", "workspace", "graphs", "plan results", "collision"):
        assert word in page
    assert "<h2>Workspace</h2>" not in page and "<h2>Plan results</h2>" not in page
    p = write_html(d, tmp_path / "bare.html")
    assert Path(p).read_text(encoding="utf-8").startswith("<html>")


def test_workspace_plan_collision_and_graph_sections():
    m = dh_robot()
    robot, res, arrays = sim_arrays(m)
    ws = api.workspace(robot, n_samples=3000, voxel=0.15)
    plan = PlanResult(path=np.array([[0.0, 0, 0], [0.5, 0.2, 0.1]]), success=True, time=0.0123,
                      iterations=42, planner="rrt_connect", length=0.55)
    from app.planning.checks import Violation, ViolationReport
    vio = ViolationReport([Violation("velocity", 1, 0.5, 2.5, 1.0)], [], 100)
    from app.planning.collision import CollisionConfig
    cfg = CollisionConfig()
    cfg.set_check(0, 2, True)
    d = build_report_data(m, "dh_demo", workspace=ws, arrays=arrays, plan=plan, violations=vio,
                          collision_config=cfg, n_collision_shapes=4)
    assert d.missing == []
    page = render_html(d)
    assert f"{ws['volume']:.6g}" in page and "Bounds min" in page
    assert page.count("data:image/png;base64,") == 4 == len(d.graphs)
    png = d.graphs[0][1]
    assert png[:8] == b"\x89PNG\r\n\x1a\n"
    assert "rrt_connect" in page and "0.55" in page and "velocity" in page and "2.5" in page
    assert "ACM edits: 1" in page and "collision shapes: 4" in page


def test_report_never_raises_on_odd_inputs():
    m = dh_robot()
    d = build_report_data(m, "x", workspace={"bogus": 1}, arrays={}, plan=None)
    assert d.workspace is None and d.graphs == []
    empty = {k: (np.zeros(0), np.zeros((0, 3))) for k in ("q", "qd")}
    assert build_report_data(m, "x", arrays=empty).graphs == []
    render_html(ReportData())                            # default dataclass renders


# -------------------------------------------------------------------------- PDF
def test_pdf_created_with_pages(qapp, tmp_path):
    m = dh_robot()
    _r, _res, arrays = sim_arrays(m)
    d = build_report_data(m, "dh_demo", arrays=arrays)
    p = tmp_path / "r.pdf"
    pages = write_pdf(d, str(p))
    data = p.read_bytes()
    assert data.startswith(b"%PDF") and len(data) > 5000
    assert pages >= 1 and len(re.findall(rb"/Type\s*/Page\b", data)) == pages


def test_pdf_missing_sections_and_many_pages(qapp, tmp_path):
    m = random_robot(10, 40)                             # long joint table -> several pages
    d = build_report_data(m, "long")
    pages = write_pdf(d, str(tmp_path / "long.pdf"))
    assert pages >= 2
    assert (tmp_path / "long.pdf").stat().st_size > 0


# --------------------------------------------------------------------------- API
def test_api_generate_report(qapp, tmp_path):
    r = api.import_urdf(str(URDF / "six_dof_with_gripper.urdf"))
    info = api.generate_report(r, str(tmp_path / "a.html"))
    assert info["format"] == "html" and info["pages"] is None and info["missing"]
    page = (tmp_path / "a.html").read_text(encoding="utf-8")
    for j in r.joint_names:
        assert j in page
    info = api.generate_report(r, str(tmp_path / "a.pdf"), plan=np.zeros((3, r.n_dof)) + [0, 0.1, 0.2][:1])
    assert info["format"] == "pdf" and info["pages"] >= 1
    assert not any(m.startswith("plan") for m in info["missing"])
    with pytest.raises(api.InvalidArgument):
        api.generate_report(r, str(tmp_path / "a.txt"))


# ------------------------------------------------------------------ File -> Export
@pytest.fixture
def win(qapp, monkeypatch):
    from PySide6.QtCore import QSettings
    from PySide6.QtWidgets import QApplication
    from app.main_window import MainWindow
    from app.settings import AppSettings
    org = "RoboticsAnalyzer_test_r83"
    monkeypatch.setattr(AppSettings, "_ORGANIZATION", org)
    monkeypatch.setattr(AppSettings, "_APPLICATION", org)
    st = QSettings(org, org)
    st.clear()
    st.sync()
    w = MainWindow()
    w.show()
    yield w
    w.project_controller._is_modified = False
    w.task_runner.wait_all(20000)
    QApplication.processEvents()
    w.close()
    st.clear()


def wait_for(cond, win, timeout=20.0):
    from PySide6.QtWidgets import QApplication
    t0 = time.monotonic()
    while not cond() and time.monotonic() - t0 < timeout:
        QApplication.processEvents()
        time.sleep(0.01)
    QApplication.processEvents()
    assert cond()


def test_menu_entries(win):
    texts = [a.text() for a in win._menu_export.actions() if a.text()]
    assert any("Report" in t for t in texts) and any("URDF" in t for t in texts) \
        and any("Meshes" in t for t in texts)


def test_export_actions_without_robot_show_info_only(win):
    with mock.patch("app.main_window.QMessageBox.information") as info, \
            mock.patch("app.main_window.QFileDialog.getSaveFileName") as dlg:
        win.export_report()
        win.export_urdf_file()
        win.export_meshes_files()
    assert info.call_count == 3 and dlg.call_count == 0


def test_export_report_urdf_meshes_from_menu(win, tmp_path):
    win.import_urdf_file(str(URDF / "six_dof_with_gripper.urdf"))
    wait_for(lambda: win._project_controller.robot is not None and
             len(win._project_controller.robot.joints) > 0, win)
    html, pdf, urdf = tmp_path / "rep.html", tmp_path / "rep.pdf", tmp_path / "u" / "robot.urdf"
    with mock.patch("app.main_window.QFileDialog.getSaveFileName", return_value=(str(html), "")):
        win.export_report()
        wait_for(html.exists, win)
    assert "Robot summary" in html.read_text(encoding="utf-8")
    with mock.patch("app.main_window.QFileDialog.getSaveFileName", return_value=(str(pdf), "PDF (*.pdf)")):
        win.export_report()
        wait_for(lambda: pdf.exists() and pdf.stat().st_size > 0, win)
    with mock.patch("app.main_window.QFileDialog.getSaveFileName", return_value=(str(urdf), "")):
        win.export_urdf_file()
        wait_for(lambda: urdf.exists() and (urdf.parent / "meshes" / "cube.stl").exists(), win)
    folder = tmp_path / "meshes_out"
    with mock.patch("app.main_window.QFileDialog.getExistingDirectory", return_value=str(folder)):
        win.export_meshes_files()
        wait_for(lambda: folder.exists() and len(list(folder.glob("*.stl"))) > 0, win)
    wait_for(lambda: "mesh file(s) written" in win.log_panel.toPlainText(), win)
    log = win.log_panel.toPlainText()
    assert "Report written" in log and "URDF written" in log


def test_export_cancelled_dialog_writes_nothing(win, tmp_path):
    win.import_urdf_file(str(URDF / "planar_2r.urdf"))
    wait_for(lambda: win._project_controller.robot is not None and
             len(win._project_controller.robot.joints) > 0, win)
    with mock.patch("app.main_window.QFileDialog.getSaveFileName", return_value=("", "")):
        win.export_report()
        win.export_urdf_file()
    assert list(tmp_path.glob("*.pdf")) == [] and list(tmp_path.glob("*.urdf")) == []
