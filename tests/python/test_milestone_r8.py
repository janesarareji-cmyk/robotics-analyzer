"""
tests/python/test_milestone_r8.py
R8 stage milestone: scripting API through the script runner, plugins (planner kind, isolation,
permission), report HTML/PDF, URDF export round trip, old-format project migration, autosave
and crash recovery, docs check, packaging smoke (opt-in) and a clean shutdown. Offscreen
MainWindow, real core_bridge.

Oracles are independent of the code under test: poses come from ``numpy_ref.fk_numpy``; the
migrated fixtures are compared with the reference project built in code by
``fixtures/projects/make_fixtures.py``.
"""

import os
import shutil
import subprocess
import sys
import threading
import time
import unittest.mock as mock
from pathlib import Path

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtCore import QSettings
from PySide6.QtWidgets import QApplication, QMessageBox

from app.importers import import_urdf
from app.main_window import MainWindow
from app.plugins.host import PluginCallError
from app.project.project_data import ProjectData
from app.project.recovery import autosave_path
from app.project.serializer import load, save
from app.settings import AppSettings
from app.sim.export import load_csv

from .numpy_ref import fk_numpy, spec_from_model
from .test_migration import VERSIONS, assert_matches, make_fixtures
from .test_plugins import PLAN_OK, manifest, mk
from .test_urdf_export import check_fk
from .test_cartesian_traj import planar_2r

REPO = Path(__file__).resolve().parents[2]
FX = Path(__file__).parent / "fixtures"
URDF = FX / "urdf" / "six_dof_with_gripper.urdf"
PLUGINS = REPO / "plugins_examples"
TEST_ORG = "RoboticsAnalyzer_test_r8milestone"
START = [-1.0, -1.5, 0.36, 0.0, -0.25, 0.0, 0.0]
GOAL = [1.0, -1.5, 0.36, 0.0, -0.25, 0.0, 0.0]
RESULTS: dict = {}          # numbers for the stage gate record (printed with -s)

ALL_DIALOGS = ("critical", "warning", "question", "information")


def _no_dialogs(monkeypatch, answer=QMessageBox.StandardButton.Yes):
    """Patch every QMessageBox entry point (modal dialogs hang offscreen). Returns the call log."""
    calls = []
    for name in ALL_DIALOGS:
        def fake(*a, _n=name, **k):
            calls.append(_n)
            return answer
        monkeypatch.setattr(QMessageBox, name, staticmethod(fake))
    return calls


@pytest.fixture
def win(qapp, monkeypatch, tmp_path):
    monkeypatch.setattr(AppSettings, "_ORGANIZATION", TEST_ORG)
    monkeypatch.setattr(AppSettings, "_APPLICATION", TEST_ORG)
    s = QSettings(TEST_ORG, TEST_ORG)
    s.clear()
    s.sync()
    calls = _no_dialogs(monkeypatch)
    root = tmp_path / "plugins"
    shutil.copytree(PLUGINS / "linear_planner", root / "linear_planner",
                    ignore=shutil.ignore_patterns("__pycache__"))
    mk(root, "bad_import", "raise RuntimeError('bad plugin at import')\n", man=manifest("bad_import"))
    reader = ("def register(host):\n    host.register_planner('Reader', f)\n    global H\n    H = host\n"
              "def f(model, s, g, **kw):\n    return H.read_text(r'%s')\n" % (root / "reader" / "x.txt"))
    mk(root, "reader", reader, man=manifest("reader", perms=["file_read"]))
    (root / "reader" / "x.txt").write_text("data")
    AppSettings().set_plugins_folder(str(root))
    w = MainWindow()
    w.show()
    QApplication.processEvents()
    w.dialog_calls = calls
    w.plugin_root = root
    yield w
    w.script_runner.cancel()
    w.project_controller._is_modified = False
    w.sim_panel.shutdown()
    w.planning_panel.shutdown()
    w.task_runner.wait_all(60000)
    w.close()
    s.clear()
    s.sync()


def _pump_until(cond, timeout=60.0):
    end = time.perf_counter() + timeout
    while time.perf_counter() < end:
        QApplication.processEvents()
        if cond():
            return True
        time.sleep(0.005)
    return cond()


def _run_script(win, code):
    sr = win.script_runner
    out, done = [], []
    sr.output.connect(lambda t, e: out.append((t, e)))
    sr.finished.connect(lambda s, m: done.append((s, m)))
    assert sr.run_string(code)
    assert _pump_until(lambda: bool(done) and not sr.is_running, 120.0), "script did not finish"
    return out, done[0]


def _import(win):
    win.import_urdf_file(str(URDF))
    assert win.task_runner.wait_all(60000)
    QApplication.processEvents()
    assert win.project_controller.has_project
    return win.project_controller.robot


# ---------------------------------------------------------------------------
# Item 1: scripts through the runner
# ---------------------------------------------------------------------------

def test_item1_script_plan_simulate_export_csv(win, tmp_path):
    csv = tmp_path / "run.csv"
    code = f"""
import robotics_api as api
robot = api.import_urdf(r'{URDF}')
# fitted finger cubes overlap at this pose (no ACM in the API): one simple link shape + far obstacle
robot.set_collision_shapes([{{"type": "box", "link": 3, "pose": [0, 0, 0.1], "size": [0.05, 0.05, 0.2]}}])
far = [{{"type": "box", "pose": {{"xyz": [3, 3, 3]}}, "size": [0.1, 0.1, 0.1]}}]
path = api.plan(robot, {START}, {GOAL}, obstacles=far, planner="rrt_connect", seed=3, time_limit=30.0)
traj = api.time_parametrize(path, robot)
res = api.simulate(robot, traj, dt=0.002, record_every=25)
rows = api.export_csv(res, r'{csv}', names=["q", "tcp_pos"], meta={{"robot": robot.name}})
print("rows", rows, "waypoints", len(path), "duration", traj.duration)
"""
    t0 = time.perf_counter()
    out, (status, msg) = _run_script(win, code)
    RESULTS["script_s"] = time.perf_counter() - t0
    assert status == "ok", (status, msg, out)
    assert csv.is_file()
    arrays, meta = load_csv(csv)
    q = np.asarray(arrays["q"][1], float)
    tcp = np.asarray(arrays["tcp_pos"][1], float)
    assert q.shape[1] == 7 and len(q) > 50 and tcp.shape == (len(q), 3)
    spec = spec_from_model(import_urdf(str(URDF)).model)
    err = max(float(np.abs(fk_numpy(spec, qi)[1][:3, 3] - ti).max()) for qi, ti in zip(q, tcp))
    RESULTS["csv_rows"] = len(q)
    RESULTS["csv_tcp_vs_numpy_fk_m"] = err
    assert err < 1e-9
    assert np.allclose(q[0], START, atol=1e-6)


def test_item1_failing_script_logged_app_alive(win):
    out, (status, msg) = _run_script(win, "import robotics_api as api\nraise RuntimeError('milestone boom')\n")
    assert status == "error" and "milestone boom" in msg
    assert any(e and "milestone boom" in t for t, e in out)
    assert not win.script_runner.is_running
    # the app still works: a good script runs after the failure
    out2, (status2, _) = _run_script(win, "print('alive')")
    assert status2 == "ok" and ("alive", False) in out2
    assert win.isVisible()


def test_item1_cancel_within_one_second(win):
    code = f"""
import robotics_api as api
robot = api.import_urdf(r'{URDF}')
print("start")
api.simulate(robot, None, 1000.0, 1e-3, record_every=1000)
print("never")
"""
    sr = win.script_runner
    out, done = [], []
    sr.output.connect(lambda t, e: out.append((t, e)))
    sr.finished.connect(lambda s, m: done.append((s, m)))
    assert sr.run_string(code)
    assert _pump_until(lambda: ("start", False) in out, 60.0)
    time.sleep(0.3)
    assert sr.is_running
    t_cancel = time.perf_counter()
    sr.cancel()
    assert _pump_until(lambda: bool(done) and not sr.is_running, 5.0)
    dt = time.perf_counter() - t_cancel
    RESULTS["cancel_s"] = dt
    assert dt < 1.0
    assert done[0][0] == "cancelled" and ("never", False) not in out


# ---------------------------------------------------------------------------
# Item 2: plugins
# ---------------------------------------------------------------------------

def test_item2_plugins_planner_isolation_permission(win):
    pm = win.plugin_manager
    status = {r.name: r.status for r in pm.results}
    assert status == {"linear_planner": "ok", "bad_import": "error", "reader": "ok"}
    bad = next(r for r in pm.results if r.name == "bad_import")
    assert "bad plugin at import" in str(getattr(bad, "message", "") or getattr(bad, "error", "") or bad)
    # planner combo shows the sample planner
    combo = win.planning_panel._planner
    keys = [combo.itemData(i) for i in range(combo.count())]
    assert "plugin:linear_planner:Linear" in keys
    # plan a path with it through the planning panel
    _import(win)
    pp = win.planning_panel
    combo.setCurrentIndex(combo.findData("plugin:linear_planner:Linear"))
    pp.set_vector("start", START)
    pp.set_vector("goal", GOAL)
    results = []
    pp.plan_finished.connect(results.append)
    pp.plan()
    assert _pump_until(lambda: bool(results) and not pp.is_running, 60.0)
    assert results[0].success and results[0].planner == "Linear"
    path = np.asarray(results[0].path, float)
    assert np.allclose(path[0], START) and np.allclose(path[-1], GOAL)
    # an isolated bad plugin did not stop the good one; the window is alive
    assert win.isVisible()
    # permission denied by the (monkeypatched) dialog: call fails, plugin stays enabled
    asked = []
    pm.permissions._ask = lambda *a: asked.append(1) or False
    with pytest.raises(PluginCallError) as ei:
        pm.plugin("reader").planners[0](planar_2r(), [0.0, 0.0], [0.5, -0.5],
                                        limits=([-3.2, -3.2], [3.2, 3.2]))
    assert ei.value.reason == "permission" and "denied" in str(ei.value)
    assert len(asked) == 1 and pm.permissions.state("reader", "file_read") == "denied"
    assert {r.name: r.status for r in pm.results}["reader"] == "ok"


# ---------------------------------------------------------------------------
# Item 3: reports; item 4: URDF export round trip
# ---------------------------------------------------------------------------

def test_item3_report_html_and_pdf(win, tmp_path, monkeypatch):
    _import(win)
    outs = {}
    for fmt in ("html", "pdf"):
        p = tmp_path / f"report.{fmt}"
        outs[fmt] = p
        monkeypatch.setattr("app.main_window.QFileDialog.getSaveFileName",
                            staticmethod(lambda *a, _p=p, **k: (str(_p), "")))
        win.export_report()
        assert _pump_until(lambda: p.is_file() and p.stat().st_size > 0 and win.task_runner.active_count == 0,
                           120.0), fmt
        QApplication.processEvents()
    html = outs["html"].read_text(encoding="utf-8")
    for sec in ("Robot summary", "Mass and COM", "six_dof_with_gripper"):
        assert sec in html, sec
    pdf = outs["pdf"].read_bytes()
    assert pdf.startswith(b"%PDF") and len(pdf) > 1000
    RESULTS["html_bytes"] = len(html)
    RESULTS["pdf_bytes"] = len(pdf)
    assert "QMessageBox.warning" not in win.dialog_calls and "warning" not in win.dialog_calls


def test_item4_urdf_export_import_equals_model(win, tmp_path, monkeypatch):
    model = _import(win)
    out = tmp_path / "exported.urdf"
    monkeypatch.setattr("app.main_window.QFileDialog.getSaveFileName",
                        staticmethod(lambda *a, **k: (str(out), "")))
    win.export_urdf_file()
    assert _pump_until(lambda: out.is_file() and out.stat().st_size > 0 and win.task_runner.active_count == 0,
                       60.0)
    QApplication.processEvents()
    imp = import_urdf(str(out))
    assert imp.model is not None, imp.report.summary()
    check_fk(model, imp.model, n=50)            # FK of 50 random q equal to 1e-9 (inside)
    RESULTS["urdf_fk_tol"] = 1e-9


# ---------------------------------------------------------------------------
# Items 5, 6: migration and recovery
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("version", VERSIONS[:3])
def test_item5_old_fixture_migrates_with_backup(win, tmp_path, version):
    ref = make_fixtures.reference_project()
    p = tmp_path / "old.robotproj"
    shutil.copy(FX / "projects" / f"v{version}.robotproj", p)
    original = p.read_bytes()
    pc = win.project_controller
    assert pc.open_project(None, str(p))
    bak = tmp_path / f"old.v{version}.bak.robotproj"
    assert bak.is_file() and bak.read_bytes() == original
    proj = pc.current_project
    assert proj.migrated_from == version
    # content equals the reference (the file version field keeps the stored version)
    assert_matches(proj, ref, version)
    pc._is_modified = False
    pc.close_project(None)


def test_item6_autosave_and_recovery_after_crash(win, tmp_path):
    pc = win.project_controller
    p = tmp_path / "crash.robotproj"
    proj = ProjectData.new("crash")
    save(proj, p)
    assert pc.open_project(None, str(p))
    pc.current_project.metadata.description = "unsaved work"
    pc.mark_modified()
    a = pc.autosave_now()
    assert a == autosave_path(p) and a.is_file()
    now = time.time()                           # the project file is older than the autosave
    os.utime(p, (now - 10, now - 10))
    os.utime(a, (now, now))
    # simulated crash: the process dies, nothing is saved, no cleanup runs
    assert load(p).metadata.description != "unsaved work"
    pc._autosave_timer.stop()
    pc._project = None
    pc._is_modified = False
    win.dialog_calls.clear()
    assert pc.open_project(None, str(p))        # dialog patched: answer Yes = recover
    assert win.dialog_calls == ["question"]
    assert pc.current_project.metadata.description == "unsaved work"
    assert pc.is_modified                       # still to be saved by the user
    assert pc.save_project(None)
    assert load(p).metadata.description == "unsaved work"
    assert not a.exists()
    pc._is_modified = False


# ---------------------------------------------------------------------------
# Item 7: docs check, packaging smoke
# ---------------------------------------------------------------------------

def test_item7_docs_exist_and_check_docs_passes():
    for name in ("api-reference.md", "plugin-guide.md", "reports.md", "packaging.md"):
        assert (REPO / "docs" / name).is_file(), name
    r = subprocess.run([sys.executable, str(REPO / "tools" / "check_docs.py")],
                       capture_output=True, text=True, timeout=120)
    assert r.returncode == 0, r.stdout + r.stderr


def test_item7_packaging_smoke_when_dist_exists():
    if os.environ.get("ANALYZER_TEST_DIST") != "1":
        pytest.skip("packaging smoke is opt-in: set ANALYZER_TEST_DIST=1 (needs packaging/build.ps1 output in dist/)")
    exes = sorted((REPO / "dist").rglob("RoboticsAnalyzer*.exe")) or sorted((REPO / "dist").rglob("*.exe"))
    if not exes:
        pytest.skip("no built exe under dist/ (run packaging/build.ps1)")
    r = subprocess.run([str(exes[0]), "--smoke"], capture_output=True, text=True, timeout=300)
    assert r.returncode == 0, r.stdout + r.stderr


# ---------------------------------------------------------------------------
# Item 8: nothing keeps running after close
# ---------------------------------------------------------------------------

def test_item8_nothing_runs_after_close(qapp, monkeypatch, tmp_path):
    monkeypatch.setattr(AppSettings, "_ORGANIZATION", TEST_ORG)
    monkeypatch.setattr(AppSettings, "_APPLICATION", TEST_ORG)
    s = QSettings(TEST_ORG, TEST_ORG)
    s.clear()
    s.sync()
    _no_dialogs(monkeypatch)
    threads_before = {t.ident for t in threading.enumerate()}
    w = MainWindow()
    w.show()
    QApplication.processEvents()
    _import(w)
    out, (status, _) = _run_script(w, "print('x')")
    assert status == "ok"
    pc = w.project_controller
    p = tmp_path / "t.robotproj"
    save(pc.current_project, p)
    pc.current_project.file_path = str(p)
    pc.mark_modified()
    assert pc.autosave_active
    pc._is_modified = False
    w.sim_panel.stop_trajectory()
    pc.close_project(None)
    w.close()
    QApplication.processEvents()
    assert w.task_runner.active_count == 0
    assert not w.script_runner.is_running
    assert not pc.autosave_active
    assert not w.sim_panel.timer.isActive()
    assert not w.graph_panel.timer.isActive()
    assert not w.planning_panel.is_running
    assert _pump_until(lambda: not [t for t in threading.enumerate()
                                    if t.ident not in threads_before and t.is_alive()
                                    and not t.daemon], 5.0)
    s.clear()
    s.sync()


def test_zz_print_gate_numbers():
    """Numbers for the PROGRESS.md gate record (run with -s)."""
    print("R8 GATE NUMBERS", {k: (f"{v:.4g}" if isinstance(v, float) else v) for k, v in RESULTS.items()})
