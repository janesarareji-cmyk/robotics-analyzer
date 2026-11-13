"""U2.7 robot comparison: numbers (ur3 / ur5 / ur10), payload logic, CSV, dock."""

import csv
import math
import os
import time

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtCore import QCoreApplication
from PySide6.QtWidgets import QToolBar

from app.analysis import compare as cmp
from app.main_window import MainWindow
from app.sample_robots import list_sample_robots
from app.settings import AppSettings

SAMPLES = {p.parent.name: (t, p) for t, p in list_sample_robots()}
NOMINAL = {"ur3": 0.50, "ur5": 0.85, "ur10": 1.30}      # UR data sheet, wrist-3 centre


@pytest.fixture(scope="module")
def studies():
    out = {}
    for n in ("ur3", "ur5", "ur10"):
        name, model = cmp.load_robot(SAMPLES[n][1])
        out[n] = cmp.compare_robot(model, name)
    return out


def test_reach_order_and_known_reaches(studies):
    r = {n: s["reach"] for n, s in studies.items()}
    print("\nreach (TCP, m):", {n: round(v, 3) for n, v in r.items()}, "nominal", NOMINAL)
    assert r["ur3"] < r["ur5"] < r["ur10"]
    # The TCP is the flange / tool point, so it lies beyond the data-sheet reach (wrist-3 centre):
    # 0.9 .. 1.5 x nominal, and the same ordering of volume and mass.
    for n, nom in NOMINAL.items():
        assert 0.9 * nom <= r[n] <= 1.5 * nom, (n, r[n], nom)
    v = [studies[n]["volume"] for n in ("ur3", "ur5", "ur10")]
    m = [studies[n]["mass"] for n in ("ur3", "ur5", "ur10")]
    assert v[0] < v[1] < v[2] and m[0] < m[1] < m[2]
    for s in studies.values():
        assert s["n_dof"] == 6 and s["volume"] > 0 and s["mass"] > 0


def test_reach_is_independent_check(studies):
    """Reach and stretched pose agree with a plain numpy FK of the stored q."""
    import core_bridge as cb
    name, model = cmp.load_robot(SAMPLES["ur5"][1])
    s = studies["ur5"]
    cm = cb.compile(model)
    origin = np.asarray(cb.fk_compiled(cm, [0.0] * 6)["frames"][0])[:3, 3]
    p = np.asarray(cb.fk_compiled(cm, s["q_reach"])["tcp"])[:3, 3] - origin
    assert np.linalg.norm(p) == pytest.approx(s["reach"], abs=1e-9)
    ps = np.asarray(cb.fk_compiled(cm, s["q_stretch"])["tcp"])[:3, 3] - origin
    assert np.linalg.norm(ps[:2]) == pytest.approx(s["reach_xy"], abs=1e-9)
    assert np.linalg.norm(ps[:2]) > 0.85 * s["reach"]       # really stretched out sideways


def test_gravity_torque_and_payload(studies):
    import core_bridge as cb
    for n, s in studies.items():
        assert s["payload"] is not None and s["payload"] > 0, n
        assert max(abs(t) for t in s["tau_gravity"]) > 1.0, n          # horizontal pose: torque
        assert len(s["tau_gravity"]) == len(s["effort"]) == 6
    # payload definition: adding exactly that mass at the TCP puts some joint on its limit
    name, model = cmp.load_robot(SAMPLES["ur5"][1])
    s = studies["ur5"]
    cm = cb.compile(model)
    dm = cb.build_dyn_model(model, cm)
    q = s["q_stretch"]
    jv = np.asarray(cb.jacobian_tcp(cm, q))[:3]
    tau0 = np.asarray(cb.inverse_dynamics(dm, q, [0.0] * 6, [0.0] * 6, [0.0, 0.0, -9.81]))
    tau_p = tau0 + s["payload"] * (-(jv.T @ np.array([0.0, 0.0, -9.81])))
    ratio = np.abs(tau_p) / np.asarray(s["effort"])
    assert ratio.max() == pytest.approx(1.0, abs=1e-6)


def test_payload_capacity_logic():
    # tau = 10, limit 30, a = 2 -> m = 10 ; second joint not limiting
    assert cmp.payload_capacity([10.0, 1.0], [30.0, 100.0], [2.0, 1.0]) == pytest.approx(10.0)
    # negative sensitivity: tau = -10, limit 30, a = -2 -> (-30 + 10)/-2 = 10
    assert cmp.payload_capacity([-10.0], [30.0], [-2.0]) == pytest.approx(10.0)
    # already over its limit: 0
    assert cmp.payload_capacity([40.0], [30.0], [1.0]) == 0.0
    # missing effort limit on a joint that carries the load: n/a
    assert cmp.payload_capacity([1.0], [float("nan")], [1.0]) is None
    # a joint the load does not move needs no limit
    assert cmp.payload_capacity([1.0, 5.0], [float("nan"), 50.0], [0.0, 1.0]) == pytest.approx(45.0)


def test_missing_effort_limit_gives_na():
    name, model = cmp.load_robot(SAMPLES["ur5"][1])
    for j in model.joints:
        j.limits.effort = 0.0
    r = cmp.compare_robot(model, "noeff", n_samples=2000)
    assert r["payload"] is None
    assert cmp.table_rows([r])[0][7] == "n/a"


def test_csv_round_trip(studies, tmp_path):
    results = [studies[n] for n in ("ur3", "ur5", "ur10")]
    path = tmp_path / "cmp.csv"
    assert cmp.export_csv(results, path) == 3
    with open(path, newline="", encoding="utf-8") as f:
        rows = list(csv.reader(f))
    assert rows[0] == cmp.HEADERS and len(rows) == 4
    for res, row in zip(results, rows[1:]):
        assert row[0] == res["name"]
        assert int(row[1]) == res["n_dof"]
        assert float(row[2]) == pytest.approx(res["mass"], rel=1e-3)
        assert float(row[3]) == pytest.approx(res["reach"], rel=1e-3)
        assert float(row[4]) == pytest.approx(res["volume"], rel=1e-3)
        assert float(row[7]) == pytest.approx(res["payload"], rel=1e-2)
        taus = [float(x) for x in row[5].split(";")]
        assert len(taus) == 6
        assert "deg" in row[8]


@pytest.fixture
def win(qapp, monkeypatch):
    monkeypatch.setattr(AppSettings, "_ORGANIZATION", "RoboticsAnalyzer_test_u2c")
    monkeypatch.setattr(AppSettings, "_APPLICATION", "RoboticsAnalyzer_test_u2c")
    w = MainWindow()
    w.settings._settings.clear()
    w.show()
    yield w
    w.project_controller._is_modified = False
    w.close()


def _wait(panel, limit=60.0):
    t0 = time.time()
    while panel.is_running and time.time() - t0 < limit:
        QCoreApplication.processEvents()
        time.sleep(0.01)
    QCoreApplication.processEvents()


def test_dock_wiring_side_bar_and_view_menu(win):
    assert win.compare_dock is not None
    bar = win.findChild(QToolBar, "tool_bar_side")
    assert win._action_view_compare in bar.actions()
    assert win._action_view_compare.iconText() == "Compare"
    assert (win._action_view_compare, win.compare_dock) in win._view_action_docks()
    assert win.compare_dock.isHidden()
    win._action_view_compare.trigger()
    QCoreApplication.processEvents()
    assert not win.compare_dock.isHidden() and win._action_view_compare.isChecked()
    win._action_view_compare.trigger()
    QCoreApplication.processEvents()
    assert win.compare_dock.isHidden() and not win._action_view_compare.isChecked()


def test_dock_runs_comparison_and_exports(win, tmp_path):
    p = win.compare_panel
    assert p.entries() == []
    assert not p.run_button.isEnabled()
    p.set_checked(["ur5"])
    assert not p.run_button.isEnabled()                  # needs two
    p.set_checked(["ur3", "ur5", "ur10"])
    assert p.run_button.isEnabled() and len(p.entries()) == 3
    assert not p.export_button.isEnabled()
    t0 = time.time()
    p.run_button.click()
    assert p.is_running and p.cancel_button.isEnabled()
    _wait(p)
    print(f"\ncompare dock run: {time.time() - t0:.2f} s")
    assert not p.is_running and len(p.results) == 3
    assert p.table.rowCount() == 3 and p.table.columnCount() == len(cmp.HEADERS)
    assert p.progress_bar.value() == 100
    names = [p.table.item(r, 0).text() for r in range(3)]
    assert names == [r["name"] for r in p.results]
    assert p.export_button.isEnabled()
    out = tmp_path / "dock.csv"
    assert p.export_csv(str(out)) == 3
    assert out.read_text(encoding="utf-8").count("\n") >= 4


def test_dock_cancel(win):
    p = win.compare_panel
    p.set_checked(["ur3", "ur5", "ur10"])
    p.samples_spin.setValue(1_000_000)
    p.run_button.click()
    t0 = time.time()
    p.cancel_button.click()
    _wait(p, 20.0)
    dt = time.time() - t0
    print(f"\ncompare cancel latency: {dt:.2f} s")
    assert not p.is_running
    assert p.results == [] and "cancel" in p.status_text.lower()
    assert dt < 5.0
