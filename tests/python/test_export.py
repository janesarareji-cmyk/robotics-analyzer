"""R6.7 data export: CSV/NPZ round trip, meta, names filter, empty error, units, UI actions."""
import json
import os
import time

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from app.graphs.series import graph_data
from app.project.project_data import SimulationSettings
from app.sim import Recorder, SimEngine
from app.sim.export import (arrays_from_series, build_meta, export_csv, export_npz, load_csv,
                            load_npz)

from .robots import random_inertias, random_robot

N = 3


@pytest.fixture(scope="module")
def run():
    rng = np.random.default_rng(11)
    model = random_robot(5, N)
    random_inertias(model, rng)
    e = SimEngine(model, SimulationSettings(physics_timestep=0.001))
    rec = Recorder(e, model)
    e.reset(rng.uniform(-1, 1, N), rng.uniform(-1, 1, N))
    e.set_tau([0.3, -0.2, 0.1])
    e.start()
    for _ in range(40):
        e.step(0.001)
    return rec


def test_csv_roundtrip_and_units(run, tmp_path):
    arrays = run.export_arrays()
    p = tmp_path / "a.csv"
    meta = build_meta(run, robot="bot", joint_names=["a", "b", "c"])
    assert export_csv(arrays, p, meta=meta) == len(arrays["q"][0])
    got, m = load_csv(p)
    assert list(got) == list(arrays)
    for k, (t, d) in arrays.items():
        np.testing.assert_allclose(got[k][0], t, rtol=1e-12, atol=0)
        np.testing.assert_allclose(got[k][1], d, rtol=1e-12, atol=0)
    hdr = m["columns"]
    assert hdr[0] == "t [s]" and "q1 [rad]" in hdr and "qd2 [rad/s]" in hdr
    assert "tcp_posx [m]" in hdr and "ke [J]" in hdr and "tau3 [N.m]" in hdr
    text = open(p, encoding="utf-8").read().splitlines()
    assert text[0].startswith("# robot: bot")


def test_meta(run, tmp_path):
    p = tmp_path / "m.csv"
    export_csv(run.export_arrays(), p, names=["q"], meta=build_meta(run, robot="bot"))
    _, m = load_csv(p)
    assert m["robot"] == "bot" and m["integrator"] == "rk4"
    assert float(m["dt"]) == 0.001 and m["gravity"] == [0.0, 0.0, -9.81]
    assert m["app version"] and m["date"]
    q = tmp_path / "m.npz"
    export_npz(run.export_arrays(), q, build_meta(run, robot="bot"))
    _, m2 = load_npz(q)
    assert m2["robot"] == "bot" and m2["dt"] == 0.001 and m2["gravity"] == [0.0, 0.0, -9.81]
    assert m2["integrator"] == "rk4" and "date" in m2


def test_npz_exact(run, tmp_path):
    arrays = run.export_arrays()
    p = tmp_path / "a.npz"
    export_npz(arrays, p, {"robot": "x"})
    with np.load(p) as z:
        assert "q_t" in z.files and "q" in z.files and "meta" in z.files
        json.loads(str(z["meta"]))
    got, m = load_npz(p)
    assert list(got) == list(arrays) and m["robot"] == "x"
    for k, (t, d) in arrays.items():
        assert np.array_equal(got[k][0], t) and np.array_equal(got[k][1], d)


def test_names_filter(run, tmp_path):
    arrays = run.export_arrays()
    p = tmp_path / "f.csv"
    export_csv(arrays, p, names=["q", "ke"])
    got, _ = load_csv(p)
    assert list(got) == ["q", "ke"]
    with pytest.raises(ValueError, match="unknown"):
        export_csv(arrays, p, names=["nope"])
    q = tmp_path / "f.npz"
    export_npz(arrays, q, {}, names=["tau"])
    assert list(load_npz(q)[0]) == ["tau"]


def test_empty_errors(tmp_path):
    model = random_robot(5, N)
    random_inertias(model, np.random.default_rng(1))
    e = SimEngine(model, SimulationSettings())
    rec = Recorder(e, e.model)
    for fn in (lambda: export_csv(rec.export_arrays(), tmp_path / "e.csv"),
               lambda: export_npz(rec.export_arrays(), tmp_path / "e.npz", {}),
               lambda: export_csv({}, tmp_path / "e2.csv")):
        with pytest.raises(ValueError, match="nothing to export"):
            fn()


def test_joined_different_rates(tmp_path):
    a = {"x": (np.array([0.0, 1.0, 2.0]), np.array([[1.0], [2.0], [3.0]])),
         "y": (np.array([0.5, 2.0]), np.array([[10.0, 11.0], [20.0, 21.0]]))}
    p = tmp_path / "j.csv"
    assert export_csv(a, p) == 4
    lines = open(p, encoding="utf-8").read().splitlines()
    assert "0,1,," in [l for l in lines if l.startswith("0,")]
    got, _ = load_csv(p)
    for k in a:
        assert np.array_equal(got[k][0], a[k][0]) and np.array_equal(got[k][1], a[k][1])


def test_series_export(run, tmp_path):
    ser = graph_data(run.export_arrays(), "joint_pos", names=["a", "b", "c"])
    arrays, units = arrays_from_series(ser)
    p = tmp_path / "s.csv"
    export_csv(arrays, p, meta={"units": units})
    got, m = load_csv(p)
    assert list(got) == ["a", "b", "c"] and "a [rad]" in m["columns"]
    np.testing.assert_allclose(got["b"][1][:, 0], run.export_arrays()["q"][1][:, 1], rtol=1e-12)


def test_ui_export_actions(qapp, run, tmp_path, monkeypatch):
    from PySide6.QtWidgets import QApplication, QFileDialog, QMessageBox

    from app.main_window import MainWindow

    w = MainWindow()
    for a in ("_action_export_run_csv", "_action_export_run_npz", "_action_export_graph"):
        assert hasattr(w, a)
    # fake sim panel recorder
    monkeypatch.setattr(type(w._sim_panel), "recorder", property(lambda s: run))
    monkeypatch.setattr(QMessageBox, "information", lambda *a, **k: None)
    warns = []
    monkeypatch.setattr(QMessageBox, "warning", lambda *a, **k: warns.append(a))
    w._graph_panel.set_kind("energy")
    paths = {}
    for scope, fmt, name in (("run", "csv", "r.csv"), ("run", "npz", "r.npz"), ("graph", None, "g.csv")):
        out = str(tmp_path / name)
        monkeypatch.setattr(QFileDialog, "getSaveFileName", lambda *a, _o=out, **k: (_o, ""))
        w.export_data(scope, fmt)
        w.task_runner.wait_all(10000)
        end = time.time() + 2
        while time.time() < end:
            QApplication.processEvents()
        paths[name] = out
        assert not warns, warns
        assert os.path.getsize(out) > 0
    got, _ = load_npz(paths["r.npz"])
    assert len(got["q"][0]) == len(run)
    got, _ = load_csv(paths["g.csv"])
    assert sorted(got) == ["ke", "pe", "total"]
    w.project_controller._is_modified = False
    w.close()
