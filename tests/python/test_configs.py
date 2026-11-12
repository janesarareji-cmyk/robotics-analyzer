"""R7.6: named configurations (store, project file 1.3.0), comparison, prismatic units (gap a).

Oracles: independent numpy FK (`numpy_ref.fk_numpy`), numpy SVD of the numeric Jacobian, numpy
angle wrapping. No collision library needed.
"""
import json
import math
import zipfile

import numpy as np
import pytest

import core_bridge as cb
from app.configs.store import ConfigStore, compare_configs, wrap_pi
from app.graphs.series import graph_data
from app.project.project_data import PROJECT_FORMAT_VERSION, ProjectData
from app.project.serializer import load, save
from app.sim import export as sx

from .numpy_ref import fk_numpy, numeric_jacobian_tcp, so3_log, spec_from_model
from .robots import random_q, random_robot
from .test_cartesian_traj import planar_2r


# ------------------------------------------------------------------ store
def test_store_basics_and_validation():
    s = ConfigStore()
    s.set("a", [0.1, 0.2])
    s.set(" b ", (1, 2))
    assert s.names == ["a", "b"] and s.get("b") == [1.0, 2.0]
    for bad in ("", "  "):
        with pytest.raises(ValueError):
            s.set(bad, [0.0])
    with pytest.raises(ValueError):
        s.set("x", [math.nan])
    with pytest.raises(ValueError):
        s.set("x", [])
    s.rename("a", "c")
    assert s.names == ["c", "b"]
    with pytest.raises(ValueError):
        s.rename("c", "b")
    assert s.unique_name("c") == "c 2"
    s.remove("c")
    assert s.names == ["b"]


def test_store_dict_round_trip():
    s = ConfigStore({"home": [0.0, 0.5, -0.5], "pick": [1.0, 2.0, 3.0]})
    assert ConfigStore.from_dict(json.loads(json.dumps(s.to_dict()))) == s


def test_validate_dof_drops_and_logs(caplog):
    s = ConfigStore({"two": [0.0, 1.0], "three": [0.0, 1.0, 2.0]})
    with caplog.at_level("WARNING", logger="robotics_analyzer.configs"):
        dropped = s.validate_dof(3)
    assert dropped == ["two"] and s.names == ["three"]
    assert "dropped" in caplog.text and "two" in caplog.text
    assert s.validate_dof(3) == []


# ------------------------------------------------------------------ project file
def _project(configs):
    p = ProjectData.new("cfg")
    p.robot_model = planar_2r()
    p.configs = ConfigStore(configs)
    return p


def test_project_round_trip(tmp_path):
    path = tmp_path / "a.robotproj"
    p = _project({"start": [0.0, 0.0], "goal": [2.2, 0.3]})
    save(p, path)
    with zipfile.ZipFile(path) as zf:
        assert "configs.json" in zf.namelist()
    q = load(path)
    assert PROJECT_FORMAT_VERSION == "1.3.0"
    assert q.configs.as_dict() == p.configs.as_dict()
    assert q.configs.names == ["start", "goal"]


def test_empty_configs_not_written_and_old_file_loads(tmp_path):
    path = tmp_path / "b.robotproj"
    save(_project({}), path)
    with zipfile.ZipFile(path) as zf:
        assert "configs.json" not in zf.namelist()
    # a 1.2.0 file: no configs.json, older version string
    old = tmp_path / "old.robotproj"
    with zipfile.ZipFile(path) as zin, zipfile.ZipFile(old, "w") as zout:
        for n in zin.namelist():
            data = zin.read(n)
            if n == "project_metadata.json":
                d = json.loads(data)
                d["format_version"] = "1.2.0"
                data = json.dumps(d).encode()
            zout.writestr(n, data)
    q = load(old)
    assert isinstance(q.configs, ConfigStore) and len(q.configs) == 0


def test_invalid_configs_file_ignored(tmp_path):
    path = tmp_path / "c.robotproj"
    save(_project({"a": [0.0, 0.0]}), path)
    bad = tmp_path / "bad.robotproj"
    with zipfile.ZipFile(path) as zin, zipfile.ZipFile(bad, "w") as zout:
        for n in zin.namelist():
            zout.writestr(n, b'{"configs": [{"name": "a"}]}' if n == "configs.json" else zin.read(n))
    assert len(load(bad).configs) == 0


# ------------------------------------------------------------------ comparison
def _six(seed=7, prismatic=0.0):
    return random_robot(seed, 6, kinds="origin_axis", prismatic=prismatic)


def _manip(spec, q):
    s = np.linalg.svd(numeric_jacobian_tcp(spec, q), compute_uv=False)
    return float(np.prod(s))


@pytest.mark.parametrize("prismatic", [0.0, 0.4])
def test_comparison_vs_numpy(prismatic):
    model = _six(prismatic=prismatic)
    spec = spec_from_model(model)
    rng = np.random.default_rng(11)
    qs = [np.array(random_q(model, rng)) * 0.4 for _ in range(3)]
    rows = compare_configs(model, [(f"c{i}", q) for i, q in enumerate(qs)])
    assert [r["name"] for r in rows] == ["c0", "c1", "c2"]
    ref_T = fk_numpy(spec, qs[0])[1]
    pris = [str(j.type).lower().endswith("prismatic") for j in model.joints]
    for q, r in zip(qs, rows):
        T = fk_numpy(spec, q)[1]
        assert np.allclose(r["tcp"], T, atol=1e-12)
        assert np.allclose(r["dp"], T[:3, 3] - ref_T[:3, 3], atol=1e-12)
        assert r["distance"] == pytest.approx(np.linalg.norm(T[:3, 3] - ref_T[:3, 3]), abs=1e-12)
        rv = so3_log(T[:3, :3] @ ref_T[:3, :3].T)
        assert np.allclose(r["drot"], rv, atol=1e-9)
        assert r["angle"] == pytest.approx(np.linalg.norm(rv), abs=1e-9)
        want = []
        for i in range(6):
            d = q[i] - qs[0][i]
            want.append(d if pris[i] else (d + math.pi) % (2 * math.pi) - math.pi)
        assert np.allclose(r["dq"], want, atol=1e-12)
        assert r["manipulability"] == pytest.approx(_manip(spec, q), rel=1e-5)
        assert r["dmanip"] == pytest.approx(_manip(spec, q) - _manip(spec, qs[0]), abs=1e-6)
    assert rows[0]["distance"] == 0.0 and rows[0]["dmanip"] == 0.0 and not any(rows[0]["dq"])
    if prismatic:
        assert any(pris)


def test_wrapped_joint_difference():
    model = planar_2r()
    rows = compare_configs(model, [("a", [3.0, 0.0]), ("b", [-3.0, 0.5])])
    # -3 - 3 = -6 wraps to 2*pi - 6 ; 0.5 stays
    assert rows[1]["dq"][0] == pytest.approx(2 * math.pi - 6.0, abs=1e-12)
    assert rows[1]["dq"][1] == pytest.approx(0.5, abs=1e-12)
    assert wrap_pi(math.pi) == pytest.approx(-math.pi)
    assert abs(rows[1]["dq"][0]) <= math.pi


def test_comparison_rejects_wrong_length():
    with pytest.raises(ValueError):
        compare_configs(planar_2r(), [("a", [0.0, 0.0]), ("b", [0.0])])
    assert compare_configs(planar_2r(), []) == []


# ------------------------------------------------------------------ gap (a): prismatic units
def _arrays(n=3, t=4):
    ts = np.linspace(0, 1, t)
    d = np.arange(t * n, dtype=float).reshape(t, n)
    arrays = {k: (ts, d + i) for i, k in enumerate(("q", "qd", "qdd", "tau", "power"))}
    arrays["power_sum"] = (ts, d[:, :1])
    return arrays


def test_series_units_with_joint_types():
    arrays = _arrays()
    types = ["revolute", "prismatic", "revolute"]
    for kind, want in (("joint_pos", ("rad", "m")), ("joint_vel", ("rad/s", "m/s")),
                       ("joint_acc", ("rad/s^2", "m/s^2")), ("joint_torque", ("N.m", "N"))):
        ser = graph_data(arrays, kind, joint_types=types)
        assert [s.unit for s in ser] == [want[0], want[1], want[0]]
    sel = graph_data(arrays, "joint_pos", joints=[1, 2], joint_types=types)
    assert [s.unit for s in sel] == ["m", "rad"]
    pw = graph_data(arrays, "power", joint_types=types)
    assert all(s.unit == "W" for s in pw)


def test_series_units_old_signature_unchanged():
    ser = graph_data(_arrays(), "joint_pos")
    assert all(s.unit == "rad|m" for s in ser)
    # short / partial type lists: missing entries keep the combined unit
    ser = graph_data(_arrays(), "joint_pos", joint_types=["prismatic"])
    assert [s.unit for s in ser] == ["m", "rad|m", "rad|m"]


def test_export_meta_and_series_units(tmp_path):
    types = ["revolute", "prismatic", "revolute"]
    meta = sx.build_meta(joint_types=types)
    assert meta["joint_types"] == types
    # raw run export: prismatic column labelled in m
    path = tmp_path / "run.csv"
    sx.export_csv({"q": _arrays()["q"]}, path, meta=meta)
    arrays, m = sx.load_csv(path)
    header = m["columns"]
    assert header[1].endswith("[rad]") and header[2].endswith("[m]") and header[3].endswith("[rad]")
    # graph-series export carries the per-joint unit
    ser = graph_data(_arrays(), "joint_vel", joint_types=types)
    arr, units = sx.arrays_from_series(ser)
    assert [units[s.name] for s in ser] == ["rad/s", "m/s", "rad/s"]
    path2 = tmp_path / "g.csv"
    sx.export_csv(arr, path2, meta={"units": units})
    header2 = sx.load_csv(path2)[1]["columns"]
    assert [h.split("[")[1] for h in header2[1:]] == ["rad/s]", "m/s]", "rad/s]"]
