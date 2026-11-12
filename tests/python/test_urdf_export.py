"""R8.3: URDF export, round trip through the R3.3 importer, mesh export, API wrappers.

Oracle: ``numpy_ref`` (independent FK written in the tests folder), plain ``xml.etree``.
"""
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest

import core_bridge as cb
import robotics_api as api
from app.importers import import_urdf
from app.importers.visuals import LinkVisual
from app.reports.mesh_export import export_meshes
from app.reports.urdf_export import export_urdf, urdf_xml

from .numpy_ref import fk_numpy, spec_from_model
from .robots import fk_test_robots, pose, random_inertias, random_q, random_robot

FX = Path(__file__).parent / "fixtures"
URDF = FX / "urdf"


def dof_index(model):
    """joint name -> q index (non-fixed, non-mimic joints)."""
    return {model.joints[i].name: k for i, k in enumerate(model.q_index_map()) if k >= 0}


def roundtrip(model, tmp_path, visuals=(), collisions=(), name="out.urdf"):
    p = tmp_path / name
    res = export_urdf(model, str(p), visuals, collisions)
    imp = import_urdf(str(p))
    assert imp.model is not None, imp.report.summary()
    return imp, res


def check_fk(m1, m2, n=50, seed=3):
    rng = np.random.default_rng(seed)
    i1, i2 = dof_index(m1), dof_index(m2)
    assert set(i1) == set(i2)
    s1, s2 = spec_from_model(m1), spec_from_model(m2)
    for _ in range(n):
        q1 = random_q(m1, rng)
        q2 = [0.0] * len(i2)
        for nme, k in i1.items():
            q2[i2[nme]] = q1[k]
        T1 = fk_numpy(s1, q1)[1]
        T2 = fk_numpy(s2, q2)[1]
        assert np.abs(T1 - T2).max() < 1e-9
        # the app's own FK agrees as well (compiled model of the imported robot)
        assert np.abs(np.array(cb.fk(m2, q2)["tcp"]) - T1).max() < 1e-9


def check_structure(m1, m2):
    n1 = {j.name: j for j in m1.joints}
    n2 = {j.name: j for j in m2.joints}
    assert set(n1) <= set(n2) and len(n2) - len(n1) in (0, 1)
    for nme, j in n1.items():
        k = n2[nme]
        assert str(j.type) == str(k.type)
        if str(j.type).endswith("Fixed"):
            continue
        assert k.limits.lower == pytest.approx(j.limits.lower, abs=1e-12)
        assert k.limits.upper == pytest.approx(j.limits.upper, abs=1e-12)
        assert k.limits.vel == pytest.approx(j.limits.vel, abs=1e-12)
        assert k.limits.effort == pytest.approx(j.limits.effort, abs=1e-12)
        assert (j.mimic is None) == (k.mimic is None)
        if j.mimic is not None:
            assert (k.mimic.joint, k.mimic.multiplier, k.mimic.offset) == \
                   (j.mimic.joint, j.mimic.multiplier, j.mimic.offset)


def parent_names(m):
    nm = [lk.name for lk in m.links]
    return {nm[int(c)]: nm[int(p)] for p, c in cb.effective_links(m)}


# ----------------------------------------------------------------- FK round trips
@pytest.mark.parametrize("key", ["six_dof_fixture", "dh_6r", "origin_axis_12r", "tree_mimic_fixed"])
def test_roundtrip_fk_structure(tmp_path, key):
    m = fk_test_robots()[key]
    random_inertias(m, np.random.default_rng(5))
    imp, _ = roundtrip(m, tmp_path)
    check_fk(m, imp.model)
    check_structure(m, imp.model)
    # tree structure by names (the importer may renumber links)
    pm, pi = parent_names(m), parent_names(imp.model)
    assert all(pi[k] == v for k, v in pm.items())
    assert imp.model.links[imp.model.tcp_link].name in (
        m.links[m.tcp_link].name, m.links[m.tcp_link].name + "_tool")


def test_roundtrip_modified_dh_prismatic(tmp_path):
    m = random_robot(21, 5, kinds="dh", prismatic=0.4, modified=True)
    imp, _ = roundtrip(m, tmp_path)
    check_fk(m, imp.model)


def test_roundtrip_standard_dh_prismatic_mimic(tmp_path):
    m = random_robot(22, 5, kinds=("dh", "origin_axis"), prismatic=0.4, mimic=True)
    imp, _ = roundtrip(m, tmp_path)
    check_fk(m, imp.model)
    check_structure(m, imp.model)


def test_roundtrip_gripper_fixture(tmp_path):
    src = import_urdf(str(URDF / "six_dof_with_gripper.urdf"))
    imp, _ = roundtrip(src.model, tmp_path)
    check_fk(src.model, imp.model)
    check_structure(src.model, imp.model)
    assert [lk.name for lk in src.model.links] == [lk.name for lk in imp.model.links]
    assert src.model.tcp_link == imp.model.tcp_link
    assert [j.name for j in src.model.joints] == [j.name for j in imp.model.joints]


def test_gimbal_lock_origin(tmp_path):
    for k, pitch in enumerate((np.pi / 2, -np.pi / 2)):
        m = random_robot(31, 3)
        m.joints[1].origin = pose([0.1, -0.2, 0.3], [0.4, pitch, -0.7])
        imp, _ = roundtrip(m, tmp_path, name=f"g{k}.urdf")
        check_fk(m, imp.model, n=10)


# ------------------------------------------------------------------- inertia
def test_inertia_exact_origin_axis(tmp_path):
    m = random_robot(41, 6)
    random_inertias(m, np.random.default_rng(1))
    imp, _ = roundtrip(m, tmp_path)
    by = {lk.name: lk for lk in imp.model.links}
    for lk in m.links:
        o = by[lk.name].inertia
        assert o.mass == pytest.approx(lk.inertia.mass, abs=1e-12)
        assert np.abs(np.array(o.com) - np.array(lk.inertia.com)).max() < 1e-12
        assert np.abs(np.array(o.inertia_tensor) - np.array(lk.inertia.inertia_tensor)).max() < 1e-12


@pytest.mark.parametrize("kw", [dict(kinds="dh"), dict(kinds="dh", modified=True),
                                dict(kinds=("dh", "origin_axis"), prismatic=0.3, branch=True, mimic=True)])
def test_inertia_dh_mass_matrix(tmp_path, kw):
    """DH frames move to the joint frame on export: compare frame independent mass matrices."""
    m = random_robot(42, 5, **kw)
    random_inertias(m, np.random.default_rng(2))
    imp, _ = roundtrip(m, tmp_path)
    r1, r2 = api.Robot(m), api.Robot(imp.model)
    i1, i2 = dof_index(m), dof_index(imp.model)
    perm = [i2[nme] for nme, _ in sorted(i1.items(), key=lambda kv: kv[1])]
    rng = np.random.default_rng(3)
    for _ in range(5):
        q1 = np.array(random_q(m, rng))
        q2 = np.zeros(len(perm))
        q2[perm] = q1
        M1, M2 = r1.mass_matrix(q1), r2.mass_matrix(q2)
        assert np.abs(M2[np.ix_(perm, perm)] - M1).max() < 1e-9 * max(1.0, np.abs(M1).max())
    assert sum(lk.inertia.mass for lk in imp.model.links) == pytest.approx(
        sum(lk.inertia.mass for lk in m.links), abs=1e-12)


# ---------------------------------------------------------------- XML (independent)
def test_xml_well_formed_counts_and_names(tmp_path):
    m = random_robot(51, 5, branch=True)
    m.tool = pose([0, 0, 0], [0, 0, 0])
    m.base = pose([0, 0, 0], [0, 0, 0])
    p = tmp_path / "x.urdf"
    export_urdf(m, str(p))
    root = ET.parse(p).getroot()
    assert root.tag == "robot"
    joints = root.findall("joint")
    assert len(joints) == len(m.joints) == 6
    assert len(root.findall("link")) == len(m.links)
    got = {j.get("name"): (j.find("parent").get("link"), j.find("child").get("link"),
                           j.get("type")) for j in joints}
    for j, (pi, ci) in zip(m.joints, cb.effective_links(m)):
        assert got[j.name] == (m.links[pi].name, m.links[ci].name, "revolute")
    assert root.find("robotics_analyzer").get("tcp_link") == m.links[m.tcp_link].name
    for j in joints:
        assert j.find("limit") is not None and len(j.find("axis").get("xyz").split()) == 3


def test_tool_offset_adds_fixed_joint(tmp_path):
    m = random_robot(52, 3)
    p = tmp_path / "t.urdf"
    res = export_urdf(m, str(p))
    root = ET.parse(p).getroot()
    assert len(root.findall("joint")) == 4
    assert root.findall("joint")[-1].get("type") == "fixed"
    assert root.find("robotics_analyzer").get("tcp_link") == m.links[m.tcp_link].name + "_tool"
    assert res.xml.startswith("<?xml")


def test_invalid_model_raises():
    with pytest.raises(ValueError):
        urdf_xml(cb.RobotModel())
    m = random_robot(53, 2)
    m.joints[0].parent_link = 5
    with pytest.raises(ValueError):
        urdf_xml(m)


# ------------------------------------------------------------- visuals and meshes
def test_meshes_copied_relative_and_reimported(tmp_path):
    src = import_urdf(str(URDF / "six_dof_with_gripper.urdf"))
    assert src.model is not None and src.visuals
    out = tmp_path / "exp"
    res = export_urdf(src.model, str(out / "r.urdf"), src.visuals, src.collisions)
    assert res.meshes and all(not r.startswith("package://") for r in res.meshes)
    assert (out / "meshes" / "cube.stl").is_file()
    root = ET.parse(out / "r.urdf").getroot()
    fns = [m.get("filename") for m in root.iter("mesh")]
    assert fns and all(f == "meshes/cube.stl" for f in fns)
    assert all(m.get("scale") == "0.5 0.5 0.5" for m in root.iter("mesh"))
    imp = import_urdf(str(out / "r.urdf"))
    assert imp.model is not None
    assert len(imp.visuals) == len(src.visuals)
    for v in imp.visuals:
        assert v.kind == "mesh" and not v.fallback and Path(v.mesh_path).is_file()
        assert v.params["scale"] == [0.5, 0.5, 0.5]


def test_package_uri_replaced(tmp_path):
    m = random_robot(61, 2)
    stl = tmp_path / "src" / "arm.stl"
    stl.parent.mkdir()
    shutil.copyfile(FX / "meshes" / "cube.stl", stl)
    v = LinkVisual(1, m.links[1].name, "mesh", {"filename": "package://pkg/arm.stl", "scale": [1, 1, 1]},
                   mesh_path=str(stl))
    miss = LinkVisual(2, m.links[2].name, "mesh", {"filename": "package://pkg/none.stl", "scale": [1, 1, 1]})
    res = export_urdf(m, str(tmp_path / "o" / "a.urdf"), [v, miss])
    assert "package://pkg/arm.stl" not in res.xml
    assert 'filename="meshes/arm.stl"' in res.xml
    assert len(res.warnings) == 1 and "none.stl" in res.warnings[0]


def test_primitives_and_colour_roundtrip(tmp_path):
    src = import_urdf(str(URDF / "primitives.urdf"))
    assert src.model is not None and src.visuals
    imp, _ = roundtrip(src.model, tmp_path, src.visuals, src.collisions)
    a = {(v.link_name, v.kind): v for v in src.visuals}
    b = {(v.link_name, v.kind): v for v in imp.visuals}
    assert set(a) == set(b)
    for k, v in a.items():
        assert np.allclose(b[k].rgba, v.rgba, atol=1e-12)
        assert np.allclose(b[k].origin_xyz, v.origin_xyz, atol=1e-12)
        if k[1] != "mesh":
            for key, val in v.params.items():
                assert np.allclose(b[k].params[key], val, atol=1e-12)
    assert len(imp.collisions) == len(src.collisions)


def test_mesh_export_loads_back(tmp_path):
    import pyvista as pv
    src = import_urdf(str(URDF / "six_dof_with_gripper.urdf"))
    paths = export_meshes(src.visuals, tmp_path / "m", "stl")
    assert len(paths) == len(src.visuals)
    for p in paths:
        assert pv.read(p).n_points > 0
    obj = export_meshes(src.visuals[:2], tmp_path / "m2", "obj", model=src.model)
    assert all(pv.read(p).n_cells > 0 for p in obj)
    with pytest.raises(ValueError):
        export_meshes(src.visuals, tmp_path / "m3", "step")


def test_mesh_export_primitives_world_pose(tmp_path):
    import pyvista as pv
    m = random_robot(71, 2)
    v = LinkVisual(1, "l1", "box", {"size": [0.2, 0.2, 0.2]}, origin_xyz=[0.1, 0, 0])
    p = export_meshes([v], tmp_path / "w", "stl", model=m, q=[0.0, 0.0])[0]
    F = np.array(cb.fk(m, [0.0, 0.0])["frames"][1])
    c = np.array(pv.read(p).center)
    assert np.allclose(c, (F @ [0.1, 0, 0, 1])[:3], atol=1e-5)


# ---------------------------------------------------------------------------- API
def test_api_export_urdf_roundtrip(tmp_path):
    r = api.import_urdf(str(URDF / "six_dof_with_gripper.urdf"))
    info = api.export_urdf(r, str(tmp_path / "e" / "r.urdf"))
    assert info["meshes"] and (tmp_path / "e" / "r.urdf").is_file()
    r2 = api.import_urdf(str(tmp_path / "e" / "r.urdf"))
    q = np.linspace(-0.4, 0.5, r.n_dof)
    assert np.allclose(r.fk(q), r2.fk(q), atol=1e-9)
    with pytest.raises(api.ExportError):
        api.export_urdf(api.Robot(cb.RobotModel()), str(tmp_path / "bad.urdf"))


def test_api_version_bumped():
    assert api.__api_version__ == "1.1"
    assert hasattr(api, "export_urdf") and hasattr(api, "generate_report")
