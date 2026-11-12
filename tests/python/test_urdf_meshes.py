"""
tests/python/test_urdf_meshes.py
R3.5: visual geometry, mesh resolution, viewport actors, archive assets.
"""

import os
import shutil
import unittest.mock as mock
import zipfile
from pathlib import Path

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication

import core_bridge as cb
from app.importers import import_report as ir
from app.importers import visuals as vis
from app.importers.urdf import import_urdf
from app.importers.visuals import LinkVisual, load_visual_mesh, resolve_mesh
from app.main_window import MainWindow
from app.project.project_data import PROJECT_FORMAT_VERSION, ProjectData
from app.project.serializer import load, save
from app.settings import AppSettings

FX = Path(__file__).parent / "fixtures"
MESHES = FX / "meshes"
PRIMS = FX / "urdf" / "primitives.urdf"

MESH_URDF = """<?xml version="1.0"?>
<robot name="m">
  <link name="base">
    <inertial><mass value="1"/><inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/></inertial>
  </link>
  <link name="l1">
    <inertial><mass value="1"/><inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/></inertial>
    <visual>
      <origin xyz="0 0 0.1"/>
      <geometry><mesh filename="{fn}" scale="{scale}"/></geometry>
    </visual>
    <collision><geometry><mesh filename="{fn}"/></geometry></collision>
  </link>
  <joint name="j1" type="revolute">
    <parent link="base"/><child link="l1"/><axis xyz="0 0 1"/>
    <limit lower="-3" upper="3" effort="5" velocity="1"/>
  </joint>
</robot>
"""


def _mesh_urdf(folder, fn, scale="1 1 1", name="m.urdf"):
    p = Path(folder) / name
    p.write_text(MESH_URDF.format(fn=fn, scale=scale), encoding="utf-8")
    return p


def _codes(res, sev):
    return [i.code for i in res.report.by_severity(sev)]


@pytest.fixture
def win(qapp):
    w = MainWindow()
    w.show()
    yield w
    w.project_controller._is_modified = False
    w.close()


def _import(win, path):
    with mock.patch("app.main_window.QMessageBox.warning") as w:
        win.import_urdf_file(str(path))
        win.task_runner.wait_all(5000)
        QApplication.processEvents()
    return w


# ---------------------------------------------------------------------------
# resolve_mesh
# ---------------------------------------------------------------------------

def test_resolve_relative(tmp_path):
    shutil.copy(MESHES / "cube.stl", tmp_path / "c.stl")
    assert resolve_mesh("c.stl", tmp_path, []) == str((tmp_path / "c.stl").resolve())
    assert resolve_mesh("nope.stl", tmp_path, []) is None


def test_resolve_absolute_and_file_url(tmp_path):
    f = tmp_path / "c.stl"
    shutil.copy(MESHES / "cube.stl", f)
    other = tmp_path / "elsewhere"
    other.mkdir()
    assert resolve_mesh(str(f), other, []) == str(f.resolve())
    assert resolve_mesh(f.resolve().as_uri(), other, []) == str(f.resolve())
    assert resolve_mesh((tmp_path / "x.stl").resolve().as_uri(), other, []) is None


def test_resolve_package_search_roots(tmp_path):
    root = tmp_path / "roots"
    (root / "pkg" / "meshes").mkdir(parents=True)
    shutil.copy(MESHES / "cube.stl", root / "pkg" / "meshes" / "a.stl")
    shutil.copy(MESHES / "cube.stl", root / "b.stl")
    urdf_dir = tmp_path / "unrelated"
    urdf_dir.mkdir()
    got = resolve_mesh("package://pkg/meshes/a.stl", urdf_dir, [root])
    assert got == str((root / "pkg" / "meshes" / "a.stl").resolve())
    # <root>/<rel> form
    got = resolve_mesh("package://pkg/b.stl", urdf_dir, [str(root)])
    assert got == str((root / "b.stl").resolve())
    assert resolve_mesh("package://pkg/meshes/a.stl", urdf_dir, []) is None


def test_resolve_package_walk_up_catkin(tmp_path):
    pkg = tmp_path / "ws" / "src" / "my_robot"
    (pkg / "urdf").mkdir(parents=True)
    (pkg / "meshes").mkdir()
    shutil.copy(MESHES / "cube.stl", pkg / "meshes" / "c.stl")
    got = resolve_mesh("package://my_robot/meshes/c.stl", pkg / "urdf", [])
    assert got == str((pkg / "meshes" / "c.stl").resolve())
    # sibling package folder in an ancestor
    sib = tmp_path / "ws" / "src" / "other_pkg" / "urdf"
    sib.mkdir(parents=True)
    got = resolve_mesh("package://my_robot/meshes/c.stl", sib, [])
    assert got == str((pkg / "meshes" / "c.stl").resolve())


def test_import_package_mesh_catkin_tree(tmp_path):
    pkg = tmp_path / "ws" / "src" / "arm_pkg"
    (pkg / "urdf").mkdir(parents=True)
    (pkg / "meshes").mkdir()
    shutil.copy(MESHES / "cube.stl", pkg / "meshes" / "c.stl")
    p = _mesh_urdf(pkg / "urdf", "package://arm_pkg/meshes/c.stl", name="arm.urdf")
    res = import_urdf(p)
    assert res.model is not None
    assert _codes(res, "warning") == []
    v = res.visuals[0]
    assert v.kind == "mesh" and not v.fallback
    assert Path(v.mesh_path) == (pkg / "meshes" / "c.stl").resolve()
    assert Path(res.collisions[0].mesh_path) == (pkg / "meshes" / "c.stl").resolve()


def test_import_package_mesh_with_search_roots_arg(tmp_path):
    root = tmp_path / "pkgs"
    (root / "zed").mkdir(parents=True)
    shutil.copy(MESHES / "cube.obj", root / "zed" / "c.obj")
    d = tmp_path / "x"
    d.mkdir()
    p = _mesh_urdf(d, "package://zed/c.obj")
    assert _codes(import_urdf(p), "warning") == [ir.URDF_MESH_MISSING]
    res = import_urdf(p, search_roots=[str(root)])
    assert _codes(res, "warning") == []
    assert Path(res.visuals[0].mesh_path) == (root / "zed" / "c.obj").resolve()


# ---------------------------------------------------------------------------
# Importer: warnings, fallback, primitives
# ---------------------------------------------------------------------------

def test_missing_mesh_warning_and_fallback(tmp_path):
    res = import_urdf(_mesh_urdf(tmp_path, "gone.stl"))
    assert res.model is not None
    assert _codes(res, "warning") == [ir.URDF_MESH_MISSING]
    v = res.visuals[0]
    assert v.fallback and v.mesh_path is None
    m = load_visual_mesh(v, triad_length=0.2)
    b = m.bounds
    assert b[1] - b[0] == pytest.approx(0.2)


def test_dae_mesh_warning_and_fallback(tmp_path):
    shutil.copy(MESHES / "cube.dae", tmp_path / "c.dae")
    res = import_urdf(_mesh_urdf(tmp_path, "c.dae"))
    assert _codes(res, "warning") == [ir.URDF_MESH_FORMAT]
    assert res.visuals[0].fallback


def test_mesh_too_large_warning_and_not_drawn(tmp_path, monkeypatch):
    shutil.copy(MESHES / "cube.stl", tmp_path / "c.stl")
    monkeypatch.setattr(vis, "MAX_TRIANGLES", 5)
    res = import_urdf(_mesh_urdf(tmp_path, "c.stl"))
    assert _codes(res, "warning") == [ir.URDF_MESH_TOO_LARGE]
    assert load_visual_mesh(res.visuals[0], 0.1) is None


def test_stl_obj_ply_load_and_scale(tmp_path):
    for ext in ("stl", "obj", "ply"):
        shutil.copy(MESHES / f"cube.{ext}", tmp_path / f"c.{ext}")
        res = import_urdf(
            _mesh_urdf(tmp_path, f"c.{ext}", scale="2 3 4", name=f"{ext}.urdf")
        )
        assert _codes(res, "warning") == [], ext
        m = load_visual_mesh(res.visuals[0], 0.1)
        b = m.bounds
        assert (b[1] - b[0], b[3] - b[2], b[5] - b[4]) == pytest.approx(
            (0.2, 0.3, 0.4), abs=1e-6
        )


def test_primitives_parsed():
    res = import_urdf(PRIMS)
    assert res.model is not None
    assert _codes(res, "warning") == []
    assert _codes(res, "info").count(ir.URDF_EXTRA_VISUAL) == 1
    assert [v.kind for v in res.visuals] == ["box", "cylinder", "sphere"]
    assert [v.link_index for v in res.visuals] == [0, 1, 2]
    assert res.visuals[0].rgba == [0.7, 0.7, 0.7, 1.0]
    assert res.visuals[1].rgba == [0.0, 0.0, 1.0, 1.0]        # named material
    assert res.visuals[2].rgba == [1.0, 0.0, 0.5 * 0.0, 0.5] or \
        res.visuals[2].rgba == [1.0, 0.0, 0.0, 0.5]           # inline colour
    assert res.visuals[1].origin_xyz == [0.0, 0.0, 0.25]
    assert len(res.collisions) == 1 and res.collisions[0].kind == "box"
    assert ir.URDF_IGNORED_ELEMENT not in [i.code for i in res.report.issues]


def test_cylinder_axis_is_link_z():
    v = LinkVisual(0, "a", "cylinder", {"radius": 0.05, "length": 0.5})
    b = load_visual_mesh(v).bounds
    assert b[5] - b[4] == pytest.approx(0.5)
    assert b[1] - b[0] == pytest.approx(0.1, abs=2e-3)


def test_link_visual_dict_round_trip():
    v = LinkVisual(3, "x", "mesh", {"filename": "a", "scale": [1, 2, 3]},
                   [1, 2, 3], [0.1, 0.2, 0.3], [0.1, 0.2, 0.3, 0.4], "C:/a.stl", True)
    assert LinkVisual.from_dict(v.to_dict()) == v


# ---------------------------------------------------------------------------
# Viewport
# ---------------------------------------------------------------------------

def test_primitives_actor_count(win):
    _import(win, PRIMS)
    assert len(win.robot_view.visual_actors) == 3
    assert len(win.project_controller.current_project.visuals) == 3


def test_visual_actors_follow_q(win):
    _import(win, PRIMS)
    model = win.project_controller.robot
    rv = win.robot_view
    visuals = win.project_controller.current_project.visuals
    for q in ([0.0, 0.0], [0.4, -0.7], [-1.2, 1.1]):
        res = cb.fk(model, q)
        rv.update(model, res)
        for actor, v in zip(rv.visual_actors, visuals):
            want = np.asarray(res["frames"][v.link_index]) @ vis.visual_origin_iso(v)
            assert np.allclose(np.asarray(actor.user_matrix), want, atol=1e-9)


def test_visual_actors_survive_rebuild(win):
    _import(win, PRIMS)
    rv = win.robot_view
    model = win.project_controller.robot
    # a much longer first joint changes the scale -> actors are rebuilt
    p = cb.Pose()
    p.xyz = [0.0, 0.0, 5.0]
    p.rpy = [0.0, 0.0, 0.0]
    model.joints[0].origin = p
    rv.update(model, cb.fk(model))
    assert len(rv.visual_actors) == 3


def test_toggle_visuals_independent_of_frames(win):
    _import(win, PRIMS)
    rv = win.robot_view
    assert rv.show_visuals
    assert all(a.visibility for a in rv.visual_actors)
    win._action_show_visuals.setChecked(False)
    assert not rv.show_visuals
    assert not any(a.visibility for a in rv.visual_actors)
    assert all(a.visibility for a in rv.frame_actors)
    win._action_show_visuals.setChecked(True)
    assert all(a.visibility for a in rv.visual_actors)
    win._action_show_frames.setChecked(False)
    assert all(a.visibility for a in rv.visual_actors)
    win._action_show_frames.setChecked(True)


def test_clear_removes_visual_actors(win):
    _import(win, PRIMS)
    rv = win.robot_view
    assert rv.visual_actors
    rv.clear()
    assert rv.visual_actors == []


def test_too_large_mesh_not_drawn(win, tmp_path, monkeypatch):
    shutil.copy(MESHES / "cube.stl", tmp_path / "c.stl")
    monkeypatch.setattr(vis, "MAX_TRIANGLES", 5)
    _import(win, _mesh_urdf(tmp_path, "c.stl"))
    assert win.robot_view.visual_actors == []


def test_missing_mesh_draws_fallback_cube(win, tmp_path):
    w = _import(win, _mesh_urdf(tmp_path, "gone.stl"))
    assert not w.called                   # R3.6: no popup, report in the dock
    assert ir.URDF_MESH_MISSING in [i.code for i in win.import_report_panel.report.issues]
    assert win.import_report_dock.isVisible()
    assert len(win.robot_view.visual_actors) == 1


# ---------------------------------------------------------------------------
# Archive
# ---------------------------------------------------------------------------

def _project_with_meshes(tmp_path):
    d = tmp_path / "src"
    d.mkdir()
    shutil.copy(MESHES / "cube.stl", d / "c.stl")
    res = import_urdf(_mesh_urdf(d, "c.stl"))
    p = ProjectData.new("meshproj")
    p.robot_model = res.model
    p.visuals = res.visuals
    p.collisions = res.collisions
    return p, d


def test_save_open_without_original_meshes(win, tmp_path):
    d = tmp_path / "src"
    d.mkdir()
    shutil.copy(MESHES / "cube.stl", d / "c.stl")
    _import(win, _mesh_urdf(d, "c.stl"))
    pc = win.project_controller
    out = tmp_path / "out.robotproj"
    assert pc.save_project_as(win, str(out))
    with zipfile.ZipFile(out) as zf:
        assert "assets/c.stl" in zf.namelist()
        assert "visuals.json" in zf.namelist()
    shutil.rmtree(d)
    assert pc.open_project(win, str(out))
    proj = pc.current_project
    assert proj.asset_dir and Path(proj.asset_dir).is_dir()
    v = proj.visuals[0]
    assert not v.fallback and Path(v.mesh_path).is_file()
    assert Path(v.mesh_path).parent == Path(proj.asset_dir)
    assert len(win.robot_view.visual_actors) == 1
    b = win.robot_view.visual_actors[0].GetBounds()
    assert b[1] - b[0] == pytest.approx(0.1, abs=1e-6)     # real mesh, not fallback
    asset_dir = proj.asset_dir
    pc.close_project(None)
    assert not Path(asset_dir).exists()


def test_asset_name_clash(tmp_path):
    p, d = _project_with_meshes(tmp_path)
    (tmp_path / "other").mkdir()
    shutil.copy(MESHES / "cube.obj", tmp_path / "other" / "c.stl")  # same basename
    second = LinkVisual(1, "l1", "mesh", {"filename": "c.stl", "scale": [1, 1, 1]},
                        mesh_path=str((tmp_path / "other" / "c.stl").resolve()))
    p.visuals = p.visuals + [second]
    out = tmp_path / "p.robotproj"
    save(p, out)
    with zipfile.ZipFile(out) as zf:
        names = sorted(n for n in zf.namelist() if n.startswith("assets/"))
    # visual + collision share one file; the clash gets _1
    assert names == ["assets/c.stl", "assets/c_1.stl"]
    q = load(out)
    paths = [v.mesh_path for v in q.visuals]
    assert len(set(paths)) == 2
    assert all(Path(x).is_file() for x in paths)
    q.cleanup_assets()


def test_roundtrip_visuals_and_collisions(tmp_path):
    p, d = _project_with_meshes(tmp_path)
    out = tmp_path / "p.robotproj"
    save(p, out)
    q = load(out)
    assert PROJECT_FORMAT_VERSION == "1.2.0"
    assert q.metadata.format_version == "1.2.0"
    assert len(q.visuals) == 1 and len(q.collisions) == 1
    assert q.visuals[0].kind == "mesh"
    assert q.visuals[0].origin_xyz == [0.0, 0.0, 0.1]
    assert q.visuals[0].link_index == 1
    q.cleanup_assets()


def test_old_project_without_visuals_json_opens(tmp_path):
    p, d = _project_with_meshes(tmp_path)
    out = tmp_path / "p.robotproj"
    save(p, out)
    old = tmp_path / "old.robotproj"
    with zipfile.ZipFile(out) as zin, zipfile.ZipFile(old, "w") as zout:
        for n in zin.namelist():
            if n == "visuals.json" or n.startswith("assets/"):
                continue
            data = zin.read(n)
            if n == "project_metadata.json":
                data = data.replace(b'"1.2.0"', b'"1.0.0"')
            zout.writestr(n, data)
    q = load(old)
    assert q.visuals == [] and q.collisions == [] and q.asset_dir is None
    assert q.metadata.format_version == "1.0.0"


def test_missing_asset_in_archive_falls_back(tmp_path, caplog):
    p, d = _project_with_meshes(tmp_path)
    out = tmp_path / "p.robotproj"
    save(p, out)
    broken = tmp_path / "broken.robotproj"
    with zipfile.ZipFile(out) as zin, zipfile.ZipFile(broken, "w") as zout:
        for n in zin.namelist():
            if not n.startswith("assets/"):
                zout.writestr(n, zin.read(n))
    with caplog.at_level("WARNING"):
        q = load(broken)
    assert q.visuals[0].fallback and q.visuals[0].mesh_path is None
    assert any("Missing asset" in r.message for r in caplog.records)
    assert load_visual_mesh(q.visuals[0], 0.1) is not None


# ---------------------------------------------------------------------------
# Settings + dialog
# ---------------------------------------------------------------------------

def test_mesh_search_roots_setting_and_dialog(qapp):
    from app.panels.mesh_paths_dialog import MeshPathsDialog
    s = AppSettings()
    old = s.mesh_search_roots()
    try:
        s.set_mesh_search_roots(["C:/a", "C:/b"])
        assert s.mesh_search_roots() == ["C:/a", "C:/b"]
        dlg = MeshPathsDialog(s.mesh_search_roots())
        dlg.add_root("C:/c")
        dlg.add_root("C:/a")          # duplicate ignored
        assert dlg.roots() == ["C:/a", "C:/b", "C:/c"]
        s.set_mesh_search_roots(dlg.roots())
        assert s.mesh_search_roots() == ["C:/a", "C:/b", "C:/c"]
    finally:
        s.set_mesh_search_roots(old)
