"""
tests/python/test_physical_props.py
R3.7: URDF inertial import, mass-property estimation, Physical panel, save/load.
"""

import math
import os
import unittest.mock as mock
from pathlib import Path

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pyvista as pv
from PySide6.QtWidgets import QApplication

import core_bridge as cb
from app.importers import import_report as ir
from app.importers import import_urdf
from app.importers.visuals import LinkVisual
from app.main_window import MainWindow
from app.panels import physical_panel as pp
from app.physics.estimate import (
    estimate_inertial,
    primitive_inertial,
    visual_inertial,
)
from app.project.project_data import ProjectData
from app.project.serializer import load, save

FX = Path(__file__).parent / "fixtures"


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _rot_rpy(rpy):
    p = cb.Pose()
    p.xyz = [0.0, 0.0, 0.0]
    p.rpy = list(rpy)
    return np.asarray(cb.pose_to_iso(p))[:3, :3]


def _tensor3(t):
    return np.array([[t[0], t[1], t[2]], [t[1], t[3], t[4]], [t[2], t[4], t[5]]])


def two_link_model(q=0.0):
    """l0 (mass 2, com 0) -> revolute z at x=1 -> l1 (mass 1, com [0.5,0,0])."""
    m = cb.RobotModel()
    for name in ("l0", "l1"):
        lk = cb.Link()
        lk.name = name
        m.links.append(lk)
    j = cb.Joint()
    j.name = "j1"
    j.parent_link = 0
    j.child_link = 1
    j.kinematics = cb.JointKinematics.OriginAxis
    j.type = cb.JointType.Revolute
    o = cb.Pose()
    o.xyz = [1.0, 0.0, 0.0]
    o.rpy = [0.0, 0.0, 0.0]
    j.origin = o
    j.axis = [0.0, 0.0, 1.0]
    j.q = q
    m.joints.append(j)
    for lk, mass, com in ((m.links[0], 2.0, [0, 0, 0]), (m.links[1], 1.0, [0.5, 0, 0])):
        i = cb.Inertia()
        i.mass = mass
        i.com = com
        i.inertia_tensor = [0.1, 0, 0, 0.1, 0, 0.1]
        lk.inertia = i
    m.tcp_link = 1
    return m


@pytest.fixture
def win(qapp):
    w = MainWindow()
    w.show()
    yield w
    w.project_controller._is_modified = False
    w.close()


def _open_two_link(win):
    proj = ProjectData.new("Phys")
    proj.robot_model = two_link_model()
    win.project_controller._set_project(proj, modified=False)
    return proj


# ---------------------------------------------------------------------------
# Python binding of the C++ side
# ---------------------------------------------------------------------------

def test_inertia_defaults_and_validate_physical_binding():
    lk = cb.Link()
    assert lk.inertia.mass == 0.0
    assert list(lk.inertia.inertia_tensor) == [0.0] * 6
    m = two_link_model()
    assert list(m.validate_physical()) == []
    m.links[1].inertia = cb.Inertia()
    msgs = list(m.validate_physical())
    assert len(msgs) == 1 and "mass not set" in msgs[0] and "'l1'" in msgs[0]
    assert m.validate() == []


# ---------------------------------------------------------------------------
# URDF inertial import
# ---------------------------------------------------------------------------

_INERTIAL_URDF = """<?xml version="1.0"?>
<robot name="r">
  <link name="base"/>
  <link name="l1">
    <inertial>
      <origin xyz="0.1 -0.2 0.3" rpy="0.3 -0.5 1.1"/>
      <mass value="2.5"/>
      <inertia ixx="0.4" ixy="0.01" ixz="-0.02" iyy="0.5" iyz="0.03" izz="0.6"/>
    </inertial>
  </link>
  <link name="l2"/>
  <joint name="j1" type="revolute">
    <parent link="base"/><child link="l1"/><axis xyz="0 0 1"/>
    <limit lower="-3" upper="3" effort="5" velocity="1"/>
  </joint>
  <joint name="j2" type="fixed">
    <parent link="l1"/><child link="l2"/>
  </joint>
</robot>
"""


def test_urdf_inertial_rotated_into_link_frame(tmp_path):
    p = tmp_path / "r.urdf"
    p.write_text(_INERTIAL_URDF, encoding="utf-8")
    res = import_urdf(p)
    assert res.model is not None
    lk = res.model.links[1]
    assert lk.inertia.mass == 2.5
    assert list(lk.inertia.com) == [0.1, -0.2, 0.3]
    rot = _rot_rpy([0.3, -0.5, 1.1])
    tensor = np.array([[0.4, 0.01, -0.02], [0.01, 0.5, 0.03], [-0.02, 0.03, 0.6]])
    expect = rot @ tensor @ rot.T
    got = _tensor3(list(lk.inertia.inertia_tensor))
    assert np.max(np.abs(got - expect)) < 1e-12


def test_urdf_missing_inertial_codes(tmp_path):
    p = tmp_path / "r.urdf"
    p.write_text(_INERTIAL_URDF, encoding="utf-8")
    rep = import_urdf(p).report
    # root without inertial -> info; l2 (child of a FIXED joint) -> warning
    assert [i.element for i in rep.by_code(ir.URDF_ROOT_NO_INERTIAL)] == ["base"]
    assert rep.by_code(ir.URDF_ROOT_NO_INERTIAL)[0].severity == ir.SEV_INFO
    no = rep.by_code(ir.URDF_NO_INERTIAL)
    assert [i.element for i in no] == ["l2"] and no[0].severity == ir.SEV_WARNING
    # l2 hangs on a fixed joint: no "mass not set"
    assert rep.by_code(ir.URM_PHYSICAL) == []
    assert rep.has_errors() is False


def test_urdf_physical_warnings_do_not_block_import(tmp_path):
    bad = _INERTIAL_URDF.replace('<mass value="2.5"/>', '<mass value="-1"/>')
    p = tmp_path / "r.urdf"
    p.write_text(bad, encoding="utf-8")
    res = import_urdf(p)
    assert res.model is not None
    phys = res.report.by_code(ir.URM_PHYSICAL)
    assert len(phys) == 1 and "negative" in phys[0].message
    assert phys[0].severity == ir.SEV_WARNING
    assert not res.report.has_errors()


def test_ignored_kinds_only_transmission_and_gazebo():
    from app.importers import urdf

    assert tuple(urdf._IGNORED_KINDS) == ("transmission", "gazebo")


# ---------------------------------------------------------------------------
# Estimation
# ---------------------------------------------------------------------------

def test_estimate_needs_exactly_one_of_density_mass():
    cube = pv.Cube()
    with pytest.raises(ValueError):
        estimate_inertial(cube)
    with pytest.raises(ValueError):
        estimate_inertial(cube, density=1.0, total_mass=1.0)


def test_box_mesh_matches_analytic_exactly():
    x, y, z = 1.0, 2.0, 3.0
    mass, com, t, warns = estimate_inertial(
        pv.Cube(x_length=x, y_length=y, z_length=z), density=1000.0
    )
    m = 1000.0 * x * y * z
    assert warns == []
    assert mass == pytest.approx(m, rel=1e-9)
    assert np.allclose(com, 0.0, atol=1e-9)
    expect = [m * (y * y + z * z) / 12, 0, 0, m * (x * x + z * z) / 12, 0,
              m * (x * x + y * y) / 12]
    assert np.allclose(t, expect, rtol=1e-9, atol=1e-9)


def test_box_mesh_offset_com_and_total_mass():
    mass, com, t, _ = estimate_inertial(
        pv.Cube(center=(1.0, 2.0, 3.0), x_length=0.2, y_length=0.4, z_length=0.6),
        total_mass=5.0,
    )
    assert mass == 5.0
    # pv.Cube stores float32 points, so the offset cube is exact only to ~1e-7
    assert np.allclose(com, [1.0, 2.0, 3.0], atol=1e-6)
    expect = [5.0 * (0.16 + 0.36) / 12, 0, 0, 5.0 * (0.04 + 0.36) / 12, 0,
              5.0 * (0.04 + 0.16) / 12]
    assert np.allclose(t, expect, rtol=1e-5, atol=1e-6)


def test_total_mass_scaling_is_linear():
    cube = pv.Cube()
    a = estimate_inertial(cube, total_mass=1.0)
    b = estimate_inertial(cube, total_mass=4.0)
    assert b[0] == 4.0
    assert np.allclose(np.array(b[2]), 4.0 * np.array(a[2]), rtol=1e-12, atol=1e-15)


def test_cylinder_mesh_256_segments_within_half_percent():
    r, h, m = 0.5, 2.0, 3.0
    mass, com, t, warns = estimate_inertial(
        pv.Cylinder(radius=r, height=h, resolution=256, direction=(0, 0, 1)), total_mass=m
    )
    assert warns == []
    ixx = m * (3 * r * r + h * h) / 12
    izz = m * r * r / 2
    assert t[0] == pytest.approx(ixx, rel=5e-3)
    assert t[3] == pytest.approx(ixx, rel=5e-3)
    assert t[5] == pytest.approx(izz, rel=5e-3)
    assert np.allclose(com, 0.0, atol=1e-9)


def test_sphere_mesh_icosphere_within_one_percent():
    r, m = 0.5, 2.0
    mass, com, t, warns = estimate_inertial(
        pv.Icosphere(radius=r, nsub=4), total_mass=m
    )
    assert warns == []
    i = 0.4 * m * r * r
    for k in (0, 3, 5):
        assert t[k] == pytest.approx(i, rel=1e-2)
    density = estimate_inertial(pv.Icosphere(radius=r, nsub=4), density=1000.0)[0]
    assert density == pytest.approx(1000.0 * 4 / 3 * math.pi * r ** 3, rel=1e-2)


def test_open_mesh_falls_back_to_convex_hull_with_warning():
    cube = pv.Cube().triangulate()
    open_mesh = cube.extract_cells(range(cube.n_cells - 2)).extract_surface(
        algorithm="dataset_surface"
    )
    assert open_mesh.n_open_edges > 0
    mass, com, t, warns = estimate_inertial(open_mesh, density=1000.0)
    assert len(warns) == 1 and "convex hull" in warns[0]
    # hull of the 8 corners of the unit cube = the cube
    assert mass == pytest.approx(1000.0, rel=1e-9)
    assert t[0] == pytest.approx(1000.0 / 6, rel=1e-9)


def test_inward_normals_still_positive_mass():
    cube = pv.Cube().triangulate()
    flipped = cube.copy()
    flipped.flip_faces(inplace=True)
    mass, _c, t, _w = estimate_inertial(flipped, density=10.0)
    assert mass == pytest.approx(10.0, rel=1e-9)


def test_primitive_box_cylinder_sphere_analytic():
    m, _c, t, _w = primitive_inertial("box", {"size": [1, 2, 3]}, None, density=2.0)
    assert m == 12.0 and t[0] == pytest.approx(12 * 13 / 12)
    m, _c, t, _w = primitive_inertial(
        "cylinder", {"radius": 0.5, "length": 2.0}, None, total_mass=3.0)
    assert m == 3.0
    assert t[5] == pytest.approx(3 * 0.25 / 2) and t[0] == pytest.approx(3 * 4.75 / 12)
    m, _c, t, _w = primitive_inertial("sphere", {"radius": 0.5}, None, total_mass=2.0)
    assert t[0] == pytest.approx(0.4 * 2 * 0.25)


def test_primitive_origin_moves_com_and_rotates_tensor():
    xyz, rpy = [0.1, 0.2, 0.3], [0.0, 0.0, math.pi / 2]
    m, com, t, _w = primitive_inertial(
        "box", {"size": [1, 2, 3]}, (xyz, rpy), total_mass=1.0)
    assert np.allclose(com, xyz)
    # rotating 90 deg about z swaps Ixx and Iyy
    ixx_local, iyy_local = (4 + 9) / 12, (1 + 9) / 12
    assert t[0] == pytest.approx(iyy_local, abs=1e-12)
    assert t[3] == pytest.approx(ixx_local, abs=1e-12)
    # mesh path with the same visual origin agrees with the analytic path
    v = LinkVisual(1, "l1", "box", {"size": [1, 2, 3]}, xyz, rpy)
    a = visual_inertial(v, total_mass=1.0)
    assert np.allclose(a[2], t, atol=1e-12) and np.allclose(a[1], com)


def test_visual_inertial_mesh_applies_origin_and_scale():
    mesh_path = str(FX / "meshes" / "cube.stl")
    v = LinkVisual(1, "l1", "mesh", {"filename": "cube.stl", "scale": [2.0, 2.0, 2.0]},
                   [1.0, 0.0, 0.0], [0.0, 0.0, 0.0], mesh_path=mesh_path)
    mass, com, t, warns = visual_inertial(v, density=1.0)
    ref = pv.read(mesh_path)
    vol = ref.volume * 8.0
    assert warns == []
    assert mass == pytest.approx(vol, rel=1e-9)
    assert com[0] == pytest.approx(1.0 + 2.0 * np.mean(ref.points[:, 0]), abs=1e-9)


# ---------------------------------------------------------------------------
# Panel
# ---------------------------------------------------------------------------

def test_panel_table_shows_model_and_status(win):
    _open_two_link(win)
    panel = win.physical_panel
    assert panel.row_count == 2
    assert panel.table.item(0, pp.COL_MASS).text() == "2"
    assert panel.status_text(0) == "OK"
    win.project_controller.robot.links[1].inertia = cb.Inertia()
    panel.rebuild()
    assert "mass not set" in panel.status_text(1)


def test_panel_edit_changes_model_and_dirty_flag(win):
    _open_two_link(win)
    pc = win.project_controller
    assert not pc.is_modified
    win.physical_panel.set_value(1, pp.COL_MASS, "3.5e0")
    assert pc.robot.links[1].inertia.mass == 3.5
    assert pc.is_modified
    pc._is_modified = False
    win.physical_panel.set_value(1, pp.COL_COM + 2, "0.25")
    assert list(pc.robot.links[1].inertia.com) == [0.5, 0.0, 0.25]
    win.physical_panel.set_value(1, pp.COL_TENSOR + 5, "2e-1")
    assert pc.robot.links[1].inertia.inertia_tensor[5] == pytest.approx(0.2)
    assert pc.is_modified


def test_panel_bad_text_reverts_and_does_not_dirty(win):
    _open_two_link(win)
    pc = win.project_controller
    win.physical_panel.set_value(1, pp.COL_MASS, "abc")
    assert pc.robot.links[1].inertia.mass == 1.0
    assert win.physical_panel.table.item(1, pp.COL_MASS).text() == "1"
    assert not pc.is_modified


def test_panel_negative_mass_shows_warning_icon_text(win):
    _open_two_link(win)
    win.physical_panel.set_value(0, pp.COL_MASS, "-1")
    assert "negative" in win.physical_panel.status_text(0)


def test_total_com_hand_value_and_follows_q(win):
    _open_two_link(win)
    panel = win.physical_panel
    assert panel.total_mass() == 3.0
    assert np.allclose(panel.total_com(), [0.5, 0.0, 0.0], atol=1e-12)
    assert "Total mass: 3" in panel.readout_text
    # move the joint through the joint panel signal path
    win.project_controller.robot.joints[0].q = math.pi / 2
    win._on_joint_q_changed()
    assert np.allclose(panel.total_com(), [1.0 / 3.0, 0.5 / 3.0, 0.0], atol=1e-9)
    assert "0.333333" in panel.readout_text


def test_scale_all_masses_to_total(win):
    _open_two_link(win)
    panel = win.physical_panel
    assert panel.scale_masses_to(6.0)
    robot = win.project_controller.robot
    assert [lk.inertia.mass for lk in robot.links] == pytest.approx([4.0, 2.0])
    assert robot.links[0].inertia.inertia_tensor[0] == pytest.approx(0.2)
    assert win.project_controller.is_modified
    assert not panel.scale_masses_to(-1.0)


def test_estimate_from_visuals_single_and_all(win):
    proj = ProjectData.new("Est")
    proj.robot_model = two_link_model()
    proj.visuals = [
        LinkVisual(0, "l0", "box", {"size": [1, 1, 1]}),
        LinkVisual(1, "l1", "cylinder", {"radius": 0.5, "length": 2.0},
                   [0.0, 0.0, 1.0], [0.0, 0.0, 0.0]),
    ]
    win.project_controller._set_project(proj, modified=False)
    panel = win.physical_panel
    robot = win.project_controller.robot
    # single link, total mass
    warns = panel.estimate_from_visuals([1], total_mass=3.0)
    assert warns == []
    assert robot.links[1].inertia.mass == pytest.approx(3.0)
    assert list(robot.links[1].inertia.com) == pytest.approx([0.0, 0.0, 1.0])
    assert win.project_controller.is_modified
    # all links, density: box 1 m^3 * 100 = 100 kg
    panel.estimate_from_visuals(None, density=100.0)
    assert robot.links[0].inertia.mass == pytest.approx(100.0)
    assert robot.links[1].inertia.mass == pytest.approx(100.0 * math.pi * 0.25 * 2.0)
    # all links, total mass shared by volume
    panel.estimate_from_visuals(None, total_mass=10.0)
    assert panel.total_mass() == pytest.approx(10.0)
    assert robot.links[0].inertia.mass / robot.links[1].inertia.mass == pytest.approx(
        1.0 / (math.pi * 0.5), rel=1e-9)
    # panel cells follow
    assert float(panel.table.item(0, pp.COL_MASS).text()) == pytest.approx(
        robot.links[0].inertia.mass, rel=1e-8)


def test_estimate_skips_link_without_visual(win):
    _open_two_link(win)
    warns = win.physical_panel.estimate_from_visuals([0], density=10.0)
    assert len(warns) == 1 and "no visual" in warns[0]
    assert win.project_controller.robot.links[0].inertia.mass == 2.0


def test_view_menu_toggle_shows_dock(win):
    win.physical_dock.hide()            # saved window layout may differ
    assert not win._action_view_physical.isChecked()
    win._action_view_physical.setChecked(True)
    assert win.physical_dock.isVisible()
    assert win.physical_dock.windowTitle() == "Physical"
    win._action_view_physical.setChecked(False)
    assert not win.physical_dock.isVisible()


# ---------------------------------------------------------------------------
# Persistence
# ---------------------------------------------------------------------------

def test_save_load_round_trip_keeps_physical_data(win, tmp_path):
    _open_two_link(win)
    win.physical_panel.set_value(1, pp.COL_MASS, "7.25")
    win.physical_panel.set_value(1, pp.COL_COM, "0.125")
    win.physical_panel.set_value(1, pp.COL_TENSOR + 1, "0.0123")
    proj = win.project_controller.current_project
    path = tmp_path / "phys.robotproj"
    save(proj, path)
    loaded = load(path)
    lk = loaded.robot_model.links[1]
    assert lk.inertia.mass == 7.25
    assert lk.inertia.com[0] == 0.125
    assert lk.inertia.inertia_tensor[1] == 0.0123
    assert loaded.robot_model.links[0].inertia.mass == 2.0


def test_import_save_load_physical_from_urdf(tmp_path):
    p = tmp_path / "r.urdf"
    p.write_text(_INERTIAL_URDF, encoding="utf-8")
    res = import_urdf(p)
    proj = ProjectData.new("r")
    proj.robot_model = res.model
    path = tmp_path / "r.robotproj"
    save(proj, path)
    back = load(path).robot_model
    a = res.model.links[1].inertia
    b = back.links[1].inertia
    assert b.mass == a.mass
    assert list(b.com) == list(a.com)
    assert list(b.inertia_tensor) == pytest.approx(list(a.inertia_tensor), abs=1e-15)
