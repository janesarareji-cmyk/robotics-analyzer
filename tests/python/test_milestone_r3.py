"""
tests/python/test_milestone_r3.py
R3 stage milestone: 6-DOF arm + mimic gripper, URDF import to save/open.
Offscreen MainWindow, real core_bridge, independent numpy FK.
"""

import os
import shutil
import unittest.mock as mock
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication

import core_bridge as cb
from app.importers import import_report as ir
from app.main_window import MainWindow

FX = Path(__file__).parent / "fixtures"
URDF_NAME = "six_dof_with_gripper.urdf"
TOL = 1e-9


@pytest.fixture
def win(qapp):
    w = MainWindow()
    w.show()
    yield w
    w.project_controller._is_modified = False
    w.close()


@pytest.fixture
def src(tmp_path):
    """Copy of fixture URDF + meshes, so the originals can be deleted later."""
    d = tmp_path / "src"
    (d / "urdf").mkdir(parents=True)
    shutil.copytree(FX / "meshes", d / "meshes")
    shutil.copy(FX / "urdf" / URDF_NAME, d / "urdf" / URDF_NAME)
    return d


def _import(win, path):
    with mock.patch("app.main_window.QMessageBox.warning") as w:
        win.import_urdf_file(str(path))
        win.task_runner.wait_all(5000)
        QApplication.processEvents()
    return w


def _set_tcp(win, link_name):
    with mock.patch("app.main_window.QInputDialog.getItem",
                    return_value=(link_name, True)):
        win._action_set_tcp.trigger()


# ---------------------------------------------------------------------------
# Independent numpy kinematics, parsed from the URDF text (no core_bridge)
# ---------------------------------------------------------------------------

def _floats(s, default):
    return [float(x) for x in s.split()] if s else list(default)


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def _axis_angle(axis, q):
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(q) * K + (1 - np.cos(q)) * (K @ K)


def _hom(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


class NumpyRobot:
    """Parse the fixture; compute link frames and mass properties."""

    def __init__(self, path):
        root = ET.parse(path).getroot()
        self.links = []
        self.inertial = {}
        for lk in root.findall("link"):
            n = lk.get("name")
            self.links.append(n)
            ine = lk.find("inertial")
            o = ine.find("origin")
            xyz = _floats(o.get("xyz") if o is not None else None, (0, 0, 0))
            self.inertial[n] = (float(ine.find("mass").get("value")), np.array(xyz))
        self.joints = []
        for j in root.findall("joint"):
            o = j.find("origin")
            xyz = _floats(o.get("xyz"), (0, 0, 0))
            rpy = _floats(o.get("rpy"), (0, 0, 0))
            ax = j.find("axis")
            mim = j.find("mimic")
            self.joints.append({
                "name": j.get("name"), "type": j.get("type"),
                "parent": j.find("parent").get("link"), "child": j.find("child").get("link"),
                "T0": _hom(_rpy(*rpy), xyz),
                "axis": _floats(ax.get("xyz"), (1, 0, 0)) if ax is not None else [1, 0, 0],
                "mimic": (mim.get("joint"), float(mim.get("multiplier")),
                          float(mim.get("offset"))) if mim is not None else None,
            })

    def frames(self, q):
        """q: dict joint name -> value (mimic joints derived)."""
        F = {self.links[0]: np.eye(4)}
        for j in self.joints:  # fixture is in topological order
            if j["mimic"] is not None:
                m, k, off = j["mimic"]
                qj = k * q[m] + off
            else:
                qj = q[j["name"]]
            if j["type"] == "prismatic":
                Tq = _hom(np.eye(3), np.asarray(j["axis"], float) * qj)
            else:
                Tq = _hom(_axis_angle(j["axis"], qj), np.zeros(3))
            F[j["child"]] = F[j["parent"]] @ j["T0"] @ Tq
        return F

    def mass_props(self, q):
        F = self.frames(q)
        total, acc = 0.0, np.zeros(3)
        for n in self.links:
            m, c = self.inertial[n]
            total += m
            acc += m * (F[n][:3, :3] @ c + F[n][:3, 3])
        return total, acc / total


def _drive(win, q_by_name):
    """Set joints through the JointPanel spinboxes; return the model q by name."""
    jp = win.joint_panel
    robot = win.project_controller.robot
    for row in jp.rows + jp.branch_rows:
        name = robot.joints[row.joint_index].name
        if name in q_by_name:
            row.spinbox.setValue(q_by_name[name] * row.to_display)
    QApplication.processEvents()
    return _q_now(win)


def _q_now(win):
    return {j.name: j.q for j in win.project_controller.robot.joints}


# ---------------------------------------------------------------------------
# Milestone
# ---------------------------------------------------------------------------

def test_fixture_shape():
    npr = NumpyRobot(FX / "urdf" / URDF_NAME)
    assert npr.links == ["base", "l1", "l2", "l3", "l4", "l5", "l6", "finger_l", "finger_r"]
    assert len(npr.joints) == 8
    assert all(j["type"] == "revolute" for j in npr.joints[:6])
    assert any(j["axis"][0] != 0 or j["axis"][1] != 0 for j in npr.joints[:6])


def test_milestone_r3(win, src, tmp_path):
    npr = NumpyRobot(src / "urdf" / URDF_NAME)
    pc = win.project_controller

    # ---- 1. import: no errors, TCP chosen info
    w = _import(win, src / "urdf" / URDF_NAME)
    assert w.call_count == 0
    report = win.import_report_panel.report
    assert report.by_severity("error") == []
    assert ir.URDF_TCP_CHOSEN in [i.code for i in report.issues]
    robot = pc.robot
    names = [lk.name for lk in robot.links]
    assert names == npr.links
    assert robot.dof() == 7
    assert names[robot.tcp_link] == "finger_l"
    assert len(pc.current_project.visuals) == 9
    assert all(not v.fallback for v in pc.current_project.visuals)

    # ---- 2. set TCP to l6; tree + chain split
    _set_tcp(win, "l6")
    assert names[robot.tcp_link] == "l6"
    mp = win.model_panel
    assert mp.link_item_count == 9
    for k in range(7):
        assert mp.is_active_item(mp.link_item(k))
    for k in range(6):
        assert mp.is_active_item(mp.joint_item(k))
    for k in (7, 8):
        assert not mp.is_active_item(mp.link_item(k))
    for k in (6, 7):
        assert not mp.is_active_item(mp.joint_item(k))
    jp = win.joint_panel
    assert [r.joint_index for r in jp.rows] == [0, 1, 2, 3, 4, 5]
    assert [r.joint_index for r in jp.branch_rows] == [6]  # jf_r is mimic: no row
    seg = win.robot_view.branch_segments
    assert seg is not None and len(seg) == 2

    # ---- 3. move joints; TCP equals independent numpy FK
    for qs in (
        [0.3, -0.5, 0.8, 0.4, -0.7, 1.1],
        [-1.2, 0.9, -0.4, -1.0, 0.6, -0.3],
    ):
        q = _drive(win, {f"j{i + 1}": v for i, v in enumerate(qs)})
        F = npr.frames(q)
        tcp = np.asarray(win.robot_view.tcp_actor.user_matrix)
        np.testing.assert_allclose(tcp, F["l6"], atol=TOL)
        np.testing.assert_allclose(cb.fk(robot)["tcp"], F["l6"], atol=TOL)
        pos = jp.tcp_text.splitlines()[0]
        assert f"X: {F['l6'][0, 3] * 1000:.2f} mm" in pos

    # ---- 4. move jf_l: finger_r frame moves by the mimic rule
    q0 = _q_now(win)
    q = _drive(win, {"jf_l": 0.025})
    assert q["jf_l"] == pytest.approx(0.025, abs=1e-9)
    F0 = npr.frames({**q0, "jf_l": 0.0})
    F = npr.frames(q)
    fr = names.index("finger_r")
    fl = names.index("finger_l")
    frames = [np.asarray(f) for f in cb.fk(robot)["frames"]]
    np.testing.assert_allclose(frames[fr], F["finger_r"], atol=TOL)
    np.testing.assert_allclose(frames[fl], F["finger_l"], atol=TOL)
    # mimic rule: finger_r slides q along its own -y axis (multiplier 1, offset 0)
    d_r = np.linalg.inv(F0["finger_r"]) @ F["finger_r"]
    np.testing.assert_allclose(d_r[:3, 3], [0.0, -0.025, 0.0], atol=TOL)
    np.testing.assert_allclose(d_r[:3, :3], np.eye(3), atol=TOL)
    seg = np.array(win.robot_view.branch_segments)
    np.testing.assert_allclose(seg[1, 1], F["finger_r"][:3, 3], atol=TOL)

    # ---- 5. total mass and COM
    panel = win.physical_panel
    total, com = npr.mass_props(q)
    assert panel.total_mass() == pytest.approx(total, abs=1e-12)
    np.testing.assert_allclose(panel.total_com(), com, atol=TOL)
    assert f"Total mass: {total:.6g} kg" in panel.readout_text
    assert f"{com[0]:.6g}" in panel.readout_text

    # ---- 6. save, close, open
    json_before = cb.urm_to_json(robot)
    vis_before = [v.to_dict() for v in pc.current_project.visuals]
    inertia_before = [(lk.inertia.mass, list(lk.inertia.com), list(lk.inertia.inertia_tensor))
                      for lk in robot.links]
    tcp_before = robot.tcp_link
    out = tmp_path / "r3.robotproj"
    assert pc.save_project_as(win, str(out))
    shutil.rmtree(src)  # meshes must now come from the archive
    assert pc.close_project(None)
    assert not pc.has_project
    assert pc.open_project(win, str(out))
    r2 = pc.robot
    assert cb.urm_to_json(r2) == json_before
    assert r2.tcp_link == tcp_before
    assert [lk.name for lk in r2.links] == names
    assert [(lk.inertia.mass, list(lk.inertia.com), list(lk.inertia.inertia_tensor))
            for lk in r2.links] == inertia_before
    vis_after = [v.to_dict() for v in pc.current_project.visuals]
    assert len(vis_after) == len(vis_before) == 9
    asset_dir = Path(pc.current_project.asset_dir)
    for a, b in zip(vis_after, vis_before):
        assert Path(a["mesh_path"]).parent == asset_dir and Path(a["mesh_path"]).is_file()
        assert not a["fallback"]
        a2, b2 = dict(a), dict(b)
        a2.pop("mesh_path"), b2.pop("mesh_path")
        assert a2 == b2
    assert len(win.robot_view.visual_actors) == 9
    assert win.model_panel.link_item_count == 9
    total2, com2 = npr.mass_props(_q_now(win))
    assert win.physical_panel.total_mass() == pytest.approx(total2, abs=1e-12)
    np.testing.assert_allclose(win.physical_panel.total_com(), com2, atol=TOL)


@pytest.mark.parametrize("bad", ["bad_xml.urdf", "cycle.urdf", "two_roots.urdf",
                                 "missing_parent.urdf"])
def test_milestone_r3_broken_keeps_old_project(win, src, bad):
    pc = win.project_controller
    _import(win, src / "urdf" / URDF_NAME)
    proj = pc.current_project
    robot_json = cb.urm_to_json(pc.robot)
    n_vis = len(proj.visuals)
    w = _import(win, FX / "urdf" / bad)
    assert w.call_count == 0
    errors = win.import_report_panel.report.by_severity("error")
    assert errors
    assert win.import_report_dock.isVisible()
    assert pc.current_project is proj
    assert cb.urm_to_json(pc.robot) == robot_json
    assert len(proj.visuals) == n_vis
