"""
tests/python/test_ui_tree.py
R3.4: UI for tree and origin_axis joints (offscreen).
"""

import os
import unittest.mock as mock
from pathlib import Path

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication, QMessageBox

import core_bridge as cb
from app.main_window import MainWindow
from app.panels.dh_editor import COL_A, TREE_READONLY_TEXT
from app.panels.model_panel import is_plain_chain

FX = Path(__file__).parent / "fixtures" / "urdf"


@pytest.fixture
def win(qapp):
    w = MainWindow()
    w.show()
    yield w
    # Avoid discard prompt on teardown
    w.project_controller._is_modified = False
    w.close()


def _import(win, name, warn=None):
    """Import fixture through the main window and wait for the worker task."""
    with mock.patch("app.main_window.QMessageBox.warning") as w:
        win.import_urdf_file(str(FX / name))
        win.task_runner.wait_all(5000)
        QApplication.processEvents()
    if warn is not None:
        warn.append(w)
    return w


def _model_json(win):
    return cb.urm_to_json(win.project_controller.robot)


def _make_dh_arm(n=3):
    m = cb.RobotModel()
    for i in range(n + 1):
        lk = cb.Link()
        lk.name = f"link{i}"
        m.links.append(lk)
    for i in range(n):
        j = cb.Joint()
        j.name = f"j{i + 1}"
        j.parent_link = i
        j.child_link = i + 1
        j.dh.a = 0.5
        j.limits = cb.default_limits(j.type)
        m.joints.append(j)
    return m


# ----------------------------------------------------------------------
# Item 0: DH editor remove-joint fix
# ----------------------------------------------------------------------

def test_remove_middle_joint_keeps_model_valid(win):
    with mock.patch("app.project.project_controller.QInputDialog.getText",
                    return_value=("Arm", True)):
        win.project_controller.new_project(win)
    win.project_controller.set_robot(_make_dh_arm(3))
    ed = win.dh_editor
    assert ed._table.rowCount() == 3
    ed._table.selectRow(1)
    ed._on_remove_joint()
    robot = win.project_controller.robot
    assert robot.validate() == []
    assert len(robot.joints) == 2
    assert len(robot.links) == 3
    assert [(j.parent_link, j.child_link) for j in robot.joints] == [(0, 1), (1, 2)]
    assert [j.name for j in robot.joints] == ["j1", "j3"]


# ----------------------------------------------------------------------
# Import flow and tree
# ----------------------------------------------------------------------

def test_import_6dof_tree(win):
    _import(win, "serial_6dof.urdf")
    pc = win.project_controller
    assert pc.has_project and pc.is_modified
    assert pc.current_project.file_path is None
    assert pc.current_project.name == pc.robot.metadata.name
    mp = win.model_panel
    assert mp.link_item_count == 7
    for k in range(7):
        assert mp.is_active_item(mp.link_item(k))
    for k in range(6):
        assert mp.is_active_item(mp.joint_item(k))
    assert mp.joint_item(0).text(0) == "joint1 [revolute]"
    # root item is link 0 and the chain nests
    assert mp.tree.topLevelItemCount() == 1
    assert mp.tree.topLevelItem(0) is mp.link_item(0)


def test_import_gripper_tree_and_view(win):
    _import(win, "gripper_mimic.urdf")
    robot = win.project_controller.robot
    mp = win.model_panel
    assert robot.tcp_link == 1
    # active chain = hand_base -> left_finger (joint 0)
    assert mp.is_active_item(mp.link_item(0))
    assert mp.is_active_item(mp.link_item(1))
    assert mp.is_active_item(mp.joint_item(0))
    assert not mp.is_active_item(mp.link_item(2))
    assert not mp.is_active_item(mp.joint_item(1))
    assert "mimic" in mp.joint_item(1).text(0)
    assert "left_finger_joint" in mp.joint_item(1).text(0)

    rv = win.robot_view
    fk = cb.fk(robot)
    assert fk["active_joints"] == [0]
    seg = rv.branch_segments
    assert seg is not None and len(seg) == 1  # one joint off the active chain
    np.testing.assert_allclose(seg[0, 0], fk["frames"][0][:3, 3], atol=1e-9)
    np.testing.assert_allclose(seg[0, 1], fk["frames"][2][:3, 3], atol=1e-9)
    expected = np.array([fk["frames"][0][:3, 3], fk["frames"][1][:3, 3], fk["tcp"][:3, 3]])
    np.testing.assert_allclose(rv.polyline_points, expected, atol=1e-9)
    assert rv.branch_actor is not None
    assert len(rv.frame_actors) == 3


def test_plain_chain_has_no_branch_actor(win):
    _import(win, "planar_2r.urdf")
    assert win.robot_view.branch_actor is None
    assert win.robot_view.branch_segments is None
    robot = win.project_controller.robot
    fk = cb.fk(robot)
    expected = np.array([f[:3, 3] for f in fk["frames"]] + [fk["tcp"][:3, 3]])
    np.testing.assert_allclose(win.robot_view.polyline_points, expected, atol=1e-9)


def test_branch_segments_move_with_q(win):
    _import(win, "gripper_mimic.urdf")
    robot = win.project_controller.robot
    before = np.array(win.robot_view.branch_segments)
    robot.joints[0].q = 0.02
    win.joint_panel.q_changed.emit()
    after = win.robot_view.branch_segments
    assert not np.allclose(before, after)
    fk = cb.fk(robot)
    np.testing.assert_allclose(after[0, 1], fk["frames"][2][:3, 3], atol=1e-9)


def test_import_via_menu_action(win):
    with mock.patch("app.main_window.QFileDialog.getOpenFileName",
                    return_value=(str(FX / "planar_2r.urdf"), "")), \
         mock.patch("app.main_window.QMessageBox.warning"):
        win._action_import_urdf.trigger()
        win.task_runner.wait_all(5000)
        QApplication.processEvents()
    assert win.project_controller.has_project
    assert win.project_controller.robot.metadata.name == "planar_2r"
    assert win._menu_import.title().replace("&", "") == "Import"


def test_import_with_errors_keeps_old_project(win):
    with mock.patch("app.project.project_controller.QInputDialog.getText",
                    return_value=("Keep", True)):
        win.project_controller.new_project(win)
    proj = win.project_controller.current_project
    with mock.patch("app.main_window.QMessageBox.warning") as warn:
        win.import_urdf_file(str(FX / "bad_xml.urdf"))
        win.task_runner.wait_all(5000)
        QApplication.processEvents()
    assert warn.call_count == 0  # R3.6: report goes to the dock, not a popup
    assert [i.code for i in win.import_report_panel.report.by_severity("error")] == [
        "URDF_BAD_XML"
    ]
    assert win.project_controller.current_project is proj
    assert win.project_controller.current_project.name == "Keep"


def test_import_modified_cancel_keeps_old_project(win):
    with mock.patch("app.project.project_controller.QInputDialog.getText",
                    return_value=("Keep", True)):
        win.project_controller.new_project(win)
    proj = win.project_controller.current_project
    assert win.project_controller.is_modified
    with mock.patch("app.project.project_controller.QMessageBox.question",
                    return_value=QMessageBox.StandardButton.Cancel):
        _import(win, "planar_2r.urdf")
    assert win.project_controller.current_project is proj
    assert win.project_controller.robot.metadata.name != "planar_2r"


def test_import_modified_discard_replaces_project(win):
    with mock.patch("app.project.project_controller.QInputDialog.getText",
                    return_value=("Keep", True)):
        win.project_controller.new_project(win)
    with mock.patch("app.project.project_controller.QMessageBox.question",
                    return_value=QMessageBox.StandardButton.Discard):
        _import(win, "planar_2r.urdf")
    assert win.project_controller.current_project.name == "planar_2r"


# ----------------------------------------------------------------------
# Set TCP link
# ----------------------------------------------------------------------

def test_set_tcp_link_moves_tcp_and_changes_joint_rows(win):
    _import(win, "gripper_mimic.urdf")
    jp = win.joint_panel
    robot = win.project_controller.robot
    # tcp on left finger: chain row = left joint; right joint is mimic (no row)
    assert [r.joint_index for r in jp.rows] == [0]
    assert len(jp.branch_rows) == 0
    assert not jp.branch_group.isVisibleTo(win)
    tcp_before = np.array(win.robot_view.tcp_actor.user_matrix)

    names = [lk.name for lk in robot.links]
    with mock.patch("app.main_window.QInputDialog.getItem",
                    return_value=("right_finger", True)):
        win._action_set_tcp.trigger()
    assert robot.tcp_link == names.index("right_finger") == 2
    assert win.project_controller.is_modified
    tcp_after = np.array(win.robot_view.tcp_actor.user_matrix)
    np.testing.assert_allclose(tcp_after, cb.fk(robot)["tcp"], atol=1e-9)
    assert not np.allclose(tcp_before, tcp_after)
    # now active chain = right (mimic) joint -> no chain rows; left joint in Other branches
    assert jp.rows == []
    assert [r.joint_index for r in jp.branch_rows] == [0]
    assert jp.branch_group.isVisibleTo(win)
    # tree styling flipped
    assert win.model_panel.is_active_item(win.model_panel.link_item(2))
    assert not win.model_panel.is_active_item(win.model_panel.link_item(1))
    # branch segment now the left finger joint
    assert len(win.robot_view.branch_segments) == 1


def test_set_tcp_cancel_changes_nothing(win):
    _import(win, "gripper_mimic.urdf")
    with mock.patch("app.main_window.QInputDialog.getItem", return_value=("", False)):
        win._action_set_tcp.trigger()
    assert win.project_controller.robot.tcp_link == 1


def test_robot_menu_after_view(win):
    titles = [a.text().replace("&", "") for a in win.menuBar().actions()]
    assert titles.index("Robot") == titles.index("View") + 1


# ----------------------------------------------------------------------
# DH editor with imported / tree robots
# ----------------------------------------------------------------------

def test_dh_editor_origin_axis_rows_read_only(win):
    _import(win, "planar_2r.urdf")  # plain chain, all origin_axis
    ed = win.dh_editor
    robot = win.project_controller.robot
    assert is_plain_chain(robot)
    assert ed._table.rowCount() == 3
    from PySide6.QtCore import Qt
    from PySide6.QtWidgets import QComboBox
    for r in range(3):
        combo = ed._table.cellWidget(r, 1)
        assert isinstance(combo, QComboBox) and not combo.isEnabled()
        assert combo.toolTip() == "Imported joint: not DH"
        for c in range(7):
            it = ed._table.item(r, c)
            assert not (it.flags() & Qt.ItemFlag.ItemIsEditable)
            assert it.toolTip() == "Imported joint: not DH"
    before = _model_json(win)
    ed._table.item(0, COL_A).setText("9.0")
    ed._on_type_combo_changed(0, ed._table.cellWidget(0, 1))
    assert _model_json(win) == before


def test_dh_editor_refuses_add_remove_on_tree(win):
    _import(win, "gripper_mimic.urdf")
    ed = win.dh_editor
    assert not is_plain_chain(win.project_controller.robot)
    before = _model_json(win)
    ed._on_add_joint()
    assert _model_json(win) == before
    assert ed.status_text == TREE_READONLY_TEXT
    ed._table.selectRow(0)
    ed._on_remove_joint()
    assert _model_json(win) == before
    assert ed.status_text == TREE_READONLY_TEXT


def test_dh_editor_convention_combo_still_works_on_tree(win):
    _import(win, "gripper_mimic.urdf")
    ed = win.dh_editor
    ed._combo_convention.setCurrentIndex(1)
    assert win.project_controller.robot.convention == cb.DHConvention.Modified


def test_dh_rows_stay_editable_on_plain_dh_chain(win):
    with mock.patch("app.project.project_controller.QInputDialog.getText",
                    return_value=("Arm", True)):
        win.project_controller.new_project(win)
    win.project_controller.set_robot(_make_dh_arm(2))
    from PySide6.QtCore import Qt
    ed = win.dh_editor
    assert ed._table.item(0, COL_A).flags() & Qt.ItemFlag.ItemIsEditable
    assert ed._table.cellWidget(0, 1).isEnabled()
    ed._on_add_joint()
    assert len(win.project_controller.robot.joints) == 3
    assert ed.status_text == ""


# ----------------------------------------------------------------------
# JointPanel row counts
# ----------------------------------------------------------------------

def test_joint_panel_rows_chain_and_branches(win):
    _import(win, "serial_6dof.urdf")
    jp = win.joint_panel
    assert [r.joint_index for r in jp.rows] == [0, 1, 2, 3, 4, 5]
    assert jp.branch_rows == []
    assert not jp.branch_group.isVisibleTo(win)


def test_joint_panel_fixed_excluded_and_other_branch_rows(win):
    _import(win, "planar_2r.urdf")
    jp = win.joint_panel
    assert [r.joint_index for r in jp.rows] == [0, 1]  # fixed tip joint excluded

    # tree: fork at the base, TCP on branch 1 -> branch 2 joints appear below
    m = cb.RobotModel()
    for i in range(4):
        lk = cb.Link()
        lk.name = f"L{i}"
        m.links.append(lk)

    def oa(name, p, c):
        j = cb.Joint()
        j.name = name
        j.parent_link, j.child_link = p, c
        j.kinematics = cb.JointKinematics.OriginAxis
        j.axis = [0.0, 0.0, 1.0]
        o = cb.Pose()
        o.xyz = [0.1, 0.0, 0.0]
        j.origin = o
        j.limits = cb.default_limits(j.type)
        return j

    m.joints.append(oa("a", 0, 1))
    m.joints.append(oa("b", 0, 2))
    m.joints.append(oa("c", 2, 3))
    m.tcp_link = 1
    win.project_controller.set_robot(m)
    assert [r.joint_index for r in jp.rows] == [0]
    assert [r.joint_index for r in jp.branch_rows] == [1, 2]
    assert jp.branch_group.isVisibleTo(win)
    # collapsed by default
    assert not jp.branch_group.isChecked()
    # branch row edit moves the model, TCP readout (active chain) unchanged
    txt = jp.tcp_text
    jp.branch_rows[0].spinbox.setValue(30.0)
    assert abs(m.joints[1].q - np.radians(30.0)) < 1e-6
    assert jp.tcp_text == txt


# ----------------------------------------------------------------------
# Selection: marker highlight + properties
# ----------------------------------------------------------------------

def test_tree_selection_highlights_marker_and_fills_properties(win):
    _import(win, "serial_6dof.urdf")
    rv = win.robot_view
    mp = win.model_panel
    assert rv.marker_joint_indices == [0, 1, 2, 3, 4, 5]
    mp.select("joint", 2)
    scales = [tuple(a.scale) for a in rv.marker_actors]
    assert scales[2] == pytest.approx((1.6, 1.6, 1.6))
    assert all(s == pytest.approx((1.0, 1.0, 1.0)) for i, s in enumerate(scales) if i != 2)
    text = win.props_panel.text
    robot = win.project_controller.robot
    assert robot.joints[2].name in text
    assert "origin_axis" in text and "Axis" in text and "Origin xyz" in text
    assert robot.links[robot.joints[2].parent_link].name in text

    mp.select("joint", 4)
    scales = [tuple(a.scale) for a in rv.marker_actors]
    assert scales[4] == pytest.approx((1.6, 1.6, 1.6))
    assert scales[2] == pytest.approx((1.0, 1.0, 1.0))  # previous reset

    mp.select("link", 3)
    assert all(tuple(a.scale) == pytest.approx((1.0, 1.0, 1.0)) for a in rv.marker_actors)
    assert "Link:" in win.props_panel.text and "Mass" in win.props_panel.text


def test_properties_show_mimic(win):
    _import(win, "gripper_mimic.urdf")
    win.model_panel.select("joint", 1)
    assert "Mimic: left_finger_joint" in win.props_panel.text


def test_highlight_joint_none_resets(win):
    _import(win, "serial_6dof.urdf")
    rv = win.robot_view
    rv.highlight_joint(1)
    rv.highlight_joint(None)
    assert all(tuple(a.scale) == pytest.approx((1.0, 1.0, 1.0)) for a in rv.marker_actors)


def test_empty_model_header(win):
    assert win.model_panel.tree.headerItem().text(0) == "No robot loaded"
    _import(win, "planar_2r.urdf")
    with mock.patch("app.project.project_controller.QMessageBox.question",
                    return_value=QMessageBox.StandardButton.Discard):
        win.project_controller.close_project(win)
    assert win.model_panel.tree.headerItem().text(0) == "No robot loaded"
    assert win.model_panel.link_item_count == 0


def test_saved_project_with_imported_robot_roundtrips(win, tmp_path):
    _import(win, "gripper_mimic.urdf")
    path = str(tmp_path / "grip.robotproj")
    assert win.project_controller.save_project_as(win, path)
    win.project_controller.close_project(win)
    assert win.project_controller.open_project(win, path)
    assert win.model_panel.link_item_count == 3
    assert win.robot_view.branch_segments is not None


def test_model_tree_indent_and_no_elide(qapp):
    from PySide6.QtCore import Qt
    from app.panels.model_panel import ModelPanel
    p = ModelPanel()
    assert p.tree.indentation() == 12
    assert p.tree.textElideMode() == Qt.TextElideMode.ElideNone
