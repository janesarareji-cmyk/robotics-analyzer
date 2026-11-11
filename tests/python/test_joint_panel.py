"""
tests/python/test_joint_panel.py
R2.8: Unit tests for JointPanel, live TCP pose readout,
slider/spinbox sync, rotation_to_rpy, and MainWindow integration.
"""

import math
import os
import random
import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import core_bridge as cb

from app.main_window import MainWindow
from app.panels.joint_panel import JointPanel, rotation_to_rpy, SLIDER_SCALE
from app.project.project_data import ProjectData


def make_test_arm(n_revolute: int = 3, add_fixed: bool = False, add_prismatic: bool = False) -> cb.RobotModel:
    """Helper: create robot model with specified joint types."""
    robot = cb.RobotModel()
    robot.metadata.name = "TestArm"
    robot.convention = cb.DHConvention.Standard

    for i in range(n_revolute + int(add_fixed) + int(add_prismatic) + 1):
        lk = cb.Link()
        lk.name = f"link{i}"
        robot.links.append(lk)

    for i in range(n_revolute):
        j = cb.Joint()
        j.name = f"rev{i + 1}"
        j.type = cb.JointType.Revolute
        j.dh.a = 0.5 * (i + 1)
        j.dh.alpha = 0.0
        j.dh.d = 0.0
        j.dh.theta = 0.0
        j.q = 0.0
        j.limits = cb.default_limits(j.type)
        robot.joints.append(j)

    if add_fixed:
        jf = cb.Joint()
        jf.name = "fixed_joint"
        jf.type = cb.JointType.Fixed
        jf.dh.a = 0.2
        jf.dh.alpha = 0.0
        jf.dh.d = 0.0
        jf.dh.theta = 0.0
        jf.q = 0.0
        jf.limits = cb.default_limits(jf.type)
        robot.joints.append(jf)

    if add_prismatic:
        jp = cb.Joint()
        jp.name = "prism_joint"
        jp.type = cb.JointType.Prismatic
        jp.dh.a = 0.0
        jp.dh.alpha = 0.0
        jp.dh.d = 0.1
        jp.dh.theta = 0.0
        jp.q = 0.0
        jp.limits = cb.default_limits(jp.type)
        robot.joints.append(jp)

    return robot


def test_row_counts_and_units_fixed_and_prismatic(qapp):
    """3R model has 3 rows; fixed joint adds no row; prismatic row uses mm."""
    robot = make_test_arm(n_revolute=3, add_fixed=True, add_prismatic=True)
    panel = JointPanel()
    panel.set_model(robot)

    # 3 revolute + 1 prismatic = 4 active rows (fixed joint excluded)
    assert len(panel.rows) == 4

    # Revolute rows use degrees
    for r in panel.rows[:3]:
        assert r.joint_type == cb.JointType.Revolute
        assert r.unit == "deg"
        assert r.unit_label.text() == "deg"

    # Prismatic row uses millimeters
    prism_row = panel.rows[3]
    assert prism_row.joint_type == cb.JointType.Prismatic
    assert prism_row.unit == "mm"
    assert prism_row.unit_label.text() == "mm"


def test_spinbox_and_slider_sync_and_model_q(qapp):
    """Setting spinbox deg value changes model.joints[i].q in rad; slider and spinbox stay in sync."""
    robot = make_test_arm(n_revolute=1)
    panel = JointPanel()
    panel.set_model(robot)

    row = panel.rows[0]

    # Set spinbox to 45 degrees
    row.spinbox.setValue(45.0)

    assert robot.joints[0].q == pytest.approx(math.radians(45.0), rel=1e-5)
    assert row.slider.value() == int(round(45.0 * SLIDER_SCALE))

    # Drag slider to -30 degrees
    slider_target = int(round(-30.0 * SLIDER_SCALE))
    row.slider.setValue(slider_target)

    assert row.spinbox.value() == pytest.approx(-30.0, abs=0.01)
    assert robot.joints[0].q == pytest.approx(math.radians(-30.0), abs=1e-4)


def test_slider_change_updates_robot_view_polyline(qapp):
    """Slider change moves RobotView polyline points to match cb.fk(model) origins."""
    win = MainWindow()
    try:
        proj = ProjectData.new("PolylineSyncTest")
        robot = make_test_arm(n_revolute=3)
        proj.robot_model = robot
        win.project_controller._set_project(proj, modified=False)

        # Move slider on joint 1 to 60 deg
        row0 = win.joint_panel.rows[0]
        row0.spinbox.setValue(60.0)

        expected_fk = cb.fk(win.project_controller.robot)
        expected_pts = np.array(
            [f[:3, 3] for f in expected_fk["frames"]] + [expected_fk["tcp"][:3, 3]],
            dtype=float,
        )

        np.testing.assert_allclose(win.robot_view.polyline_points, expected_pts, atol=1e-9)
    finally:
        win.close()


def test_tcp_readout_2r_planar(qapp):
    """TCP readout of 2R planar at q=(90°,-90°) -> x=1000mm, y=1000mm, z=0, yaw=0 (a1=a2=1 m)."""
    robot = cb.RobotModel()
    for i in range(3):
        lk = cb.Link()
        robot.links.append(lk)
    for i in range(2):
        j = cb.Joint()
        j.name = f"j{i + 1}"
        j.type = cb.JointType.Revolute
        j.dh.a = 1.0
        robot.joints.append(j)

    robot.joints[0].q = math.pi / 2.0
    robot.joints[1].q = -math.pi / 2.0

    panel = JointPanel()
    panel.set_model(robot)

    fk_res = cb.fk(robot)
    panel.update_tcp_readout(fk_res["tcp"])

    text = panel.tcp_text
    assert "X: 1000.00 mm" in text
    assert "Y: 1000.00 mm" in text
    assert "Z: 0.00 mm" in text
    assert "Yaw: 0.00°" in text


def test_rotation_to_rpy_round_trip_and_gimbal_lock():
    """rotation_to_rpy round-trip against pose_to_iso for 50 random RPY; gimbal-lock does not return NaN."""
    random.seed(42)
    for _ in range(50):
        r_orig = random.uniform(-math.pi, math.pi)
        p_orig = random.uniform(-math.radians(80), math.radians(80))
        y_orig = random.uniform(-math.pi, math.pi)

        pose = cb.Pose()
        pose.rpy = [r_orig, p_orig, y_orig]
        mat = cb.pose_to_iso(pose)

        r_calc, p_calc, y_calc = rotation_to_rpy(mat[:3, :3])

        pose2 = cb.Pose()
        pose2.rpy = [r_calc, p_calc, y_calc]
        mat2 = cb.pose_to_iso(pose2)

        np.testing.assert_allclose(mat[:3, :3], mat2[:3, :3], atol=1e-9)

    # Gimbal lock at pitch = +90° and -90°
    for pitch_gl in [math.pi / 2.0, -math.pi / 2.0]:
        pose_gl = cb.Pose()
        pose_gl.rpy = [0.0, pitch_gl, 0.7]
        mat_gl = cb.pose_to_iso(pose_gl)

        r_gl, p_gl, y_gl = rotation_to_rpy(mat_gl[:3, :3])
        assert not math.isnan(r_gl)
        assert not math.isnan(p_gl)
        assert not math.isnan(y_gl)


def test_out_of_limit_q_shows_warning_and_does_not_modify_model(qapp):
    """Out-of-limit q: model unchanged, warning shown on row."""
    robot = make_test_arm(n_revolute=1)
    # Set q outside limits [-pi, pi]
    robot.joints[0].q = 10.0

    panel = JointPanel()
    panel.set_model(robot)
    panel.show()

    # Model value must not be silently overwritten
    assert robot.joints[0].q == pytest.approx(10.0)

    row = panel.rows[0]
    assert not row.warning_label.isHidden()

    # Spinbox shows clamped upper limit
    upper_deg = math.degrees(math.pi)
    assert row.spinbox.value() == pytest.approx(upper_deg, rel=1e-3)

    # Bring inside limits -> warning hides
    row.spinbox.setValue(0.0)
    assert row.warning_label.isHidden()
    panel.close()


def test_zero_all_and_reset(qapp):
    """Zero all and Reset behave as specified."""
    robot = make_test_arm(n_revolute=2)
    robot.joints[0].q = 0.5
    robot.joints[1].q = -0.7

    panel = JointPanel()
    panel.set_model(robot, save_baseline=True)

    # Move joints away from baseline
    panel.rows[0].spinbox.setValue(45.0)
    panel.rows[1].spinbox.setValue(30.0)
    assert abs(robot.joints[0].q - 0.5) > 0.1

    # Zero all
    panel._on_zero_all()
    assert robot.joints[0].q == pytest.approx(0.0)
    assert robot.joints[1].q == pytest.approx(0.0)

    # Reset
    panel._on_reset()
    assert robot.joints[0].q == pytest.approx(0.5)
    assert robot.joints[1].q == pytest.approx(-0.7)


def test_dh_editor_edits_rebuild_joint_panel(qapp):
    """DH editor type change/joint add triggers JointPanel rebuild with correct row count."""
    win = MainWindow()
    try:
        proj = ProjectData.new("RebuildTest")
        robot = make_test_arm(n_revolute=3)
        proj.robot_model = robot
        win.project_controller._set_project(proj, modified=False)

        assert len(win.joint_panel.rows) == 3

        # Add joint in DH editor
        win.dh_editor._on_add_joint()
        assert len(win.joint_panel.rows) == 4

        # Change joint type to Fixed in DH editor
        win.dh_editor.set_joint_type(0, cb.JointType.Fixed)
        assert len(win.joint_panel.rows) == 3
    finally:
        win.close()


def test_q_edit_sets_dirty_flag(qapp):
    """Editing joint value via spinbox sets controller dirty flag."""
    win = MainWindow()
    try:
        proj = ProjectData.new("DirtyFlagTest")
        robot = make_test_arm(n_revolute=2)
        proj.robot_model = robot
        win.project_controller._set_project(proj, modified=False)

        assert not win.project_controller.is_modified

        win.joint_panel.rows[0].spinbox.setValue(20.0)

        assert win.project_controller.is_modified
    finally:
        win.close()


def test_slider_drag_does_not_rebuild_dh_table_widgets(qapp):
    """Moving slider 50 times does not rebuild DH table widgets; q cell updates."""
    from app.panels.dh_editor import COL_TYPE, COL_Q
    win = MainWindow()
    try:
        proj = ProjectData.new("NoRebuildTest")
        robot = make_test_arm(n_revolute=3)
        proj.robot_model = robot
        win.project_controller._set_project(proj, modified=False)

        table = win.dh_editor._table
        type_widget_before = table.cellWidget(0, COL_TYPE)
        q_item_before = table.item(0, COL_Q)
        assert type_widget_before is not None
        assert q_item_before is not None

        slider = win.joint_panel.rows[0].slider
        for i in range(1, 51):
            slider.setValue(i * 10)

        # Widgets must be identical instances (no table rebuild)
        assert table.cellWidget(0, COL_TYPE) is type_widget_before
        assert table.item(0, COL_Q) is q_item_before

        # Q cell must show the new value
        expected_rad = win.project_controller.robot.joints[0].q
        assert float(q_item_before.text()) == pytest.approx(expected_rad, abs=1e-5)
    finally:
        win.close()

