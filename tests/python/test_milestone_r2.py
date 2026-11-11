"""
tests/python/test_milestone_r2.py
R2.9: 3-DOF end-to-end milestone test for Stage R2.

Validates the full R2 stack:
1. Building 3-DOF arm through DH editor widget UI.
2. Validation, frame triads, TCP, polyline.
3. Live FK with independent NumPy comparison and TCP readout verification.
4. Viewport polyline points vs independent NumPy frame origins.
5. Project save, close, open round-trip.
6. Modified DH (Craig) equivalence.
7. Link length edit and visual scale rebuild thresholds.
8. Out-of-limit q handling and validation reporting.
9. Zero all and Reset buttons.
10. Corrupt project load error handling and recovery.
Plus a 200-sample randomized numerical equivalence check for both conventions.
"""

import math
import unittest.mock
import zipfile
import numpy as np
import pytest

from PySide6.QtWidgets import QApplication

import core_bridge as cb
from app.main_window import MainWindow
from app.panels.dh_editor import (
    COL_A, COL_ALPHA, COL_D, COL_THETA, COL_Q
)
from app.panels.joint_panel import rotation_to_rpy


# ----------------------------------------------------------------------
# Independent NumPy Kinematics (Zero dependency on core_bridge kinematics)
# ----------------------------------------------------------------------

def independent_dh_standard(a: float, alpha: float, d: float, theta: float) -> np.ndarray:
    """Standard DH matrix: Rz(theta) * Tz(d) * Tx(a) * Rx(alpha)."""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0.0,      sa,       ca,      d],
        [0.0,     0.0,      0.0,    1.0],
    ], dtype=float)


def independent_dh_modified(a: float, alpha: float, d: float, theta: float) -> np.ndarray:
    """Modified DH (Craig) matrix: Rx(alpha) * Tx(a) * Rz(theta) * Tz(d)."""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [      ct,      -st,  0.0,        a],
        [ st * ca,  ct * ca,  -sa, -d * sa],
        [ st * sa,  ct * sa,   ca,  d * ca],
        [     0.0,      0.0,  0.0,      1.0],
    ], dtype=float)


def independent_pose_to_matrix(xyz: list[float], rpy: list[float]) -> np.ndarray:
    """Rigid transform matrix: Translation * Rz(yaw) * Ry(pitch) * Rx(roll)."""
    x, y, z = xyz
    r, p, yaw = rpy
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(yaw), np.sin(yaw)

    Rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]], dtype=float)
    Ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]], dtype=float)
    Rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]], dtype=float)
    R = Rz @ Ry @ Rx

    T = np.eye(4, dtype=float)
    T[:3, :3] = R
    T[:3, 3] = [x, y, z]
    return T


# ----------------------------------------------------------------------
# 200-sample Randomized Numerical Check
# ----------------------------------------------------------------------

def test_fk_200_random_q_vs_independent_numpy():
    """Verify core_bridge.fk vs independent NumPy implementation over 200 random q samples."""
    rng = np.random.default_rng(42)

    # 1. Standard DH model
    r_sdh = cb.RobotModel()
    r_sdh.convention = cb.DHConvention.Standard
    r_sdh.tool.xyz = [0.1, 0.0, 0.0]
    for i in range(4):
        lk = cb.Link()
        lk.name = f"link_{i}"
        r_sdh.links.append(lk)

    j1 = cb.Joint()
    j1.name = "j1"
    j1.dh.a = 0.0
    j1.dh.alpha = math.pi / 2.0
    j1.dh.d = 0.3

    j2 = cb.Joint()
    j2.name = "j2"
    j2.dh.a = 0.5
    j2.dh.alpha = 0.0
    j2.dh.d = 0.0

    j3 = cb.Joint()
    j3.name = "j3"
    j3.dh.a = 0.4
    j3.dh.alpha = 0.0
    j3.dh.d = 0.0

    for j in [j1, j2, j3]:
        r_sdh.joints.append(j)

    for _ in range(200):
        q = rng.uniform(-np.pi, np.pi, 3)
        T_base = independent_pose_to_matrix(r_sdh.base.xyz, r_sdh.base.rpy)
        A1 = independent_dh_standard(0.0, math.pi / 2.0, 0.3, q[0])
        A2 = independent_dh_standard(0.5, 0.0, 0.0, q[1])
        A3 = independent_dh_standard(0.4, 0.0, 0.0, q[2])
        T_tool = independent_pose_to_matrix(r_sdh.tool.xyz, r_sdh.tool.rpy)
        T_tcp_expected = T_base @ A1 @ A2 @ A3 @ T_tool

        fk_res = cb.fk(r_sdh, list(q))
        np.testing.assert_allclose(fk_res["tcp"], T_tcp_expected, atol=1e-9)

    # 2. Modified DH model
    r_mdh = cb.RobotModel()
    r_mdh.convention = cb.DHConvention.Modified
    r_mdh.tool.xyz = [0.5, 0.0, 0.0]
    for i in range(4):
        lk = cb.Link()
        lk.name = f"link_{i}"
        r_mdh.links.append(lk)

    m1 = cb.Joint()
    m1.name = "m1"
    m1.mdh.a = 0.0
    m1.mdh.alpha = 0.0
    m1.mdh.d = 0.3

    m2 = cb.Joint()
    m2.name = "m2"
    m2.mdh.a = 0.0
    m2.mdh.alpha = math.pi / 2.0
    m2.mdh.d = 0.0

    m3 = cb.Joint()
    m3.name = "m3"
    m3.mdh.a = 0.5
    m3.mdh.alpha = 0.0
    m3.mdh.d = 0.0

    for m in [m1, m2, m3]:
        r_mdh.joints.append(m)

    for _ in range(200):
        q = rng.uniform(-np.pi, np.pi, 3)
        T_base = independent_pose_to_matrix(r_mdh.base.xyz, r_mdh.base.rpy)
        M1 = independent_dh_modified(0.0, 0.0, 0.3, q[0])
        M2 = independent_dh_modified(0.0, math.pi / 2.0, 0.0, q[1])
        M3 = independent_dh_modified(0.5, 0.0, 0.0, q[2])
        T_tool = independent_pose_to_matrix(r_mdh.tool.xyz, r_mdh.tool.rpy)
        T_tcp_expected = T_base @ M1 @ M2 @ M3 @ T_tool

        fk_res = cb.fk(r_mdh, list(q))
        np.testing.assert_allclose(fk_res["tcp"], T_tcp_expected, atol=1e-9)


# ----------------------------------------------------------------------
# End-to-End Milestone Flow Test
# ----------------------------------------------------------------------

def test_milestone_r2_end_to_end(qapp, tmp_path):
    """
    Stage R2 Milestone: Comprehensive 10-step end-to-end integration test.
    Executes in a single continuous workflow on MainWindow without mocking core_bridge.
    """
    win = MainWindow()
    win.show()

    try:
        # --------------------------------------------------------------
        # Step 1: New project -> build 3-DOF arm through DH editor widget
        # --------------------------------------------------------------
        with unittest.mock.patch(
            "app.project.project_controller.QInputDialog.getText",
            return_value=("MilestoneArm", True),
        ):
            opened = win.project_controller.new_project(win)
        assert opened
        assert win.project_controller.has_project

        dh_ed = win.dh_editor
        # Convention: Standard DH
        dh_ed._combo_convention.setCurrentIndex(0)

        # Tool pose: x=0.1
        dh_ed._tool_x.setValue(0.1)

        # Add 3 joints through DH editor UI
        dh_ed._on_add_joint()
        dh_ed._on_add_joint()
        dh_ed._on_add_joint()

        # Row 0 (Joint 1): a=0, alpha=pi/2, d=0.3, theta=0
        dh_ed._table.item(0, COL_A).setText("0.0")
        dh_ed._table.item(0, COL_ALPHA).setText(str(math.pi / 2.0))
        dh_ed._table.item(0, COL_D).setText("0.3")
        dh_ed._table.item(0, COL_THETA).setText("0.0")

        # Row 1 (Joint 2): a=0.5, alpha=0, d=0, theta=0
        dh_ed._table.item(1, COL_A).setText("0.5")
        dh_ed._table.item(1, COL_ALPHA).setText("0.0")
        dh_ed._table.item(1, COL_D).setText("0.0")
        dh_ed._table.item(1, COL_THETA).setText("0.0")

        # Row 2 (Joint 3): a=0.4, alpha=0, d=0, theta=0
        dh_ed._table.item(2, COL_A).setText("0.4")
        dh_ed._table.item(2, COL_ALPHA).setText("0.0")
        dh_ed._table.item(2, COL_D).setText("0.0")
        dh_ed._table.item(2, COL_THETA).setText("0.0")

        robot = win.project_controller.robot

        # --------------------------------------------------------------
        # Step 2: Validate returns empty list. Viewport has 4 triads + TCP + 5-pt polyline
        # --------------------------------------------------------------
        errors = robot.validate()
        assert errors == []
        assert len(robot.joints) == 3
        assert len(robot.links) == 4

        # 4 frame triads (base + 3 joints) + 1 TCP triad
        assert len(win.robot_view._frame_actors) == 4
        assert win.robot_view._tcp_actor is not None
        assert win.robot_view.polyline_points is not None
        assert len(win.robot_view.polyline_points) == 5

        # --------------------------------------------------------------
        # Step 3: Set q via JointPanel spinboxes to (30°, -45°, 60°)
        # --------------------------------------------------------------
        panel = win.joint_panel
        assert len(panel.rows) == 3

        panel.rows[0].spinbox.setValue(30.0)
        panel.rows[1].spinbox.setValue(-45.0)
        panel.rows[2].spinbox.setValue(60.0)

        q1 = math.radians(30.0)
        q2 = math.radians(-45.0)
        q3 = math.radians(60.0)

        # Independent NumPy calculation
        T_base = independent_pose_to_matrix(robot.base.xyz, robot.base.rpy)
        A1 = independent_dh_standard(0.0, math.pi / 2.0, 0.3, q1)
        A2 = independent_dh_standard(0.5, 0.0, 0.0, q2)
        A3 = independent_dh_standard(0.4, 0.0, 0.0, q3)
        T_tool = independent_pose_to_matrix(robot.tool.xyz, robot.tool.rpy)

        F0 = T_base
        F1 = F0 @ A1
        F2 = F1 @ A2
        F3 = F2 @ A3
        T_tcp_expected = F3 @ T_tool

        fk_actual = cb.fk(robot)
        np.testing.assert_allclose(fk_actual["tcp"], T_tcp_expected, atol=1e-9)

        # Check TCP readout text matches expected values to 2 decimals
        exp_x_mm = T_tcp_expected[0, 3] * 1000.0
        exp_y_mm = T_tcp_expected[1, 3] * 1000.0
        exp_z_mm = T_tcp_expected[2, 3] * 1000.0
        exp_r, exp_p, exp_yaw = rotation_to_rpy(T_tcp_expected[:3, :3])
        exp_r_deg = math.degrees(exp_r)
        exp_p_deg = math.degrees(exp_p)
        exp_yaw_deg = math.degrees(exp_yaw)

        tcp_text = panel.tcp_text
        assert f"X: {exp_x_mm:.2f} mm" in tcp_text
        assert f"Y: {exp_y_mm:.2f} mm" in tcp_text
        assert f"Z: {exp_z_mm:.2f} mm" in tcp_text
        assert f"Roll: {exp_r_deg:.2f}°" in tcp_text
        assert f"Pitch: {exp_p_deg:.2f}°" in tcp_text
        assert f"Yaw: {exp_yaw_deg:.2f}°" in tcp_text

        # --------------------------------------------------------------
        # Step 4: Viewport polyline points equal numpy-computed frame origins
        # --------------------------------------------------------------
        expected_origins = np.array([
            F0[:3, 3],
            F1[:3, 3],
            F2[:3, 3],
            F3[:3, 3],
            T_tcp_expected[:3, 3],
        ], dtype=float)

        np.testing.assert_allclose(win.robot_view.polyline_points, expected_origins, atol=1e-9)

        # --------------------------------------------------------------
        # Step 5: Save project to temp .robotproj, close, open again
        # --------------------------------------------------------------
        save_file = str(tmp_path / "test_arm.robotproj")
        saved = win.project_controller.save_project_as(win, save_file)
        assert saved

        # Close project
        win.project_controller.close_project(win)
        assert not win.project_controller.has_project
        assert win.dh_editor.get_model() is None
        assert len(win.joint_panel.rows) == 0
        assert win.robot_view.polyline_points is None

        # Reopen project
        reopened = win.project_controller.open_project(win, save_file)
        assert reopened
        assert win.project_controller.has_project

        reopened_robot = win.project_controller.robot
        assert reopened_robot.convention == cb.DHConvention.Standard
        assert len(reopened_robot.joints) == 3
        assert win.joint_panel.rows[0].spinbox.value() == pytest.approx(30.0, abs=0.01)
        assert win.joint_panel.rows[1].spinbox.value() == pytest.approx(-45.0, abs=0.01)
        assert win.joint_panel.rows[2].spinbox.value() == pytest.approx(60.0, abs=0.01)
        assert win.joint_panel.tcp_text == tcp_text
        np.testing.assert_allclose(win.robot_view.polyline_points, expected_origins, atol=1e-9)

        # --------------------------------------------------------------
        # Step 6: Switch convention to Modified with MDH-equivalent arm -> same TCP
        # --------------------------------------------------------------
        # For the same physical arm in MDH:
        # m1: a=0, alpha=0, d=0.3
        # m2: a=0, alpha=pi/2, d=0
        # m3: a=0.5, alpha=0, d=0
        # tool: xyz=(0.5, 0, 0) [0.4 + 0.1 = 0.5]
        dh_ed = win.dh_editor
        dh_ed._combo_convention.setCurrentIndex(1)  # Modified
        dh_ed._tool_x.setValue(0.5)

        dh_ed._table.item(0, COL_A).setText("0.0")
        dh_ed._table.item(0, COL_ALPHA).setText("0.0")
        dh_ed._table.item(0, COL_D).setText("0.3")

        dh_ed._table.item(1, COL_A).setText("0.0")
        dh_ed._table.item(1, COL_ALPHA).setText(str(math.pi / 2.0))
        dh_ed._table.item(1, COL_D).setText("0.0")

        dh_ed._table.item(2, COL_A).setText("0.5")
        dh_ed._table.item(2, COL_ALPHA).setText("0.0")
        dh_ed._table.item(2, COL_D).setText("0.0")

        # TCP must match the exact same TCP matrix as before
        fk_mdh = cb.fk(win.project_controller.robot)
        np.testing.assert_allclose(fk_mdh["tcp"], T_tcp_expected, atol=1e-9)

        # --------------------------------------------------------------
        # Step 7: Edit link a from 0.5 to 0.7 -> scale rebuild occurs only if > 1%
        # --------------------------------------------------------------
        scale_before = win.robot_view.triad_length
        actors_before = list(win.robot_view.actors)

        # Significant edit (>1% reach change): a3 from 0.5 to 0.7 (reach 0.8 -> 1.0, 25% change)
        dh_ed._table.item(2, COL_A).setText("0.7")
        scale_after = win.robot_view.triad_length
        actors_after = list(win.robot_view.actors)

        assert abs(scale_after - scale_before) / scale_before > 0.01
        assert actors_after != actors_before

        # Minor edit (<= 1% reach change): a3 from 0.7 to 0.701 (0.1% change)
        scale_mid = win.robot_view.triad_length
        actors_mid = list(win.robot_view.actors)

        dh_ed._table.item(2, COL_A).setText("0.701")
        assert win.robot_view.triad_length == pytest.approx(scale_mid, rel=1e-5)
        # Actors list and instances remain identical (update in place)
        assert win.robot_view.actors == actors_mid

        # Restore a3 back to 0.5 for subsequent steps
        dh_ed._table.item(2, COL_A).setText("0.5")

        # --------------------------------------------------------------
        # Step 8: Out of limits q via editor (shrink limits)
        # --------------------------------------------------------------
        # Current q on joint 0 is 30° (~0.5236 rad). Shrink limits to upper=0.2 rad
        lim = win.project_controller.robot.joints[0].limits
        lim.upper = 0.2
        win.project_controller.robot.joints[0].limits = lim
        dh_ed._validate_and_notify()

        # Warning shown on row 0
        assert not win.joint_panel.rows[0].warning_label.isHidden()
        # Model q is not silently changed
        assert win.project_controller.robot.joints[0].q == pytest.approx(math.radians(30.0), abs=1e-4)

        # Validation reports the violation
        errs = win.project_controller.robot.validate()
        assert any("outside limits" in e for e in errs)

        # Project save still succeeds
        assert win.project_controller.save_project(win)

        # Restore valid limits
        lim.upper = math.pi
        win.project_controller.robot.joints[0].limits = lim
        dh_ed._validate_and_notify()
        assert win.joint_panel.rows[0].warning_label.isHidden()

        # --------------------------------------------------------------
        # Step 9: Zero all -> TCP at q=0; Reset -> returns to earlier q
        # --------------------------------------------------------------
        # Save baseline at current q (30°, -45°, 60°)
        win.joint_panel.save_baseline()

        # Click Zero all
        win.joint_panel._btn_zero.click()
        assert win.joint_panel.rows[0].spinbox.value() == pytest.approx(0.0, abs=1e-3)
        assert win.joint_panel.rows[1].spinbox.value() == pytest.approx(0.0, abs=1e-3)
        assert win.joint_panel.rows[2].spinbox.value() == pytest.approx(0.0, abs=1e-3)

        # TCP at q=0 matches numpy computation
        M1_zero = independent_dh_modified(0.0, 0.0, 0.3, 0.0)
        M2_zero = independent_dh_modified(0.0, math.pi / 2.0, 0.0, 0.0)
        M3_zero = independent_dh_modified(0.5, 0.0, 0.0, 0.0)
        T_zero_expected = T_base @ M1_zero @ M2_zero @ M3_zero @ independent_pose_to_matrix(
            win.project_controller.robot.tool.xyz, win.project_controller.robot.tool.rpy
        )
        np.testing.assert_allclose(cb.fk(win.project_controller.robot)["tcp"], T_zero_expected, atol=1e-9)

        # Click Reset -> restores earlier q
        win.joint_panel._btn_reset.click()
        assert win.joint_panel.rows[0].spinbox.value() == pytest.approx(30.0, abs=0.01)
        assert win.joint_panel.rows[1].spinbox.value() == pytest.approx(-45.0, abs=0.01)
        assert win.joint_panel.rows[2].spinbox.value() == pytest.approx(60.0, abs=0.01)

        # --------------------------------------------------------------
        # Step 10: Corrupt robot_model.json in zip -> open fails cleanly
        # --------------------------------------------------------------
        corrupt_zip = str(tmp_path / "corrupt_arm.robotproj")
        with zipfile.ZipFile(save_file, "r") as zin:
            with zipfile.ZipFile(corrupt_zip, "w") as zout:
                for item in zin.infolist():
                    data = zin.read(item.filename)
                    if item.filename == "robot_model.json":
                        data = b"{ this is corrupt json !!!"
                    zout.writestr(item, data)

        with unittest.mock.patch(
            "app.project.project_controller.QMessageBox.critical"
        ) as mock_crit, unittest.mock.patch.object(
            win.project_controller, "_confirm_discard", return_value=True
        ), unittest.mock.patch(
            "app.project.project_controller.QInputDialog.getText",
            return_value=("RecoveredArm", True),
        ):
            res = win.project_controller.open_project(win, corrupt_zip)
            assert not res
            mock_crit.assert_called_once()

            # Window remains active and responsive to new projects
            recovered = win.project_controller.new_project(win)
            assert recovered
            assert win.project_controller.current_project.name == "RecoveredArm"
            assert win.isVisible()

    finally:
        win.close()
