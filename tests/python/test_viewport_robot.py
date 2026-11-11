"""
tests/python/test_viewport_robot.py
R2.7: Tests for 3D robot visualization in the PyVista viewport.
Validates RobotView frame triads, polyline links, joint markers, TCP,
in-place updates, visibility toggles, and MainWindow integration.
"""

from unittest.mock import patch
import os
import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pyvista as pv
import core_bridge as cb

from app.main_window import MainWindow
from app.project.project_data import ProjectData
from app.viewport.robot_view import RobotView


def make_3r_planar_robot() -> cb.RobotModel:
    """Helper: create 3R planar robot arm with known dimensions."""
    robot = cb.RobotModel()
    robot.metadata.name = "Planar3R"
    robot.convention = cb.DHConvention.Standard

    # Base link + 3 links = 4 links
    for i in range(4):
        lk = cb.Link()
        lk.name = f"link{i}"
        robot.links.append(lk)

    # 3 revolute joints with a = [1.0, 0.8, 0.5]
    lengths = [1.0, 0.8, 0.5]
    for i in range(3):
        j = cb.Joint()
        j.name = f"j{i + 1}"
        j.type = cb.JointType.Revolute
        j.dh.a = lengths[i]
        j.dh.alpha = 0.0
        j.dh.d = 0.0
        j.dh.theta = 0.0
        j.q = 0.0
        j.limits = cb.default_limits(j.type)
        robot.joints.append(j)

    return robot


def test_3r_planar_model_actors_and_polyline():
    """3R planar model: actor count equals expected; polyline points == frame origins + TCP."""
    plotter = pv.Plotter(off_screen=True)
    try:
        rv = RobotView(plotter)
        robot = make_3r_planar_robot()
        fk_res = cb.fk(robot)

        rv.update(robot, fk_res)

        # Expected actors: 4 frame triads + 1 TCP triad + 1 polyline + 3 revolute markers = 9
        assert len(rv.actors) == 9
        assert len(rv.frame_actors) == 4
        assert rv.tcp_actor is not None
        assert rv.polyline_actor is not None
        assert len(rv.marker_actors) == 3

        # Polyline points: [frame0_xyz, frame1_xyz, frame2_xyz, frame3_xyz, tcp_xyz]
        expected_pts = np.array(
            [f[:3, 3] for f in fk_res["frames"]] + [fk_res["tcp"][:3, 3]],
            dtype=float,
        )
        np.testing.assert_allclose(rv.polyline_points, expected_pts, atol=1e-9)
    finally:
        plotter.close()


def test_update_in_place_new_q_moves_polyline():
    """update() with new q moves polyline points to new FK result, actor count unchanged."""
    plotter = pv.Plotter(off_screen=True)
    try:
        rv = RobotView(plotter)
        robot = make_3r_planar_robot()
        fk_res1 = cb.fk(robot)
        rv.update(robot, fk_res1)

        initial_count = len(rv.actors)
        pts1 = np.copy(rv.polyline_points)

        # New joint angles
        robot.joints[0].q = 0.4
        robot.joints[1].q = -0.5
        robot.joints[2].q = 0.6
        fk_res2 = cb.fk(robot)

        rv.update(robot, fk_res2)

        # Actor count unchanged
        assert len(rv.actors) == initial_count

        expected_pts2 = np.array(
            [f[:3, 3] for f in fk_res2["frames"]] + [fk_res2["tcp"][:3, 3]],
            dtype=float,
        )
        np.testing.assert_allclose(rv.polyline_points, expected_pts2, atol=1e-9)
        assert not np.allclose(rv.polyline_points, pts1)
    finally:
        plotter.close()


def test_empty_model_draws_base_and_tcp_only():
    """Empty model (0 joints) draws base + TCP only, no exception."""
    plotter = pv.Plotter(off_screen=True)
    try:
        rv = RobotView(plotter)
        empty_robot = cb.RobotModel()
        fk_res = cb.fk(empty_robot)

        rv.update(empty_robot, fk_res)

        # Only base triad (frames[0]) and TCP triad
        assert len(rv.actors) == 2
        assert len(rv.frame_actors) == 1
        assert rv.tcp_actor is not None
        assert rv.polyline_actor is None
        assert len(rv.marker_actors) == 0
    finally:
        plotter.close()


def test_visibility_toggles():
    """Toggles hide/show each group and asserts actor visibility."""
    plotter = pv.Plotter(off_screen=True)
    try:
        rv = RobotView(plotter)
        robot = make_3r_planar_robot()
        fk_res = cb.fk(robot)
        rv.update(robot, fk_res)

        # 1. Toggle Frames
        rv.set_show_frames(False)
        assert not rv.show_frames
        for a in rv.frame_actors:
            assert a.GetVisibility() == 0

        rv.set_show_frames(True)
        assert rv.show_frames
        for a in rv.frame_actors:
            assert a.GetVisibility() == 1

        # 2. Toggle Links
        rv.set_show_links(False)
        assert not rv.show_links
        assert rv.polyline_actor.GetVisibility() == 0
        for m in rv.marker_actors:
            assert m.GetVisibility() == 0

        rv.set_show_links(True)
        assert rv.show_links
        assert rv.polyline_actor.GetVisibility() == 1
        for m in rv.marker_actors:
            assert m.GetVisibility() == 1

        # 3. Toggle TCP
        rv.set_show_tcp(False)
        assert not rv.show_tcp
        assert rv.tcp_actor.GetVisibility() == 0

        rv.set_show_tcp(True)
        assert rv.show_tcp
        assert rv.tcp_actor.GetVisibility() == 1
    finally:
        plotter.close()


def test_main_window_integration_and_fk_failure_path(qapp):
    """MainWindow updates polyline on set_robot; FK failure logs warning and keeps last points."""
    win = MainWindow()
    try:
        proj = ProjectData.new("IntegrationProject")
        robot = make_3r_planar_robot()
        proj.robot_model = robot
        win.project_controller._set_project(proj, modified=False)

        pts_orig = np.copy(win.robot_view.polyline_points)
        assert pts_orig is not None

        # Update robot angles
        robot2 = make_3r_planar_robot()
        robot2.joints[0].q = 0.7
        robot2.joints[1].q = -0.3
        win.project_controller.set_robot(robot2)

        pts_updated = np.copy(win.robot_view.polyline_points)
        assert not np.allclose(pts_updated, pts_orig)

        # FK failure path: simulate error from cb.fk
        with patch("core_bridge.fk", side_effect=RuntimeError("Simulated FK failure")):
            robot3 = make_3r_planar_robot()
            robot3.joints[0].q = 1.5
            win.project_controller.set_robot(robot3)

        # Polyline points must be preserved from before failure
        pts_after_failure = win.robot_view.polyline_points
        np.testing.assert_allclose(pts_after_failure, pts_updated, atol=1e-9)

        # Check log panel contains warning
        log_text = win.log_panel.text()
        assert "Forward kinematics failed" in log_text or "Simulated FK failure" in log_text
    finally:
        win.close()


def test_camera_not_reset_on_robot_changed(qapp):
    """Camera is not reset on robot_changed; position is preserved."""
    win = MainWindow()
    try:
        proj = ProjectData.new("CameraTest")
        robot = make_3r_planar_robot()
        proj.robot_model = robot
        win.project_controller._set_project(proj, modified=False)

        cam = win.viewport.plotter.camera
        orig_pos = tuple(cam.position)
        # Move camera manually
        cam.position = (orig_pos[0] + 10.0, orig_pos[1] + 10.0, orig_pos[2] + 10.0)
        custom_pos = tuple(cam.position)

        # Trigger robot_changed
        robot2 = make_3r_planar_robot()
        robot2.joints[0].q = 0.5
        win.project_controller.set_robot(robot2)

        after_pos = tuple(win.viewport.plotter.camera.position)
        np.testing.assert_allclose(after_pos, custom_pos, atol=1e-5)
    finally:
        win.close()


def test_view_robot_submenu_actions(qapp):
    """View > Robot submenu has 3 checkable actions, default checked."""
    win = MainWindow()
    try:
        assert win._menu_robot is not None
        assert "Robot" in win._menu_robot.title()

        actions = win._menu_robot.actions()
        assert len(actions) == 3

        titles = [a.text() for a in actions]
        assert any("Frames" in t for t in titles)
        assert any("Links" in t for t in titles)
        assert any("TCP" in t for t in titles)

        for a in actions:
            assert a.isCheckable()
            assert a.isChecked()
    finally:
        win.close()
