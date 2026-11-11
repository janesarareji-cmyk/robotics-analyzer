"""
tests/python/test_robot_in_project.py
R2.6: Tests for robot model integration into project, controller,
MainWindow dock, and DH parameter editor.
"""

import math
import os
import zipfile
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QWidget
import core_bridge as cb

from app.main_window import MainWindow
from app.project.project_controller import ProjectController
from app.project.project_data import ProjectData
from app.project.serializer import (
    save,
    load,
    ProjectFormatError,
    ProjectSerializationError,
)
from app.panels.dh_editor import DHEditorWidget, COL_A, COL_TYPE
from app.settings import AppSettings


def make_valid_3joint_robot() -> cb.RobotModel:
    """Helper: construct a valid 3-joint serial robot model with 4 links."""
    robot = cb.RobotModel()
    robot.metadata.name = "Arm3DOF"
    robot.convention = cb.DHConvention.Modified

    robot.base.xyz = [0.1, 0.2, 0.3]
    robot.base.rpy = [0.0, 0.1, 0.2]
    robot.tool.xyz = [0.4, 0.5, 0.6]
    robot.tool.rpy = [0.3, 0.4, 0.5]

    for i in range(4):
        lk = cb.Link()
        lk.name = f"link{i}"
        robot.links.append(lk)

    for i in range(3):
        j = cb.Joint()
        j.name = f"joint{i + 1}"
        j.type = cb.JointType.Revolute
        j.mdh.a = 0.15 * (i + 1)
        j.mdh.alpha = 0.05 * (i + 1)
        j.mdh.d = 0.25 * (i + 1)
        j.mdh.theta = 0.0
        j.q = 0.1 * (i + 1)
        j.limits = cb.default_limits(j.type)
        robot.joints.append(j)

    return robot


def test_save_load_round_trip_3_joint_robot(tmp_path):
    """save/load preserves full 3-joint RobotModel structure and properties."""
    proj = ProjectData.new("RoundTripBot", "Testing 3-joint persistence")
    robot = make_valid_3joint_robot()
    proj.robot_model = robot

    path = tmp_path / "arm3dof.robotproj"
    save(proj, path)

    loaded = load(path)
    loaded_robot = loaded.robot_model

    assert loaded_robot is not None
    assert loaded_robot.metadata.name == "Arm3DOF"
    assert loaded_robot.convention == cb.DHConvention.Modified
    assert len(loaded_robot.joints) == 3
    assert len(loaded_robot.links) == 4
    assert loaded_robot.dof() == 3

    # Verify joints
    for i in range(3):
        orig_j = robot.joints[i]
        load_j = loaded_robot.joints[i]
        assert load_j.name == orig_j.name
        assert load_j.type == orig_j.type
        assert load_j.mdh.a == pytest.approx(orig_j.mdh.a, rel=1e-6)
        assert load_j.mdh.alpha == pytest.approx(orig_j.mdh.alpha, rel=1e-6)
        assert load_j.mdh.d == pytest.approx(orig_j.mdh.d, rel=1e-6)
        assert load_j.q == pytest.approx(orig_j.q, rel=1e-6)

    # Verify base and tool poses
    assert loaded_robot.base.xyz == pytest.approx([0.1, 0.2, 0.3], rel=1e-6)
    assert loaded_robot.base.rpy == pytest.approx([0.0, 0.1, 0.2], rel=1e-6)
    assert loaded_robot.tool.xyz == pytest.approx([0.4, 0.5, 0.6], rel=1e-6)
    assert loaded_robot.tool.rpy == pytest.approx([0.3, 0.4, 0.5], rel=1e-6)


def test_old_file_without_robot_loads_empty(tmp_path):
    """A .robotproj archive without robot_model.json loads with empty RobotModel."""
    proj = ProjectData.new("LegacyProject")
    path = tmp_path / "legacy.robotproj"

    # Manually write archive without robot_model.json
    with zipfile.ZipFile(path, "w") as zf:
        import json
        zf.writestr("project_metadata.json", json.dumps(proj.metadata.to_dict()))
        zf.writestr("simulation_settings.json", json.dumps(proj.simulation_settings.to_dict()))

    loaded = load(path)
    assert loaded.robot_model is not None
    assert len(loaded.robot_model.joints) == 0
    assert loaded.robot_model.convention == cb.DHConvention.Standard


def test_bad_robot_json_error_not_crash(tmp_path, qapp):
    """Corrupt robot_model.json causes load error and clean controller handling without crash."""
    from unittest.mock import patch
    proj = ProjectData.new("BadRobotProject")
    path = tmp_path / "bad_robot.robotproj"

    # Write archive with corrupt robot_model.json
    with zipfile.ZipFile(path, "w") as zf:
        import json
        zf.writestr("project_metadata.json", json.dumps(proj.metadata.to_dict()))
        zf.writestr("simulation_settings.json", json.dumps(proj.simulation_settings.to_dict()))
        zf.writestr("robot_model.json", "INVALID JSON CONTENT")

    with pytest.raises(ProjectFormatError):
        load(path)

    # Controller handles error gracefully without blocking
    settings = AppSettings()
    controller = ProjectController(settings)
    widget = QWidget()
    with patch("app.project.project_controller.QMessageBox.critical") as mock_crit:
        result = controller.open_project(widget, str(path))
        assert result is False
        assert not controller.has_project
        mock_crit.assert_called_once()


def test_editing_in_editor_changes_controller_robot_and_dirty_flag(qapp):
    """Editing cell in DHEditorWidget updates controller robot and marks project modified."""
    window = MainWindow()
    try:
        proj = ProjectData.new("EditorSyncProject")
        robot = make_valid_3joint_robot()
        proj.robot_model = robot
        window.project_controller._set_project(proj, modified=False)

        assert not window.project_controller.is_modified

        # Edit cell in table
        window.dh_editor._table.item(0, COL_A).setText("0.8888")

        assert window.project_controller.is_modified
        assert window.project_controller.robot.joints[0].dh.a == pytest.approx(0.8888, rel=1e-5)
    finally:
        window.close()


def test_robot_changed_emitted(qapp, qtbot):
    """ProjectController.set_robot emits robot_changed signal."""
    settings = AppSettings()
    controller = ProjectController(settings)
    proj = ProjectData.new("SignalProject")
    controller._set_project(proj, modified=False)

    new_model = cb.RobotModel()
    new_model.metadata.name = "EmittedBot"

    with qtbot.waitSignal(controller.robot_changed, timeout=1000) as blocker:
        controller.set_robot(new_model)

    assert blocker.args[0].metadata.name == "EmittedBot"
    assert controller.is_modified


def test_joint_type_change_applies_default_limits(qapp):
    """Changing joint type in editor applies default limits per joint type."""
    robot = cb.RobotModel()
    j = cb.Joint()
    j.name = "j1"
    j.type = cb.JointType.Revolute
    j.limits = cb.default_limits(cb.JointType.Revolute)
    robot.joints.append(j)

    editor = DHEditorWidget()
    editor.set_model(robot)

    # Switch to Prismatic
    editor.set_joint_type(0, cb.JointType.Prismatic)
    assert robot.joints[0].type == cb.JointType.Prismatic
    assert robot.joints[0].limits.lower == pytest.approx(0.0)
    assert robot.joints[0].limits.upper == pytest.approx(0.5)

    # Switch to Fixed
    editor.set_joint_type(0, cb.JointType.Fixed)
    assert robot.joints[0].type == cb.JointType.Fixed
    assert robot.joints[0].limits.lower == pytest.approx(0.0)
    assert robot.joints[0].limits.upper == pytest.approx(0.0)


def test_validate_errors_shown(qapp):
    """Validation errors appear in error label when model is invalid, and hide when fixed."""
    robot = cb.RobotModel()
    j = cb.Joint()
    j.name = "j1"
    j.type = cb.JointType.Revolute
    # Invalid limits: lower > upper
    j.limits.lower = 1.0
    j.limits.upper = -1.0
    robot.joints.append(j)

    editor = DHEditorWidget()
    editor.show()
    editor.set_model(robot)

    assert not editor._error_label.isHidden()
    assert editor._error_label.isVisible()
    assert len(editor._error_label.text()) > 0
    assert "lower limit" in editor._error_label.text()

    # Fix robot
    robot.joints[0].limits.lower = -1.0
    robot.joints[0].limits.upper = 1.0
    robot.joints[0].parent_link = 0
    robot.joints[0].child_link = 1
    lk0 = cb.Link()
    lk0.name = "base"
    lk1 = cb.Link()
    lk1.name = "l1"
    robot.links.append(lk0)
    robot.links.append(lk1)

    editor._validate_and_notify()
    assert editor._error_label.isHidden()
    assert not editor._error_label.isVisible()
    assert editor._error_label.text() == ""
    editor.close()


def test_convention_and_base_tool_edits_persist_save_load(tmp_path, qapp):
    """Edits to convention and base/tool spinboxes persist through save and load."""
    window = MainWindow()
    try:
        proj = ProjectData.new("PersistTest")
        robot = make_valid_3joint_robot()
        proj.robot_model = robot
        window.project_controller._set_project(proj, modified=False)

        # Change convention to Standard (index 0)
        window.dh_editor._combo_convention.setCurrentIndex(0)

        # Change base XYZ
        window.dh_editor._base_x.setValue(2.5)
        window.dh_editor._base_y.setValue(3.5)
        window.dh_editor._base_z.setValue(4.5)

        # Change tool XYZ
        window.dh_editor._tool_x.setValue(0.75)
        window.dh_editor._tool_y.setValue(1.25)
        window.dh_editor._tool_z.setValue(1.75)

        save_path = tmp_path / "edited.robotproj"
        save(window.project_controller.current_project, save_path)

        loaded = load(save_path)
        assert loaded.robot_model.convention == cb.DHConvention.Standard
        assert loaded.robot_model.base.xyz == pytest.approx([2.5, 3.5, 4.5], rel=1e-5)
        assert loaded.robot_model.tool.xyz == pytest.approx([0.75, 1.25, 1.75], rel=1e-5)
    finally:
        window.close()


def test_dock_toggle_in_view_menu(qapp):
    """DH parameter editor dock can be toggled via View menu action."""
    window = MainWindow()
    try:
        window.show()
        assert window.dh_dock is not None
        assert window.dh_dock.isVisible()
        assert window._action_view_dh.isChecked()

        # Toggle off
        window._action_view_dh.trigger()
        assert not window.dh_dock.isVisible()
        assert not window._action_view_dh.isChecked()

        # Toggle on
        window._action_view_dh.trigger()
        assert window.dh_dock.isVisible()
        assert window._action_view_dh.isChecked()
    finally:
        window.close()
