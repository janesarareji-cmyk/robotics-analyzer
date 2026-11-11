"""
tests/python/test_dh_editor.py
R2.3: Tests for the DHEditorWidget standard DH parameter editor.
"""

import os
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import core_bridge as cb
from app.panels.dh_editor import DHEditorWidget, COL_A, COL_ALPHA, COL_D, COL_THETA, COL_Q, COL_JOINT


def make_robot(n_joints: int = 3) -> cb.RobotModel:
    """Helper: create a robot with n revolute joints."""
    robot = cb.RobotModel()
    robot.metadata.name = f"TestBot_{n_joints}DOF"
    for i in range(n_joints):
        j = cb.Joint()
        j.name = f"joint{i + 1}"
        j.type = cb.JointType.Revolute
        j.dh.a = 0.1 * (i + 1)
        j.dh.alpha = 0.0
        j.dh.d = 0.2 * (i + 1)
        j.q = 0.0
        robot.joints.append(j)
    return robot


def test_dh_editor_creation(qapp):
    """DHEditorWidget creates without error."""
    editor = DHEditorWidget()
    assert editor is not None


def test_dh_editor_set_model_populates_table(qapp):
    """set_model() fills the table with one row per joint."""
    robot = make_robot(3)
    editor = DHEditorWidget()
    editor.set_model(robot)
    assert editor._table.rowCount() == 3


def test_dh_editor_joint_names_in_table(qapp):
    """Joint names appear in the Name column."""
    robot = make_robot(2)
    editor = DHEditorWidget()
    editor.set_model(robot)
    assert editor._table.item(0, COL_JOINT).text() == "joint1"
    assert editor._table.item(1, COL_JOINT).text() == "joint2"


def test_dh_editor_a_values_in_table(qapp):
    """DH 'a' values are shown correctly in the table."""
    robot = make_robot(2)
    editor = DHEditorWidget()
    editor.set_model(robot)
    assert abs(float(editor._table.item(0, COL_A).text()) - 0.1) < 1e-5


def test_dh_editor_model_changed_signal(qapp, qtbot):
    """Editing a cell emits model_changed signal."""
    robot = make_robot(2)
    editor = DHEditorWidget()
    editor.set_model(robot)

    with qtbot.waitSignal(editor.model_changed, timeout=1000):
        editor._table.item(0, COL_A).setText("0.999")


def test_dh_editor_edit_updates_model(qapp):
    """Editing DH 'a' value updates the underlying model."""
    robot = make_robot(2)
    editor = DHEditorWidget()
    editor.set_model(robot)
    editor._table.item(0, COL_A).setText("0.555")
    assert abs(robot.joints[0].dh.a - 0.555) < 1e-5


def test_dh_editor_add_joint(qapp):
    """Add Joint button adds a joint to the model and table."""
    robot = make_robot(2)
    editor = DHEditorWidget()
    editor.set_model(robot)
    editor._on_add_joint()
    assert editor._table.rowCount() == 3
    assert len(robot.joints) == 3


def test_dh_editor_remove_joint(qapp):
    """Remove Joint button removes the selected joint."""
    robot = make_robot(3)
    editor = DHEditorWidget()
    editor.set_model(robot)
    editor._table.selectRow(1)
    editor._on_remove_joint()
    assert editor._table.rowCount() == 2
    assert len(robot.joints) == 2


def test_dh_editor_invalid_float_reverts(qapp):
    """Entering a non-numeric value does not corrupt the model."""
    robot = make_robot(1)
    original_a = robot.joints[0].dh.a
    editor = DHEditorWidget()
    editor.set_model(robot)
    editor._table.item(0, COL_A).setText("not_a_number")
    # Model should be unchanged
    assert abs(robot.joints[0].dh.a - original_a) < 1e-9


def test_dh_editor_no_model(qapp):
    """DHEditorWidget works with no model set (empty table)."""
    editor = DHEditorWidget()
    assert editor._table.rowCount() == 0
    editor._on_add_joint()   # should not crash
    editor._on_remove_joint()  # should not crash
