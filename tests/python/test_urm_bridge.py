"""
tests/python/test_urm_bridge.py
R2.4: Tests for the pybind11 URM Python bindings and hardening.
"""

import os
import math
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import core_bridge as cb


def test_joint_type_enum():
    """JointType enum values are accessible from Python."""
    assert cb.JointType.Revolute is not None
    assert cb.JointType.Prismatic is not None
    assert cb.JointType.Fixed is not None


def test_dh_convention_enum():
    """DHConvention enum values are accessible from Python."""
    assert cb.DHConvention.Standard is not None
    assert cb.DHConvention.Modified is not None


def test_dh_params_readwrite():
    """DHParams fields can be set and read from Python."""
    dh = cb.DHParams()
    dh.a = 0.3
    dh.alpha = 1.5708
    dh.d = 0.1
    dh.theta = 0.0
    assert abs(dh.a - 0.3) < 1e-9
    assert abs(dh.alpha - 1.5708) < 1e-9


def test_pose_readwrite():
    """Pose fields xyz and rpy can be set and read from Python."""
    p = cb.Pose()
    p.xyz = [1.0, 2.0, 3.0]
    p.rpy = [0.1, 0.2, 0.3]
    assert abs(p.xyz[0] - 1.0) < 1e-9
    assert abs(p.rpy[2] - 0.3) < 1e-9
    assert "xyz" in repr(p)


def test_joint_creation():
    """Joint can be created and fields set from Python."""
    j = cb.Joint()
    j.name = "shoulder"
    j.type = cb.JointType.Revolute
    j.q = 0.5
    assert j.name == "shoulder"
    assert j.type == cb.JointType.Revolute
    assert abs(j.q - 0.5) < 1e-9


def test_default_limits_callable():
    """default_limits returns correct limits per type."""
    r_lim = cb.default_limits(cb.JointType.Revolute)
    assert abs(r_lim.lower - (-math.pi)) < 1e-6
    assert abs(r_lim.upper - math.pi) < 1e-6

    p_lim = cb.default_limits(cb.JointType.Prismatic)
    assert abs(p_lim.lower - 0.0) < 1e-6
    assert abs(p_lim.upper - 0.5) < 1e-6

    f_lim = cb.default_limits(cb.JointType.Fixed)
    assert abs(f_lim.lower - 0.0) < 1e-6
    assert abs(f_lim.upper - 0.0) < 1e-6


def test_link_creation():
    """Link can be created and fields set from Python."""
    l = cb.Link()
    l.name = "upper_arm"
    l.geometry_file = "meshes/upper_arm.stl"
    assert l.name == "upper_arm"
    assert l.geometry_file == "meshes/upper_arm.stl"


def test_robot_model_creation():
    """RobotModel can be created with metadata from Python."""
    robot = cb.RobotModel()
    robot.metadata.name = "TestBot"
    robot.metadata.author = "Claude"
    assert robot.metadata.name == "TestBot"
    assert robot.schema_version == 1
    assert robot.convention == cb.DHConvention.Standard
    assert robot.dof() == 0


def test_robot_model_convention_readwrite():
    """RobotModel convention can be modified."""
    robot = cb.RobotModel()
    robot.convention = cb.DHConvention.Modified
    assert robot.convention == cb.DHConvention.Modified


def test_robot_model_base_tool_poses():
    """RobotModel base and tool poses can be accessed and modified."""
    robot = cb.RobotModel()
    robot.base.xyz = [0.1, 0.2, 0.3]
    robot.tool.rpy = [0.0, 1.57, 0.0]
    assert abs(robot.base.xyz[0] - 0.1) < 1e-9
    assert abs(robot.tool.rpy[1] - 1.57) < 1e-9


def test_robot_model_add_joints():
    """Adding joints to RobotModel updates dof() correctly."""
    robot = cb.RobotModel()
    robot.metadata.name = "3DOF"

    for i in range(3):
        j = cb.Joint()
        j.name = f"joint{i+1}"
        j.type = cb.JointType.Revolute
        robot.joints.append(j)

    assert robot.dof() == 3


def test_robot_model_set_get_q():
    """set_q and get_q round-trip through Python."""
    robot = cb.RobotModel()
    for i in range(3):
        j = cb.Joint()
        j.type = cb.JointType.Revolute
        robot.joints.append(j)

    q_in = [0.1, 0.2, 0.3]
    robot.set_q(q_in)
    q_out = robot.get_q()
    assert len(q_out) == 3
    for a, b in zip(q_in, q_out):
        assert abs(a - b) < 1e-9


def test_set_q_wrong_length_raises_value_error():
    """set_q raises ValueError (std::invalid_argument) when input length != dof."""
    robot = cb.RobotModel()
    for i in range(2):
        j = cb.Joint()
        j.type = cb.JointType.Revolute
        robot.joints.append(j)

    with pytest.raises(ValueError, match="q size mismatch"):
        robot.set_q([0.1])

    with pytest.raises(ValueError, match="q size mismatch"):
        robot.set_q([0.1, 0.2, 0.3])


def test_robot_model_validate():
    """validate() returns a list of error strings for invalid models."""
    robot = cb.RobotModel()
    robot.links.append(cb.Link())
    j1 = cb.Joint()
    j1.name = "j1"
    j1.limits.lower = 1.0
    j1.limits.upper = 0.0  # invalid: lower > upper
    robot.joints.append(j1)

    errors = robot.validate()
    assert isinstance(errors, list)
    assert len(errors) > 0


def test_robot_model_fixed_joint_excluded():
    """Fixed joints are not counted in dof() from Python."""
    robot = cb.RobotModel()
    j_fixed = cb.Joint()
    j_fixed.type = cb.JointType.Fixed
    j_rev = cb.Joint()
    j_rev.type = cb.JointType.Revolute
    robot.joints.append(j_fixed)
    robot.joints.append(j_rev)
    robot.joints.append(j_rev)
    assert robot.dof() == 2


def test_robot_model_json_round_trip():
    """to_json / from_json round-trip through Python."""
    robot = cb.RobotModel()
    robot.metadata.name = "Puma560"
    robot.metadata.version = "2.0.0"
    robot.convention = cb.DHConvention.Modified
    robot.base.xyz = [0.1, 0.2, 0.3]

    for i in range(6):
        j = cb.Joint()
        j.name = f"joint{i+1}"
        j.type = cb.JointType.Revolute
        j.dh.a = 0.1 * (i + 1)
        j.q = 0.05 * (i + 1)
        robot.joints.append(j)

    json_str = robot.to_json()
    assert len(json_str) > 10

    loaded = cb.RobotModel.from_json(json_str)
    assert loaded.metadata.name == "Puma560"
    assert loaded.convention == cb.DHConvention.Modified
    assert abs(loaded.base.xyz[0] - 0.1) < 1e-9
    assert loaded.dof() == 6
    for i in range(6):
        assert abs(loaded.joints[i].dh.a - robot.joints[i].dh.a) < 1e-9


def test_from_json_bad_json_raises_runtime_error():
    """from_json raises RuntimeError on malformed JSON or unknown type."""
    with pytest.raises(RuntimeError):
        cb.RobotModel.from_json("{bad json")

    with pytest.raises(RuntimeError):
        cb.RobotModel.from_json('{"joints": [{"name": "j", "type": "bad_type"}]}')


def test_pose_whole_list_assignment():
    """Assigning whole list to pose attributes modifies the C++ struct [R2.4-4]."""
    robot = cb.RobotModel()
    robot.base.xyz = [1.5, 2.5, 3.5]
    assert list(robot.base.xyz) == [1.5, 2.5, 3.5]
    robot.tool.rpy = [0.1, 0.2, 0.3]
    assert list(robot.tool.rpy) == [0.1, 0.2, 0.3]


def test_urm_repr():
    """RobotModel __repr__ works from Python."""
    robot = cb.RobotModel()
    robot.metadata.name = "REPR_Test"
    r = repr(robot)
    assert "REPR_Test" in r
