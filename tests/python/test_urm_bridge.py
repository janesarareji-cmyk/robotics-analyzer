"""
tests/python/test_urm_bridge.py
R3.1: Tests for the pybind11 URM Python bindings and hardening.
Includes schema v2 fields, JointKinematics, Mimic, q_index_map, and v1 migration fixture.
"""

import os
import math
from pathlib import Path
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import core_bridge as cb


def test_joint_type_enum():
    """JointType enum values are accessible from Python."""
    assert cb.JointType.Revolute is not None
    assert cb.JointType.Prismatic is not None
    assert cb.JointType.Fixed is not None


def test_joint_kinematics_enum():
    """JointKinematics enum values are accessible from Python."""
    assert cb.JointKinematics.DH is not None
    assert cb.JointKinematics.OriginAxis is not None


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


def test_mimic_struct_and_joint_mimic():
    """Mimic struct and Joint.mimic optional field operate correctly."""
    mi = cb.Mimic()
    mi.joint = "j_master"
    mi.multiplier = 2.0
    mi.offset = -0.5
    assert mi.joint == "j_master"
    assert mi.multiplier == 2.0
    assert mi.offset == -0.5
    assert "j_master" in repr(mi)

    j = cb.Joint()
    assert j.mimic is None
    j.mimic = mi
    assert j.mimic is not None
    assert j.mimic.joint == "j_master"
    assert j.mimic.multiplier == 2.0
    assert j.mimic.offset == -0.5

    j.mimic = None
    assert j.mimic is None


def test_joint_creation_and_schema_v2_fields():
    """Joint can be created and schema v2 fields read/write."""
    j = cb.Joint()
    j.name = "shoulder"
    j.type = cb.JointType.Revolute
    j.parent_link = 2
    j.child_link = 3
    j.kinematics = cb.JointKinematics.OriginAxis
    j.origin.xyz = [0.1, 0.2, 0.3]
    j.origin.rpy = [0.0, 0.5, 0.0]
    j.axis = [0.0, 1.0, 0.0]
    j.q = 0.5

    assert j.name == "shoulder"
    assert j.type == cb.JointType.Revolute
    assert j.parent_link == 2
    assert j.child_link == 3
    assert j.kinematics == cb.JointKinematics.OriginAxis
    assert list(j.origin.xyz) == [0.1, 0.2, 0.3]
    assert list(j.origin.rpy) == [0.0, 0.5, 0.0]
    assert list(j.axis) == [0.0, 1.0, 0.0]
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
    assert robot.schema_version == 2
    assert robot.tcp_link == -1
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


def test_robot_model_tcp_link():
    """RobotModel.tcp_link can be set and read."""
    robot = cb.RobotModel()
    assert robot.tcp_link == -1
    robot.tcp_link = 4
    assert robot.tcp_link == 4


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


def test_robot_model_q_index_map():
    """q_index_map correctly identifies actuated vs fixed/mimic joints."""
    robot = cb.RobotModel()
    j1 = cb.Joint()
    j1.type = cb.JointType.Revolute
    j2 = cb.Joint()
    j2.type = cb.JointType.Fixed
    j3 = cb.Joint()
    j3.type = cb.JointType.Revolute
    j4 = cb.Joint()
    j4.type = cb.JointType.Revolute
    mi = cb.Mimic()
    mi.joint = "joint1"
    j4.mimic = mi
    j5 = cb.Joint()
    j5.type = cb.JointType.Prismatic

    robot.joints.append(j1)
    robot.joints.append(j2)
    robot.joints.append(j3)
    robot.joints.append(j4)
    robot.joints.append(j5)

    assert robot.dof() == 3
    assert robot.q_index_map() == [0, -1, 1, -1, 2]


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


def test_robot_model_validate_new_rules():
    """validate() reports error strings for schema v2 violations."""
    robot = cb.RobotModel()
    robot.links.append(cb.Link())
    robot.links.append(cb.Link())

    j1 = cb.Joint()
    j1.name = "j1"
    j1.parent_link = 1
    j1.child_link = 0  # Invalid: parent_link >= child_link and child_link == 0
    robot.joints.append(j1)

    errors = robot.validate()
    assert isinstance(errors, list)
    assert any("must be less than child_link" in e for e in errors)
    assert any("child_link cannot be 0" in e for e in errors)


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
    """to_json / from_json round-trip through Python with schema v2."""
    robot = cb.RobotModel()
    robot.metadata.name = "Puma560"
    robot.metadata.version = "2.0.0"
    robot.convention = cb.DHConvention.Modified
    robot.base.xyz = [0.1, 0.2, 0.3]
    robot.tcp_link = 6

    for i in range(6):
        robot.links.append(cb.Link())
        j = cb.Joint()
        j.name = f"joint{i+1}"
        j.type = cb.JointType.Revolute
        j.parent_link = i
        j.child_link = i + 1
        j.kinematics = cb.JointKinematics.DH
        j.dh.a = 0.1 * (i + 1)
        j.q = 0.05 * (i + 1)
        robot.joints.append(j)
    robot.links.append(cb.Link())

    json_str = robot.to_json()
    assert len(json_str) > 10

    loaded = cb.RobotModel.from_json(json_str)
    assert loaded.metadata.name == "Puma560"
    assert loaded.convention == cb.DHConvention.Modified
    assert loaded.schema_version == 2
    assert loaded.tcp_link == 6
    assert abs(loaded.base.xyz[0] - 0.1) < 1e-9
    assert loaded.dof() == 6
    for i in range(6):
        assert abs(loaded.joints[i].dh.a - robot.joints[i].dh.a) < 1e-9
        assert loaded.joints[i].parent_link == i
        assert loaded.joints[i].child_link == i + 1


def test_from_json_bad_json_raises_runtime_error():
    """from_json raises RuntimeError on malformed JSON or unknown type."""
    with pytest.raises(RuntimeError):
        cb.RobotModel.from_json("{bad json")

    with pytest.raises(RuntimeError):
        cb.RobotModel.from_json('{"joints": [{"name": "j", "type": "bad_type"}]}')


def test_v1_fixture_migration_via_from_json():
    """Loading schema v1 JSON fixture migrates to tree schema in memory."""
    fixture_path = Path(__file__).parent / "fixtures" / "robot_model_v1.json"
    assert fixture_path.exists(), f"Missing fixture at {fixture_path}"

    with open(fixture_path, "r", encoding="utf-8") as f:
        v1_text = f.read()

    loaded = cb.RobotModel.from_json(v1_text)
    assert loaded.schema_version == 1
    assert loaded.tcp_link == 2
    assert len(loaded.joints) == 2
    assert len(loaded.links) == 3

    assert loaded.joints[0].parent_link == 0
    assert loaded.joints[0].child_link == 1
    assert loaded.joints[0].kinematics == cb.JointKinematics.DH
    assert list(loaded.joints[0].axis) == [0.0, 0.0, 1.0]
    assert loaded.joints[0].mimic is None

    assert loaded.joints[1].parent_link == 1
    assert loaded.joints[1].child_link == 2
    assert loaded.joints[1].kinematics == cb.JointKinematics.DH

    # Migrated model passes validate() with zero errors
    assert loaded.validate() == []


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
