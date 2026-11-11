"""
tests/python/test_urdf_import.py
R3.3: URDF parser tests (fixtures in tests/python/fixtures/urdf/).
"""

import math
import os
from pathlib import Path

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import core_bridge as cb
from app.importers import import_urdf
from app.importers import import_report as ir

FIX = Path(__file__).parent / "fixtures" / "urdf"


def _fx(name):
    return str(FIX / name)


def _write(tmp_path, text, name="robot.urdf"):
    p = tmp_path / name
    p.write_text(text, encoding="utf-8")
    return str(p)


def _codes(report, severity=None):
    issues = report.issues if severity is None else report.by_severity(severity)
    return [i.code for i in issues]


def _assert_canonical(model):
    """joints[i].child_link == i + 1 and parent < child (R3.3 rule 5)."""
    for i, j in enumerate(model.joints):
        assert j.child_link == i + 1
        assert j.parent_link < j.child_link


# ---------------------------------------------------------------------------
# Report types
# ---------------------------------------------------------------------------

def test_report_add_query_and_text():
    rep = ir.ImportReport()
    rep.add(ir.SEV_WARNING, ir.URDF_MISSING_LIMIT, "no limit", "j1")
    rep.add(ir.SEV_INFO, ir.URDF_TCP_CHOSEN, "tcp")
    assert not rep.has_errors()
    assert len(rep.by_severity(ir.SEV_WARNING)) == 1
    assert rep.issues[0].element == "j1"
    assert rep.issues[1].element is None
    rep.add(ir.SEV_ERROR, ir.URDF_CYCLE, "loop")
    assert rep.has_errors()
    text = rep.to_text()
    assert ir.URDF_MISSING_LIMIT in text and "[j1]" in text
    assert ir.URDF_CYCLE in text


def test_issue_code_constants_match_spec_names():
    """Every code from the R3.3 table is a module constant with its own name."""
    names = [
        "URDF_FILE_NOT_FOUND", "URDF_BAD_XML", "URDF_XACRO",
        "URDF_UNSUPPORTED_JOINT", "URDF_UNKNOWN_JOINT_TYPE", "URDF_MISSING_LINK",
        "URDF_DUPLICATE_NAME", "URDF_MULTIPLE_PARENTS", "URDF_MULTIPLE_ROOTS",
        "URDF_CYCLE", "URDF_ZERO_AXIS", "URDF_MIMIC_UNKNOWN", "URDF_INVALID_MODEL",
        "URDF_CONTINUOUS_LIMITS", "URDF_MISSING_LIMIT", "URDF_IGNORED_ELEMENT",
        "URDF_TCP_CHOSEN",
    ]
    for n in names:
        assert getattr(ir, n) == n


# ---------------------------------------------------------------------------
# Valid fixtures
# ---------------------------------------------------------------------------

def test_planar_2r_structure_and_report():
    res = import_urdf(_fx("planar_2r.urdf"))
    assert res.model is not None
    assert _codes(res.report, "error") == []
    assert _codes(res.report, "warning") == []
    assert _codes(res.report, "info") == [ir.URDF_TCP_CHOSEN]
    m = res.model
    assert len(m.links) == 4
    assert len(m.joints) == 3
    assert m.dof() == 2
    assert list(m.q_index_map()) == [0, 1, -1]
    assert m.tcp_link == 3
    assert m.metadata.name == "planar_2r"
    assert [j.name for j in m.joints] == ["j1", "j2", "j_tip"]
    assert [l.name for l in m.links] == ["base", "l1", "l2", "tip"]
    assert m.joints[1].parent_link == 1 and m.joints[1].child_link == 2
    assert m.joints[2].type == cb.JointType.Fixed
    assert m.joints[0].kinematics == cb.JointKinematics.OriginAxis
    _assert_canonical(m)
    assert m.validate() == []


def test_planar_2r_fk_known_answer():
    m = import_urdf(_fx("planar_2r.urdf")).model
    res = cb.fk(m, [math.pi / 2.0, -math.pi / 2.0])
    tcp = res["tcp"]
    assert np.allclose(tcp[:3, 3], [1.0, 1.0, 0.0], atol=1e-12)
    res0 = cb.fk(m, [0.0, 0.0])
    assert np.allclose(res0["tcp"][:3, 3], [2.0, 0.0, 0.0], atol=1e-12)


def test_serial_6dof_structure_and_report():
    res = import_urdf(_fx("serial_6dof.urdf"))
    assert res.model is not None
    assert _codes(res.report, "error") == []
    assert _codes(res.report, "warning") == []
    # visual + inertial skipped (once each), plus the TCP info
    assert sorted(_codes(res.report, "info")) == sorted(
        [ir.URDF_IGNORED_ELEMENT, ir.URDF_IGNORED_ELEMENT, ir.URDF_TCP_CHOSEN]
    )
    m = res.model
    assert len(m.links) == 7
    assert len(m.joints) == 6
    assert m.dof() == 6
    assert m.tcp_link == 6
    assert list(m.q_index_map()) == [0, 1, 2, 3, 4, 5]
    assert [j.name for j in m.joints] == [f"joint{i}" for i in range(1, 7)]
    assert m.joints[1].axis == [0.0, 1.0, 0.0]
    _assert_canonical(m)
    assert m.validate() == []


def test_gripper_mimic_tree_structure():
    res = import_urdf(_fx("gripper_mimic.urdf"))
    assert res.model is not None
    assert _codes(res.report, "error") == []
    assert _codes(res.report, "warning") == []
    m = res.model
    assert len(m.links) == 3
    assert len(m.joints) == 2
    # mimic joint is not counted in dof()
    assert m.dof() == 1
    assert list(m.q_index_map()) == [0, -1]
    # both fingers depth 1: tie -> lowest index (left_finger)
    assert m.tcp_link == 1
    tcp_info = res.report.by_code(ir.URDF_TCP_CHOSEN)[0]
    assert tcp_info.element == "left_finger"
    right = m.joints[1]
    assert right.name == "right_finger_joint"
    assert right.mimic is not None
    assert right.mimic.joint == "left_finger_joint"
    assert right.mimic.multiplier == 1.0
    assert right.mimic.offset == 0.0
    assert m.joints[0].mimic is None
    _assert_canonical(m)
    assert m.validate() == []


def test_gripper_fk_both_fingers():
    m = import_urdf(_fx("gripper_mimic.urdf")).model
    res0 = cb.fk(m, [0.0])
    assert np.allclose(res0["frames"][1][:3, 3], [0.0, 0.01, 0.0], atol=1e-12)
    assert np.allclose(res0["frames"][2][:3, 3], [0.0, -0.01, 0.0], atol=1e-12)
    res = cb.fk(m, [0.02])
    # left finger moves +y by q, right finger moves -y by q (mimic)
    assert np.allclose(res["frames"][1][:3, 3], [0.0, 0.03, 0.0], atol=1e-12)
    assert np.allclose(res["frames"][2][:3, 3], [0.0, -0.03, 0.0], atol=1e-12)
    assert np.allclose(res["tcp"][:3, 3], [0.0, 0.03, 0.0], atol=1e-12)


def test_continuous_joint_limits_and_warning():
    res = import_urdf(_fx("continuous_joint.urdf"))
    assert res.model is not None
    assert _codes(res.report, "error") == []
    assert _codes(res.report, "warning") == [ir.URDF_CONTINUOUS_LIMITS]
    j = res.model.joints[0]
    assert j.type == cb.JointType.Revolute
    assert j.limits.lower == pytest.approx(-math.pi)
    assert j.limits.upper == pytest.approx(math.pi)
    assert j.limits.vel == pytest.approx(2.0)
    assert j.limits.effort == pytest.approx(5.0)
    assert res.model.dof() == 1


def test_missing_limit_warnings_and_defaults():
    res = import_urdf(_fx("missing_limit.urdf"))
    assert res.model is not None
    assert _codes(res.report, "error") == []
    # j1: no <limit>; j2: <limit> without effort
    assert _codes(res.report, "warning") == [ir.URDF_MISSING_LIMIT, ir.URDF_MISSING_LIMIT]
    j1, j2 = res.model.joints
    assert j1.limits.lower == pytest.approx(-math.pi)
    assert j1.limits.upper == pytest.approx(math.pi)
    assert j1.limits.effort == pytest.approx(100.0)  # default_limits(Revolute)
    assert j2.limits.lower == pytest.approx(-1.0)
    assert j2.limits.upper == pytest.approx(1.0)
    assert j2.limits.vel == pytest.approx(1.0)
    assert j2.limits.effort == pytest.approx(100.0)  # missing -> default


def test_reversed_file_order_gives_canonical_order():
    res = import_urdf(_fx("serial_reversed.urdf"))
    assert res.model is not None
    assert _codes(res.report, "error") == []
    m = res.model
    assert [j.name for j in m.joints] == ["j1", "j2", "j3"]
    assert [j.child_link for j in m.joints] == [1, 2, 3]
    _assert_canonical(m)
    assert m.tcp_link == 3


def test_prismatic_initial_q_clamped_into_limits(tmp_path):
    text = """<?xml version="1.0"?>
<robot name="pr">
  <link name="base"/><link name="slider"/>
  <joint name="s" type="prismatic">
    <parent link="base"/><child link="slider"/>
    <axis xyz="0 0 2"/>
    <limit lower="0.1" upper="0.5" effort="5" velocity="1"/>
  </joint>
</robot>"""
    res = import_urdf(_write(tmp_path, text))
    assert res.model is not None
    j = res.model.joints[0]
    assert j.q == pytest.approx(0.1)  # 0 clamped up to lower
    assert j.limits.lower == pytest.approx(0.1)
    # axis normalised from (0, 0, 2)
    assert j.axis == pytest.approx([0.0, 0.0, 1.0])
    assert res.model.validate() == []


def test_mimic_multiplier_and_offset_read(tmp_path):
    text = """<?xml version="1.0"?>
<robot name="mm">
  <link name="base"/><link name="a"/><link name="b"/>
  <joint name="ja" type="revolute">
    <parent link="base"/><child link="a"/><axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="5" velocity="1"/>
  </joint>
  <joint name="jb" type="revolute">
    <parent link="base"/><child link="b"/><axis xyz="0 0 1"/>
    <limit lower="-3" upper="3" effort="5" velocity="1"/>
    <mimic joint="ja" multiplier="-2" offset="0.5"/>
  </joint>
</robot>"""
    res = import_urdf(_write(tmp_path, text))
    assert res.model is not None
    mi = res.model.joints[1].mimic
    assert mi.joint == "ja"
    assert mi.multiplier == pytest.approx(-2.0)
    assert mi.offset == pytest.approx(0.5)
    assert res.model.dof() == 1


def test_ignored_top_level_elements_info_once_per_kind(tmp_path):
    text = """<?xml version="1.0"?>
<robot name="ign">
  <material name="red"><color rgba="1 0 0 1"/></material>
  <material name="blue"><color rgba="0 0 1 1"/></material>
  <transmission name="t"/>
  <gazebo><plugin name="p"/></gazebo>
  <link name="base"/><link name="l1"/>
  <joint name="j1" type="revolute">
    <parent link="base"/><child link="l1"/><axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="5" velocity="1"/>
  </joint>
</robot>"""
    res = import_urdf(_write(tmp_path, text))
    assert res.model is not None
    ignored = res.report.by_code(ir.URDF_IGNORED_ELEMENT)
    assert len(ignored) == 3  # material, transmission, gazebo: once each
    assert all(i.severity == "info" for i in ignored)


def test_import_report_no_visuals_or_collisions_until_r35():
    res = import_urdf(_fx("planar_2r.urdf"))
    assert res.visuals == []
    assert res.collisions == []


# ---------------------------------------------------------------------------
# Error fixtures: one error each, model None
# ---------------------------------------------------------------------------

@pytest.mark.parametrize(
    "fixture, code",
    [
        ("xacro.urdf", ir.URDF_XACRO),
        ("bad_xml.urdf", ir.URDF_BAD_XML),
        ("two_roots.urdf", ir.URDF_MULTIPLE_ROOTS),
        ("cycle.urdf", ir.URDF_CYCLE),
        ("floating.urdf", ir.URDF_UNSUPPORTED_JOINT),
        ("missing_parent.urdf", ir.URDF_MISSING_LINK),
        ("zero_axis.urdf", ir.URDF_ZERO_AXIS),
        ("unknown_type.urdf", ir.URDF_UNKNOWN_JOINT_TYPE),
    ],
)
def test_error_fixture_reports_code(fixture, code):
    res = import_urdf(_fx(fixture))
    assert res.model is None
    assert _codes(res.report, "error") == [code]
    assert res.report.has_errors()


def test_xacro_message_points_to_xacro_tool():
    res = import_urdf(_fx("xacro.urdf"))
    assert "xacro" in res.report.issues[0].message


def test_missing_file_returns_error_not_exception():
    res = import_urdf(str(FIX / "does_not_exist.urdf"))
    assert res.model is None
    assert _codes(res.report, "error") == [ir.URDF_FILE_NOT_FOUND]


def test_inline_duplicate_link_name(tmp_path):
    text = """<robot name="d"><link name="a"/><link name="a"/></robot>"""
    res = import_urdf(_write(tmp_path, text))
    assert res.model is None
    assert _codes(res.report, "error") == [ir.URDF_DUPLICATE_NAME]


def test_inline_duplicate_joint_name(tmp_path):
    text = """<robot name="d"><link name="a"/><link name="b"/><link name="c"/>
      <joint name="j" type="revolute"><parent link="a"/><child link="b"/></joint>
      <joint name="j" type="revolute"><parent link="b"/><child link="c"/></joint>
    </robot>"""
    res = import_urdf(_write(tmp_path, text))
    assert res.model is None
    assert _codes(res.report, "error") == [ir.URDF_DUPLICATE_NAME]


def test_inline_multiple_parents(tmp_path):
    text = """<robot name="mp">
      <link name="base"/><link name="a"/><link name="b"/>
      <joint name="j1" type="revolute"><parent link="base"/><child link="a"/>
        <axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="1"/></joint>
      <joint name="j2" type="revolute"><parent link="base"/><child link="b"/>
        <axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="1"/></joint>
      <joint name="j3" type="revolute"><parent link="a"/><child link="b"/>
        <axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="1"/></joint>
    </robot>"""
    res = import_urdf(_write(tmp_path, text))
    assert res.model is None
    assert _codes(res.report, "error") == [ir.URDF_MULTIPLE_PARENTS]


def test_inline_mimic_unknown(tmp_path):
    text = """<robot name="mu">
      <link name="base"/><link name="a"/>
      <joint name="j1" type="revolute"><parent link="base"/><child link="a"/>
        <axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="1"/>
        <mimic joint="nope"/></joint>
    </robot>"""
    res = import_urdf(_write(tmp_path, text))
    assert res.model is None
    assert _codes(res.report, "error") == [ir.URDF_MIMIC_UNKNOWN]


def test_inline_zero_roots_is_cycle(tmp_path):
    text = """<robot name="zr"><link name="a"/><link name="b"/>
      <joint name="j1" type="revolute"><parent link="a"/><child link="b"/>
        <axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="1"/></joint>
      <joint name="j2" type="revolute"><parent link="b"/><child link="a"/>
        <axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="1"/></joint>
    </robot>"""
    res = import_urdf(_write(tmp_path, text))
    assert res.model is None
    assert _codes(res.report, "error") == [ir.URDF_CYCLE]


def test_inline_invalid_model_from_bad_limits(tmp_path):
    """lower > upper passes the parser but fails validate(): URDF_INVALID_MODEL per message."""
    text = """<robot name="im"><link name="base"/><link name="a"/>
      <joint name="j1" type="prismatic"><parent link="base"/><child link="a"/>
        <axis xyz="0 0 1"/><limit lower="1" upper="0" effort="1" velocity="1"/></joint>
    </robot>"""
    res = import_urdf(_write(tmp_path, text))
    assert res.model is None
    errs = _codes(res.report, "error")
    assert len(errs) >= 1
    assert set(errs) == {ir.URDF_INVALID_MODEL}


def test_inline_wrong_root_tag_is_bad_xml(tmp_path):
    res = import_urdf(_write(tmp_path, "<model name='x'/>"))
    assert res.model is None
    assert _codes(res.report, "error") == [ir.URDF_BAD_XML]


def test_inline_bad_number_is_bad_xml(tmp_path):
    text = """<robot name="bn"><link name="base"/><link name="a"/>
      <joint name="j1" type="revolute"><parent link="base"/><child link="a"/>
        <origin xyz="1 x 0"/><axis xyz="0 0 1"/>
        <limit lower="-1" upper="1" effort="1" velocity="1"/></joint>
    </robot>"""
    res = import_urdf(_write(tmp_path, text))
    assert res.model is None
    assert _codes(res.report, "error") == [ir.URDF_BAD_XML]


# ---------------------------------------------------------------------------
# Round trip
# ---------------------------------------------------------------------------

@pytest.mark.parametrize(
    "fixture",
    ["planar_2r.urdf", "serial_6dof.urdf", "gripper_mimic.urdf", "continuous_joint.urdf"],
)
def test_imported_model_round_trips_through_urm_json(fixture):
    m = import_urdf(_fx(fixture)).model
    assert m is not None
    s = cb.urm_to_json(m)
    m2 = cb.urm_from_json(s)
    assert cb.urm_to_json(m2) == s
