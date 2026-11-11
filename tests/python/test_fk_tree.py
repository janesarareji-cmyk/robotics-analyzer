"""R3.2: compiled model + tree FK Python tests."""
import math
import numpy as np
import pytest
import core_bridge as cb


def _oa(name, p, c, xyz, axis=(0, 0, 1)):
    j = cb.Joint()
    j.name = name
    j.parent_link, j.child_link = p, c
    j.type = cb.JointType.Revolute
    j.kinematics = cb.JointKinematics.OriginAxis
    o = cb.Pose()
    o.xyz = list(xyz)
    j.origin = o
    j.axis = list(axis)
    return j


def _tree():
    m = cb.RobotModel()
    for _ in range(4):
        m.links.append(cb.Link())
    m.joints.append(_oa("j1", 0, 1, (1, 0, 0)))
    m.joints.append(_oa("j2", 0, 2, (0, 1, 0)))
    m.joints.append(_oa("j3", 1, 3, (0.5, 0, 0)))
    return m


def test_fk_returns_active_joints():
    m = _tree()
    m.tcp_link = 3
    res = cb.fk(m, [math.pi / 2, 0.0, 0.0])
    assert res["active_joints"] == [0, 2]
    assert np.allclose(res["frames"][3][:3, 3], [1, 0.5, 0])
    assert cb.active_chain(m) == [0, 2]
    m.tcp_link = 2
    assert cb.active_chain(m) == [1]


def test_wrong_q_size_value_error():
    with pytest.raises(ValueError):
        cb.fk(_tree(), [0.0])


def test_compile_error_value_error():
    m = _tree()
    m.joints[0].child_link = 9
    with pytest.raises(ValueError):
        cb.compile(m)


def test_fk_compiled_equals_fk():
    m = _tree()
    m.tcp_link = 3
    q = [0.3, -0.2, 0.5]
    cm = cb.compile(m)
    a, b = cb.fk(m, q), cb.fk_compiled(cm, q)
    assert a["active_joints"] == b["active_joints"]
    assert np.allclose(a["tcp"], b["tcp"], atol=1e-12)
    for fa, fb in zip(a["frames"], b["frames"]):
        assert np.allclose(fa, fb, atol=1e-12)


def test_legacy_default_links_serial():
    m = cb.RobotModel()
    for _ in range(3):
        m.links.append(cb.Link())
    for i in range(2):
        j = cb.Joint()
        j.name = f"j{i}"
        j.dh.a = 1.0
        m.joints.append(j)
    res = cb.fk(m, [0.0, 0.0])
    assert np.allclose(res["tcp"][:3, 3], [2, 0, 0])
    assert res["active_joints"] == [0, 1]


def test_validate_mimic_of_fixed():
    m = _tree()
    m.joints[0].type = cb.JointType.Fixed
    mm = cb.Mimic()
    mm.joint = "j1"
    m.joints[1].mimic = mm
    assert any("fixed joint" in e for e in m.validate())
    with pytest.raises(ValueError):
        cb.compile(m)
