"""
tests/python/test_compiled_arrays.py
R4.1: compiled layout freeze. compile_arrays shapes, contents, doc match,
effective_links, joint_links delegation.
"""

import re
from pathlib import Path

import numpy as np
import pytest

import core_bridge as cb
from app.panels.model_panel import joint_links

DOC = Path(__file__).resolve().parents[2] / "docs" / "compiled-model-layout.md"

FIELDS = [
    "n_joints", "n_links", "n_dof", "tcp_link", "layout_version",
    "joint_index", "parent_link", "child_link", "type", "q_index",
    "src_q_index", "pre", "post", "axis", "mult", "offset", "base", "tool",
]


def dh_chain(n=3):
    """Serial DH robot with n revolute joints and n+1 links."""
    m = cb.RobotModel()
    for i in range(n + 1):
        lk = cb.Link()
        lk.name = f"link{i}"
        m.links.append(lk)
    for i in range(n):
        j = cb.Joint()
        j.name = f"j{i + 1}"
        j.type = cb.JointType.Revolute
        j.kinematics = cb.JointKinematics.DH
        j.parent_link = i
        j.child_link = i + 1
        j.dh.a = 0.3 + 0.1 * i
        j.dh.alpha = 0.2 * (i + 1)
        j.dh.d = 0.05 * i
        j.dh.theta = 0.1 * i
        m.joints.append(j)
    return m


def tree_model():
    """Origin/axis tree with a branch at link 1. TCP on link 2."""
    m = cb.RobotModel()
    for i in range(4):
        m.links.append(cb.Link())

    def oa(name, p, c, typ, xyz, axis):
        j = cb.Joint()
        j.name = name
        j.type = typ
        j.parent_link = p
        j.child_link = c
        j.kinematics = cb.JointKinematics.OriginAxis
        o = cb.Pose()
        o.xyz = xyz
        o.rpy = [0.1, 0.0, 0.0]
        j.origin = o
        j.axis = axis
        return j

    m.joints.append(oa("root", 0, 1, cb.JointType.Revolute, [0, 0, 0.2], [0, 0, 1]))
    m.joints.append(oa("b1", 1, 2, cb.JointType.Revolute, [0.5, 0, 0], [0, 1, 0]))
    m.joints.append(oa("b2", 1, 3, cb.JointType.Prismatic, [0, 0.3, 0], [1, 0, 0]))
    m.tcp_link = 2
    return m


def mimic_model():
    """Two revolute joints, second mimics the first (2 * q + 0.1)."""
    m = dh_chain(2)
    mm = cb.Mimic()
    mm.joint = "j1"
    mm.multiplier = 2.0
    mm.offset = 0.1
    m.joints[1].mimic = mm
    return m


def _motion(typ, axis, qe):
    T = np.eye(4)
    if typ == 0:  # Revolute: Rodrigues
        a = np.asarray(axis, float)
        a = a / np.linalg.norm(a)
        K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
        T[:3, :3] = np.eye(3) + np.sin(qe) * K + (1 - np.cos(qe)) * K @ K
    elif typ == 1:  # Prismatic
        T[:3, 3] = np.asarray(axis, float) * qe
    return T


def fk_from_arrays(a, q):
    """Independent numpy FK from the frozen arrays (no core_bridge FK call)."""
    frames = [np.asarray(a["base"], float)]
    frames += [None] * (int(a["n_links"]) - 1)
    for k in range(int(a["n_joints"])):
        typ = int(a["type"][k])
        if typ == 2:
            motion = np.eye(4)
        else:
            src = int(a["src_q_index"][k])
            qe = a["mult"][k] * q[src] + a["offset"][k]
            motion = _motion(typ, a["axis"][k], qe)
        A = a["pre"][k] @ motion @ a["post"][k]
        c = int(a["child_link"][k])
        p = int(a["parent_link"][k])
        frames[c] = frames[p] @ A
    tcp = frames[int(a["tcp_link"])] @ np.asarray(a["tool"], float)
    return frames, tcp


@pytest.mark.parametrize("make", [lambda: dh_chain(3), tree_model, mimic_model])
def test_shapes_and_dtypes(make):
    m = make()
    a = cb.compile_arrays(m)
    n = int(a["n_joints"])
    assert n == len(m.joints)
    assert int(a["layout_version"]) == 1
    assert int(a["n_links"]) == len(m.links)
    assert int(a["n_dof"]) == m.dof()
    for name in ["joint_index", "parent_link", "child_link", "type", "q_index", "src_q_index"]:
        assert a[name].shape == (n,), name
        assert a[name].dtype == np.int32, name
    for name in ["pre", "post"]:
        assert a[name].shape == (n, 4, 4), name
        assert a[name].dtype == np.float64, name
    assert a["axis"].shape == (n, 3) and a["axis"].dtype == np.float64
    for name in ["mult", "offset"]:
        assert a[name].shape == (n,) and a[name].dtype == np.float64, name
    for name in ["base", "tool"]:
        assert a[name].shape == (4, 4) and a[name].dtype == np.float64, name


@pytest.mark.parametrize("make", [lambda: dh_chain(3), tree_model, mimic_model])
def test_contents_match_compiled_fk(make):
    """Multiply pre*motion*post in numpy from the arrays; compare with fk_compiled."""
    m = make()
    a = cb.compile_arrays(m)
    cm = cb.compile(m)
    rng = np.random.default_rng(7)
    for _ in range(4):
        q = list(rng.uniform(-1.2, 1.2, size=m.dof()))
        frames_np, tcp_np = fk_from_arrays(a, q)
        res = cb.fk_compiled(cm, q)
        for k, f in enumerate(res["frames"]):
            np.testing.assert_allclose(frames_np[k], f, atol=1e-12)
        np.testing.assert_allclose(tcp_np, res["tcp"], atol=1e-12)


def test_compiled_order_is_child_sorted():
    m = tree_model()
    a = cb.compile_arrays(m)
    assert list(a["child_link"]) == sorted(a["child_link"])


def test_mimic_fields():
    a = cb.compile_arrays(mimic_model())
    # Joint 1 (compiled k=1) is driven by DOF 0 with mult 2, offset 0.1.
    assert int(a["q_index"][1]) == -1
    assert int(a["src_q_index"][1]) == 0
    assert a["mult"][1] == pytest.approx(2.0)
    assert a["offset"][1] == pytest.approx(0.1)
    assert a["mult"][0] == pytest.approx(1.0) and a["offset"][0] == pytest.approx(0.0)


def test_type_enum_values():
    m = tree_model()
    a = cb.compile_arrays(m)
    assert list(a["type"]) == [0, 0, 1]  # Revolute, Revolute, Prismatic


def test_doc_matches_code():
    text = DOC.read_text(encoding="utf-8")
    for name in FIELDS:
        assert f"`{name}`" in text, f"field {name} missing in docs/compiled-model-layout.md"
    assert "layout_version" in text


def test_effective_links_normal_chain():
    m = dh_chain(3)
    assert cb.effective_links(m) == [(0, 1), (1, 2), (2, 3)]


def test_effective_links_tree():
    m = tree_model()
    assert cb.effective_links(m) == [(0, 1), (1, 2), (1, 3)]


def test_effective_links_legacy():
    """Legacy R2 serial data: every joint 0/0, links = joints + 1 -> (i, i+1)."""
    m = cb.RobotModel()
    for i in range(3):
        m.links.append(cb.Link())
    for i in range(2):
        j = cb.Joint()
        j.name = f"j{i + 1}"
        j.type = cb.JointType.Revolute
        m.joints.append(j)
    assert cb.effective_links(m) == [(0, 1), (1, 2)]
    assert joint_links(m) == [(0, 1), (1, 2)]


def test_joint_links_delegates_normal_and_tree():
    assert joint_links(dh_chain(2)) == [(0, 1), (1, 2)]
    assert joint_links(tree_model()) == [(0, 1), (1, 2), (1, 3)]


def test_legacy_link_count_mismatch_pure_links_compile_throws():
    """effective_links is pure; compile() rejects a legacy model with bad link count."""
    m = cb.RobotModel()
    for _ in range(2):
        m.links.append(cb.Link())
    for i in range(2):
        j = cb.Joint()
        j.name = f"j{i + 1}"
        j.type = cb.JointType.Revolute
        m.joints.append(j)
    assert cb.effective_links(m) == [(0, 1), (1, 2)]
    with pytest.raises(ValueError):
        cb.compile(m)
    with pytest.raises(ValueError):
        cb.compile_arrays(m)
