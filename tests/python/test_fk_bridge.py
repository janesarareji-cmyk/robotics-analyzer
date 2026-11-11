"""
tests/python/test_fk_bridge.py
R2.5: Forward Kinematics Python bridge unit tests.
"""

import math
import numpy as np
import pytest

import core_bridge as cb


def test_fk_return_dict_structure():
    """fk(model) returns dict with frames and tcp ndarrays of shape (4,4) float64."""
    model = cb.RobotModel()
    model.links.append(cb.Link())
    for i in range(3):
        j = cb.Joint()
        j.name = f"j{i+1}"
        j.type = cb.JointType.Revolute
        j.dh.a = 0.5
        model.joints.append(j)
        model.links.append(cb.Link())

    res = cb.fk(model)
    assert isinstance(res, dict)
    assert "frames" in res
    assert "tcp" in res

    frames = res["frames"]
    assert isinstance(frames, list)
    assert len(frames) == 4  # joints + 1

    for idx, f in enumerate(frames):
        assert isinstance(f, np.ndarray), f"frame {idx} is not ndarray"
        assert f.shape == (4, 4), f"frame {idx} shape is not (4,4)"
        assert f.dtype == np.float64, f"frame {idx} dtype is not float64"
        np.testing.assert_allclose(f[3, :], [0.0, 0.0, 0.0, 1.0], atol=1e-12)

    tcp = res["tcp"]
    assert isinstance(tcp, np.ndarray)
    assert tcp.shape == (4, 4)
    assert tcp.dtype == np.float64
    np.testing.assert_allclose(tcp[3, :], [0.0, 0.0, 0.0, 1.0], atol=1e-12)


def test_fk_2r_planar():
    """fk matches analytical 2R planar arm results."""
    model = cb.RobotModel()
    model.links.append(cb.Link())
    for i in range(2):
        j = cb.Joint()
        j.name = f"j{i+1}"
        j.type = cb.JointType.Revolute
        j.dh.a = 1.0
        model.joints.append(j)
        model.links.append(cb.Link())

    # q = (0, 0) -> TCP = (2, 0, 0)
    res0 = cb.fk(model, [0.0, 0.0])
    np.testing.assert_allclose(res0["tcp"][:3, 3], [2.0, 0.0, 0.0], atol=1e-12)

    # q = (π/2, -π/2) -> TCP = (1, 1, 0)
    res1 = cb.fk(model, [math.pi / 2.0, -math.pi / 2.0])
    np.testing.assert_allclose(res1["tcp"][:3, 3], [1.0, 1.0, 0.0], atol=1e-12)


def test_fk_wrong_length_raises_value_error():
    """fk(model, q) with wrong q length raises ValueError."""
    model = cb.RobotModel()
    model.links.append(cb.Link())
    for i in range(2):
        j = cb.Joint()
        j.name = f"j{i+1}"
        j.type = cb.JointType.Revolute
        model.joints.append(j)
        model.links.append(cb.Link())

    with pytest.raises(ValueError, match="q size mismatch"):
        cb.fk(model, [0.1])

    with pytest.raises(ValueError, match="q size mismatch"):
        cb.fk(model, [0.1, 0.2, 0.3])


def test_pose_to_iso():
    """pose_to_iso returns 4x4 matrix with valid homogeneous row."""
    pose = cb.Pose()
    pose.xyz = [1.0, 2.0, 3.0]
    pose.rpy = [0.1, 0.2, 0.3]

    mat = cb.pose_to_iso(pose)
    assert isinstance(mat, np.ndarray)
    assert mat.shape == (4, 4)
    assert mat.dtype == np.float64
    np.testing.assert_allclose(mat[:3, 3], [1.0, 2.0, 3.0], atol=1e-12)
    np.testing.assert_allclose(mat[3, :], [0.0, 0.0, 0.0, 1.0], atol=1e-12)
