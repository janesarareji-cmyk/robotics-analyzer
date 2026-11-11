"""R5.2: CPU batch FK (cb.fk_batch). No GPU needed."""
import numpy as np
import pytest

import core_bridge as cb
from .robots import fk_test_robots, sample_q, random_robot

ROBOTS = fk_test_robots()


@pytest.mark.parametrize("name", list(ROBOTS))
def test_matches_fk_compiled_1000(name):
    m = ROBOTS[name]
    cm = cb.compile(m)
    q = sample_q(m, 1000)
    res = cb.fk_batch(cm, q)
    assert res["position"].shape == (1000, 3)
    assert res["rotation"].shape == (1000, 3, 3)
    for i in range(1000):
        T = cb.fk_compiled(cm, list(q[i]))["tcp"]
        assert np.abs(res["position"][i] - T[:3, 3]).max() < 1e-12
        assert np.abs(res["rotation"][i] - T[:3, :3]).max() < 1e-12


@pytest.mark.parametrize("name", list(ROBOTS))
def test_base_frame_is_world_without_base(name):
    m = ROBOTS[name]
    cm = cb.compile(m)
    base = cb.compile_arrays(m)["base"]
    q = sample_q(m, 200, seed=3)
    w = cb.fk_batch(cm, q, frame="world")
    b = cb.fk_batch(cm, q, frame="base")
    pos = b["position"] @ base[:3, :3].T + base[:3, 3]
    rot = np.einsum("ab,nbc->nac", base[:3, :3], b["rotation"])
    assert np.abs(pos - w["position"]).max() < 1e-12
    assert np.abs(rot - w["rotation"]).max() < 1e-12


def test_thread_count_does_not_change_result():
    m = ROBOTS["origin_axis_12r"]
    cm = cb.compile(m)
    q = sample_q(m, 50_000, seed=5)
    a = cb.fk_batch(cm, q, n_threads=1)
    b = cb.fk_batch(cm, q, n_threads=0)
    c = cb.fk_batch(cm, q, n_threads=7)
    assert np.array_equal(a["position"], b["position"])
    assert np.array_equal(a["rotation"], c["rotation"])


def test_empty_batch():
    m = ROBOTS["dh_6r"]
    cm = cb.compile(m)
    res = cb.fk_batch(cm, np.zeros((0, 6)))
    assert res["position"].shape == (0, 3)
    assert res["rotation"].shape == (0, 3, 3)


def test_single_sample():
    m = ROBOTS["dh_6r"]
    cm = cb.compile(m)
    q = sample_q(m, 1)
    res = cb.fk_batch(cm, q)
    T = cb.fk_compiled(cm, list(q[0]))["tcp"]
    assert np.abs(res["position"][0] - T[:3, 3]).max() < 1e-12


def test_errors():
    m = ROBOTS["dh_6r"]
    cm = cb.compile(m)
    with pytest.raises(ValueError):
        cb.fk_batch(cm, np.zeros((4, 5)))
    with pytest.raises(ValueError):
        cb.fk_batch(cm, np.zeros(6))
    with pytest.raises(ValueError):
        cb.fk_batch(cm, np.zeros((4, 6)), frame="tool")


def test_zero_dof_robot_returns_tool_pose():
    m = random_robot(21, 3)
    for j in m.joints:
        j.type = cb.JointType.Fixed
    cm = cb.compile(m)
    res = cb.fk_batch(cm, np.zeros((5, 0)))
    T = cb.fk_compiled(cm, [])["tcp"]
    assert np.abs(res["position"] - T[:3, 3]).max() < 1e-12
