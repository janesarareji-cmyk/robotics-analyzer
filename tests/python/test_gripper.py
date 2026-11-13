"""U2.4 gripper open/close."""

import os

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtCore import QCoreApplication

import core_bridge as cb
from app.gripper import gripper_dofs
from app.importers.urdf import import_urdf
from app.kin_utils import current_q
from app.main_window import MainWindow
from app.sample_robots import list_sample_robots
from app.settings import AppSettings

SAMPLES = {p.parent.name: (t, p) for t, p in list_sample_robots()}


def _model(name):
    return import_urdf(str(SAMPLES[name][1])).model


def test_detection_on_all_samples():
    found = {n: gripper_dofs(_model(n)) for n in SAMPLES}
    assert len(SAMPLES) == 9
    for n, g in found.items():
        if n in ("xarm5_gripper", "panda"):
            assert len(g) >= 1, n
        else:
            assert g == [], n
    g = found["xarm5_gripper"]
    assert [x.name for x in g] == ["drive_joint"]
    assert g[0].open_value == 0.0 and g[0].closed_value == pytest.approx(0.85)
    g = found["panda"]
    assert [x.name for x in g] == ["panda_finger_joint1"]
    assert g[0].open_value == pytest.approx(0.04) and g[0].closed_value == 0.0


@pytest.fixture
def win(qapp, monkeypatch):
    monkeypatch.setattr(AppSettings, "_ORGANIZATION", "RoboticsAnalyzer_test_u2g")
    monkeypatch.setattr(AppSettings, "_APPLICATION", "RoboticsAnalyzer_test_u2g")
    w = MainWindow()
    w.settings._settings.clear()
    yield w
    w.project_controller._is_modified = False
    w.close()


def _load(win, name):
    path = SAMPLES[name][1]
    res = import_urdf(str(path))
    win._import_path = str(path)
    win._on_urdf_import_result(res)
    QCoreApplication.processEvents()


@pytest.mark.parametrize("name", ["xarm5_gripper", "panda"])
def test_open_close_buttons_move_dof_and_fk(win, name):
    _load(win, name)
    jp = win.joint_panel
    model = win.project_controller.robot
    g = gripper_dofs(model)[0]
    assert not jp.gripper_group.isHidden()
    jp._btn_g_close.click()
    assert current_q(model)[g.dof_index] == pytest.approx(g.closed_value)
    fk_closed = cb.fk(model)
    jp._btn_g_open.click()
    assert current_q(model)[g.dof_index] == pytest.approx(g.open_value)
    fk_open = cb.fk(model)
    assert any(not np.allclose(a, b) for a, b in zip(fk_open["frames"], fk_closed["frames"]))
    jp._g_slider.setValue(50)
    assert current_q(model)[g.dof_index] == pytest.approx(0.5 * (g.open_value + g.closed_value))
    # row widget follows
    row = [r for r in jp._all_rows() if r.joint_index == g.joint_index][0]
    assert row.spinbox.value() * row.to_model == pytest.approx(current_q(model)[g.dof_index], abs=1e-3)


def test_finger_link_moves_with_close():
    model = _model("panda")
    g = gripper_dofs(model)[0]
    model.joints[g.joint_index].q = g.open_value
    fa = cb.fk(model)
    model.joints[g.joint_index].q = g.closed_value
    fb = cb.fk(model)
    keys = [k for k in fa if np.asarray(fa[k]).ndim >= 2]
    assert keys
    assert any(not np.allclose(np.asarray(fa[k]), np.asarray(fb[k])) for k in keys)


def test_arm_only_has_no_group(win):
    _load(win, "ur5")
    assert win.joint_panel.gripper_group.isHidden()
