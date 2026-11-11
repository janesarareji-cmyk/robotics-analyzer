"""
tests/python/test_milestone_r4.py
R4 stage milestone: IK, singularity, dynamics and energy on the 6-DOF fixture,
driven through an offscreen MainWindow with the real core_bridge.

Expected values come from independent numpy code: the URDF-parsing FK of the
R3 milestone (``NumpyRobot``) and ``numpy_ref`` (numeric Jacobian, Lagrangian
dynamics). ``cb.fk`` / ``cb.jacobian`` / ``cb.inverse_dynamics`` are never used
to produce an expected value. Spec item 8 (Pinocchio) is not done on purpose.
"""

import math
import os
import shutil
import unittest.mock as mock
from pathlib import Path

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication

import core_bridge as cb
from app.dyn_utils import integrate_rk4_forward_dynamics
from app.kin_utils import current_q, write_q
from app.main_window import MainWindow
from app.panels.joint_panel import rotation_to_rpy

from .numpy_ref import (inertias_from_model, numeric_jacobian_tcp, numpy_dynamics,
                        spec_from_model)
from .test_milestone_r3 import NumpyRobot

FX = Path(__file__).parent / "fixtures" / "urdf"
URDF = "six_dof_with_gripper.urdf"
Q_TRUE = [0.3, -0.5, 0.8, 0.4, -0.7, 1.1]       # joints j1..j6
Q_GENERIC = [0.3, 0.5, 0.4, 0.2, 0.6, 0.3, 0.0]  # 7 DOF (last = finger)
Q_WRIST = [0.3, 0.5, 0.4, 0.2, -2.305417, 0.3, 0.0]  # sigma_min ~ 1e-9 (R4.9 scan)
G = [0.0, 0.0, -9.81]


@pytest.fixture
def win(qapp):
    w = MainWindow()
    w.show()
    yield w
    w.project_controller._is_modified = False
    w.ik_dock.hide()                 # window layout is persisted on close
    w.analysis_dock.hide()
    w.close()


@pytest.fixture(scope="module")
def npr():
    return NumpyRobot(FX / URDF)


def _pump(win):
    win.task_runner.wait_all(10000)
    QApplication.processEvents()


def _load(win):
    """Spec item 1: import the fixture and set the TCP link to l6."""
    with mock.patch("app.main_window.QMessageBox.warning") as w:
        win.import_urdf_file(str(FX / URDF))
        _pump(win)
    assert w.call_count == 0
    model = win.project_controller.robot
    names = [lk.name for lk in model.links]
    win.set_tcp_link(names.index("l6"))
    win.project_controller._is_modified = False
    assert names[model.tcp_link] == "l6"
    return model


def _set_q(win, q):
    model = win.project_controller.robot
    write_q(model, q)
    win.joint_panel.set_model(model, save_baseline=False)
    win._on_joint_q_changed()
    win.project_controller._is_modified = False


def _qdict(q6):
    return {f"j{i + 1}": v for i, v in enumerate(q6)}


def _tcp_np(npr, model):
    """Independent numpy TCP (frame of l6) at the model's current q."""
    q = {j.name: j.q for j in model.joints}
    return npr.frames(q)["l6"]


def _fill(ik, T):
    r, p, y = rotation_to_rpy(T[:3, :3])
    ik.set_target(T[:3, 3], [math.degrees(r), math.degrees(p), math.degrees(y)])


def _pose_err(T, T_target):
    dp = np.linalg.norm(T[:3, 3] - T_target[:3, 3])
    dR = np.abs(T[:3, :3] - T_target[:3, :3]).max()
    return dp, dR


def _set_fields(fields, values):
    for f, v in zip(fields, values):
        f.setText(repr(float(v)))


def _gravity_setting(win, g):
    win.project_controller.current_project.simulation_settings.gravity = list(g)
    win.analysis_panel.refresh()


def _numpy_tau(model, q, qd, qdd, g):
    spec = spec_from_model(model)
    M, c, grav = numpy_dynamics(spec, inertias_from_model(model), q, qd, g)
    return M @ np.asarray(qdd) + c + grav, grav


# ------------------------------------------------------------------ 1
def test_fixture_import_and_tcp(win, npr):
    model = _load(win)
    assert model.dof() == 7
    assert [lk.name for lk in model.links] == npr.links
    assert all(lk.inertia.mass > 0 for lk in model.links)


# ------------------------------------------------------------------ 2 and 3
def test_ik_solutions_listed_sorted_applied(win, npr):
    model = _load(win)
    ik = win.ik_panel
    _set_q(win, [0.0] * 7)
    # target = independent numpy FK of q_true
    T_target = npr.frames({**_qdict(Q_TRUE), "jf_l": 0.0})["l6"]
    _fill(ik, T_target)
    ik.starts_spin.setValue(40)
    ik.solve_button.click()
    assert ik.is_running
    _pump(win)
    sols = ik.solutions
    assert len(sols) >= 1 and ik.table.rowCount() == len(sols)

    # 3. sorted ascending by distance, every solution passes the numpy FK check
    dist = [s["distance"] for s in sols]
    assert dist == sorted(dist)
    for s in sols:
        q = {f"j{i + 1}": s["q"][i] for i in range(6)}
        q["jf_l"] = s["q"][6]
        T = npr.frames(q)["l6"]
        dp, dR = _pose_err(T, T_target)
        assert dp < 1e-5 and dR < 1e-5
    # solutions are pairwise distinct
    for a in range(len(sols)):
        for b in range(a + 1, len(sols)):
            assert np.abs(np.asarray(sols[a]["q"]) - np.asarray(sols[b]["q"])).max() >= 1e-3

    # 2. select the first row (preview), apply: TCP equals target (numpy FK)
    ik.table.selectRow(0)
    QApplication.processEvents()
    assert ik.is_previewing and not win.project_controller.is_modified
    ik.apply_button.click()
    assert win.project_controller.is_modified
    q_applied = current_q(model)
    assert q_applied == pytest.approx(list(sols[0]["q"]))
    T = _tcp_np(npr, model)
    dp, dR = _pose_err(T, T_target)
    assert dp < 1e-5 and dR < 1e-5
    # the viewport TCP matches too
    np.testing.assert_allclose(np.asarray(win.robot_view.tcp_actor.user_matrix), T, atol=1e-5)


# ------------------------------------------------------------------ 4
def _numpy_sigma(model):
    J = numeric_jacobian_tcp(spec_from_model(model), current_q(model))
    J = J[:, np.linalg.norm(J, axis=0) > 1e-12]       # drop off-chain DOF columns
    return np.linalg.svd(J, compute_uv=False)


def test_singularity_flag_and_sigma_min(win):
    model = _load(win)
    ap = win.analysis_panel

    _set_q(win, Q_GENERIC)
    assert ap.near_singular_text == "ok"
    s = _numpy_sigma(model)
    assert s[-1] > 1e-3
    assert float(ap.sigma_min_text) == pytest.approx(s[-1], rel=1e-3)
    assert float(ap.condition_text) == pytest.approx(s[0] / s[-1], rel=1e-3)
    info = cb.analyze_singularity(cb.jacobian_tcp(cb.compile(model), current_q(model)))
    assert info["sigma_min"] == pytest.approx(s[-1], abs=1e-5)
    assert info["sigma_max"] == pytest.approx(s[0], abs=1e-5)

    _set_q(win, Q_WRIST)
    assert ap.near_singular_text == "NEAR SINGULAR"
    s = _numpy_sigma(model)
    assert s[-1] < 1e-3
    info = cb.analyze_singularity(cb.jacobian_tcp(cb.compile(model), current_q(model)))
    assert info["sigma_min"] == pytest.approx(s[-1], abs=1e-5)
    assert info["near_singular"]

    _set_q(win, Q_GENERIC)
    assert ap.near_singular_text == "ok"


# ------------------------------------------------------------------ 5
def test_torques_equal_numpy_lagrangian_and_gravity_is_linear(win):
    model = _load(win)
    ap = win.analysis_panel
    _set_q(win, Q_GENERIC)
    n = model.dof()
    qd = [0.1 * (k + 1) - 0.3 for k in range(n)]
    qdd = [0.05 * (k - 2) for k in range(n)]
    _set_fields(ap.qd_fields, qd)
    _set_fields(ap.qdd_fields, qdd)
    q = current_q(model)

    tau_np, _ = _numpy_tau(model, q, qd, qdd, G)
    tau = np.asarray(ap.tau)
    scale = np.abs(tau_np).max()
    assert np.abs(tau - tau_np).max() <= 1e-8 * scale
    assert tau == pytest.approx(tau_np, rel=1e-8, abs=1e-8 * scale)
    # the table shows the same numbers (printed to 5 significant digits)
    for k in range(n):
        assert float(ap.torque_table.item(k, 5).text()) == pytest.approx(
            tau_np[k], rel=1e-4, abs=1e-4)

    # gravity linearity: qd = qdd = 0, torques at g and 2g differ by the gravity torque
    _set_fields(ap.qd_fields, [0.0] * n)
    _set_fields(ap.qdd_fields, [0.0] * n)
    _gravity_setting(win, [0.0, 0.0, -9.81])
    t1 = np.asarray(ap.tau)
    _gravity_setting(win, [0.0, 0.0, -19.62])
    t2 = np.asarray(ap.tau)
    _, grav_np = _numpy_tau(model, q, [0.0] * n, [0.0] * n, G)
    gs = np.abs(grav_np).max()
    assert np.abs((t2 - t1) - grav_np).max() <= 1e-8 * gs       # t(2g) - t(g) = gravity torque
    assert np.abs(t1 - grav_np).max() <= 1e-8 * gs             # t(g) = gravity torque
    assert np.abs(t2 - 2.0 * t1).max() <= 1e-10 * gs            # linear in g
    _gravity_setting(win, [0.0, 0.0, 0.0])
    assert np.asarray(ap.tau) == pytest.approx(np.zeros(n), abs=1e-12)


# ------------------------------------------------------------------ 6
def test_forward_dynamics_round_trip_and_energy_conservation(win):
    model = _load(win)
    cm = cb.compile(model)
    dm = cb.build_dyn_model(model, cm)
    q = list(Q_GENERIC)
    n = model.dof()
    rng = np.random.default_rng(404)
    qd = list(rng.normal(size=n) * 0.5)
    tau = rng.normal(size=n) * 2.0

    qdd = cb.forward_dynamics(dm, q, qd, list(tau), tuple(G))
    tau_back = np.asarray(cb.inverse_dynamics(dm, q, qd, list(qdd), tuple(G)))
    assert np.abs(tau_back - tau).max() <= 1e-10 * max(1.0, np.abs(tau).max())

    # energy conservation: tau = 0, RK4, 0.5 s
    q0 = list(Q_GENERIC)
    qd0 = [0.4, -0.3, 0.5, 0.2, -0.4, 0.3, 0.1]
    steps, dt = 5000, 1e-4
    qh, qdh = integrate_rk4_forward_dynamics(dm, q0, qd0, lambda t, qq, vv: [0.0] * n,
                                             dt, steps, G)
    assert qh.shape == (steps + 1, n)
    e0 = cb.total_energy(dm, q0, qd0, tuple(G))
    e1 = cb.total_energy(dm, list(qh[-1]), list(qdh[-1]), tuple(G))
    ke0 = cb.kinetic_energy(dm, q0, qd0)
    assert ke0 > 1e-3
    ref = max(abs(e0), ke0)
    assert abs(e1 - e0) / ref < 1e-8
    # the arm really moved
    assert np.abs(qh[-1] - np.asarray(q0)).max() > 1e-3


# ------------------------------------------------------------------ 7
def _snapshot(win):
    ap = win.analysis_panel
    return {
        "tau": list(ap.tau),
        "sigma": (ap.sigma_min_text, ap.condition_text, ap.manipulability_text,
                  ap.near_singular_text),
        "energy": ap.energy_texts,
        "table": [[ap.torque_table.item(r, c).text() for c in range(ap.torque_table.columnCount())]
                  for r in range(ap.torque_table.rowCount())],
    }


def test_save_close_open_keeps_model_and_analysis(win, tmp_path):
    model = _load(win)
    pc = win.project_controller
    _set_q(win, Q_GENERIC)
    ap = win.analysis_panel
    n = model.dof()
    # qd / qdd entries are session state (not saved): leave them at the default 0
    before = _snapshot(win)
    json_before = cb.urm_to_json(model)
    tcp_before = model.tcp_link
    inertia_before = [(lk.inertia.mass, list(lk.inertia.com), list(lk.inertia.inertia_tensor))
                      for lk in model.links]
    g_before = list(pc.current_project.simulation_settings.gravity)
    assert before["tau"] and len(before["tau"]) == n

    out = tmp_path / "r4.robotproj"
    assert pc.save_project_as(win, str(out))
    assert pc.close_project(None)
    assert not pc.has_project
    assert ap.torque_table.rowCount() == 0
    assert pc.open_project(win, str(out))

    r2 = pc.robot
    assert cb.urm_to_json(r2) == json_before
    assert r2.tcp_link == tcp_before and r2.links[r2.tcp_link].name == "l6"
    assert [(lk.inertia.mass, list(lk.inertia.com), list(lk.inertia.inertia_tensor))
            for lk in r2.links] == inertia_before
    assert list(pc.current_project.simulation_settings.gravity) == g_before
    after = _snapshot(win)
    assert after == before
