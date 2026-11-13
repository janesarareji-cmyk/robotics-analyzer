"""
tests/python/test_ui_kin_dyn.py
R4.9: IK panel, Analysis panel, target triad, View-menu toggles (offscreen).
"""

import math
import os
import unittest.mock as mock
from pathlib import Path

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication

import core_bridge as cb
from app.kin_utils import current_q, write_q
from app.main_window import MainWindow
from app.panels.analysis_panel import MASS_WARNING
from app.panels.ik_panel import NO_SOLUTION_TEXT, target_iso
from app.panels.joint_panel import rotation_to_rpy
from app.panels.num_field import NumField

FX = Path(__file__).parent / "fixtures" / "urdf"
URDF = "six_dof_with_gripper.urdf"
Q_GENERIC = [0.3, 0.5, 0.4, 0.2, 0.6, 0.3, 0.0]
Q_WRIST = [0.3, 0.5, 0.4, 0.2, -2.305417, 0.3, 0.0]   # sigma_min ~ 1e-9 (scanned)


@pytest.fixture
def win(qapp):
    w = MainWindow()
    w.collision_service.enabled = False         # these tests move through poses; U3 has its own
    w.show()
    yield w
    w.project_controller._is_modified = False
    # window layout is persisted on close: leave the new docks hidden
    w.ik_dock.hide()
    w.analysis_dock.hide()
    w.close()


def _pump(win):
    win.task_runner.wait_all(10000)
    QApplication.processEvents()


def _load(win):
    """Import the 6-DOF fixture and make l6 the TCP."""
    with mock.patch("app.main_window.QMessageBox.warning") as w:
        win.import_urdf_file(str(FX / URDF))
        _pump(win)
    assert w.call_count == 0
    model = win.project_controller.robot
    names = [lk.name for lk in model.links]
    win.set_tcp_link(names.index("l6"))
    win.project_controller._is_modified = False
    return model


def _set_q(win, q):
    model = win.project_controller.robot
    write_q(model, q)
    win.joint_panel.set_model(model, save_baseline=False)
    win._on_joint_q_changed()
    win.project_controller._is_modified = False


def _tcp(model, q=None):
    return np.asarray(cb.fk(model)["tcp"] if q is None else cb.fk(model, q)["tcp"])


def _fill_from_pose(ik, T):
    r, p, y = rotation_to_rpy(T[:3, :3])
    ik.set_target(T[:3, 3], [math.degrees(r), math.degrees(p), math.degrees(y)])


def _solve(win):
    win.ik_panel.solve_button.click()
    _pump(win)


# ---------------------------------------------------------------- creation
def test_panels_hidden_at_start_and_toggle(win):
    assert win.ik_panel is not None and win.analysis_panel is not None
    assert win.ik_dock.windowTitle() == "Inverse Kinematics"
    assert win.analysis_dock.windowTitle() == "Analysis"
    assert not win.ik_dock.isVisible()
    assert not win.analysis_dock.isVisible()
    assert win._action_view_ik.isCheckable() and not win._action_view_ik.isChecked()
    assert win._action_view_analysis.isCheckable() and not win._action_view_analysis.isChecked()
    win._action_view_ik.trigger()
    win._action_view_analysis.trigger()
    QApplication.processEvents()
    assert win.ik_dock.isVisible() and win.analysis_dock.isVisible()
    # IK is open but covered by the Analysis tab: a click raises it (U1-fix)
    win._action_view_ik.trigger()
    QApplication.processEvents()
    assert win.ik_dock.isVisible() and win._action_view_ik.isChecked()
    win._action_view_ik.trigger()   # now current: a click hides it
    QApplication.processEvents()
    assert not win.ik_dock.isVisible()


def test_panels_inert_without_project(win):
    ik = win.ik_panel
    assert not ik.solve_button.isEnabled()
    assert not ik.apply_button.isEnabled()
    assert win.analysis_panel.torque_table.rowCount() == 0
    ik.solve()          # no crash, no task
    assert not ik.is_running


# ---------------------------------------------------------------- IK panel
def test_use_current_tcp_fills_fields(win):
    model = _load(win)
    _set_q(win, Q_GENERIC)
    win.ik_panel.tcp_button.click()
    T = _tcp(model)
    r, p, y = rotation_to_rpy(T[:3, :3])
    vals = [f.value() for f in win.ik_panel.target_fields]
    assert vals[:3] == pytest.approx(list(T[:3, 3]), abs=1e-8)
    assert vals[3:] == pytest.approx([math.degrees(r), math.degrees(p), math.degrees(y)],
                                     abs=1e-6)


def test_target_triad_follows_fields_and_clears(win):
    _load(win)
    rv = win.robot_view
    assert rv.target_actor is None
    win.ik_panel.tcp_button.click()
    assert rv.target_actor is not None
    win.ik_panel.target_fields[0].set_value(0.1)
    m = np.asarray(rv.target_actor.user_matrix)
    assert m[0, 3] == pytest.approx(0.1, abs=1e-9)
    rv.set_target_pose(None)
    assert rv.target_actor is None


def test_solve_reachable_lists_solutions_and_ui_not_blocked(win):
    model = _load(win)
    ik = win.ik_panel
    T = _tcp(model, Q_GENERIC)
    _fill_from_pose(ik, T)
    ik.solve_button.click()
    # handler returned before the result: button disabled, nothing listed yet
    assert ik.is_running
    assert not ik.solve_button.isEnabled()
    assert ik.table.rowCount() == 0
    _pump(win)
    assert not ik.is_running
    assert ik.solve_button.isEnabled()
    assert ik.table.rowCount() >= 1
    assert ik.table.rowCount() == len(ik.solutions)
    assert ik.status_text != NO_SOLUTION_TEXT


def test_preview_reaches_target_and_cancel_restores(win):
    model = _load(win)
    ik = win.ik_panel
    q0 = list(Q_GENERIC)
    _set_q(win, [0.0] * 7)
    T = _tcp(model, q0)
    _fill_from_pose(ik, T)
    _solve(win)
    assert ik.table.rowCount() >= 1
    q_before = current_q(model)
    assert q_before == pytest.approx([0.0] * 7)

    ik.table.selectRow(0)
    QApplication.processEvents()
    assert ik.is_previewing
    assert _tcp(model)[:3, 3] == pytest.approx(T[:3, 3], abs=1e-5)
    assert np.allclose(_tcp(model)[:3, :3], T[:3, :3], atol=1e-5)
    # joint panel rows show the previewed values, project not dirty
    q_prev = current_q(model)
    row0 = win.joint_panel.rows[0]
    assert row0.spinbox.value() == pytest.approx(math.degrees(q_prev[0]), abs=0.01)
    assert not win.project_controller.is_modified
    # viewport moved to the preview TCP
    assert np.allclose(np.asarray(win.robot_view.tcp_actor.user_matrix), _tcp(model), atol=1e-9)

    ik.cancel_button.click()
    assert current_q(model) == pytest.approx(q_before)
    assert not ik.is_previewing
    assert not win.project_controller.is_modified
    assert np.allclose(np.asarray(win.robot_view.tcp_actor.user_matrix), _tcp(model), atol=1e-9)


def test_apply_commits_and_marks_dirty(win):
    model = _load(win)
    ik = win.ik_panel
    _set_q(win, [0.0] * 7)
    T = _tcp(model, Q_GENERIC)
    _fill_from_pose(ik, T)
    _solve(win)
    assert not ik.apply_button.isEnabled()      # nothing selected yet
    ik.table.selectRow(0)
    QApplication.processEvents()
    assert ik.apply_button.isEnabled()
    q_sol = list(ik.solutions[0]["q"])
    assert not win.project_controller.is_modified
    ik.apply_button.click()
    assert win.project_controller.is_modified
    assert current_q(model) == pytest.approx(q_sol)
    assert not ik.is_previewing
    # Cancel after Apply must not undo the commit
    ik.cancel_button.click()
    assert current_q(model) == pytest.approx(q_sol)


def test_unreachable_target_shows_message_no_popup(win):
    _load(win)
    ik = win.ik_panel
    ik.set_target([5.0, 5.0, 5.0], [0.0, 0.0, 0.0])
    with mock.patch("app.main_window.QMessageBox.warning") as w1, \
            mock.patch("PySide6.QtWidgets.QMessageBox.exec") as w2:
        ik.starts_spin.setValue(5)
        _solve(win)
    assert w1.call_count == 0 and w2.call_count == 0
    assert ik.table.rowCount() == 0
    assert ik.status_text == NO_SOLUTION_TEXT
    assert not ik.apply_button.isEnabled()


def test_position_only_option_reaches_position(win):
    model = _load(win)
    ik = win.ik_panel
    _set_q(win, [0.0] * 7)
    T = _tcp(model, Q_GENERIC)
    ik.set_target(T[:3, 3], [90.0, 0.0, 0.0])       # unrelated orientation
    ik.position_only_checkbox.setChecked(True)
    _solve(win)
    assert ik.table.rowCount() >= 1
    ik.table.selectRow(0)
    QApplication.processEvents()
    assert _tcp(model)[:3, 3] == pytest.approx(T[:3, 3], abs=1e-5)


def test_ik_clears_on_project_close(win):
    model = _load(win)
    ik = win.ik_panel
    ik.tcp_button.click()
    _solve(win)
    assert ik.table.rowCount() >= 1
    assert win.robot_view.target_actor is not None
    win.project_controller.close_project(win)
    assert ik.table.rowCount() == 0
    assert not ik.solve_button.isEnabled()
    assert win.robot_view.target_actor is None
    assert win.analysis_panel.torque_table.rowCount() == 0


# ---------------------------------------------------------------- NumField
def test_numfield_scientific_and_invalid(qapp):
    f = NumField(1.0)
    f.setText("2.5e-3")
    assert f.value() == pytest.approx(2.5e-3) and f.valid()
    assert f.styleSheet() == ""
    f.setText("abc")
    assert f.value() == pytest.approx(2.5e-3)        # previous value kept
    assert not f.valid()
    assert "7a2020" in f.styleSheet()               # red
    f.setText("1E+2")
    assert f.value() == 100.0 and f.valid() and f.styleSheet() == ""
    f.setText("nan")
    assert not f.valid() and f.value() == 100.0


def test_ik_fields_accept_scientific(win):
    _load(win)
    f = win.ik_panel.target_fields[1]
    f.setText("1e-2")
    assert f.value() == pytest.approx(0.01)
    win.ik_panel.damping_field.setText("5e-2")
    assert win.ik_panel.damping_field.value() == pytest.approx(0.05)


# ---------------------------------------------------------------- analysis
def test_singularity_near_for_wrist_aligned_not_for_generic(win):
    _load(win)
    ap = win.analysis_panel
    _set_q(win, Q_GENERIC)
    assert ap.near_singular_text == "ok"
    _set_q(win, Q_WRIST)
    assert ap.near_singular_text == "NEAR SINGULAR"
    assert float(ap.sigma_min_text) < 1e-3
    _set_q(win, Q_GENERIC)
    assert ap.near_singular_text == "ok"


def test_singularity_mode_and_threshold(win):
    model = _load(win)
    ap = win.analysis_panel
    _set_q(win, Q_GENERIC)
    cm = cb.compile(model)
    ref = cb.analyze_singularity(cb.jacobian_tcp(cm, current_q(model)), mode="position",
                                 threshold=1e-3)
    ap.mode_combo.setCurrentText("position")
    assert float(ap.sigma_min_text) == pytest.approx(ref["sigma_min"], rel=1e-3)
    ap.threshold_field.setText("1e3")                # huge threshold: always near
    assert ap.near_singular_text == "NEAR SINGULAR"


def _expected_tau(model, q, qd, qdd, g):
    cm = cb.compile(model)
    dm = cb.build_dyn_model(model, cm)
    return list(cb.inverse_dynamics(dm, q, qd, qdd, tuple(g)))


def test_torque_table_equals_inverse_dynamics(win):
    model = _load(win)
    ap = win.analysis_panel
    _set_q(win, Q_GENERIC)
    n = model.dof()
    qd = [0.1 * (k + 1) for k in range(n)]
    qdd = [0.05 * (k - 2) for k in range(n)]
    for f, v in zip(ap.qd_fields, qd):
        f.setText(f"{v:.6g}")
    for f, v in zip(ap.qdd_fields, qdd):
        f.setText(f"{v:.6g}")
    qd = [f.value() for f in ap.qd_fields]
    qdd = [f.value() for f in ap.qdd_fields]
    exp = _expected_tau(model, current_q(model), qd, qdd, [0.0, 0.0, -9.81])
    assert ap.tau == pytest.approx(exp, rel=1e-12, abs=1e-12)
    tbl = ap.torque_table
    assert tbl.rowCount() == n
    for k in range(n):
        assert float(tbl.item(k, 5).text()) == pytest.approx(exp[k], rel=1e-4, abs=1e-4)
    # parts add up: gravity + coriolis + M*qdd == total
    for k in range(n):
        s = sum(float(tbl.item(k, c).text()) for c in (2, 3, 4))
        assert s == pytest.approx(exp[k], rel=1e-3, abs=1e-3)


def test_qd_qdd_default_zero(win):
    _load(win)
    ap = win.analysis_panel
    assert len(ap.qd_fields) == 7 and len(ap.qdd_fields) == 7
    assert all(f.value() == 0.0 for f in ap.qd_fields + ap.qdd_fields)


def test_torque_limit_column_red_when_exceeded(win):
    model = _load(win)
    ap = win.analysis_panel
    _set_q(win, Q_GENERIC)
    tbl = ap.torque_table
    # fixture efforts are 10: giant qdd on joint 1 must exceed it
    ap.qdd_fields[1].setText("1e3")
    tau1 = ap.tau[1]
    assert abs(tau1) > 10.0
    assert tbl.item(1, 6).text() == "10"
    brush = tbl.item(1, 5).foreground().color().name()
    assert brush == "#ff5555"
    ap.qdd_fields[1].setText("0")
    exceeded = abs(ap.tau[1]) > 10.0
    assert (tbl.item(1, 5).foreground().color().name() == "#ff5555") == exceeded
    ap.qdd_fields[1].setText("-1e3")
    assert tbl.item(1, 5).foreground().color().name() == "#ff5555"
    # a joint inside its limit is not red
    ok = [k for k in range(model.dof()) if abs(ap.tau[k]) <= 10.0]
    assert ok and tbl.item(ok[0], 5).foreground().color().name() != "#ff5555"


def test_gravity_from_project_settings(win):
    model = _load(win)
    ap = win.analysis_panel
    _set_q(win, Q_GENERIC)
    tau_a = ap.tau
    exp_a = _expected_tau(model, current_q(model), [0.0] * 7, [0.0] * 7, [0.0, 0.0, -9.81])
    assert tau_a == pytest.approx(exp_a, rel=1e-12, abs=1e-12)
    win.project_controller.current_project.simulation_settings.gravity = [0.0, 0.0, -1.62]
    ap.refresh()
    exp_b = _expected_tau(model, current_q(model), [0.0] * 7, [0.0] * 7, [0.0, 0.0, -1.62])
    assert ap.tau == pytest.approx(exp_b, rel=1e-12, abs=1e-12)
    assert ap.tau != pytest.approx(tau_a, rel=1e-6)
    # zero gravity, zero velocity: no torque at all
    win.project_controller.current_project.simulation_settings.gravity = [0.0, 0.0, 0.0]
    ap.refresh()
    assert ap.tau == pytest.approx([0.0] * 7, abs=1e-12)


def test_energy_group(win):
    model = _load(win)
    ap = win.analysis_panel
    _set_q(win, Q_GENERIC)
    qd = [0.2] * 7
    for f in ap.qd_fields:
        f.setText("0.2")
    cm = cb.compile(model)
    dm = cb.build_dyn_model(model, cm)
    q = current_q(model)
    g = (0.0, 0.0, -9.81)
    ke, pe, te = ap.energy_texts
    assert float(ke) == pytest.approx(cb.kinetic_energy(dm, q, qd), rel=1e-5)
    assert float(pe) == pytest.approx(cb.potential_energy(dm, q, g), rel=1e-5)
    assert float(te) == pytest.approx(cb.total_energy(dm, q, qd, g), rel=1e-5)


def test_mass_not_set_warning_disables_forward_dynamics(win):
    model = _load(win)
    ap = win.analysis_panel
    assert not ap.mass_warning_visible
    assert ap.forward_dynamics_enabled
    for lk in model.links:
        lk.inertia = cb.Inertia()          # mass 0 = not set
    ap.invalidate()
    assert ap.mass_warning_visible
    assert MASS_WARNING in ap.mass_warning_text
    assert not ap.forward_dynamics_enabled


def test_mass_warning_only_for_moving_links(win):
    model = _load(win)
    ap = win.analysis_panel
    # base link (root) mass 0 alone is fine
    i = cb.Inertia()
    model.links[0].inertia = i
    ap.invalidate()
    assert not ap.mass_warning_visible


def test_forward_dynamics_button_fills_qdd(win):
    model = _load(win)
    ap = win.analysis_panel
    _set_q(win, Q_GENERIC)
    ap.forward_dynamics_button.click()
    cm = cb.compile(model)
    dm = cb.build_dyn_model(model, cm)
    qdd = cb.forward_dynamics(dm, current_q(model), [0.0] * 7, [0.0] * 7, (0.0, 0.0, -9.81))
    assert [f.value() for f in ap.qdd_fields] == pytest.approx(list(qdd), rel=1e-8, abs=1e-9)


def test_cache_not_rebuilt_on_q_change_but_on_robot_change(win):
    model = _load(win)
    ap = win.analysis_panel
    ik = win.ik_panel
    c0, d0, k0 = ap.cache.compile_count, ap.cache.dyn_build_count, ik.cache.compile_count
    for q in (Q_GENERIC, Q_WRIST, [0.0] * 7, Q_GENERIC):
        _set_q(win, q)
    assert ap.cache.compile_count == c0
    assert ap.cache.dyn_build_count == d0
    ik.tcp_button.click()
    _solve(win)
    k1 = ik.cache.compile_count
    _solve(win)
    assert ik.cache.compile_count == k1             # solve reuses the compiled model
    win.set_tcp_link(len(model.links) - 1)          # robot_changed
    assert ap.cache.compile_count == c0 + 1
    assert ap.cache.dyn_build_count == d0 + 1


def test_physical_edit_rebuilds_dynamics(win):
    model = _load(win)
    ap = win.analysis_panel
    _set_q(win, Q_GENERIC)
    tau_a = ap.tau
    lk = model.links[2]
    inertia = lk.inertia
    inertia.mass = inertia.mass * 3.0
    lk.inertia = inertia
    win._on_physical_edited()
    assert ap.tau != pytest.approx(tau_a, rel=1e-9)
    exp = _expected_tau(model, current_q(model), [0.0] * 7, [0.0] * 7, [0.0, 0.0, -9.81])
    assert ap.tau == pytest.approx(exp, rel=1e-12, abs=1e-12)


def test_target_iso_matches_pose_to_iso():
    T = target_iso([0.1, 0.2, 0.3], [10.0, 20.0, 30.0])
    p = cb.Pose()
    p.xyz = [0.1, 0.2, 0.3]
    p.rpy = [math.radians(10), math.radians(20), math.radians(30)]
    assert np.allclose(T, np.asarray(cb.pose_to_iso(p)))
