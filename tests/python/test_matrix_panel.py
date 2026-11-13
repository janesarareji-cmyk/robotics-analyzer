"""U4b.5: live matrix panel: values match FK / Jacobian, formats parse back, throttle, pause, cost."""
import os
import time

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtCore import QCoreApplication
from PySide6.QtGui import QGuiApplication

import core_bridge as cb
from app.analysis.matrices import (
    compute_matrices, export_text, format_matrix, parse_latex_block, parse_numpy_block,
)
from app.kin_utils import current_q, dof_limits, write_q
from app.main_window import MainWindow
from app.settings import AppSettings

from .robots import random_q, random_robot
from .test_live_blocking import load, put_q

ORG = "RoboticsAnalyzer_test_u4b5"


def test_compute_matches_cb_at_random_q():
    for seed in range(5):
        model = random_robot(40 + seed, 6, kinds="origin_axis")
        model.base = cb.Pose()
        model.base.xyz = [0.3, -0.2, 0.1]
        model.base.rpy = [0.1, 0.2, 0.3]
        rng = np.random.default_rng(seed)
        q = np.array(random_q(model, rng))
        cm = cb.compile(model)
        d = compute_matrices(model, q, None, cm, joint=2)
        fk = cb.fk(model, list(q))
        T0 = np.asarray(cb.pose_to_iso(model.base), float)
        assert np.max(np.abs(d.T_world_tcp - np.asarray(fk["tcp"]))) < 1e-12
        assert np.max(np.abs(T0 @ d.T_base_tcp - np.asarray(fk["tcp"]))) < 1e-12
        J = np.asarray(cb.jacobian_tcp(cm, list(q)), float)
        assert np.max(np.abs(d.jacobian - J)) < 1e-12 and d.jacobian.shape == (6, 6)
        s = cb.analyze_singularity(J)
        assert d.sigma_min == pytest.approx(s["sigma_min"], abs=1e-14)
        assert d.manipulability == pytest.approx(s["manipulability"], abs=1e-14)
        # joint transform: parent * T_parent_child == child frame, base * T_base_link too
        from app.panels.model_panel import joint_links
        p, c = joint_links(model)[2]
        Tp, Tc = np.asarray(fk["frames"][p]), np.asarray(fk["frames"][c])
        assert np.max(np.abs(Tp @ d.T_parent_child - Tc)) < 1e-12
        assert np.max(np.abs(T0 @ d.T_base_link - Tc)) < 1e-12


def test_identity_base_has_no_world_matrix():
    model = random_robot(3, 4, kinds="origin_axis")
    model.base = cb.Pose()
    d = compute_matrices(model, np.zeros(4), None, None, jacobian=False)
    assert d.T_world_tcp is None and d.jacobian is None
    assert np.allclose(d.T_base_tcp, np.asarray(cb.fk(model, [0.0] * 4)["tcp"]))


def test_copy_formats_parse_back():
    model = random_robot(7, 5, kinds="origin_axis")
    q = np.array(random_q(model, np.random.default_rng(1)))
    cm = cb.compile(model)
    d = compute_matrices(model, q, None, cm, joint=1)
    for units in ("m", "mm"):
        scale = np.array([1, 1, 1, 1.0])
        t_np = parse_numpy_block(export_text(d, "numpy", units, 6))
        t_la = parse_latex_block(export_text(d, "latex", units, 6))
        for got in (t_np, t_la):
            k = 1000.0 if units == "mm" else 1.0
            exp = d.T_base_tcp.copy()
            exp[:3, 3] *= k
            assert np.max(np.abs(got["T_base_tcp"] - exp)) < 1e-6
            assert np.max(np.abs(got["J"] - d.jacobian)) < 1e-6
            assert got["T_parent_child"].shape == (4, 4) and got["q"].shape == (1, 5)
        assert np.allclose(t_np["T_base_link"], t_la["T_base_link"], atol=1e-6)
    txt = export_text(d, "text", "m", 4)
    assert "T_base_tcp =" in txt and txt.count("\n") > 20
    assert format_matrix(np.eye(2), "numpy") == "np.array([[1.0000, 0.0000],\n          [0.0000, 1.0000]])"
    with pytest.raises(ValueError):
        format_matrix(np.eye(2), "xml")


@pytest.fixture
def win(qapp, monkeypatch):
    monkeypatch.setattr(AppSettings, "_ORGANIZATION", ORG)
    monkeypatch.setattr(AppSettings, "_APPLICATION", ORG)
    AppSettings()._settings.clear()
    w = MainWindow()
    w.settings._settings.clear()
    w.show()
    yield w
    w.project_controller._is_modified = False
    w.sim_dock.hide()
    w.close()


def test_panel_follows_every_q_change_and_matches_fk(win):
    model = load(win, "ur5")
    mp = win.matrix_panel
    win._action_view_matrices.setChecked(True)
    QCoreApplication.processEvents()
    mp.refresh_now()
    assert mp.data is not None and "deg" in mp.q_text
    rng = np.random.default_rng(0)
    lo, hi = (np.asarray(v) for v in dof_limits(model))
    for k in range(5):
        q = lo + rng.random(len(lo)) * (hi - lo)
        put_q(win, q)
        time.sleep(0.04)                      # past the 33 ms throttle
        QCoreApplication.processEvents()
        mp.refresh_now()
        fk = cb.fk(model, list(q))
        assert np.max(np.abs(mp.data.T_world_tcp - np.asarray(fk["tcp"])) if mp.data.T_world_tcp is not None
                      else np.abs(mp.data.T_base_tcp - np.asarray(fk["tcp"]))) < 1e-12
    # the 4 decimals shown
    first = mp.tcp_text.splitlines()[0].split()
    assert all(len(x.split(".")[1]) == 4 for x in first)
    mp.set_units("m")
    assert mp.tcp_text != ""
    mp.set_joint(2)
    assert mp.data.joint == 2
    mp.show_jacobian(True)
    assert len(mp.jacobian_text.splitlines()) == 6


def test_throttle_pause_hidden_and_cost(win):
    model = load(win, "panda")
    mp = win.matrix_panel
    win._action_view_matrices.setChecked(True)
    QCoreApplication.processEvents()
    mp.refresh_now()
    n0 = mp.refresh_count
    q = np.array(current_q(model))
    for k in range(60):                       # a burst of 60 changes in far less than 1 s
        q[0] = 0.01 * k
        write_q(model, q)
        win._update_robot_view()
    assert mp.refresh_count - n0 <= 3         # throttled: at most one immediate + one pending
    t0 = time.time()
    while time.time() - t0 < 0.2:
        QCoreApplication.processEvents()
    assert mp.data is not None and mp.data.q[0] == pytest.approx(0.59)
    # pause freezes the numbers
    mp.set_paused(True)
    frozen = mp.tcp_text
    q[0] = -0.5
    write_q(model, q)
    win._update_robot_view()
    time.sleep(0.05)
    QCoreApplication.processEvents()
    assert mp.tcp_text == frozen
    mp.set_joint(1)
    assert mp.data.q[0] == pytest.approx(0.59)          # same frozen q
    mp.set_paused(False)
    assert mp.data.q[0] == pytest.approx(-0.5) and mp.tcp_text != frozen
    # cost of one refresh (panda 8 DOF, Jacobian shown)
    mp.show_jacobian(True)
    times = []
    for _ in range(40):
        mp.refresh_now()
        times.append(mp.last_refresh_ms)
    assert np.median(times) < 2.0, np.median(times)
    # hidden dock: nothing is computed
    win._action_view_matrices.setChecked(False)
    QCoreApplication.processEvents()
    n1 = mp.refresh_count
    q[0] = 0.2
    write_q(model, q)
    win._update_robot_view()
    time.sleep(0.05)
    QCoreApplication.processEvents()
    assert mp.refresh_count == n1


def test_sigma_bar_red_near_singularity_and_copy_clipboard(win):
    model = load(win, "ur5")
    mp = win.matrix_panel
    win._action_view_matrices.setChecked(True)
    q = np.array([0.0, -1.4, 1.6, -1.8, -1.5, 0.0])
    put_q(win, q)
    mp.refresh_now()
    assert "NEAR" not in mp.sigma_text and mp.sigma_bar_value > 30
    q[4] = 0.0                                          # wrist singularity
    put_q(win, q)
    mp.refresh_now()
    assert "NEAR A SINGULARITY" in mp.sigma_text and mp.sigma_bar_value < 10
    text = mp.copy("numpy")
    assert QGuiApplication.clipboard().text() == text
    parsed = parse_numpy_block(text)
    assert parsed["T_base_tcp"].shape == (4, 4)
    for fmt in ("text", "latex"):
        assert mp.copy(fmt) != ""
