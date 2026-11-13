"""
tests/python/test_milestone_f1.py
F1 stage milestone: (a) Pinocchio item (skipped, no wheel), (b) torque-aware TOPP on the six_dof
fixture around the R7 wall obstacle, (c) shown ghost set round trip. (d) the A1 milestone runs
in the same full pytest run (not duplicated here).

Oracles independent of the code under test: torque from ``numpy_ref.numpy_dynamics`` (Lagrangian,
no core_bridge), collision recheck by the separating-axis ``DenseCheck`` of the R7 milestone.
"""

import unittest.mock as mock

import numpy as np
import pytest

from app.planning.checks import check_trajectory
from app.planning.collision import CollisionWorld
from app.planning.shortcut import time_parametrize
from app.project.serializer import save
from app.sim.mode import DYNAMIC

from .numpy_ref import inertias_from_model, numpy_dynamics, spec_from_model
from .test_milestone_r7 import (EFFORT_CHECK, GOAL, RMS_LIMIT, START, DenseCheck,  # noqa: F401
                                _load, _play, _plan, win)

RESULTS: dict = {}
G = (0.0, 0.0, -9.81)
TORQUE_REL_TOL = 1e-6
N_NUMPY_SAMPLES = 40


def test_a_pinocchio_validation():
    try:
        import pinocchio
        real = hasattr(pinocchio, "buildModelFromUrdf")   # an unrelated PyPI "pinocchio" exists
    except ImportError:
        real = False
    if not real:
        pytest.skip("Pinocchio has no Python 3.13 Windows wheel (F1.1, see PROGRESS Decisions)")
    pytest.fail("pinocchio is importable now: write the validator test (F1.2 was skipped)")


def test_b_topp_around_wall(win):
    model = _load(win)
    proj = win.project_controller.current_project
    dense = DenseCheck(model, proj)
    pp = _plan(win)
    path = np.asarray(pp.result.path)
    shapes, cfg = pp._collision_source()
    world = CollisionWorld(model, shapes, cfg)

    scale = time_parametrize(path, model, collision=world)
    topp = time_parametrize(path, model, limits={"effort": EFFORT_CHECK}, collision=world,
                            method="topp")
    tr = topp.trajectory
    RESULTS.update(dur_scale=scale.trajectory.duration, dur_topp=tr.duration,
                   topp_interp=topp.interpolation)
    assert tr.duration <= scale.trajectory.duration + 1e-9, (tr.duration, scale.trajectory.duration)

    # (1) library checker, every constraint kind (default tol 1e-9 absolute)
    rep = check_trajectory(tr, model, limits={"effort": EFFORT_CHECK})
    assert rep.ok, rep.summary()
    assert not rep.of_kind("torque") and not rep.of_kind("velocity")
    assert not rep.of_kind("acceleration") and not rep.of_kind("position")
    RESULTS["checker_tol"] = 1e-9

    # (2) independent numpy torque on dense samples
    spec, inert = spec_from_model(model), inertias_from_model(model)
    peak = 0.0
    for t in np.linspace(0.0, tr.duration, N_NUMPY_SAMPLES):
        M, c, grav = numpy_dynamics(spec, inert, tr.q(t), tr.qd(t), G)
        peak = max(peak, float(np.max(np.abs(M @ tr.qdd(t) + c + grav))))
    RESULTS["peak_torque_numpy"] = peak
    RESULTS["torque_limit"] = EFFORT_CHECK
    assert peak <= EFFORT_CHECK * (1 + TORQUE_REL_TOL), f"peak {peak} vs {EFFORT_CHECK}"

    # (3) dense collision recheck of the timed trajectory
    ts = np.append(np.arange(0.0, tr.duration, 1e-2), tr.duration)
    for t in ts:
        e, s = dense.collides(np.asarray(tr.q(t)))
        assert not e and not s, f"collision at t={t:.3f}"
    RESULTS["collision_samples"] = len(ts)

    # (4) dynamic computed-torque playback of the TOPP trajectory
    sp = win.sim_panel
    assert sp.load_trajectory(tr)
    _play(sp, DYNAMIC, tr)
    t, q = sp.recorder.export_arrays()["q"]
    desired = np.array([tr.q(x) for x in t])
    rms = float(np.sqrt(np.mean((q - desired) ** 2)))
    RESULTS.update(track_rms=rms, track_samples=len(t))
    assert rms < RMS_LIMIT, rms
    sp.stop_trajectory()


def test_c_ghost_set_round_trip(win, tmp_path):
    _load(win)                                   # adds ghost configs "start" and "goal"
    gp, rv, pc = win.ghost_panel, win.robot_view, win.project_controller
    assert gp.add_configuration("mid", [0.0, -1.5, 0.36, 0.0, -0.25, 0.0, 0.0]) == "mid"
    for k in ("goal", "mid", "start"):
        assert gp.set_shown(k, True)
    names = gp.shown_names
    colours = [gp.ghost_color(k) for k in names]
    assert names == ["goal", "mid", "start"] and len(set(colours)) == 3
    assert rv.ghost_count == 3
    out = tmp_path / "f1.robotproj"
    save(pc.current_project, out)
    pc._is_modified = False
    pc.close_project(None)
    assert rv.ghost_count == 0
    with mock.patch("app.main_window.QMessageBox.warning") as w:
        assert pc.open_project(None, str(out))
    assert w.call_count == 0
    assert gp.shown_names == names
    assert [gp.ghost_color(k) for k in names] == colours
    assert rv.ghost_count == 3 and sorted(rv.ghost_keys) == ["goal", "mid", "start"]
    pc._is_modified = False


def test_zz_print_gate_numbers():
    print("F1 GATE NUMBERS", {k: (f"{v:.4g}" if isinstance(v, float) else v) for k, v in RESULTS.items()})
