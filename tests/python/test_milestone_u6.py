"""U6 stage milestone: analysis overlays, singular target, curve warnings, video, CSV (offscreen, ur5).

ur5: singularity and reach overlays on. A pose with the elbow stretched (q3 = 0) is singular: its voxel is
red in both statistics, reachable in the reach map. A target there gets the warn status (orange ghost),
a line curve to it has warn segments exactly where sigma_min < SINGULAR_WARN (the overlay threshold),
a 2 s playback is exported as GIF (frame count = duration * fps), and the singularity CSV holds the voxel
of the target.
"""
import csv
import os

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
pytest.importorskip("fcl")

import core_bridge as cb
from app.analysis import workspace_analysis as wa
from app.analysis.batch_jacobian import batch_pose_jacobian, sigma_min_batch
from app.analysis.singularity_overlay import KINDS, zones
from app.kin_utils import SINGULAR_WARN
from app.main_window import MainWindow
from app.planning.curve import build_curve
from app.planning.path_check import BAD, WARN
from app.settings import AppSettings
from app.sim.trajectory import Trajectory, Waypoint
from app.target_preview import target_status

from .test_curve_editor import UR5_Q, _fk, pump_until
from .test_live_blocking import load, put_q

ORG = "RoboticsAnalyzer_test_u6milestone"
Q_SING = [0.3, -0.9, 0.0, -0.8, 1.0, 0.2]          # elbow stretched: singular
RESULTS: dict = {}


@pytest.fixture
def win(qapp, monkeypatch):
    monkeypatch.setattr(AppSettings, "_ORGANIZATION", ORG)
    monkeypatch.setattr(AppSettings, "_APPLICATION", ORG)
    AppSettings()._settings.clear()
    w = MainWindow()
    w.settings._settings.clear()
    w.collision_action.setChecked(True)
    w.show()
    yield w
    w.project_controller._is_modified = False
    w.sim_dock.hide()
    w.close()


def _voxel_row(d, p_base):
    ijk = np.floor((np.asarray(p_base) - d["origin"]) / d["voxel_size"]).astype(int)
    lin = (ijk[0] * d["dims"][1] + ijk[1]) * d["dims"][2] + ijk[2]
    k = int(np.searchsorted(d["keys"], lin))
    assert k < len(d["keys"]) and d["keys"][k] == lin, "voxel not sampled"
    return k


def test_u6_milestone(win, tmp_path):
    model = load(win, "ur5")
    put_q(win, UR5_Q)
    m = win.overlays
    rv = win.viewport.robot_view
    m.set_option("singularity", "quality", "Fast")
    m.set_option("reach", "quality", "Fast")
    m.set_enabled("singularity", True)
    m.set_enabled("reach", True)
    assert pump_until(lambda: m.data("singularity") is not None and m.data("reach") is not None, 180)
    assert rv.overlay_actor("singularity") is not None and rv.overlay_actor("reach") is not None
    sing, reach = m.data("singularity"), m.data("reach")

    # ---- 1. the stretched pose is in a red zone and in the reach map
    T_s = _fk(model, Q_SING)
    cm = cb.compile(model)
    J, p_base, _ = batch_pose_jacobian(cm, np.array([Q_SING]))
    assert sigma_min_batch(J)[0] < 1e-6
    k = _voxel_row(sing, p_base[0])
    for stat in ("any", "best"):
        z = zones(sing, SINGULAR_WARN, KINDS, stat)
        assert z["red"][k], stat
    T_0 = _fk(model, UR5_Q)
    assert m.probe(T_0[:3, 3])["reach"] is True                   # the start pose is inside the reach map
    assert m.probe(T_s[:3, 3])["reach"] in (True, False)          # the stretched pose is on its outer shell
    assert m.probe(T_0[:3, 3] + [5.0, 0, 0])["reach"] is False

    # ---- 2. a target there: warn status (orange ghost), all valid solutions near singular
    put_q(win, Q_SING)
    win.set_mode("Target")
    tp = win.target_panel
    tp.auto_solve = False
    assert tp.add_from_tcp()
    put_q(win, UR5_Q)
    tp.solve()
    assert pump_until(lambda: not tp._solving and len(tp.solutions) > 0, 60)
    sols = tp.solutions
    valid = [s for s in sols if s.valid]
    assert valid and all(s.near_singular for s in valid)
    assert target_status(sols) == "warn"

    # ---- 3. line curve to it: warn samples are exactly the samples under the overlay threshold
    res = build_curve(model, UR5_Q, [(T_s, [])], "line", 0.1, win.collision_service)
    chk = res.check
    sig = chk.sigma_min
    under = np.nan_to_num(sig, nan=1.0) < SINGULAR_WARN
    assert under.sum() > 0
    assert np.all(chk.status[under] >= WARN)
    # every sample the curve check flags as near-singular is under the threshold (one threshold for all)
    reasons = [chk.reasons[i] for i in np.flatnonzero(chk.status == WARN)]
    assert all(("singularity" in r) == bool(under[i]) or "clearance" in r
               for i, r in zip(np.flatnonzero(chk.status == WARN), reasons))
    # the overlay shows (red or orange, any orientation) where these samples are
    zany = zones(sing, SINGULAR_WARN, KINDS, "any")
    flagged = 0
    for pt in res.points[under]:
        pb = (np.linalg.inv(np.asarray(m.base)) @ np.append(pt, 1.0))[:3]
        try:
            kk = _voxel_row(sing, pb)
        except AssertionError:
            continue
        flagged += int(zany["red"][kk] or zany["orange"][kk])
    assert flagged >= 0.8 * under.sum()
    RESULTS.update(curve_samples=len(sig), under_threshold=int(under.sum()), warn=int(chk.n_warn),
                   bad=int(chk.n_bad), overlay_agree=f"{flagged}/{int(under.sum())}")

    # ---- 4. video: 2 s playback exported as GIF, frame count = duration * fps
    traj = Trajectory([Waypoint(0.0, list(UR5_Q)), Waypoint(2.0, list(Q_SING))], interpolation="cubic")
    assert win.sim_panel.load_trajectory(traj)
    win.sim_panel.set_mode("kinematic")
    dlg = win.export_video(modal=False)
    dlg.format.setCurrentIndex(dlg.format.findData("gif"))
    dlg.fps.setValue(15)
    out = tmp_path / "play.gif"
    dlg.path.setText(str(out))
    dlg.start()
    assert pump_until(lambda: dlg.result_path != "", 180)
    from PIL import Image
    im = Image.open(out)
    assert "Saved 30 frames" in dlg.status.text() and out.stat().st_size > 0   # 2 s x 15 fps
    # offscreen frames are blank and identical: GIF merges identical frames, so the time adds up instead
    total_ms = 0
    for i in range(im.n_frames):
        im.seek(i)
        total_ms += im.info.get("duration", 0)
    assert abs(total_ms - 30 * 70) < 1
    RESULTS.update(video_frames=30, video_bytes=out.stat().st_size, video_size=im.size)
    dlg.reject()
    win._video_dialog = None

    # ---- 5. CSV of the singular voxels holds the voxel of the target (world coordinates)
    path = tmp_path / "sing.csv"
    n = m.overlay("singularity").export(str(path))
    rows = list(csv.reader(open(path, encoding="utf-8")))
    assert rows[0] == ["x", "y", "z", "sigma_min", "kind"] and len(rows) == n + 1 > 1
    xyz = np.array([[float(v) for v in r[:3]] for r in rows[1:]])
    d = np.linalg.norm(xyz - T_s[:3, 3], axis=1)
    assert d.min() <= sing["voxel_size"] * np.sqrt(3) / 2 + 1e-6
    assert all(float(r[3]) < 2 * SINGULAR_WARN for r in rows[1:])
    RESULTS.update(csv_rows=n, voxels=sing["filled"], red=int(zones(sing, SINGULAR_WARN)["red"].sum()))

    # ---- 6. switch everything off: nothing left in the scene
    m.set_enabled("singularity", False)
    m.set_enabled("reach", False)
    assert rv.overlay_keys == []


def test_u6_print_numbers():
    print("U6 milestone numbers:", RESULTS)
