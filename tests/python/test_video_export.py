"""U6.5: playback bar, screenshot and video export (frame count, GIF / PNG, cancel)."""
import os
import time
from pathlib import Path

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
pytest.importorskip("fcl")

import core_bridge as cb
from app.kin_utils import current_q
from app.main_window import MainWindow
from app.settings import AppSettings
from app.sim import video_export as ve
from app.sim.trajectory import Trajectory, Waypoint
from app.task_runner import TaskRunner

from .test_curve_editor import pump_until
from .test_live_blocking import load

ORG = "RoboticsAnalyzer_test_u6video"


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


@pytest.fixture
def runner(qapp):
    r = TaskRunner()
    yield r
    r.shutdown()


def _fake_grab(w=64, h=48):
    state = {"n": 0}

    def grab():
        state["n"] += 1
        f = np.zeros((h, w, 3), np.uint8)
        f[:, :, 0] = state["n"] % 256
        f[:, :, 1] = np.arange(w, dtype=np.uint8)[None, :]
        return f

    return grab, state


def _run(exp, path, fmt, fps, dur, grab, seek):
    result = {}
    exp.finished.connect(lambda p, n: result.update(done=(p, n)))
    exp.cancelled.connect(lambda: result.update(cancelled=True))
    exp.failed.connect(lambda m: result.update(failed=m))
    exp.start(path, fmt, fps, dur, grab, seek)
    assert pump_until(lambda: bool(result), 60), "export did not finish"
    return result


def test_frame_count_and_formats():
    assert ve.frame_count(2.0, 30) == 60
    assert ve.frame_count(0.01, 30) == 1
    assert ve.frame_count(1.5, 24) == 36
    f = ve.available_formats()
    assert "png" in f and "gif" in f                      # Pillow is installed
    assert ve.normalize_path("a/b.mp4", "gif").endswith("b.gif")
    assert ve.normalize_path("a/frames.png", "png").endswith("frames")


def test_png_sequence_frame_count(runner, tmp_path):
    exp = ve.VideoExporter(runner)
    grab, st = _fake_grab()
    times = []
    r = _run(exp, str(tmp_path / "frames"), "png", 30, 2.0, grab, times.append)
    assert r["done"][1] == 60
    files = sorted(Path(r["done"][0]).glob("frame_*.png"))
    assert len(files) == 60 and files[0].stat().st_size > 0
    assert len(times) == 60 and abs(times[1] - 1 / 30) < 1e-12 and abs(times[-1] - 59 / 30) < 1e-12
    from PIL import Image
    im = np.asarray(Image.open(files[7]))
    assert im.shape == (48, 64, 3) and im[0, 0, 0] == 8        # frames are in order


def test_gif_export_frames_and_size(runner, tmp_path):
    exp = ve.VideoExporter(runner)
    grab, _ = _fake_grab(40, 30)
    path = str(tmp_path / "out.gif")
    r = _run(exp, path, "gif", 25, 1.0, grab, lambda t: None)
    assert r["done"][0] == path and r["done"][1] == 25
    from PIL import Image
    im = Image.open(path)
    assert im.n_frames == 25 and im.size == (40, 30)
    assert im.info["duration"] == 40                          # 25 fps
    assert os.path.getsize(path) > 0


def test_cancel_removes_output(runner, tmp_path):
    exp = ve.VideoExporter(runner)
    grab, st = _fake_grab()
    res = {}
    exp.cancelled.connect(lambda: res.update(c=True))
    exp.finished.connect(lambda p, n: res.update(f=True))
    exp.progress.connect(lambda i, n: exp.cancel() if i == 5 else None)
    exp.start(str(tmp_path / "frames"), "png", 30, 4.0, grab, lambda t: None)
    assert pump_until(lambda: bool(res), 60)
    assert res.get("c") and not res.get("f")
    assert not (tmp_path / "frames").exists()
    assert st["n"] < 120 and not exp.running


def test_bad_frame_fails_cleanly(runner, tmp_path):
    exp = ve.VideoExporter(runner)
    r = _run(exp, str(tmp_path / "f"), "png", 30, 1.0, lambda: np.zeros((4, 4)), lambda t: None)
    assert "failed" in r and "uint8" in r["failed"]
    assert not exp.running


def test_mp4_when_ffmpeg_exists(runner, tmp_path):
    if "mp4" not in ve.available_formats():
        pytest.skip("no ffmpeg on this machine (MP4 entry is not offered)")
    exp = ve.VideoExporter(runner)
    grab, _ = _fake_grab(64, 48)
    r = _run(exp, str(tmp_path / "o.mp4"), "mp4", 30, 1.0, grab, lambda t: None)
    assert os.path.getsize(r["done"][0]) > 0


def _motion(win):
    model = load(win, "ur5")
    q0 = np.array(current_q(model))
    q1 = q0 + np.array([0.5, -0.3, 0.3, 0.2, -0.2, 0.1])
    traj = Trajectory([Waypoint(0.0, q0.tolist()), Waypoint(2.0, q1.tolist())], interpolation="cubic")
    assert win.sim_panel.load_trajectory(traj)
    win.sim_panel.set_mode("kinematic")
    return model, traj


def test_playback_bar_follows_motion(win):
    model, traj = _motion(win)
    bar = win._playback_bar
    assert bar.isVisible() and abs(bar.duration - 2.0) < 1e-9
    sp = win.sim_panel
    # scrub: slider middle -> position 1 s
    bar.slider.setValue(500)
    assert abs(sp.playback_time - 1.0) < 1e-6
    q_mid = np.array(current_q(model))
    assert np.allclose(q_mid, traj.q(1.0), atol=1e-9)
    # frame step
    bar.btn_fwd.click()
    assert abs(sp.playback_time - (1.0 + 1 / 30)) < 1e-6
    bar.btn_back.click()
    bar.btn_back.click()
    assert abs(sp.playback_time - (1.0 - 1 / 30)) < 1e-6
    assert abs(float(bar.label.text().split("/")[0]) - sp.playback_time) < 0.006
    # loop flag goes to the timeline
    bar.loop.setChecked(True)
    assert sp.timeline.loop
    bar.loop.setChecked(False)
    # play / pause
    bar.btn_play.click()
    assert pump_until(lambda: sp.is_playing and bar.playing, 5)
    bar.btn_play.click()
    assert pump_until(lambda: not sp.is_playing and not bar.playing, 5)
    # no trajectory: the bar hides
    win.sim_panel.clear_trajectory()
    assert pump_until(lambda: not bar.isVisible(), 5)


def test_export_video_from_window_two_seconds(win, tmp_path):
    model, traj = _motion(win)
    from app.panels.video_dialog import VideoExportDialog
    dlg = win.export_video(modal=False)
    assert isinstance(dlg, VideoExportDialog)
    dlg.format.setCurrentIndex(dlg.format.findData("png"))
    dlg.fps.setValue(10)
    out = tmp_path / "shots"
    dlg.path.setText(str(out))
    assert "20 frames" in dlg.info.text()
    win.sim_panel.seek_trajectory(0.7)
    win._video_resume = win.sim_panel.playback_time
    dlg.start()
    assert pump_until(lambda: dlg.result_path != "", 120)
    frames = sorted(out.glob("frame_*.png"))
    assert len(frames) == 20 and all(f.stat().st_size > 0 for f in frames)
    # the motion went back to where it was
    assert abs(win.sim_panel.playback_time - 0.7) < 1e-6
    from PIL import Image
    assert Image.open(frames[0]).size[0] > 100
    dlg.reject()
    win._video_dialog = None


def test_export_without_motion_is_refused(win):
    load(win, "ur5")
    assert win.export_video(modal=False) is None
    assert "motion" in win._status_label.text().lower()


def test_screenshot_png(win, tmp_path):
    load(win, "ur5")
    p = tmp_path / "view.png"
    win.save_screenshot_to(str(p))
    assert p.exists() and p.stat().st_size > 100
    from PIL import Image
    im = Image.open(p)
    assert im.size[0] > 100 and im.size[1] > 100
    arr = win.grab_view((320, 180))
    assert arr.shape == (180, 320, 3) and arr.dtype == np.uint8
