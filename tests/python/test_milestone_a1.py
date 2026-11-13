"""A1.9: stage A1 milestone (audit fixes), one end-to-end flow per risk area.

(a) close flow Cancel / Discard / Save, (b) six corruption kinds, (c) slow permission dialog,
(d) bad arrays to GPU FK and a bad model to compile, (e) gantry workspace and PRM cancel.
Helpers come from the A1.x tests. Offscreen Qt, every QMessageBox monkeypatched.
"""
import shutil
import sys
import threading
import time
import unittest.mock as mock
import zipfile
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from PySide6.QtWidgets import QMessageBox

import core_bridge as cb

from app.analysis.workspace_analysis import analyze_workspace
from app.planning.planner import plan
from app.project import serializer
from app.project.recovery import load_best
from app.project.serializer import load

from .test_audit_numerics import _Blocked, gantry
from .test_cartesian_traj import planar_2r
from .test_close_flow import SB, win  # noqa: F401  (fixture)
from .test_compiled_arrays import tree_model
from .test_load_safety import _corrupt_kinds, _good, _rewrite
from .test_plugins import DictSettings, mgr, mk, status
from .test_workspace import _gpu_or_skip

FIXTURES = Path(__file__).parent / "fixtures" / "projects"


@pytest.fixture
def excepthook_spy():
    seen = []
    old = sys.excepthook
    sys.excepthook = lambda t, v, tb: seen.append((t, v))
    yield seen
    sys.excepthook = old


def _quiet_boxes(monkeypatch, shown):
    for name in ("critical", "warning", "information"):
        monkeypatch.setattr(QMessageBox, name, staticmethod(lambda *a, **k: shown.append(a)))


# ---------------------------------------------------------------- (a) close flow
def test_a_close_flow_cancel_discard_save(win, tmp_path, monkeypatch, excepthook_spy):
    pc = win.project_controller
    src = tmp_path / "fx.robotproj"
    shutil.copy2(FIXTURES / "v1.3.0.robotproj", src)
    assert pc.open_project(win, str(src))
    pc.current_project.metadata.description = "edited-a1"
    pc.mark_modified()
    asset = tmp_path / "assets_tmp"
    asset.mkdir()
    pc.current_project.asset_dir = str(asset)
    auto = pc.autosave_now()
    assert auto is not None and auto.is_file()

    # Cancel: window stays, project and autosave kept
    monkeypatch.setattr(QMessageBox, "question", mock.Mock(return_value=SB.Cancel))
    assert win.close() is False
    assert win.isVisible() and pc.has_project and auto.is_file()

    # Save: file written and reloads equal to the edited project
    monkeypatch.setattr(QMessageBox, "question", mock.Mock(return_value=SB.Save))
    assert win.close() is True
    assert load(src).metadata.description == "edited-a1"
    assert not auto.exists()

    # Discard: reopen, edit, Discard -> file unchanged, autosave and temp dir gone
    win.show()
    assert pc.open_project(win, str(src))
    pc.current_project.metadata.description = "never-saved"
    pc.mark_modified()
    asset2 = tmp_path / "assets_tmp2"
    asset2.mkdir()
    pc.current_project.asset_dir = str(asset2)
    auto2 = pc.autosave_now()
    assert auto2 is not None and auto2.is_file()
    monkeypatch.setattr(QMessageBox, "question", mock.Mock(return_value=SB.Discard))
    assert win.close() is True
    assert not pc.has_project and not auto2.exists() and not asset2.exists()
    assert load(src).metadata.description == "edited-a1"
    assert excepthook_spy == []


# ---------------------------------------------------------------- (b) corruption
def _corrupt_deflate(path):
    with zipfile.ZipFile(path) as zf:
        zi = zf.getinfo("project_metadata.json")
    raw = bytearray(Path(path).read_bytes())
    start = zi.header_offset + 30 + len(zi.filename.encode()) + len(zi.extra)
    for i in range(start, start + zi.compress_size):
        raw[i] = 0xFF
    Path(path).write_bytes(bytes(raw))


@pytest.mark.parametrize("idx", range(6))
def test_b_corrupt_file_falls_back_or_dialog(win, tmp_path, monkeypatch, excepthook_spy, idx):
    pc = win.project_controller
    shown = []
    _quiet_boxes(monkeypatch, shown)
    monkeypatch.setattr(QMessageBox, "question", mock.Mock(return_value=SB.No))

    def corrupt(p):
        if idx < 5:
            _rewrite(p, **_corrupt_kinds(p)[idx][1])
        else:
            _corrupt_deflate(p)

    # no .prev: error dialog, open returns False, nothing unhandled
    out = _good(tmp_path, "nofall.robotproj")
    corrupt(out)
    assert pc.open_project(win, str(out)) is False
    assert shown

    # with .prev: falls back to it
    out2 = _good(tmp_path, "fall.robotproj")
    corrupt(out2)
    shutil.copy2(_good(tmp_path, "good.robotproj", desc="from-prev"), serializer.prev_path(out2))
    proj, opened = load_best(out2)
    assert Path(opened) == serializer.prev_path(out2)
    assert proj.metadata.description == "from-prev"
    pc._is_modified = False
    assert pc.open_project(win, str(out2)) is True
    assert pc.current_project.metadata.description == "from-prev"
    assert excepthook_spy == []


# ---------------------------------------------------------------- (c) plugin dialog
def test_c_slow_permission_dialog_and_unanswered_ask(tmp_path, monkeypatch, qapp):
    import app.plugins.host as ph
    import app.plugins.permissions as pm
    offset = [0.0]

    def fake():
        return time.monotonic() + offset[0]

    monkeypatch.setattr(pm, "clock", fake, raising=False)
    monkeypatch.setattr(ph, "time", SimpleNamespace(monotonic=fake, sleep=time.sleep))

    def slow_ask(plugin, perm, text):
        offset[0] += 15.0
        return True

    code = ("def register(host):\n    host.read_text(__file__)\n"
            "    host.register_planner('P', lambda *a, **k: None)\n")
    mk(tmp_path / "pl", "p", code, perms=["file_read"])
    m = mgr(tmp_path / "pl", ask=slow_ask, load_timeout=2.0)
    m.scan()
    assert status(m, "p") == "ok", m.result("p").message

    monkeypatch.setattr(pm, "ASK_TIMEOUT", 0.2, raising=False)
    s = DictSettings()
    pmgr = pm.PermissionManager(s)
    out = {}

    def work():
        try:
            pmgr.check("q", ["file_read"], "file_read")
        except PermissionError as exc:
            out["e"] = exc

    t = threading.Thread(target=work, daemon=True)
    t.start()
    t.join(3)
    assert "e" in out and s.d == {} and pmgr.state("q", "file_read") == "ask"
    pmgr.shutdown()


# ---------------------------------------------------------------- (d) bad input
def _three_links_one_joint():
    m = cb.RobotModel()
    for _ in range(3):
        m.links.append(cb.Link())
    m.joints.append(tree_model().joints[0])
    return m


def test_d_bad_arrays_gpu_alive():
    gb = _gpu_or_skip()
    good = cb.compile_arrays(tree_model())
    q = np.zeros((4, 3))
    bad = dict(good)
    arr = np.array(good["src_q_index"], copy=True)
    arr[0] = 99
    bad["src_q_index"] = arr
    with pytest.raises(ValueError):
        gb.fk_batch(bad, q)
    assert gb.device_info()["available"]
    assert gb.fk_batch(good, q)["info"]["backend"] == "cuda"


def test_d_bad_model_compile_value_error():
    with pytest.raises(ValueError):
        cb.compile(_three_links_one_joint())


# ---------------------------------------------------------------- (e) numerics
def test_e_gantry_workspace_complete_and_prm_cancel():
    m = gantry(1.0)
    res = analyze_workspace(m, {"n_samples": 20000, "seed": 1, "want_metrics": False},
                            cb.CancelToken(), None)
    assert res["n_outside"] == 0

    p = planar_2r()
    evt = threading.Event()
    threading.Timer(0.1, evt.set).start()
    t0 = time.perf_counter()
    r = plan(p, [0.0, 0.0], [1.0, 1.0], _Blocked([0.0, 0.0], [1.0, 1.0]), planner="prm",
             time_limit=30, seed=0, cancel=evt)
    assert time.perf_counter() - t0 < 0.5
    assert not r.success and r.reason == "cancelled"
