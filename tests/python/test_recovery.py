"""R8.4 atomic save, autosave, crash recovery, diagnosis and the kill-during-save test."""
import json
import os
import random
import shutil
import subprocess
import sys
import textwrap
import time
import zipfile
from pathlib import Path

import pytest
from PySide6.QtCore import QSettings
from PySide6.QtTest import QTest
from PySide6.QtWidgets import QMessageBox

from app.project import diagnose as dg
from app.project import serializer
from app.project.project_controller import ProjectController
from app.project.project_data import ProjectData
from app.project.recovery import (
    autosave_path,
    load_best,
    newer_autosave,
    remove_autosave,
    write_autosave,
)
from app.project.serializer import ProjectSerializationError, load, prev_path, save
from app.settings import AppSettings

FIX = Path(__file__).parent / "fixtures" / "projects"
REPO = Path(__file__).resolve().parents[2]


@pytest.fixture
def settings(monkeypatch):
    s = QSettings("RoboticsAnalyzer_test", "RoboticsAnalyzer_test")
    s.clear()
    s.sync()
    monkeypatch.setattr(AppSettings, "_ORGANIZATION", "RoboticsAnalyzer_test")
    monkeypatch.setattr(AppSettings, "_APPLICATION", "RoboticsAnalyzer_test")
    yield AppSettings()
    s.clear()
    s.sync()


@pytest.fixture
def controller(qapp, settings):
    c = ProjectController(settings=settings)
    yield c
    c._is_modified = False
    c.close_project(None)


def _dialogs(monkeypatch, answer=QMessageBox.StandardButton.Yes):
    calls = []
    for name in ("critical", "warning", "question", "information"):
        def fake(*a, _n=name, **k):
            calls.append(_n)
            return answer
        monkeypatch.setattr(QMessageBox, name, staticmethod(fake))
    return calls


def _proj(name="p", desc=""):
    p = ProjectData.new(name, desc)
    p.physical_properties = {"k": list(range(50))}
    return p


def _names(d):
    return sorted(f.name for f in Path(d).iterdir())


# ------------------------------------------------------------------ atomic save
def test_save_leaves_only_the_project_file(tmp_path):
    out = tmp_path / "a.robotproj"
    save(_proj(), out)
    save(_proj(desc="second"), out)
    assert _names(tmp_path) == ["a.robotproj"]          # no .tmp, no .prev after success
    assert load(out).metadata.description == "second"


def test_write_error_midway_leaves_old_file_and_no_partial(tmp_path, monkeypatch):
    out = tmp_path / "a.robotproj"
    save(_proj(desc="good"), out)
    before = out.read_bytes()
    real = serializer._write_json
    n = {"c": 0}

    def boom(zf, name, data):
        n["c"] += 1
        if n["c"] == 3:
            raise OSError("disk full")
        return real(zf, name, data)

    monkeypatch.setattr(serializer, "_write_json", boom)
    with pytest.raises(ProjectSerializationError):
        save(_proj(desc="bad"), out)
    assert out.read_bytes() == before
    assert _names(tmp_path) == ["a.robotproj"]


def test_unexpected_error_midway_also_cleans_tmp(tmp_path, monkeypatch):
    out = tmp_path / "a.robotproj"
    save(_proj(desc="good"), out)
    before = out.read_bytes()
    monkeypatch.setattr(serializer, "_write_visuals",
                        lambda *a: (_ for _ in ()).throw(RuntimeError("bug")))
    with pytest.raises(RuntimeError):
        save(_proj(), out)
    assert out.read_bytes() == before and _names(tmp_path) == ["a.robotproj"]


def test_failed_final_move_restores_previous(tmp_path, monkeypatch):
    out = tmp_path / "a.robotproj"
    save(_proj(desc="good"), out)
    before = out.read_bytes()
    real = os.replace

    def flaky(src, dst):
        if str(dst) == str(out) and str(src).endswith(".tmp"):
            raise OSError("locked")
        return real(src, dst)

    monkeypatch.setattr(os, "replace", flaky)
    with pytest.raises(ProjectSerializationError):
        save(_proj(desc="new"), out)
    monkeypatch.undo()
    assert out.read_bytes() == before and _names(tmp_path) == ["a.robotproj"]


def test_load_best_uses_prev_when_main_is_damaged(tmp_path):
    out = tmp_path / "a.robotproj"
    save(_proj(desc="orig"), out)
    shutil.copy(out, prev_path(out))
    out.write_bytes(out.read_bytes()[:40])               # truncated main
    proj, used = load_best(out)
    assert used == prev_path(out) and proj.metadata.description == "orig"
    prev_path(out).unlink()
    with pytest.raises(ProjectSerializationError):
        load_best(out)


def test_damaged_main_never_overwrites_good_prev(tmp_path):
    out = tmp_path / "a.robotproj"
    save(_proj(desc="orig"), out)
    shutil.copy(out, prev_path(out))
    out.write_bytes(b"garbage")
    save(_proj(desc="fresh"), out)                       # damaged main is not moved over .prev
    assert load(out).metadata.description == "fresh"
    assert load(prev_path(out)).metadata.description == "orig"


# ---------------------------------------------------------------------- autosave
def test_autosave_name_and_atomic_write(tmp_path):
    p = tmp_path / "robot one.robotproj"
    pr = _proj()
    save(pr, p)
    a = write_autosave(pr, p)
    assert a.name == "robot one.autosave.robotproj" == autosave_path(p).name
    assert load(a).metadata.name == "p"
    assert _names(tmp_path) == ["robot one.autosave.robotproj", "robot one.robotproj"]
    assert pr.file_path == str(p)                         # project state untouched
    remove_autosave(p)
    assert _names(tmp_path) == ["robot one.robotproj"]


def test_autosave_interval_setting_default_and_minimum(settings):
    assert settings.autosave_interval_s() == 120
    settings.set_autosave_interval_s(5)
    assert settings.autosave_interval_s() == 15
    settings.set_autosave_interval_s(300)
    assert settings.autosave_interval_s() == 300


def test_autosave_only_when_modified_and_removed_on_save_and_close(controller, monkeypatch,
                                                                   tmp_path):
    _dialogs(monkeypatch)
    p = tmp_path / "a.robotproj"
    save(_proj(), p)
    assert controller.open_project(None, str(p))
    a = autosave_path(p)
    assert controller.autosave_now() is None and not a.exists()      # clean: no autosave
    controller.mark_modified()
    assert controller.autosave_now() == a and a.is_file()
    assert controller.save_project(None)                             # clean save removes it
    assert not a.exists()
    controller.mark_modified()
    controller.autosave_now()
    assert a.is_file()
    assert controller.autosave_active
    controller._is_modified = False
    assert controller.close_project(None)
    assert not a.exists() and not controller.autosave_active


def test_autosave_timer_fires_with_injected_interval(qapp, settings, monkeypatch, tmp_path):
    _dialogs(monkeypatch)
    c = ProjectController(settings=settings, autosave_interval_ms=20)
    p = tmp_path / "t.robotproj"
    save(_proj(), p)
    assert c.open_project(None, str(p))
    c.mark_modified()
    QTest.qWait(250)
    assert autosave_path(p).is_file()
    c._is_modified = False
    c.close_project(None)
    assert not c.autosave_active
    assert not autosave_path(p).exists()


def test_default_interval_from_setting(controller, settings):
    assert controller.autosave_interval_ms == 120_000
    settings.set_autosave_interval_s(30)
    assert controller.autosave_interval_ms == 30_000
    controller.set_autosave_interval_ms(50)
    assert controller.autosave_interval_ms == 50


def test_unsaved_project_is_not_autosaved(controller, tmp_path):
    controller._set_project(_proj(), modified=True)
    assert controller.autosave_now() is None


# ---------------------------------------------------------------------- recovery
def _crashed_session(tmp_path):
    """A project on disk plus a NEWER autosave with different content."""
    p = tmp_path / "r.robotproj"
    save(_proj(desc="saved"), p)
    pr = _proj(desc="unsaved work")
    a = write_autosave(pr, p)
    now = time.time()
    os.utime(p, (now - 10, now - 10))
    os.utime(a, (now, now))
    return p, a


def test_newer_autosave_detection(tmp_path):
    p, a = _crashed_session(tmp_path)
    assert newer_autosave(p) == a
    os.utime(a, (time.time() - 100, time.time() - 100))
    assert newer_autosave(p) is None
    p.unlink()
    assert newer_autosave(p) == a
    assert newer_autosave(tmp_path / "none.robotproj") is None


def test_recovery_path_picks_the_autosave(controller, monkeypatch, tmp_path):
    calls = _dialogs(monkeypatch, QMessageBox.StandardButton.Yes)
    p, a = _crashed_session(tmp_path)
    assert controller.open_project(None, str(p))
    assert calls == ["question"]
    proj = controller.current_project
    assert proj.metadata.description == "unsaved work"
    assert proj.file_path == str(p) and controller.is_modified     # user must still save
    assert controller.save_project(None)
    assert load(p).metadata.description == "unsaved work"
    assert not a.exists()


def test_recovery_declined_opens_saved_and_drops_autosave(controller, monkeypatch, tmp_path):
    _dialogs(monkeypatch, QMessageBox.StandardButton.No)
    p, a = _crashed_session(tmp_path)
    assert controller.open_project(None, str(p))
    assert controller.current_project.metadata.description == "saved"
    assert not controller.is_modified and not a.exists()


def test_no_prompt_without_autosave(controller, monkeypatch, tmp_path):
    calls = _dialogs(monkeypatch)
    p = tmp_path / "x.robotproj"
    save(_proj(), p)
    assert controller.open_project(None, str(p))
    assert calls == []


def test_open_damaged_main_falls_back_to_prev(controller, monkeypatch, tmp_path):
    calls = _dialogs(monkeypatch)
    p = tmp_path / "d.robotproj"
    save(_proj(desc="orig"), p)
    shutil.copy(p, prev_path(p))
    p.write_bytes(b"not a zip")
    assert controller.open_project(None, str(p))
    assert controller.current_project.metadata.description == "orig"
    assert controller.is_modified and "warning" in calls


def test_startup_recovery_scans_recent_projects(controller, settings, monkeypatch, tmp_path):
    calls = _dialogs(monkeypatch)
    p, a = _crashed_session(tmp_path)
    settings.add_recent_project(str(p))
    assert controller.check_startup_recovery(None)
    assert calls == ["question"]
    assert controller.current_project.metadata.description == "unsaved work"
    assert not controller.check_startup_recovery(None)      # already open


# --------------------------------------------------------------------- diagnosis
DIAG = [
    ("corrupt_not_a_zip", dg.NOT_A_ZIP, "not a ZIP archive"),
    ("corrupt_truncated", dg.NOT_A_ZIP, "truncated"),
    ("corrupt_bad_crc", dg.BAD_CRC, "'robot_model.json' is damaged"),
    ("corrupt_no_metadata", dg.MISSING_METADATA, "no 'project_metadata.json'"),
    ("corrupt_bad_json", dg.INVALID_JSON, "line 3, column"),
    ("corrupt_bad_schema", dg.WRONG_SCHEMA, "unknown integrator 'bogus'"),
    ("corrupt_no_name", dg.WRONG_SCHEMA, "field 'name'"),
    ("corrupt_bad_robot", dg.WRONG_SCHEMA, "not a valid robot model"),
    ("corrupt_missing_asset", dg.MISSING_ASSET, "assets/cube.stl"),
    ("corrupt_unknown_version", dg.UNKNOWN_VERSION, "'2.0.0'"),
    ("corrupt_newer_version", dg.NEWER_VERSION, "format 1.9.0"),
]


@pytest.mark.parametrize("name,code,text", DIAG)
def test_each_corrupt_fixture_gives_expected_diagnosis(name, code, text):
    rep = dg.diagnose(FIX / f"{name}.robotproj")
    assert code in rep.codes, rep.text()
    issue = next(i for i in rep.issues if i.code == code)
    assert text in issue.message and issue.action
    assert not rep.ok and text in rep.text()


def test_clean_fixtures_and_missing_file():
    for v in ("1.0.0", "1.1.0", "1.2.0", "1.3.0"):
        assert dg.diagnose(FIX / f"v{v}.robotproj").ok
    rep = dg.diagnose(FIX / "does_not_exist.robotproj")
    assert rep.codes == [dg.NOT_FOUND]


def test_salvage_loads_what_it_can():
    proj, warns = dg.salvage(FIX / "corrupt_bad_crc.robotproj")        # robot damaged
    assert proj.metadata.name == "fixture six dof" and warns
    assert proj.robot_model.dof() == 0
    assert proj.trajectory is not None and len(proj.configs) == 2

    proj, warns = dg.salvage(FIX / "corrupt_bad_json.robotproj")       # sim settings invalid
    assert proj.robot_model.dof() == 6 and proj.simulation_settings.integrator == "rk4"
    assert any("simulation_settings.json" in w for w in warns)

    proj, warns = dg.salvage(FIX / "corrupt_no_metadata.robotproj")
    assert proj.metadata.name == "corrupt_no_metadata" and proj.robot_model.dof() == 6

    proj, warns = dg.salvage(FIX / "corrupt_missing_asset.robotproj")
    assert proj.visuals[0].fallback and proj.visuals[0].mesh_path is None

    proj, warns = dg.salvage(FIX / "corrupt_not_a_zip.robotproj")
    assert proj is None and warns
    proj, _ = dg.salvage(FIX / "v1.3.0.robotproj")
    assert proj is not None
    proj.cleanup_assets()


# ----------------------------------------------------------------- kill during save
_CHILD = textwrap.dedent('''
    import sys
    sys.path[:0] = [{repo!r}, {bindings!r}]
    from pathlib import Path
    from app.project.project_data import ProjectData
    from app.project.serializer import save
    path, ready = sys.argv[1], Path(sys.argv[2])
    p = ProjectData.new("big")
    p.physical_properties = {{"data": [i * 0.001 for i in range(150000)], "marker": "BIG"}}
    i = 0
    while True:
        save(p, path)
        if i == 0:
            ready.write_text("go")
        i += 1
''')


def test_kill_during_save_never_leaves_truncated_file(tmp_path):
    """Hard-kill a process that saves in a loop at random moments (seeded)."""
    child = tmp_path / "child.py"
    child.write_text(_CHILD.format(repo=str(REPO), bindings=str(REPO / "bindings")))
    path = tmp_path / "big.robotproj"
    big = ProjectData.new("big")
    big.physical_properties = {"data": [i * 0.001 for i in range(150000)], "marker": "BIG"}
    save(big, path)                                    # the original always exists
    rng = random.Random(8402)
    t0 = time.time()
    opened_from_prev = 0
    for run in range(8):
        ready = tmp_path / f"ready{run}"
        proc = subprocess.Popen([sys.executable, str(child), str(path), str(ready)],
                                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        try:
            deadline = time.time() + 20
            while not ready.exists() and time.time() < deadline:
                time.sleep(0.01)
            assert ready.exists(), "child never finished its first save"
            time.sleep(rng.uniform(0.0, 0.35))
        finally:
            proc.kill()
            proc.wait()
        # the main file is either complete or absent (kill between the two moves);
        # then the .prev is complete. Never a truncated archive.
        if path.exists():
            assert zipfile.is_zipfile(path)
            with zipfile.ZipFile(path) as zf:
                assert zf.testzip() is None
        proj, used = load_best(path)
        if used != path:
            opened_from_prev += 1
        assert proj.metadata.name == "big"
        assert proj.physical_properties["marker"] == "BIG"
        assert len(proj.physical_properties["data"]) == 150000
        assert proj.physical_properties["data"][-1] == pytest.approx(149.999)
        # the next run starts from a clean main file (user would re-save)
        if used != path:
            shutil.copy(used, path)
        for junk in tmp_path.glob("big.robotproj.*"):
            junk.unlink()
    assert time.time() - t0 < 60
