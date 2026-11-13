"""A1.2: load and save safety (audit H2, M8, LOW recover-No / backup / configs)."""
import json
import logging
import os
import shutil
import zipfile
from pathlib import Path

import pytest
from PySide6.QtCore import QSettings
from PySide6.QtWidgets import QMessageBox

from app.project import serializer
from app.project.migrations import make_backup
from app.project.project_controller import ProjectController
from app.project.project_data import ProjectData
from app.project.recovery import load_best, write_autosave
from app.project.serializer import ProjectFormatError, ProjectSerializationError, load, save
from app.settings import AppSettings
from app.configs.store import ConfigStore


@pytest.fixture
def controller(qapp, monkeypatch):
    org = "RoboticsAnalyzer_test_a12"
    s = QSettings(org, org)
    s.clear()
    s.sync()
    monkeypatch.setattr(AppSettings, "_ORGANIZATION", org)
    monkeypatch.setattr(AppSettings, "_APPLICATION", org)
    c = ProjectController(settings=AppSettings())
    yield c
    c._is_modified = False
    c.close_project(None)
    s.clear()
    s.sync()


def _good(tmp_path, name="a.robotproj", desc="good"):
    p = ProjectData.new("p", desc)
    out = tmp_path / name
    save(p, out)
    return out


def _rewrite(path, **members):
    """Rewrite the archive; a member value None removes it, bytes replaces it, else JSON."""
    with zipfile.ZipFile(path) as zf:
        items = {n: zf.read(n) for n in zf.namelist()}
    for k, v in members.items():
        name = k.replace("__", ".")
        if v is None:
            items.pop(name, None)
        elif isinstance(v, bytes):
            items[name] = v
        else:
            items[name] = json.dumps(v).encode()
    with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED) as zf:
        for n, b in items.items():
            zf.writestr(n, b)


def _meta(path):
    with zipfile.ZipFile(path) as zf:
        return json.loads(zf.read("project_metadata.json"))


def _corrupt_kinds(path):
    nm = dict(_meta(path))
    del nm["name"]
    return [
        ("no name", dict(project_metadata__json=nm)),
        ("meta list", dict(project_metadata__json=[1, 2])),
        ("bad sim", dict(simulation_settings__json={"integrator": "bogus"})),
        ("non utf8 robot", dict(robot_model__json=b"\xff\xfe\x00bad")),
        ("visual no link_index",
         dict(visuals__json={"visuals": [{"kind": "box"}], "collisions": []})),
    ]


@pytest.mark.parametrize("idx", range(5))
def test_corruption_maps_to_format_error_and_prev_fallback(tmp_path, idx):
    """Audit H2: every corruption kind -> ProjectFormatError; load_best uses .prev."""
    out = _good(tmp_path)
    label, change = _corrupt_kinds(out)[idx]
    _rewrite(out, **change)
    with pytest.raises(ProjectFormatError):
        load(out)
    prev = serializer.prev_path(out)
    shutil.copy2(_good(tmp_path, "good.robotproj", desc="from-prev"), prev)
    proj, opened = load_best(out)
    assert Path(opened) == prev
    assert proj.metadata.description == "from-prev"


def test_damaged_deflate_stream_is_serialization_error(tmp_path):
    """Audit H2: damaged compressed bytes (zlib.error) -> ProjectSerializationError."""
    out = _good(tmp_path)
    with zipfile.ZipFile(out) as zf:
        zi = zf.getinfo("project_metadata.json")
    raw = bytearray(out.read_bytes())
    start = zi.header_offset + 30 + len(zi.filename.encode()) + len(zi.extra)
    for i in range(start, start + zi.compress_size):
        raw[i] = 0xFF
    out.write_bytes(bytes(raw))
    with pytest.raises(ProjectSerializationError):
        load(out)


def test_open_project_corrupt_shows_dialog_not_exception(controller, tmp_path, monkeypatch):
    """Audit H2: open_project on a bad-value file shows a dialog and returns False."""
    out = _good(tmp_path)
    _rewrite(out, simulation_settings__json={"integrator": "bogus"})
    shown = []
    monkeypatch.setattr(QMessageBox, "critical", staticmethod(lambda *a, **k: shown.append(a)))
    assert controller.open_project(None, str(out)) is False
    assert shown


def test_replace_failure_keeps_target(tmp_path, monkeypatch):
    """Audit M8: the target is never moved away; a failing replace leaves the old file."""
    out = _good(tmp_path, desc="old")
    before = out.read_bytes()
    real = os.replace
    calls = []

    def fake(src, dst, *a, **k):
        calls.append((str(src), str(dst)))
        if len(calls) >= 1:    # the only replace is tmp -> target
            raise OSError("boom")
        return real(src, dst, *a, **k)

    monkeypatch.setattr(os, "replace", fake)
    with pytest.raises(ProjectSerializationError):
        save(ProjectData.new("p", "new"), out)
    monkeypatch.setattr(os, "replace", real)
    assert out.exists() and out.read_bytes() == before
    assert not out.with_name(out.name + ".tmp").exists()


def test_target_exists_at_every_replace(tmp_path, monkeypatch):
    """Audit M8: at every os.replace during a save the target file exists."""
    out = _good(tmp_path)
    real = os.replace
    seen = []

    def spy(src, dst, *a, **k):
        seen.append(Path(out).exists())
        return real(src, dst, *a, **k)

    monkeypatch.setattr(os, "replace", spy)
    save(ProjectData.new("p", "again"), out)
    assert seen and all(seen)


def test_recover_no_with_corrupt_main_keeps_autosave(controller, tmp_path, monkeypatch):
    """LOW: answering No to Recover must not delete the autosave when the main file fails."""
    out = _good(tmp_path)
    a = write_autosave(ProjectData.new("p", "autosaved"), out)
    os.utime(a, (a.stat().st_atime, a.stat().st_mtime + 100))
    _rewrite(out, simulation_settings__json={"integrator": "bogus"})
    monkeypatch.setattr(QMessageBox, "question",
                        staticmethod(lambda *a, **k: QMessageBox.StandardButton.No))
    monkeypatch.setattr(QMessageBox, "critical", staticmethod(lambda *a, **k: None))
    assert controller.open_project(None, str(out)) is False
    assert a.is_file()


def test_recover_no_with_good_main_removes_autosave(controller, tmp_path, monkeypatch):
    """LOW: No + good main file: autosave is removed after the load succeeded."""
    out = _good(tmp_path)
    a = write_autosave(ProjectData.new("p", "x"), out)
    os.utime(a, (a.stat().st_atime, a.stat().st_mtime + 100))
    monkeypatch.setattr(QMessageBox, "question",
                        staticmethod(lambda *a, **k: QMessageBox.StandardButton.No))
    assert controller.open_project(None, str(out)) is True
    assert not a.exists()


def test_two_old_files_same_stem_two_backups(tmp_path):
    """LOW: a different old file with the same stem gets its own numbered backup."""
    f = tmp_path / "proj.robotproj"
    f.write_bytes(b"first")
    b1 = make_backup(f, "1.0.0")
    f.write_bytes(b"second-different")
    b2 = make_backup(f, "1.0.0")
    assert b1 is not None and b2 is not None and b1 != b2
    assert b1.read_bytes() == b"first" and b2.read_bytes() == b"second-different"
    assert make_backup(f, "1.0.0") is None            # same content: skipped
    assert sorted(p.name for p in tmp_path.glob("*.bak*")) == [
        "proj.v1.0.0.bak.1.robotproj", "proj.v1.0.0.bak.robotproj"]


def test_configs_bad_entry_skipped_others_load(caplog):
    """LOW: one bad configs.json entry does not drop the others."""
    d = {"configs": [{"name": "ok", "q": [0.0, 1.0]},
                     {"q": [1.0]},
                     {"name": "nan", "q": [float("nan"), 0.0]},
                     {"name": "short", "q": "x"},
                     {"name": "ok2", "q": [2.0, 3.0]}]}
    with caplog.at_level(logging.WARNING):
        s = ConfigStore.from_dict(d)
    assert s.names == ["ok", "ok2"]
    msgs = " ".join(r.getMessage() for r in caplog.records)
    assert "1" in msgs and "2" in msgs and "3" in msgs
