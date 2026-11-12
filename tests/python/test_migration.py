"""R8.4 project migration: stored fixtures of every old format version, backup, newer-file refusal."""
import importlib.util
import json
import shutil
import zipfile
from pathlib import Path

import pytest
from PySide6.QtCore import QSettings
from PySide6.QtWidgets import QMessageBox

from app.project import migrations as mig
from app.project.project_controller import ProjectController
from app.project.project_data import PROJECT_FORMAT_VERSION
from app.project.serializer import ProjectFormatError, load, save
from app.settings import AppSettings

FIX = Path(__file__).parent / "fixtures" / "projects"
VERSIONS = ("1.0.0", "1.1.0", "1.2.0", "1.3.0")

_spec = importlib.util.spec_from_file_location("make_fixtures", FIX / "make_fixtures.py")
make_fixtures = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(make_fixtures)


@pytest.fixture(scope="module")
def reference():
    return make_fixtures.reference_project()


@pytest.fixture
def controller(qapp, monkeypatch):
    s = QSettings("RoboticsAnalyzer_test", "RoboticsAnalyzer_test")
    s.clear()
    s.sync()
    monkeypatch.setattr(AppSettings, "_ORGANIZATION", "RoboticsAnalyzer_test")
    monkeypatch.setattr(AppSettings, "_APPLICATION", "RoboticsAnalyzer_test")
    c = ProjectController(settings=AppSettings())
    yield c
    c._is_modified = False
    c.close_project(None)
    s.clear()
    s.sync()


def _no_dialogs(monkeypatch):
    calls = []
    for name in ("critical", "warning", "question", "information"):
        def fake(*a, _n=name, **k):
            calls.append((_n, a))
            return QMessageBox.StandardButton.Yes
        monkeypatch.setattr(QMessageBox, name, staticmethod(fake))
    return calls


def _same_visual(a, b):
    da, db = a.to_dict(), b.to_dict()
    da.pop("mesh_path"), db.pop("mesh_path")
    return da == db


def assert_matches(proj, ref, version):
    """Field-by-field comparison with the reference; ``version`` says which fields exist."""
    v = tuple(int(x) for x in version.split("."))
    assert proj.metadata.name == ref.metadata.name
    assert proj.metadata.project_id == ref.metadata.project_id
    assert proj.metadata.format_version == version
    assert proj.robot_model.to_json() == ref.robot_model.to_json()
    assert proj.physical_properties == ref.physical_properties
    assert proj.simulation_settings.to_dict() == ref.simulation_settings.to_dict()
    if v >= (1, 1, 0):
        assert len(proj.visuals) == 2 and len(proj.collisions) == 1
        assert all(_same_visual(a, b) for a, b in zip(proj.visuals, ref.visuals))
        assert _same_visual(proj.collisions[0], ref.collisions[0])
        mesh = Path(proj.visuals[0].mesh_path)
        assert mesh.read_bytes() == make_fixtures.MESH.read_bytes()
    else:
        assert proj.visuals == [] and proj.collisions == [] and proj.asset_dir is None
    if v >= (1, 2, 0):
        assert proj.trajectory.to_dict() == ref.trajectory.to_dict()
    else:
        assert proj.trajectory is None
    if v >= (1, 3, 0):
        assert proj.collision.to_dict() == ref.collision.to_dict()
        assert proj.configs.as_dict() == ref.configs.as_dict()
    else:
        assert proj.collision is None and len(proj.configs) == 0


# ------------------------------------------------------------------ registry
def test_every_format_step_has_a_migration():
    assert PROJECT_FORMAT_VERSION == "1.3.0"
    assert mig.VERSIONS[-1] == PROJECT_FORMAT_VERSION
    assert set(mig.MIGRATIONS) == set(mig.VERSIONS[:-1])
    assert all(callable(f) for f in mig.MIGRATIONS.values())


def test_chain_runs_from_each_version():
    assert mig.migrate({}, "1.3.0") == []
    assert mig.migrate({}, "1.2.0") == ["1.2.0"]
    assert mig.migrate({}, "1.0.0") == ["1.0.0", "1.1.0", "1.2.0"]
    with pytest.raises(KeyError):
        mig.migrate({}, "0.9.0")


def test_version_helpers():
    assert mig.parse_version("1.2.0") == (1, 2, 0)
    assert mig.parse_version("x") is None and mig.parse_version(3) is None
    assert mig.is_newer("1.4.0") and mig.is_newer("2.0.0") and not mig.is_newer("1.3.0")
    assert mig.is_older("1.2.0") and not mig.is_older("1.3.0")


# ------------------------------------------------------------------ fixtures
@pytest.mark.parametrize("version", VERSIONS)
def test_stored_fixture_opens_and_equals_reference(version, reference):
    path = FIX / f"v{version}.robotproj"
    with zipfile.ZipFile(path) as zf:
        meta = json.loads(zf.read("project_metadata.json"))
    assert meta["format_version"] == version        # real file of that era
    proj = load(path)
    try:
        assert_matches(proj, reference, version)
        assert proj.migrated_from == (None if version == PROJECT_FORMAT_VERSION else version)
    finally:
        proj.cleanup_assets()


@pytest.mark.parametrize("version", VERSIONS)
def test_fixture_layouts_match_their_era(version):
    with zipfile.ZipFile(FIX / f"v{version}.robotproj") as zf:
        names = set(zf.namelist())
    v = tuple(int(x) for x in version.split("."))
    assert ("visuals.json" in names) == (v >= (1, 1, 0))
    assert (any(n.startswith("assets/") for n in names)) == (v >= (1, 1, 0))
    assert ("trajectory.json" in names) == (v >= (1, 2, 0))
    assert ("collision.json" in names) == (v >= (1, 3, 0))
    assert ("configs.json" in names) == (v >= (1, 3, 0))


@pytest.mark.parametrize("version", VERSIONS[:-1])
def test_save_after_open_writes_current_format(version, reference, tmp_path):
    proj = load(FIX / f"v{version}.robotproj")
    out = tmp_path / "up.robotproj"
    save(proj, out)
    q = load(out)
    assert q.metadata.format_version == PROJECT_FORMAT_VERSION and q.migrated_from is None
    assert q.robot_model.to_json() == reference.robot_model.to_json()
    assert q.simulation_settings.to_dict() == reference.simulation_settings.to_dict()
    proj.cleanup_assets()
    q.cleanup_assets()


# -------------------------------------------------------------------- backup
def test_backup_created_exactly_once_and_original_untouched(controller, monkeypatch, tmp_path):
    _no_dialogs(monkeypatch)
    p = tmp_path / "proj.robotproj"
    shutil.copy(FIX / "v1.1.0.robotproj", p)
    original = p.read_bytes()
    bak = tmp_path / "proj.v1.1.0.bak.robotproj"
    assert controller.open_project(None, str(p))
    assert bak.read_bytes() == original and p.read_bytes() == original
    assert controller.current_project.migrated_from == "1.1.0"
    first_mtime = bak.stat().st_mtime_ns
    controller._is_modified = False
    controller.close_project(None)
    assert controller.open_project(None, str(p))          # second open: no new backup
    assert bak.stat().st_mtime_ns == first_mtime
    assert sorted(f.name for f in tmp_path.iterdir()) == sorted([bak.name, p.name])


def test_current_format_file_gets_no_backup(controller, monkeypatch, tmp_path):
    _no_dialogs(monkeypatch)
    p = tmp_path / "cur.robotproj"
    shutil.copy(FIX / "v1.3.0.robotproj", p)
    assert controller.open_project(None, str(p))
    assert [f.name for f in tmp_path.iterdir()] == ["cur.robotproj"]


def test_migration_is_memory_only_until_user_saves(controller, monkeypatch, tmp_path):
    _no_dialogs(monkeypatch)
    p = tmp_path / "old.robotproj"
    shutil.copy(FIX / "v1.0.0.robotproj", p)
    before = p.read_bytes()
    assert controller.open_project(None, str(p))
    assert p.read_bytes() == before and not controller.is_modified
    assert controller.save_project(None)
    assert p.read_bytes() != before
    assert load(p).metadata.format_version == PROJECT_FORMAT_VERSION


def test_backup_helper_names(tmp_path):
    p = tmp_path / "a b.robotproj"
    p.write_bytes(b"x")
    assert mig.backup_path(p, "1.0.0").name == "a b.v1.0.0.bak.robotproj"
    assert mig.make_backup(p, "1.0.0") is not None
    assert mig.make_backup(p, "1.0.0") is None


# --------------------------------------------------------------- newer / unknown
def test_newer_file_refused_without_writes(controller, monkeypatch, tmp_path):
    calls = _no_dialogs(monkeypatch)
    p = tmp_path / "new.robotproj"
    shutil.copy(FIX / "corrupt_newer_version.robotproj", p)
    before = p.read_bytes()
    with pytest.raises(ProjectFormatError, match="newer version"):
        load(p)
    assert controller.open_project(None, str(p)) is False
    assert not controller.has_project
    assert any(n == "critical" and "newer" in str(a) for n, a in calls)
    assert p.read_bytes() == before
    assert [f.name for f in tmp_path.iterdir()] == ["new.robotproj"]   # no backup, no tmp


def test_unknown_version_refused(tmp_path):
    with pytest.raises(ProjectFormatError, match="newer version"):     # 2.0.0
        load(FIX / "corrupt_unknown_version.robotproj")
    old = tmp_path / "zero.robotproj"
    with zipfile.ZipFile(FIX / "v1.0.0.robotproj") as zin, zipfile.ZipFile(old, "w") as zout:
        for n in zin.namelist():
            data = zin.read(n)
            if n == "project_metadata.json":
                data = data.replace(b'"1.0.0"', b'"0.9.0"')
            zout.writestr(n, data)
    with pytest.raises(ProjectFormatError, match="Incompatible"):
        load(old)
    garbled = tmp_path / "g.robotproj"
    with zipfile.ZipFile(FIX / "v1.0.0.robotproj") as zin, zipfile.ZipFile(garbled, "w") as zout:
        for n in zin.namelist():
            data = zin.read(n)
            if n == "project_metadata.json":
                data = data.replace(b'"1.0.0"', b'"banana"')
            zout.writestr(n, data)
    with pytest.raises(ProjectFormatError, match="Unknown"):
        load(garbled)
