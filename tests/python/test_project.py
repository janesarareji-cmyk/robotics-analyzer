"""
tests/python/test_project.py
R1.5: ProjectData and serializer unit tests.
"""

import os
import pytest
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from app.project.project_data import (
    ProjectData, ProjectMetadata, SimulationSettings, PROJECT_FORMAT_VERSION
)
from app.project.serializer import (
    save, load, ProjectSerializationError, ProjectFormatError
)


# ------------------------------------------------------------------
# ProjectMetadata tests
# ------------------------------------------------------------------

def test_metadata_creation():
    """ProjectMetadata creates with required fields."""
    meta = ProjectMetadata(name="TestRobot")
    assert meta.name == "TestRobot"
    assert meta.format_version == PROJECT_FORMAT_VERSION
    assert meta.project_id != ""


def test_metadata_to_from_dict():
    """ProjectMetadata round-trips through dict serialization."""
    meta = ProjectMetadata(name="Robot1", description="Test")
    d = meta.to_dict()
    meta2 = ProjectMetadata.from_dict(d)
    assert meta2.name == meta.name
    assert meta2.description == meta.description
    assert meta2.project_id == meta.project_id


def test_metadata_touch_updates_modified_at():
    """touch() updates modified_at timestamp."""
    meta = ProjectMetadata(name="R")
    original = meta.modified_at
    import time; time.sleep(0.01)
    meta.touch()
    assert meta.modified_at != original


# ------------------------------------------------------------------
# SimulationSettings tests
# ------------------------------------------------------------------

def test_simulation_settings_defaults():
    """SimulationSettings has correct default values."""
    s = SimulationSettings()
    assert s.integrator == "rk4"
    assert s.physics_timestep == pytest.approx(0.001)
    assert s.render_framerate == pytest.approx(60.0)
    assert len(s.gravity) == 3
    assert s.gravity[2] == pytest.approx(-9.81)


def test_simulation_settings_to_from_dict():
    """SimulationSettings round-trips through dict serialization."""
    s = SimulationSettings(integrator="euler", physics_timestep=0.005)
    d = s.to_dict()
    s2 = SimulationSettings.from_dict(d)
    assert s2.integrator == "euler"
    assert s2.physics_timestep == pytest.approx(0.005)


# ------------------------------------------------------------------
# ProjectData tests
# ------------------------------------------------------------------

def test_project_data_new():
    """ProjectData.new() creates a fresh project."""
    proj = ProjectData.new("MyRobot", "Test project")
    assert proj.name == "MyRobot"
    assert proj.metadata.description == "Test project"
    assert proj.robot_model is None
    assert proj.physical_properties is None
    assert proj.file_path is None
    assert not proj.is_saved


def test_project_data_touch():
    """ProjectData.touch() updates the metadata timestamp."""
    import time
    proj = ProjectData.new("R")
    original = proj.metadata.modified_at
    time.sleep(0.01)
    proj.touch()
    assert proj.metadata.modified_at != original


# ------------------------------------------------------------------
# Serializer tests
# ------------------------------------------------------------------

def test_save_creates_file(tmp_path):
    """save() creates a .robotproj file at the given path."""
    proj = ProjectData.new("SaveTest")
    path = tmp_path / "test.robotproj"
    save(proj, path)
    assert path.exists()
    assert path.stat().st_size > 0


def test_save_sets_file_path(tmp_path):
    """save() sets project.file_path after saving."""
    proj = ProjectData.new("SaveTest")
    path = tmp_path / "test.robotproj"
    save(proj, path)
    assert proj.file_path == str(path)


def test_save_updates_modified_at(tmp_path):
    """save() updates project.metadata.modified_at."""
    import time
    proj = ProjectData.new("SaveTest")
    original = proj.metadata.modified_at
    time.sleep(0.01)
    path = tmp_path / "test.robotproj"
    save(proj, path)
    assert proj.metadata.modified_at != original


def test_load_round_trip(tmp_path):
    """save() then load() produces an equivalent ProjectData."""
    proj = ProjectData.new("RoundTrip", "A test project")
    proj.simulation_settings.integrator = "euler"
    proj.simulation_settings.physics_timestep = 0.002

    path = tmp_path / "roundtrip.robotproj"
    save(proj, path)

    loaded = load(path)

    assert loaded.name == "RoundTrip"
    assert loaded.metadata.description == "A test project"
    assert loaded.metadata.project_id == proj.metadata.project_id
    assert loaded.simulation_settings.integrator == "euler"
    assert loaded.simulation_settings.physics_timestep == pytest.approx(0.002)
    assert loaded.file_path == str(path)
    assert loaded.is_saved


def test_load_file_not_found(tmp_path):
    """load() raises ProjectSerializationError for missing file."""
    with pytest.raises(ProjectSerializationError):
        load(tmp_path / "nonexistent.robotproj")


def test_load_invalid_zip(tmp_path):
    """load() raises ProjectSerializationError for a corrupt archive."""
    path = tmp_path / "corrupt.robotproj"
    path.write_bytes(b"this is not a zip file")
    with pytest.raises(ProjectSerializationError):
        load(path)


def test_save_wrong_extension(tmp_path):
    """save() raises ProjectSerializationError for wrong file extension."""
    proj = ProjectData.new("Test")
    with pytest.raises(ProjectSerializationError):
        save(proj, tmp_path / "wrong.zip")


def test_load_missing_metadata_raises_format_error(tmp_path):
    """load() raises ProjectFormatError when project_metadata.json is absent."""
    import zipfile
    path = tmp_path / "bad.robotproj"
    with zipfile.ZipFile(path, "w") as zf:
        zf.writestr("other_file.json", "{}")
    with pytest.raises(ProjectFormatError):
        load(path)


def test_save_load_with_robot_model_placeholder(tmp_path):
    """save/load preserves robot_model placeholder dict."""
    proj = ProjectData.new("WithModel")
    proj.robot_model = {"placeholder": True, "dof": 3}
    path = tmp_path / "model.robotproj"
    save(proj, path)
    loaded = load(path)
    assert loaded.robot_model == {"placeholder": True, "dof": 3}
