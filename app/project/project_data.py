"""
app/project/project_data.py
Robotics Analyzer — project data model.
R1.5: ProjectData dataclass representing one robot project in memory.

The .robotproj file format is a ZIP archive containing:
  project_metadata.json  — name, description, version, timestamps
  robot_model.json       — Unified Robot Model (populated in R2)
  physical_properties.json — mass, inertia, COM (populated in R3)
  simulation_settings.json — integrator, timestep, etc. (populated in R6)

This file defines the in-memory representation only.
See serializer.py for load/save operations.
"""

from __future__ import annotations

import uuid
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Any

from app.configs.store import ConfigStore
from app.target import TargetStore


# Current file format version. Increment on breaking schema changes.
PROJECT_FORMAT_VERSION = "1.4.0"


@dataclass
class ProjectMetadata:
    """Project identification and audit fields."""

    name: str
    description: str = ""
    project_id: str = field(default_factory=lambda: str(uuid.uuid4()))
    format_version: str = PROJECT_FORMAT_VERSION
    created_at: str = field(
        default_factory=lambda: datetime.now(timezone.utc).isoformat()
    )
    modified_at: str = field(
        default_factory=lambda: datetime.now(timezone.utc).isoformat()
    )

    def touch(self) -> None:
        """Update modified_at to the current UTC time."""
        self.modified_at = datetime.now(timezone.utc).isoformat()

    def to_dict(self) -> dict[str, Any]:
        """Serialize to a JSON-compatible dict."""
        return {
            "name": self.name,
            "description": self.description,
            "project_id": self.project_id,
            "format_version": self.format_version,
            "created_at": self.created_at,
            "modified_at": self.modified_at,
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> ProjectMetadata:
        """Deserialize from a dict (loaded from JSON)."""
        return cls(
            name=data["name"],
            description=data.get("description", ""),
            project_id=data.get("project_id", str(uuid.uuid4())),
            format_version=data.get("format_version", PROJECT_FORMAT_VERSION),
            created_at=data.get("created_at", datetime.now(timezone.utc).isoformat()),
            modified_at=data.get("modified_at", datetime.now(timezone.utc).isoformat()),
        )


INTEGRATORS = ("euler", "rk4", "rk45")


@dataclass
class SimulationSettings:
    """Simulation configuration. Populated fully in R6."""

    integrator: str = "rk4"
    physics_timestep: float = 0.001       # seconds
    render_framerate: float = 60.0        # Hz
    gravity: list[float] = field(default_factory=lambda: [0.0, 0.0, -9.81])
    # RK45 (Dormand-Prince) controls, used when integrator == "rk45" (R6.2)
    rk45_atol: float = 1e-9
    rk45_rtol: float = 1e-7
    rk45_dt_min: float = 1e-7
    rk45_dt_max: float = 0.01
    # Computed-torque tracking gains per DOF (R6.4); None = defaults (Kp 100, Kd 20)
    track_kp: list[float] | None = None
    track_kd: list[float] | None = None
    # Recorder (R6.5)
    record_memory_mb: float = 64.0
    record_every: int = 1
    record_overwrite: bool = True

    def __post_init__(self) -> None:
        if not (1 <= float(self.record_memory_mb) <= 2048):
            raise ValueError("record_memory_mb must be in [1, 2048]")
        if int(self.record_every) < 1:
            raise ValueError("record_every must be >= 1")
        if self.integrator not in INTEGRATORS:
            raise ValueError(f"unknown integrator {self.integrator!r}")

    def to_dict(self) -> dict[str, Any]:
        return {
            "integrator": self.integrator,
            "physics_timestep": self.physics_timestep,
            "render_framerate": self.render_framerate,
            "gravity": self.gravity,
            "rk45_atol": self.rk45_atol,
            "rk45_rtol": self.rk45_rtol,
            "rk45_dt_min": self.rk45_dt_min,
            "rk45_dt_max": self.rk45_dt_max,
            "track_kp": self.track_kp,
            "track_kd": self.track_kd,
            "record_memory_mb": self.record_memory_mb,
            "record_every": self.record_every,
            "record_overwrite": self.record_overwrite,
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> SimulationSettings:
        return cls(
            integrator=data.get("integrator", "rk4"),
            physics_timestep=float(data.get("physics_timestep", 0.001)),
            render_framerate=float(data.get("render_framerate", 60.0)),
            gravity=list(data.get("gravity", [0.0, 0.0, -9.81])),
            rk45_atol=float(data.get("rk45_atol", 1e-9)),
            rk45_rtol=float(data.get("rk45_rtol", 1e-7)),
            rk45_dt_min=float(data.get("rk45_dt_min", 1e-7)),
            rk45_dt_max=float(data.get("rk45_dt_max", 0.01)),
            track_kp=data.get("track_kp"),
            track_kd=data.get("track_kd"),
            record_memory_mb=float(data.get("record_memory_mb", 64.0)),
            record_every=int(data.get("record_every", 1)),
            record_overwrite=bool(data.get("record_overwrite", True)),
        )


@dataclass
class ProjectData:
    """
    In-memory representation of one Robotics Analyzer project.

    Fields populated now (R1.5, R2.6):
        metadata, simulation_settings, robot_model, file_path

    Fields populated later:
        physical_properties — R3 (mass, inertia, COM)
    """

    metadata: ProjectMetadata
    simulation_settings: SimulationSettings = field(
        default_factory=SimulationSettings
    )
    robot_model: Any = None                            # Populated in R2 (cb.RobotModel)
    physical_properties: dict[str, Any] | None = None  # Populated in R3
    file_path: str | None = None                       # Set after first save
    visuals: list[Any] = field(default_factory=list)       # R3.5 LinkVisual
    collisions: list[Any] = field(default_factory=list)    # R3.5 LinkVisual
    asset_dir: str | None = None   # temp dir with extracted meshes (R3.5)
    trajectory: Any = None         # app.sim.trajectory.Trajectory or None (R6.4)
    collision: Any = None          # app.planning.collision.CollisionConfig or None (R7.2)
    configs: Any = field(default_factory=ConfigStore)   # named joint configurations (R7.6)
    targets: Any = field(default_factory=TargetStore)   # named TCP targets (U4a.1, optional file)
    migrated_from: str | None = None   # format version of the file if it was migrated (R8.4)

    def __post_init__(self) -> None:
        """Ensure robot_model is always a valid RobotModel instance."""
        if self.robot_model is None:
            import core_bridge as cb
            self.robot_model = cb.RobotModel()

    @property
    def robot(self) -> Any:
        """Alias for robot_model."""
        return self.robot_model

    @robot.setter
    def robot(self, value: Any) -> None:
        self.robot_model = value

    @classmethod
    def new(cls, name: str, description: str = "") -> ProjectData:
        """Create a new empty project with the given name."""
        return cls(
            metadata=ProjectMetadata(name=name, description=description)
        )

    def touch(self) -> None:
        """Mark the project as modified (update timestamp)."""
        self.metadata.touch()

    def cleanup_assets(self) -> None:
        """Remove the temp dir of extracted meshes, if any."""
        if self.asset_dir:
            import shutil
            shutil.rmtree(self.asset_dir, ignore_errors=True)
            self.asset_dir = None

    @property
    def name(self) -> str:
        """Return the project name."""
        return self.metadata.name

    @property
    def is_saved(self) -> bool:
        """Return True if the project has been saved to a file."""
        return self.file_path is not None
