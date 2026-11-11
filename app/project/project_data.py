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


# Current file format version. Increment on breaking schema changes.
PROJECT_FORMAT_VERSION = "1.0.0"


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


@dataclass
class SimulationSettings:
    """Simulation configuration. Populated fully in R6."""

    integrator: str = "rk4"
    physics_timestep: float = 0.001       # seconds
    render_framerate: float = 60.0        # Hz
    gravity: list[float] = field(default_factory=lambda: [0.0, 0.0, -9.81])

    def to_dict(self) -> dict[str, Any]:
        return {
            "integrator": self.integrator,
            "physics_timestep": self.physics_timestep,
            "render_framerate": self.render_framerate,
            "gravity": self.gravity,
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> SimulationSettings:
        return cls(
            integrator=data.get("integrator", "rk4"),
            physics_timestep=float(data.get("physics_timestep", 0.001)),
            render_framerate=float(data.get("render_framerate", 60.0)),
            gravity=list(data.get("gravity", [0.0, 0.0, -9.81])),
        )


@dataclass
class ProjectData:
    """
    In-memory representation of one Robotics Analyzer project.

    Fields populated now (R1.5):
        metadata, simulation_settings, file_path

    Fields populated later:
        robot_model         — R2 (Unified Robot Model)
        physical_properties — R3 (mass, inertia, COM)
    """

    metadata: ProjectMetadata
    simulation_settings: SimulationSettings = field(
        default_factory=SimulationSettings
    )
    robot_model: dict[str, Any] | None = None          # Populated in R2
    physical_properties: dict[str, Any] | None = None  # Populated in R3
    file_path: str | None = None                       # Set after first save

    @classmethod
    def new(cls, name: str, description: str = "") -> ProjectData:
        """Create a new empty project with the given name."""
        return cls(
            metadata=ProjectMetadata(name=name, description=description)
        )

    def touch(self) -> None:
        """Mark the project as modified (update timestamp)."""
        self.metadata.touch()

    @property
    def name(self) -> str:
        """Return the project name."""
        return self.metadata.name

    @property
    def is_saved(self) -> bool:
        """Return True if the project has been saved to a file."""
        return self.file_path is not None
