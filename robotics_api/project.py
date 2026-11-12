"""Project and robot file functions of the API (R8.1)."""
from __future__ import annotations

from pathlib import Path
from typing import Any

from ._util import guard
from .errors import ImportFailed, InvalidArgument, ProjectError
from .robot import Robot


class Project:
    """A robot project (.robotproj). ``robot`` is a snapshot ``Robot``; assign a ``Robot`` to
    replace the model of the project. Plain attributes: ``name``, ``description``, ``path``."""

    def __init__(self, data: Any):
        self._data = data

    @property
    def name(self) -> str:
        """Project name (settable)."""
        return str(self._data.metadata.name)

    @name.setter
    def name(self, value: str) -> None:
        self._data.metadata.name = str(value)

    @property
    def description(self) -> str:
        """Free text description (settable)."""
        return str(self._data.metadata.description)

    @description.setter
    def description(self, value: str) -> None:
        self._data.metadata.description = str(value)

    @property
    def path(self) -> str | None:
        """File path after open or save, else None."""
        return self._data.file_path

    @property
    def robot(self) -> Robot:
        """Snapshot of the project robot (a change to it does not change the project)."""
        import core_bridge as cb
        m = self._data.robot_model
        return Robot(cb.urm_from_json(m.to_json()), self.name, self._data.visuals,
                     self._data.collisions)

    @robot.setter
    def robot(self, robot: Robot) -> None:
        import core_bridge as cb
        if not isinstance(robot, Robot):
            raise InvalidArgument("robot must be a robotics_api.Robot")
        self._data.robot_model = cb.urm_from_json(robot._model.to_json())

    def __repr__(self) -> str:
        return f"Project(name={self.name!r}, path={self.path!r})"


@guard(ProjectError)
def new_project(name: str = "Untitled") -> Project:
    """A new empty project (not saved)."""
    from app.project.project_data import ProjectData
    return Project(ProjectData.new(str(name)))


@guard(ProjectError)
def open_project(path: str) -> Project:
    """Open a .robotproj file. Raises ``ProjectError`` for a missing or damaged file."""
    from app.project import serializer
    p = Path(path)
    if not p.is_file():
        raise ProjectError(f"project file not found: {p}")
    return Project(serializer.load(p))


@guard(ProjectError)
def save_project(project: Project, path: str) -> str:
    """Save ``project`` to ``path`` (must end with .robotproj). Returns the path."""
    from app.project import serializer
    if not isinstance(project, Project):
        raise InvalidArgument("project must be a robotics_api.Project")
    serializer.save(project._data, path)
    return str(path)


@guard(ImportFailed)
def import_urdf(path: str, search_roots: list[str] | None = None) -> Robot:
    """Import a URDF file and return the ``Robot`` (meshes are resolved next to the file or in
    ``search_roots``). ``robot.import_report`` holds the warnings. Raises ``ImportFailed`` (with
    ``.report``) when the file cannot be imported."""
    from app.importers import import_urdf as _imp
    res = _imp(str(path), search_roots)
    text = res.report.to_text()
    if res.model is None:
        raise ImportFailed(f"cannot import {path}:\n{text}", text)
    robot = Robot(res.model, Path(path).stem, res.visuals, res.collisions)
    robot.import_report = text
    return robot
