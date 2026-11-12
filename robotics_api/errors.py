"""Exceptions of the scripting API (R8.1).

Every error raised by ``robotics_api`` is a ``RoboticsApiError``. Raw exceptions of the
bridge (``core_bridge``) or of the app never leak: they are mapped and chained (``__cause__``).
"""
from __future__ import annotations


class RoboticsApiError(Exception):
    """Base class of all API errors."""


class InvalidArgument(RoboticsApiError, ValueError):
    """A bad argument: wrong shape, wrong number of joints (n_dof), non-finite value, bad name."""


class ProjectError(RoboticsApiError):
    """A project file cannot be read, written or has a bad format."""


class ImportFailed(RoboticsApiError):
    """A robot file (URDF) cannot be imported. ``report`` holds the import report text."""

    def __init__(self, message: str, report: str = ""):
        super().__init__(message)
        self.report = report


class ComputationError(RoboticsApiError):
    """A numerical computation failed (kinematics, dynamics, IK)."""


class SimulationError(RoboticsApiError):
    """The simulation failed (for example the mass matrix is not positive definite)."""


class PlanningError(RoboticsApiError):
    """No collision free path was found, or the planner could not start."""


class AnalysisError(RoboticsApiError):
    """A workspace or other analysis failed."""


class ExportError(RoboticsApiError):
    """Data cannot be written to a file."""


class ScriptCancelled(RoboticsApiError):
    """The script was cancelled (cooperative cancel, see ``check_cancel``)."""


class ScriptTimeLimit(ScriptCancelled):
    """The script ran into the time limit setting."""
