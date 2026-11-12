"""robotics_api: stable Python scripting API of Robotics Analyzer (R8.1).

Versioning (semantic): ``__api_version__ = "MAJOR.MINOR"``. A MINOR release only ADDS
(functions, optional arguments, dict keys, result fields). A MAJOR release may break existing
calls. Scripts written for ``1.x`` keep working on every later ``1.y``.

The API exposes plain Python and numpy types only. Units: metres, radians, seconds, newtons,
newton metres, kilograms. Every error is a ``RoboticsApiError``. Long calls (``simulate``,
``plan``, ``workspace``) can be cancelled by the script runner (``check_cancel``).

Quick start::

    import robotics_api as api
    robot = api.import_urdf("planar_2r.urdf")
    T = robot.fk([0.3, -0.4])          # 4x4 TCP pose, metres
"""
from ._cancel import CancelToken, cancel_scope, check_cancel, is_cancelled
from .analysis import export_csv, export_npz, export_urdf, generate_report, workspace
from .errors import (AnalysisError, ComputationError, ExportError, ImportFailed,
                     InvalidArgument, PlanningError, ProjectError, RoboticsApiError,
                     ScriptCancelled, ScriptTimeLimit, SimulationError)
from .motion import Trajectory, plan, simulate, time_parametrize
from .project import Project, import_urdf, new_project, open_project, save_project
from .robot import Robot

__api_version__ = "1.1"

__all__ = [
    "__api_version__", "Robot", "Project", "Trajectory",
    "open_project", "new_project", "save_project", "import_urdf",
    "simulate", "plan", "time_parametrize", "workspace", "export_csv", "export_npz",
    "export_urdf", "generate_report",
    "check_cancel", "is_cancelled", "CancelToken", "cancel_scope",
    "RoboticsApiError", "InvalidArgument", "ProjectError", "ImportFailed", "ComputationError",
    "SimulationError", "PlanningError", "AnalysisError", "ExportError", "ScriptCancelled",
    "ScriptTimeLimit",
]
