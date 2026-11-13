"""
app/importers/import_report.py
R3.3: Shared import report types and issue codes (used by URDF import and R3.6).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, List, Optional

# ---------------------------------------------------------------------------
# Severities
# ---------------------------------------------------------------------------

SEV_ERROR = "error"
SEV_WARNING = "warning"
SEV_INFO = "info"

# ---------------------------------------------------------------------------
# Issue codes (R3.3). New codes go here and into docs/import-report.md (R3.6).
# ---------------------------------------------------------------------------

URDF_FILE_NOT_FOUND = "URDF_FILE_NOT_FOUND"
URDF_BAD_XML = "URDF_BAD_XML"
URDF_XACRO = "URDF_XACRO"
URDF_UNSUPPORTED_JOINT = "URDF_UNSUPPORTED_JOINT"
URDF_UNKNOWN_JOINT_TYPE = "URDF_UNKNOWN_JOINT_TYPE"
URDF_MISSING_LINK = "URDF_MISSING_LINK"
URDF_DUPLICATE_NAME = "URDF_DUPLICATE_NAME"
URDF_MULTIPLE_PARENTS = "URDF_MULTIPLE_PARENTS"
URDF_MULTIPLE_ROOTS = "URDF_MULTIPLE_ROOTS"
URDF_CYCLE = "URDF_CYCLE"
URDF_ZERO_AXIS = "URDF_ZERO_AXIS"
URDF_MIMIC_UNKNOWN = "URDF_MIMIC_UNKNOWN"
URDF_INVALID_MODEL = "URDF_INVALID_MODEL"
URDF_CONTINUOUS_LIMITS = "URDF_CONTINUOUS_LIMITS"
URDF_MISSING_LIMIT = "URDF_MISSING_LIMIT"
URDF_LIMIT_MISSING = "URDF_LIMIT_MISSING"
URDF_NON_FINITE = "URDF_NON_FINITE"
URDF_IGNORED_ELEMENT = "URDF_IGNORED_ELEMENT"
URDF_TCP_CHOSEN = "URDF_TCP_CHOSEN"
# R3.5
URDF_MESH_MISSING = "URDF_MESH_MISSING"
URDF_MESH_FORMAT = "URDF_MESH_FORMAT"
URDF_MESH_TOO_LARGE = "URDF_MESH_TOO_LARGE"
URDF_EXTRA_VISUAL = "URDF_EXTRA_VISUAL"
# R3.7
URDF_NO_INERTIAL = "URDF_NO_INERTIAL"
URDF_ROOT_NO_INERTIAL = "URDF_ROOT_NO_INERTIAL"
URM_PHYSICAL = "URM_PHYSICAL"


@dataclass
class ImportIssue:
    """One problem or note found during import."""

    severity: str
    code: str
    message: str
    element: Optional[str] = None


@dataclass
class ImportReport:
    """List of issues from one import."""

    issues: List[ImportIssue] = field(default_factory=list)

    def add(
        self,
        severity: str,
        code: str,
        message: str,
        element: Optional[str] = None,
    ) -> ImportIssue:
        issue = ImportIssue(severity, code, message, element)
        self.issues.append(issue)
        return issue

    def has_errors(self) -> bool:
        return any(i.severity == SEV_ERROR for i in self.issues)

    def by_severity(self, severity: str) -> List[ImportIssue]:
        return [i for i in self.issues if i.severity == severity]

    def by_code(self, code: str) -> List[ImportIssue]:
        return [i for i in self.issues if i.code == code]

    def summary(self) -> str:
        """Fixed summary line: 'N errors, M warnings, K info'."""
        return (
            f"{len(self.by_severity(SEV_ERROR))} errors, "
            f"{len(self.by_severity(SEV_WARNING))} warnings, "
            f"{len(self.by_severity(SEV_INFO))} info"
        )

    def to_text(self) -> str:
        """One line per issue: '[SEVERITY] CODE: message (element)', then the summary line."""
        lines = []
        for i in self.issues:
            where = f" ({i.element})" if i.element else ""
            lines.append(f"[{i.severity.upper()}] {i.code}: {i.message}{where}")
        lines.append(self.summary())
        return "\n".join(lines)


@dataclass
class ImportResult:
    """Result of an import. `model` is None when the import failed."""

    model: Any  # core_bridge.RobotModel | None
    report: ImportReport = field(default_factory=ImportReport)
    visuals: List[Any] = field(default_factory=list)
    collisions: List[Any] = field(default_factory=list)
