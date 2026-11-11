# Import report

Every import (URDF now, CAD later) returns an `ImportReport`
(`app/importers/import_report.py`). The report is shown in the
**Import Report** dock (View > Import Report) and one line is written to the
Log panel.

## Output format

`ImportReport.to_text()` writes one line per issue, in the order found:

    [SEVERITY] CODE: message (element)

The element part is left out when the issue names no link or joint. The last
line is always the summary:

    N errors, M warnings, K info

Example:

    [WARNING] URDF_MISSING_LIMIT: joint 'j1' has no <limit>; using defaults (j1)
    [WARNING] URDF_MISSING_LIMIT: joint 'j2' <limit> has no effort; using defaults (j2)
    [INFO] URDF_TCP_CHOSEN: tcp_link set to link 'l2' (index 2). It can be changed later (R3.4 'Set TCP link'). (l2)
    0 errors, 2 warnings, 1 info

"Save report..." in the dock writes the full report (all severities) with the
same format to a `.txt` file.

## Dock behaviour

-   The dock lists one row per issue: severity, code, message, element.
-   Filter boxes (Errors, Warnings, Info) hide rows. "Save report..." always
    saves all rows.
-   Clicking a row with an element name selects that joint or link in the
    Robot Model tree. Joints are looked up first, then links. No match, or no
    model loaded, does nothing.
-   Errors show the dock and bring it to the front.
-   Warnings show the dock.
-   Info-only reports update the dock but do not open it. Open it from the View
    menu.
-   No popup is shown for an import report.

## Severity policy

-   **error**: the model is unusable. No model is loaded. The old project is kept.
-   **warning**: the model is loaded, but something was approximated or is
    missing (mesh missing, unsupported mesh format, continuous joint mapped to
    ±π, limit missing). The model is loaded.
-   **info**: a choice was made, or something was skipped on purpose (TCP link
    chosen, element kind ignored, extra visual ignored).

## Code table

Every code constant in `app/importers/` is listed here. The test
`tests/python/test_import_report.py` checks that this table matches the
constants. Add a row here for every new code.

| Code | Severity | When |
|------|----------|------|
| `URDF_FILE_NOT_FOUND` | error | File path missing or unreadable. |
| `URDF_BAD_XML` | error | XML parse error, or root tag is not `<robot>`, or a number attribute is malformed. |
| `URDF_XACRO` | error | xacro namespace or `${` found. Convert with `xacro` first. |
| `URDF_UNSUPPORTED_JOINT` | error | `floating` or `planar` joint. |
| `URDF_UNKNOWN_JOINT_TYPE` | error | Any other unknown joint type. |
| `URDF_MISSING_LINK` | error | Joint parent or child name matches no `<link>`. |
| `URDF_DUPLICATE_NAME` | error | Two links or two joints have the same name. |
| `URDF_MULTIPLE_PARENTS` | error | A link is the child of two joints. |
| `URDF_MULTIPLE_ROOTS` | error | More than one root link. |
| `URDF_CYCLE` | error | No root link (the joints form a cycle), or a loop beside a valid root. |
| `URDF_ZERO_AXIS` | error | `<axis>` has zero length. |
| `URDF_MIMIC_UNKNOWN` | error | `<mimic>` names a joint that does not exist. |
| `URDF_INVALID_MODEL` | error | The built model fails `validate()`. |
| `URDF_CONTINUOUS_LIMITS` | warning | Continuous joint mapped to limits ±π. |
| `URDF_MISSING_LIMIT` | warning | `<limit>` or its velocity/effort is missing. Defaults are used. |
| `URDF_MESH_MISSING` | warning | Mesh file not found in the URDF folder or mesh search paths. A box fallback is drawn. |
| `URDF_MESH_FORMAT` | warning | Mesh format not supported. A box fallback is drawn. |
| `URDF_MESH_TOO_LARGE` | warning | Mesh has too many triangles. It is not drawn. |
| `URDF_IGNORED_ELEMENT` | info | An element kind is skipped (`<transmission>`, `<gazebo>`, `<inertial>` before R3.7, etc.). Once per kind. |
| `URDF_TCP_CHOSEN` | info | Names the link chosen as `tcp_link` (deepest link). It can be changed. |
| `URDF_EXTRA_VISUAL` | info | A second visual on a link is ignored. Only the first is drawn. |

When R3.7 adds physical-data codes (`URDF_NO_INERTIAL`,
`URDF_ROOT_NO_INERTIAL`, and others), they go in this table with the same
policy.
