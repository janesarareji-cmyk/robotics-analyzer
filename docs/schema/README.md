# Data formats

Links to the schema documents. The text of each format is in the document it links to; this page
gives the overview and the version history.

## Robot model (URM)

The Unified Robot Model is the in-memory and JSON description of a robot (links, joints, tree
structure, origin/axis or DH kinematics, limits, mimic, physical properties, tool and TCP).

-   [URM schema v2](../urm-schema-v2.md): fields, DOF and `q_index` map, active chain, JSON examples,
    link inertial data.
-   [Compiled model layout](../compiled-model-layout.md): the frozen flat layout used by the kinematic and
    GPU code (`layout_version` 1).
-   [Compiled kinematic model](../compiled-model.md): data structures and compile rules.
-   [URDF import](../importers.md): how a URDF becomes a URM (canonical joint order, limits, mimic, meshes).

## Project file (`.robotproj`)

A `.robotproj` file is a zip archive with JSON members. `metadata.format_version` names the format.
Current version: `1.4.0` (`PROJECT_FORMAT_VERSION` in `app/project/project_data.py`).

| Version | Added (optional members) | Reference |
|---|---|---|
| 1.0.0 | base archive: robot model and metadata | [migration and recovery](../project-recovery.md) |
| 1.1.0 | `visuals.json`, `assets/` (meshes) | [importers](../importers.md) |
| 1.2.0 | `trajectory.json` | [trajectory](../trajectory.md) |
| 1.3.0 | `collision.json`, `configs.json` | [collision](../collision.md), [trajectory](../trajectory.md) (format 1.3.0 section) |
| 1.4.0 | none (`configs.json` may hold `shown`, the shown ghosts) | [trajectory](../trajectory.md) (format 1.4.0 section) |

Rules that hold for every version:

-   Files written by an older version always load. A migration works in memory and never changes the
    file; the file is rewritten only on save.
-   A file with a newer `format_version` than the application is refused, with a message.
-   Before the first save of a migrated file, the old file is copied to `<name>.v<old>.bak.robotproj`
    (created once, never overwritten).
-   Saves are atomic. Autosave and recovery files are described in [project recovery](../project-recovery.md).

The migration code is `app/project/migrations.py`; how to add a version is in
[developer notes](../developer-notes.md#adding-a-project-format-migration).

## Simulation and graph exports

-   CSV: a joined table with a `# columns` header line. NPZ: the same arrays as a numpy archive.
    Format and column names: [graphs](../graphs.md) (export section).
-   Report files (HTML, PDF) and URDF export: [reports](../reports.md).

## Plugin manifest

`plugin.json` required keys: `name`, `version`, `api_version`, `entry_point`, `kind`; optional
`permissions`, `description`. See [plugin guide](../plugin-guide.md#pluginjson).
