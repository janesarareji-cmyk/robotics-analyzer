# Importers

Files: `app/importers/urdf.py`, `app/importers/import_report.py`,
`app/importers/visuals.py`. Project archive side: `app/project/serializer.py`.
Introduced in R3.3 (URDF), R3.5 (visuals and meshes), R3.7 (inertial).
Report codes: `docs/import-report.md`. Physical data: `docs/physical-properties.md`.

The importer builds a URM (`core_bridge.RobotModel`) through the C++ bindings.
The URM has no visual data, so visuals and collisions are returned as Python
`LinkVisual` lists beside it.

## Entry point

    import_urdf(path, search_roots=None) -> ImportResult

`ImportResult(model, report, visuals, collisions)`.

- `search_roots`: extra folders for `package://` mesh lookup.
- Bad input never raises. Expected errors become `error` issues in `report`,
  and `model` is `None`. Only `_Fatal` is caught here, so other exceptions
  propagate.
- Parsing uses the stdlib `xml.etree.ElementTree`. No new C++ dependency.

## Stages

1. Read file. Missing file: `URDF_FILE_NOT_FOUND`. Contains `xmlns:xacro`,
   `<xacro:` or `${`: `URDF_XACRO` (fatal, convert with `xacro` first).
   XML error or root not `<robot>`: `URDF_BAD_XML`.
2. `<transmission>` and `<gazebo>` are skipped. One `info` per kind.
3. Links: duplicate name is fatal (`URDF_DUPLICATE_NAME`).
4. Joints (checked in file order):
   - duplicate name: fatal.
   - `floating` or `planar`: fatal `URDF_UNSUPPORTED_JOINT`.
   - other type not in revolute, continuous, prismatic, fixed: fatal
     `URDF_UNKNOWN_JOINT_TYPE`.
   - parent or child link missing: `URDF_MISSING_LINK`.
   - child used by two joints: `URDF_MULTIPLE_PARENTS`.
   - non-fixed joint with axis length < 1e-9: `URDF_ZERO_AXIS`.
   - Missing `<axis>` defaults to (1, 0, 0).
5. Tree: the root is the one link that is no joint's child. No root:
   `URDF_CYCLE`. More than one root: `URDF_MULTIPLE_ROOTS`. Links not reached
   from the root by BFS: `URDF_CYCLE`.

## Canonical order

Links are numbered by BFS from the root. Children are visited in the file
order of their joints. Joints are then sorted by child index, so
`joints[i].child_link == i + 1` for every i. `parent_link` is always smaller
than `child_link`.

## tcp_link

`tcp_link` = deepest link (depth from root). Ties go to the lowest index.
Info `URDF_TCP_CHOSEN` names the link. The user can change it later (R3.4).

## Joints

- All imported joints use `kinematics = OriginAxis`. Origin comes from
  `<origin>`, axis is normalised.
- `fixed` -> `JointType.Fixed`. `prismatic` -> Prismatic. `revolute` and
  `continuous` -> Revolute.
- Initial `q` = 0 clamped into `[lower, upper]`.

### Limits

- `continuous`: mapped to [-pi, pi], warning `URDF_CONTINUOUS_LIMITS`.
- No `<limit>`: type defaults (revolute [-pi, pi], prismatic [0, 0.5]),
  warning `URDF_MISSING_LIMIT`.
- `<limit>` present but `lower` or `upper` attribute missing: value 0.0 is
  used (code in `_joint_limits`, no warning). Check the URDF for this case.
- Missing `velocity` or `effort`: type default and `URDF_MISSING_LIMIT`.

### Mimic

`<mimic joint multiplier offset>` is stored as `Joint.mimic`. Target must be
a joint name, else fatal `URDF_MIMIC_UNKNOWN`. Defaults: multiplier 1,
offset 0. Chain and fixed-target checks happen in `compile()` (see
`docs/compiled-model.md`).

## Inertial

`_read_inertial` sets `Link.inertia`:

- No `<inertial>`: mass stays 0. Root gets info `URDF_ROOT_NO_INERTIAL`,
  other links warning `URDF_NO_INERTIAL`.
- `mass` from `<mass value>`. `com` = `<inertial><origin xyz>`.
- Tensor from `<inertia>` is rotated into the link frame:
  `T = R * I * R^T`, `R` from the inertial `rpy`. Stored as
  `ixx, ixy, ixz, iyy, iyz, izz`.

## Validation and physical check

- `model.validate()` errors: each becomes `URDF_INVALID_MODEL` (error).
  `model` is `None`, visuals and collisions are empty.
- `model.validate_physical()` messages: warning `URM_PHYSICAL`. Model is kept.
  Rules in `docs/physical-properties.md`.

## Visuals and collisions

Run only after validation passes (`_parse_geometry`).

- `<visual>`: first one per link with a geometry is kept. Extra ones get info
  `URDF_EXTRA_VISUAL` and are skipped.
- `<collision>`: every one with a geometry is kept. No extra-visual check.
- Shape kinds, checked in order: `box` (size), `cylinder` (radius, length),
  `sphere` (radius), `mesh` (filename, scale). A geometry with none of these,
  or a mesh with no filename, is skipped.
- Colour: `<material><color rgba>`, or a top-level named material. Default
  (0.7, 0.7, 0.7, 1.0).
- Each `LinkVisual` has: `link_index`, `link_name`, `kind`, `params`,
  `origin_xyz`, `origin_rpy` (in link frame), `rgba`, `mesh_path`
  (absolute, or None), `fallback` (bool). `to_dict` and `from_dict` round-trip.
  `visual_origin_iso(v)` gives the 4x4 origin.

## Mesh resolution (`resolve_mesh`)

Order, first existing file wins (only regular files count):

1. `package://pkg/rel`: for each search root, `root/pkg/rel` then `root/rel`.
   Then, from the URDF folder upward, each ancestor `anc`: `anc/rel` when
   `anc` is named `pkg`, and `anc/pkg/rel`.
2. `file://`: URL path converted to a local path.
3. Absolute path: used as is.
4. Relative path: relative to the URDF folder.

Not found: `fallback = True`. For visuals, warning `URDF_MESH_MISSING`.
For collisions the fallback is silent (no report).

## Supported formats

`.stl`, `.obj`, `.ply` (case-insensitive). Read with PyVista `pv.read`. Other
suffix: `fallback = True`, warning `URDF_MESH_FORMAT`. Read exception: same
code, fallback set. Visuals only report; collisions are silent.

## Too large and fallback cube

- More than `MAX_TRIANGLES = 2_000_000` triangles: warning
  `URDF_MESH_TOO_LARGE` (visuals only). `fallback` stays False. At draw time
  `load_visual_mesh` returns `None`, so the mesh is not drawn. Estimate raises
  "mesh too large".
- Fallback cube: `fallback_cube(edge)` = centred cube with edge =
  `triad_length` (default 0.05 m). Drawn for missing, unreadable, unsupported
  or `fallback` meshes. Also used for unknown kinds.
- Mesh scale is applied at load time (`load_visual_mesh`), not at import.

## Project archive (`serializer.py`)

`PROJECT_FORMAT_VERSION = "1.1.0"` (`app/project/project_data.py`). Loader
accepts any 1.x. Files from 1.0.0 have no `visuals.json`: visuals and
collisions load as empty lists.

Archive members added in 1.1.0:

- `visuals.json`: `{"visuals": [...], "collisions": [...]}` (always written,
  may be empty). Items are `LinkVisual.to_dict()`.
- `assets/<name>`: each distinct existing, non-fallback mesh file, copied
  once. Same basename from another folder gets `stem_1.ext`, `stem_2.ext`.
  The item's `mesh_path` becomes `assets/<name>`. Fallback items keep their
  original `mesh_path` unchanged.

On load:

- All `assets/*` members are extracted into a new `tempfile.mkdtemp`
  folder (prefix `robotproj_assets_`). Names are taken with `PurePosixPath`
  name only (no zip-slip).
- `mesh_path` is rewritten to the temp file. A missing asset logs a warning,
  sets `mesh_path = None` and `fallback = True`.
- Temp folder is held in `ProjectData.asset_dir`, and removed with
  `shutil.rmtree` when the project data releases it
  (`app/project/project_data.py:149-152`).

Issue codes and their meaning: `docs/import-report.md`.
