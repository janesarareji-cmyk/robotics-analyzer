# Stage R3 spec pack: Import

Authority: `docs/MASTER_PLAN.md` (workflow, gate, push). Stage review is
BLOCKING (do not start R4 before `stage-approved`).

Order: R3.1 (spec in `PROGRESS.md`) → R3.2 → R3.3 → R3.4 → R3.5 → R3.6 →
R3.7. R3.8 (CAD) is NOT in this pack: Reviewer re-plans it after R3.7
(needs the Windows proof build). Stage R3 gate runs after R3.7; R3.8 may
follow as its own mini-stage `R3b` with its own review.

Common rules for every substage:
-   Builder starts with `graphify update .` and `graphify query` for the
    area. Reuse existing classes. No duplicate files.
-   Tests first-class: every numbered rule below has at least one test.
-   After each substage: `.\run_tests.ps1` green, `graphify update .`,
    commit `R3.<y>: <summary>`. Update the substage row in `PROGRESS.md`
    to `committed`. Do not wait for review between substages.
-   Old tests must stay green. Saved v1 projects and v1 URM JSON must open.
-   Python is the only place for UI and importers. C++ only for URM and
    kinematics.

---------------------------------------------------------------------

## R3.2 --- Compiled kinematic model + tree FK (C++ + bindings)

Goal: one compiled form for all joints. FK works for trees. Results for
existing DH robots stay identical.

Files: `core/kin/compiled_model.hpp/.cpp` (new), edit `core/kin/fk.hpp/.cpp`,
`core/CMakeLists.txt`, `bindings/kin_bindings.cpp`, tests.

1.  `struct CompiledJoint { int parent_link; int child_link; JointType type;
    Eigen::Isometry3d T_origin; Eigen::Vector3d axis; int q_index;
    int mimic_master; double mimic_mult; double mimic_off; }`
    (`mimic_master = -1` when not mimic; `q_index = -1` for fixed and mimic).
    `struct CompiledModel { std::vector<CompiledJoint> joints; int n_links;
    int n_dof; int tcp_link; Eigen::Isometry3d base; Eigen::Isometry3d tool; }`.
    Joints are in the URM joint order (already topological because
    parent_link < child_link, R3.1).
2.  `CompiledModel compile(const urm::RobotModel&)`. Does not call
    `validate()`. Throws `std::invalid_argument` for indices out of range.
    `tcp_link == -1` → last link.
3.  Compile rules:
    -   Every joint compiles to `A = pre · motion(axis, q_eff) · post`.
        `CompiledJoint` therefore has fields `pre`, `post` (Isometry3d) and
        `axis` (no separate `T_origin`; replace it in item 1). Document this
        in `compiled_model.hpp`.
    -   `kinematics == OriginAxis`: `pre = pose_to_iso(joint.origin)`,
        `post = I`, `axis` = normalised `joint.axis`.
    -   `kinematics == DH` (axis = z). With θ0, d0 = the stored theta, d:
        Standard revolute:  pre = I, motion = Rz(θ0+q), post = Tz(d)·Tx(a)·Rx(α).
        Standard prismatic: pre = Rz(θ0), motion = Tz(d0+q), post = Tx(a)·Rx(α).
        Modified revolute:  pre = Rx(α)·Tx(a), motion = Rz(θ0+q), post = Tz(d).
        Modified prismatic: pre = Rx(α)·Tx(a)·Rz(θ0), motion = Tz(d0+q), post = I.
        Fixed (both): pre = full A with q = 0, motion = identity, post = I.
        The product must equal `dh_transform` / `mdh_transform` from R2.5.
    -   motion for revolute = `AngleAxis(q_eff, axis)`, prismatic =
        `Translation(axis * q_eff)`, fixed = identity.
    -   `q_eff = q` for normal joints; for mimic joints
        `q_eff = mult·q_master_eff + offset` (master must be earlier in
        joint order; if not → `std::invalid_argument`).
4.  `q_index` comes from `RobotModel::q_index_map()` (R3.1). Do not
    recompute it differently.
5.  `FKResult forward_kinematics(const CompiledModel&, const std::vector<double>& q)`:
    `q.size() == n_dof` else `std::invalid_argument`.
    `frames.size() == n_links`, `frames[0] = base`,
    `frames[child] = frames[parent] · pre · motion · post`.
    `tcp = frames[tcp_link] · tool`.
    NOTE: for serial DH robots `frames` is the same list as in R2.5
    (frames[i+1] after joint i). Existing `forward_kinematics(RobotModel)`
    and the `q` overload now call `compile()` then the new function.
    `FKResult` keeps `frames` and `tcp`.
6.  `std::vector<int> active_chain(const urm::RobotModel&)`: joint indices
    on the path from link 0 to `tcp_link`, in order. Bind to Python.
    Python `fk()` result dict gains `"active_joints"` (list of joint
    indices) and keeps `"frames"`, `"tcp"`.
7.  The `fk(model, q)` overload takes `q` of size `dof()` (non-fixed,
    non-mimic). Wrong size → `ValueError` in Python.
8.  No allocation beyond the result vectors during FK. `compile()` may
    allocate. Document that callers compile once and reuse for many q
    (R5 relies on this).

Tests:
-   All R2.5 C++ and Python FK tests pass unchanged (tol 1e-12).
-   DH vs MDH planar equivalence still passes.
-   Random-chain test: 200 random DH robots (3 to 7 joints, random a,
    alpha, d, theta, mixed revolute/prismatic/fixed, both conventions,
    seed 42), 5 random q each: compiled FK equals a test-local direct
    multiplication of A matrices built from the textbook formulas
    (written in the test, not calling `dh_transform`). Tol 1e-12.
-   Origin-axis revolute about (0,0,1) with origin xyz=(1,0,0): q=π/2 →
    TCP (1,0,0) rotated: frame rot = Rz(π/2). Origin-axis revolute about
    an off-axis unit vector vs `Eigen::AngleAxis` by hand. Prismatic along
    (0,1,0).
-   Tree: links 0,1,2,3; joints j1 (0→1), j2 (0→2), j3 (1→3), all
    origin_axis revolute z, origins (1,0,0),(0,1,0),(0.5,0,0). q=(π/2,0,0)
    → frames[3] translation by hand: (1,0,0)+Rz(π/2)(0.5,0,0) = (1,0.5,0).
    `tcp_link=3` and `tcp_link=2` give different TCP. `active_chain` for
    tcp_link=3 is [0,2]; for tcp_link=2 is [1].
-   Mimic: master q=0.4, mult=−1, offset=0.1 → slave q_eff=−0.3; `dof()`
    excludes slave; q vector length matches.
-   Fixed joint in tree ignores q. Wrong-size q throws. `compile()` throws
    on bad index. Compiled FK does not modify the model.
-   Python: `fk` returns `active_joints`; wrong q size `ValueError`;
    `active_chain` callable.

Done when: tests green, graphify, commit `R3.2: compiled model + tree FK`.

---------------------------------------------------------------------

## R3.3 --- URDF parser (Python)

Goal: URDF file → URM `RobotModel` with a report. No meshes (R3.5), no UI
(R3.4), no inertials (R3.7).

Files: `app/importers/__init__.py`, `app/importers/urdf.py`,
`app/importers/import_report.py` (shared report type, used by R3.6),
`tests/python/test_urdf_import.py`, fixtures under
`tests/python/fixtures/urdf/`.

1.  `ImportIssue(severity, code, message, element=None)` where severity is
    `"error" | "warning" | "info"`. `ImportReport` holds a list of issues,
    `has_errors()`, `by_severity()`, `to_text()`. Stable string `code`s
    (e.g. `URDF_NO_ROOT`, `URDF_CYCLE`, `URDF_FLOATING_JOINT`,
    `URDF_XACRO`, `URDF_BAD_XML`, `URDF_UNKNOWN_JOINT_TYPE`,
    `URDF_MISSING_LINK`, `URDF_MULTIPLE_ROOTS`, `URDF_CONTINUOUS_LIMITS`).
2.  `def import_urdf(path) -> tuple[RobotModel | None, ImportReport]`. Never
    raises for bad input: errors go to the report and the model is `None`.
    Parsing: stdlib `xml.etree.ElementTree` (no new dependency). Reject
    files containing `xacro:` namespace or `${` with `URDF_XACRO` ("xacro
    not supported, convert with `xacro` first").
3.  Supported joint types: `revolute`, `continuous` (limits ±π, warning
    `URDF_CONTINUOUS_LIMITS`, treated as revolute), `prismatic`, `fixed`.
    `floating` and `planar` → error. Unknown type → error.
4.  Per joint: `<origin xyz rpy>` (default zeros) → `Joint.origin`;
    `<axis xyz>` (default 1 0 0 per URDF spec) normalised →
    `Joint.axis`; zero-length axis → error. `kinematics = OriginAxis`.
    `<limit lower upper velocity effort>` → `JointLimits` (revolute /
    prismatic: all four required by URDF; missing → defaults from
    `default_limits(type)` + warning). `<mimic joint multiplier offset>`
    (defaults 1, 0) → `Joint.mimic`. Joint name preserved. DH fields left
    default and unused.
5.  Links → `Link` (name only for now). Build tree: link names → indices.
    Root = the one link that is no joint's child; zero roots → `URDF_CYCLE`
    error; more than one → `URDF_MULTIPLE_ROOTS` error. Missing parent or
    child link → `URDF_MISSING_LINK`. Re-index links in a topological order
    with root = 0 and `parent_link < child_link`; joints re-ordered to
    match (a joint's child index == its position + 1 only if the file is a
    plain chain; in general set `child_link` per joint and keep
    `links.size() == joints.size()+1`). A link that is the child of two
    joints → error (URDF trees only).
6.  `tcp_link`: the end of the longest root-to-leaf path (ties: first in
    file order). Add an `info` issue telling the user which link was
    chosen and that it can be changed.
7.  `convention` stays Standard (unused for origin_axis joints).
    Metadata name = `<robot name>`.
8.  Result model must pass `validate()`; if not, add the messages as
    errors and return `None`.
9.  `<transmission>`, `<gazebo>`, `<material>`, `<visual>`, `<collision>`,
    `<inertial>`: ignored without error in R3.3 (info issue
    `URDF_IGNORED_ELEMENT` once per element kind; visual/inertial are
    handled in R3.5 and R3.7).

Fixtures (small hand-written URDF files): 2R planar serial; 6-DOF serial;
gripper with two prismatic fingers + mimic (tree); continuous joint;
missing limit; xacro file; bad XML; two roots; cycle; floating joint;
missing parent link; zero axis; unknown joint type.

Tests: for each fixture the report codes and error/warning counts; model
structure (joint count, `dof()`, `q_index_map()`, `parent_link`,
`child_link`, `tcp_link`, mimic fields); FK of the 2R planar import equals
the hand value (a1=a2=1, q=(π/2,−π/2) → (1,1,0)); FK of the gripper tree
for both finger links; `import_urdf` on a nonexistent path → report error,
model `None`; imported model round-trips through `to_json/from_json`.

Done when: tests green, graphify, commit `R3.3: URDF parser`.

---------------------------------------------------------------------

## R3.4 --- UI for tree and origin_axis joints (Python UI)

Goal: import from the menu, see the tree, edit safely. Reuse `MainWindow`,
`ProjectController`, `DHEditorWidget`, `JointPanel`, `RobotView`.

1.  File > Import > URDF… (file dialog, `.urdf`). Runs `import_urdf` in
    `TaskRunner`. On success: new project (or replace the current robot
    after the usual discard-changes confirmation) with the model; on
    errors: show the report (R3.6 dock will use the same report; here a
    message box with `report.to_text()` is enough) and keep the old state.
2.  `app/panels/robot_tree.py`: `RobotTreePanel` dock "Robot Tree": QTreeWidget
    with links as nodes and joints as the edges (child item under the
    parent link, label `joint_name [type]`), active-chain items bold,
    other branches grey. Selecting an item selects the joint/link
    (signal `selection_changed(kind, index)`). Read-only in R3.4.
3.  `DHEditorWidget`: rows of `origin_axis` joints are read-only (cells not
    editable, row tinted) with tooltip "Imported joint: not DH". Type
    change, add/remove joint on such rows is refused with a status
    message. Adding a new DH joint to a robot whose joints are not a plain
    chain is refused (message: "Tree robots are not editable here").
    Edits to the robot-level convention only affect DH rows.
4.  `JointPanel`: rows = `active_chain` non-fixed, non-mimic joints
    (from `fk()["active_joints"]`). Collapsed group "Other branches"
    lists the remaining non-fixed, non-mimic joints with the same
    sliders (they move the model, TCP readout does not change). Panel
    rebuilds when `tcp_link`, topology or limits change.
5.  `RobotView`: draw ALL link frames from `frames` (not assuming a chain).
    The polyline is drawn per joint from the parent link origin to the
    child link origin (one segment per joint), not as one chain line.
    Active-chain segments blue; other branches dim grey. TCP triad at
    `tcp_link` + tool. Keep in-place updates; rebuild on topology change.
6.  Properties: View menu entry for the tree dock; selecting a joint in the
    tree highlights its marker in the viewport (marker enlarged).
7.  Menu action "Set TCP link…" (dialog listing links) writes
    `model.tcp_link`, marks dirty, updates everything.
8.  R2.9 open item: add assertions for DH table cells after save/close/
    reopen in the R2 milestone test.

Tests `tests/python/test_ui_tree.py` (offscreen): import 6-DOF fixture →
tree has 7 link items; active chain bold; import gripper fixture → branch
grey, polyline has one segment per joint, TCP link change moves the TCP
triad; DH editor refuses edit on origin_axis row (model unchanged);
JointPanel row counts for chain and branch group; import with errors keeps
the old project; Set TCP link; dock toggle in View menu; no regression:
R2 milestone test green.

Done when: tests green, graphify, commit `R3.4: tree UI`.

---------------------------------------------------------------------

## R3.5 --- Meshes and assets (Python)

Goal: URDF visual geometry shown in the viewport; project archive keeps
the mesh files.

1.  Extend `import_urdf`: parse `<link><visual>` elements (first visual
    per link is enough; others → info). Geometry: `box size`, `cylinder
    radius length`, `sphere radius`, `mesh filename scale`. `<origin>` of
    the visual = pose inside the link frame. Colour from `<material>`
    (rgba) if present, else default grey. Store in a new Python-side
    `LinkVisual(kind, params, origin_xyz, origin_rpy, rgba, mesh_path)`
    list carried by an `ImportResult` (the C++ URM does not store visuals in
    R3). Visuals are saved in the project as `visuals.json` +
    `assets/` inside the archive (serializer change; project format
    version bumped and old projects without it open fine, visuals empty).
2.  Mesh path resolution `resolve_mesh(filename, urdf_dir, search_roots)`:
    `package://<pkg>/<rel>` → try each search root (`<root>/<pkg>/<rel>`,
    `<root>/<rel>`), then walk up from the URDF directory looking for a
    folder named `<pkg>`; `file://` and absolute paths used as is;
    relative paths relative to the URDF directory. Search roots come from
    `AppSettings` (`import/mesh_search_roots`, list of paths, editable in
    a small dialog under File > Import > Mesh search paths…).
3.  Loading: PyVista reads `.stl`, `.obj`, `.ply`. `.dae` and others →
    warning `URDF_MESH_FORMAT` and a fallback (small box at the link
    origin, flagged in the report). Missing file → warning
    `URDF_MESH_MISSING`, fallback box, import still succeeds. Mesh
    `scale` applied. Reject meshes over 2,000,000 triangles with a
    warning and decimation note (do not decimate in R3.5, skip draw).
4.  `RobotView`: visual actors per link attached with `user_matrix =
    frames[link] · visual_origin`; updated in place on q change (the same
    mechanism as triads). Toggle View > Robot > Show Visuals (default on)
    and Show Frames independent of each other. Mesh actors cached per
    project (load once).
5.  Collision geometry: parsed and stored in `visuals.json` under
    `collisions` but NOT drawn (used in R7). Keep the schema minimal.
6.  Archive: on save, mesh files copied into `assets/` (relative names
    unique, collisions resolved by suffix), `visuals.json` refers to the
    archive-relative names. On open, meshes are read from the archive
    (extract to a temp cache dir per session). A saved project opens on a
    machine without the original mesh files.

Fixtures: STL cube (ASCII, tiny), OBJ, a `.dae` file, a URDF with
`package://` mesh and a `catkin`-style tree under a temp dir, URDF with
primitives only.

Tests `tests/python/test_urdf_meshes.py`: resolve rules (each form);
missing mesh → warning, fallback; dae → warning; primitives actor count;
visual actors move with q (compare `user_matrix` with `fk()` frames ·
origin, tol 1e-9); toggles; save → delete original mesh files → open →
meshes still shown; old project file without `visuals.json` opens.

Done when: tests green, graphify, commit `R3.5: meshes and assets`.

---------------------------------------------------------------------

## R3.6 --- Import validation and error report (Python UI)

Goal: one clear report for every import, clickable.

1.  `ImportReportPanel` dock "Import Report" (View menu toggle). Table:
    severity icon, code, message, element. Filter checkboxes for
    error/warning/info. Row click selects the joint/link in the tree
    (uses `selection_changed` from R3.4) when `element` names one.
2.  The report is shown after every import (URDF now; CAD later), never
    hidden when it has errors or warnings; info-only reports show only in
    the dock (no popup).
3.  `ImportReport.to_text()` format fixed:
    `[SEVERITY] CODE: message (element)` one per line + summary line
    `N errors, M warnings, K info`. "Save report…" writes it to a file.
4.  Severity policy (document in `docs/import-report.md`): error = model
    unusable (no model loaded); warning = model loaded but something was
    approximated (mesh missing, continuous joint, ignored element,
    unsupported format); info = a choice was made (TCP link, tcp, ignored
    element kind). A table of all codes with severity and meaning.
5.  After a successful import run `model.validate()` again and add each
    message as a warning `URM_VALIDATE` (model still loaded, user can fix).
6.  Log panel gets one line per import: path, counts of each severity.
7.  Replace the R3.4 message box by the dock (errors still raise the dock
    to front).

Tests `tests/python/test_import_report.py`: dock row counts per fixture;
filters; row click selects tree item; `to_text()` golden file compare;
save report file; warnings keep the model loaded; errors keep the old
project; `docs/import-report.md` code table equals the set of codes used
in `app/importers/` (test greps the code constants).

Done when: tests green, graphify, commit `R3.6: import report`.

---------------------------------------------------------------------

## R3.7 --- Physical properties (C++ URM + Python)

Goal: mass, centre of mass, inertia per link: import, estimate, edit.
Needed for R4 dynamics.

1.  C++ `urm::Inertial { double mass = 0; std::array<double,3> com{};
    std::array<double,6> inertia{}; }` with inertia order
    (ixx, ixy, ixz, iyy, iyz, izz) about the COM, in the link frame
    (kg·m²). `Link.inertial` (already `Link` has `Inertia`? Builder checks
    `core/urm/link.hpp`; reuse it if present, else add). JSON key
    `"inertial"` on each link, optional. Missing → mass 0 (means "not
    set"). Schema: optional keys, `schema_version` stays 2, old files open.
    Bind to Python (whole-object assignment rule).
2.  `validate()` new rules (pass+fail test each): mass ≥ 0; inertia finite;
    if mass > 0: inertia matrix positive definite and principal moments
    satisfy the triangle inequality (Ia + Ib ≥ Ic, tol 1e-9); COM finite.
    Violations are returned as messages (not exceptions).
3.  URDF import: parse `<inertial><origin xyz rpy/><mass value/><inertia
    ixx ixy ixz iyy iyz izz/></inertial>`. If the inertial origin has a
    non-zero rpy, rotate the tensor into the link frame:
    `I_link = R · I · Rᵀ`. Missing inertial → warning
    `URDF_NO_INERTIAL` per link (not for the root if mass is 0 and it is
    flagged as fixed base: info instead). Mass ≤ 0 on a moving link →
    warning.
4.  Estimate tool `estimate_inertial(mesh, density=None, total_mass=None)`
    in `app/physics/estimate.py`: uniform density, volume and COM and
    inertia from a closed triangle mesh (numpy, divergence-theorem
    method, see Eberly "Polyhedral mass properties"); mesh not watertight
    → warning and use the convex hull. Exactly one of `density` or
    `total_mass` is given. For URDF primitives use the analytic formulas
    (box, cylinder, sphere).
5.  Panel `PhysicalPropertiesPanel` dock "Physical": table, one row per
    link: mass, COM x y z, six inertia terms (editable, SI units,
    scientific notation allowed), a status icon from `validate()`. Buttons:
    "Estimate from visual" (selected link or all links, asks density or
    total mass), "Scale all masses to total…". Shows total mass and total
    COM (world, at current q). Edits mark the project dirty and are
    saved in the URM JSON.
6.  Gravity vector setting in project `simulation_settings` already
    exists or is added: default (0, 0, −9.81). Only stored in R3.

Tests: C++ `test_urm.cpp`: inertial round-trip, missing → mass 0, each new
validate rule pass/fail, old v2 JSON opens. Python: URDF inertial import
with rotated origin (compare the rotated tensor to numpy `R·I·Rᵀ`, tol
1e-12); box estimate vs analytic (tol 1e-9 for exact mesh, since the cube
mesh is exact), cylinder mesh with 256 segments vs analytic within 0.5 %,
sphere mesh (icosphere subdiv 4) within 1 %; total_mass scaling; open
mesh falls back with a warning; panel edit changes the model and dirty
flag; total COM matches hand value for a two-link fixture; save/load
round-trip.

Done when: tests green, graphify, commit `R3.7: physical properties`.

---------------------------------------------------------------------

## R3 stage milestone and gate (Builder does this after R3.7)

`tests/python/test_milestone_r3.py` (offscreen MainWindow, real
`core_bridge`):
1.  Import `tests/python/fixtures/urdf/six_dof_with_gripper.urdf`
    (6 revolute chain + 2 prismatic finger branch + mimic, STL visuals,
    full inertials). Report has no errors.
2.  Tree dock shows all links; active chain is the 6 arm joints; fingers
    drawn dim.
3.  Move joints with the JointPanel; TCP readout equals an independent
    numpy FK (origin+axis math written in the test).
4.  Mimic finger follows the master finger.
5.  Total mass and COM shown; equal to the numpy sum from the fixture.
6.  Save, close, open: robot, visuals (meshes from the archive), physical
    data, tcp_link all identical.
7.  Import the broken fixtures: errors shown, old project unchanged.

Stage gate = `docs/MASTER_PLAN.md` section 1.3. Then:
-   Set stage status `stage-ready-for-review` in `PROGRESS.md`.
-   Push `main` and tag `stage-R3-done` (when a remote exists).
-   STOP. R3 is a blocking-review stage. Wait for Reviewer.
