# Stage R3 spec pack: Import

Authority: `docs/MASTER_PLAN.md` (workflow, gate, push). Stage review is
BLOCKING (do not start R4 before `stage-approved`).

Order: R3.1 (DONE, commit `fadcca7`, spec archived in
`docs/specs/R3.1-spec.md`) → R3.2 → R3.3 → R3.4 → R3.5 → R3.6 → R3.7.
R3.8 (CAD) is NOT in this pack: Reviewer re-plans it after R3.7 (needs the
Windows proof build). Stage R3 gate runs after R3.7; R3.8 follows as its
own mini-stage `R3b` with its own spec and review.

Start point for a new Builder session: the first row of R3.2..R3.7 in
`PROGRESS.md` whose status is not `committed`.

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
-   C++ change → rebuild before tests: `cmake --build build --config
    Release` (copies `core_bridge*.pyd` to the repo root), then
    `.\run_tests.ps1`.
-   Spec unclear or contradicts the code → do NOT guess. Write the
    question in `PROGRESS.md` → "Blockers", stop, tell the user.

Code facts (checked 2026-11-11, R3.1 code). Use these names:
-   Python module: `import core_bridge as cb`. URM JSON: `cb.urm_to_json(model)`,
    `cb.urm_from_json(s)`. FK: `cb.fk(model)`, `cb.fk(model, q)`,
    `cb.pose_to_iso(pose)`. `model.q_index_map()`, `model.validate()`.
-   Binding rule: `model.joints` / `model.links` are opaque vectors, so
    `model.joints[i].q = x` works in place. But `std::array` and
    `std::optional` fields (`Pose.xyz`, `Pose.rpy`, `Joint.axis`,
    `Joint.mimic`, `Inertia.com`, `Inertia.inertia_tensor`) are COPIES:
    assign the whole list / object (`j.axis = [0,0,1]`), never element-wise.
-   Link inertial data already exists: `urm::Inertia { mass, com,
    inertia_tensor }` on `Link::inertia`, JSON key `"inertia"` (R3.7 uses
    it, does not add a new struct).
-   Project file: zip `.robotproj` with `project_metadata.json`,
    `robot_model.json`, `physical_properties.json` (unused legacy dict),
    `simulation_settings.json` (already has `gravity`,
    default [0,0,−9.81]). `PROJECT_FORMAT_VERSION = "1.0.0"`
    (`app/project/project_data.py`); loader accepts same major only.
-   UI: `MainWindow` docks `model_dock` ("Robot Model", `ModelPanel`,
    placeholder QTreeWidget), `props_dock` ("Properties",
    `PropertiesPanel`, placeholder), `dh_dock`, `joint_dock`, `log_dock`.
    View menu has one checkable action per dock + "Robot" submenu
    (Show Frames / Links / TCP). `ProjectController` signals:
    `project_opened`, `project_saved`, `project_closed`,
    `modified_changed`, `robot_changed`. `JointPanel.q_changed` →
    `MainWindow._on_joint_q_changed`. Discard prompt:
    `ProjectController._confirm_discard(parent)` (private).
-   `TaskRunner.submit_fn(fn) -> FunctionTask`; connect
    `task.signals.result / error / finished`. Tests: `runner.wait_all(5000)`
    then `QApplication.processEvents()`.
-   `AppSettings.get(key, default)` / `set(key, value)` (QSettings).
-   Versions: Python 3.13, PySide6 6.12, PyVista 0.49 (`pv.Icosphere`
    exists), NumPy 2.5.

---------------------------------------------------------------------

## R3.2 --- Compiled kinematic model + tree FK (C++ + bindings)

Goal: one compiled form for all joints. FK works for trees. Results for
existing DH robots stay identical.

Files: `core/kin/compiled_model.hpp/.cpp` (new), edit `core/kin/fk.hpp/.cpp`,
`core/CMakeLists.txt`, `bindings/kin_bindings.cpp`, tests.

1.  Types (namespace `kin`, file `compiled_model.hpp`):
    ```cpp
    struct CompiledJoint {
        int joint_index;          // index into RobotModel::joints
        int parent_link;
        int child_link;
        urm::JointType type;
        Eigen::Isometry3d pre;    // A = pre · motion(axis, q_eff) · post
        Eigen::Isometry3d post;
        Eigen::Vector3d axis;     // unit vector, in the frame after `pre`
        int q_index;              // -1 for fixed and mimic joints
        int src_q_index;          // dof index that drives this joint (-1 for fixed)
        double mult;              // q_eff = mult * q[src_q_index] + offset
        double offset;            // normal joint: mult = 1, offset = 0
    };
    struct CompiledModel {
        std::vector<CompiledJoint> joints;   // sorted by child_link ascending
        int n_links; int n_dof; int tcp_link; // tcp_link resolved (never -1)
        Eigen::Isometry3d base; Eigen::Isometry3d tool;
    };
    ```
    Document the `A = pre · motion · post` rule in `compiled_model.hpp`.
    ORDER: compiled joints are sorted by `child_link` ascending. Reason:
    URM joint order is NOT guaranteed topological (example: joint0 2→3,
    joint1 0→2 is legal). Because `parent_link < child_link` and every
    child is unique, sorting by `child_link` guarantees `frames[parent]`
    exists before it is used. `joint_index` maps back to the URM joint.
2.  `CompiledModel compile(const urm::RobotModel&)`. Does not call
    `validate()`. Throws `std::invalid_argument` when: a link index is out
    of range; `parent_link >= child_link`; a mimic target name is missing
    or the mimic chain has a cycle; a mimic chain ends on a fixed joint.
    `tcp_link == -1` → `links.size() - 1`.
3.  Compile rules:
    -   Every joint compiles to `A = pre · motion(axis, q_eff) · post`.
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
    -   Mimic is flattened at compile time, so FK needs no ordering and no
        extra buffer. Follow the mimic chain to its root master M (a
        non-mimic joint). Compose: `q_eff = m2·(m1·q_M + o1) + o2 =
        (m1·m2)·q_M + (m2·o1 + o2)`. Store `src_q_index = q_index of M`,
        `mult`, `offset`. Normal joint: `src_q_index = q_index`, mult 1,
        offset 0. Fixed joint: `src_q_index = -1`, q ignored.
    -   `q_eff` for revolute/prismatic: `mult * q[src_q_index] + offset`.
4.  `q_index` comes from `RobotModel::q_index_map()` (R3.1). Do not
    recompute it differently.
5.  `FKResult forward_kinematics(const CompiledModel&, const std::vector<double>& q)`:
    `q.size() == n_dof` else `std::invalid_argument`.
    `frames.size() == n_links`, `frames[0] = base`,
    loop over compiled joints (child_link order):
    `frames[child] = frames[parent] · pre · motion · post`.
    `tcp = frames[tcp_link] · tool`.
    NOTE: for serial DH robots (joint i: parent i, child i+1) `frames` is
    the same list as in R2.5. Existing `forward_kinematics(RobotModel)`
    and the `q` overload now call `compile()` then the new function.
    `FKResult` keeps `frames` and `tcp`. Update the `FKResult` doc comment
    in `fk.hpp`: `frames.size() == links.size()`, `frames[k]` = pose of
    link k; `tcp = frames[tcp_link] · tool`.
6.  `std::vector<int> active_chain(const urm::RobotModel&)`: URM joint
    indices on the path from link 0 to the resolved `tcp_link`, ordered
    root → tip (includes fixed and mimic joints on the path). Bind to
    Python as `core_bridge.active_chain(model)`. Python `fk()` result dict
    gains `"active_joints"` (same list) and keeps `"frames"`, `"tcp"`.
    Also bind `core_bridge.compile(model)` → opaque `CompiledModel`
    object and `core_bridge.fk_compiled(cm, q)` → same dict (for R4/R5
    reuse; Python tests only check it equals `fk(model, q)`).
7.  The `fk(model, q)` overload takes `q` of size `dof()` (non-fixed,
    non-mimic). Wrong size → `ValueError` in Python. `compile()` errors
    (`std::invalid_argument`) → `ValueError` in Python.
7a. `validate()` new rule (C++, pass + fail test): a mimic target must not
    be a fixed joint (message names both joints). Same check as the
    compile error in item 2, so a validated model always compiles.
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
-   Origin-axis revolute, origin xyz=(1,0,0), axis (0,0,1), tool
    xyz=(1,0,0), q=π/2 → frames[1] translation (1,0,0), rotation
    Rz(π/2); TCP translation (1,1,0). Origin-axis revolute about an
    off-axis unit vector (e.g. normalised (1,1,1)) vs a hand-built
    `Eigen::AngleAxisd`. Prismatic along (0,1,0), q=0.3 → child origin
    moves +0.3 in y.
-   Order: joint list given NOT sorted (joint0 = 2→3, joint1 = 0→2,
    joint2 = 0→1) → FK equals the same tree listed sorted.
-   Mimic chain: A (master), B mimics A (m=2, o=0.1), C mimics B (m=−1,
    o=0.2) → C q_eff = −2·q_A + 0.1 (hand value); mimic of a fixed joint
    → `compile()` throws and `validate()` reports it.
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

1.  `import_report.py` (dataclasses):
    `ImportIssue(severity, code, message, element=None)`; severity is
    `"error" | "warning" | "info"`; `element` = link or joint name or None.
    `ImportReport` holds `issues: list[ImportIssue]`, `add(...)`,
    `has_errors()`, `by_severity(sev)`, `to_text()`.
    `ImportResult(model, report, visuals=[], collisions=[])`: `model` is
    `RobotModel | None`; `visuals`/`collisions` stay empty until R3.5.
    Codes are module-level string constants in `import_report.py` (one
    place, so R3.6 can grep them). Full code list for R3.3:

    | Code | Severity | When |
    |------|----------|------|
    | `URDF_FILE_NOT_FOUND` | error | path missing / unreadable |
    | `URDF_BAD_XML` | error | XML parse error, or root tag not `<robot>` |
    | `URDF_XACRO` | error | xacro namespace or `${` found |
    | `URDF_UNSUPPORTED_JOINT` | error | `floating` or `planar` joint |
    | `URDF_UNKNOWN_JOINT_TYPE` | error | any other unknown type |
    | `URDF_MISSING_LINK` | error | joint parent/child names no `<link>` |
    | `URDF_DUPLICATE_NAME` | error | two links or two joints same name |
    | `URDF_MULTIPLE_PARENTS` | error | link is child of two joints |
    | `URDF_MULTIPLE_ROOTS` | error | more than one root link |
    | `URDF_CYCLE` | error | no root link (cycle) |
    | `URDF_ZERO_AXIS` | error | `<axis>` norm < 1e-9 |
    | `URDF_MIMIC_UNKNOWN` | error | mimic names a missing joint |
    | `URDF_INVALID_MODEL` | error | built model fails `validate()` |
    | `URDF_CONTINUOUS_LIMITS` | warning | continuous joint mapped to ±π |
    | `URDF_MISSING_LIMIT` | warning | `<limit>` or its velocity/effort missing |
    | `URDF_IGNORED_ELEMENT` | info | element kind skipped (once per kind) |
    | `URDF_TCP_CHOSEN` | info | which link became `tcp_link` |

    New codes in later substages go into the same file and the same table
    in `docs/import-report.md` (R3.6).
2.  `def import_urdf(path) -> ImportResult`. Never raises for bad input:
    errors go to the report and `model` is `None`. Parsing: stdlib
    `xml.etree.ElementTree` (no new dependency). Reject files containing
    an `xmlns:xacro` attribute, any `xacro:` tag, or `${` with
    `URDF_XACRO` ("xacro not supported, convert with `xacro` first").
    Build the model with `core_bridge` (`cb.RobotModel()`, `cb.Joint()`,
    `cb.Link()`, assign whole objects / whole lists, R2.4 rule).
    Units: URDF is SI (m, rad) like URM; no conversion.
    URDF `rpy` = fixed-axis XYZ, `R = Rz(y)·Ry(p)·Rx(r)` = the same as
    `urm.Pose`. Copy the numbers directly.
3.  Supported joint types: `revolute`, `continuous` (limits ±π, warning
    `URDF_CONTINUOUS_LIMITS`, treated as revolute), `prismatic`, `fixed`.
    `floating` and `planar` → error. Unknown type → error.
4.  Per joint: `<origin xyz rpy>` (default zeros) → `Joint.origin`;
    `<axis xyz>` (default 1 0 0 per URDF spec) normalised →
    `Joint.axis`; zero-length axis → `URDF_ZERO_AXIS`.
    `kinematics = OriginAxis`. Fixed joints: axis ignored (keep default).
    Limits (URDF spec: `<limit>` is required for revolute/prismatic;
    `lower`/`upper` default 0; `velocity`/`effort` required):
    -   `<limit>` present → lower/upper from attributes (default 0.0);
        missing `velocity` or `effort` → value from
        `default_limits(type)` + warning `URDF_MISSING_LIMIT`.
    -   `<limit>` absent → `default_limits(type)` + `URDF_MISSING_LIMIT`.
    -   continuous → lower −π, upper +π, vel/effort from `<limit>` if
        given, type Revolute, warning `URDF_CONTINUOUS_LIMITS`.
    -   fixed → `default_limits(Fixed)`.
    Initial `q`: 0 clamped into [lower, upper] (so `validate()` passes).
    `<mimic joint multiplier offset>` (defaults 1, 0) → `Joint.mimic`.
    Joint name preserved. DH fields left default and unused.
5.  Links → `Link` (name only; inertia left at C++ default until R3.7).
    Build tree: link names → indices. Root = the one link that is no
    joint's child; zero roots → `URDF_CYCLE`; more than one →
    `URDF_MULTIPLE_ROOTS`. Missing parent or child link →
    `URDF_MISSING_LINK`. Link child of two joints →
    `URDF_MULTIPLE_PARENTS`. A cycle not caught by the root rule (a loop
    beside a valid root) → `URDF_CYCLE` (detect with a BFS from the root:
    a link not reached = cycle/disconnected).
    CANONICAL ORDER (required): number links by BFS from the root (root =
    0; children of a link in file order of their joints). Then sort
    joints so that `joints[i].child_link == i + 1`. This always holds
    for a valid URDF tree (n links, n−1 joints, every non-root link has
    exactly one parent joint) and gives `parent_link < child_link`.
6.  `tcp_link`: the deepest link (most joints from root); ties → the
    lowest link index. Add `URDF_TCP_CHOSEN` info naming the link and
    saying it can be changed (R3.4 "Set TCP link…"). `tool` = identity.
7.  `convention` stays Standard (unused for origin_axis joints).
    `metadata.name` = `<robot name>` attribute.
8.  Result model must pass `validate()`; if not, add each message as
    `URDF_INVALID_MODEL` error and return `model = None`.
9.  `<transmission>`, `<gazebo>`, `<material>` (top level), `<visual>`,
    `<collision>`, `<inertial>`: skipped, info `URDF_IGNORED_ELEMENT`
    once per element kind. R3.5 stops skipping visual/collision/material,
    R3.7 stops skipping inertial.
10. Do NOT touch the UI, project files or `ProjectController` in R3.3.

Fixtures (small hand-written URDF files): 2R planar serial; 6-DOF serial;
gripper with two prismatic fingers + mimic (tree); continuous joint;
missing limit; xacro file; bad XML; two roots; cycle; floating joint;
missing parent link; zero axis; unknown joint type.

2R planar fixture (exact, so FK is checkable): links `base, l1, l2,
tip`; joints `j1` revolute base→l1 origin 0, axis z; `j2` revolute l1→l2
origin xyz (1,0,0), axis z; `j_tip` fixed l2→tip origin xyz (1,0,0).
Then `tcp_link` = tip (index 3), `dof()` = 2.

Tests `tests/python/test_urdf_import.py`: for each fixture the report
codes and error/warning counts; model structure (joint count, `dof()`,
`q_index_map()`, `parent_link`, `child_link`, canonical order
`child_link == i+1`, `tcp_link`, mimic fields); a fixture whose joints
are listed in reverse file order still gives canonical order; FK of the
2R planar import: q=(π/2,−π/2) → TCP xyz (1,1,0), tol 1e-12; FK of the
gripper tree for both finger links vs hand numbers; `import_urdf` on a
nonexistent path → `URDF_FILE_NOT_FOUND`, model `None`; imported model
round-trips through `cb.urm_to_json` / `cb.urm_from_json` (equal JSON).

Done when: tests green, graphify, commit `R3.3: URDF parser`.

---------------------------------------------------------------------

## R3.4 --- UI for tree and origin_axis joints (Python UI)

Goal: import from the menu, see the tree, edit safely. Reuse `MainWindow`,
`ProjectController`, `DHEditorWidget`, `JointPanel`, `RobotView`.

Definition used below: a robot is a **plain chain** when for every joint
i: `parent_link == i` and `child_link == i + 1`. All R2 DH robots are
plain chains.

0.  Bug fix first (R3.1 regression): `DHEditorWidget._on_remove_joint`
    removes joints but not links and leaves stale `parent_link` /
    `child_link`, so `validate()` fails. Fix: only on a plain chain;
    after removal renumber `parent_link = i`, `child_link = i + 1`, and
    remove the child link of each removed joint so `links.size() ==
    joints.size() + 1`. Test: 3-joint DH arm, remove the middle joint →
    `validate() == []`, 2 joints, 3 links.
1.  File menu gets submenu "Import" with action "URDF…" (file dialog
    filter `*.urdf`). Flow:
    a. Run `import_urdf(path)` via `TaskRunner.submit_fn`.
    b. On result with `model is None` → message box with
       `report.to_text()`; current project unchanged.
    c. On success → new public method
       `ProjectController.import_robot(parent_widget, model, name) -> bool`:
       asks `_confirm_discard` if the current project is modified (Cancel
       → return False, nothing changes), then creates a NEW project
       (name = robot name), sets the robot, marks it modified (unsaved),
       emits `project_opened`. Warnings/info → message box only if
       there are warnings (R3.6 replaces the box with the dock).
2.  Robot tree: REUSE `app/panels/model_panel.py` `ModelPanel` (the
    "Robot Model" dock already exists with a placeholder QTreeWidget). Do
    NOT create a new tree panel file. `ModelPanel.set_model(model,
    active_joints)`: root item = link 0; under each link item one item per
    child joint labelled `joint_name [revolute|prismatic|fixed]` (+
    ` mimic→<master>` if mimic), and under the joint item its child link
    item. Active-chain joints and links bold; others grey. Signal
    `selection_changed(str kind, int index)` with kind `"link"` or
    `"joint"`; method `select(kind, index)`. Read-only in R3.4. Empty
    model → header "No robot loaded".
3.  `PropertiesPanel` (existing placeholder): shows read-only properties
    of the item selected in the tree: joint (name, type, kinematics,
    parent/child link names, origin xyz/rpy, axis, limits, mimic) or link
    (name, index, inertia mass). No editing in R3.4.
4.  `DHEditorWidget`: rows of `origin_axis` joints are read-only (cells and
    type combo disabled, row tinted) with tooltip "Imported joint: not
    DH". Add/remove joint when the robot is not a plain chain is refused:
    nothing changes, status text "Tree robots are not editable here".
    Convention combo still works (only affects DH rows). An all-DH plain
    chain behaves exactly as in R2 (R2 tests unchanged).
5.  `JointPanel`: top section rows = active-chain joints that are
    non-fixed and non-mimic (from `fk()["active_joints"]`), in chain
    order. Below, a collapsed group "Other branches" (QGroupBox or
    collapsible) with rows for all other non-fixed, non-mimic joints, in
    joint order (they move the model; TCP readout uses the active chain
    so it does not change). For a plain chain the group is hidden and the
    panel looks exactly as in R2. Rebuild when `tcp_link`, topology,
    joint types or limits change (not on q change).
6.  `RobotView`:
    -   One triad per link: `frames[k]` for k in 0..links-1 (as now).
    -   Joint markers at `frames[child_link]` of each non-fixed joint.
    -   `polyline_points` keeps its R2 meaning for the ACTIVE CHAIN:
        origins of the links on the path link 0 → `tcp_link` (in order)
        then the TCP origin, blue. For a plain chain with `tcp_link` = last
        this equals R2 (frames + TCP), so R2 tests stay valid.
    -   New `branch_actor` / `branch_segments`: one line segment per joint
        NOT on the active chain, parent link origin → child link origin,
        dim grey (#777777, opacity 0.6). Absent (None) for a plain chain.
    -   TCP triad at `frames[tcp_link] · tool` (= `fk()["tcp"]`).
    -   In-place update on q change; rebuild on topology/tcp_link change.
    -   `highlight_joint(index | None)`: marker of that joint drawn 1.6×
        larger; previous highlight reset.
7.  Wiring: tree `selection_changed("joint", i)` → `RobotView.highlight_joint(i)`
    and `PropertiesPanel` update. `robot_changed` / `project_opened` →
    `ModelPanel.set_model`.
8.  Menu "Robot" (new top-level menu after View) with action "Set TCP
    link…": dialog (QInputDialog item list of link names). Writes
    `model.tcp_link` (index), marks dirty, emits `robot_changed`
    (everything refreshes).
9.  R2.9 open item: in `test_milestone_r2.py` add assertions that the DH
    table cells after save/close/reopen equal the values before.

Tests `tests/python/test_ui_tree.py` (offscreen, use the R3.3 fixtures):
remove-middle-joint fix; import 6-DOF fixture → tree has 7 link items,
active chain bold; import gripper fixture → finger items grey,
`branch_segments` count = joints off the active chain, `polyline_points`
= active-chain origins + TCP (tol 1e-9); Set TCP link moves the TCP triad
and changes JointPanel rows; DH editor refuses edit on origin_axis row
and add/remove on a tree (model JSON unchanged); JointPanel row counts for
chain and "Other branches"; import with errors keeps the old project;
import when modified + Cancel keeps the old project; tree selection
enlarges the marker and fills PropertiesPanel; no regression: all R2
tests green.

Done when: tests green, graphify, commit `R3.4: tree UI`.

---------------------------------------------------------------------

## R3.5 --- Meshes and assets (Python)

Goal: URDF visual geometry shown in the viewport; project archive keeps
the mesh files.

1.  Extend `import_urdf`: parse `<link><visual>` elements (first visual
    per link; others → info `URDF_EXTRA_VISUAL`). Geometry: `box size`,
    `cylinder radius length`, `sphere radius`, `mesh filename scale`.
    `<origin>` of the visual = pose inside the link frame (default 0).
    Colour: inline `<material><color rgba>` or a named top-level
    `<material>`; else grey (0.7,0.7,0.7,1).
    New file `app/importers/visuals.py`: dataclass
    `LinkVisual(link_index: int, link_name: str, kind: str ("box"|"cylinder"|"sphere"|"mesh"),
    params: dict, origin_xyz: list[float], origin_rpy: list[float],
    rgba: list[float], mesh_path: str | None, fallback: bool = False)`
    with `to_dict()` / `from_dict()`. `link_index` uses the CANONICAL link
    index from R3.3. Filled into `ImportResult.visuals`; `<collision>`
    elements → same class into `ImportResult.collisions`. The C++ URM
    does not store visuals in R3 (`Link.geometry_file` stays unused).
    Project storage: `ProjectData` gains `visuals: list[LinkVisual]` and
    `collisions: list[LinkVisual]` (default empty). Serializer writes
    `visuals.json` = `{"visuals": [...], "collisions": [...]}` and
    `assets/` into the archive. `PROJECT_FORMAT_VERSION` → `"1.1.0"`
    (same major, so old loader logic still accepts it). Old 1.0.0
    projects without `visuals.json` open with empty lists.
    `ProjectController.import_robot(...)` (R3.4) gains `visuals=None,
    collisions=None` and stores them in the new project.
    New report codes (add to `import_report.py`): `URDF_MESH_MISSING`
    (warning), `URDF_MESH_FORMAT` (warning), `URDF_MESH_TOO_LARGE`
    (warning), `URDF_EXTRA_VISUAL` (info).
2.  Mesh path resolution `resolve_mesh(filename, urdf_dir, search_roots)`:
    `package://<pkg>/<rel>` → try each search root (`<root>/<pkg>/<rel>`,
    `<root>/<rel>`), then walk up from the URDF directory looking for a
    folder named `<pkg>`; `file://` and absolute paths used as is;
    relative paths relative to the URDF directory. First existing file
    wins; none → `None`. Search roots come from `AppSettings`
    (`import/mesh_search_roots`, list of paths, editable in a small
    dialog under File > Import > Mesh search paths…). `import_urdf(path,
    search_roots=None)` gets the roots as an argument (no settings access
    inside the importer, so it stays testable).
    `mesh_path` stored in `LinkVisual` = the resolved absolute path at
    import time.
3.  Loading (in `app/importers/visuals.py`, `load_visual_mesh(v) ->
    pv.PolyData`): PyVista reads `.stl`, `.obj`, `.ply`. `.dae` and others
    → warning `URDF_MESH_FORMAT`, `fallback = True`. Missing file →
    warning `URDF_MESH_MISSING`, `fallback = True`, import still
    succeeds. Fallback geometry = cube of edge = current triad length at
    the visual origin. Mesh `scale` applied. Meshes over 2,000,000
    triangles → warning `URDF_MESH_TOO_LARGE`, not drawn (no decimation
    in R3.5). Primitives built with `pv.Cube`, `pv.Cylinder` (URDF
    cylinder axis = link z; PyVista default axis is x → set
    `direction=(0,0,1)`), `pv.Sphere`.
4.  `RobotView.set_visuals(visuals)` (call on project open / import)
    creates one actor per visual (mesh loaded once, cached by path).
    `update()` sets each actor's `user_matrix = frames[link_index] ·
    pose_to_iso(visual origin)` in place on q change. New toggle
    `set_show_visuals(bool)` + View > Robot > "Show Visuals" (default on),
    independent of Show Frames. `clear()` also removes visual actors.
5.  Collision geometry: parsed and stored in `visuals.json` under
    `collisions` but NOT drawn (used in R7). Keep the schema minimal.
6.  Archive: on save, each distinct mesh file is written into `assets/`
    (name = file basename; name clash → `name_1.stl`, `name_2.stl`…);
    `visuals.json` stores `mesh_path` as the archive-relative name
    (`assets/name.stl`). On open, `assets/*` are extracted to a temp cache
    dir per project (`tempfile.mkdtemp`, removed on project close) and
    `mesh_path` is rewritten to the extracted absolute path. A saved
    project opens on a machine without the original mesh files. Missing
    asset inside the archive → log warning, fallback cube, project still
    opens.

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
    (`ModelPanel.select(kind, index)` from R3.4) when `element` names
    one: look the name up in the loaded model's joints first, then links;
    no match (or no model) → nothing happens. New file
    `app/panels/import_report_panel.py`, method `set_report(report)`.
2.  The report is shown after every import (URDF now; CAD later), never
    hidden when it has errors or warnings; info-only reports show only in
    the dock (no popup).
3.  `ImportReport.to_text()` format fixed:
    `[SEVERITY] CODE: message (element)` one per line + summary line
    `N errors, M warnings, K info`. "Save report…" writes it to a file.
4.  Severity policy (document in `docs/import-report.md`):
    -   error = model unusable, no model loaded (old project kept);
    -   warning = model loaded but something was approximated or is
        missing (mesh missing, unsupported mesh format, continuous joint
        mapped to ±π, limit missing, physical data missing/bad);
    -   info = a choice was made or something was skipped on purpose (TCP
        link chosen, element kind ignored, extra visual ignored).
    The doc has a table of ALL codes (from R3.3 + R3.5 + later) with
    severity and meaning. This policy wins over any older wording.
5.  (No new validation step here. `import_urdf` already turns
    `validate()` failures into `URDF_INVALID_MODEL` errors, R3.3 item 8.
    R3.7 adds physical-data warnings.)
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

1.  REUSE the existing `urm::Inertia { mass, com, inertia_tensor }` on
    `Link::inertia` (`core/urm/link.hpp`, JSON key `"inertia"`, already
    bound as `cb.Inertia`). Do NOT add a new struct or JSON key. Meaning:
    `inertia_tensor` order (ixx, ixy, ixz, iyy, iyz, izz) about the COM,
    axes parallel to the link frame, kg·m²; `com` in the link frame, m.
    Change defaults: `mass = 0.0` and `inertia_tensor = {0,0,0,0,0,0}`
    (mass 0 means "not set"; old defaults 1.0 / identity were fake
    values). `from_json`: missing `"inertia"` or missing `"mass"` →
    0.0. Stored values in old files are kept as they are.
    `schema_version` stays 2 (no new keys). Update `docs/urm-schema-v2.md`
    and any C++/Python test that asserted the old defaults.
2.  New `std::vector<std::string> RobotModel::validate_physical() const`
    (bound to Python). Kept SEPARATE from `validate()` on purpose:
    `validate()` = structural errors (import fails), `validate_physical()`
    = physical warnings (model still loads). Rules (pass + fail test
    each): mass finite and ≥ 0; com and tensor finite; if mass > 0:
    tensor positive definite (all eigenvalues > 1e-12) and principal
    moments satisfy the triangle inequality (Ia + Ib ≥ Ic − 1e-9 for each
    permutation); a non-root link that is the child of a non-fixed joint
    with mass == 0 → message "mass not set". Messages name the link.
3.  URDF import: parse `<inertial><origin xyz rpy/><mass value/><inertia
    ixx ixy ixz iyy iyz izz/></inertial>`. `com = origin.xyz`. Rotate the
    tensor into link-frame axes: `I_link = R · I · Rᵀ`, R from origin rpy
    (same RPY rule as `Pose`). New report codes: `URDF_NO_INERTIAL`
    (warning, non-root link without `<inertial>`), `URDF_ROOT_NO_INERTIAL`
    (info, root link without `<inertial>`), `URM_PHYSICAL` (warning, one
    per `validate_physical()` message after import). Physical warnings
    never block the import. Add them to `docs/import-report.md`.
4.  Estimate tool in new `app/physics/estimate.py`:
    `estimate_inertial(mesh: pv.PolyData, density=None, total_mass=None)
    -> (mass, com[3], tensor[6], warnings: list[str])`. Exactly one of
    `density` / `total_mass` given, else `ValueError`. Uniform density;
    volume, COM and inertia from a closed triangle mesh with numpy
    (divergence-theorem / signed-tetrahedron method, Eberly "Polyhedral
    mass properties"); triangulate first. Not watertight
    (`mesh.n_open_edges > 0`) → warning and use the convex hull
    (`mesh.delaunay_3d().extract_surface().triangulate()`; no new
    dependency). Visual meshes are first transformed by their visual
    origin so results are in the link frame. Primitives (box, cylinder,
    sphere) use analytic formulas, function
    `primitive_inertial(kind, params, origin, density|total_mass)`.
5.  Panel `PhysicalPropertiesPanel` dock "Physical": table, one row per
    link: mass, COM x y z, six inertia terms (editable, SI units,
    scientific notation allowed), a status icon from
    `validate_physical()`. New file `app/panels/physical_panel.py`, dock
    "Physical" with a View-menu toggle. Buttons:
    "Estimate from visual" (selected link or all links, asks density or
    total mass), "Scale all masses to total…". Shows total mass and total
    COM in world at current q: `Σ m_k · (frames[k] · com_k) / Σ m_k`
    (frames from `cb.fk`). Edits mark the project dirty and are saved in
    the URM JSON (`robot_model.json`). `physical_properties.json` in the
    project zip stays a legacy empty dict: the URM is the only source of
    truth for mass data.
6.  Gravity: already exists (`SimulationSettings.gravity`, default
    [0,0,−9.81], saved in `simulation_settings.json`). No work in R3.

Tests: C++ `test_urm.cpp`: inertia round-trip, missing → mass 0, each new
`validate_physical` rule pass/fail, `validate()` unaffected by bad
inertia, old v2 JSON opens. Python: URDF inertial import
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
0.  Builder writes the fixture `tests/python/fixtures/urdf/six_dof_with_gripper.urdf`:
    links `base, l1..l6, finger_l, finger_r` (9 links); joints `j1..j6`
    revolute (non-trivial origins and axes, not all z), `jf_l`
    prismatic l6→finger_l (axis y), `jf_r` prismatic l6→finger_r (axis
    −y) with `<mimic joint="jf_l" multiplier="1" offset="0"/>`. Every
    link has an `<inertial>` and a visual (STL cube fixture from R3.5).
    `dof()` = 7. Auto `tcp_link` (R3.3 rule) = `finger_l` (depth 7).
1.  Import the fixture. Report has no errors (info `URDF_TCP_CHOSEN`
    expected).
2.  "Set TCP link…" → `l6`. Tree dock shows all 9 links; active chain =
    j1..j6; finger joints in "Other branches"; finger segments in
    `branch_segments` (dim).
3.  Move joints with the JointPanel; TCP readout equals an independent
    numpy FK (origin+axis math written in the test, no `cb.fk`), tol
    1e-9.
4.  Move `jf_l` → `finger_r` frame moves by the mimic rule (numpy check).
5.  Total mass and COM shown; equal to the numpy sum from the fixture.
6.  Save, close, open: robot, visuals (meshes from the archive), physical
    data, tcp_link all identical.
7.  Import the broken fixtures: errors shown, old project unchanged.

Stage gate = `docs/MASTER_PLAN.md` section 1.3. Then:
-   Set stage status `stage-ready-for-review` in `PROGRESS.md`.
-   Push `main` and tag `stage-R3-done` (when a remote exists).
-   STOP. R3 is a blocking-review stage. Wait for Reviewer.
