# Progress and Handoff

Shared status file between the Builder (Claude subagents launched by Sonnet) and the
Reviewer (Claude Sonnet, Claude Code). Read this file first. Update it at
every handoff. Keep it short.

## Current

-   **Stage:** R4 --- Kinematics and dynamics (status: `planned`, spec pack written, ready for Builder)
-   **Next substage:** R4.5 --- Singularity indicators
-   **Owner now:** Builder subagents (Sonnet orchestrates)
-   **Blockers:** none
-   **Review mode:** stage-batch, BLOCKING at end of R4
-   **Spec pack:** `docs/specs/R4-spec.md`
-   **Stage R2 gate: PASSED** (Reviewer verified: CTest 5/5, pytest 146/146). Tag `stage-R2-done` = `91a53a0`.
-   **Last known test count (2026-11-11, R3 gate):** CTest 6/6 passed; pytest 302 passed (6 new in `test_milestone_r3.py`).
-   **Stage R3 gate (Builder run):** milestone `tests/python/test_milestone_r3.py` PASS (6 tests: 1 fixture-shape, 1 full flow items 1-6, 4 broken-fixture cases item 7). Items 1,2,3,4,5,6,7,9,10 checked. `run_tests.ps1` not used (execution policy), same commands run by hand. `build/` and binaries not tracked. `graphify update .` run, `graph.json` committed. Item 8 (docs per module): only `docs/import-report.md` and `docs/urm-schema-v2.md` exist; no separate one-page docs for compiled model, visuals/assets, physical panel (not done, reviewer decide). Item 5 (GPU) n/a. No product code changed in the milestone. Tag `stage-R3-done`. Milestone commit `d329649`.

-   Stage R3 review (Sonnet, 2026-11-11): APPROVED. Independent run CTest 6/6, pytest 302. Gate item 8 docs fixed in 2d3d304. Tag stage-R3-done = 6249b0e (docs commit 2d3d304 follows the tag). Accepted deviations: R3.2 theta0/d0 in `pre`; legacy 0/0 serial fallback (effective_links); R3.7 multi-link total-mass estimate shared by volume. R4 first item: unify joint_links() with effective_links.

### New agent? Read in this order

1.  `.agents/rules/*.md` (Builder: `continue-protocol.md` has the loop).
2.  This file: "Current", "Substage log", "Open review items", "Decisions".
3.  `docs/MASTER_PLAN.md` section 1 (workflow, stage gate, push) and the current stage section.
4.  Current spec pack `docs/specs/R3-spec.md` ("Common rules" and "Code facts" first).
5.  `docs/urm-schema-v2.md` (URM data model).
6.  Run `graphify update .` then `graphify query "<area>"` before touching code.

## Handoff protocol (stage-batch, from R3)

1.  Builder builds every substage of the stage in order. Per substage:
    tests green → `graphify update .` → row status `committed` → commit
    `R<x>.<y>: <summary>`.
2.  After the last substage: stage milestone test + stage gate
    (`docs/MASTER_PLAN.md` 1.3), results under "Current", tag
    `stage-R<x>-done`, stage status `stage-ready-for-review`, Owner →
    Reviewer.
3.  Reviewer sets `stage-approved` or `stage-changes-requested` with an
    `R<x>-fix` list under "Open review items", and writes the next spec pack.
4.  A substage red after two fix attempts → row `blocked`, write it under
    "Blockers", stop, wait for the user.
5.  Advisor (Opus) consulted → record it in "Decisions".

Status values: `planned` · `in-progress` · `committed` · `blocked` ·
`stage-ready-for-review` · `stage-approved` · `stage-changes-requested`.
Legacy (R0-R2 rows only): `ready-for-review` · `approved` ·
`changes-requested`.

## Substage log

| ID    | Summary                                   | Tests                              | Status   |
|-------|-------------------------------------------|------------------------------------|----------|
| R0.1  | Hardware audit                            | docs/hardware-audit.md             | approved |
| R0.2  | C++20 core_lib + CMake                    | tests/cpp/test_placeholder.cpp     | approved |
| R0.3  | (not tagged in code --- fill in)          | ?                                  | ?        |
| R0.4  | pybind11 bridge                           | tests/python/test_bridge.py        | approved |
| R0.5  | Eigen 3.4.0                               | tests/cpp/test_eigen.cpp           | approved |
| R0.6  | CUDA vector-add on SM 12.0                | tests/cpp/test_cuda.cpp            | approved |
| R0.7  | PyVista viewport in Qt                    | tests/python/test_viewport_smoke.py| approved |
| R0.8  | Pinocchio decision (deferred)             | docs/hardware-audit.md             | approved |
| R1.1  | Main window shell                         | tests/python/test_ui_smoke.py      | approved |
| R1.2  | AppSettings                               | tests/python/test_settings.py      | approved |
| R1.3  | Logger + log panel                        | tests/python/test_logger.py        | approved |
| R1.4  | TaskRunner                                | tests/python/test_task_runner.py   | approved |
| R1.5  | ProjectData + serializer                  | tests/python/test_project.py       | approved |
| R1.6  | ProjectController                         | tests/python/test_project_controller.py | approved |
| R1.7  | Viewport in main window                   | tests/python/test_viewport_in_main_window.py | approved |
| R2.1  | URM schema + JSON round-trip              | tests/cpp/test_urm.cpp             | approved (follow-ups → R2.4) |
| R2.2  | URM pybind11 bindings                     | tests/python/test_urm_bridge.py    | approved |
| R2.3  | DH editor widget                          | tests/python/test_dh_editor.py     | approved (wiring → R2.6) |
| R2.4  | URM hardening                             | test_urm.cpp, test_urm_bridge.py   | approved |
| R2.5  | FK engine (DH + MDH), C++ + bindings      | test_fk.cpp, test_fk_bridge.py     | approved (follow-ups → R2.6) |
| R2.6  | Wire model + DH editor into app/project   | test_robot_in_project.py           | approved |
| R2.7  | Viewport: links, frames, TCP              | test_viewport_robot.py             | approved (follow-ups → R2.8) |
| R2.8  | Joint control panel + TCP pose readout    | test_joint_panel.py                | approved (follow-up → R2.9) |
| R2.9  | 3-DOF end-to-end milestone test           | test_milestone_r2.py               | approved |
| R3.1  | URM schema v2: tree + origin/axis joints  | test_urm.cpp, test_urm_bridge.py   | stage-approved |
| R3.2  | Compiled kinematic model + tree FK        | test_fk.cpp, test_compiled.cpp, test_fk_tree.py | stage-approved |
| R3.3  | URDF parser → URM (no meshes)             | test_urdf_import.py                | stage-approved |
| R3.4  | UI for tree/origin_axis joints            | test_ui_tree.py                    | stage-approved |
| R3.5  | Mesh/asset resolution + visual meshes     | test_urdf_meshes.py                | stage-approved |
| R3.6  | Import validation + error report          | test_import_report.py              | stage-approved |
| R3.7  | Physical properties (import/estimate/manual) | test_physical_props.py, test_urm.cpp | stage-approved |
| R3.8  | CAD import (mini-stage R3b, spec after R3.7) | tbd                | planned  |
| R4.1 | Compiled model freeze + to_arrays | test_compiled_arrays.py, test_compiled_arrays.cpp | committed |
| R4.2 | Jacobian | test_jacobian.py | committed |
| R4.3 | Numerical IK | test_ik.py | committed |
| R4.4 | Multiple IK solutions | test_ik_multi.py | committed |
| R4.5 | Singularity indicators | test_singularity.py | planned |
| R4.6 | Inverse dynamics (RNEA) | test_dynamics.cpp, test_dynamics.py | planned |
| R4.7 | Mass matrix + forward dynamics | test_dynamics.py | planned |
| R4.8 | Energy and torque tools | test_energy.py | planned |
| R4.9 | IK + analysis panels | test_ui_kin_dyn.py | planned |
| R4.10 | R4 milestone | test_milestone_r4.py | planned |

R0--R1 rows marked `approved` from existing code and tests. Reviewer has
not re-run them in this file's first version.

## Substage specs

Specs do not live in this file. Find them here:

-   Stage R3 (current): `docs/specs/R3-spec.md` (R3.2 to R3.7 + stage milestone).
-   Stage R4 (current): `docs/specs/R4-spec.md`.
-   R3.1 (done): `docs/specs/R3.1-spec.md`.
-   Stage R2 (closed): `docs/specs/R2-spec.md`.
-   R3.8 CAD: not specified yet. Reviewer writes it as mini-stage `R3b` after R3.7.

## Open review items

-   (info) R2.6 DH editor: assign whole lists to `Pose.xyz`/`rpy`
    (`model.base.xyz = [...]`); element assignment is silently lost.
-   (info) R2.5 weak tests (no 3D known-answer) → follow-ups at top of R2.6.
-   (resolved) URM serial chain → tree in R3.1 (see Decisions, Opus).
-   (resolved R3.4) R2.9 test now asserts DH table cells after reopen.
-   (resolved R3.4 item 0) DH editor remove-joint leaves stale parent_link/child_link and extra links → validate() fails.
-   (info) R3 review: `joint_links()` in app/panels/model_panel.py duplicates C++ `kin::effective_links` (legacy 0/0 serial rule). Unify in R4 (bind effective_links or expose via compile).

## Decisions

-   2026-11-11: Pinocchio deferred; custom C++20/Eigen engine (R0.8).
-   2026-11-11: Roles --- Gemini 3.8 Medium builds, Sonnet 5.5 reviews,
    Opus advises only when absolutely necessary.
-   2026-11-11: DH/MDH convention is per robot, not per joint (R2.4).
    `from_json` loads without validating; `validate()` is explicit.
-   2026-11-11: OPUS consulted (tree vs chain for URDF). Answer: B-lite. URM becomes a TREE now (schema v2): `Joint.parent_link`, `child_link` (parent < child, base link 0, links.size()==joints.size()+1 kept), `Joint.kinematics: "dh"|"origin_axis"`, `Joint.origin: Pose`, `Joint.axis`, optional `Joint.mimic {joint,multiplier,offset}` (mimic not counted in dof()), `RobotModel.tcp_link` (default last link; tool pose hangs off it). One precompile step turns every joint into (T_origin, axis, type, parent, q_index); FK/Jacobian/dynamics/CUDA read only compiled form. Never convert origin_axis → DH. DH editor refuses origin_axis joints. UI + IK act on active chain root→tcp_link; other branches drawn and flagged. Risks: (1) joint index vs dof index vs link index confusion → one tested `q_index` map; (2) DH↔origin round trip → one-way only; (3) CUDA layout → flat SoA of compiled form in topological order, freeze in R4 before R5.
-   2026-11-11: R2.7 scale formula changed in R2.9 (adds tool offset, floor 0.05 → 0.01). Accepted.
-   2026-11-11: URDF parsing uses Python stdlib `xml.etree` in `app/` (no new C++ dependency). Importer builds URM through bindings.
-   2026-11-11: Plan audit (Sonnet, no Opus). R3 spec pack fixed for a fresh Builder: (1) compiled joints sorted by child_link (URM joint order is not topological); (2) mimic chains flattened at compile time to the root master; mimic of a fixed joint is invalid; (3) URDF import emits canonical order `joints[i].child_link == i+1`; returns `ImportResult(model, report, visuals, collisions)`; full report-code table; (4) tree UI reuses `ModelPanel`/`PropertiesPanel`, `polyline_points` keeps R2 meaning for the active chain, new `branch_segments`; (5) R3.7 reuses existing `Link.inertia` (defaults → mass 0 = not set), physical checks in separate `validate_physical()` (warnings); (6) project format 1.0.0 → 1.1.0 for `visuals.json` + `assets/`; (7) R3.1 counted `committed` under stage-batch, reviewed with stage R3.
-   2026-11-11: Workflow change by user: Gemini retired; Sonnet orchestrates, Builder = Claude subagents (Haiku/Sonnet tiers), Sonnet reviews each substage diff.

-   2026-11-11: R3.2 legacy serial fallback in compile() when all joints 0/0 (Sonnet decision)
-   2026-11-11: Workflow: Sonnet orchestrates, Claude subagents build (Haiku: R3.3, R3.6; Sonnet: R3.2, R3.4, R3.5, R3.7, milestone). No Opus used in R3.
-   2026-11-11: OPUS consulted (R4.6 dynamics formulation, planned decision point). Answer: Featherstone RNEA in the link frame with motion subspace S_k = ^kX_post^-1 [a;0] (revolute S=[Rp^T a; Rp^T(a x pp)], prismatic S=[0; Rp^T a]), base acceleration a0=[0; -R_base^T g], mimic via mult on qd/qdd and tau[src]+=mult*S^T f, fixed joints still propagate force, CRBA per DOF with mult_i*mult_j coupling and 2v when si==sj, friction per DOF outside g and C (tanh smoothing), reference tests: planar 2R, cart-pendulum, post-rotation case. Details in R4-spec R4.6/R4.7.
-   2026-11-11: R4 decisions (Sonnet): friction not stored in URM in R4 (no schema bump); frozen compiled layout covers kinematics only, dynamics in separate dyn::DynModel; no 6R closed-form IK in R4; viewport target dragging moved to R6 backlog.

-   2026-11-11: effective_links is pure; link-count check lives in compile() (Sonnet decision, R4.1).

## Risks

-   CUDA Toolkit 13.5 vs driver max 13.1. Update driver before R5.
