# Progress and Handoff

Shared status file between the Builder (Claude subagents launched by Sonnet) and the
Reviewer (Claude Sonnet, Claude Code). Read this file first. Update it at
every handoff. Keep it short.

## Current

-   **Stage:** R6 --- stage-approved (Sonnet review 2026-11-12)
-   **Next substage:** R7.1 --- Trajectory generation + constraint checks (spec `docs/specs/R7-spec.md`)
-   **Stage R6 gate (Builder run, 2026-11-12):** milestone `tests/python/test_milestone_r6.py` PASS (8 tests: items 1-2 import l6 + 4-waypoint cubic trajectory saved/reopened equal; 3 kinematic playback; 4a dynamic computed-torque tracking, 4b energy balance; 5 all 10 graph kinds; 6 CSV+NPZ export; 7 memory + timers; 1 print test). CTest 9/9 (`ctest --test-dir build -C Release`); pytest 824 passed (816 before + 8 new). `run_tests.ps1` not used (execution policy), same commands by hand; C++ not rebuilt (no C++ changed in R6). Gate items: 1 rows R6.1-R6.8 committed; 2 CTest + pytest green; 3 milestone green; 4 numerical references green (below); 5 GPU/CPU equivalence n/a (R5 covers it); 6 `graphify update .` run, `graph.json` committed; 7 new dependency pyqtgraph 0.14.0 proof-built in R6.6 (`requirements.txt`); 8 docs `docs/simulation.md`, `docs/trajectory.md`, `docs/graphs.md` (signals table, units, library, export formats); 9 this file; 10 R2-R5 milestones still green in the full pytest run. `build/` and binaries not tracked. No product code changed in R6.8. Unrelated working-tree change left uncommitted: `tests/python/fixtures/robot_model_v1.json`.
-   **Stage R6 numbers (six_dof_with_gripper, 7 DOF, TCP l6, RTX/CPU Python path):** trajectory 4 waypoints cubic clamped, 4 s. Kinematic final q error vs last waypoint 0.0 (limit 1e-9), 60 samples at 4x speed, all 14 signals finite. Dynamic computed torque (gravity on, RK4 dt 1 ms, Kp 100 Kd 20): tracking RMS 3.67e-11 rad, max 2.17e-10 rad over 4000 samples (limit 1e-3). Free-motion energy (gravity 0, tau 0, friction 0, RK4 1 ms, 1 s, E0 0.618 J): |dE|/E0 = 6.47e-15 (limit 1e-8). Export CSV and NPZ re-import equal to recorder arrays, max diff 0 (exact %.17g); TCP pos vs independent numpy FK (`numpy_ref.fk_numpy`) 1.67e-16 m (limit 1e-9). Recorder buffer 67,108,704 bytes <= 64 MiB cap (67,108,864), 0 dropped; no active sim/graph timers after stop, hide or close. Other R6 numbers: RK4 20,500 steps/s (R6.2); recorder 0.030 ms/sample (R6.6).
-   **R6 notes for the reviewer:** sim panel controls (dt, integrator, scale, gravity, friction) change the engine only, not project SimulationSettings. Kinematic mode run buttons are disabled; trajectory playback drives the clock. Recorder clears on STOPPED->RUNNING and reset, not on Stop/Pause. Project format 1.2.0 (`trajectory.json`, old files load). Milestone drives dynamic playback through `player.advance(0.02)` (deterministic) and kinematic playback through the dock QTimer.
-   Stage R6 review (Sonnet, 2026-11-12): APPROVED. Independent run: CTest 9/9, pytest 824. Read: app/sim/recorder.py, playback.py, export.py, graphs/series.py, trajectory.py, engine.py diff; no blocking defects. Tag stage-R6-done = bcd8f75. Fixed during stage: recorder cleared on Stop (now clears on new run). Accepted: kinematic playback runs the engine clock via set_state; pyqtgraph 0.14; CSV joined table with `# columns` line. Info items (not blocking): (a) prismatic unit shown as revolute for per-joint series; (b) RingBuffer.view() copies full buffer when wrapped (64 MB per live update at cap); (c) dynamic tracking test has tiny error (3.7e-11) because feedforward is exact and start state equals trajectory start: no disturbance case. R7 spec pack written.
-   **Owner now:** Reviewer (Sonnet)
-   **Blockers:** none
-   **Review mode:** stage-batch, non-blocking (R7)
-   **Spec pack:** `docs/specs/R7-spec.md`
-   **Stage R2 gate: PASSED** (Reviewer verified: CTest 5/5, pytest 146/146). Tag `stage-R2-done` = `91a53a0`.
-   **Last known test count (2026-11-11, R3 gate):** CTest 6/6 passed; pytest 302 passed (6 new in `test_milestone_r3.py`).
-   **Stage R3 gate (Builder run):** milestone `tests/python/test_milestone_r3.py` PASS (6 tests: 1 fixture-shape, 1 full flow items 1-6, 4 broken-fixture cases item 7). Items 1,2,3,4,5,6,7,9,10 checked. `run_tests.ps1` not used (execution policy), same commands run by hand. `build/` and binaries not tracked. `graphify update .` run, `graph.json` committed. Item 8 (docs per module): only `docs/import-report.md` and `docs/urm-schema-v2.md` exist; no separate one-page docs for compiled model, visuals/assets, physical panel (not done, reviewer decide). Item 5 (GPU) n/a. No product code changed in the milestone. Tag `stage-R3-done`. Milestone commit `d329649`.

-   **Stage R4 gate (Builder run, 2026-11-11):** milestone `tests/python/test_milestone_r4.py` PASS (6 tests: import+TCP l6, IK listed/sorted/applied vs independent numpy FK, singularity flag + sigma_min vs numpy SVD of numeric Jacobian, torques vs `numpy_dynamics` at 1e-8 rel + gravity linearity, FD/RNEA round trip + 0.5 s RK4 energy |dE|/E0 < 1e-8, save/close/open identical). CTest 8/8 passed (`ctest --test-dir build -C Release`); pytest 461 passed (455 before + 6 new). `run_tests.ps1` not used (execution policy), same commands run by hand after `cmake --build build --config Release`. Gate items: 1 rows R4.1-R4.10 committed; 2 CTest + pytest green; 3 milestone green; 4 numerical references green (closed-form, numpy Lagrangian, numeric Jacobian); 5 GPU/CPU equivalence n/a (R5 onward); 6 `graphify update .` run, `graph.json` committed; 7 no new dependency; 8 docs `docs/jacobian-ik.md`, `docs/dynamics.md`, `docs/analysis-panels.md` written, `docs/compiled-model-layout.md` re-checked against code (matches); 9 this file; 10 earlier milestones (R2, R3) still green in the full pytest run. `build/` and binaries not tracked. Spec item 8 (Pinocchio validator) NOT done (needs user decision). No product code changed. Milestone commit `51f15ed`. Tag `stage-R4-done` local only (on the `docs(progress): R4 ready for review` commit), no remote, not pushed. Unrelated working-tree changes left uncommitted: `tests/python/fixtures/robot_model_v1.json` and some `graphify-out/` files.
-   **Stage R5 gate (Builder run, 2026-11-11):** milestone `tests/python/test_milestone_r5.py` PASS (9 tests on the real RTX 5060, none skipped: items 1,2,4 GPU workspace 1e6 via panel; item 3 1e4 subset GPU vs CPU; items 5a cancel, 5b CPU by setting vs GPU 1e5, 5c fake OOM fallback, 5d ANALYZER_FORCE_CPU; item 6 FK 1e6 fp32; item 7 benchmark JSON x2). CTest 9/9 (`ctest --test-dir build -C Release`); pytest 673 passed (664 before + 9 new). `run_tests.ps1` not used (execution policy), same commands run by hand after `cmake --build build --config Release`. Gate items: 1 rows R5.1-R5.7 committed; 2 CTest + pytest green; 3 milestone green; 4 numerical references green (R2-R4 tests unchanged, numpy oracle); 5 GPU/CPU equivalence (see next line); 6 `graphify update .` run, `graph.json` committed; 7 no new dependency (no CuPy; CUB ships with the CUDA toolkit); 8 docs `docs/gpu-compute.md`, `docs/gpu-kernels.md`, `docs/workspace-panels.md` written (plus `docs/gpu-precision-decision.md`, `docs/gpu-ik-decision.md`, `docs/workspace-sampling.md` from R5.2-R5.4); 9 this file; 10 R2, R3, R4 milestones still green in the full pytest run. `build/` and binaries not tracked. No product code changed in R5.7. Unrelated working-tree change left uncommitted: `tests/python/fixtures/robot_model_v1.json`.
-   **Stage R5 gate item 5 (GPU/CPU equivalence, numbers):** FK fp32 1e6 samples vs CPU double on (double)(float)q (`test_gpu_fk.py`, `docs/gpu-precision-decision.md`): worst pos 1.56e-6 m (tol 7.29e-5, 12R), rot 7.13e-7 rad; six_dof fixture in milestone item 6: pos 2.04e-7 (tol 6.17e-6), rot 4.68e-7 (tol 5.71e-6), ortho 1.29e-6 (tol 2.86e-6). fp64 GPU vs CPU: pos <= 3.2e-15, rot <= 1.7e-15 (limit 1e-12). Default precision fp32 (fp64 e2e 68.5 ms > 50 ms; 1e6 e2e fp32 37.6 ms, CPU 32 threads 127.3 ms). Workspace (`test_workspace.py` + milestone): 1e4 subset points max diff 1.79e-7 m (tol 6.17e-6), voxel set differs 2.03e-4 (limit 1e-3); 1e5 CPU vs GPU voxels differ 1.15e-4; 1e6 workspace on GPU 14.2 ms, VRAM peak 23.6 MiB of 6144 MiB budget, chunk 1000192, 1 chunk. GPU IK (`test_ik_batch.py`, `docs/gpu-ik-decision.md`): 1e5 targets CPU 82.8 ms 99.945 % vs GPU+polish 65.8 ms 99.955 % = 1.26x, NO-GO (rule 5x); product path = CPU batch IK.
-   Stage R5 status: `stage-ready-for-review`, non-blocking. Tag `stage-R5-done` local only, no remote, not pushed. R6 spec pack not written: next stage cannot start until Sonnet writes `docs/specs/R6-spec.md`.
-   Stage R5 review (Sonnet, 2026-11-11): APPROVED. Independent run: CTest 9/9, pytest 673. Read: app/compute/backend.py, core/compute/budget.hpp, cuda/gpu_manager.cu, core/compute/philox.hpp, core/compute/workspace.cpp (first 170 lines); no defects. Tag stage-R5-done = dd9a343. Accepted: fp32 default FK; GPU IK NO-GO (CPU batch IK is product path, flag GPU_IK_EXPERIMENTAL / USE_GPU_IK off); GPU IK prototype stays in repo. Spec pack R6 written.
-   Stage R4 review (Sonnet, 2026-11-11): APPROVED. Independent run: CTest 8/8, pytest 461. Verified: IK 2R finds both solutions with n_starts=30 for seeds 0..9 (tests use 60). Worst relative errors: M vs numpy oracle 5.8e-13, RNEA/FD round trip 1.4e-14, energy drift 2.5e-14. Tag stage-R4-done = b532f85. Accepted deviations: adaptive IK damping; weighted-row convergence check; gravity torque = +dV/dq (spec sign fixed in e1ea059); fixed-joint merge into parent; Richardson numeric Jacobian in the oracle. Not done on purpose: Pinocchio validator (R4-late, user decision), viewport target dragging (R6 backlog), friction not stored in URM.
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
| R4.1 | Compiled model freeze + to_arrays | test_compiled_arrays.py, test_compiled_arrays.cpp | stage-approved |
| R4.2 | Jacobian | test_jacobian.py | stage-approved |
| R4.3 | Numerical IK | test_ik.py | stage-approved |
| R4.4 | Multiple IK solutions | test_ik_multi.py | stage-approved |
| R4.5 | Singularity indicators | test_singularity.py | stage-approved |
| R4.6 | Inverse dynamics (RNEA) | test_dynamics.cpp, test_dynamics.py | stage-approved |
| R4.7 | Mass matrix + forward dynamics | test_dynamics.cpp, test_mass_matrix.py | stage-approved |
| R4.8 | Energy and torque tools | test_energy.py | stage-approved |
| R4.9 | IK + analysis panels | test_ui_kin_dyn.py | stage-approved |
| R4.10 | R4 milestone | test_milestone_r4.py | stage-approved |
| R5.1 | GPU Compute Manager | test_budget.cpp, test_gpu_manager.py | stage-approved |
| R5.2 | Batch FK kernel + precision decision (STOP after) | test_gpu_fk.py, test_fk_batch_cpu.py | stage-approved |
| R5.3 | Workspace sampling | test_workspace.py | stage-approved |
| R5.4 | CPU batch IK + GPU IK go/no-go | test_ik_batch.py | stage-approved |
| R5.5 | Python backend, fallback, Workspace panel | test_compute_backend.py, test_ui_workspace.py | stage-approved |
| R5.6 | Performance dashboard | test_performance.py | stage-approved |
| R5.7 | R5 milestone | test_milestone_r5.py | stage-approved |
| R6.1 | Simulation core | test_sim_core.py | committed |
| R6.2 | Integrators | test_integrators.py | committed |
| R6.3 | Manual mode + UI | test_sim_panel.py | committed |
| R6.4 | Trajectory + playback | test_trajectory.py, test_ui_trajectory.py | stage-approved |
| R6.5 | State tracking | test_recorder.py | stage-approved |
| R6.6 | Graph suite | test_graph_series.py, test_ui_graphs.py | stage-approved |
| R6.7 | Data export | test_export.py | stage-approved |
| R6.8 | R6 milestone | test_milestone_r6.py | stage-approved |
| R7.1 | Trajectory generation + checks | test_profiles.py, test_cartesian_traj.py, test_traj_checks.py | planned |
| R7.2 | Collision geometry | test_collision.py | planned |
| R7.3 | Planner decision (OMPL vs own) | PoC | planned |
| R7.4 | Path planning | test_planner.py | planned |
| R7.5 | Workspace + manipulability analysis | test_workspace_analysis.py | planned |
| R7.6 | Ghosts + comparison | test_configs.py, test_ui_ghosts.py | planned |
| R7.7 | R7 milestone | test_milestone_r7.py | planned |

R0--R1 rows marked `approved` from existing code and tests. Reviewer has
not re-run them in this file's first version.

## Substage specs

Specs do not live in this file. Find them here:

-   Stage R7 (current): `docs/specs/R7-spec.md`.
-   Stage R6 (closed): `docs/specs/R6-spec.md`.
-   Stage R5 (closed): `docs/specs/R5-spec.md`.
-   Stage R3 (closed): `docs/specs/R3-spec.md` (R3.2 to R3.7 + stage milestone).
-   Stage R4 (closed): `docs/specs/R4-spec.md`.
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
-   2026-11-11: OPUS consulted (R5 GPU design, GPU memory model decision point). Answer: fp32 default with CPU fp64 reference; templated kernel, precision rule at R5.2 (fp64 default for FK only if e2e_fp64 <= 2x e2e_fp32 and <= 50 ms per 1e6); GPU computes in base frame, host applies base in double; thread-per-sample, joint tables in shared memory (3x4 float), SoA q [dof][N] and output [12][N], SASS-only build for sm_120 (driver 13.1 vs toolkit 13.5), own Philox RNG shared by CPU and GPU, chunk size = clamp(floor(budget/(2*bytes_per_sample)),65536,4M) with 0.6 headroom, 50 ms launch target for WDDM TDR, OOM -> halve chunk -> CPU, sticky error -> GPU dead, GPU IK with unrolled fp32 Cholesky plus CPU fp64 polish and a go/no-go rule (>=5x faster, convergence within 1 point). Details in R5-spec.
-   2026-11-11: R5 decisions (Sonnet): GPU code in a separate optional pybind11 module `gpu_bridge` (core_bridge never links CUDA, so CPU fallback always imports); no CuPy (new dependency, needs Windows proof build); CPU batch FK and CPU batch IK are built in R5 as reference and fallback.
-   2026-11-11: R5.2 precision rule applied mechanically (Opus rule): GPU FK default precision = fp32, fp64 available via precision="fp64". Workspace and GPU IK always fp32.
-   2026-11-11: R5.4 GPU IK go/no-go = NO-GO (Sonnet applied Opus rule mechanically). 1e5 targets, 6-DOF fixture, RTX 5060 Laptop / Ryzen 9 8940HX 32 threads: CPU fp64 82.8 ms 99.945%; GPU fp32 + fp64 polish 65.8 ms 99.955% (1.26x, rule needs 5x; rate OK). CPU polish ~30 ms caps speedup at 2.7x. GPU IK stays behind GPU_IK_EXPERIMENTAL, backend uses CPU batch IK. See `docs/gpu-ik-decision.md`.
-   2026-11-11: R5.5 (Sonnet): Backend.solve_ik_batch always CPU (USE_GPU_IK=False, GPU IK NO-GO); workspace cloud/voxel actors in base frame with robot base as user_matrix; get_backend("cuda") without GPU returns CPU with reason.
-   2026-11-11: R6.2 (Sonnet): integrators stay Python on cb.forward_dynamics; engine computes tau_inv/tcp/energy lazily (refresh at end of advance) to meet the speed floor; no C++ sim_step.

-   2026-11-12: R6.6 (Sonnet builder): plot library = pyqtgraph 0.14.0 (pip install proof build OK offscreen: PlotWidget 1e5 points, grab() works); added `pyqtgraph>=0.13` to requirements.txt. Recorder clears on new run, not on Stop.

## Risks

-   CUDA Toolkit 13.5 vs driver max 13.1. Update driver before R5.
-   Driver 592.19 (CUDA 13.1) with toolkit 13.5: R5.1 verifies; an InsufficientDriver or UnsupportedPtxVersion error means the USER must update the driver (Builder writes a Blocker).
