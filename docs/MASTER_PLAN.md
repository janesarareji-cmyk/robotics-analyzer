# Master Plan: Stages R3 to R8

Note 2026-11-11: 'Gemini'/'Builder' now means Builder subagents launched by Sonnet.

Owner: Reviewer (Claude Sonnet 5.5). Builder: Gemini. Written 2026-11-11
after Stage R2 gate passed. Stages R0 to R2 are DONE and not repeated.

This file answers: what is built, in what order, how it is tested, when it
is pushed, when it is reviewed. `PROGRESS.md` stays the live status file.
`ROADMAP.md` stays the high-level scope list. This file is the detail.

---------------------------------------------------------------------

## 1. How work runs (stage-batch workflow)

### 1.1 Cycle for every stage

| Step | Who | Action | Output |
|------|-----|--------|--------|
| A | Claude | Writes the **stage spec pack**: one spec for every substage of the stage, all at once, before Gemini starts. Written at stage start (not now) because specs must match the real code at that time. | `docs/specs/R<x>-spec.md`, rows `planned` in `PROGRESS.md` |
| B | Gemini | Builds ALL substages of the stage in order. After each substage: tests green, `graphify update .`, local commit `R<x>.<y>: <summary>`. | commits |
| C | Gemini | Stage test: full `.\run_tests.ps1`, stage milestone test, stage gate list (section 1.3). | gate result in `PROGRESS.md` |
| D | Gemini | Push (section 1.4). | pushed `main` + tag `stage-R<x>-done` |
| E | Claude | Review (section 1.2). | `approved` / `R<x>-fix` list |
| F | Gemini | Next stage (back to step A). | |

### 1.2 Review cadence (decision, can be changed by the user)

Review is at the END of a stage, not after each substage.

-   **Blocking review** = Gemini does NOT start the next stage until Claude
    sets the stage `approved`. Applies to: **R3, R4** (schema and math that
    everything later depends on) and **R5.2 precision decision** (section 4, R5.2).
-   **Non-blocking review** = Gemini pushes, starts the next stage at once.
    Claude reviews the pushed stage when the user says "review". Findings
    become `R<x>-fix` substages done FIRST in the next session. Applies to:
    **R5 (after the R5.2 decision), R6, R7, R8**.
-   Maximum review lag: one stage. Never two unreviewed stages at once.
-   Claude review of a stage = graphify query, Haiku runs tests, read every
    stage diff (`git diff stage-R<x-1>-done..stage-R<x>-done`), findings in
    Caveman, then verdict. Tags: `stage-R2-done` = commit `91a53a0` (R2
    closed). If a tag is missing (no remote yet), Gemini still creates it
    locally at the stage gate; diff base = previous stage tag.

### 1.3 Stage gate (all must be true before push)

1.  Every substage row of the stage is committed and its own tests pass.
2.  `.\run_tests.ps1` all green: all CTest targets and all pytest files.
3.  Stage milestone test passes (named per stage below).
4.  Numerical reference tests pass where the stage has math.
5.  GPU/CPU equivalence tests pass (R5 onward, where applicable).
6.  `graphify update .` run; `graphify-out/graph.json` newer than all source.
7.  New dependencies verified on Windows in a proof build BEFORE use.
8.  Docs written for each new module (`docs/<module>.md`, 1 page each).
9.  `PROGRESS.md` updated: rows, gate results, decisions.
10. App starts and the previous milestone flows still work (no regression).

### 1.4 Push rule

-   Today there is **no git remote**. The user must add one once:
    `git remote add origin <url>` and approve the first push.
-   After that Gemini runs, after the stage gate: `git push origin main`
    then `git tag stage-R<x>-done` then `git push origin stage-R<x>-done`.
-   Never force-push. Never push with a red test. Never push `build/`.
-   Claude never pushes.

### 1.5 Failure rules (unchanged, restated)

-   A substage fails → insert `R<x>.<y>-fix` right after it. Fix before
    anything else. Never skip.
-   **Stop rule for batches:** if one substage fails its tests twice,
    Gemini stops the stage, writes the failure into `PROGRESS.md`
    (Blockers), and waits for the user. No stacking new code on a red
    substage.
-   A fix that failed twice is a case for the Opus advisor (see rule 4 in
    CLAUDE.md). Log it in `PROGRESS.md` Decisions.

### 1.5a Role split in every stage

-   Gemini: all product code, tests, builds, graphify, commits, pushes.
-   Claude Sonnet: stage spec pack, review, decisions.
-   Haiku: run tests and report, write `PROGRESS.md`, git chores for Claude.
-   Opus: only the listed decision points (section 8).

### 1.6 Standing technical rules (every stage)

-   GPU budget: 8 GB VRAM hard ceiling. Batch sizes computed from free
    VRAM at run time. Never hard-coded.
-   Benchmarks record hardware and software versions.
-   One active robot per project. N-DOF, practical sizes.
-   New dependency = Windows proof build first, recorded in
    `docs/hardware-audit.md`.
-   C++20, Eigen 3.4, pybind11, PySide6, PyVista. No Pinocchio before
    R4-late (optional validator only).
-   Every schema change bumps `schema_version` and ships a migration + a
    stored old-version fixture test.
-   The app stays runnable after every substage.

---------------------------------------------------------------------

## 2. Stage R3: Import   (BLOCKING review)

Goal: load a robot from URDF (and later CAD) into the URM, show it, edit
it, with mass/inertia data and a clear error report.

Design decision already made (Opus, logged in PROGRESS.md): the URM becomes
a **tree** (schema v2). Joints get `parent_link`, `child_link`, `kinematics`
(`dh` | `origin_axis`), `origin`, `axis`, optional `mimic`; robot gets
`tcp_link`. All kinematics read one **compiled form**. Never convert
origin_axis back to DH. The DH editor refuses origin_axis joints. UI and
IK work on the **active chain** (path from link 0 to `tcp_link`); other
branches are drawn and flagged.

| ID | Module | What is built | Tests / done when |
|----|--------|---------------|-------------------|
| R3.1 | URM schema v2 (DONE, `fadcca7`) | Fields above, v1→v2 migration, `dof()` rule (non-fixed and non-mimic), `q_index_map()`, new `validate()` rules, `docs/urm-schema-v2.md`. Spec: `docs/specs/R3.1-spec.md`. | C++ and Python tests in spec; v1 fixture opens |
| R3.2 | Compiled kinematic model + tree FK | C++ `kin::CompiledModel`: per joint (pre, post, axis, type, parent, child, q_index, mimic source/mult/offset), sorted by child_link, built once per model. FK for tree: `frames[child] = frames[parent] * pre * motion(axis,q_eff) * post`. DH/MDH joints compile to the same form (exact split, see R3 spec). Mimic flattened to its root master at compile time: `q_eff = mult*q[src] + offset`. `fk()` for existing DH robots unchanged in result. Python: `fk()` also returns per-link frames for trees; `active_chain(model)` returns joint indices root→tcp_link. | R2 FK tests still pass bit-for-bit (tol 1e-12); tree test (3 links, 2 branches) vs hand numbers; mimic test; origin_axis revolute/prismatic vs closed form; random-model test: 200 random chains, DH-compiled FK == old FK |
| R3.3 | URDF parser | Python (`app/importers/urdf.py`, stdlib `xml.etree`): `<robot>`, `<link>`, `<joint>` types revolute, continuous (limits ±π, flagged), prismatic, fixed; `<origin xyz rpy>`, `<axis>`, `<limit>`, `<mimic>`, `<parent>/<child>`. Floating and planar joints: rejected with clear error (not supported v1). Xacro: not supported v1, clear error. Link order made topological. Root link must be unique. Output: `ImportResult(model, report, visuals, collisions)`; model built via bindings in canonical order (`joints[i].child_link == i+1`); `tcp_link` = deepest link (user can change later). | Fixtures: serial 6-DOF, branched gripper, mimic joint, continuous joint, bad XML, missing parent, two roots, cycle, floating joint. FK of imported 2R equals hand value |
| R3.4 | UI for tree / origin_axis | Robot tree view (reuses `ModelPanel`), read-only `PropertiesPanel`; DH editor shows origin_axis joints read-only and refuses edits; JointPanel lists active-chain joints plus a collapsed "other branches" group; viewport draws all branches, non-active branches dimmed and labelled; File > Import URDF; asserts DH table cells after project reopen (open item from R2.9). | Offscreen UI tests, import flow end-to-end, branch drawn, DH refusal |
| R3.5 | Meshes and assets | Resolve `package://`, relative paths and absolute paths (search roots list in project settings). Load STL, OBJ, PLY with PyVista; DAE not supported v1 (warning, falls back to primitive). URDF primitives (box, cylinder, sphere) drawn. Visual vs collision geometry kept apart (collision stored, not drawn yet). Mesh `scale`, `origin`. Meshes copied into the project archive (`assets/`) so a saved project is self-contained. | Fixtures with STL, missing mesh (warning, robot still loads), package:// resolve, archive round-trip |
| R3.6 | Import validation + report | One `ImportReport` (errors, warnings, info with link/joint names) shown in a dock; clickable rows select the joint/link; report saved as text; severity rules fixed (error = model unusable, warning = model usable). Re-runs `validate()` after import. | Report content per fixture; UI test |
| R3.7 | Physical properties | Existing `Link.inertia` (mass, COM, tensor; defaults changed to mass 0 = not set), from URDF `<inertial>`. Estimate: uniform density from mesh volume (PyVista), user sets density or total mass. Manual editor table. Validation in new `validate_physical()` (warnings, separate from structural `validate()`): mass ≥ 0, inertia positive definite, triangle inequality. Stored in URM, schema v2, no bump. Total mass and COM readout. | Box/cylinder analytic inertia vs estimate (tol 1%), URDF inertial import, validation cases |
| R3.8 | CAD import + guided joints | Split into four sub-parts; **re-plan before starting** (needs a Windows proof of the CAD library). R3.8a feasibility: STEP reader candidates (pythonocc/OpenCASCADE, cadquery) proof build on Windows, decision logged. R3.8b: STEP assembly → parts tessellated to meshes + part tree. R3.8c: guided joint wizard (pick two parts, pick joint type, axis from cylindrical face or manual, origin) producing `origin_axis` joints. R3.8d: manual mode (edit origin/axis numbers) and automatic mode (detect cylindrical faces as revolute candidates, user confirms). | PoC built; sample STEP assembly of 3 parts becomes a 2-joint robot whose FK matches the expected pose |

**R3 milestone test** (`test_milestone_r3.py`): import a 6-DOF URDF with
meshes and a gripper branch → report has no errors → set TCP link to the flange → viewport draws all
branches → joint panel moves the active chain → TCP readout equals an
independent numpy FK → mass/COM shown → save, close, open, same state.

---------------------------------------------------------------------

## 3. Stage R4: Kinematics and dynamics   (BLOCKING review)

Goal: complete analytic engine on the CPU, correct against references.
All algorithms read the compiled model from R3.2. The **compiled layout is
frozen at the end of R4.1** (flat arrays, topological order) because CUDA
in R5 copies it as is.

| ID | Module | What is built | Tests / done when |
|----|--------|---------------|-------------------|
| R4.1 | Compiled model freeze | Document layout in `docs/compiled-model-layout.md` (field order, types, alignment, SoA arrays). Add `to_arrays()` exporting numpy arrays. Version field in the layout. | Layout round-trip test; doc matches code |
| R4.2 | Jacobian | Geometric Jacobian (6×dof) at TCP and at any frame, for the active chain; handles prismatic, mimic (merged column), fixed. Numeric Jacobian by finite difference as the test oracle. | Analytic vs finite-difference tol 1e-6 on 200 random poses, DH and origin_axis robots; 2R planar closed form |
| R4.3 | Numerical IK | Damped least squares, position-only and full pose, weights, joint-limit handling (clamp + null-space push), step and iteration limits, tolerance report (converged, error, iterations). | Reach known poses (FK of random q → IK → FK error < 1e-6) for 100 cases per robot; unreachable pose reports failure without crash |
| R4.4 | Multiple IK solutions + validation | Multi-start from seeded random configs, dedupe by joint distance, rank by distance to current q; optional closed-form for 2R/3R planar and 6R spherical wrist as cross-check; each solution validated (limits, FK error). | All elbow-up/down solutions found for 2R planar; each returned solution passes FK check |
| R4.5 | Singularity indicators | σ_min, σ_max, condition number, Yoshikawa manipulability, near-singular flag with threshold setting. | 2R planar: singular at q2 = 0 and π; values vs closed form |
| R4.6 | Inverse dynamics | RNEA over the tree using `Link.inertial`; gravity vector setting (default 0,0,−9.81); torque for given (q, qd, qdd). | 2R planar closed-form torques tol 1e-9; 3R spatial vs independent numpy Lagrange implementation; energy consistency check |
| R4.7 | Mass matrix + forward dynamics | CRBA mass matrix M(q), bias C(q,qd), gravity g(q); forward dynamics qdd = M⁻¹(τ − C − g). Joint friction (viscous, Coulomb) optional fields. | M symmetric positive definite; ID(FD(τ)) == τ round trip tol 1e-9; vs independent numpy |
| R4.8 | Energy and torque tools | Kinetic, potential, total energy; torque limits check against `effort` limits; power per joint. | Energy conservation test with zero friction over a short RK4 run (stub integrator in tests only), drift < 1e-6 |
| R4.9 | UI panels | IK panel (target pose entry or drag target in viewport, solution list, apply solution), Jacobian/singularity readout, dynamics readout (torques for current pose, energy). Heavy work runs in `TaskRunner`, UI stays responsive. | Offscreen UI tests |
| R4.10 | R4 milestone | 6-DOF URDF: IK to target, solution applied, singularity flagged near wrist alignment, torques shown, all vs independent numpy. Optional (decision R4-late): Pinocchio via Miniconda as a validator only, never a runtime dependency. | `test_milestone_r4.py` |

---------------------------------------------------------------------

## 4. Stage R5: GPU compute   (precision decision is a blocking point)

Goal: batch FK, workspace sampling, batch Jacobian/IK on the RTX 5060
(8 GB), correct against CPU, with CPU fallback when no GPU.

Pre-condition: update NVIDIA driver (toolkit 13.5 vs driver max 13.1 is a
known risk, see `PROGRESS.md` Risks). R5.1 verifies it first.

| ID | Module | What is built | Tests / done when |
|----|--------|---------------|-------------------|
| R5.1 | GPU Compute Manager | Device query (name, SM, VRAM total/free, driver, toolkit), VRAM budget (max 8 GB, leave headroom), `max_batch(bytes_per_item)` computed from free VRAM, error handling (no GPU, out of memory → smaller batch → CPU fallback). | Unit tests with fake VRAM numbers; real device query on this machine |
| R5.2 | Precision decision + batch FK kernel | **Decision point (Opus)**: consumer GPU FP64 is very slow. Choose fp32 compute with fp64 verification, or fp64 for everything. Kernel: one thread per sample, uniform per-joint math, reads frozen SoA layout. | CPU vs GPU TCP error under tolerance chosen in the decision (record it), 1e6 samples |
| R5.3 | Workspace sampling | Random and grid sampling of q within limits; chunked so VRAM stays under budget; reduce to point cloud (TCP positions) and occupancy voxel grid; progress and cancel via TaskRunner. | Planar 2R reachable annulus area vs analytic within 1%; memory never exceeds budget (logged) |
| R5.4 | Batch Jacobian + batch IK | GPU Jacobian for many q; batch DLS IK for many targets, per-item convergence flags. Only built if R5.2 shows enough speedup; otherwise CPU multi-thread and a note in Decisions. | CPU vs GPU solutions agree (error tolerance recorded); convergence rate report |
| R5.5 | CuPy / Python backend + fallback | Python `compute.get_backend()` returns GPU or CPU backend with the same API; automatic fallback; setting to force CPU. | Same results on both backends; no-GPU path tested by forcing CPU |
| R5.6 | Performance and memory dashboard | Panel with benchmark (samples/s CPU vs GPU), VRAM used, last run settings; benchmark file records hardware and software versions. | Dashboard shows values; benchmark JSON has versions |
| R5.7 | R5 milestone | 1e6-sample workspace of the 6-DOF robot computed on GPU, shown as a point cloud, matches CPU on a 1e4 subset, VRAM log under 8 GB. | `test_milestone_r5.py` (skipped with clear message if no GPU) |

---------------------------------------------------------------------

## 5. Stage R6: Simulation and graphs   (non-blocking review)

Goal: play motion over time, track states, plot everything, export data.

| ID | Module | What is built | Tests / done when |
|----|--------|---------------|-------------------|
| R6.1 | Simulation core | `SimState` (t, q, qd, qdd, τ, TCP), state machine (stopped, running, paused), fixed or variable step, time scale. | State machine tests |
| R6.2 | Integrators | Semi-implicit Euler, RK4, adaptive RK45 (error control); used with forward dynamics from R4.7. | Pendulum vs analytic (small angle) and energy drift per integrator; step-size order test |
| R6.3 | Manual mode | Kinematic mode (sliders move q directly, as in R2) and dynamic mode (apply torques/gravity, robot moves by forward dynamics); mode switch. | Free-fall pendulum test in dynamic mode |
| R6.4 | Trajectory model + playback | Trajectory = list of timed waypoints (q(t)); loader/saver in project; playback with timeline slider, play/pause/seek, speed. Auto mode follows the trajectory (kinematic) or tracks it with computed torque (dynamic). | Seek returns exact interpolated q; playback tests |
| R6.5 | State tracking | Ring buffers for joint and TCP signals (pos, vel, acc, torque, energy, manipulability); memory cap setting. | Buffer wrap tests; no unbounded growth |
| R6.6 | Graph suite | Full set: joint pos/vel/acc/torque vs time, TCP xyz and orientation, TCP path 3D, energy (kinetic, potential, total), manipulability/σ_min, power. Graph library per `TECH_STACK.md`. Cursor linked to timeline. | Graph data tests (series values), UI offscreen test |
| R6.7 | Data export | CSV and NPZ export of any graph and of the whole run; metadata header (robot, time, settings). | Export file re-read equals buffers |
| R6.8 | R6 milestone | Play a trajectory on the 6-DOF robot, all graphs fill, export, re-import numbers check. | `test_milestone_r6.py` |

---------------------------------------------------------------------

## 6. Stage R7: Planning and advanced analysis   (non-blocking review)

| ID | Module | What is built | Tests / done when |
|----|--------|---------------|-------------------|
| R7.1 | Trajectory generation + constraints | Joint-space: trapezoidal velocity, quintic polynomial, S-curve; Cartesian: straight line and arc with IK per step; checks for velocity, acceleration, torque, limit violations with a violation report. | Boundary conditions exact; constraint violations detected |
| R7.2 | Collision geometry (feasibility first) | Proof build of a collision library on Windows (candidate: FCL, or own simple BVH over sphere/capsule/box approximations). Decision logged. Then: self-collision with adjacent-link exclusion (allowed collision matrix) and robot-only environment. | Known colliding / non-colliding poses of a test robot |
| R7.3 | Planner feasibility (OMPL) | Windows proof build of OMPL. If it does not build in reasonable effort, use own C++ planners (RRT-Connect, PRM) behind the same interface. Decision logged. | PoC result recorded |
| R7.4 | Path planning | Planner interface; planners (RRT-Connect, RRT*, PRM; more if OMPL builds); path shortcutting and smoothing; result handed to R7.1 trajectory generation. Cancel and progress via TaskRunner. | Plans in a free space and around a self-collision obstacle; path valid at every sample |
| R7.5 | Workspace, singularity, manipulability analysis | Reachable volume, dexterity and manipulability maps, singularity regions; built on R5 batch compute (CPU fallback). Slice views and 3D clouds. | 2R/3R analytic comparisons |
| R7.6 | Ghost configurations + comparison | Store configurations (named), show translucent ghost robots, compare TCP poses and joint differences; IK solutions shown as ghosts. | Ghost count and pose tests; compare table values |
| R7.7 | R7 milestone | Plan a collision-free motion for the 6-DOF robot between two poses, generate a trajectory, simulate it, show ghosts of start/goal, workspace map. | `test_milestone_r7.py` |

---------------------------------------------------------------------

## 7. Stage R8: Extensibility and release   (non-blocking review, then final full review)

| ID | Module | What is built | Tests / done when |
|----|--------|---------------|-------------------|
| R8.1 | Python scripting API | Stable `robotics_api` module (project, robot, FK/IK, dynamics, simulate, plan, export). Script console dock and "run script" menu. API versioned and documented. Runs in a worker so UI stays alive; cancel supported. | API tests per function; example scripts run |
| R8.2 | Plugin interfaces + permissions | Plugin manifest (name, version, API version, entry point, requested permissions), loader, plugin kinds (importer, planner, analysis panel). Permission prompts (file access, network off by default). Bad plugin cannot crash the app. | Sample plugin loads, bad plugin isolated, permission denied path tested |
| R8.3 | Reports and export | Report generator (HTML and PDF): robot summary, DH table, mass, workspace, graphs, plan results; export meshes and URDF (round-trip with R3.3 importer). | Report content tests; URDF export → import equals model |
| R8.4 | Project migration and recovery | Every project schema version has a migration; automatic backup before migrate; autosave and crash recovery; corrupt-file diagnosis messages. | All stored old-version fixtures open; kill-during-save recovery test |
| R8.5 | Installer and packaging | Packaging tool chosen after a Windows proof build (candidates: PyInstaller or Nuitka, plus an installer maker). Includes CUDA runtime handling and clean-machine test notes. | Installer builds; install on a clean user profile; app starts |
| R8.6 | Documentation | User guide, API reference, developer notes, schema docs. | Docs build with no broken links |
| R8.7 | Reliability and regression | Full regression suite, performance benchmarks vs recorded baselines, memory-leak check on long simulation, error-injection tests. Release checklist signed off. | All suites green; benchmark within 10% of baseline |
| R8.8 | Final full review | Claude reviews R6 to R8 (any unreviewed stage) plus a whole-project pass: requirements in `SRS.md` traced to tests. | Trace table complete; verdict `approved` |

---------------------------------------------------------------------

## 8. Opus decision points (rare, logged in PROGRESS.md "Decisions")

| When | Question |
|------|----------|
| Before R3.1 | Done: tree vs chain (answer: tree, B-lite) |
| Start of R4.6 | RNEA/CRBA formulation for the compiled tree and handling of mimic and fixed joints in dynamics |
| R5.2 | fp32 vs fp64 on this GPU; tolerance values |
| R7.2 / R7.3 | Only if the library proof builds are unclear |
| Any fix that fails twice | Narrow question on that fix |

---------------------------------------------------------------------

## 9. Risks and mitigations

| Risk | Stage | Mitigation |
|------|-------|-----------|
| Index confusion (joint index, dof index, link index) | R3, R4 | One tested `q_index_map`; no raw index math outside it |
| DH ↔ origin_axis conversion errors | R3 | One-way only (DH → compiled); never convert back |
| CAD library will not build on Windows | R3.8 | Proof build first; CAD import can slip without blocking R4 |
| CUDA toolkit/driver mismatch | R5 | Driver update before R5.1; check in GPU Manager |
| FP64 slow on consumer GPU | R5 | Decision at R5.2, verify fp32 against CPU fp64 |
| 8 GB VRAM overflow | R5, R7 | Dynamic batch size from free VRAM, chunked sampling |
| OMPL Windows build | R7 | Fallback own planners behind the same interface |
| Two unreviewed stages stack up | R6 to R8 | Max review lag is one stage |
| Schema change breaks old projects | All | Migration plus stored old fixture in every schema bump |
| Large stage batch hides a bug | All | Per-substage tests and commit; stop rule after 2 failures |

---------------------------------------------------------------------

## 10. Summary timeline (order only, no dates)

R3 (8 modules) → review gate → R4 (10) → review gate → R5 (7) → R6 (8) →
R7 (7) → R8 (8). Push after every stage. Review: blocking after R3 and R4;
non-blocking after R5, R6, R7; final full review at R8.8.

Stage spec packs are written by Claude at the start of each stage (step A
in 1.1) because they depend on the real code at that time. R3 pack:
`docs/specs/R3-spec.md` (R3.2-R3.7; R3.1 done, archived in
`docs/specs/R3.1-spec.md`). R3.8 (CAD) is re-planned as mini-stage `R3b`
after R3.7. R4 pack is written after R3 is `stage-approved`.
