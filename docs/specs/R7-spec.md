# Stage R7 spec pack: Planning and advanced analysis

Authority: `docs/MASTER_PLAN.md` section 6 (workflow, gate, push in section 1).
Review: non-blocking. Builder = Claude subagents launched by Sonnet. Sonnet reviews each substage diff.

Order: R7.1 → R7.2 → R7.3 (STOP after: decision) → R7.4 → R7.5 → R7.6 → R7.7.
Suggested tier: R7.1 Sonnet (numerics); R7.2 Sonnet (proof build + C++); R7.3 Sonnet
(proof build); R7.4 Sonnet; R7.5 Sonnet; R7.6 Haiku/Sonnet (UI glue); R7.7 Sonnet.

## Common rules (every substage)

-   Same as R6 spec "Common rules" (`docs/specs/R6-spec.md` lines 13-36): graphify first,
    independent oracles, tests per rule, commit `R7.<y>: <summary>`, row `committed`,
    UI never runs a long loop (TaskRunner), offscreen Qt tests.
-   Baseline after R6: CTest 9/9, pytest 824. Old tests stay green.
-   Commands: `cmake --build build --config Release`, `ctest --test-dir build -C Release --output-on-failure`,
    `python -m pytest -q`.
-   `app/main_window.py` is cp1252 + CRLF: edit by bytes, keep the encoding.
-   Any modal `QMessageBox` in code under test must be monkeypatched in tests (it hangs offscreen).
-   Spec unclear, or red twice → "Blockers", row `blocked`, stop.
-   New dependency (FCL, OMPL, any library) needs a Windows proof build FIRST, decision
    in `PROGRESS.md` "Decisions" with versions. No proof → use the own-code fallback named in the row.
-   Code-fact: the 8 GB VRAM rule stays; R7 runs on CPU unless a row says GPU.

## Code facts (checked 2026-11-12, after R6)

-   `app/sim/trajectory.py`: `Waypoint`, `Trajectory(waypoints, interpolation, clamped)`,
    `q/qd/qdd(t)`, `limit_warnings`, `to_dict/from_dict`. First waypoint t must be 0.
    `app/sim/playback.py` `TrajectoryPlayer` (kinematic + dynamic computed torque).
-   `app/sim/engine.py` `SimEngine` (`torque_fn`, `set_state`, `step_callbacks`),
    `app/sim/recorder.py` `Recorder` (`SIGNALS`, `export_arrays`), `app/sim/export.py`.
-   Kinematics/dynamics via `core_bridge` (`cb.compile`, `cb.fk`, `cb.jacobian_tcp`,
    `cb.analyze_singularity`, `cb.build_dyn_model`, `cb.inverse_dynamics`, `cb.forward_dynamics`).
    `app/kin_utils.py` `dof_limits`. IK: R4 + CPU batch IK in R5 (`app/compute/backend.py`
    `Backend.solve_ik_batch`, `sample_workspace`).
-   Compiled model (R4.1 frozen layout, `docs/compiled-model-layout.md`) is the only input of
    kinematic code. Link geometry: visuals/collisions from R3.5 (`ImportResult.collisions`,
    stored in the project; collision geometry is stored, not drawn).
-   Graphs: pyqtgraph 0.14 behind `app/graphs/plot_backend.py`. Viewport: `app/viewport/robot_view.py`
    (`set_tcp_path` draws a polyline; workspace cloud actors exist from R5.5).
-   Project format 1.2.0 (`trajectory.json`). A new file/section bumps to 1.3.0 and old files must load.
-   Known R6 gaps to fix on the way: (a) series units for prismatic joints (`app/graphs/series.py`
    `_joint_unit`, `app/sim/export.py` `arrays_from_series`): R7.6 or R7.1 may fix with joint types.
    (b) `RingBuffer.view()` copies the whole buffer each call when wrapped; live graphs at 64 MB
    cap cost a 64 MB copy per 100 ms: fix in R7.5 or earlier if it shows in a benchmark (keep
    a contiguous double-write buffer or return two slices).

## R7.1 --- Trajectory generation and constraint checks

Files: `app/planning/__init__.py`, `app/planning/profiles.py`, `app/planning/cartesian.py`,
`app/planning/checks.py`, tests `test_profiles.py`, `test_cartesian_traj.py`, `test_traj_checks.py`.

1.  Joint-space point-to-point profiles per DOF, synchronised (all DOF finish at the same T):
    `trapezoid(q0, q1, vmax, amax)`, `quintic(q0, q1, T)` (zero v and a at ends),
    `scurve(q0, q1, vmax, amax, jmax)`. Each returns a callable object with `q/qd/qdd(t)`,
    `duration`, and can be sampled into a `Trajectory` (`to_trajectory(dt)` linear or cubic knots).
2.  Cartesian: `line(T0, T1, ...)` straight TCP line (position linear, orientation slerp),
    `arc(center/via point)`; each sample solved by IK (seeded from the previous q, branch continuity:
    joint step limit); failure at a sample → result with `failed_at` index, no exception.
3.  `check_trajectory(traj_or_samples, model, limits)` → `ViolationReport` list of
    (kind, dof, t, value, limit) for: position limits, velocity limit, acceleration limit, torque
    (`effort`, via RNEA with gravity) and optional TCP speed. Report has `ok`, `summary()`.
4.  Limits come from URM joint limits; missing limit = unlimited (info line, not error).

Tests: trapezoid/quintic/scurve: boundary q, qd, qdd exact (1e-12), duration formula vs closed form,
vmax/amax/jmax never exceeded (numeric scan), all DOF end together; quintic matches the closed-form
polynomial; line: TCP error to the ideal line < 1e-6 m at every sample, orientation slerp angle linear;
arc: constant radius to 1e-6; IK failure reported at the right index on an unreachable endpoint;
checks: a profile with amax too high is flagged with the right dof/time, a feasible one passes,
torque limit exceeded on a heavy 2R at gravity-loaded pose (numpy oracle).

Done when: tests green, graphify, commit `R7.1: trajectory generation and checks`.

## R7.2 --- Collision geometry (feasibility first)

Files: `docs/collision-decision.md`, `core/collision/` (if C++), `app/planning/collision.py`, tests.

0.  Proof build on Windows: try FCL (vcpkg or pip `python-fcl` wheel). Time-box one attempt. If it
    imports and a box/sphere/capsule distance query works, use it behind the interface below.
    Otherwise implement own simple checker: shapes sphere, capsule, box (OBB); broad phase AABB
    per link; narrow phase analytic sphere-sphere, sphere-capsule, capsule-capsule, sphere-box,
    OBB-OBB (SAT), capsule-box by sampling or GJK. Write the decision (library, version, reason) in
    `docs/collision-decision.md` and `PROGRESS.md` "Decisions". Mesh geometry: approximate each link
    mesh by its oriented bounding box or a capsule/sphere fit (user choice), never exact mesh-mesh in R7.
1.  Interface (one file): `CollisionWorld(model, shapes)` with `set_q(q)`, `in_collision() -> bool`,
    `pairs() -> list[(linkA, linkB, depth_or_distance)]`, `min_distance()`. Shapes per link built from
    the stored collision geometry (R3.5) or from fit.
2.  Allowed collision matrix: adjacent links (parent-child) excluded by default; user edit list
    saved in the project (format 1.3.0, `collision.json`); "disable pairs always colliding at the
    zero pose" helper. Environment: list of static boxes/spheres/cylinders, robot-only
    collisions against them, saved in the project.
3.  Batch check: `check_batch(q_array)` → bool array (CPU loop is acceptable; target ≥ 20 000
    checks/s on the 6-DOF fixture or record the number).

Tests: known colliding and non-colliding poses of a test robot (hand-computed geometry), adjacent
link exclusion, ACM edit round trip (save/load, old 1.2.0 file still loads), environment box hit /
miss at an analytic distance (tol 1e-6), shape-shape pair queries vs closed forms (sphere-sphere,
sphere-box), speed number recorded.

Done when: tests green, decision recorded, graphify, commit `R7.2: collision geometry`.

## R7.3 --- Planner feasibility (OMPL) --- STOP after this substage

Files: `docs/planner-decision.md`, proof notes.

0.  Windows proof build of OMPL (pip `ompl` wheel, conda, or vcpkg). Time-box one attempt.
    PoC: RRT-Connect plans a 2-DOF path around one obstacle and returns waypoints.
1.  Decision: OMPL available (use it behind the interface of R7.4) or own planners
    (RRT-Connect, PRM, RRT* in Python or C++, same interface). Log in `PROGRESS.md` "Decisions"
    with versions. Do NOT add a dependency to `requirements.txt` without the proof.
2.  Record the PoC result (time, path length) in `docs/planner-decision.md`.

Tests: PoC script/test marked `skipif` OMPL missing. Done when: decision written, commit
`R7.3: planner decision`. Then write nothing more: continue with R7.4 (non-blocking), but if OMPL
needs a user decision (driver, conda), write it in "Blockers" and stop.

## R7.4 --- Path planning

Files: `app/planning/planner.py`, `app/planning/shortcut.py`, `app/panels/planning_panel.py`, tests.

1.  Planner interface: `plan(model, start_q, goal_q, collision, limits, planner="rrt_connect",
    time_limit, seed) -> PlanResult(path, success, time, iterations, reason)`. Goal may also be a
    TCP pose (IK solutions as goals). Planners: RRT-Connect, RRT*, PRM (more if OMPL). Seeded RNG:
    same seed, same path. Cancel flag and progress callback checked every N iterations (TaskRunner).
2.  Edge validity: dense interpolation with step ≤ resolution (default 0.02 rad); joint limits
    respected. Shortcutting (random pair shortcut) and smoothing (path to cubic via R6.4
    `Trajectory`); result handed to R7.1 time parametrisation (`trapezoid`/`quintic` per segment or
    one `Trajectory` through the waypoints with a velocity check) and `check_trajectory`.
3.  Panel: start/goal capture from current q or named configuration, planner combo, time limit,
    plan button (worker), result summary, "Send to trajectory" (fills the R6.4 trajectory editor).

Tests: free space start→goal path valid at every sample (1e-3 step re-check with the collision
world); around a self-collision / environment obstacle on the 2R planar robot (path never in
collision at a 1e-3 check); same seed same path; cancel returns quickly; unreachable goal (inside
obstacle) reports failure, no crash; shortcut never makes the path longer or invalid; panel offscreen.

Done when: tests green, graphify, commit `R7.4: path planning`.

## R7.5 --- Workspace, singularity, manipulability analysis

Files: `app/analysis/workspace_analysis.py`, `app/panels/workspace_panel.py` (extend R5.5),
`docs/workspace-analysis.md`, tests.

1.  Reachable volume from the R5 voxel occupancy (volume = filled voxels × voxel size³, error bound
    stated), dexterity/manipulability map: per voxel mean/min σ_min and Yoshikawa w of the samples
    that landed there (batch Jacobian on CPU; GPU only if R5 backend already exposes it).
2.  Singularity regions: voxels or q-space slices where σ_min < threshold (setting).
3.  Views: 3D cloud coloured by the chosen metric, 2D slice (plane x/y/z at an offset) as a
    pyqtgraph image with colour bar. Heavy work in a worker, cancel supported, VRAM budget unchanged.
4.  Fix known gap (b) in the Code facts if the live graph benchmark shows a cost.

Tests: planar 2R: volume vs analytic annulus area (1%), w = l1 l2 |sin q2| per sample vs closed form,
singular set at q2 = 0, π; 3R planar comparison; slice values equal direct voxel lookup; empty
workspace → clear message.

Done when: tests green, graphify, commit `R7.5: workspace and manipulability analysis`.

## R7.6 --- Ghost configurations and comparison

Files: `app/configs/store.py`, `app/panels/ghost_panel.py`, `app/viewport/robot_view.py` (ghosts),
project format 1.3.0 `configs.json`, tests.

1.  Named configurations `{name: q}` stored in the project (validated n_dof, dropped with a log
    warning when the robot DOF changes, like the trajectory).
2.  Ghosts: translucent copies of the robot at stored q (and IK solutions from the R4.9 list), colour
    per ghost, show/hide, max 8 ghosts, delete. Drawing reuses the viewport robot builder; no
    per-frame loop.
3.  Comparison table: for two or more configurations, TCP position/orientation difference, joint
    differences (wrapped to ±π for revolute), TCP distance, Δ manipulability.
4.  Fix known gap (a) (prismatic units) here if not done.

Tests: store save/load round trip, old 1.2.0 file loads with empty configs, n_dof change drops
them, ghost count == shown configs, ghost TCP == independent numpy FK, comparison values vs numpy,
wrapped joint difference, panel offscreen.

Done when: tests green, graphify, commit `R7.6: ghosts and comparison`.

## R7.7 --- R7 milestone and stage gate

`tests/python/test_milestone_r7.py` (offscreen `MainWindow`, real `core_bridge`):
1.  Import `six_dof_with_gripper.urdf`, TCP link `l6`, add one environment obstacle that blocks the
    straight joint-space line between two stored poses (start, goal).
2.  Plan a collision-free path (seeded), shortcut/smooth, time-parametrise (R7.1), `check_trajectory`
    ok; every sample at 1e-2 step is collision-free (independent check).
3.  Send to trajectory, play back kinematic and dynamic (RMS error limit as R6), recorder has samples.
4.  Ghosts of start and goal shown (count 2, poses equal numpy FK).
5.  Workspace map computed for the robot (reduced samples, e.g. 1e5), σ_min/manipulability slice
    draws; volume within the stated bound of an independent estimate.
6.  Save, close, open: configs, collision settings, trajectory identical.
7.  No running timers/workers after close.

Stage gate = `docs/MASTER_PLAN.md` 1.3. Also: docs one page each `docs/planning.md`
(profiles, Cartesian, checks, planner interface, decisions), `docs/collision.md`,
`docs/workspace-analysis.md` (if not yet), project format 1.3.0 in `docs/trajectory.md`.
Record gate numbers in `PROGRESS.md` "Current". Commit `R7: stage milestone test and docs`, tag
`stage-R7-done` locally, no push. Stage status `stage-ready-for-review`. R8 spec not written: STOP.
