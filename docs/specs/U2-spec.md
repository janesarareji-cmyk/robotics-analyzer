# U2 spec pack: usability fixes, interaction features, validation gaps

Stage U2 follows U1/U1-fix (2026-11-13). User request: "do everything other than CAD".
Builders: Claude subagents, one substage at a time, Sonnet (orchestrator) reviews each diff.

## Common rules

- Product code: `app/`, `robotics_api/`, `tests/`, `tools/`. No C++/CUDA change unless a substage says so.
- `app/main_window.py` is NOT valid UTF-8 (cp1252 bytes). The Edit tool refuses it: edit it with a
  small Python script that reads/writes bytes (latin-1) and keeps everything else byte-identical.
- Run `graphify query "<area>"` before reading code. Reuse existing code; do not duplicate.
- Per substage: new tests, touched test files green, then the FULL suite
  `python -X faulthandler -m pytest tests/python -q -p no:cacheprovider` (about 5 min, timeout 600000),
  `python -m app.main --smoke` exit 0, `graphify update .`, then commit `U2.<n>: <summary>` staging only
  the files of that substage plus `graphify-out/graph.json` (never `tests/python/fixtures/robot_model_v1.json`
  or other `graphify-out/*`), message ending with the line
  `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`. Never push.
- Two failed fix attempts on the same test: stop and report.
- Baseline after U1-fix: 1320 passed, 5 skipped, exit 0.
- Sample robots: `robots/<name>/<name>.urdf` (kuka_iiwa, panda, ur3, ur5, ur10, xarm5, xarm5_gripper,
  xarm6, xarm7), listed by `app/sample_robots.list_sample_robots()`.
- Docs: add/adjust a short section in `docs/user-guide.md` (and the topic doc if one exists);
  `python tools/check_docs.py` must stay OK.

## U2.1 Small fixes (sonnet)

1. Import Report dock pops up only when the report has ERRORS. Warnings/info: report is filled,
   the dock stays as it is; the status bar says "Imported with N warnings (View > Import Report)".
2. Start pose after import / sample load: per DOF q0 = 0 when 0 is inside [lo, hi], else the middle
   of the range. Applied where the joint panel / project gets its first q after an URDF import (find
   the clamp that turned Panda joint4 0 into a limit value). Saved projects keep their saved q.
3. Frame triads: when a model with visual meshes is loaded, View > Robot > "Show frames" defaults
   to OFF (TCP triad stays). Models without visuals keep frames ON. User toggles still work.
4. Project / robot name: when a URDF is imported and the URDF robot name is a generic one
   (`UF_ROBOT`, empty, `robot`) or when loaded via Sample Robot, use the sample title (SOURCE.txt line 1)
   for a sample, else the file stem. Do not change the URDF importer's model data otherwise.
Tests: `tests/python/test_u2_small.py`.

## U2.2 PRM cancel latency (sonnet)

Tests `test_milestone_a1.py::test_e_...prm_cancel` and `test_audit_numerics.py::test_m7_prm_cancel_in_blocked_space`
sometimes measure 0.51 s against a 0.5 s limit under load. Find the PRM loop parts that run long
without checking `is_cancelled` (roadmap connection / k-nearest / collision batch) and check cancel
inside them so worst-case latency is well below 0.5 s (target < 0.15 s on this machine, measured in the
test, print it). Do NOT relax the test limits.
Tests: extend `test_audit_numerics.py` with a latency measurement under a 4-thread CPU load helper
(threads spinning numpy) asserting < 0.3 s.

## U2.3 Floor grid + drag-and-drop URDF (sonnet)

1. Viewport floor: a grid plane at z = base z (world frame), size from robot reach (estimate from FK
   bounding box of link origins at q0, at least 1 m, rounded up to 0.5 m), 0.1 m minor cells,
   subtle colour that works on the dark background, below the robot (no z-fighting with base mesh:
   offset -1 mm). Plus world axes at origin (short). View > Robot > "Show floor" toggle (default ON),
   saved in AppSettings. Updates when a new robot loads. No effect on picking/camera reset other than
   being included in the fit.
2. Drag and drop: the main window accepts dropped `.urdf` files (one file) -> `import_urdf_file(path)`;
   `.robotproj` -> open project via the controller's open flow (same unsaved-changes prompt).
   Other files: status bar message, ignored.
Tests: `tests/python/test_u2_floor_drop.py` (floor actor exists/visible/toggle/rebuild on new robot;
simulated QDropEvent with QMimeData urls for urdf, robotproj, txt).

## U2.4 Gripper open/close (sonnet)

Gripper detection (in a new `app/kin/gripper.py` or `app/gripper.py`): a DOF is a gripper DOF when
its joint name matches `drive_joint|finger|gripper` (case-insensitive) and it is not a mimic joint.
Must hold for samples:
xarm5_gripper -> `drive_joint` (open = lower limit 0, closed = upper limit 0.85), panda ->
`panda_finger_joint1` (open = upper 0.04, closed = lower 0). For revolute gripper DOFs with lo == 0,
"open" = 0; for prismatic finger DOFs, "open" = upper. Provide `gripper_dofs(model) -> list[GripperDof(dof_index, name, open_value, closed_value)]`.
UI: Joints panel shows a "Gripper" group (only when gripper DOFs exist) with Open / Close buttons
and an opening slider 0-100 % mapping linearly open->closed; buttons move the DOF(s) and update the
viewport like a slider move. Arm-only robots: no group.
Tests: `tests/python/test_gripper.py` (detection on all 9 samples: only xarm5_gripper and panda have
gripper DOFs; Open/Close set q; FK of finger link moves).

## U2.5 Teach waypoints (sonnet)

Joints panel (or Simulation panel trajectory section; pick where the trajectory editor already lives
- read `app/sim/trajectory.py`, sim panel, planning panel `trajectory_ready`): button "Add current pose
as waypoint", "Clear waypoints", waypoint count label, "Play" which builds the existing cubic
`Trajectory` through the taught waypoints (time per segment from max joint distance / default joint
speed, min 0.5 s) and starts kinematic playback through the existing playback path. Waypoints are stored
in the project trajectory (existing format, no format bump unless needed; if needed bump
1.4.0 -> 1.5.0 with a no-op migration like F1.4). Undo last waypoint button.
Tests: `tests/python/test_teach.py` (add 3 poses, play, final q == last waypoint, save/open round trip).

## U2.6 Drag TCP in 3D (sonnet)

Interactive TCP target in the viewport: View > Robot > "Drag TCP" (toolbar button, shortcut T).
When on: a draggable handle at the TCP (PyVista `add_sphere_widget` or a box/plane widget; pick the
one that works with pyvistaqt here; position-only drag is the minimum, orientation via the IK panel's
roll/pitch/yaw is kept). On drag, solve IK (existing CPU IK, seed = current q, position + current
orientation; fall back to position-only if orientation cannot be met, or if the robot has < 6 arm
DOF, e.g. xArm 5) and update the robot live at most ~30 Hz (throttle; drop intermediate events).
Unreachable target: robot stays at the last good solution, handle turns red, status bar says
"Target not reachable". Release: final q committed (project modified, joint sliders updated, undo not
required). Mimic/gripper DOFs are not changed by the IK.
Must not block the GUI: IK for one target is fast (< 5 ms typical) so it can run on the GUI thread
with the throttle; measure and print in test.
Tests: `tests/python/test_drag_tcp.py` (call the drag callback directly with a target 5 cm from the TCP
on ur5 and xarm5: TCP reaches target within 1e-4 m; unreachable target 5 m away: q unchanged, flag set;
throttle drops events).

## U2.7 Robot comparison (sonnet)

New dock "Compare" (side bar label "Compare"): pick 2..N robots from the sample list and/or loaded
URDF files; table with: DOF, total mass, reach (max TCP distance from base over workspace samples),
workspace volume (existing `analyze_workspace` CPU, same samples/voxel for all, default 2e4 samples,
h 0.05 m), max gravity torque per joint at the stretched-out pose (q where reach is max among samples)
and the joint effort limit, payload capacity estimate at that pose (max extra point mass at TCP such
that every |tau| <= effort limit; "n/a" when limits missing), joint limit ranges. Runs in the
TaskRunner (cancellable, progress). Export table CSV. Reuse existing workspace/dynamics code; no new
dependency.
Tests: `tests/python/test_compare.py` (ur3 vs ur5 vs ur10: reach order ur3 < ur5 < ur10 and close
to the known nominal reaches 0.50/0.85/1.30 m within 10 %; payload > 0; CSV round trip).

## U2.8 Pinocchio validator via conda-forge (sonnet)

`pin` has no Windows pip wheel. Use micromamba (official `mamba-org` release exe, placed in
`tools/.pinenv/` which is git-ignored) to create an env with `pinocchio` from conda-forge and
Python 3.12. A script `tools/pinocchio_validate.py` (runs inside that env, needs only pinocchio + numpy)
reads a URDF, random q / qd / qdd (seeded), computes FK of every link, frame Jacobian of the TCP link,
RNEA, CRBA mass matrix, gravity, and writes JSON. The app side `tools/validate_against_pinocchio.py`
(main env) runs the same q through `core_bridge` (compile/FK/Jacobian/RNEA/mass matrix used by the app),
calls the env's python on the script via subprocess, compares, and writes
`docs/validation/pinocchio-report.md` (per robot: max abs/rel errors; tolerance 1e-9 rel for dynamics,
1e-10 m for FK). Run it on all 9 sample robots + existing fixtures (DH robots via URDF export).
Mapping notes: Pinocchio orders joints by tree; map by joint name. Mimic joints: Pinocchio may not
support mimic from URDF -> for robots with mimic joints, validate the arm with mimic joints treated as
fixed at 0 on both sides (document). Base/world fixed joints are fine.
Test `tests/python/test_pinocchio_validation.py`: skip with a clear reason when the env is missing;
when present, run on ur5 and panda and assert tolerances.
If micromamba or the conda-forge solve fails twice: STOP and report (do not substitute another library
without the orchestrator).

## U2.9 TOPP with per-waypoint velocity (sonnet; read F1.3 notes in PROGRESS.md first)

F1.3 output goes through the cubic `Trajectory` without per-waypoint velocity, so a time-scaled cubic
through (q(s_i), t_i) rings at acceleration jumps and needs a rescale loop: result 8-13 % slower than
the solver optimum in bang-bang cases. Fix: add an optional per-waypoint velocity to `Trajectory`
(piecewise cubic Hermite: positions + velocities at knots) used when `TimedPath` comes from TOPP
(velocities = dq/ds(s_i) * sdot_i). Keep existing cubic-spline behaviour and file format for other
trajectories; if the Hermite form must be saved, add an optional member (project format bump
1.4.0 -> 1.5.0 if U2.5 has not already bumped it, else reuse, with no-op migration). Re-run the F1.3
benchmarks: planar heavy 2R, double integrator, six_dof 40 N m path; target: TOPP trajectory duration
within 3 % of the solver optimum and no limit violations at dense check (dt 1e-3).
Tests: extend `tests/python/test_topp.py`; keep F1 milestone green.

## Stage end (U2.10, orchestrator)

Real-window screenshots of: floor + frames off, gripper open/closed, drag TCP, compare dock,
teach play. Full suite green, PROGRESS.md updated, commit per substage `U2.<n>: <summary>`.
