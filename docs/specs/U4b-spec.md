# U4b spec pack: curve from the current pose, follow it, live matrix panel

Plan: `docs/plans/U3-U6-plan.md`. Common rules: `docs/specs/U3-spec.md` "Common rules" (prefix
`U4b.<n>`, tag `stage-U4b-done`). Needs U3 + U4a.

Goal: from where the arm is now, the user makes a curve to a target (line, arc, spline with
draggable via points), sees it as a 3D tube with problem segments in red, plays it on a timeline,
and watches the transform matrices change live.

## Code facts

- `app/planning/cartesian.py`: `line(model, T0, T1, q_seed, n, duration, profile, ik_options,
  max_joint_step, cm)` and `arc(model, T0, via, T1, ...)` -> `CartesianPath(t, targets, q,
  failed_at, message)`, `.to_trajectory()`. `rot_exp`, `rot_log`, `arc_geometry`.
- `RobotView.set_tcp_path(points)` draws a polyline (used by Graphs `path_3d_changed`).
- Trajectory: `app/sim/trajectory.py` (`Trajectory`, `Waypoint(t, q, qd=None)`, Hermite since U2.9);
  `TrajectoryPlayer` (`seek`, `play`, `pause`, `advance`); `TimelinePanel`.
- TOPP: `shortcut.time_parametrize(..., method="topp")`, `app/planning/topp.py`.
- `SINGULAR_WARN` (U4a.3), `CollisionService` (U3.3), targets (U4a.1).

## U4b.1 Spline Cartesian path (sonnet)

`cartesian.spline(model, poses: list[4x4], q_seed, n_per_seg=30, ...)`: position = centripetal
Catmull-Rom (or natural cubic) through start, via points, end; orientation = slerp between
consecutive key poses (via points take the orientation of the interpolation unless the user set one).
Same `CartesianPath` result. Tests `tests/python/test_cartesian_spline.py` (passes through every
via point 1e-9; C1 continuous position; straight when via points are collinear).

## U4b.2 Path check + coloured tube (sonnet)

`app/planning/path_check.py check_path(model, cpath, collision_service) -> PathCheck` per sample:
`reachable` (IK solved, error < 1e-4 m), `colliding`, `sigma_min`, `clearance`, `joint_jump`
(> max_joint_step = branch flip). Segment status: ok / warn (sigma_min < SINGULAR_WARN or clearance
< 10 mm) / bad (unreachable, colliding, joint jump).
`RobotView.set_path_tube(points, scalars, mode)`: tube radius = 0.4 % of robot reach, colour modes
"status" (green/orange/red), "speed" (after timing), "clearance", "singularity"; small spheres at
via points; start/end markers. Replaces the plain polyline for this feature (Graphs path stays).
"Play" is disabled while any segment is bad, tooltip says why and where (sample index, time).
Tests `tests/python/test_path_check.py` (line through the robot base -> unreachable/colliding segment
found; ur5 line through the wrist singularity -> warn segment found).

## U4b.3 Curve editor (sonnet)

Panel "Curve" (in Target dock as a second tab): start = current pose (always the live arm pose),
end = selected target (U4a). Type: Line / Arc (one via point) / Spline (N via points).
Via points: add (midpoint of the longest segment), delete, drag in 3D with sphere widgets (as TCP
drag, 30 Hz throttle), typed xyz. Each change re-runs path + check (debounced 150 ms, worker) and
redraws the tube. "Timing": constant TCP speed (mm/s field) or time-optimal (TOPP with torque limits).
"Generate" -> trajectory into the timeline. Several targets: "Chain" mode A -> B -> C (curves joined,
each from the end of the previous), shows total cycle time.
Tests `tests/python/test_curve_editor.py`.

## U4b.4 Follow + timeline (sonnet)

Playing uses the existing `TrajectoryPlayer` (kinematic) with collision blocking from U3.4. Timeline:
Play / Pause / Stop / speed (0.1x-2x) / scrub slider (seek), current time / total. During play the
ghost of the end pose stays visible, the tube shows a moving marker at the current sample.
Tests: final TCP within 1e-4 m of target; scrub to t gives the same q as `traj.sample(t)`.

## U4b.5 Live matrix panel (sonnet)

Dock "Matrices": updates at <= 30 Hz on any q change (slider, drag, playback):
- 4x4 tool pose T_base_tcp (and T_world_tcp if base pose not identity), 4 decimals, units mm/m toggle.
- Selected joint transform: combo of joints; shows T_parent_child for that joint at current q and
  the cumulative T_base_link.
- Joint values (deg / mm), Jacobian 6xN (toggle), manipulability w and sigma_min with a coloured bar
  (red below SINGULAR_WARN).
- Values that changed since last frame flash subtly (200 ms) - optional, skip if it costs > 1 ms.
- "Pause updates", "Copy as text / numpy / LaTeX" buttons.
Cost: refresh < 2 ms (print). Tests `tests/python/test_matrix_panel.py` (matches cb FK at random q
1e-12; Jacobian matches `cb` Jacobian; copy formats parse back).

## U4b.6 Stage end

Milestone `tests/python/test_milestone_u4b.py`: ur5 + box, target behind box, spline with 2 via points
around the box, check all ok, TOPP timing, play, dense recheck (no collision, TCP on the curve within
1e-3 m), matrix panel T equals FK at 5 sampled times, singular line case blocked with warn segments.
Full gate, screenshots (tube colours, via handles, matrix panel during play), GUI skill check.
