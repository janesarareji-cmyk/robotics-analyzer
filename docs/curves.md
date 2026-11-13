# Curves, follow and live matrices (U4b)

Code: `app/planning/cartesian.py` (line, arc, spline), `app/planning/path_check.py` (check + tube
colours), `app/planning/curve.py` (legs, chain, timing), `app/panels/curve_panel.py` (Curve dock),
`app/viewport/curve_widget.py` (via point handles), `app/viewport/robot_view.py` (`set_path_tube`).

## Spline path

`spline(model, poses, q_seed, n_per_seg=30, ...)`: position is a centripetal Catmull-Rom curve
through start, via points and end (C1 at the via points, every via point is an exact sample, collinear
points stay on their line). Orientation slerps between the key poses by the fraction of the segment;
`via_poses(T0, T1, positions)` gives via points the orientation of the interpolation. Times follow the
arc length (`profile="quintic"` makes it rest to rest). Same `CartesianPath` result as `line` / `arc`.
`line`, `arc` and `spline` take `limits=(lo, hi)` so a caller can lock joints (the gripper).

## Path check

`check_path(model, cpath, collision)` returns a `PathCheck` with one status per sample:

| status | when |
|--------|------|
| bad (red) | IK did not solve it or the TCP is more than 0.1 mm off, the robot collides with itself, the floor or an object (contacts that already exist at sample 0 and do not get deeper are allowed), or the joint solution jumps more than 0.5 rad (branch flip). Samples behind the first unreachable one are bad too. |
| warn (orange) | `sigma_min` of the TCP Jacobian below `SINGULAR_WARN` (0.05), or collision clearance below 10 mm (or half the start clearance when the robot already starts tighter) |
| ok (green) | everything else |

`problem_text()` names the first bad sample and its time. Play and Generate stay off while any
sample is bad; the tooltip shows that text.

## Tube

`RobotView.set_path_tube(points, colors, radius, via_points)`: tube radius is 0.4 % of the robot reach,
a green sphere at the start, a white one at the end, yellow spheres at the via points,
`set_path_marker(point)` a pink marker (the current sample while playing). Colour modes: problems,
TCP speed, collision clearance, singularity closeness (`tube_colors`).

## Curve dock

Start is always the live arm pose. Type Line, Arc (one via point, default pushed up from the middle)
or Spline (any number of via points). Via points: "Add via point" (middle of the longest part), drag
the yellow sphere in 3D, or type X Y Z. Every change re-runs path + check after 150 ms in a worker
thread (own collision world, like the Target dock). "Chain" visits the ticked targets in list order;
each curve starts where the previous one ended and the cycle time is the sum. Timing: constant TCP
speed (mm/s) or time-optimal (TOPP with the joint speed, acceleration and torque limits of F1.3).
"Generate" puts the timed trajectory into the timeline, "Play" also plays it (kinematic mode, collision
blocking of U3.4 stays on). Via points are kept per target for the session, they are not saved yet.

## Follow and timeline (U4b.4)

"Play" uses the normal `TrajectoryPlayer` (kinematic) with the collision blocking of U3.4. A time-optimal
curve is sampled every 0.04 s into a linear trajectory, so the timeline plays exactly what Generate
made. While it plays: a pink marker runs along the tube (also when you scrub the timeline slider), a
purple ghost shows the end pose, the tube and ghost stay when you switch to the Simulation tab. The
status bar shows "Motion finished" at the end. Speed 0.1x to 4x, scrub, Pause and Stop are the
timeline's own. Editing the curve drops the generated trajectory, marker and ghost.
Fix: Play from the Target and Curve docks now keeps the clock running while the Simulation dock is hidden.

## Matrices dock (U4b.5)

`app/analysis/matrices.py` (`compute_matrices`, `export_text`) and `app/panels/matrix_panel.py`. The
dock updates at most 30 times a second on any q change (joint slider, 3D drag, IK Apply, playback,
scrub); a hidden dock computes nothing. It shows the 4 x 4 tool pose `T_base_tcp` (and `T_world_tcp` when
the base pose is not the identity), the chosen joint's `T_parent_child` and cumulative `T_base_link`, the
joint values (degrees, prismatic in mm), the Jacobian 6 x N (world frame, toggle), the manipulability w
and sigma_min with a bar that turns red below `SINGULAR_WARN`. Translation units mm or m, 4 decimals.
"Pause updates" freezes the numbers (changing the joint then shows that joint at the frozen q).
Copy as text, numpy (`np.array(...)` assignments that run as Python) or LaTeX (`bmatrix`); the copied
translation uses the units chosen on the panel. One refresh costs about 1 ms (panda, Jacobian shown).
Not done: the short flash of changed values (optional in the spec).
