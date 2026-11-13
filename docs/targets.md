# Targets, ranked IK and Play to target (U4a)

Code: `app/target.py` (pose helpers, `Target`, `TargetStore`), `app/viewport/target_widget.py`
(`TargetController`), `app/analysis/ik_rank.py`, `app/target_preview.py`,
`app/analysis/target_motion.py`, `app/panels/target_panel.py`.

## Target pose

A target is a named 4x4 TCP pose in the frame of `cb.fk(...)["tcp"]` (label `"base"`). The list is
saved as the optional archive member `targets.json` (format `{"format": 1, "targets": [...]}`).
The project format version stays 1.4.0: old files have no member (empty list), and the member is
only written when there is a target. Roll, pitch, yaw: `R = Rz(yaw) Ry(pitch) Rx(roll)`.

Rotating in 3D: step buttons in the panel (5 degrees about the target's own axis) and typed
roll / pitch / yaw. Ring handles were not built (heavy in PyVista); the sphere handle drags the position.
"Place on surface" uses the VTK cell picker on a click (not a camera drag): position = hit point,
tool z axis = minus surface normal, x axis keeps the previous x direction. The target is lifted 10 mm
off the surface by default ("Lift off surface" field): a gripper whose fingers reach past the TCP
(panda) would otherwise start in contact and every solution would be invalid.

## Ranking

`rank_solutions(model, target, q_current, collision, weights)` runs `solve_ik_multi` (60 starts) with
the gripper DOFs locked, then scores each solution (0 to 1, 1 = best):

| Part | Definition |
|------|------------|
| motion | `1 - d / d_max`, d = joint distance to the current q |
| limits | smallest distance to a limit divided by half the joint range |
| manip | `sigma_min / max sigma_min` over the solutions |
| clearance | collision clearance clipped at 0.10 m, divided by 0.10 |

Total = weighted sum, default weights 0.4, 0.2, 0.2, 0.2 (normalised; saved in the settings key
`target/rank_weights`). A colliding or out-of-limit solution is invalid and listed last with a
reason. Speed: panda (8 DOF) 25 solutions, about 20 ms. `SINGULAR_WARN = 0.05` (`app/kin_utils.py`)
is the sigma_min below which a valid solution is "near singular" (orange target cone); the curve
check and overlays of the next stages use the same constant.

## Play to target

`plan_target_motion` builds one trajectory: joint path to the pre-pose (straight when free, else the
RRT-Connect planner with TOPP timing, collision world = U3 shapes, gripper locked), then optionally
the straight Cartesian approach line (IK per 5 mm, every sample collision checked) and its
reverse as retreat. The last waypoint of the approach is exactly the chosen solution. The motion is
sent to the timeline (kinematic mode) and played at once; playback blocking of U3.4 applies.
Dense samples use linear interpolation at 0.04 s.
