# U4a spec pack: 3D target, ranked IK, preview, play to target

Plan: `docs/plans/U3-U6-plan.md`. Common rules: `docs/specs/U3-spec.md` "Common rules" (commit
prefix `U4a.<n>`, tag `stage-U4a-done`). Needs U3 (CollisionService) merged.

Goal: the user puts a target in 3D, sees all IK solutions ranked, picks one (ghost preview),
and plays a collision-checked motion to it.

## Code facts

- IK: `cb.solve_ik(cm, lo, hi, target4x4, q_seed, options)` and
  `cb.solve_ik_multi(cm, lo, hi, target, q_current, options, n_starts=30, seed=42, dedupe_tol=1e-3)`
  -> solutions deduped, validated, sorted by distance to q_current (`core/kin/ik_multi.hpp`).
  Used by `app/panels/ik_panel.py:264`, `app/panels/ghost_panel.py add_ik_solutions`,
  `app/planning/planner.py:417`.
- Singularity: `app/kin_utils.py singularity_at(cm, q, mode, threshold)`; manipulability /
  sigma_min also in `app/analysis/workspace_analysis.sample_metrics`.
- Viewport: `RobotView.set_target_pose(iso)` (triad), `add_ghost(key, model, fk_result, color, ...)`,
  `remove_ghost`, `clear_ghosts`. Drag widget pattern: `app/viewport/tcp_drag.py` (sphere widget,
  30 Hz throttle, plain methods for tests).
- Planner: `app/planning/planner.plan(model, start, goal, world, (lo,hi), cancel, progress, **opts)`,
  `shortcut.time_parametrize(..., method="topp")`. Playback: `TrajectoryPlayer`, timeline panel.
- CollisionService (U3.3) for clearance and blocking.

## U4a.1 Target object + 3D handle (sonnet)

`app/target.py`: `Target(name, pose4x4, frame="base")` list in the project (`targets.json`, optional,
project format bump with no-op migration). `app/viewport/target_widget.py TargetController`:
- A target frame (triad + small cone tool shape) with drag handles: translate by dragging the origin
  sphere (as TCP drag), rotate by three ring handles (one per axis; drag angle around the axis) or,
  if rings are too heavy in PyVista, by keyboard while hovered: R/P/Y keys + mouse wheel, 5 deg steps
  (document the choice).
- "Place on surface": click mode, `plotter.pick_mouse_position` / cell pick on visuals, floor and
  objects; target z axis = -surface normal (tool points into the surface), position = hit point.
- Typed entry: X Y Z (mm) and roll/pitch/yaw (deg) fields in the new Target panel (U4a.4).
- New target default = current TCP pose. Saved targets combo.
Plain methods for tests (`move_to(xyz)`, `rotate(axis, deg)`, `place_at(point, normal)`).
Tests `tests/python/test_target.py`.

## U4a.2 Ranked IK solutions (sonnet)

`app/analysis/ik_rank.py rank_solutions(model, target, q_current, collision_service, weights=None)`:
calls `solve_ik_multi` (n_starts 60, position+orientation; position-only for < 6 arm DOF with a note).
Per solution score parts (each normalised to 0..1, 1 = best):
- motion: 1 - (weighted joint distance / max distance among solutions)
- limit margin: min over DOFs of distance to nearest limit / half range
- manipulability: sigma_min / max sigma_min among solutions
- clearance: CollisionService min distance clipped at 0.10 m / 0.10 (colliding -> solution invalid)
Total = weighted sum (default weights 0.4, 0.2, 0.2, 0.2, editable in a small settings popup,
saved in AppSettings). Return list of `RankedSolution(q, total, parts, valid, reason)` best first;
invalid (collision, limit) at the end with reason. Runs in TaskRunner (cancel); < 300 ms for 7 DOF
(print it).
Tests `tests/python/test_ik_rank.py` (ur5 target with 8 analytic-like solutions: elbow up/down both
found; order changes when weights change; colliding solution marked invalid).

## U4a.3 Ghost preview (sonnet)

Selecting a ranked solution shows a ghost arm (green #66bb6a valid, red #ff5252 invalid, 0.35
opacity) via `RobotView.add_ghost`. "Show all" shows up to 8 ghosts with the selected one highlighted.
Target triad colour: green reachable, red no valid solution, orange valid but sigma_min below the
singularity threshold (shared constant `SINGULAR_WARN` in `app/kin_utils.py`, used by U4b and U6).
Tests in `test_ik_rank.py` / `tests/python/test_target_ui.py`.

## U4a.4 Target panel + Play to target (sonnet)

New dock "Target" (Move/Target mode in U5): target list (add from TCP, add by click, delete,
rename), typed pose fields, "Solve" button, ranked table (rank, total score bar, motion, limits,
manip., clearance, valid/reason), "Preview", "Play to target":
- Motion = planner joint-space path from current q to chosen q (collision-free with U3 shapes;
  straight line used when already free), TOPP timing, sent to the timeline and played at once.
- Approach / retreat: checkbox + distance (default 100 mm) along tool -z: play to the pre-pose first,
  then a straight Cartesian line (`app/planning/cartesian.line`) to the target; retreat = reverse.
- Collision checked all the way (U3.4 playback blocking).
Tests `tests/python/test_target_ui.py` (end q equals chosen solution 1e-9; TCP at target 1e-4 m;
approach line straight within 1e-4 m).

## U4a.5 Stage end

Milestone `tests/python/test_milestone_u4a.py`: panda, place target on a box top by click, rank,
pick 2nd solution, play with approach, dense recheck no collision, TCP error < 1e-4 m, save/reopen
targets equal. Full gate, real-window screenshots (target handle, ranked table, ghosts), GUI skill
check; fixes go to U4a-fix.
