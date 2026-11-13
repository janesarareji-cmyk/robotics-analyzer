# Plan U3-U6: real collision, target + curve motion, usability, analysis overlays

Agreed with the user on 2026-11-14. This is what we build next, in this order.
Each stage gets its own spec pack `docs/specs/<stage>-spec.md`, written before the
stage starts. Workflow rules: `.agents/rules/dev-workflow.md`.

## Goal

1. Robots act like real ones: links never pass through each other, the floor or objects.
2. The user sets a target in 3D, picks the best IK solution, plays the motion, then
   makes a curve from the current pose and the arm follows it while the transform
   matrices update live.
3. Singularity analysis visible in 3D (red zones, toggle).
4. GUI easy to understand, checked with the installed GUI skills.

## Facts found before planning (2026-11-14)

- Collision (`app/planning/collision.py`, `CollisionWorld`, python-fcl) is used only by
  the planner, TOPP and the Collision panel. Joint sliders, Drag TCP
  (`app/viewport/tcp_drag.py`), Teach and playback never check collision.
- Link shapes are fitted boxes (`fit_mesh`, PCA OBB), not the real mesh.
- All 9 sample robots in `robots/` have `<collision>` meshes in their URDF.
- Planning goal is a typed joint vector only (`app/panels/planning_panel.py`); no 3D target.
- `RobotView.set_target_pose` (IK triad) and `RobotView.set_tcp_path` (path line from
  the Graphs panel) exist in `app/viewport/robot_view.py`.
- Workspace analysis (R7.5, `app/analysis/workspace_analysis.py`) already computes
  sigma_min / manipulability per voxel; shown only as a 2D slice.
- Joint limits alone cannot stop self-collision; only the collision check can.

## U3: Real collision

- Link shapes: convex hull of each URDF collision mesh (FCL Convex), not a box.
- Self-collision on by default. Auto allowed-collision matrix: sample random poses,
  skip pairs always in contact (neighbours) or never in contact.
- Floor collision: arm cannot go below the floor grid.
- Objects: add box / cylinder from the toolbar, drag in 3D; robot collides with them.
- Live blocking in joint sliders, Drag TCP, IK apply, Teach and playback: stop at the
  last collision-free pose. Colliding links red, status bar names the pair.
  Toggle "Allow pass-through" turns blocking off.
- Grasp: closed gripper on an object attaches it (moves with the gripper); open drops it.
- Optional second robot in the scene with robot-vs-robot collision.
- Planner uses the same shapes as the live check.
- Test target: one collision check < 2 ms (30 Hz drag stays smooth).

## U4a: Target and IK

- Set target: drag a target frame with move/rotate handles, click a surface, type
  XYZ + RPY, or pick a saved target.
- IK solutions ranked by: joint distance from current q, joint-limit margin,
  manipulability, collision clearance. Best first, scores shown.
- Click a solution: ghost arm preview, green OK / red bad (collision, limit, singular).
- Play: arm moves to the chosen solution, collision checked all the way.
- Approach / retreat offset along tool axis (e.g. 10 cm).

## U4b: Curve and live matrix

- Curve from the current pose to the target: line, arc (via point) or spline through
  via points dragged in 3D.
- Curve drawn as a 3D tube; colour by speed, collision clearance or singularity closeness.
- Check before Play: unreachable, colliding or near-singular segments shown red.
- Arm follows the curve with IK at every step. Timeline Play / Pause / scrub.
- Live matrix panel (30 Hz, pause and copy): 4x4 tool pose, transform of a chosen joint,
  joint angles, Jacobian and manipulability.
- Several targets in a row (A -> B -> C) with cycle time.

## U5: Usability (installed skills: UI Design Review, Design)

1. Measure: script 6 core tasks (load robot, jog, set target + reach, plan around an
   object, simulate, compare robots); count clicks; real-window screenshots.
2. Skills review the screenshots: UI Design Review (fix list, most important first);
   Design: design-critique, ux-copy (labels, messages), accessibility-review
   (contrast, 150 % Windows scaling).
3. Redesign: mode bar Model | Move | Target | Plan | Simulate | Analyze (each shows
   only its panels); start screen; toolbar icon + text; Joints panel in degrees with
   limit bars; Cartesian jog buttons; view cube; link name on hover; F1 help;
   undo/redo; real version in the title (not v0.1.0).
4. Measure again. Target: load robot and reach a target in <= 4 clicks.
5. Skill screenshot check also at every stage end (U3, U4a, U4b, U6).
   Skills are web-oriented: they judge look and flow; Qt behaviour is tested by pytest.

## U6: Analysis overlays and playback

View -> Analysis overlays (each a toggle, off by default):
- Reach map: reachable / dexterous regions.
- Singularity zones: red = singular, orange = warning; threshold slider; split by kind
  (wrist, elbow, shoulder) where separable; GPU, cached until robot/tool changes;
  CSV export. Same threshold as the joint panel sigma_min bar, the target ghost
  warning (U4a) and red curve segments (U4b).
- Torque / speed overlay: live bars at the joints during motion.
Also: timeline scrub bar under the viewport; video / GIF export of playback;
screenshot button.

## Build rules

- Sonnet builders, one commit per substage `<stage>.<n>: <summary>`.
- Full tests once at stage end, then `graphify update .`, tag `stage-<x>-done`, push.
- Opus reviews each stage spec pack and the stage result. GUI skills check screenshots.
- No stage lock; review findings become `<x>-fix` substages. Stop after 2 failed fix attempts.

## Out of scope

CAD import, task builder, base placement helper, robot code export, scene import,
keep-out zones, speed override, tool library, planner benchmark, measure tool,
MuJoCo physics mode.
