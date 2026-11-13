# U3 spec pack: real collision

Plan: `docs/plans/U3-U6-plan.md`. Workflow: `.agents/rules/dev-workflow.md` (Sonnet builders,
one commit per substage, full tests once at stage end, no stage lock).

Goal: the robot never passes through itself, the floor or objects while the user moves it
(sliders, Drag TCP, IK, Teach, playback), using shapes that follow the real link meshes.

## Common rules (all U-stages U3-U6)

- Product code: `app/`, `robotics_api/`, `tests/`, `tools/`. C++/CUDA only where a substage says so.
- `app/main_window.py` is NOT valid UTF-8 (cp1252 bytes). Edit it with a small Python script that
  reads/writes bytes (latin-1) and keeps every other byte identical. The Edit tool refuses it.
- `graphify query "<area>"` before reading code. Reuse existing code; do not duplicate.
- Per substage: write its tests, run only those test files, commit `U3.<n>: <summary>` staging only
  the substage files (never `tests/python/fixtures/robot_model_v1.json`, never `graphify-out/*`
  except `graph.json` at stage end). Message ends with
  `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`.
- Stage end (U3.9): full suite `python -X faulthandler -m pytest tests/python -q -p no:cacheprovider`
  (timeout 600000), `ctest --test-dir build -C Release` if C++ changed, `python -m app.main --smoke`,
  `python tools/check_docs.py`, `graphify update .`, PROGRESS.md, tag `stage-U3-done`, push main + tag.
- Baseline: 1368 passed, 5 skipped, exit 0.
- Sample robots: `robots/<name>/<name>.urdf` (9). All have `<collision>` meshes.
- Docs: short section in `docs/user-guide.md` and the topic doc (`docs/collision.md`).
- Two failed fix attempts on the same stage test: stop, write Blockers, wait for the user.

## Code facts

- `app/planning/collision.py`: `Shape(kind, params, link, pose)` kinds sphere/box/capsule/cylinder;
  `Shape.to_fcl()`; `fit_mesh(points, mode)` PCA box / other fits; `shapes_from_geometry(items, fit_mode)`;
  `shapes_from_project(project)` (collisions, else visuals); `EnvShape` (world sphere/box/cylinder,
  saved in project `collision.json`); `CollisionConfig` (ACM edits, `environment`, `fit_mode`);
  `CollisionWorld(model, shapes, config)`: `set_q`, `in_collision`, `pairs` -> (linkA, linkB, -depth),
  env id `-1-k`, `min_distance`, `check_batch`, `disable_always_colliding(q)` (zero pose only).
  Broad phase = bounding spheres in numpy, narrow phase = fcl.collide per pair.
- Used today only by planner (`app/planning/planner.py`), shortcut, TOPP, Collision panel,
  Planning panel. NOT by `JointPanel` (`q_changed` -> `MainWindow._on_joint_q_changed` main_window.py:944),
  `TcpDragController` (`app/viewport/tcp_drag.py`, `q_changed`/`committed`), Teach
  (`timeline_panel.teach_*`), `TrajectoryPlayer` (`app/sim/playback.py`).
- q helpers: `app/kin_utils.py` `current_q`, `write_q`, `dof_limits`, `CompiledCache`.
- Gripper DOFs: `app/gripper.py gripper_dofs(model)`.
- Floor: `RobotView._build_floor` at base z - 1 mm.

## U3.1 Convex hull link shapes (sonnet)

1. New shape kind `"convex"`: params `{"vertices": (n,3) array, "faces": (m,3) int array}` built in
   new `app/planning/hull.py` from the mesh points. scipy is NOT installed and no new dependency is
   allowed: use VTK through PyVista (`pv.PolyData(points).delaunay_3d().extract_surface().triangulate()`)
   which gives the convex hull. Simplify hulls to <= 64 vertices (keep extreme points by direction
   sampling over ~200 directions, then hull again) so fcl stays fast. Facts checked 2026-11-14:
   `fcl.Convex` and `fcl.Halfspace` exist in python-fcl 0.7.0.11.
2. `to_fcl()` for convex: `fcl.Convex(vertices, n_faces, faces_flat)`; `bounding_radius` = max vertex norm.
3. `fit_mesh(..., mode="hull")` and make `"hull"` the default `fit_mode` for new projects and imports.
   Old projects with `fit_mode` saved keep it. Collision panel fit combo gets "Convex hull".
4. Cache hulls per mesh path + scale (dict on module level, cleared on robot change).
Tests `tests/python/test_hull.py`: hull contains all mesh points (tol 1e-9); vertex cap; fcl sphere
vs hull distance against a known cube mesh; all 9 sample robots build a `CollisionWorld` with hulls.

## U3.2 Auto allowed-collision matrix (sonnet)

`CollisionWorld.auto_acm(n_samples=2000, seed=0) -> dict` (or a free function in `collision.py`):
sample q uniformly in limits (gripper DOFs included), per robot-robot link pair count collisions.
Disable pairs: adjacent links (already), pairs colliding in >= 95 % of samples ("always"), pairs
colliding in 0 samples AND whose links are separated by >= 3 joints along the tree path ("never").
Keep user ACM edits (explicit edits win). Store result in `CollisionConfig` with a `source: "auto"`
marker so the panel shows "auto" vs "user". Run automatically on URDF import / sample load in a
`TaskRunner` worker (progress, cancel); result saved in the project.
Must hold: at each robot's start pose q0 no self-collision is reported for all 9 samples; xarm5_gripper
fingers vs gripper base not flagged; a hand-made folded pose of ur5 (elbow folded back into the base,
build it in the test) IS flagged.
Tests `tests/python/test_auto_acm.py`. Time: < 10 s per robot at 2000 samples (print it).

## U3.3 Floor + live collision service (sonnet)

1. Floor as an environment half-space: fcl `Halfspace` (normal +z, offset base z) or a large thin
   box, link 0 (base) excluded. Toggle in Collision panel "Floor collision" (default ON), saved.
2. New `app/collision_service.py` `CollisionService(QObject)`: owns one `CollisionWorld` for the
   current project (rebuilt on robot / shapes / ACM / environment change), `check(q) -> CheckResult`
   (`ok: bool`, `pairs: list[(a,b,depth)]`, `names: list[str]`), `first_free(q_from, q_to, steps)`
   returns the last collision-free q on the straight joint segment (binary search, 1e-3 rad tol).
   Signals `collision_changed(CheckResult)`. Flag `enabled` (user "Allow pass-through" = off).
3. Speed: `check(q)` median < 2 ms on xarm6 and panda (print median and p95). If slower: cache
   fk, skip fcl for pairs whose bounding spheres do not overlap (already), and use `fcl.collide`
   with `enable_contact=False` for the yes/no path.
Tests `tests/python/test_collision_service.py`.

## U3.4 Live blocking in all motion inputs (sonnet)

Route every q change through `CollisionService` when enabled:
- `JointPanel` slider / spinbox edits: if the new q collides, write `first_free(old, new)` instead,
  move the slider back to it, status bar "Blocked: <linkA> touches <linkB>".
- `TcpDragController`: after IK, check; if colliding, keep the last free q and colour the handle red
  with status "Blocked by collision".
- IK panel "Apply", Ghost "apply", Planning "send to robot", Teach Add/Play, `TrajectoryPlayer`
  (kinematic and dynamic): during playback a colliding sample stops playback at the last free sample
  with status "Playback stopped: collision at t = ... s".
- Gripper close: closing stops at first contact (finger vs object or finger vs finger hull).
- Toolbar toggle "Collision on/off" (shield icon text "Collision") = `CollisionService.enabled`,
  View menu entry, saved in AppSettings. Off = old pass-through behaviour.
Tests `tests/python/test_live_blocking.py`: for ur5 and panda, slider move into a known self
collision ends at a free q close to contact (distance to contact < 2e-3 rad along the move);
drag TCP into the floor stops above the floor; playback through an obstacle stops before it.

## U3.5 Red highlight of colliding links (sonnet)

`RobotView.set_collision_highlight(links: set[int])`: colliding link visual meshes (or link lines
when no meshes) turn red (#ff3030) with full opacity; others back to normal. Driven by
`CollisionService.collision_changed`. Also env objects touched turn red. Status bar text from names.
Tests in `test_live_blocking.py` (actor colour property check).

## U3.6 Objects in the scene (sonnet)

Toolbar "Add object" menu: Box / Cylinder / Sphere (default sizes 0.1 m, placed 0.4 m in front of the
robot on the floor). Objects are `EnvShape`s (existing project save). Each object gets a 3D actor and a
PyVista box/transform widget to drag it (position; rotation via typed RPY in the Collision panel list).
Select object in the Collision panel list -> edit name, size, xyz, rpy, delete. Collision world
rebuilds on change (debounced 100 ms).
Tests `tests/python/test_scene_objects.py` (add, move, save/reopen, collides with robot).

## U3.7 Grasp (sonnet)

When the gripper closes and at least one finger hull touches an object (or the object is between
fingers within 5 mm), the object becomes attached: it is removed from `environment` and stored as an
attached shape on the TCP link (pose relative to TCP frozen at attach time); it moves with the arm,
collides with the environment and robot links (not with the gripper links). Opening the gripper
releases it: object goes back to `environment` at its current world pose (no gravity drop: keep pose;
note in docs). Status bar "Grasped <name>" / "Released <name>". Saved in the project
(`attached_to: <link name>`). Works for xarm5_gripper and panda.
Tests `tests/python/test_grasp.py`.

## U3.8 Second robot (optional, sonnet) - do last, skip if time > budget

File -> "Add second robot..." places another URDF at a user base pose (dialog: sample list or file,
xyz/yaw). Second robot: own joint sliders (Joints panel tab "Robot 2"), drawn in the viewport,
collision robot-vs-robot both ways through `CollisionService`. Not used by planner/TOPP/analysis
(documented). Saved in the project (format bump, migration no-op step).
Tests `tests/python/test_second_robot.py`. If this substage would need > 1 day of work, write
"U3.8 deferred" in PROGRESS.md and continue.

## U3.9 Stage end

Milestone `tests/python/test_milestone_u3.py`: load ur5 -> auto ACM -> add box -> jog into box blocked
-> drag TCP into floor blocked -> plan around the box (existing planner, hull shapes) -> play -> no
collision in a dense recheck -> grasp a box with xarm5_gripper, move, release -> save/reopen equal.
Full gate as in Common rules. Real-window screenshots (`scratchpad shots` style script) of: red
highlight, blocked status, object + grasp; GUI skill check of those screenshots (UI Design Review
`ui-review`, Design `design-critique`) when the skills are loaded; fixes go to U3-fix.
