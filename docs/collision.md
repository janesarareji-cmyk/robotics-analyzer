# Collision (R7.2, R7.6, R7.7)

Code: `app/planning/collision.py` (`Shape`, `EnvShape`, `CollisionConfig`, `CollisionWorld`,
`fit_mesh`, `shapes_from_geometry`, `shapes_from_project`, `shape_distance`), UI
`app/panels/collision_panel.py`. Library decision: `docs/collision-decision.md` (python-fcl).

## Shapes
Sphere, box, capsule, cylinder (axis = local z). Robot shapes are attached to a link (pose in the
link frame). Meshes are fitted, never used as meshes: oriented box (default), sphere or capsule;
every fit contains all vertices. Robots with no stored collision geometry use their visuals; a link
with no shape is never checked.

## Convex hulls (U3.1)
Kind `convex`: `params {"vertices": (n,3), "faces": (m,3)}`, outward triangles, built by
`app/planning/hull.py` from the mesh points with an own incremental hull in numpy (no scipy; VTK
`delaunay_3d` was tried and rejected: on cylinder-like meshes its surface is not convex). At most 64
vertices: a bigger hull is rebuilt from the 64 most important points (greedy farthest point) and
its vertices are pushed outward along their normals until every mesh point is inside again
(conservative). A flat or collinear mesh falls back to the oriented box. `fit_mode "hull"` is the
default for new projects and imports; a saved `collision.json` keeps its own `fit_mode`. Hulls are
cached per mesh path + scale (+ mtime), cleared when a project opens.

## Automatic ACM (U3.2)
`CollisionWorld.auto_acm(n_samples=2000, seed=0, q0=None)` samples `q` uniformly inside the joint
limits (all DOFs, gripper included) and counts collisions per link pair. Disabled: pairs colliding in
>= 95 % of samples ("always"), pairs that never collide and are >= 3 joints apart on the tree
("never"), and pairs colliding at the start pose ("start"; hulls of neighbouring meshes overlap
there). User edits always win. The result is stored in `CollisionConfig.auto` (+ `auto_info`),
saved as `acm_auto` in `collision.json`, and shown in the Collision dock with source `auto (reason)`
next to `user`. It runs in a worker after every URDF import / sample load (`app/auto_acm.py`), and on
the dock button "Recompute automatic matrix". About 0.1 to 0.4 s per sample robot at 2000 samples.

## Floor and live service (U3.3)
`CollisionConfig.floor` (default on, saved as `floor`, old files load with it on) adds a floor: a big
thin box whose top is 1 mm under the base (same plane as the viewport grid). Links that never move
(fixed to the base) are not checked against it. Pair id of the floor: `FLOOR_ID` (-1000000). A
`CollisionWorld` built without a config has no floor (library default); the project paths (service,
planning panel, Collision dock) use the project config, floor on. The scripting API planner keeps
its old behaviour (no floor).

`app/collision_service.py` `CollisionService(QObject)` owns one world per project, rebuilt lazily
after robot / shapes / ACM / environment changes. `check(q) -> CheckResult` (`ok`, `pairs`, `names`),
`first_free(q_from, q_to)` (last free q on the joint segment, 16 samples then bisection to 1e-3),
signal `collision_changed` (only when the colliding pair set changes), `enabled = False` = pass-through.
A start that already touches something (UR zero pose: wrist under the base plane) may move on while
no new contact pair appears. Speed (median per check, random poses): xarm6 0.05 ms, panda 0.07 ms.

## Live blocking (U3.4)
`app/motion_guard.py` `MotionGuard` sits between every motion input and the model q. `filter(q_new)`
returns the last free q on the joint segment from the current q (`CollisionService.first_free`) and a
`blocked` flag, status text "Blocked: A touches B"; `reject_jump(q_new)` refuses a whole far target.
Wired inputs: joint slider / spinbox (the slider goes back to the last free value), Drag TCP (handle
turns red, "Blocked: ..."), gripper open/close slider (closing stops at the first contact, finger vs
object or finger vs finger hull), IK Apply (a colliding solution is not applied; preview is
cancelled), Teach Add (refuses a pose that touches the robot itself or an object; floor-only contact
is allowed), trajectory playback (kinematic and dynamic): `TrajectoryPlayer.collision_check` stops at
the last free sample, kinematic mode bisects the time to the contact; status "Playback stopped:
collision at t = ... s". Toolbar and Robot menu action "Collision" (saved in AppSettings
`collision/enabled`, default on); off = old pass-through. Not guarded: Zero all / Reset joint buttons,
sim state replay when seeking, Ghost panel (it has no apply-to-robot action) and Planning (it plans
a trajectory; playing it is guarded).

## Allowed collision matrix (ACM)
Default: parent-child link pairs are not checked. `CollisionConfig.edits[(a, b)]` = `True` (force
check) or `False` (never check) overrides. `CollisionWorld.disable_always_colliding()` (panel button
"zero pose") disables the pairs that collide at q = 0 (e.g. the two fitted gripper fingers).

## Environment
Static `EnvShape`s in the world frame (sphere, box, cylinder) with `xyz`, `rpy`, `name`. Panel:
add/remove shape, ACM edit list. Environment shape k has pair id `-1 - k`.

## Queries
`w.check_batch(q_array)` (bool per row, about 34 700 checks/s on 6 DOF), `w.set_q(q)`,
`w.in_collision()`, `w.pairs()`, `w.min_distance()` (signed, negative = penetration).

## Project format 1.3.0
New optional members: `collision.json` (ACM edits, environment, fit mode; written only when not
default) and `configs.json` (named joint poses). Files 1.0.x-1.2.x load with `collision = None`
and an empty config store. Details: `docs/trajectory.md`.
