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
`app/planning/hull.py` from the mesh points (VTK Delaunay through PyVista, no scipy). At most 64
vertices: a bigger hull is rebuilt from support points in fewer directions and inflated about its
centroid so every mesh point stays inside (conservative). A flat or collinear mesh falls back to the
oriented box. `fit_mode "hull"` is the default for new projects and imports; a saved `collision.json`
keeps its own `fit_mode`. Hulls are cached per mesh path + scale (+ mtime), cleared when a project
opens.

## Automatic ACM (U3.2)
`CollisionWorld.auto_acm(n_samples=2000, seed=0, q0=None)` samples `q` uniformly inside the joint
limits (all DOFs, gripper included) and counts collisions per link pair. Disabled: pairs colliding in
>= 95 % of samples ("always"), pairs that never collide and are >= 3 joints apart on the tree
("never"), and pairs colliding at the start pose ("start"; hulls of neighbouring meshes overlap
there). User edits always win. The result is stored in `CollisionConfig.auto` (+ `auto_info`),
saved as `acm_auto` in `collision.json`, and shown in the Collision dock with source `auto (reason)`
next to `user`. It runs in a worker after every URDF import / sample load (`app/auto_acm.py`), and on
the dock button "Recompute automatic matrix". About 0.1 to 0.4 s per sample robot at 2000 samples.

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
