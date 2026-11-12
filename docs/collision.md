# Collision (R7.2, R7.6, R7.7)

Code: `app/planning/collision.py` (`Shape`, `EnvShape`, `CollisionConfig`, `CollisionWorld`,
`fit_mesh`, `shapes_from_geometry`, `shapes_from_project`, `shape_distance`), UI
`app/panels/collision_panel.py`. Library decision: `docs/collision-decision.md` (python-fcl).

## Shapes
Sphere, box, capsule, cylinder (axis = local z). Robot shapes are attached to a link (pose in the
link frame). Meshes are fitted, never used as meshes: oriented box (default), sphere or capsule;
every fit contains all vertices. Robots with no stored collision geometry use their visuals; a link
with no shape is never checked.

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
