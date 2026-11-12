# Collision library decision (R7.2)

Date: 2026-11-12. Platform: Windows 11, Python 3.13.15, numpy 2.5.4.

## Decision

Use **python-fcl 0.7.0.11** (FCL bindings, prebuilt Windows wheel, `pip install python-fcl`)
behind `app/planning/collision.py` (`CollisionWorld`). Added to `requirements.txt`.

## Proof build (one attempt, time-boxed)

-   `pip install --only-binary=:all: python-fcl` installed a wheel at once (no compiler, no vcpkg).
-   `import fcl` works. Sphere-box distance 3 - 0.5 - 1 = 1.5 returned exactly. Sphere inside
    capsule returned -1.0 (FCL gives a sentinel, not a depth, from `distance` on overlap).
-   Raw `fcl.distance` speed about 790 000 calls/s (sphere-box).

## Consequences and limits

-   Overlap depth: `distance()` is not used for overlaps. `collide()` with contacts gives
    `penetration_depth`; `CollisionWorld` reports signed distance (negative = minus depth).
    Box-box depth checked at 1e-6 in `tests/python/test_collision.py`.
-   Shapes: sphere, box, capsule, cylinder (FCL exact cylinder; no capsule approximation needed).
-   Meshes are never used as meshes: `fit_mesh` gives a PCA oriented box (default), a sphere or
    a capsule. All fits contain every vertex (conservative). PCA box is near-tight, not minimal.
-   Own broad phase: numpy bounding-sphere test per shape pair before any FCL call.
-   If `fcl` is missing, `CollisionConfig` and project load still work; `CollisionWorld`
    raises `RuntimeError("python-fcl is not installed ...")`. No own-code fallback was written
    (not needed: proof passed).
-   Pair ids: link index >= 0; environment shape k is `-1 - k` (`env_id`).

## Speed (R7.2, `check_batch`, CPU, Python loop)

6-DOF random `origin_axis` robot, 6 capsules (non-adjacent pairs checked): about 34 700
checks/s (2000 random q, one machine, Ryzen 9 8940HX). Target 20 000 met. Cost is dominated by
`cb.fk_compiled` (all link frames, one call per q) and the FCL transform updates.

## Project format

1.2.0 -> 1.3.0. New optional member `collision.json` (ACM edits, environment, mesh fit mode).
Written only when the config is non-default. Older files load with `collision = None`.
