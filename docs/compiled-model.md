# Compiled kinematic model

Files: `core/kin/compiled_model.hpp`, `core/kin/compiled_model.cpp`,
`core/kin/fk.hpp`, `core/kin/fk.cpp`. Python: `bindings/kin_bindings.cpp`.
Introduced in R3.2. Schema: `docs/urm-schema-v2.md`.

The URM (`urm::RobotModel`) is the source of truth. The compiled form is a
flat, pre-computed view of it. FK, Jacobians, dynamics and later CUDA code read
only the compiled form, never the URM joint list directly.

## Data structures

`CompiledJoint` (one per URM joint, fixed joints included):

| Field | Meaning |
|-------|---------|
| `joint_index` | index into `RobotModel::joints` |
| `parent_link`, `child_link` | effective link indices (see legacy rule) |
| `type` | `urm::JointType` (Revolute, Prismatic, Fixed) |
| `pre` | Isometry applied before the motion |
| `post` | Isometry applied after the motion |
| `axis` | unit vector, in the frame after `pre` |
| `q_index` | own DOF index; -1 for fixed and mimic joints |
| `src_q_index` | DOF index that drives this joint; -1 for fixed |
| `mult`, `offset` | `q_eff = mult * q[src_q_index] + offset` |

`CompiledModel`:

| Field | Meaning |
|-------|---------|
| `joints` | sorted by `child_link` ascending |
| `n_links` | `links.size()` (1 for an empty model with no joints) |
| `n_dof` | `RobotModel::dof()` |
| `tcp_link` | resolved, never -1 |
| `base`, `tool` | `pose_to_iso` of `model.base` and `model.tool` |

## Joint rule

Every joint compiles to one transform:

    A = pre * motion(axis, q_eff) * post

- Revolute: `motion` = AngleAxis(q_eff, axis).
- Prismatic: `motion` = Translation(axis * q_eff).
- Fixed: `motion` = identity; `q` is not used.

FK (`forward_kinematics(cm, q)`): for each joint in stored order,
`frames[child] = frames[parent] * A`. Then `tcp = frames[tcp_link] * tool`.
`frames[0] = base`. A link with no incoming joint keeps the identity frame
(`validate()` rejects such models before normal use).

## Compile table

`pre` / `post` per joint kind. `Rz`, `Rx` are rotations; `Tz`, `Tx` are
translations. `dh_transform` and `mdh_transform` are the full DH/MDH matrices
from `fk.hpp`.

| Kinematics | Joint | pre | post |
|------------|-------|-----|------|
| DH (Standard) | Fixed | `dh_transform(a, alpha, d, theta)` | I |
| DH (Standard) | Revolute | `Rz(theta)` | `Tz(d) * Tx(a) * Rx(alpha)` |
| DH (Standard) | Prismatic | `Rz(theta) * Tz(d)` | `Tx(a) * Rx(alpha)` |
| DH (Modified) | Fixed | `mdh_transform(a, alpha, d, theta)` | I |
| DH (Modified) | Revolute | `Rx(alpha) * Tx(a) * Rz(theta)` | `Tz(d)` |
| DH (Modified) | Prismatic | `Rx(alpha) * Tx(a) * Rz(theta) * Tz(d)` | I |
| OriginAxis | any | `pose_to_iso(origin)` | I |

The stored `theta` / `d` zero point sits in `pre` (or in `pre`/`post` above),
so `mult` never scales it. OriginAxis joints use `axis` normalised. Zero axis
on a non-fixed joint throws. DH joints keep `axis` = Z.

Convention (Standard or Modified) comes from `RobotModel::convention`, one
value per robot.

## Sort by child_link, and why

`compile()` sorts `joints` by `child_link`. `child_link` values are unique
(duplicates throw) and `parent_link < child_link` is enforced. So a parent
frame is always written before a child frame reads it. Ascending child order is
therefore a valid topological order. The URM joint order is not topological in
general, so the sort is required.

## Mimic flattening

A mimic joint `i` depends on master `m1`, which may depend on `m2`, and so on,
until a non-mimic root `r`. Compile walks the chain once and flattens it:

    A = 1, B = 0
    for each mimic step (k, c) along the chain:
        B = A * c + B
        A = A * k
    mult = A, offset = B, src_q_index = q_index(r)

Check with two steps: `q_i = k1*q_m1 + c1`, `q_m1 = k2*q_r + c2` gives
`q_i = (k1*k2)*q_r + (k1*c2 + c1)`. The loop produces the same result.

Errors: mimic target name not found; mimic cycle (more steps than joints);
chain ends on a Fixed joint.

## q_index and src_q_index

- `q_index`: from `RobotModel::q_index_map()`. Non-fixed, non-mimic joints get
  the next DOF index in URM joint order. Others get -1.
- `src_q_index`: for a normal non-fixed joint, equals its own `q_index`. For a
  mimic joint, equals the root master's `q_index`. Fixed joints get -1.

`q` passed to FK is always indexed by DOF (`n_dof` entries), never by joint
or link index.

## Legacy serial fallback: `kin::effective_links`

Used by `compile()` and `active_chain()`. If there is at least one joint and
every joint has `parent_link == 0` and `child_link == 0`, the model is treated
as R2 serial data. Joint `i` then gets effective `(parent i, child i + 1)`.
This requires `links.size() == joints.size() + 1`, else `std::invalid_argument`.

A mixed model (some 0/0, some not) is not legacy. It fails the normal
`parent < child` rule. The model itself is never modified. Schema v2 never
produces 0/0 joints (child 0 is forbidden), so this path is only for old data.

## tcp_link and active_chain

- `tcp_link == -1` resolves to `n_links - 1`. Other values must be in range,
  else `std::invalid_argument("tcp_link out of range")`.
- `active_chain(model)`: walks from `tcp_link` back to link 0 through effective
  parents. Returns URM joint indices, root to tip, fixed and mimic joints
  included. Empty when `tcp_link == 0`. Throws if `tcp_link` is not reachable.

The UI and IK act on this chain. Other branches are drawn but not part of
the active chain.

## Compile once, reuse

`compile()` does not call `validate()`. Compile once and reuse the result for
many `q`. `forward_kinematics(model)` and `forward_kinematics(model, q)`
compile on every call, so use `forward_kinematics(cm, q)` in loops. R5 relies
on this.

## Errors

`compile()` and `active_chain()` throw `std::invalid_argument` on:
link index out of range; `parent_link >= child_link`; duplicate child link;
mimic target missing, cyclic, or ending on a fixed joint; legacy link count
mismatch; zero axis; `tcp_link` out of range or unreachable.
`forward_kinematics(cm, q)` throws on `q.size() != n_dof`.

pybind11 maps `std::invalid_argument` to Python `ValueError`.

## Python bindings

Module `core_bridge` (built from `bindings/kin_bindings.cpp`):

- `compile(model) -> CompiledModel`. `CompiledModel` exposes read-only
  `n_links`, `n_dof`, `tcp_link`. The joint list is not exposed to Python.
- `fk_compiled(cm, q) -> dict`. Same dict as `fk(model, q)`.
- `fk(model) -> dict` uses `model.get_q()`. `fk(model, q) -> dict`.
- `active_chain(model) -> list[int]`.
- `pose_to_iso(pose) -> 4x4 ndarray`.

FK dict keys:

- `frames`: list of 4x4 arrays, `frames[k]` = pose of link k in world.
- `tcp`: 4x4 array, `frames[tcp_link] * tool`.
- `active_joints`: list of URM joint indices on the active chain, root to tip.

`fk()` calls `compile()` and `active_chain()` on every call. `fk_compiled()`
recomputes `active_joints` by walking `cm.joints`, so it does not need the URM.
