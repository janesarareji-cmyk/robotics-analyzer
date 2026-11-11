# Compiled model layout (frozen, layout_version 1)

Source: `core/kin/compiled_arrays.hpp` / `.cpp`. Python: `cb.compile_arrays(model)`
returns a dict with the same names. `cb.effective_links(model)` is separate.

Frozen: changing any field, order, dtype or shape needs a new `layout_version`
and a review note here. R5 (CUDA) copies these arrays as they are.

## Enum values

`JointType` (from `core/urm/joint.hpp`, stored in `type`):

| Name      | Value |
|-----------|-------|
| Revolute  | 0     |
| Prismatic | 1     |
| Fixed     | 2     |

## Order and meaning

All per-joint arrays are in compiled order: `CompiledModel::joints`, sorted by
`child_link` ascending. Index `k` is the compiled joint `k`, not the URM joint
index. Use `joint_index[k]` to get the URM joint index.

Matrices are row-major 4x4. Element `(r, c)` is at offset `r*4 + c`. The last
row is `0 0 0 1`.

Joint motion: `A = pre * motion(axis, q_eff) * post`,
`q_eff = mult * q[src_q_index] + offset`. Fixed joints have no motion.
Link frame `k` is `frames[parent] * A`. `frames[0] = base`. TCP = `frames[tcp_link] * tool`.

## Fields

`n` = `n_joints`. Byte sizes: int32 = 4, float64 = 8. Numpy dtype in brackets.

| Field          | Shape    | Dtype     | Content                                              |
|----------------|----------|-----------|------------------------------------------------------|
| `n_joints`     | scalar   | int       | number of compiled joints `n`                        |
| `n_links`      | scalar   | int       | number of links                                      |
| `n_dof`        | scalar   | int       | number of DOF (non-fixed, non-mimic joints)          |
| `tcp_link`     | scalar   | int       | resolved TCP link index (never -1)                   |
| `layout_version` | scalar | int       | 1                                                    |
| `joint_index`  | (n,)     | int32     | URM joint index                                      |
| `parent_link`  | (n,)     | int32     | effective parent link                                |
| `child_link`   | (n,)     | int32     | effective child link                                 |
| `type`         | (n,)     | int32     | JointType value (see enum table)                     |
| `q_index`      | (n,)     | int32     | own DOF index, -1 for fixed and mimic joints         |
| `src_q_index`  | (n,)     | int32     | DOF index that drives this joint, -1 for fixed       |
| `pre`          | (n,4,4)  | float64   | transform before motion, row-major                   |
| `post`         | (n,4,4)  | float64   | transform after motion, row-major                    |
| `axis`         | (n,3)    | float64   | unit motion axis, in the frame after `pre`           |
| `mult`         | (n,)     | float64   | mimic multiplier (1 for normal joints)               |
| `offset`       | (n,)     | float64   | mimic offset (0 for normal joints)                   |
| `base`         | (4,4)    | float64   | base pose, row-major                                 |
| `tool`         | (4,4)    | float64   | tool pose, row-major                                 |

Byte offsets inside each array: element `k` of an int32 array starts at
byte `4*k`. Element `k` of a float64 `(n,)` array starts at byte `8*k`.
`axis` element `(k, j)` starts at byte `8*(3*k + j)`. `pre` / `post`
element `(k, r, c)` starts at byte `8*(16*k + 4*r + c)`. `base` / `tool`
element `(r, c)` starts at byte `8*(4*r + c)`.

## Example: 2-joint planar arm

Two revolute DH joints, `a = 1, alpha = 0, d = 0, theta0 = 0`, standard
convention. Link 0 is the base. TCP on link 2. `n_links = 3`, `n_dof = 2`.

Joint 0 (`k = 0`): child link 1. Joint 1 (`k = 1`): child link 2.

| Field         | k = 0                                  | k = 1                                  |
|---------------|----------------------------------------|----------------------------------------|
| `joint_index` | 0                                      | 1                                      |
| `parent_link` | 0                                      | 1                                      |
| `child_link`  | 1                                      | 2                                      |
| `type`        | 0 (Revolute)                           | 0 (Revolute)                           |
| `q_index`     | 0                                      | 1                                      |
| `src_q_index` | 0                                      | 1                                      |
| `pre`         | identity: 1 0 0 0 / 0 1 0 0 / 0 0 1 0 / 0 0 0 1 | same as k = 0                 |
| `post`        | 1 0 0 1 / 0 1 0 0 / 0 0 1 0 / 0 0 0 1 | same as k = 0                         |
| `axis`        | (0, 0, 1)                              | (0, 0, 1)                              |
| `mult`        | 1.0                                    | 1.0                                    |
| `offset`      | 0.0                                    | 0.0                                    |

`base` and `tool` are identity. With `q = (q0, q1)`, the TCP frame is
`Rz(q0) * Tx(1) * Rz(q1) * Tx(1)`.
