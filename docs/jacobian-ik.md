# Jacobian, IK, singularity (R4.2 to R4.5)

Files: `core/kin/jacobian.cpp`, `ik.cpp`, `ik_multi.cpp`, `singularity.cpp`;
Python glue `app/kin_utils.py`; bindings in `bindings/kin_bindings.cpp`.
All functions read the compiled model (`cb.compile(model)`); compile once.

## Jacobian (R4.2)

`cb.jacobian(cm, q, link, point_local=(0,0,0))` returns a numpy `(6, n_dof)`.
Rows 0-2 linear velocity, rows 3-5 angular velocity, both in the WORLD frame,
of the point `point_local` given in the frame of `link`.
`cb.jacobian_tcp(cm, q)`: link `tcp_link`, point = translation of `tool`.
The angular rows are those of `tcp_link` (the tool rotation does not change
the world angular velocity).

Columns are per DOF (`n_dof`), never per joint. For every joint on the path
base to `link` (walk `parent_link`; off-path joints give zero columns):
`F = frames[parent] * pre`, `z = R(F) * axis`, `o = t(F)`, `p` = world point.
Revolute: `v = z x (p - o)`, `w = z`. Prismatic: `v = z`, `w = 0`. Fixed: no
column. `J[:, src_q_index] += mult * [v; w]`.

Mimic merge: a mimic slave on the path adds `mult` times its column to its
master's DOF column (`mult = 1` for normal joints). `offset` never appears.
The Jacobian of the base link is zero. Wrong `q` size or a bad link index
raises `ValueError`.

## IK (R4.3): damped least squares

`cb.solve_ik(cm, lo, hi, target_4x4, q0, options=None)` returns a dict
`q, converged, pos_error, rot_error, iterations, message`. The target is the
TCP of the active chain. Non-convergence is a result, not an exception.

Error: `e_pos = p_target - p_tcp`; `e_rot` = axis-angle vector of
`R_target * R_tcp^T` (angle in [0, pi]). Weights `W` (diagonal, rows x y z rx
ry rz) scale `e` and `J`. Position-only drops rows 3-5.

Step: `dq = Jw^T (Jw Jw^T + lam^2 I)^-1 ew` (LDLT solve, no explicit inverse).
ADAPTIVE damping used in the code: `lam = damping * max(1e-3, min(1, s/0.1))`
with `s` = weighted error norm `sqrt(wp^2 + wr^2)`. Full `damping` while the
error is large, shrinking linearly below 0.1, floor `1e-3 * damping`. This
gives fast final convergence near singular poses.
Then `dq` is scaled so `max|dq| <= max_step`, `q += dq`, and `q` is clamped to
`[lo, hi]` when `enforce_limits`. Null-space push (`null_gain > 0`):
`P * null_gain * (q_mid - q)/range` is added to `dq` before the step limit
(`P = I - Jw^T (Jw Jw^T + lam^2 I)^-1 Jw`).

Stop: `converged` when weighted `|e_pos| < tol_pos` and (position-only or
weighted `|e_rot| < tol_rot`). `"max iterations"` at `max_iter`. `"stalled"`
when the clamped step is below 1e-12 for two iterations in a row. The returned
`q` is the best-error iterate; `pos_error`/`rot_error` are unweighted norms at
that `q`. `q0` or limit vectors of wrong size raise `ValueError`.

Options (dict; an unknown key raises `ValueError`), defaults:

| key | default |
|---|---|
| `position_only` | false |
| `weights` | `[1,1,1,1,1,1]` |
| `damping` | 0.05 |
| `max_iter` | 100 |
| `tol_pos` | 1e-6 m |
| `tol_rot` | 1e-6 rad |
| `max_step` | 0.3 (inf-norm per iteration) |
| `enforce_limits` | true |
| `null_gain` | 0.0 (off) |

Limits come from `app/kin_utils.dof_limits(model)`: the master joint of each
DOF. Mimic joint limits are not enforced in R4.

## Multiple solutions (R4.4)

`cb.solve_ik_multi(cm, lo, hi, target, q_current, options, n_starts=30,
seed=42, dedupe_tol=1e-3)` returns a list of dicts `q, pos_error, rot_error,
distance`.

- Start 0 is `q_current`. Starts 1..n-1 are uniform random in `[lo, hi]`
  (`std::mt19937_64(seed)`): same seed, same list.
- Only converged results are kept.
- Dedupe: two solutions are equal when the inf-norm of the joint difference is
  `< dedupe_tol`. A revolute DOF whose limits span `>= 2 pi` is compared after
  wrapping to `(-pi, pi]`; other DOFs are compared raw.
- Validation: `q` inside limits (tol 1e-9) and the FK error recomputed with
  `forward_kinematics` below the weighted tolerances; failing ones are dropped.
- Sorted ascending by `distance` = inf-norm to `q_current`.
- `n_starts = 1` equals `solve_ik` from `q_current`. `n_starts < 1` raises.

## Singularity (R4.5)

`cb.analyze_singularity(J, mode="full", threshold=1e-3)` returns a dict
`singular_values, sigma_min, sigma_max, condition, manipulability,
near_singular`. Mode `full` uses 6 rows, `position` rows 0-2, `angular` rows
3-5; any other value raises `ValueError`. Columns that are exactly zero
(off-chain DOFs) are dropped before the SVD. With `r = min(rows, nonzero
columns)`: `sigma_min` = smallest of the `r` values, `sigma_max` = largest,
`condition = sigma_max / sigma_min` (`inf` below 1e-300), `manipulability` =
product of the `r` values (Yoshikawa), `near_singular = sigma_min <
threshold`. No nonzero column: all zeros, `near_singular = true`.
Helper: `app/kin_utils.singularity_at(cm, q, mode, threshold)`.

Caveat: `sigma_min` in `full` mode mixes metres and radians; use `position` or
`angular` for a unit-consistent indicator.
