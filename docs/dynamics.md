# Rigid-body dynamics (R4.6 to R4.8)

Files: `core/dyn/dynamics.hpp/.cpp`, `bindings/dyn_bindings.cpp`,
`app/dyn_utils.py`. Math from the Opus consult (see `PROGRESS.md` Decisions).
Notation: Featherstone. Motion vector `v = [w; v]` (angular first), force
`f = [n; f]`. Everything is in the link frame `k` (the frame AFTER `post`).

## Model

`dm = cb.build_dyn_model(model, cm)`; built once per model, not per q.
Per compiled joint: `parent_link, child_link, type, src_q_index, mult, offset,
pre, post, axis` and the constant motion subspace `S` (6-vector). With
`post = [Rp pp]` and joint axis `a`:
- revolute: `S = [Rp^T a ; Rp^T (a x pp)]`
- prismatic: `S = [0 ; Rp^T a]`
- fixed: `S = 0`

Reason: in the frame after the motion, the axis is unchanged by its own
rotation, so `S = [a; 0]` there; `post` maps that frame to the link frame.
Per link: spatial inertia at the link origin from `Link::inertia` (mass `m`,
COM `c`, tensor `Ic` about the COM from `ixx ixy ixz iyy iyz izz`):
`I = [Ic + m [c][c]^T , m [c] ; m [c]^T , m 1]`. Mass 0 is allowed.

## RNEA (inverse dynamics)

`X_k = ^kX_lambda` is built from `A_k^-1`, `A_k = pre * motion(qe) * post`,
`qe = mult*q[src] + offset`; `qde = mult*qd[src]`, `qdde = mult*qdd[src]`
(`offset` never enters the derivatives; fixed joint: `qde = qdde = 0`).

Gravity is the base acceleration: `v_0 = 0`, `a_0 = [0 ; -R_base^T g]`, `g` =
WORLD gravity vector (default `(0, 0, -9.81)`). Forward pass (index order is
topological because `parent < child`):
- `v_k = X_k v_lambda + S qde`
- `a_k = X_k a_lambda + S qdde + v_k x_m (S qde)`
- `f_k = I a_k + v_k x_f (I v_k)`

Backward pass `k = n-1..1`: for non-fixed joints `tau[src_k] += mult_k * S^T
f_k`; always (fixed joints too) `f_lambda += X_k^T f_k`. Branches need nothing
special. The result is the ACTUATOR torque that produces `qdd`. `tau` is
indexed by DOF (`src_q_index`), so a mimic slave adds `mult` times its torque
to its master's DOF.

Sign convention: gravity torque `g(q) = +dV/dq` with `V = -sum m g . p_com`.
It is the torque the actuators must apply to hold the pose. Helpers:
`gravity_torque = RNEA(q,0,0,g)`, `bias_torque = RNEA(q,qd,0,g)` (includes
gravity), `coriolis_torque = bias - gravity` (this is `C(q,qd) qd`).

## Mass matrix (CRBA)

`Ic_k = I_k`; for `k = n-1..1`: `Ic_lambda += X_k^T Ic_k X_k` (through fixed
joints too). For each non-fixed joint `i` (`si = src_q_index`): `F = Ic_i S_i`;
`M[si,si] += mult_i^2 S_i^T F`; then walk up the parents: `F = X_j^T F`, and
for each non-fixed ancestor `j`: `v = mult_i mult_j S_j^T F`,
`M[si,sj] += v`, `M[sj,si] += v`. Sibling branches stay zero. The result equals
`J^T M_full J` for mimic joints (a mimic on its master's chain adds `2v` on the
diagonal, which is correct).

## Forward dynamics

`qdd = M^-1 (tau - tau_fric - b)`, `b` = bias torque (with gravity). LLT
solve. If `M` is not positive definite: `RuntimeError` ("massless link or DOF
with no inertia"). The optional `armature` (per DOF) is added to the diagonal
of `M`, which makes a massless leaf solvable.

## Friction

`cb.Friction` with `viscous`, `coulomb` (per DOF) and `eps` (default 1e-3):
`tau_fric = viscous*qd + coulomb*tanh(qd/eps)`. Applied per DOF, never per
mimic joint. Added in `inverse_dynamics`, subtracted in `forward_dynamics`. It
is NOT part of `g` or `C`. Friction is not stored in the URM in R4 (call
argument only).

## Energy and tools (R4.8)

- `kinetic_energy = 0.5 qd^T M qd` (equals the spatial hook
  `cb.dyn_kinetic_energy_spatial`).
- `potential_energy = -sum m_k g . p_com,k`, zero reference at the world origin.
- `total_energy = KE + PE`. `cb.joint_power(tau, qd)` = elementwise product.
- `app/dyn_utils.check_torque_limits(model, tau)`: per DOF
  `{dof, joint_name, tau, limit, ratio, exceeded, no_limit}`; limit =
  `joint.limits.effort` of the master joint; `ratio = |tau|/limit`,
  `exceeded = ratio > 1`. Limit <= 0: `no_limit`, `exceeded` false, ratio `inf`
  (0 for `tau = 0`). Mimic slaves are not listed.
- `integrate_rk4_forward_dynamics(dm, q0, qd0, tau_fn, dt, steps, gravity)`:
  STUB fixed-step RK4 on `forward_dynamics`, for tests and the energy check.
  It is NOT a simulation engine (no friction argument, no limits, no
  contacts). The real engine is R6.

## Errors and hooks

Wrong sizes: `ValueError`. Mass matrix not positive definite: `RuntimeError`.
`cb.dyn_frames(dm, q)` (test hook) returns the world frames from the dynamics
transforms; it equals the `cb.fk` frames.
