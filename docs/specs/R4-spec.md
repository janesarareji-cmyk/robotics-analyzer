# Stage R4 spec pack: Kinematics and dynamics (CPU)

Authority: `docs/MASTER_PLAN.md` section 3 (workflow, gate, push in section 1).
Stage review is BLOCKING (do not start R5 before `stage-approved`).
Note 2026-11-11: "Builder" = Claude subagents launched by Sonnet (see
`.agents/rules/dev-workflow.md`). Sonnet reviews each substage diff before
the next one starts.

Order: R4.1 → R4.2 → R4.3 → R4.4 → R4.5 → R4.6 → R4.7 → R4.8 → R4.9 → R4.10.
Start point: the first row of R4.1..R4.10 in `PROGRESS.md` not `committed`.
Suggested Builder tier (orchestrator may change it): R4.1, R4.5, R4.8 Haiku;
R4.2, R4.3, R4.4, R4.6, R4.7, R4.9, R4.10 Sonnet.

## Common rules (every substage)

-   Start with `graphify update .` and `graphify query` for the area. Reuse
    existing classes. No duplicate files.
-   Every numbered rule below has at least one test. Tests that check math
    use an INDEPENDENT oracle (see "Test oracles") not the code under test.
-   After each substage: build if C++ changed, `ctest` + `pytest` green,
    `graphify update .`, commit `R4.<y>: <summary>`, row `committed` in
    `PROGRESS.md`. No wait for review between substages (Sonnet reviews the
    diff of each one before the next is launched).
-   Old tests stay green (baseline after R3: CTest 6/6, pytest 302).
-   Commands: `cmake --build build --config Release` (copies
    `core_bridge*.pyd` to the repo root), then
    `ctest --test-dir build -C Release --output-on-failure` and
    `python -m pytest -q`. (`run_tests.ps1` is blocked by PowerShell
    execution policy: do not change the policy.)
-   Spec unclear or contradicts the code → do NOT guess. Write the question
    in `PROGRESS.md` → "Blockers", set the row `blocked`, stop.
-   Same substage red after two fix attempts → stop, same procedure.
-   Dynamics math (R4.6, R4.7) comes from an Opus consult (see Decisions in
    `PROGRESS.md`). Implement it exactly. Do not "simplify" a formula.
-   C++ only for numerics (`core/`), Python only for UI and glue.
    New C++ files go into `core/CMakeLists.txt`; new C++ tests into
    `tests/cpp/CMakeLists.txt`.

## Code facts (checked 2026-11-11, after R3)

-   `kin::CompiledModel` (`core/kin/compiled_model.hpp`): `joints`
    (sorted by `child_link`), `n_links`, `n_dof`, `tcp_link`, `base`,
    `tool`. `kin::CompiledJoint`: `joint_index, parent_link, child_link,
    type, pre, post, axis, q_index, src_q_index, mult, offset`. Rule:
    `A = pre · motion(axis, q_eff) · post`, `q_eff = mult*q[src_q_index] +
    offset`. `frames[child] = frames[parent] · A`. The link frame is the
    frame AFTER `post`. `frames[0] = base`. TCP = `frames[tcp_link] · tool`.
-   `kin::compile(model)`, `kin::active_chain(model)` (URM joint indices
    root → tcp_link), `kin::effective_links(model)` (legacy 0/0 serial
    rule), `kin::forward_kinematics(cm, q)` → `FKResult{frames, tcp}`.
    `q` has `n_dof` entries (non-fixed, non-mimic joints, URM joint order).
-   Python (`import core_bridge as cb`): `cb.compile(model)`,
    `cb.fk_compiled(cm, q)`, `cb.fk(model, q)` → dict `frames`, `tcp`,
    `active_joints`; `cb.active_chain(model)`; `model.q_index_map()`;
    `model.dof()`. Binding rule: `std::array`/`std::optional` fields are
    COPIES (assign whole lists/objects). Vectors of joints/links are
    opaque (in-place edit works). Poses cross the boundary as 4x4 numpy
    arrays (see `fk_compiled` in `bindings/kin_bindings.cpp`).
-   Inertia data: `urm::Link::inertia` = `Inertia{mass, com[3],
    inertia_tensor[6] = (ixx, ixy, ixz, iyy, iyz, izz)}`: mass kg, COM in
    the link frame, tensor about the COM, axes parallel to the link frame.
    Mass 0 = not set. (The master plan says `Link.inertial`: the real name
    is `inertia`.)
-   `JointLimits{lower, upper, vel, effort}` on each `urm::Joint`.
-   Gravity: `SimulationSettings.gravity` in `app/project/project_data.py`,
    default `[0, 0, -9.81]`, saved in `simulation_settings.json`.
-   UI: `MainWindow` docks, `ProjectController` signals (`robot_changed`,
    `project_opened`, ...), `JointPanel.q_changed`, `RobotView`
    (`update`, actors), `TaskRunner.submit_fn(fn)` with signals
    `result/error/finished`; tests use `runner.wait_all(5000)` then
    `QApplication.processEvents()`.
-   Open R3 review item (do it in R4.1): `joint_links()` in
    `app/panels/model_panel.py` duplicates the C++ legacy rule
    `kin::effective_links`.

## Decisions fixed for R4 (do not reopen)

1.  Friction is NOT stored in the URM in R4 (no schema bump). It is an
    optional argument of the dynamics calls (arrays per DOF). Storing it
    in the URM is a later stage.
2.  The frozen compiled layout (R4.1) covers kinematics only. Dynamics use
    a separate struct `dyn::DynModel` built from `RobotModel` +
    `CompiledModel` (R4.6). R5 decides separately what CUDA needs.
3.  All Jacobians and dynamics are per DOF (`n_dof` columns/rows), never
    per joint. A mimic joint adds `mult` times its column to its master's
    DOF column.
4.  World frame everywhere for outputs (Jacobian, targets). Link-frame
    quantities stay inside the dynamics functions.
5.  Errors: bad input → `std::invalid_argument` (Python `ValueError`);
    numerical failure (not positive definite) → `std::runtime_error`
    (Python `RuntimeError`). IK non-convergence is NOT an exception: it is
    a result field.
6.  IK joint limits come from `joint.limits` of each DOF's master joint.
    Mimic joint limits are not enforced in R4 (noted in docs).

## Test oracles (shared helpers, built in R4.2 and reused)

-   `tests/python/numpy_ref.py` (new, R4.2): pure numpy, NO `core_bridge`
    import. `fk_numpy(spec, q)` builds the world frames from a plain
    Python description of a robot (list of joints with `parent, child,
    pre(4x4), post(4x4), axis, type, src, mult, offset` + `base`, `tool`).
    `numeric_jacobian(spec, q, link, point_local, h=1e-6)` by central
    difference (position rows) and by the rotation difference
    `log(R(q+h) R(q-h)^T)/2h` (angular rows).
    `spec_from_model(model)` may use the model objects to FILL the plain
    description (pre/post computed with numpy from origin/axis or DH
    formulas written in the helper), but the maths never calls `cb.fk`.
-   `numpy_dynamics(spec, inertias, q, qd, g)` (R4.6): independent
    Lagrangian reference: `M = Σ m Jv^T Jv + Jw^T Iw Jw` with `Jv, Jw`
    from `numeric_jacobian` at each link COM, `Iw = R I R^T`; the gravity
    torque `g(q)` = `+dV/dq` by central difference (`V = -Σ m g·p_com`, `g`
    the gravity vector; the actuator torque that holds the pose is +dV/dq;
    corrected 2026-11-11, R4.6 report); `C·qd` from the
    Christoffel form with `dM/dq` by central difference.
-   Random test models: helper `random_robot(seed, n, kinds)` returning a
    `RobotModel` (origin_axis and DH, revolute and prismatic, one with a
    branch, one with a mimic). Put in `tests/python/robots.py`.

---------------------------------------------------------------------

## R4.1 --- Compiled model freeze (C++ + bindings + docs)

Goal: fix the flat layout that R5 (CUDA) copies as is, and finish R3 item.

Files: `core/kin/compiled_arrays.hpp/.cpp` (new), `bindings/kin_bindings.cpp`,
`docs/compiled-model-layout.md` (new), `app/panels/model_panel.py`, tests.

1.  `struct CompiledArrays` (namespace `kin`), plain vectors, SoA, all
    in compiled (child-sorted) order, row-major, `double` for numbers:
    `std::vector<int32_t> joint_index, parent_link, child_link, type,
    q_index, src_q_index;` (`type` = `JointType` as int: Revolute 0,
    Prismatic 1, Fixed 2: take the real enum values from `joint.hpp`
    and document them), `std::vector<double> pre` (n·16), `post` (n·16),
    `axis` (n·3), `mult` (n), `offset` (n); scalars `n_joints, n_links,
    n_dof, tcp_link, layout_version = 1`; `base` (16), `tool` (16).
    4x4 matrices are row-major, last row (0 0 0 1).
2.  `CompiledArrays to_arrays(const CompiledModel&)` and
    `CompiledModel from_arrays(const CompiledArrays&)` (used only by the
    round-trip test; throws `std::invalid_argument` on wrong sizes or a
    wrong `layout_version`).
3.  Python: `cb.compile_arrays(model)` → dict of numpy arrays with the same
    names (int32 / float64; matrices shaped (n,4,4), axis (n,3), base and
    tool (4,4)) plus the scalars and `layout_version`.
4.  `docs/compiled-model-layout.md`: field order, dtype, shape, byte
    offsets, enum values, "frozen: changing it needs a new
    `layout_version` and an Opus-free review note", and a one-page example
    for a 2-joint robot with numbers. A test greps the doc for every field
    name in `CompiledArrays` (doc matches code).
5.  Bind `cb.effective_links(model)` → list of `(parent, child)` per URM
    joint. Replace the Python copy `joint_links()` in
    `app/panels/model_panel.py` by a call to it (same results; all R3 UI
    tests unchanged).

Tests: C++ `tests/cpp/test_compiled.cpp` (extend) or new
`test_compiled_arrays.cpp`: round-trip `from_arrays(to_arrays(cm))` gives
identical FK (tol 1e-15) for a DH robot, a tree, a mimic robot; wrong
`layout_version` throws. Python `tests/python/test_compiled_arrays.py`:
shapes and dtypes; array contents equal `cb.compile` values via
`cb.fk_compiled` rebuilt in numpy (multiply `pre·motion·post` in numpy,
tol 1e-12); doc-vs-code test; `effective_links` legacy case and normal
case; `joint_links()` unchanged results.

Done when: tests green, graphify, commit `R4.1: compiled layout freeze`.

---------------------------------------------------------------------

## R4.2 --- Jacobian (C++ + bindings)

Goal: geometric Jacobian at any frame, per DOF.

Files: `core/kin/jacobian.hpp/.cpp` (new), `core/CMakeLists.txt`,
`bindings/kin_bindings.cpp`, `tests/python/numpy_ref.py`,
`tests/python/robots.py`, tests.

1.  `Eigen::MatrixXd jacobian(const CompiledModel&, const std::vector<double>& q,
    int link, const Eigen::Vector3d& point_local = {0,0,0})`: 6 x `n_dof`,
    rows 0-2 linear velocity, rows 3-5 angular velocity, both in the WORLD
    frame, of the point `point_local` expressed in link `link` frame.
    Overload `jacobian_tcp(cm, q)`: link `tcp_link`, point = `tool`
    translation, AND the angular part is that of link `tcp_link` (tool
    rotation does not change angular velocity in world axes).
2.  Algorithm (use exactly): compute frames (reuse FK code, no extra
    allocation beyond result). For every compiled joint k on the path from
    link 0 to `link` (walk parents; joints not on the path give zero
    contribution):
    -   joint frame after `pre`: `F_k = frames[parent_k] · pre_k`; world
        axis `z_k = R(F_k) · axis_k`; world origin `o_k = t(F_k)`.
    -   `p` = world position of the point.
    -   revolute: `col_v = z_k × (p − o_k)`, `col_w = z_k`.
    -   prismatic: `col_v = z_k`, `col_w = 0`.
    -   fixed: nothing.
    -   `J[:, src_q_index_k] += mult_k · [col_v; col_w]`. (`mult_k = 1`
        for a normal joint, so a mimic slave on the path merges into its
        master's column. `offset` never appears.)
3.  Link index out of range or `q.size() != n_dof` → `std::invalid_argument`.
4.  Python: `cb.jacobian(cm, q, link, point_local=(0,0,0))` and
    `cb.jacobian_tcp(cm, q)` → numpy (6, n_dof). Also
    `cb.jacobian(model, q, ...)` overloads that compile internally are NOT
    added (callers compile once).

Tests: `numpy_ref.py` + `robots.py` created here. Analytic vs
`numeric_jacobian` (central difference) on 200 random poses for random DH
robots and random origin_axis robots (tol 1e-6; position rows and angular
rows), at the TCP and at an interior link with a nonzero `point_local`.
2R planar closed form: `J_v = [[-l1 s1 - l2 s12, -l2 s12], [l1 c1 + l2 c12,
l2 c12]]`, angular row z = `[1, 1]` (tol 1e-12). Prismatic robot (cart +
pendulum). Mimic: a gripper-like tree; the master column equals the sum of
the independent-DOF columns scaled by `mult` (compile the same robot with
the mimic as an independent DOF in the test and combine, tol 1e-12). Fixed
joint in the chain adds no column. Branch: DOFs off the path to `link` have
zero columns. Wrong size throws. Jacobian of the base link is zero.

Done when: tests green, graphify, commit `R4.2: Jacobian`.

---------------------------------------------------------------------

## R4.3 --- Numerical IK (C++ + bindings)

Goal: damped least squares IK for any compiled model (the target is the
TCP of the active chain).

Files: `core/kin/ik.hpp/.cpp` (new), `core/CMakeLists.txt`,
`bindings/kin_bindings.cpp`, tests.

1.  ```cpp
    struct IKOptions {
        bool   position_only = false;
        std::array<double,6> weights = {1,1,1,1,1,1}; // rows: x y z rx ry rz
        double damping = 0.05;       // lambda in J^T (J J^T + lambda^2 I)^-1
        int    max_iter = 100;
        double tol_pos = 1e-6;       // m
        double tol_rot = 1e-6;       // rad
        double max_step = 0.3;       // rad or m per iteration, infinity norm
        bool   enforce_limits = true;
        double null_gain = 0.0;      // null-space joint-limit push, 0 = off
    };
    struct IKResult {
        std::vector<double> q;
        bool   converged;
        double pos_error, rot_error; // norms at the returned q
        int    iterations;
        std::string message;         // "converged" | "max iterations" | "stalled"
    };
    IKResult solve_ik(const CompiledModel&, const std::vector<double>& q_limits_lo,
        const std::vector<double>& q_limits_hi, const Eigen::Isometry3d& target,
        const std::vector<double>& q0, const IKOptions&);
    ```
2.  Error vector `e` (6): `e_pos = p_target − p_tcp` (world),
    `e_rot` = axis-angle vector of `R_target · R_tcp^T` (world; use
    `Eigen::AngleAxisd`, angle in [0, π]). Weighted: `e_w = W e`,
    `J_w = W J` (`jacobian_tcp` from R4.2). Position-only: rows 3-5
    dropped (use only rows 0-2).
3.  Step: `dq = J_w^T (J_w J_w^T + λ² I)^-1 e_w` (solve with `LDLT`, do
    not form an inverse). If `‖dq‖_inf > max_step` scale `dq` down to
    `max_step`. `q ← q + dq`.
4.  Limits (`enforce_limits`): after each step clamp `q` into
    `[lo, hi]`. Null-space push when `null_gain > 0`: `dq_null =
    (I − J⁺J) · null_gain · (q_mid − q)/range`, added before clamping
    (`J⁺` = the damped pseudo-inverse of step 3).
5.  Stop when `‖e_pos‖ < tol_pos` and (position-only or `‖e_rot‖ < tol_rot`)
    → `converged = true`. Stop at `max_iter` → `"max iterations"`. If the
    step after clamping is below 1e-12 (inf norm) for two iterations in a
    row → `"stalled"`. Never throws for an unreachable target;
    `q.size() != n_dof` or limit vectors of the wrong size →
    `std::invalid_argument`. Returned `q` is always the best-error
    iterate, `pos_error`/`rot_error` computed at that `q`.
6.  Python: `cb.solve_ik(cm, lo, hi, target_4x4, q0, options_dict_or_None)`
    → dict (`q`, `converged`, `pos_error`, `rot_error`, `iterations`,
    `message`). Options as a dict with the names above (unknown key →
    `ValueError`). Helper in Python `app/kin_utils.py` (new):
    `dof_limits(model)` → `(lo, hi)` lists of size `dof()` from the master
    joint of each DOF (`q_index_map()`).

Tests: 100 random reachable cases per robot (a random DH 6R, a random
origin_axis 6R, a 3R planar, a branched robot): `q_true` random inside
limits → target = FK → IK from a perturbed `q0` (start within 0.5 rad of
`q_true`; plus 20 cases from `q0 = 0`) → FK error < 1e-6 in position and
rotation for the cases that report `converged` (require ≥ 90 % converged
from the perturbed start; report the rate in the test message). Position
only mode on a 3R planar. Unreachable target (distance 10 m): returns
without crash, `converged = False`, `message == "max iterations"` or
`"stalled"`. Limits: all returned `q` inside limits. Weights: zero weight on
rz makes the solver ignore a rotation about z (error still reported
unweighted). Wrong sizes throw. Determinism: same inputs, same output.

Done when: tests green, graphify, commit `R4.3: numerical IK`.

---------------------------------------------------------------------

## R4.4 --- Multiple IK solutions + validation (C++ + Python)

Goal: several distinct solutions, ranked, each validated.

Files: `core/kin/ik_multi.hpp/.cpp` (new), `bindings/kin_bindings.cpp`,
tests.

1.  `std::vector<IKSolution> solve_ik_multi(cm, lo, hi, target, q_current,
    IKOptions, int n_starts = 30, uint64_t seed = 42, double dedupe_tol = 1e-3)`:
    start 0 = `q_current`; starts 1..n−1 = uniform random in `[lo, hi]`
    (`std::mt19937_64`, seed given, so results are reproducible). Run
    `solve_ik` from each start. Keep only converged results.
2.  Dedupe: two solutions are the same if the inf-norm of the joint
    difference is `< dedupe_tol`. (Revolute joints wrapped to the limit
    range are compared after wrapping to `(−π, π]` ONLY when the joint
    limits span ≥ 2π; otherwise compared raw.)
3.  Validation of each kept solution: all `q` inside limits (tol 1e-9),
    FK error below the IK tolerances recomputed with `forward_kinematics`
    (not trusting the solver's own number). Failing ones dropped.
4.  Ranking: ascending inf-norm distance to `q_current`.
5.  `struct IKSolution { std::vector<double> q; double pos_error, rot_error;
    double distance; }`. Python: `cb.solve_ik_multi(...)` → list of dicts.
6.  Closed-form cross-check (TEST ONLY, in `tests/python/closed_form.py`,
    pure numpy): 2R planar analytic solutions (elbow up and elbow down) for
    a target `(x, y)`, and 3R planar for a target `(x, y, phi)`. No 6R
    spherical-wrist solver in R4 (decision: not needed for the milestone;
    revisit in R7 if wanted).

Tests: 2R planar (limits ±π, `l1 = l2 = 1`): target at (1.2, 0.8) →
exactly 2 solutions, equal to the closed-form pair (tol 1e-5); a target at
full stretch → 1 solution (within dedupe); unreachable → empty list.
3R planar with orientation: every IK solution passes an independent numpy
FK check (tol 1e-6) and appears in the closed-form set. 6R random robot: all
returned solutions pass the FK check and limits; list sorted by distance;
same seed → identical list; different seed may differ but every solution
valid. `n_starts = 1` equals `solve_ik` from `q_current`.

Done when: tests green, graphify, commit `R4.4: multiple IK solutions`.

---------------------------------------------------------------------

## R4.5 --- Singularity indicators (C++ + bindings)

Files: `core/kin/singularity.hpp/.cpp` (new), `bindings/kin_bindings.cpp`,
tests.

1.  `struct SingularityInfo { std::vector<double> singular_values;
    double sigma_min, sigma_max, condition, manipulability; bool near_singular; }`.
    `SingularityInfo analyze_singularity(const Eigen::MatrixXd& J,
    const std::string& mode = "full", double threshold = 1e-3)`.
2.  `mode`: `"full"` uses all 6 rows, `"position"` rows 0-2, `"angular"`
    rows 3-5, other → `std::invalid_argument`. Drop columns that are
    exactly zero (DOFs not on the active chain) before the SVD. Let
    `r = min(rows, nonzero columns)`. `singular_values` = the first `r`
    values (descending); `sigma_min = singular_values[r−1]`,
    `sigma_max = singular_values[0]`; `condition = sigma_max/sigma_min`
    (`+inf` if `sigma_min < 1e-300`); `manipulability` (Yoshikawa) =
    product of the `r` singular values (= `sqrt(det(J J^T))` when
    `rows ≤ columns`); `near_singular = sigma_min < threshold`.
    If `J` has no nonzero column: all zeros, `near_singular = true`.
3.  Python: `cb.analyze_singularity(J, mode="full", threshold=1e-3)` → dict.
    Convenience in `app/kin_utils.py`: `singularity_at(cm, q, mode,
    threshold)` = `jacobian_tcp` then analyze.

Tests: 2R planar position mode: `manipulability = |l1 l2 sin q2|`, `sigma`
values equal sqrt of eigenvalues of `A^T A` where `A` is the 2x2 position
block (tol 1e-12); `near_singular` true at `q2 = 0` and `q2 = π`, false at
`q2 = π/2`. 6-DOF arm with wrist alignment (`q5 = 0` on a spherical-wrist
robot built with origin_axis joints) → `near_singular` in full mode with
`sigma_min < 1e-9`; a generic pose → false. Threshold setting changes the
flag. Unknown mode throws. Zero-column handling (robot with a branch: the
off-chain DOF columns do not count). Random matrices vs `numpy.linalg.svd`
(tol 1e-10).

Done when: tests green, graphify, commit `R4.5: singularity indicators`.

---------------------------------------------------------------------

## R4.6 --- Inverse dynamics, RNEA (C++ + bindings)

Source: Opus consult 2026-11-11 (logged in `PROGRESS.md` → Decisions).
Notation: Featherstone. Spatial motion vector `v = [ω; v]` (angular first),
spatial force `f = [n; f]`. For `T = [R p]` (the map from frame B coords to
frame A coords), `^A X_B = [R 0; [p]R R]`, `^B X_A = [R^T 0; −R^T [p] R^T]`,
force transform `^A X_B* = (^B X_A)^T`. `[p]` is the skew matrix.
`v ×m = [[ω] 0; [v] [ω]]`, `v ×* = [[ω] [v]; 0 [ω]] = −(v ×m)^T`.
Everything is expressed in the link frame k (the frame after `post`).

Files: `core/dyn/dynamics.hpp/.cpp` (new), `core/CMakeLists.txt`,
`bindings/dyn_bindings.cpp` (new, registered from `core_bridge.cpp`),
`bindings/CMakeLists.txt`, tests.

1.  `struct DynModel` (namespace `dyn`), built once per model:
    `dyn::DynModel build_dyn_model(const urm::RobotModel&, const kin::CompiledModel&)`.
    Per compiled joint (same order as `cm.joints`): copy of `parent_link,
    child_link, type, src_q_index, mult`; the motion subspace `S_k`
    (6-vector, constant): with `post_k = [Rp pp]` and joint axis `a`:
    -   revolute: `S_k = [Rp^T a ; Rp^T (a × pp)]`
    -   prismatic: `S_k = [0 ; Rp^T a]`
    -   fixed: none (zero).
    (Reason: the axis is unchanged by the rotation about itself, so in the
    post-motion frame `S = [a; 0]`; `post` maps that frame to the link
    frame.) Per link k: spatial inertia at the link origin from `Link::inertia`
    (`c`, `Ic` built from the 6 tensor terms in the order ixx ixy ixz iyy
    iyz izz):
    `I_k = [Ic + m [c][c]^T , m [c] ; m [c]^T , m·1₃]` (Eigen 6x6).
    Also stores `n_dof`, `n_links`, base rotation, `pre/post/axis/offset` for
    the per-q transform. Mass 0 links are allowed (zero inertia). Does not
    call `validate_physical()`.
2.  `std::vector<double> inverse_dynamics(const DynModel&, q, qd, qdd,
    const Eigen::Vector3d& gravity_world, const Friction* friction = nullptr)`.
    All vectors size `n_dof` else `std::invalid_argument`.
    Per joint k (`λ = parent`): `qe = mult·q[src] + offset`,
    `qde = mult·qd[src]`, `qdde = mult·qdd[src]` (offset never enters
    derivatives; fixed joint: `qde = qdde = 0`, no `S` term). `A_k =
    pre·motion(qe)·post`. `X_k := ^kX_λ` is built from `A_k^-1`
    (`[R^T 0; −R^T[p] R^T]` with `R, p` the rotation and translation of
    `A_k`).
    Base (fixed link 0): `v_0 = 0`, `a_0 = [0 ; −R_base^T g]` where `g` is
    the gravity vector in WORLD and `R_base` the base rotation (so gravity
    is inside every `a_k`).
    Forward pass `k = 1..n−1` (index order = topological because
    `parent < child`):
    -   `v_k = X_k v_λ + S_k qde`
    -   `a_k = X_k a_λ + S_k qdde + v_k ×m (S_k qde)`
    -   `f_k = I_k a_k + v_k ×* (I_k v_k)`
    Backward pass `k = n−1..1`:
    -   non-fixed: `τ[src_k] += mult_k · S_k^T f_k`
    -   always (also for fixed joints): `f_λ += X_k^T f_k`.
    Branches need nothing special (children are summed before the parent
    because `parent < child`).
    The result `τ` is the actuator effort that produces `qdd`.
3.  Friction (R4.7 defines the struct; R4.6 only reserves the argument):
    `inverse_dynamics` returns pure rigid-body torque when `friction` is
    null. With friction: `τ += τ_fric(qd)` (R4.7 item 4).
4.  Convenience: `gravity_torque(dm, q, g)` = `RNEA(q, 0, 0, g)`;
    `bias_torque(dm, q, qd, g)` = `RNEA(q, qd, 0, g)` (includes gravity);
    `coriolis_torque(dm, q, qd, g)` = `bias − gravity` (this is `C(q,qd)qd`).
5.  Python: `cb.build_dyn_model(model, cm)` → opaque object;
    `cb.inverse_dynamics(dm, q, qd, qdd, gravity=(0,0,-9.81))`,
    `cb.gravity_torque`, `cb.bias_torque`, `cb.coriolis_torque` → lists.
    Also `cb.dyn_frames(dm, q)` → list of 4x4 world frames computed by the
    dynamics transforms (test-only hook; must equal `cb.fk`).

Tests (C++ `tests/cpp/test_dynamics.cpp` for 1 to 3, Python for the rest):
1.  Case A planar 2R, revolute z, `pre_2 = trans(l1,0,0)`, `post = I`, COM
    `c_i = (r_i,0,0)`, `Izz = I1, I2`, gravity `(0,−g0,0)`, `s2 = sin q2`,
    `c2 = cos q2`, `s12 = sin(q1+q2)`, `c12 = cos(q1+q2)`:
    -   `M11 = I1 + I2 + m1 r1² + m2 (l1² + r2² + 2 l1 r2 c2)`
    -   `M12 = I2 + m2 (r2² + l1 r2 c2)`, `M22 = I2 + m2 r2²`
    -   `C·qd = [−m2 l1 r2 s2 (2 qd1 qd2 + qd2²) ; m2 l1 r2 s2 qd1²]`
    -   `g1 = g0 [(m1 r1 + m2 l1) c1 + m2 r2 c12]`, `g2 = g0 m2 r2 c12`.
    `τ = M qdd + C qd + g` must match (tol 1e-10 with trig, 1e-12 without).
2.  Case B cart + pendulum: prismatic along x then revolute z, cart mass
    `m1` at the origin, pendulum `c = (r,0,0)`, `Izz = I2`, gravity
    `(0,−g0,0)`: `M = [[m1+m2, −m2 r sθ],[−m2 r sθ, I2 + m2 r²]]`,
    `C·qd = [−m2 r cθ θ̇² ; 0]`, `g = [0 ; m2 g0 r cθ]`.
3.  Case C `post` with rotation (frame check): one revolute z, `pre = I`,
    `post = Rx(90°)` with translation `(l,0,0)`, COM `c = (0,0,d)` in the
    link frame, `Ic = diag(Ixx, Iyy, Izz)` all different, gravity
    `(0,−g0,0)`: the joint axis is the link y axis; `M = Iyy + m (l² + d²)`,
    `g(q) = m g0 (l cos q + d sin q)`. Variant: `pre = Ry(37°)` and a
    rotated base with the gravity rotated consistently: the same numbers
    (invariance).
4.  3R spatial (origin_axis, non-trivial origins/axes, COM offsets,
    full tensors) vs `numpy_dynamics` (the independent Lagrangian from the
    oracle section): `τ` for 50 random `(q, qd, qdd)`, tol 1e-8 relative.
5.  Mimic: robot with a mimic slave (`mult = −2`, `offset = 0.3`): `τ` equals
    `Jm^T τ_full` where `τ_full` is the RNEA of the same robot compiled with
    the slave as an independent DOF, `Jm` the DOF map with `mult`; compare
    with `qd, qdd` mapped the same way (tol 1e-10).
6.  Fixed joint in the middle of a chain passes force through (compare with
    the same robot where the fixed link inertia is merged by hand into its
    child, tol 1e-10). Branch: two fingers on one parent give the same τ as
    the sum of the two independent single-finger robots' contributions
    (tol 1e-10).
7.  KE identity: `0.5 Σ v_k^T I_k v_k` (exposed via the test hook
    `cb.dyn_kinetic_energy_spatial(dm, q, qd)`) equals `0.5 qd^T M qd`
    (after R4.7) to 1e-12 relative.
8.  `cb.dyn_frames` equals `cb.fk` frames (1e-12). Wrong sizes throw.
    Gravity zero and `qd = qdd = 0` → `τ = 0`.

Pitfalls (from the consult; the Builder must check each one):
-   `S_k` must be in the LINK frame (after `post`), not in the `pre`/axis
    frame. Case C catches this.
-   `X_k` is `^kX_λ` built from `A_k^-1`, and forces use `X_k^T`.
    `a_0 = −R_base^T g`: not `+g`, not world frame.
-   Mimic/fixed bookkeeping: index `τ` by `src_q_index`, not joint index;
    `mult` in `qde`, `qdde` and in the projection; `offset` not in
    derivatives; fixed joints still propagate `f`.

Done when: tests green, graphify, commit `R4.6: inverse dynamics`.

---------------------------------------------------------------------

## R4.7 --- Mass matrix and forward dynamics (C++ + bindings)

Files: edit `core/dyn/dynamics.hpp/.cpp`, `bindings/dyn_bindings.cpp`, tests.

1.  `Eigen::MatrixXd mass_matrix(const DynModel&, q)`: CRBA over the tree:
    -   `Ic_k = I_k` for all links; for `k = n−1..1`:
        `Ic_λ += X_k^T Ic_k X_k` (also through fixed joints).
    -   `M` is `n_dof × n_dof`, zero-initialised. For each non-fixed
        compiled joint `i` with `si = src_q_index_i`:
        ```
        F = Ic_i S_i
        M[si,si] += mult_i² · S_i^T F
        j = i
        while parent[j] != 0:
            F = X_j^T F ; j = parent[j]
            if j is non-fixed:
                v = mult_i · mult_j · S_j^T F
                M[si,sj] += v ; M[sj,si] += v
        ```
        (When `si == sj` this adds `2v`: correct for a mimic on the same
        chain as its master.) Sibling branches are never visited: their
        entries stay 0. Result equals `J^T M_full J`.
    Here `parent[j]` means the compiled joint of the parent link
    (walk the joint whose `child_link` equals the parent link); stop at
    link 0.
2.  `struct Friction { std::vector<double> viscous, coulomb; double eps = 1e-3; }`
    (each size `n_dof`, default zeros). `friction_torque(f, qd)` =
    `viscous·qd + coulomb·tanh(qd/eps)` per DOF. Friction is applied per
    DOF, never per mimic joint, and stays OUT of `g` and `C`.
3.  `std::vector<double> forward_dynamics(const DynModel&, q, qd, tau,
    gravity, const Friction* = nullptr, const std::vector<double>* armature = nullptr)`:
    `qdd = M^-1 (τ − τ_fric − b)` with `b = bias_torque(q, qd, g)` (includes
    gravity). `M` gets `armature[i]` added on its diagonal when given.
    Solve with `LLT`; on failure throw `std::runtime_error("mass matrix not
    positive definite: massless link or DOF with no inertia")`.
4.  Inverse dynamics with friction: `τ = RNEA + friction_torque(qd)` (R4.6
    item 3 argument becomes active here).
5.  Python: `cb.mass_matrix(dm, q)` → numpy (n,n); `cb.Friction` (settable
    `viscous`, `coulomb`, `eps` lists/float); `cb.forward_dynamics(dm, q,
    qd, tau, gravity=(0,0,-9.81), friction=None, armature=None)`;
    `cb.inverse_dynamics(..., friction=None)`.

Tests:
-   Case A/B/C mass matrices equal the R4.6 closed forms (1e-12 / 1e-10).
-   Random robots (≥ 20 seeds, DH + origin_axis + one branched + one mimic):
    `‖M − M^T‖ ≤ 1e-12 ‖M‖`; min eigenvalue > 0 (for robots with all
    masses set); `M` equals the RNEA column trick (`RNEA(q,0,e_i) − g(q)`,
    tol 1e-10 rel); `M` equals `numpy_dynamics` `M` (tol 1e-8 rel);
    `RNEA(q, qd, FD(q, qd, τ)) = τ` (rel 1e-10); `g(q)` vs central
    difference of `V = −Σ m g·p_com` (h = 1e-6, tol 1e-7·max(1,‖g‖));
    skew-symmetry `qd^T Ṁ qd − 2 qd^T (C qd) = 0` with
    `Ṁ = [M(q+h qd) − M(q−h qd)]/2h`, h = 1e-6, rel tol 1e-6;
    `0.5 qd^T M qd` equals the spatial KE hook (1e-12 rel).
-   Mimic robot: `M` equals `Jm^T M_full Jm` (1e-10). Branch: `M[f1,f2] = 0`.
-   Friction: viscous only → `τ_fric = Fv qd` exactly; Coulomb: `tanh`
    value at `qd = 1` equals `Fc tanh(1/eps)`; FD with friction round trip
    with `inverse_dynamics(..., friction)` (1e-10); friction does not
    change `gravity_torque` or `coriolis_torque`.
-   Massless leaf link (mass 0, DOF with no inertia): `forward_dynamics`
    throws `RuntimeError`; with `armature` it works.
-   Wrong sizes throw `ValueError`.

Done when: tests green, graphify, commit `R4.7: mass matrix and forward dynamics`.

---------------------------------------------------------------------

## R4.8 --- Energy and torque tools (C++ + Python)

Files: edit `core/dyn/dynamics.hpp/.cpp`, `bindings/dyn_bindings.cpp`,
`app/kin_utils.py` (or `app/dyn_utils.py`, new), tests.

1.  `double kinetic_energy(dm, q, qd)` = `0.5 qd^T M qd`.
    `double potential_energy(dm, q, gravity)` = `−Σ_k m_k g · p_com,k`
    (world COM position of link k from the dynamics transforms; zero
    reference at the world origin).
    `total_energy = KE + PE`. Python: `cb.kinetic_energy`, `cb.potential_energy`,
    `cb.total_energy`.
2.  `std::vector<double> joint_power(tau, qd)` = elementwise product.
    Python `cb.joint_power(tau, qd)`.
3.  `app/dyn_utils.py`: `check_torque_limits(model, tau)` → list of
    dicts `{dof, joint_name, tau, limit, ratio, exceeded}` where the limit is
    `joint.limits.effort` of the DOF's master joint, `ratio = |tau|/limit`,
    `exceeded = ratio > 1`. A limit of 0 or negative → `exceeded` False and
    `ratio = inf if tau != 0 else 0` with a flag `"no_limit": True`.
4.  `app/dyn_utils.py`: `integrate_rk4_forward_dynamics(dm, q0, qd0, tau_fn,
    dt, steps, gravity)` → arrays of `q`, `qd` (stub integrator; used by
    tests and the energy test; documented as not a simulation engine: the
    real one is R6).

Tests: `0.5 qd^T M qd` equals the spatial KE hook; PE of a pendulum =
`m g0 r sinθ`-type closed form (case A with gravity `(0,−g0,0)`);
energy conservation, zero friction, `τ = 0`: 3R spatial random robot,
RK4, `dt = 1e-4`, 1 s, `|ΔE|/E0 < 1e-8`; with viscous friction energy
decreases monotonically; torque limit check: below, at, above the limit,
mimic slave not listed (only DOFs), no-limit case; power equals `τ·qd`.

Done when: tests green, graphify, commit `R4.8: energy and torque tools`.

---------------------------------------------------------------------

## R4.9 --- UI panels (Python UI)

Goal: use the engine from the app. Heavy work in `TaskRunner`.

Files: `app/panels/ik_panel.py` (new), `app/panels/analysis_panel.py` (new),
`app/main_window.py`, `app/viewport/robot_view.py` (target triad),
tests `tests/python/test_ui_kin_dyn.py`. Reuse `JointPanel`, `ModelPanel`,
`ProjectController`, `RobotView`, `app/kin_utils.py`, `app/dyn_utils.py`.

1.  `IKPanel` dock "Inverse Kinematics":
    -   target fields: x y z (m) and roll pitch yaw (deg), button "Use
        current TCP", checkbox "Position only", spin boxes for damping,
        max iterations, number of starts.
    -   "Solve" runs `solve_ik_multi` through `TaskRunner.submit_fn`; the UI
        stays responsive (button disabled while running, status text).
    -   solution list (table): index, distance, pos error, rot error;
        selecting a row previews it (joint panel values update, viewport
        moves) without committing; "Apply" commits it to the model
        (`model.joints[i].q`, marks the project dirty, emits `robot_changed`
        or the same path `JointPanel` uses). "Cancel preview" restores q.
    -   a target triad actor in `RobotView` (`set_target_pose(iso | None)`),
        drawn when the panel has a valid target, hidden when cleared.
        (Dragging the target in the viewport is NOT in R4: moved to the R6
        backlog.)
    -   no solution → message "No solution found" in the panel (no popup).
2.  `AnalysisPanel` dock "Analysis" with three groups:
    -   Jacobian / singularity: shows `sigma_min`, `sigma_max`, condition,
        manipulability, a "near singular" indicator (red label), a mode
        combo (full / position / angular) and a threshold spin box. Updates
        on every q change (fast, in the UI thread; Jacobian is small).
    -   Dynamics: gravity from the project's `SimulationSettings.gravity`
        (editable there only), `qd` and `qdd` entries default 0; table of
        torque per DOF (gravity + bias + `M qdd`), with a torque-limit
        column from `check_torque_limits` (red when exceeded); shows a
        warning "mass not set" if any non-root link under a non-fixed joint
        has mass 0 (then `forward dynamics` is disabled).
    -   Energy: KE, PE, total for the current `q`, `qd`.
3.  Both panels have View-menu toggles (checkable, hidden at start), clear
    when the project closes or the robot has no joints, and rebuild their
    compiled/dynamics model on `robot_changed` (use a small cache keyed on
    a model-change counter; do not recompile on every q change).
4.  All numeric text fields accept scientific notation; invalid text keeps
    the previous value and shows the field red.

Tests (offscreen): panel creation and toggles; "Use current TCP" fills the
fields with `fk()["tcp"]`; solving a reachable target on the 6-DOF fixture
(`tests/python/fixtures/urdf/six_dof_with_gripper.urdf`) lists ≥ 1 solution;
preview moves the TCP to the target (tol 1e-5) and "Cancel preview"
restores the old q; "Apply" changes the model q and sets dirty; an
unreachable target shows the message and no rows; the UI thread is not
blocked (the Solve handler returns before the result; use `wait_all`);
analysis panel shows near-singular for a wrist-aligned pose and not for a
generic one; torque table values equal `cb.inverse_dynamics` for the same
inputs; gravity from the project settings is used (change it, torques
change); mass-not-set warning on a model with zero masses; no regression
(all earlier UI tests green).

Done when: tests green, graphify, commit `R4.9: IK and analysis panels`.

---------------------------------------------------------------------

## R4.10 --- R4 milestone and stage gate

Builder does this after R4.9. `tests/python/test_milestone_r4.py`
(offscreen `MainWindow`, real `core_bridge`), using
`six_dof_with_gripper.urdf` (R3 fixture; it has inertials on all links):

1.  Import the fixture, set TCP link to `l6`.
2.  IK to a reachable target (FK of a chosen `q_true`): at least one
    solution, applied; TCP equals the target within 1e-5 (independent
    numpy FK from the R3 milestone helper, no `cb.fk`).
3.  Several solutions are listed and sorted; every one passes the numpy FK
    check.
4.  A wrist-aligned pose: the analysis panel flags near singular; a generic
    pose does not; `sigma_min` equals `numpy.linalg.svd` of a numpy
    numeric Jacobian (tol 1e-5).
5.  Torques shown for the current pose equal the independent
    `numpy_dynamics` torque (tol 1e-8 rel); gravity change in the project
    settings changes them as expected (linear in `g`).
6.  Forward dynamics round trip: `RNEA(FD(τ)) = τ` (1e-10); energy
    conservation over a 0.5 s RK4 run with `τ = 0` (`|ΔE|/E0 < 1e-8`).
7.  Save, close, open: the robot, TCP link, and masses are unchanged and
    the analysis panel gives identical numbers.
8.  Optional (decision R4-late, do NOT do it unless the user says so):
    Pinocchio via Miniconda as a validator only, never a runtime
    dependency.

Stage gate = `docs/MASTER_PLAN.md` section 1.3. In addition:
-   Docs: `docs/jacobian-ik.md` (R4.2 to R4.5), `docs/dynamics.md`
    (R4.6 to R4.8, with the formulas and sign conventions of this spec),
    `docs/compiled-model-layout.md` (R4.1), `docs/analysis-panels.md`
    (R4.9). One page each (gate item 8).
-   Record results in `PROGRESS.md` → "Current". Commit
    `R4: stage milestone test`. Tag `stage-R4-done` locally (no remote:
    do not push). Set stage status `stage-ready-for-review`.
-   STOP. R4 is a blocking-review stage. Wait for the Reviewer.
