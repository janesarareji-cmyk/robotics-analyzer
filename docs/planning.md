# Planning (R7.1, R7.4, R7.7)

Code: `app/planning/` (`profiles.py`, `cartesian.py`, `checks.py`, `planner.py`, `shortcut.py`,
`collision.py`), UI `app/panels/planning_panel.py`. No Qt in `app/planning/`. Path planning API and
numbers: `docs/path-planning.md`.

## Profiles (`profiles.py`)
`trapezoid(q0, q1, vmax, amax)`, `quintic(q0, q1, T)`, `scurve(q0, q1, vmax, amax, jmax)`. All
DOF share one scaled scalar profile s(t) (one duration, straight joint-space line). `JointProfile`
has `q/qd/qdd(t)` and `to_trajectory(dt)` (R6 `Trajectory`).

## Cartesian paths (`cartesian.py`)
`line` and `arc` sample the TCP path (position + orientation slerp) and solve each sample by IK
seeded from the previous one. A sample whose solution jumps more than `max_joint_step` counts as a
branch change and fails. Failure is a result (`failed_at`, `q` solved so far), not an exception.

## Checks (`checks.py`)
`check_trajectory(traj, model, limits=None, dt=0.01, gravity, check_torque=True)` returns a
`ViolationReport` (`ok`, one entry per contiguous run per kind and DOF, worst sample). Kinds:
position, velocity, acceleration, torque (RNEA with gravity), TCP speed. Limits come from the URM
(`vel`, `effort`, position); the URM has no acceleration limit; `limits=` overrides. A missing limit
is an info line.

## Planner interface (`planner.py`)
`plan(model, q0, goal_q_or_pose, collision, limits, planner, time_limit, seed, ...)` returns
`PlanResult` (never raises). Planners: RRT-Connect, RRT*, PRM. `collision` is a `CollisionWorld`,
`None`, or any object with `check_batch(q_array)`. `shortcut()` and `time_parametrize()` (cubic
through the waypoints, linear fallback) give a timed `Trajectory` for the R6 editor/player.

## Time-optimal parametrisation with torque limits (`topp.py`, F1.3)
`time_parametrize(path, model, limits, ..., method="topp", n_grid=100, gravity=...)`; the default
`method="scale"` is unchanged. API: `robotics_api.time_parametrize(path, robot, method="topp")`
(API 1.2). Planning panel: "Timing" combo, entry "Time-optimal (torque limits)".

Method: TOPP by reachability analysis (Pham and Pham 2018, "A New Approach to Time-Optimal Path
Parameterization based on Reachability Analysis", IEEE T-RO 34(3)). The path is a C2 spline
`q(s)`, `s` in [0, 1] (chord length), on `n_grid` stages. With `x = sdot^2` and `u = sddot` every
limit is linear in `(u, x)`: velocity `q'_j^2 x <= vmax_j^2`, acceleration
`|q'_j u + q''_j x| <= amax_j`, torque `|a_j u + b_j x + c_j| <= effort_j` with
`c = RNEA(q,0,0)`, `a = RNEA(q,0,q') - c`, `b = RNEA(q,q',q'') - c` (exact, Coriolis is quadratic
in `qd`; RNEA with gravity). A backward pass gives the controllable intervals `K_i` (`K_N = {0}`),
a forward greedy pass takes the largest `u` that keeps `x_{i+1} = x_i + 2 ds u` in `K_{i+1}`.
Each step is a 2-variable LP solved exactly by vertex enumeration (`_lp2d`, no scipy). Times:
`dt_i = 2 ds_i / (sqrt(x_i) + sqrt(x_{i+1}))`. The result is a cubic `Trajectory` through
`(q(s_i), t_i)`, rest at both ends.

-   Limits: velocity and acceleration as for "scale" (`vmax`, `amax`; defaults joint velocity
    limit or 1, and 2); torque = `limits["effort"]` or the URM joint efforts (none = unlimited).
-   Friction is excluded: it is not linear in `x`. The result is re-checked with
    `check_trajectory` (the app friction model is not used in the check either, as in R7).
-   Residual error: limits hold at grid points only and the cubic spline in time rings at the
    acceleration jumps of a time-optimal profile (a few percent). If `check_trajectory` still shows
    a violation the time is scaled up by `max(1.02, 1.001 sqrt(worst ratio))`, at most 20 times; the
    final scale is in `TimedPath.time_scale` and `TimedPath.notes` (the solver duration is there too).
    The cost is up to about 13 % on a pure bang-bang profile (ringing is about 0.27 of an
    acceleration jump); a smoother interpolation class would remove it. (The spec text gives a fixed
    1.02 step; the adaptive step is never smaller.)
-   Cubic geometry is used only when a dense sample stays inside the position limits and is
    collision free; else the polyline is used and every segment is solved rest-to-rest.
-   Infeasible: `PlanningError("torque limits cannot be met on this path at rest-to-rest ...")`
    when the holding torque at the start or end is above the limit, or a controllable set is empty.
-   Not covered: torque limits between grid points, friction, rest-to-rest only (no start or end
    velocity), no jerk limit.

## Workflow in the app
Ghost panel stores named poses ("start", "goal"); Planning dock reads them, runs the planner in a
`TaskRunner` worker, "Send to trajectory" fills the Simulation timeline and `project.trajectory`.
Collision panel supplies obstacles and the ACM.

## Decisions
-   Own planners (2026-11-12): OMPL has no Windows wheel for Python 3.13 (`docs/planner-decision.md`).
-   Collision: python-fcl 0.7.0.11 (`docs/collision-decision.md`).
-   Planning stays inside joint limits, no revolute wrap (v1). Torque is not a planning constraint:
    `time_parametrize(check_torque=False)` by default; use `check_trajectory` afterwards.
-   Milestone note (R7.7): the fixture effort limit (10 N m) is below its own static gravity torque
    (25.9 N m on dof 1 at q = 0), so the milestone checks torque against an explicit 40 N m.
