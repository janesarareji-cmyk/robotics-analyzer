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
