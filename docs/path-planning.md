# Path planning (R7.4)

Code: `app/planning/planner.py` (planners), `app/planning/shortcut.py` (shortcut, time parametrisation),
`app/panels/planning_panel.py` (panel). Own planners in Python (OMPL not installable, see `planner-decision.md`).

## API

```python
from app.planning.planner import plan
res = plan(model, start_q, goal_q, collision, limits=None, planner="rrt_connect",
           time_limit=5.0, seed=0, resolution=0.02, step=0.3, cancel=None, progress=None)
# PlanResult(path (n, dof) or None, success, time, iterations, reason, planner, length, raw_length)
```

-   `goal_q`: joint vector, or a 4x4 TCP pose (all IK solutions inside the limits and collision free
    become goals, the first reached wins).
-   `collision`: a `CollisionWorld`, or `None` (joint limits only), or any object with `check_batch(q_array)`.
-   `limits`: `(lo, hi)`, dict `pos_lo/pos_hi`, or `None` (URM limits; a joint with no limit is sampled in +-pi).
-   `planner`: `rrt_connect`, `rrt_star`, `prm`. Same seed gives the same path. RRT* stops 300
    iterations after its first path, PRM adds 40 nodes per round: only `time_limit` makes a run time dependent.
-   `cancel`: callable, `threading.Event`, or object with `is_cancelled` (checked every iteration).
    `progress(iteration, planner)` every 16 iterations.
-   Failures never raise: `success=False` and `reason` (`start in collision`, `goal in collision`,
    `start/goal outside joint limits`, `time limit reached`, `cancelled`, `no IK solution ...`).
-   Edge validity: dense interpolation, spacing <= `resolution` (0.02 rad), far end checked first.
    Shortcutting (random pair, `shortcut.shortcut`) never makes the path longer and keeps every edge free.
-   No revolute wrap: planning stays inside the joint limits (v1).

## Time parametrisation

`time_parametrize(path, model, vmax, amax, collision)` returns `TimedPath(trajectory, report, ...)`:
one R6.4 `Trajectory` through the waypoints (cubic; linear fallback when the spline would leave the
limits or hit something), segment times from a trapezoid rule per DOF, scaled up by 1.25 until
`check_trajectory` has no velocity/acceleration violation. `report` is the final `ViolationReport`.

## Panel

Start/goal columns (typed, "= current q" buttons, named-configuration combos: call
`set_named_configs({name: q})` from R7.6), planner, time limit, seed, resolution, shortcut, Plan (worker),
Cancel, summary, Send to trajectory (`trajectory_ready` signal; connect to `SimPanel.set_trajectory`).
Collision: `set_project(project)` uses the stored shapes and `project.collision or CollisionConfig()`;
there is no collision settings UI yet, so an environment obstacle is set through code/API
(`project.collision.environment`) until a later substage. The goal in the panel is a joint vector
(the API also takes a pose).

## Numbers (planar 2R, one box obstacle, Ryzen 9 8940HX)

RRT-Connect 168 ms, RRT* 1.1 s (first path + 300 iterations), PRM 0.4 s.
