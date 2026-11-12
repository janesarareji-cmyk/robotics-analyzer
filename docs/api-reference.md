# Scripting API reference (`robotics_api`, R8.1)

Code: `robotics_api/` (top-level package next to `app/`), runner `app/script_runner.py`, console dock
`app/panels/script_panel.py`, examples `examples/*.py`.
Import: `import robotics_api as api`. The repo root must be on `sys.path` (the app and the tests
do this; for a standalone script set `PYTHONPATH` to the repo root and `bindings/`).

## Version and compatibility rule

`robotics_api.__api_version__ = "1.0"` (`MAJOR.MINOR`, semantic):

-   MINOR (1.0 -> 1.1): additive only (new function, new optional argument, new key in a returned dict,
    new attribute). A script written for `1.x` keeps working on every later `1.y`.
-   MAJOR (1.x -> 2.0): may remove or change behaviour. Scripts check `api.__api_version__`.
-   Not part of the API: anything starting with `_`, and every module under `app/` or `core_bridge`.

The API takes and returns plain Python and numpy types only (lists, dicts, `float64` arrays, strings).
Every error is a `RoboticsApiError` (see Errors); raw bridge exceptions never leak.

## Units and conventions

Metres, radians, seconds, newtons, newton metres, kilograms. Poses are 4x4 homogeneous matrices in
the WORLD frame (base pose included). A joint vector has one entry per DOF (`robot.n_dof`):
radians for revolute, metres for prismatic joints; fixed and mimic joints are not DOFs. Gravity default
`[0, 0, -9.81]` m/s^2 (world vector).

## Project and files

| Function | Returns | Notes |
|---|---|---|
| `new_project(name="Untitled")` | `Project` | not saved |
| `open_project(path)` | `Project` | `ProjectError` for a missing or damaged file |
| `save_project(project, path)` | path | path must end with `.robotproj`; `ProjectError` otherwise |
| `import_urdf(path, search_roots=None)` | `Robot` | `ImportFailed` (`.report` text) on errors; `robot.import_report` holds warnings |

`Project`: `name`, `description`, `path` (None until saved), `robot` (snapshot `Robot`; assign a `Robot`
to replace the project model).

## `Robot`

Created by `import_urdf`, `Project.robot`, `Robot.from_json(text)`. Owns a snapshot of the model.

| Member | Meaning |
|---|---|
| `n_dof`, `joint_names`, `joint_types` | per DOF, in q order |
| `limits` | array `(n_dof, 2)` = `[lower, upper]` (rad or m) |
| `fk(q)` | 4x4 TCP pose (m) |
| `jacobian(q)` | `(6, n_dof)` world frame at the TCP; rows 0-2 linear (m/s per unit), rows 3-5 angular (rad/s) |
| `ik(pose, seed=None, all_solutions=False, position_only=False, n_starts=30, random_seed=42)` | nearest solution (array) or `None`; with `all_solutions=True` a list sorted by distance to `seed` (maybe empty). Solutions respect joint limits |
| `inverse_dynamics(q, qd, qdd, gravity=None)` | torques (N.m, N) |
| `forward_dynamics(q, qd, tau, gravity=None)` | accelerations |
| `mass_matrix(q)` | `(n_dof, n_dof)` |
| `energy(q, qd, gravity=None)` | `{"ke", "pe", "total"}` in J |
| `manipulability(q)` | Yoshikawa measure of the full 6-row Jacobian (product of singular values, mixed units) |
| `set_collision_shapes(list_of_dicts)`, `collision_shapes` | link geometry for planning: `{"type": box/sphere/cylinder/capsule, "link": index, "pose": ..., "size": ...}` |
| `to_json()`, `copy()` | URM JSON, independent copy |

`pose` arguments of shapes and obstacles: a 4x4 matrix, `{"xyz": [...], "rpy": [...]}` or 3 / 6 numbers (m, rad,
URDF roll-pitch-yaw). `size`: box `[x, y, z]` m, sphere radius m, cylinder / capsule `[radius, length]` m.

## Motion

-   `Trajectory(times, waypoints, interpolation="cubic")`: `duration`, `n_dof`, `times`, `waypoints`, `q(t)`,
    `qd(t)`, `qdd(t)`, `sample(dt)` -> dict `t, q, qd, qdd`; `ok` / `report` after `time_parametrize`.
-   `time_parametrize(path, robot, vmax=None, amax=None, interpolation="cubic")` -> `Trajectory`. Segment times
    are scaled until no velocity or acceleration limit is violated.
-   `plan(robot, start, goal, obstacles=None, planner="rrt_connect", seed=0, time_limit=5.0, resolution=0.02,
    shortcut=True)` -> array `(n_points, n_dof)`. `goal` is a joint vector or a 4x4 pose. `obstacles`: list of
    `{"type": "box"|"sphere"|"cylinder", "pose": ..., "size": ...}` in the world frame. `planner`: `rrt_connect`,
    `rrt_star`, `prm`. Same seed, same path. Raises `PlanningError` when there is no path. Collision geometry
    comes from the URDF or `set_collision_shapes` (obstacles without robot geometry raise `InvalidArgument`).
-   `simulate(robot, control=None, duration=None, dt=0.001, integrator="rk4", q0=None, qd0=None, gravity=None,
    record_every=1, kp=None, kd=None)` -> dict of arrays. `control`: `None` (zero torque), a torque vector, a
    callable `tau = f(t, q, qd)`, or a `Trajectory` (computed-torque tracking, default gains 100 and 20).
    Result keys: `t`, `q`, `qd`, `qdd`, `tau`, `tcp_pos`, `tcp_quat`, `tcp_vel`, `ke`, `pe`, `total`, `power`,
    `power_sum`, `sigma_min`, `manipulability` (units in `docs/simulation.md`). Sample 0 is the initial state.
-   `workspace(robot, n_samples=200000, voxel=None, seed=42, metrics=False)` -> dict: `volume` (m^3),
    `volume_err_bound`, `filled`, `voxel_size`, `origin`, `dims`, `keys`, `count`, `centers` `(filled, 3)`.
-   `export_csv(result, path, names=None, meta=None)` and `export_npz(result, path, names=None, meta=None)`:
    write a `simulate` result (format: docstring of `app/sim/export.py`, `docs/graphs.md`); return the row count.

## Errors

`RoboticsApiError` (base) with subclasses `InvalidArgument` (also a `ValueError`: wrong shape, wrong `n_dof`,
non-finite value, unknown name), `ProjectError`, `ImportFailed`, `ComputationError`, `SimulationError`,
`PlanningError`, `AnalysisError`, `ExportError`, `ScriptCancelled` and `ScriptTimeLimit(ScriptCancelled)`.
The original exception is kept as `__cause__`.

## Cancel (cooperative)

`simulate`, `plan` and `workspace` call `check_cancel()` often and raise `ScriptCancelled` when the runner asked
to stop (Cancel button, or the time limit -> `ScriptTimeLimit`). A script with its own long loop calls
`api.check_cancel()` inside the loop. `api.is_cancelled()` tests without raising. Outside the runner these are
no-ops. The token is thread-local: threads started by the script are not cancelled. A script stuck in pure
Python code that never calls the API cannot be stopped.

## Script runner (`app/script_runner.py`)

`ScriptRunner(task_runner, project_provider=None, time_limit=None)`: `run_string(code, name)`, `run_file(path)`,
`cancel()`, `is_running`; signals `started()`, `output(text, is_err)` (one line), `finished(status, message)` with
status `ok` / `error` / `cancelled` / `timeout` (message = traceback for `error`). One script at a time; the
script runs in a `TaskRunner` worker thread, the GUI stays responsive. The script namespace has
`robotics_api` (also `api`) and `app`, a read-only snapshot: `has_project`, `project_name`, `project_path`,
`robot` (a `Robot` copy or None), `q` (current joint vector), `version`. For `run_file`, `__file__` is set and the
script folder is on `sys.path` while it runs. Output uses thread-aware stdout / stderr: only writes made by the
script thread are captured, the app's own output is untouched (streams are restored when the script ends,
even if the script replaced them). A script exception never leaves the runner. Setting: `time_limit` seconds
(default None).

UI: View -> Script console (dock, hidden at start: editor, Run, Cancel, output); File -> Run script... (file
dialog). Output also goes to the log panel (`[script]` / `[script stderr]`).

## Examples

`examples/fk_sweep.py` (FK sweep), `examples/plan_and_simulate.py` (plan around a box, time parametrize,
simulate the tracking), `examples/export_data.py` (simulate and export CSV and NPZ). They use
`examples/planar_2r.urdf` and run headless or in the console.
