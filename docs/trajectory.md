# Trajectory and playback (R6.4)

Code: `app/sim/trajectory.py`, `app/sim/playback.py`, `app/panels/timeline_panel.py`.

## Model
`Trajectory(waypoints, interpolation, clamped)`; `Waypoint(t, q)` with one q value per DOF. Rules: at least 2 waypoints, first `t == 0`, `t` strictly increasing, all values finite. Waypoints outside the joint limits give a warning (`limit_warnings`), not an error.

## Interpolation
- `linear`: piecewise linear q, qd piecewise constant, qdd = 0.
- `cubic`: per-DOF cubic spline, C2 inside. `clamped=True`: zero velocity at both ends (default). `clamped=False`: natural spline (zero qdd at the ends).
- `cubic` with a velocity on EVERY waypoint (`Waypoint(t, q, qd)`, U2.9, produced by TOPP): piecewise cubic Hermite through `(q, qd)` at the knots (C1; `clamped` unused). `trajectory.json` then has an optional `"qd": [..]` per waypoint; it is written only in that case, older readers ignore it, so the project format stays 1.4.0. Editing the waypoint table (timeline panel) drops the velocities and gives the normal spline.
`q(t), qd(t), qdd(t)` clamp `t` to `[0, duration]`; q is exact at the knots.

## Playback (`TrajectoryPlayer`)
- KINEMATIC: the engine clock runs, `advance(wall_dt)` moves playback time by `wall_dt * speed` and calls `engine.set_state(t, q, qd, qdd)`; torque = RNEA(q, qd, qdd). No dynamics step. The last state equals the last waypoint exactly.
- DYNAMIC (computed torque): `engine.torque_fn = RNEA(q_d, qd_d, qdd_d) + Kp (q_d - q) + Kd (qd_d - qd)`, the engine integrates the real state. Gains `track_kp, track_kd` per DOF (defaults 100 and 20). Armature is added to the feed-forward when set.
- Speed 0.1 .. 4.0, optional loop, seek (clamped), pause/resume, stop (restores the initial pose). At the end the clock pauses and `finished` is set.
- Tracking error (RMS, max of |q_d - q|) is shown in the status line in DYNAMIC mode.
Milestone result (6-DOF fixture + gripper, 4 waypoints, 4 s, gravity on, RK4 dt 1 ms): tracking RMS 3.7e-11 rad, max 2.2e-10 rad.

## UI
Timeline panel (in the Simulation dock): waypoint table (t, q per DOF; add, delete, move, capture current pose), invalid rows are marked but kept, last valid trajectory is stored in the project; transport play/pause/stop, speed, loop, slider. Editing the table during playback discards the player.

## Project format 1.2.0
The project zip gets an optional `trajectory.json`:
```
{"interpolation": "cubic", "clamped": true,
 "waypoints": [{"t": 0.0, "q": [..]}, ...]}
```
`PROJECT_FORMAT_VERSION` = 1.2.0. Files 1.0.x and 1.1.x (no trajectory file) still load with `trajectory = None`. An invalid trajectory file is ignored with a warning. A trajectory whose DOF count differs from the robot is dropped (with a log line) on open and when the robot changes.

## Project format 1.3.0 (R7)
`PROJECT_FORMAT_VERSION` = 1.3.0. Two more optional members, both absent in files 1.0.x-1.2.x
(they load with `collision = None` and an empty config store):
-   `collision.json`: `{"format", "fit_mode", "acm_edits": [[a, b, bool], ...], "environment": [{"name", "kind", "params", "xyz", "rpy"}]}`; written only when not default (`docs/collision.md`).
-   `configs.json`: named joint poses `{name: [q...]}` (ghost panel); poses whose length differs from the robot DOF are dropped with a log line on open and when the robot changes.
`trajectory.json` is unchanged. A planned path sent to the editor is stored like any trajectory.
Milestone R7 (`tests/python/test_milestone_r7.py`): trajectory, collision settings and configs are identical after save, close, open.

## Project format 1.4.0 (F1.4)
`PROJECT_FORMAT_VERSION` = 1.4.0. No new member. `configs.json` gets an optional list `"shown"`: `[{"name": str, "color": "#rrggbb"}]`, the ghosts shown in the Ghosts dock, in show order (max 8). Entries with an unknown name, a bad colour or a duplicate are skipped with a log warning. Files 1.0.0-1.3.0 load with no ghost shown; the 1.3.0 to 1.4.0 migration step is a no-op (backup rule as for every older file). On open the saved set is drawn again; show, hide, delete and rename mark the project modified. Replacing the robot hides the ghosts (and clears the saved set).
