# Stage R6 spec pack: Simulation and graphs

Authority: `docs/MASTER_PLAN.md` section 5 (workflow, gate, push in section 1).
Review: non-blocking (max one unreviewed stage; R5 review done before this spec).
"Builder" = Claude subagents launched by Sonnet (`.agents/rules/dev-workflow.md`).
Sonnet reviews each substage diff before the next one starts.

Order: R6.1 → R6.2 → R6.3 → R6.4 → R6.5 → R6.6 → R6.7 → R6.8.
Start point: first row of R6.1..R6.8 in `PROGRESS.md` not `committed`.
Suggested Builder tier: R6.2 Sonnet (numerics); R6.1, R6.3, R6.4, R6.5, R6.8 Sonnet;
R6.6, R6.7 Sonnet (UI + proof build); pure glue/doc edits Haiku.

## Common rules (every substage)

-   Start with `graphify update .` and `graphify query` for the area. Reuse
    existing code. No duplicate files.
-   Every numbered rule has at least one test. Math tests use an INDEPENDENT
    oracle (closed-form pendulum, `tests/python/numpy_ref.py`,
    `numpy_dynamics`), not the code under test.
-   After each substage: build if C++ changed, `ctest` + `pytest` green,
    `graphify update .`, commit `R6.<y>: <summary>`, row `committed` in
    `PROGRESS.md`.
-   Baseline after R5: CTest 9/9, pytest 673. Old tests stay green.
-   Commands: `cmake --build build --config Release`, then
    `ctest --test-dir build -C Release --output-on-failure` and
    `python -m pytest -q`. (`run_tests.ps1` is blocked by execution policy.)
-   Spec unclear or contradicts the code → do NOT guess. Write the question in
    `PROGRESS.md` → "Blockers", row `blocked`, stop. Same substage red after two
    fix attempts → same.
-   The UI thread never runs a long loop. Simulation steps run in a worker
    (`TaskRunner`) or a `QTimer` budgeted per tick (see R6.1). Tests for UI use
    offscreen Qt.
-   Python for state machine, UI, buffers, export; C++ (`core/`) only if a hot
    loop needs it. R6.2 integrators are Python on top of `cb.forward_dynamics`
    first; move to C++ only if the R6.2 speed test fails (see there).
-   New dependencies need a Windows proof build first (R6.6 item 0 only).

## Code facts (checked 2026-11-11, after R5)

-   Dynamics (`core_bridge`, `bindings/dyn_bindings.cpp`): `build_dyn_model(model, cm)`
    → `DynModel` (`n_dof`); `forward_dynamics(dm, q, qd, tau, gravity=(0,0,-9.81),
    friction=None, armature=None)` → qdd list; `inverse_dynamics(dm,q,qd,qdd,gravity,
    friction)`; `mass_matrix`, `gravity_torque`, `bias_torque`, `coriolis_torque`,
    `kinetic_energy(dm,q,qd)`, `potential_energy(dm,q,gravity)`, `total_energy`,
    `joint_power(tau,qd)`; `Friction` (viscous, coulomb, eps lists per DOF).
    `forward_dynamics` throws `RuntimeError` if M is not positive definite.
-   Kinematics: `cb.compile(model)` → `CompiledModel`; `fk`, `jacobian_tcp(cm,q)`,
    `analyze_singularity(J, mode, threshold)` (see `docs/jacobian-ik.md`),
    `app/kin_utils.py` (`dof_limits`), `app/dyn_utils.py`
    (`integrate_rk4_forward_dynamics` is a STUB: R6.2 replaces its use,
    keep it for the R4 tests).
-   Project: `app/project/project_data.py` `SimulationSettings` (integrator,
    physics_timestep, render_framerate, gravity; "populated fully in R6") and
    `app/project/serializer.py` (zip `.robotproj`, `PROJECT_FORMAT_VERSION` 1.1.0,
    `_write_json` / `_read_json`). R6.4 adds a trajectory file (format 1.2.0,
    old 1.0/1.1 files must still load).
-   UI: `TaskRunner.submit_fn(fn)` → `FunctionTask` with signals;
    `JointPanel` (R2.8) drives q; `AnalysisPanel` (R4.9) shows
    torque/energy for a q; `MainWindow` docks (Workspace, Performance are
    tabbed with Joints, hidden at start, View-menu toggles). Viewport:
    `app/viewport/robot_view.py`.
-   Graph library per `TECH_STACK.md`: PyQtGraph (NOT installed yet);
    matplotlib 3.11.2 is installed.
-   Friction is not stored in URM (R4 decision). In R6 friction and armature are
    simulation settings (`SimulationSettings`), default zero.

## R6.1 --- Simulation core and state machine

Goal: a UI-free simulation object that holds the state and steps it.

Files: `app/sim/__init__.py`, `app/sim/state.py`, `app/sim/engine.py`, tests.

1.  `SimState` dataclass: `t`, `q`, `qd`, `qdd`, `tau` (applied), `tau_inv`
    (RNEA torque for `qdd`, filled by dynamic mode), `tcp` (4x4), `energy` dict
    (ke, pe, total). numpy arrays of length n_dof. `copy()` deep.
2.  `SimEngine(model, settings)`: owns `cb.compile`, `build_dyn_model`; methods
    `reset(q0, qd0=None)`, `step(dt)` → new `SimState` (one physics step, no
    clock), `state` property. Mimic and fixed joints follow the compiled
    q-index map (q has n_dof entries, as everywhere).
3.  State machine `SimClock` (or inside the engine): `STOPPED`, `RUNNING`,
    `PAUSED`; `start()`, `pause()`, `resume()`, `stop()` (reset to the initial
    state); illegal transitions raise `ValueError` (e.g. `pause()` when STOPPED).
    Signals through a small callback list (`on_state_changed`) so the core has
    no Qt dependency; a thin Qt adapter lives in R6.3.
4.  Time control: fixed step `dt` (default `settings.physics_timestep`) or
    variable (the integrator returns the accepted step); `time_scale` (> 0, default 1.0,
    max 100); `advance(wall_dt)` runs `ceil` steps so sim time grows by
    `wall_dt * time_scale`, but at most `max_steps_per_advance` (default 200)
    per call; the remainder is dropped and `engine.lag` reports the dropped sim
    time (no spiral of death).
5.  Gravity vector from settings; friction `Friction` and `armature` optional
    from settings (zero default).

Tests (`tests/python/test_sim_core.py`): state machine all legal and illegal
transitions; `stop()` restores the initial state; `advance` with `time_scale`
2.0 grows sim time by twice the wall time (fixed dt); `max_steps_per_advance`
cap sets `lag > 0` and never loops beyond it; `SimState.copy` is deep; a 2R
robot with zero torque and zero gravity keeps q and qd constant.

Done when: tests green, graphify, commit `R6.1: simulation core`.

## R6.2 --- Integrators

Goal: three integrators on `forward_dynamics`, tested against analytic and
energy references.

Files: `app/sim/integrators.py`, tests; optional C++ only per item 5.

1.  Interface: `Integrator.step(f, t, y, dt)` where state `y = [q, qd]`
    (length 2n) and `f(t, y)` returns `[qd, qdd]` using
    `cb.forward_dynamics`. Torque input `tau(t, q, qd)` is a callable given to
    the engine (zero by default). Names: `"euler"` (semi-implicit Euler:
    qd += dt*qdd, then q += dt*qd), `"rk4"`, `"rk45"` (Dormand-Prince, adaptive).
2.  RK45: error control `atol=1e-9`, `rtol=1e-7` (settings), step size factor
    0.9, min factor 0.2, max factor 5, `dt_min = 1e-7`, `dt_max = 0.01`
    (settings); rejected steps retry with the smaller step; below `dt_min` →
    raise `RuntimeError("step size underflow")`. Returns the accepted `dt` and
    the next suggested `dt`.
3.  `forward_dynamics` `RuntimeError` (non-positive-definite M) is not
    swallowed: the engine goes to `PAUSED` and records `engine.error`.
4.  `SimulationSettings.integrator` accepts `"euler"`, `"rk4"`, `"rk45"`
    (default stays `"rk4"`). Unknown name → `ValueError`. Settings round trip
    unchanged for old files.
5.  Speed test: RK4 on the 6-DOF fixture must run at least 2000 steps/s on
    this machine in pure Python glue (measured in the test, generous floor
    1000). If it fails, the Builder moves the integrator loop into C++ (a
    `sim_step` binding that calls forward dynamics internally) and keeps the
    Python interface; log the decision under "Decisions".

Tests (`tests/python/test_integrators.py`): planar pendulum (1-link, point
mass, from the R4 fixtures): small-angle period within 0.5 % of `2π√(L/g)`
for RK4 and RK45; convergence order by halving `dt`: Euler error ratio ≈ 2
(1.7 to 2.4), RK4 ratio ≈ 16 (12 to 20); energy drift over 5 s on a frictionless
pendulum |dE|/E0 < 1e-8 (RK4, dt 1e-3) and < 1e-6 (RK45); damped case (viscous
friction) energy never increases; adaptive step shrinks near a stiff-ish case
(large initial qd) and grows when smooth; non-positive-definite M test pauses
the engine; unknown integrator name → ValueError.

Done when: tests green, graphify, commit `R6.2: integrators`.

## R6.3 --- Manual mode (kinematic and dynamic) + UI controls

Goal: run the simulation from the UI; two modes.

Files: `app/sim/mode.py`, `app/panels/sim_panel.py`, `app/main_window.py`, tests.

1.  Modes: `KINEMATIC` (sliders or joint panel set q directly; qd, qdd = 0;
    `tau_inv` computed by RNEA from finite differences only if the user turns
    on "estimate torque", default off) and `DYNAMIC` (state evolves by forward
    dynamics from applied torque and gravity). Mode switch keeps q; switching
    to DYNAMIC sets qd = 0; switching to KINEMATIC drops qd.
2.  Dynamic manual torque: per-joint torque entries (spin boxes, Nm / N) in the
    panel, applied constant while RUNNING; "gravity on/off", "friction on/off"
    (settings), "armature" per joint optional.
3.  `SimPanel` dock "Simulation" (View toggle, hidden at start, tabified with
    Joints): Start / Pause / Resume / Stop buttons (enabled per state), mode
    combo, integrator combo, dt field, time-scale spin, status line (sim time,
    step count, lag, error). No long loop on the UI thread: a `QTimer`
    (interval = 1000/render_framerate) calls `engine.advance(wall_dt)` and then
    pushes `state.q` to the viewport and the joint panel (with the update
    guard, no feedback loop). The timer runs only while RUNNING and is stopped
    when the panel is hidden or the project closes.
4.  The viewport and TCP readout follow `engine.state` at each tick.
5.  Project close / robot change / model edit → engine reset to STOPPED with a
    new engine for the new model.

Tests (`tests/python/test_sim_panel.py`): free-fall pendulum in dynamic mode
(release from horizontal, gravity on): after 0.5 s sim time q matches a
reference RK4 run (independent numpy dynamics) within 1e-6; kinematic mode
slider change updates the state q and keeps qd = 0; mode switch rules;
buttons enable per state; timer stops on pause, stop and hide; the UI handler
returns fast (a 1 s simulated wall time at scale 1 does not block the Qt event
loop; check with a `QTimer` heartbeat); project close resets the engine.

Done when: tests green, graphify, commit `R6.3: manual simulation mode`.

## R6.4 --- Trajectory model and playback

Goal: timed waypoints, interpolation, playback with timeline, save in project.

Files: `app/sim/trajectory.py`, `app/panels/timeline_panel.py` (or inside the
sim panel), `app/project/project_data.py`, `app/project/serializer.py`, tests.

1.  `Waypoint(t, q)`; `Trajectory(waypoints, interpolation)` with
    `interpolation` in `"linear"`, `"cubic"` (natural cubic spline per DOF via
    numpy, zero-velocity ends optional flag `clamped=True` default True).
    Validation: strictly increasing t, q length n_dof, within joint limits
    (warning list, not an error), at least 2 waypoints. `q(t)`, `qd(t)`,
    `qdd(t)` (analytic for linear = piecewise constant qd, qdd = 0 except at
    knots; cubic analytic), `duration`. `t` is clamped to `[0, duration]`.
2.  Waypoint editor in the panel: table (t, q1..qn), add (from current q),
    delete, move up/down, "Capture current pose". Invalid rows are marked, not
    silently dropped.
3.  Playback: timeline slider (0..duration), Play / Pause / Seek / speed
    (0.1x..4x), loop checkbox. Kinematic playback sets q = `traj.q(t)` each
    tick (state machine from R6.1). Dynamic playback ("track with computed
    torque"): torque = RNEA(q_d, qd_d, qdd_d) + `Kp*(q_d - q) + Kd*(qd_d - qd)`
    (`Kp`, `Kd` per joint in settings, defaults 100 and 20), dynamics integrates
    the real state; the tracking error is shown in the status line.
4.  Project: `trajectory.json` inside `.robotproj` (format 1.2.0). Old files
    load with no trajectory. A project saved with a trajectory reloads equal.
    New robot model with a different n_dof invalidates the trajectory
    (warning in the log, trajectory dropped).

Tests (`tests/python/test_trajectory.py`, `test_ui_trajectory.py`): `q(t)`
equals waypoint q at each waypoint t (exact for both interpolations); linear
midpoint exact; cubic C1 continuity at knots (`qd` jump < 1e-9); clamped ends
have `qd(0)=qd(T)=0`; `qd` equals numeric derivative of `q` and `qdd` of `qd`
(1e-5); seek returns the exact interpolated q; kinematic playback end state
equals the last waypoint; dynamic tracking of a smooth trajectory on the 2R
robot ends with RMS error < 1e-3 rad (gravity on); project save/load round
trip; format 1.1.0 file still loads; n_dof change drops the trajectory.

Done when: tests green, graphify, commit `R6.4: trajectory and playback`.

## R6.5 --- State tracking (ring buffers)

Goal: record signals during a run with bounded memory.

Files: `app/sim/recorder.py`, settings key, tests.

1.  `RingBuffer(capacity, width)`: numpy preallocated `(capacity, width)` array
    plus a time array; `push(t, row)`, `view()` returns the data in time order
    (copy-free where possible, a copy is allowed when wrapped), `clear()`,
    `__len__`, `dropped` counter (samples overwritten).
2.  `Recorder(engine, model)`: signals per sample: `q`, `qd`, `qdd`, `tau`
    (applied or RNEA tau for kinematic mode if "estimate torque"), TCP position
    (3) and orientation (quaternion w,x,y,z, 4, sign-continuous), TCP linear
    velocity (`J v` first 3, from `jacobian_tcp` and `qd`), `ke`, `pe`, `total`,
    `power` (per DOF and sum, `joint_power`), `sigma_min` and manipulability
    (`analyze_singularity`). Names/units in one table `SIGNALS` (name, width,
    unit, source). Sampling decimation `record_every` (default 1 step) so the
    sample rate can be lower than the physics rate (setting).
3.  Memory cap: setting `sim/record_memory_mb` (default 64, min 1, max 2048);
    capacity = cap / (bytes per row); when full the oldest samples are
    overwritten (default) or recording stops (`overwrite=False`), flag in the
    status line.
4.  `Recorder.export_arrays()` → dict name → `(t, data)` numpy arrays in time
    order (used by R6.6 and R6.7).
5.  No unbounded growth: a 60 s run at 1 kHz for 6 DOF stays under the cap.

Tests (`tests/python/test_recorder.py`): ring buffer wrap keeps the last
`capacity` samples in order and `dropped` counts the overwritten ones; capacity
from the memory cap is exact; `overwrite=False` stops; signals equal an
independent recomputation (KE+PE, TCP position from numpy FK, power = tau·qd)
at 1e-9; quaternion sign continuous over a full rotation; decimation keeps
every nth; memory stays under cap after 100 000 pushes (check array sizes,
`tracemalloc` allowed).

Done when: tests green, graphify, commit `R6.5: state tracking`.

## R6.6 --- Graph suite

Goal: plots of every recorded signal, linked to the timeline.

Files: `app/panels/graph_panel.py`, `app/graphs/series.py`, `requirements.txt`,
`docs/graphs.md`, tests.

0.  Library proof build (Windows, before writing UI): `pip install pyqtgraph`,
    smoke test offscreen (`PlotWidget`, plot 1e5 points, `grab()` succeeds),
    note the version. If it works, add `pyqtgraph>=0.13` to `requirements.txt`
    and use it. If it fails, use matplotlib with the `QtAgg` canvas
    (already installed) and write the reason under "Decisions". Keep the plot
    calls behind a thin `app/graphs/plot_backend.py` so the choice is one file.
1.  `app/graphs/series.py`: pure functions `graph_data(recorder_arrays, kind,
    **opts)` → list of `Series(name, unit, t, y)` for kinds: `joint_pos`,
    `joint_vel`, `joint_acc`, `joint_torque`, `tcp_xyz`, `tcp_orientation`
    (quaternion and roll-pitch-yaw), `tcp_path_3d` (x, y, z arrays),
    `energy` (ke, pe, total), `manipulability` (sigma_min and manipulability),
    `power` (per joint and sum). Decimation for display: at most 5000 points
    per series by min/max bucket decimation (peaks kept).
2.  `GraphPanel` dock "Graphs" (View toggle, hidden at start): graph kind combo,
    joint checklist, auto-range, pause (freeze the view), clear; live update
    every 100 ms while RUNNING and the dock is visible (timer stops when
    hidden); a vertical cursor line linked to the timeline (moving the cursor
    seeks the trajectory; seeking moves the cursor); legend with units; "TCP
    path 3D" opens as a polyline in the PyVista viewport (not in the 2D plot).
3.  Plotting never blocks: data is taken from the recorder snapshot on the UI
    thread at most every 100 ms and is decimated; no per-sample UI call.

Tests (`tests/python/test_graph_series.py`, `test_ui_graphs.py`): each kind
returns the right series count, names, units and values (compare with the
recorder arrays and independent numpy); min/max decimation keeps the global
max and min and never more than 5000 points; offscreen panel: all kinds draw
without error after a run, cursor <-> timeline link both directions, the update
timer stops when hidden, clear empties the plot.

Done when: tests green, graphify, commit `R6.6: graph suite`.

## R6.7 --- Data export

Files: `app/sim/export.py`, `app/main_window.py` (File/Export menu), tests.

1.  `export_csv(recorder_arrays, path, names=None, meta=None)`: one CSV, header
    comment lines (`# robot`, `# date`, `# integrator`, `# dt`, `# gravity`,
    `# app version`, `# git commit` when available), column header with units
    (`q1 [rad]`), rows in time order. Multiple signals of different rates →
    separate files with suffix or a joined table on the finest time base
    (choose joined, empty cells for missing).
2.  `export_npz(recorder_arrays, path, meta)`: `np.savez_compressed` with
    arrays `<name>_t`, `<name>`, and a JSON string `meta`.
3.  Export one graph (the series shown) or the whole run (all signals), from
    the Graph panel and the File menu; file dialogs; export runs in a worker
    for large buffers.
4.  Re-import helpers `load_csv`, `load_npz` (used by the tests and R6.8).

Tests (`tests/python/test_export.py`): CSV re-read equals the buffers at
float repr precision (`rtol 1e-12`); NPZ re-read equal exactly; metadata present
and correct; names filter; empty recorder → clear error; units in headers.

Done when: tests green, graphify, commit `R6.7: data export`.

## R6.8 --- R6 milestone and stage gate

`tests/python/test_milestone_r6.py` (offscreen `MainWindow`, real `core_bridge`):
1.  Import `six_dof_with_gripper.urdf`, TCP link `l6`.
2.  Define a trajectory of 4 waypoints within the joint limits (cubic, clamped),
    duration 4 s. Save the project, reopen it, trajectory equal.
3.  Kinematic playback to the end: final q equals the last waypoint (1e-9),
    all recorder signals have samples.
4.  Dynamic playback with computed torque, gravity on: tracking RMS error
    below 1e-3 rad; energy balance check: `ΔE_total` over a gravity-free
    free-motion run with zero torque and zero friction, |dE|/E0 < 1e-8 (RK4).
5.  All graph kinds draw offscreen.
6.  Export CSV and NPZ of the whole run; re-import; arrays equal; TCP position
    in the file equals independent numpy FK of the recorded q (1e-9).
7.  Memory: recorder arrays below the configured cap; no running timers after
    stop/close.

Stage gate = `docs/MASTER_PLAN.md` section 1.3. In addition:
-   Docs, one page each: `docs/simulation.md` (core, state machine,
    integrators, modes), `docs/trajectory.md` (model, interpolation, tracking,
    project format 1.2.0), `docs/graphs.md` (signals table, units, library
    choice, export formats).
-   Record gate results with numbers in `PROGRESS.md` → "Current"
    (item 5 GPU equivalence: n/a, R5 covers it).
-   Commit `R6: stage milestone test and docs`, tag `stage-R6-done` locally, no
    push (no remote). Stage status `stage-ready-for-review`. R7 spec is not
    written yet, so STOP and tell the user.

---------------------------------------------------------------------
