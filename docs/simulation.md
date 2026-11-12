# Simulation (R6.1 - R6.3)

Code: `app/sim/` (no Qt) and the Simulation dock `app/panels/sim_panel.py`.

## Core
- `SimState` (`state.py`): `t, q, qd, qdd, tau, tau_inv, tcp (4x4), energy {ke, pe, total}`; one array entry per DOF.
- `SimEngine(model, settings)` (`engine.py`): compiles the model, builds the dynamics model, owns the state.
  - `step(dt)`: one integrator step, then `qdd` by forward dynamics (`cb.forward_dynamics`, gravity, optional friction and armature). Torque comes from `set_tau` or from the optional `torque_fn(t, q, qd)` (feedback control).
  - `advance(wall_dt)`: runs `wall_dt * time_scale` sim seconds in steps of `dt`, at most `max_steps_per_advance` steps; the rest is reported in `lag`. `tau_inv`, tcp and energy are filled once per advance (`refresh`), so the step stays cheap.
  - `reset(q0, qd0)` sets the initial state and stops the clock. `stop()` restores the initial state. `set_state(t, q, qd, qdd=None)` replaces the state (playback).
  - A `RuntimeError` from the dynamics (M not positive definite, adaptive step underflow) is stored in `engine.error` and pauses a running engine; the state is left unchanged.
  - `step_callbacks` run after every step or `set_state` (the Recorder hooks here); `reset_callbacks` after reset.

## State machine
`SimClock`: `STOPPED --start--> RUNNING <--pause/resume--> PAUSED`, and `stop` from RUNNING or PAUSED back to STOPPED. Any other transition raises `ValueError`. `on_state_changed(new, old)` callbacks drive the UI and the Recorder (a new run = STOPPED -> RUNNING clears the recorder).

## Integrators (`integrators.py`)
Common call `integrator(accel, t, q, qd, dt) -> (q_new, qd_new, dt_accepted)`.
- `euler`: semi-implicit Euler.
- `rk4`: classic RK4 on `[q, qd]` (default). Speed 20,500 steps/s on the 6-DOF fixture, so no C++ step was needed.
- `rk45`: Dormand-Prince, adaptive, settings `rk45_atol, rk45_rtol, rk45_dt_min, rk45_dt_max`; suggests `next_dt`.
Energy check (gravity-free, zero torque, zero friction, RK4, dt 1 ms, 1 s): |dE|/E0 = 6.5e-15 (`test_milestone_r6.py`).

## Modes (`mode.py`)
- KINEMATIC: the user sets q (joint panel or trajectory playback), `qd = qdd = 0` for manual edits; the run buttons are off. Optional finite-difference torque estimate from the last 3 slider samples.
- DYNAMIC: state evolves by forward dynamics from the applied torque (tau spin boxes) with gravity and friction toggles.
- Switching keeps q, sets qd = 0 and stops the clock.

## UI driving
The Simulation dock owns a `QTimer` (interval from `render_framerate`), active only while RUNNING and visible. Each tick calls `engine.advance` (or `player.advance` during trajectory playback). The UI thread never runs a long loop. Dock panel controls (dt, integrator, scale, gravity, friction) change the engine only, not the project `SimulationSettings`.

## Settings (`SimulationSettings`)
`integrator, physics_timestep, render_framerate, gravity, rk45_*, track_kp, track_kd, record_memory_mb, record_every, record_overwrite`. Friction and armature are engine settings, default zero (friction is not stored in URM).
