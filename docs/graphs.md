# Graph suite (R6.6)

Dock "Graphs" (View menu, hidden at start). Plots the signals the `Recorder` (R6.5) stores.

- Library: pyqtgraph 0.14.0 (`pyqtgraph>=0.13` in `requirements.txt`). All plot calls are in
  `app/graphs/plot_backend.py` (`PlotView`); switching library touches only that file.
- Data: `app/graphs/series.py`, pure. `graph_data(recorder.export_arrays(), kind, joints, names)`
  returns `Series(name, unit, t, y)` lists. Kinds: `joint_pos`, `joint_vel`, `joint_acc`,
  `joint_torque`, `tcp_xyz`, `tcp_orientation` (qw qx qy qz + roll pitch yaw, R = Rz Ry Rx),
  `tcp_path_3d` (x y z), `energy` (ke pe total), `manipulability` (sigma_min, manipulability),
  `power` (per joint + sum). Joint unit strings come from `SIGNALS` ("rad|m": revolute|prismatic).
- Decimation: min/max per bucket, at most 5000 points per series, global peaks kept. The 3D path
  uses one common uniform stride so x, y, z stay aligned.
- Live view: a 100 ms `QTimer`, active only while the sim is RUNNING and the dock is visible.
  Each tick takes buffer views (`export_arrays(copy=False)`), decimates, redraws. No per-sample UI call.
- Controls: kind combo, joint checklist (joint kinds and power), auto-range, Pause (freezes the
  view, recording goes on), Clear (empties plot and recorder).
- Cursor: vertical line at the playback time. Dragging it calls `SimPanel.seek_trajectory`;
  playback / timeline seek moves it (`SimPanel.position_changed`). A guard prevents feedback.
- "TCP path 3D": not drawn in 2D. `GraphPanel.path_3d_changed` -> `RobotView.set_tcp_path`
  (yellow polyline in the PyVista viewport).
- Recorder lifetime: cleared when a new run starts (STOPPED -> RUNNING) and on reset / Clear;
  data survives Stop and Pause, so graphs and export still work after Stop.

## Recorded signals (R6.5)
`Recorder` (`app/sim/recorder.py`) stores one ring-buffer row per recorded step while the clock is RUNNING (kinematic playback included). `n` = DOF count; unit "a/b" with a bar = revolute/prismatic.

| Signal | Width | Unit (revolute or prismatic) | Source |
|---|---|---|---|
| q | n | rad or m | state.q |
| qd | n | rad/s or m/s | state.qd |
| qdd | n | rad/s^2 or m/s^2 | state.qdd |
| tau | n | N.m or N | applied torque (RNEA if torque estimate is on) |
| tcp_pos | 3 | m | FK tcp translation |
| tcp_quat | 4 | 1 (w,x,y,z) | TCP rotation, sign-continuous |
| tcp_vel | 3 | m/s | (J qd)[:3] |
| ke, pe, total | 1 each | J | kinetic, potential, sum |
| power | n | W | tau * qd per joint |
| power_sum | 1 | W | sum of power |
| sigma_min, manipulability | 1 each | mixed | singularity analysis of the TCP Jacobian |

Memory: `record_memory_mb` (1 - 2048, default 64) sets the ring capacity = bytes / (8 * (width + 1)). `record_every` decimates; `record_overwrite=False` stops recording when full. Milestone: 64 MB cap, buffer 67,108,704 bytes (at the cap by design, preallocated).

## Export (R6.7)
File > Export, scope "run" (all signals) or "graph" (the shown series, full resolution), format by file extension.
- CSV (`export_csv`, `load_csv`): `# key: value` metadata lines, a `# columns:` line (signal widths), a header row with units, joined on the union of time stamps (empty cell = no sample). Floats `%.17g`, exact round trip.
- NPZ (`export_npz`, `load_npz`): `<name>_t`, `<name>` arrays and a JSON string `meta` (`names`, robot, date, app version, git commit, integrator, dt, gravity, joint names).
Written in a worker task from copied arrays. Milestone: CSV and NPZ re-import equal to the recorder arrays (max diff 0), TCP position equals independent numpy FK within 1.7e-16 m.
