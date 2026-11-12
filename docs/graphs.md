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
