# IK and Analysis panels (R4.9)

Files: `app/panels/ik_panel.py`, `app/panels/analysis_panel.py`,
`app/panels/num_field.py`, `app/kin_utils.py` (`CompiledCache`),
`app/main_window.py`, `app/viewport/robot_view.py` (target triad).
Both docks are tabbed with the joint dock, hidden at start, and toggled from
the View menu (checkable). They clear when the project closes or the robot has
no joints, and rebuild their compiled / dynamics model on `robot_changed` or a
physical-property edit. A cache keyed on a change counter (`CompiledCache`)
avoids recompiling on a q change.

## Inverse Kinematics dock

- Target: x y z [m], roll pitch yaw [deg] (`Rz Ry Rx`), button "Use current
  TCP" (fills the fields from the current TCP), checkbox "Position only",
  damping, max iterations, number of starts (default 30).
- A target triad is drawn in the viewport (`RobotView.set_target_pose`) while
  the target is valid, and hidden when cleared. No dragging in R4 (R6 backlog).
- "Solve": runs `cb.solve_ik_multi` in a `TaskRunner` worker. The button is
  disabled and the status shows "Solving..." until the result arrives. A result
  for a stale robot (model changed meanwhile) is dropped.
- Table: `#`, `Distance` (inf-norm to the current q), `Pos error`, `Rot error`,
  sorted by distance. No solution: status "No solution found", no popup.
- Preview vs apply. Selecting a row PREVIEWS it: the model q is written, the
  joint panel and viewport update, the project is NOT dirty, and the q from
  before the first preview is remembered. "Cancel preview" restores that q.
  "Apply" commits the previewed q (project dirty, log line, `applied` signal);
  Cancel after Apply does not undo it. Moving a joint by hand ends the preview.

## Analysis dock

Jacobian / singularity: mode combo (full / position / angular), threshold
(default 1e-3). Shows `sigma_min`, `sigma_max`, condition, manipulability and a
red "NEAR SINGULAR" label (green "ok" otherwise). Updates on every q change in
the UI thread.

Dynamics: gravity comes from `SimulationSettings.gravity` of the project
(default `0 0 -9.81`; edit it there, not here). `qd` and `qdd` entry rows (one
field per DOF, default 0). Torque table columns: DOF, Joint, Gravity, Coriolis,
`M*qdd`, Total, Limit, Ratio. Total = `cb.inverse_dynamics` at the current q,
qd, qdd; the three parts add up to it. Limit and Ratio come from
`check_torque_limits`; the Total, Limit and Ratio cells are red when the limit
is exceeded. "Forward dynamics" fills `qdd` with the free acceleration
(`tau = 0`). The warning "mass not set" (and a disabled forward dynamics
button) appears when a link under a non-fixed joint has mass 0; a massless root
link is fine.

Energy: KE, PE and total for the current q and qd.

All numeric fields (`NumField`) accept scientific notation (`1e-3`). Invalid
text keeps the previous value and turns the field red.
