# Analysis overlays (U6)

Overlays draw analysis results in the 3D view. They live in the **Overlays** dock (Analyze mode, Ctrl+6)
and in View -> Analysis overlays. Each one is a toggle, off by default.

## How they work

-   Every overlay has a toggle, an opacity slider and a colour bar (legend) in its card.
-   The result is computed in a worker (progress bar, Cancel). While it runs, the window stays usable.
-   The result is kept per robot structure and tool (a hash of the compiled model) and of the
    collision matrix. Switching an overlay off and on again does not compute again; "Recompute" does.
-   A robot change cancels running jobs and drops the old result. Changing a tool offset or a joint
    limit makes a new hash, so the next toggle computes again.
-   Code: `app/analysis/overlays.py` (`Overlay`, `OverlayManager`), `app/panels/overlay_panel.py`,
    drawing in `RobotView.set_overlay_voxels`, `set_overlay_outline`, `clear_overlay`.

## Reach map

Voxels the tool (TCP) can reach, coloured by dexterity (mean Yoshikawa manipulability of the samples in
the voxel, scaled to its 95th percentile; yellow = free to move). It uses the same sampling and voxel keys
as `analyze_workspace`, so the voxel count is identical to the Workspace dock for the same samples, voxel
size and seed.

-   Quality: Fast (150 000 samples, reach/25 voxels), Normal (400 000, reach/40), Fine (1 000 000, reach/60).
    ur5, Normal: 73 933 voxels in 1.3 s.
-   Slice at a height: only the voxel layer at the chosen z (base frame) is drawn, a table-height map.
-   Live probe: while a target is dragged (Target mode) the voxel under it is outlined, green = reachable,
    red = no sample reached it.
-   The numbers behind it (`app/analysis/batch_jacobian.py`): the Jacobian of a whole batch from
    `cb.fk_batch` by forward differences (step 1e-6), singular values by the eigenvalues of J J^T.
    Agrees with `cb.jacobian_tcp` to 1e-5; 1e6 samples in 3.4 s on the CPU.

## Singularity zones

Voxels where the arm is singular: red = sigma_min (smallest singular value of the 6 x n Jacobian) below the
threshold, orange = below twice the threshold. The threshold starts at `SINGULAR_WARN` (0.05), the value of the
bars in the Joints and Matrices panels, the orange target ghost (Target mode) and the orange curve segments
(Curve dock). The slider (0.005 to 0.3, log scale) changes only the overlay; "Reset" goes back to 0.05.

-   Per voxel the smallest sigma_min per kind is stored, so threshold and kind boxes redraw at once.
    A voxel is red when ANY sample inside it is singular, so with every orientation counted nearly each
    voxel is red when the wrist is included: the wrist kind (tool orientation) exists at almost every
    position. It is therefore off at the start; tick "Wrist" to see it.
-   Kinds (robots with >= 6 DOF whose first and last three joints are revolute, otherwise all "general"):
    -   wrist: the last three axes are nearly dependent (sigma_min of the axes matrix < 0.2), e.g. q5 = 0 on a UR.
    -   elbow: shoulder to wrist-centre distance within 0.4 % of its maximum or minimum (arm stretched or folded).
    -   shoulder: the wrist centre is within 4 % of the reach of the smallest distance to the first axis.
    -   general: singular for another reason.
    The wrist centre is the closest point of axis n-3 and axis n-2 (spherical wrist: the wrist point, e.g.
    kuka_iiwa; offset wrist: the crossing of the two forearm axes, e.g. ur5, xarm6). The label is a heuristic:
    the first match in the order wrist, elbow, shoulder.
-   Quality: Fast 200 000, Normal 600 000, Fine 2 million samples. 1e6 samples with classification: 4.4 s (ur5,
    CPU, batch Jacobian). The GPU is not used: there is no GPU Jacobian kernel (GPU IK was NO-GO, see
    `docs/gpu-ik-decision.md`), and the batch Jacobian runs on all CPU threads.
-   "Export CSV..." writes x, y, z (world, m), sigma_min and kind of the red and orange voxels shown.
-   Checked: ur5 poses with q5 = 0 are wrist zones, q3 = 0 poses are elbow zones, a planar 2R arm
    (position metric) shows the singular circles at l1 + l2 and |l1 - l2| within one voxel.

## Joint load

A torque bar (T) and a speed bar (S) next to every joint of the arm, drawn over the 3D view
(`app/viewport/load_bars.py`, mouse clicks pass through). Green up to 70 % of the limit, orange up to 100 %,
red above; a bar is full at 100 %.

-   Torque: `cb.inverse_dynamics` (RNEA) at the current q, qd, qdd with the project gravity; ratio = |tau| / effort
    limit of the joint. Speed: |qd| / velocity limit. A limit of 0 means "no limit" (bar empty).
-   Slider, Cartesian jog and TCP drag have qd = qdd = 0, so the bars show the static gravity torque
    (equal to `cb.gravity_torque`). Kinematic playback and simulation give qd, qdd from the trajectory or the
    state. The bars follow the camera when you rotate or zoom.
-   Refresh at most 30 Hz, the newest state is always drawn after the pause. Cost of one update (RNEA + ratios):
    0.2 ms on ur5. Gripper joints are not drawn.
