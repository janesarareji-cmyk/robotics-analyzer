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

-   Per voxel the smallest sigma_min per kind and the largest sigma_min (with its kind) are stored, so threshold, kind
    boxes and the statistic redraw at once without a new compute.
-   "Show": **Unavoidable (best tool orientation)** (default) paints a voxel when even its best sample is singular: the
    zones no tool orientation can avoid (ur5: the outer shell at full stretch and the cylinder around the first axis,
    about 5 % red and 11 % orange of the voxels). **Any tool orientation (worst)** paints a voxel when some sample in
    it is singular; with the wrist included that is nearly every voxel (the wrist singularity exists at every
    position), so it is only useful with the kind boxes (e.g. only Elbow).
-   "Cut open" hides the half of the cubes with base y > 0 so the robot inside stays visible (reach map too).
-   Samples per voxel matter: the "best" value is biased low with few samples. Normal uses reach/25 voxels and 1 million
    samples (about 30 per voxel on ur5); Fast 300 000 and reach/18; Fine 3 million and reach/40.
-   Kinds (robots with >= 6 DOF whose first and last three joints are revolute, otherwise all "general"):
    -   wrist: the last three axes are nearly dependent (sigma_min of the axes matrix < 0.2), e.g. q5 = 0 on a UR.
    -   elbow: shoulder to wrist-centre distance within 0.4 % of its maximum or minimum (arm stretched or folded).
    -   shoulder: the wrist centre is within 4 % of the reach of the smallest distance to the first axis.
    -   general: singular for another reason.
    The wrist centre is the closest point of axis n-3 and axis n-2 (spherical wrist: the wrist point, e.g.
    kuka_iiwa; offset wrist: the crossing of the two forearm axes, e.g. ur5, xarm6). The label is a heuristic:
    the first match in the order wrist, elbow, shoulder.
-   1e6 samples with classification: 4.4 s (ur5,
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

## Playback bar, screenshot and video

-   **Playback bar**: when a motion is loaded (Simulation dock, Curve, Target) a bar appears at the bottom of
    the 3D view: play / pause, one-frame step back and forward (1/30 s), the scrub slider, the time and Loop.
    It drives the same player as the Simulation dock (`SimPanel.play_trajectory`, `seek_trajectory`,
    `step_trajectory`); no second playback logic. The bar hides when there is no motion.
-   **Screenshot**: toolbar button "Screenshot" (also File -> Export -> Screenshot) saves the 3D view as PNG, with the
    overlays as shown.
-   **File -> Export -> Playback video...** opens the export dialog: format, frame rate (default 30), size, file.
    Frame `i` shows the time `i / fps`; the frame count is `round(duration * fps)` (a 2 s motion at 30 fps gives 60
    frames). The frames are rendered on the GUI thread (one per timer tick, the window stays alive), and encoded in a
    worker with a bounded queue. Cancel stops at once and removes the partial file.
-   Formats (`app/sim/video_export.py`), only those that work on the machine are offered:
    -   MP4: needs `imageio-ffmpeg` or an `ffmpeg` program on the PATH (frames go to ffmpeg as raw video, H.264).
        Neither is installed here and the app adds no dependency, so on this machine MP4 is not offered.
    -   GIF: Pillow (always present with PyVista). The GIF delay has 10 ms steps: 30 fps plays at 33 fps.
    -   PNG frames: a folder `frame_0001.png`, ...
-   Size: "Viewport size" is a grab of the view including the joint load bars; "1280 x 720" and "1920 x 1080" scale the
    view up (and crop to the aspect), so make the 3D view large for a sharp result.

## Milestone and review numbers (U6)

-   `tests/python/test_milestone_u6.py` (ur5): the stretched pose (q3 = 0) is red in both statistics and sigma_min is
    below 1e-6; a target there has the warn status with every valid IK solution near singular (orange ghost); a line
    curve from the start pose to it: 52 samples, 1 under the threshold (flagged by the curve check, in a red or orange
    overlay voxel), 23 bad (the straight line leaves the reachable set); a 2 s playback exports 30 frames at 15 fps;
    the CSV (1390 voxel rows in Fast quality) holds the voxel of the target. GIF stores identical frames once, so a
    blank offscreen test video has one image with the full duration.
-   Screenshots in `docs/screens/u6/` (real window, `tools/u6_shots.py`), all read. GUI skills: UI Design Review and
    Accessibility Review. Fixed from them: red-everywhere singularity picture (statistic "unavoidable", coarser voxels),
    reach map hid the robot (cut open), joint load plates overlapped and leader lines crossed the dots (three-pass
    drawing, offset), bars stale after a resize or a video seek (redraw on resize, flush before a grab), speeds lost on
    a seek (kept per pose), controls not following options (sync), long overlay cards (controls only while the
    overlay is on), a zero-length curve crashed the tube (end marks only), dock title repeated by the tab (title bar
    hidden while a tab shows the name), model tree clipped (dock 340 px in Model mode), small playback glyphs.
    Text alternative: the load bars carry an accessible description with the percentages.
-   Not done: colour is the only difference between red and orange voxels (the legend names both, the CSV has the
    value); NVDA test; 200 % scaling pass; MP4 on this machine (no ffmpeg).
