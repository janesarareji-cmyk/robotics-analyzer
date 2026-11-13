# U6 spec pack: analysis overlays, timeline, video export

Plan: `docs/plans/U3-U6-plan.md`. Common rules: `docs/specs/U3-spec.md` "Common rules" (prefix
`U6.<n>`, tag `stage-U6-done`). Needs U3-U5.

Goal: the analyzer shows where the robot can reach, where it is singular (red zones) and how
hard the joints work, all as 3D toggles; playback can be scrubbed and exported as video.

## Code facts

- `app/analysis/workspace_analysis.py`: `analyze_workspace(model, options, cancel, progress)` ->
  voxel result with fields (count, sigma_min min/mean, manipulability), `voxel_centers`, `field`,
  `slice_image`; `sample_metrics(cm, q, mode)`. GPU sampling via `app/compute/backend.py`
  (`get_backend`, workspace on `gpu_bridge`), CPU fallback.
- `RobotView.set_workspace_points`, `set_workspace_voxels(centers, voxel_size, ...)`.
- `SINGULAR_WARN` (U4a.3) shared threshold. Dynamics: `cb` RNEA (inverse dynamics), recorder signals
  (`app/sim/recorder.py`) include torque and qd.
- Playback/timeline from U4b.4.

## U6.1 Overlay framework (sonnet)

View -> "Analysis overlays" submenu + Analyze-mode panel "Overlays": each overlay is a toggle (off by
default), has an opacity slider and a legend (colour bar). Overlays compute in TaskRunner (progress,
cancel), cache per robot + tool + ACM hash, recompute button. `app/analysis/overlays.py` base class
`Overlay(name, compute(model, opts, cancel, progress) -> data, actors(data, view))`.
Tests `tests/python/test_overlays.py`.

## U6.2 Reach map (sonnet)

Overlay "Reach": voxels reachable by the TCP (existing workspace analysis), coloured by dexterity
(manipulability mean). Optional slice plane (height slider) to see a table-height map. Live: while the
U4a target is dragged, the voxel under it is outlined.
Tests: voxel count equals `analyze_workspace` on the same options.

## U6.3 Singularity zones (sonnet)

Overlay "Singularity": voxels with sigma_min min < threshold red (#ff3030), < 2x threshold orange,
else hidden. Threshold slider (log scale, default `SINGULAR_WARN`). Kind split when possible:
for robots with a spherical wrist (last 3 axes intersect within 1 mm, detect from the compiled model)
classify each singular sample as wrist (sigma_min of the wrist 3x3 block small), elbow (arm fully
stretched: shoulder-wrist distance near max), shoulder (wrist centre near the j1 axis); otherwise
"general". Checkboxes per kind. GPU sampling when available (`gpu_bridge`), CPU fallback; 1e6 samples
target < 5 s on GPU (print). CSV export (x, y, z, sigma_min, kind). Joint panel / matrix panel bar,
target ghost warning and curve warn segments use the same threshold value.
Tests `tests/python/test_singularity_overlay.py` (ur5: wrist singular set contains q5 = 0 poses;
elbow set contains stretched poses; planar 2R known singular circle radii found within voxel size).

## U6.4 Torque / speed overlay (sonnet)

Overlay "Joint load": during motion (slider, drag, playback) a small bar next to each joint in 3D
(billboard or 2D overlay anchored to the projected joint position): torque / effort limit and
speed / velocity limit, green -> orange (> 70 %) -> red (> 100 %). Torque from RNEA with current
q, qd, qdd (kinematic playback: from trajectory derivatives; jog: static gravity torque).
Refresh <= 30 Hz, cost < 2 ms (print).
Tests `tests/python/test_load_overlay.py` (static gravity torque equals `cb` RNEA with qd=qdd=0).

## U6.5 Timeline scrub + video / GIF export (sonnet)

Timeline under the viewport (if not already from U4b.4): scrub slider, frame step buttons, loop.
File -> Export -> "Playback video..." : MP4 (imageio-ffmpeg only if already installed; else check
`pip index` and do a Windows proof install per dependency rule; fallback GIF via Pillow if present,
else PNG frame sequence) at chosen fps (30), resolution (viewport size or 1920x1080), with overlays
as shown. "Screenshot" toolbar button -> PNG of the viewport. Runs in a worker with progress/cancel
(render on GUI thread, encode in worker).
Tests `tests/python/test_video_export.py` (frame count = duration * fps, file non-empty; GIF/PNG
fallback path).

## U6.6 Stage end

Milestone `tests/python/test_milestone_u6.py`: ur5 singular overlay on + reach overlay + target in a
red zone shows orange/red ghost + curve warn segments agree with overlay threshold; video export of
a 2 s playback; CSV export. Full gate, screenshots of each overlay, GUI skill check, Opus final review
of U3-U6, tag `v1.1`.
