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
