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
