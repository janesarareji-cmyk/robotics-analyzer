# Workspace analysis (R7.5)

Code: `app/analysis/workspace_analysis.py`, UI: `app/panels/workspace_panel.py` ("Analysis" group),
slice widget `SliceView` in `app/graphs/plot_backend.py`. CPU only (no GPU path: the R5 backend
has no batch Jacobian). VRAM budget is not touched.

## Method

`analyze_workspace(model, options, cancel=None, progress=None)`:

1. q uniform in the joint limits (numpy `default_rng(seed)`, chunks of 20 000), or an explicit
   `q` array (tests, grid studies).
2. TCP position in the BASE frame: `cb.fk_batch(cm, q, "base")`.
3. Per sample: `cb.jacobian_tcp` then `cb.analyze_singularity(J, mode)` -> sigma_min and
   Yoshikawa w (product of the nonzero singular values). `metric_mode`: `"position"` (rows 0-2,
   default; planar 2R gives w = l1 l2 |sin q2|) or `"full"` (6 rows, mixed units).
4. Voxel `floor((p - origin) / h)`, key `lin = (ix*ny + iy)*nz + iz` (same as R5). Per voxel:
   count, mean/min sigma_min, mean/min w. Sparse: sorted keys, memory ~ occupied voxels only.

Options: `n_samples` (2e5), `seed` (42), `voxel_size` (0 = reach/100), `metric_mode`, `threshold`
(1e-2, singular if the statistic of sigma_min is below it), `singular_stat` (`"min"` or `"mean"`),
`want_metrics` (False = occupancy only, fast), `bounds_lo/hi` (default `+-(reach + h)`), `q`.
Unknown key -> `ValueError`.

## Volume and error bound

`volume = filled voxels * h^3`. Counting every touched voxel over-estimates the boundary and
unsampled interior voxels under-estimate. `volume_err_bound = boundary voxels * h^3`, where a
boundary voxel is a filled voxel with at least one empty 6-neighbour (a conservative bound: the
true set differs from the filled set only in boundary cells, assuming dense sampling). For a
thin (one-layer) planar study every voxel is boundary, so use the bound only for 3D sets.
Measured: planar 2R annulus, h = 0.005 m, 3000 x 3000 q grid: area error 0.59 % (rule of thumb:
over-estimate ~ perimeter * 2h / pi).

## Singularity regions

`singular_mask` (per filled voxel) = statistic of sigma_min < `threshold`; `n_singular`.
Planar 2R: singular voxels sit on the two circles r = |l1 - l2| and r = l1 + l2 (q2 = 0, pi).

## Views

* 3D cloud: voxel centers coloured by the chosen metric (`display_changed` carries `scalars`,
  drawn by `RobotView.set_workspace_points(points, transform, scalars)`).
* 2D slice: `slice_image(res, field, axis, offset)` -> NaN where empty, extent in meters;
  `field` in `mean_smin, min_smin, mean_w, min_w, count, singular`. Shown with a colour bar.
  `voxel_value(res, ix, iy, iz, field)` is the direct lookup (tests: slice == lookup).
* Empty workspace: `res["message"]` is set and shown in the panel; `filled == 0`.

Heavy work runs in a `TaskRunner` worker; Cancel stops at the next chunk (partial result,
`cancelled=True`). `volume_from_occupancy(r5_result)` gives the volume of an R5 occupancy result.

## Numbers (Ryzen 9 8940HX, one thread, Python loop over `cb.jacobian_tcp` + `cb.analyze_singularity`)

~5.5 us per sample (2.3 us Jacobian + 3.1 us SVD): 1e5 samples with metrics 0.39 s total on the
planar 2R, 1e6 about 4 s. No C++ batch Jacobian was needed.
