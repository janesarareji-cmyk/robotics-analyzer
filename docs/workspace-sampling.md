# Workspace sampling (R5.3)

API: `core_bridge.sample_workspace(cm, lo, hi, options, cancel=None, progress=None)` (CPU,
double FK) and `gpu_bridge.sample_workspace(arrays, lo, hi, options, cancel=None, progress=None)`
(CUDA fp32, CPU fallback). Same result dict. `core_bridge.CancelToken` works for both
(`cancel()`, `is_cancelled()`, `reset()`).

Options: `mode` ("random"|"grid"), `n_samples`, `seed` (42), `voxel_size`, `bounds_lo/hi`
(base frame; default `[-R, R]^3` + 1 voxel, R = reach bound), `want_counts` (True),
`want_points` (False), `max_points` (1e6), `budget_bytes` (0 = auto), `divisions` (grid),
`chunk` (test hook, multiple of 256), `n_threads` (CPU). Unknown keys raise `ValueError`.

Frame: the TCP is sampled in the BASE frame. The host never applies `base` here.

Sampling: Philox4x32-10, key = seed, counter = (lo32(i), hi32(i), dof_block, 0), lane d % 4
of block d / 4. `u = (x >> 8) * 2^-24`, `q = fma(u, hi_f - lo_f, lo_f)` in float (explicit
`fmaf`, bit identical on CPU and GPU). Samples depend only on the global index i, not on
chunk size or thread count. The CPU reference runs double FK on the float q.

Grid mode: `divisions[d]` per DOF, total = product (must be <= `n_samples`, else
`ValueError`). Index i -> k_d by mixed radix, DOF 0 fastest (`k_0 = i % div_0`,
`i //= div_0`, ...). `q_d = lo_f + k_d * step`, `step = (hi_f - lo_f) / (div_d - 1)`
(endpoints included); `div_d == 1` -> the center of the range.

Voxels: `floor((p - origin) / voxel)` per axis (double on the CPU, float on the GPU).
`lin = (ix * ny + iy) * nz + iz`. Outside points are counted in `n_outside`.
Caps: occupied bitset 1024^3 voxels (128 MB), counts 256 MB (67.1 M voxels); over a cap ->
`ValueError` naming the maximum. `want_counts=False` avoids the counts cap.

GPU vs CPU: points within fp32 error of a voxel boundary can land in different voxels.
Tests allow 0.1 % of the occupied voxels to differ (measured at most 3e-5).

Memory: persistent device data = bitset + counts + tables; no q buffer. `want_points` adds
2 device + 2 pinned buffers. Chunk = `compute::chunk_size(...)`, first chunk timed, later
chunks scaled to about 50 ms. `info["vram_peak_bytes"]` is the sum of live device
allocations made by the run (small CUB temp buffers of the bit count are not included).
Explicit `budget_bytes` too small for the grid -> `ValueError`. Automatic budget (not
enough free VRAM), out of memory below the minimum chunk, or a sticky CUDA error -> CPU
reference with `info["fallback_reason"]` set. Compaction of the occupied voxels runs on
the GPU (`cub::DeviceSelect::Flagged`) when it fits the budget, else on the host.

Occupied-voxel area: counting occupied voxels over-estimates a region by about
`perimeter * 2h / pi` (cells that touch the boundary). For the 2R annulus at h = 0.02 that is
about 2.4 % of 2*pi, so the test compares with the exact number of intersecting cells.
