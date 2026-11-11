# GPU kernels (R5)

Code: `cuda/fk_kernel.cu`, `cuda/fk_device.cuh`, `cuda/workspace_kernel.cu`,
`cuda/workspace_gpu.cpp`, `core/compute/philox.hpp`.

## FK kernel (`fk_tcp_kernel<T>`, T = float or double)

-   One thread per sample (256 threads per block). The TCP pose is walked along the active
    path root to tip with the rotation and position in registers.
-   Joint tables sit in shared memory: `FK_TAB_PER_JOINT = 29` values per joint (pre 12,
    post 12 as 3x4 row-major, axis 3, mult, offset), then tool 12; plus an int table
    (type, src_q). At most `FK_MAX_JOINTS = 64` joints on the path. Fixed joints are folded
    (pre = pre * post, post = I). Mimic joints use `mult * q[src] + offset`.
-   Layout is SoA, element e of joint j at `e * n + j`. Input q: `[n_dof][ld]`. Output:
    `[12][ldo]` = px py pz r00..r22. Memory access is coalesced over samples.
-   The kernel works in the BASE frame; the host multiplies the robot base in double
    (`frame="world"`).
-   Host side (`gpu_bridge.fk_batch`): pooled pinned and device buffers (pool cap 1 Mi
    samples per slot), 2 streams, chunked pipeline.

## Precision

-   Default fp32, fp64 selectable (`precision="fp64"`). Rule and numbers:
    `docs/gpu-precision-decision.md`. fp32 tolerances: pos <= 8 n 1.19e-7 R, rot <=
    8 n 1.19e-7, RMS <= tol/10, orthonormality <= 4 n 1.19e-7. Reference: CPU double on
    `(double)(float)q`. fp64 GPU vs CPU <= 1e-12.
-   Workspace sampling always uses fp32. Built SASS-only for sm_120, no `--use_fast_math`.

## Workspace kernel (`ws_kernel`)

-   q is generated on the device (no q buffer): Philox4x32-10, counter
    `(lo32(sample), hi32(sample), dof_block, 0)`, key `(lo32(seed), hi32(seed))`; dof d uses
    lane `d % 4` of block `d / 4`. `u = (x >> 8) * 2^-24`, `q = fma(u, hi - lo, lo)` in float.
    The same header runs on CPU and GPU, so q is identical and independent of chunk size.
    Grid mode: mixed radix, dof 0 fastest.
-   The TCP is voxelized in the base frame into an occupied bitset (`atomicOr`) and optional
    per-voxel counts (`atomicAdd`); samples outside the grid are counted in `n_outside`.
    Optional points go to per-chunk buffers (2 device + 2 pinned, one pair per stream);
    only the first `max_points` samples write a point.
-   Occupied voxel list: bit count and compaction with CUB (`DeviceReduce`, `DeviceSelect`).

## Launch time (WDDM TDR) rule

Windows kills a display-driver launch that runs too long. Both host loops time the first
chunk and scale later chunks so each launch stays near 50 ms (`kTargetKernelMs`). Debug
builds synchronise after every launch. A forced chunk (`options["chunk"]`) is not scaled.

## GPU IK (experimental)

`cuda/ik_kernel.cu` (fp32 solver) is kept as a prototype behind `GPU_IK_EXPERIMENTAL`;
NO-GO, see `docs/gpu-ik-decision.md`.
