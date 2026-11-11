# GPU batch IK: go / no-go (R5.4)

Verdict: **NO-GO**. The GPU IK stays in the repository as a prototype behind the flag
`GPU_IK_EXPERIMENTAL` (`gpu_bridge.GPU_IK_EXPERIMENTAL`). The compute backend does
not use it. The CPU multi-thread `core_bridge.solve_ik_batch` is the product path.

## Rule (R5 spec, R5.4 item 4)

Keep GPU IK only if BOTH hold, at 1e5 targets, end-to-end including the CPU polish:

-   (a) at least 5x faster than `cb.solve_ik_batch` on all CPU threads;
-   (b) final convergence rate within 1 percentage point of the CPU rate.

## Numbers (1e5 targets, median of 5 runs after warm-up)

Setup: 6-DOF fixture (`six_dof_with_gripper.urdf`, tcp `l6`), targets = FK of random q
inside the limits, start = `q_true + U(-0.5, 0.5)/sqrt(dof)` clipped to limits. Tool:
`tools/bench_ik.py`, data: `benchmarks/r5_ik_2026-11-11.json`.

| Path                               | End-to-end (ms) | Converged            |
|------------------------------------|-----------------|----------------------|
| CPU fp64, 32 threads               | 82.8            | 99.945 % (99945/1e5) |
| GPU fp32 + CPU fp64 polish         | 65.8 (runs 66.4, 67.2, 65.8, 56.6, 45.9) | 99.955 % (99955/1e5) |

GPU run split (last run): host pack + H2D + kernel + D2H = 13.3 ms (kernel only
1.8 ms), CPU polish = 30.1 ms. GPU alone (before polish) converged 99902 of 1e5
(99.902 %).

-   Rule (a): speedup = 82.8 / 65.8 = **1.26x** (best single run 1.8x). Needs 5x. **Fails.**
-   Rule (b): rate gap CPU minus GPU = -0.01 points (GPU slightly better). **Holds.**

Run-to-run note: GPU end-to-end times fell from 66 ms to 46 ms over the 5 repeats
(clock ramp-up). Even the best run is far below 5x.

## Why no-go

The fp32 kernel is fast (1.8 ms of device time for 1e5 targets, 99.9 % converged on
its own), but the mandatory CPU fp64 polish costs about 30 ms for 1e5 items: every item
needs at least one fp64 FK and error check plus Jacobian work, multi-threaded. The CPU
solver itself is already quick on this machine (32 threads, 0.8 us per target
amortised), so the polish is about a third of the full CPU solve. Even with a zero-cost
GPU, the speedup is bounded by 82.8 / 30.1 = 2.7x. The 5x rule cannot be met with the
polish as specified. Packing, transfers and widening add 11 ms more.

## Machine and versions

-   CPU: AMD Ryzen 9 8940HX, 32 threads (OMEN 16).
-   GPU: NVIDIA GeForce RTX 5060 Laptop, SM 12.0, 8150 MiB, driver reports CUDA 13.1.
-   CUDA toolkit 13.5 (nvcc `cuda_13.5.r13.5`), MSVC 19.44, SASS-only `sm_120`.
-   Windows 11 (10.0.26300), Python 3.13.15, numpy 2.5.4.
-   Commit at run time: `3b5796e` plus the uncommitted R5.4 working tree.

## Prototype summary (for a later revisit)

-   `cuda/ik_kernel.cuh/.cu`: one thread per target, base frame, `J[6][NMAX]` with
    NMAX 6 or 12 in registers, 21-entry `J J^T`, unrolled 6x6 Cholesky, lambda_min
    5e-3, pivot guard, uniform step scaling (0.2 rad / 0.05 m), limit clamp, accept or
    reject with lambda x10 or /2, 100 iteration cap, stall rule, NaN to status NAN,
    128 threads/block, `__launch_bounds__(128)`.
-   `cuda/ik_gpu.hpp/.cpp`: chunking from the R5.1 budget, first chunk timed then scaled
    to about 50 ms per launch, OOM halving, CPU fallback (no polish).
-   `cuda/gpu_bridge.cpp`: `gpu_bridge.solve_ik_batch(arrays, lo, hi, targets, q0,
    options=None, chunk=0, n_threads=0)` returns the CPU dict plus `info`
    (`gpu_ms`, `kernel_ms`, `polish_ms`, `n_gpu_converged`, `n_polish_converged`,
    `chunks`, `chunk`, `backend`, `fallback_reason`).
-   Limits: at most 12 DOF; `null_gain` not supported; weights must be uniform inside
    the position triple and inside the rotation triple (the GPU works in the base
    frame); GPU tolerance is `max(tol, 1e-4)`.
-   Ideas that could change the verdict (not done): skip the polish for items whose GPU
    error is already below 1e-6 (needs an fp64 residual on the GPU or a cheap host check
    instead of a full `solve_ik`), or accept an fp32 result tolerance of 1e-4. Both
    change the spec rule, so they need a user decision.

## Tests

`tests/python/test_ik_batch.py` (GPU part, skipped without a device): converged items
FK error below 1e-6 against numpy FK on three robots; rate within 1 point of the CPU on
a 1e4 subset; NaN inputs give status NAN; forced small chunks give identical results;
info counts consistent; bad inputs give ValueError.
