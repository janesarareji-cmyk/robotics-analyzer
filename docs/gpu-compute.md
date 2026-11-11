# GPU compute (R5)

Code: `cuda/gpu_manager.cu/.cuh`, `core/compute/budget.hpp`, `cuda/gpu_bridge.cpp`
(pybind11 module `gpu_bridge`), `app/compute/backend.py`, `app/compute/budget.py`.

## Modules

-   `core_bridge` never links CUDA. The CUDA code lives in the separate optional
    module `gpu_bridge`. If it cannot be imported, `app.compute.backend` uses the CPU.
-   `gpu_bridge` API: `device_info()`, `mem_info()`, `is_dead()`, `fk_batch(arrays, q,
    precision, frame)`, `sample_workspace(arrays, lo, hi, options, cancel, progress)`,
    `sample_q`, `solve_ik_batch` (experimental, see below), `set_fake_oom(n)` (test hook),
    `release_buffers()`.
-   Inputs are the frozen compiled arrays (`cb.compile_arrays(model)`, R4.1), never the URM.

## GPU manager (`gpu_manager`)

-   `query_device()` and `mem_info()` never throw: no device or driver error gives
    `available = false` plus an `error` string.
-   Target: RTX 5060 Laptop, SM 12.0, SASS-only build (`CMAKE_CUDA_ARCHITECTURES 120`).
    A driver that is too old shows up as an unavailable device; the USER must update
    the NVIDIA driver.

## Budget rule

`compute::chunk_size(free, user_budget, persistent, bytes_per_sample)`:

    budget = min(user_budget, 0.6 * free - 256 MiB) - persistent
    chunk  = floor(budget / (2 * bytes_per_sample))     (2 = double buffering)

Clamped to 65536..4194304 samples, rounded down to a multiple of 256. It returns 0
("use the CPU") when the budget is not positive or the chunk is below 65536.
Nothing is hard-coded: free VRAM is read at run time. The user budget is the setting
`compute/vram_budget_mb` (default 6144 MiB, range 256..7168, always below the 8 GB
ceiling). `app.compute.budget.max_batch()` gives 1e6 when there is no GPU.
`info["vram_peak_bytes"]` of a run never exceeds `info["budget_bytes"]`.

## Error policy

-   Out of memory (not sticky): free, halve the chunk, retry. Below 65536 samples the
    run is handed to the CPU (`info["backend"] == "cpu"`, `fallback_reason` set).
-   Sticky CUDA error (context corrupt): `gpu::mark_dead`, `is_dead()` is true for the
    rest of the process, every later call goes to the CPU.
-   `CUDA_CHECK` throws `gpu::CudaError` (a `RuntimeError` in Python).
-   Bad input (non-finite limits, grid too large, budget too small for the voxel grid
    with an explicit budget) is a `ValueError`, never a fallback.
-   `set_fake_oom(n)` makes the next n allocations fail, to test the fallback.

## Python backends (`app/compute/backend.py`)

`get_backend(preference, settings)` returns `CpuBackend` or `GpuBackend`. Order: env
`ANALYZER_FORCE_CPU=1` (always wins) > argument > setting `compute/backend` > `"auto"`.
Preferences: `auto`, `cpu`, `cuda`. `backend.reason` says why.

Both backends have `fk_batch`, `sample_workspace`, `solve_ik_batch`, `device_info`. Result
dicts carry `info` with `backend`, `fallback`, `fallback_reason`. `GpuBackend._run` re-runs
a call on the CPU when the GPU cannot finish it (OOM below the minimum chunk, sticky
error, dead GPU) and logs a warning; `ValueError` propagates.

Batch IK: GPU IK is NO-GO (`docs/gpu-ik-decision.md`). `GpuBackend.solve_ik_batch` calls the
CPU multi-thread solver unless `USE_GPU_IK` is set (off).

Precision: `docs/gpu-precision-decision.md`. GPU FK default fp32, fp64 selectable. The
CPU backend always computes in double.
