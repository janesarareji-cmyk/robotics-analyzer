# Stage R5 spec pack: GPU compute

Authority: `docs/MASTER_PLAN.md` section 4 (workflow, gate, push in section 1).
Review: the R5.2 precision decision is a BLOCKING point (Builder stops after
R5.2, see R5.2 item 9). After Sonnet confirms it, R5.3 to R5.7 run
non-blocking and the stage review is non-blocking (max one unreviewed stage).
Note: "Builder" = Claude subagents launched by Sonnet (see
`.agents/rules/dev-workflow.md`). Sonnet reviews each substage diff before the
next one starts.

Order: R5.1 → R5.2 (STOP, decision) → R5.3 → R5.4 → R5.5 → R5.6 → R5.7.
Start point: the first row of R5.1..R5.7 in `PROGRESS.md` not `committed`.
Suggested Builder tier: R5.1, R5.2, R5.3, R5.4, R5.7 Sonnet; R5.5, R5.6 Sonnet
or Haiku.

## Common rules (every substage)

-   Start with `graphify update .` and `graphify query` for the area. Reuse
    existing code. No duplicate files.
-   Tests first-class. Math checks compare against the CPU double engine
    (R2 to R4, already verified) or an independent numpy oracle
    (`tests/python/numpy_ref.py`).
-   After each substage: build, `ctest` + `pytest` green, `graphify update .`,
    commit `R5.<y>: <summary>`, row `committed` in `PROGRESS.md`.
-   Baseline after R4: CTest 8/8, pytest 461. Old tests stay green.
-   Commands: `cmake --build build --config Release`, then
    `ctest --test-dir build -C Release --output-on-failure` and
    `python -m pytest -q`. (`run_tests.ps1` is blocked by PowerShell
    execution policy: do not change the policy.)
-   Spec unclear or contradicts the code → do NOT guess. Write the question
    in `PROGRESS.md` → "Blockers", set the row `blocked`, stop.
-   Same substage red after two fix attempts → stop, same procedure.
-   A CUDA failure that looks like a driver problem
    (`cudaErrorInsufficientDriver`, `cudaErrorUnsupportedPtxVersion`, no
    device) → `blocked`: the USER must update the NVIDIA driver (do not try).
-   GPU budget: 8 GB VRAM hard ceiling, batch sizes from free VRAM at run
    time, never hard-coded (`docs/MASTER_PLAN.md` 1.6).
-   Benchmarks record hardware and software versions.
-   Tests that need a GPU use `pytest.skip("no CUDA device")` /
    C++ exit code 77-style skip message when none is present. All other
    tests must pass on a machine without a GPU.

## Code facts (checked 2026-11-11, after R4)

-   Machine: RTX 5060 Laptop, SM 12.0, 8151 MiB, driver 592.19 (reports
    CUDA 13.1), CUDA Toolkit 13.5 (nvcc 13.5.37), MSVC 19.44, CMake 4.4.
    No CuPy installed. numpy 2.5.
-   Build: top-level `CMakeLists.txt` has `LANGUAGES CXX CUDA`,
    `CMAKE_CUDA_ARCHITECTURES 120`, `add_subdirectory(core)`,
    `add_subdirectory(cuda)`, `add_subdirectory(bindings)`,
    `add_subdirectory(tests/cpp)`. `cuda/` holds `vector_add.cu/.cuh` in the
    static lib `cuda_lib`; `tests/cpp/test_cuda.cpp` links it. The pybind11
    module `core_bridge` links only `core_lib` (NO CUDA). It is copied to
    the repo root by a post-build step.
-   Compiled model: `kin::CompiledModel`, `kin::CompiledArrays`
    (`core/kin/compiled_arrays.hpp`): SoA, child-sorted: `joint_index,
    parent_link, child_link, type, q_index, src_q_index` (int32), `pre,
    post` (n·16 doubles, row-major 4x4), `axis` (n·3), `mult, offset`,
    `base, tool` (16), `n_joints, n_links, n_dof, tcp_link,
    layout_version = 1`. `kin::to_arrays`, `kin::from_arrays`; Python
    `cb.compile_arrays(model)` → dict of numpy arrays. Layout doc:
    `docs/compiled-model-layout.md` (frozen: R5 must NOT change it).
-   CPU engine (double): `kin::forward_kinematics(cm, q)`,
    `kin::jacobian_tcp`, `kin::solve_ik(cm, lo, hi, target, q0, opts)`
    (adaptive damping, defaults in `docs/jacobian-ik.md`),
    `kin::solve_ik_multi`. Python `cb.fk_compiled`, `cb.jacobian_tcp`,
    `cb.solve_ik`, `cb.solve_ik_multi`; `app.kin_utils.dof_limits(model)`.
-   UI: `MainWindow` docks (IK and Analysis docks added in R4.9),
    `RobotView` (`set_target_pose`, actors), `TaskRunner.submit_fn(fn)` with
    signals `result/error/finished`; tests use `runner.wait_all(5000)` then
    `QApplication.processEvents()`. `AppSettings.get/set`.
-   Test fixture: `tests/python/fixtures/urdf/six_dof_with_gripper.urdf`
    (set `model.tcp_link` to link `l6` as `test_milestone_r3.py` does).
    Helpers: `tests/python/numpy_ref.py` (`fk_numpy`), `robots.py`
    (`random_robot`).

## Decisions fixed for R5 (from the Opus consult, 2026-11-11; do not reopen)

1.  The GPU code lives in a SEPARATE optional pybind11 module `gpu_bridge`
    (own `.pyd`, copied to the repo root like `core_bridge`). `core_bridge`
    never links CUDA. If `gpu_bridge` cannot be imported (no driver, no DLL)
    the app uses the CPU backend. CuPy is NOT used (no new dependency).
2.  Precision default: fp32 compute, CPU fp64 is the reference. One templated
    kernel `template<typename T>` so fp64 is available (`precision="fp64"`).
    R5.2 measures and applies a fixed rule (R5.2 item 8). Workspace sampling
    and GPU IK always use fp32. No double-double.
3.  GPU FK computes in the BASE frame (the `base` matrix is NOT applied on
    the device). The host applies `base` in double afterwards when the caller
    asks for `frame="world"` (default).
4.  Joint tables on the device: float copy, SoA, `pre`/`post` as 3x4 (12
    floats) each, staged into `__shared__` memory at block start (about
    150 B per joint). Not `__constant__` (not reentrant across models and
    streams). Max joints 64, max links 64 (else `ValueError`).
5.  Frames per thread: rotation 3x3 + position (12 floats), no quaternions.
    TCP-only variant walks the active path and keeps one running frame in
    registers (workspace, IK). All-frames variant uses a local
    `frames[MAX_LINKS][12]` array.
6.  Layouts on the device: q input SoA `[dof][N]` float (leading dimension
    `ld` passed explicitly); output SoA `[12][N]` (px py pz r00..r22).
    Python returns `position (N,3)` and `rotation (N,3,3)`.
7.  Launch: 256 threads/block, `__launch_bounds__(256)`, grid-stride loop.
    SASS only: build with `-gencode arch=compute_120,code=sm_120`
    (`CMAKE_CUDA_ARCHITECTURES 120-real`). PTX from toolkit 13.5 does not JIT
    on the 13.1 driver. Use `1.0f`, `sincosf`; never `--use_fast_math`.
    Check the SASS (`cuobjdump --dump-sass`) for `DMUL`/`DADD` in fp32
    kernels once (R5.2).
8.  RNG: own stateless `__host__ __device__` Philox4x32-10 header
    (`core/compute/philox.hpp`, shared by CPU and GPU). Key = seed, counter =
    (global_sample_index, dof_block). `u = (x >> 8) * 2^-24`, `q = lo + u *
    (hi − lo)`. Results do not depend on chunk size or launch shape. The
    chunk start offset MUST be part of the counter.
9.  TDR: default 2 s. Do not touch the registry. Target at most 50 ms per
    kernel launch: time the first chunk, then `chunk *= 50 ms / t_measured`
    (clamped to the chunk limits).
10. Errors: `cudaErrorMemoryAllocation` is not sticky → free, halve the
    chunk, retry; below 65536 samples → CPU fallback + log warning. Sticky
    errors (IllegalAddress, LaunchTimeout, ...) → mark the GPU dead for the
    process, use the CPU from then on, no retry. `CUDA_CHECK` macro → C++
    exception → Python `RuntimeError`; `cudaGetLastError()` after every
    launch; debug builds sync after every launch.
11. `backend="auto" | "cpu" | "cuda"`. Env `ANALYZER_FORCE_CPU=1` forces
    CPU. A test hook can fake an out-of-memory error (R5.5) so the fallback
    path is tested on the GPU machine too.
12. fp32 test tolerances (1e6 samples, max abs error vs CPU double): with
    `n` = number of joints on the active path, `R` = reach = sum of
    `|pre.p| + |post.p|` over the path plus `|tool.p|`:
    `tol_pos = 8 · n · 1.19e-7 · R` (m), `tol_rot = 8 · n · 1.19e-7` (rad,
    geodesic angle); RMS must be at most `tol/10`; orthonormality
    `‖R^T R − I‖ ≤ 4 · n · 1.19e-7`. The CPU reference MUST use `q` rounded
    to float (`(double)(float)q`), else q quantisation gives false failures.

---------------------------------------------------------------------

## R5.1 --- GPU Compute Manager (C++ + bindings)

Goal: device query, VRAM budget, chunk-size rule, error policy. First step
verifies the driver.

Files: `core/compute/budget.hpp` (new, header-only, NO CUDA, bound in
`core_bridge`), `cuda/gpu_manager.cuh/.cu` (new), `cuda/gpu_bridge.cpp` (new
pybind11 module `gpu_bridge`), `cuda/CMakeLists.txt`, `CMakeLists.txt` (arch
`120-real`), `bindings/core_bridge.cpp` (budget binding), tests.

1.  Verify first: build and run the existing `test_cuda` on this machine. If
    it fails with a driver error → Blocker (user updates the driver). Record
    versions (`nvidia-smi`, `nvcc --version`, driver, toolkit, runtime) in
    `docs/hardware-audit.md` under a new "R5.1 GPU verification" section.
2.  `compute::chunk_size(free_bytes, user_budget_bytes, persistent_bytes,
    bytes_per_sample, headroom = 0.6, reserve_bytes = 256 MiB, min = 65536,
    max = 4'194'304)`:
    `budget = min(user_budget, headroom·free − reserve) − persistent`;
    `chunk = floor(budget / (2 · bytes_per_sample))` (the 2 is double
    buffering), clamp to `[min, max]`, round DOWN to a multiple of 256.
    If `budget <= 0` or the result would be below `min` → returns 0 (means
    "use CPU"). Pure function, bound as `cb.chunk_size(...)`.
3.  `struct DeviceInfo { bool available; std::string name; int sm_major,
    sm_minor; size_t total_bytes, free_bytes; int driver_version,
    runtime_version; std::string error; }`. `gpu::query_device()` never
    throws: no device or driver error → `available = false`, `error` set.
    `gpu::mem_info()` → free/total. Python `gpu_bridge.device_info()` → dict,
    `gpu_bridge.mem_info()`.
4.  Error policy object `gpu::GpuState { bool dead; ... }` (process-wide):
    `CUDA_CHECK(expr)` macro throws `gpu::CudaError` (derived from
    `std::runtime_error`) with the error name and string; helper
    `bool is_sticky(cudaError_t)`, `bool is_oom(cudaError_t)`;
    `mark_dead(msg)`. A test hook `gpu::set_fake_oom(int n)` makes the next
    `n` allocations report OOM (used in R5.5 tests).
5.  `gpu_bridge` module skeleton: `device_info`, `mem_info`, `is_dead()`,
    `set_fake_oom(n)`. The module must import on a machine without a GPU
    (functions then report `available = False`). CMake: new target
    `gpu_bridge` in `cuda/CMakeLists.txt` (needs the Python/pybind11 setup
    from `bindings/CMakeLists.txt`: reuse its pybind11 `FetchContent`; add a
    post-build copy of the `.pyd` to the repo root).
6.  A budget setting: `app/compute/budget.py` is NOT built yet (R5.5). In R5.1
    only the C++ function exists.

Tests: C++ `tests/cpp/test_budget.cpp` (fake numbers: plenty of VRAM → max
chunk; tiny VRAM → 0; multiple of 256; persistent bytes subtract; headroom
respected; zero or negative inputs rejected with `std::invalid_argument`).
Python `tests/python/test_gpu_manager.py`: `cb.chunk_size` cases; `import
gpu_bridge` works; `device_info()` on this machine reports the RTX 5060,
SM 12.0, total about 8 GB (skipped with a clear message if not available);
`mem_info()` free ≤ total; fake OOM hook toggles. `test_cuda` still green.

Done when: tests green, graphify, commit `R5.1: GPU compute manager`.

---------------------------------------------------------------------

## R5.2 --- Batch FK kernel + precision decision (C++/CUDA + bindings)

Goal: batch FK on CPU (multi-thread, double) and GPU (templated), compare,
measure, and apply the precision rule. STOP afterwards.

Files: `core/kin/fk_batch.hpp/.cpp` (new, CPU), `cuda/fk_kernel.cuh/.cu`
(new), `cuda/gpu_bridge.cpp`, `bindings/kin_bindings.cpp`,
`tools/bench_fk.py` (new), `docs/gpu-precision-decision.md` (new), tests.

1.  CPU batch FK (reference, also the fallback):
    `kin::FkBatchResult fk_batch_cpu(const CompiledModel&, const double* q,
    size_t n, size_t ld, int n_threads = 0)`: `q` SoA `[dof][N]` (leading
    dimension `ld ≥ n`), result `position [3][N]` and `rotation [9][N]` in the
    BASE frame (like the GPU; no `base` applied), `n_threads = 0` →
    `std::thread::hardware_concurrency()`. Uses the compiled joint list and
    walks only the active path root → `tcp_link` (reuse `kin::active_chain`
    logic; tool applied; fixed and mimic joints handled exactly as in
    `forward_kinematics`). Python: `cb.fk_batch(cm, q (N,dof) float64,
    frame="world"|"base", n_threads=0)` → dict `position (N,3)`, `rotation
    (N,3,3)`; for `frame="world"` the double `base` matrix is applied.
    `q.shape[1] != n_dof` → `ValueError`.
2.  CPU consistency test: for 1000 random q, `fk_batch` row i equals
    `cb.fk_compiled(cm, q_i)["tcp"]` (tol 1e-12).
3.  GPU kernel `template<typename T> __global__ void fk_tcp_kernel(...)`
    exactly as in Decisions 3 to 7. Host precompute per model (in
    `cuda/fk_kernel.cu`): float/double tables as SoA with `pre`/`post` as
    3x4; for FIXED joints fold `A = pre · post` into `pre` (post = I) at table
    build time; for revolute joints the kernel uses `sincosf` + Rodrigues
    for the axis rotation; an "active path" list (joint order root → tip,
    max 64) so only those joints are visited; mimic: `q_eff = mult ·
    q[src_q_index · ld + i] + offset`.
4.  `gpu_bridge.fk_batch(arrays_dict, q (N,dof), precision="fp32"|"fp64",
    frame="world"|"base")` → dict like `cb.fk_batch`, `arrays_dict` =
    `cb.compile_arrays(model)`. It transposes q to a pinned SoA buffer, runs
    chunked (R5.1 `chunk_size`, 50 ms TDR target, fake-OOM aware), copies the
    outputs back, applies `base` in double on the host for `frame="world"`.
    Releases the GIL during the work. Also returns `info`: `{precision,
    chunks, kernel_ms, total_ms}`.
5.  Kernel details: fp32 kernels contain no double arithmetic (check the SASS
    once with `cuobjdump --dump-sass`, record "no DMUL/DADD" in
    `docs/gpu-precision-decision.md`).
6.  Correctness tests (`tests/python/test_gpu_fk.py`, GPU tests skip without a
    device): for the 6-DOF fixture, a random 6R DH robot, a random
    origin_axis 12R, a tree with mimic and fixed joints (use
    `robots.random_robot`): 1e6 random q (seed 42) in limits; fp32 vs CPU
    double on `(double)(float)q`: max abs position error ≤ `tol_pos` and
    rotation ≤ `tol_rot`, RMS ≤ tol/10, orthonormality (Decisions 12);
    fp64 GPU vs CPU: ≤ 1e-12 position, ≤ 1e-12 rotation. Edge cases: N = 1,
    N = 255, 256, 257, N not a multiple of the chunk, empty array (N = 0
    returns empty arrays), wrong `q` width → `ValueError`, more than 64
    joints → `ValueError`. Same result for different chunk sizes (force a
    small chunk via the options argument `chunk=65536` in a test hook).
7.  Benchmark `tools/bench_fk.py`: N = 1e6, a 12-joint robot, fp32 and fp64
    GPU, and CPU (all threads): kernel time (cudaEvent, reported by `info`),
    end-to-end time including host transposes and copies, repeated 5 times,
    median. Writes `benchmarks/r5_fk_<date>.json` with hardware and software
    versions (GPU name, SM, driver, toolkit, runtime, MSVC, numpy, CPU model,
    thread count, git commit).
8.  Decision rule (apply it mechanically, from the Opus consult): the FK
    default precision is fp64 ONLY IF `e2e_fp64 ≤ 2 · e2e_fp32` AND
    `e2e_fp64 ≤ 50 ms` for 1e6 samples. Otherwise the default is fp32 and
    fp64 stays an option. Workspace sampling and GPU IK always use fp32.
    Write the numbers, the rule and the result in
    `docs/gpu-precision-decision.md` (also the tolerances of Decisions 12 and
    the observed errors).
9.  STOP RULE for this substage: after the commit, set the stage status in
    `PROGRESS.md` "Current" to `stage-ready-for-review` with the note "R5.2
    precision decision", Owner "Sonnet reviewer", and STOP (do not start
    R5.3). Sonnet checks the numbers, sets the status back to `in-progress`
    (or writes an `R5-fix` list) and the Builder continues with R5.3.

Done when: tests green, graphify, commit `R5.2: batch FK and precision decision`.

---------------------------------------------------------------------

## R5.3 --- Workspace sampling (C++/CUDA + bindings)

Goal: random sampling of q in limits, reduced to a voxel grid and optional
point cloud, chunked, cancellable, with a CPU reference.

Files: `core/compute/philox.hpp` (new, host/device), `core/compute/workspace.hpp/.cpp`
(CPU reference), `cuda/workspace_kernel.cuh/.cu` (new), `cuda/gpu_bridge.cpp`,
`bindings/kin_bindings.cpp` (CPU binding), tests.

1.  Philox4x32-10 (Decisions 8), `__host__ __device__`, tested against the
    published known-answer vectors (counter 0/key 0, counter all-ones/key
    all-ones, and the pi-digits vector: look them up in the Random123 test
    vectors; if you cannot verify them offline, write a Blocker instead of
    inventing numbers). Reproducibility test: same seed and sample index give
    the same `q` on CPU and GPU (bit-identical `u`).
2.  Options struct `WorkspaceOptions { size_t n_samples; uint64_t seed = 42;
    double voxel_size; double bounds_lo[3], bounds_hi[3] (base frame; default
    `[-R, R]^3 + 1 voxel` from the reach R of R5.2); bool want_counts = true;
    bool want_points = false; size_t max_points = 1'000'000; size_t
    budget_bytes (0 = auto from free VRAM) }`.
3.  Result: `occupied` (bitset grid, `uint32` words, `atomicOr`) with the
    cap 1024³ voxels (128 MB); `counts` (uint32 per voxel, `atomicAdd`, cap
    256 MB ≈ 400³); `occupied_voxels` (N,3) int32 compacted
    (`cub::DeviceSelect::Flagged` on the GPU); `points` (M,3) float32 only if
    asked, at most `max_points`, taken from the first samples by index (so it
    is reproducible). `grid_dims`, `origin`, `voxel_size`, `n_samples_done`,
    `cancelled`, `info` (chunks, chunk size, VRAM peak seen, ms). A request
    over the grid caps → `ValueError` with the maximum allowed voxel count.
4.  GPU path: kernel generates `q` on the device (Philox, no q buffer),
    computes the TCP (TCP-only variant, base frame, fp32), voxelizes with
    `floor((p − origin) / voxel)`, points outside the grid are counted in
    `n_outside` and dropped. Chunking per R5.1 `chunk_size` and Decisions 9
    (50 ms launches), 2 CUDA streams with 2 pinned host buffers (only for
    `want_points`), events before buffer reuse. Cancellation: a
    `std::atomic<bool>` (exposed to Python as a `CancelToken` object with
    `cancel()` / `is_cancelled()`) checked between chunks; progress callback
    `progress(done, total)` called between chunks (acquire the GIL only for
    the call). The GIL is released during the run.
5.  CPU reference `kin::sample_workspace_cpu(...)` (multi-thread, same
    Philox samples, same voxel rule). It uses double FK, so GPU (fp32) and CPU
    grids may differ for points that lie within fp32 error of a voxel
    boundary: the tests allow 0.1 % (document this in the docs). Python
    `cb.sample_workspace(cm, lo, hi, options_dict, cancel=None, progress=None)`
    and `gpu_bridge.sample_workspace(arrays, lo, hi, options_dict, cancel=None,
    progress=None)` with the same result dict.
6.  Grid sampling mode (deterministic): `mode="grid"` with `divisions` per DOF
    (total samples = product, capped by `n_samples`; if the product exceeds
    the cap → `ValueError`). Index → q mapping by mixed radix; documented.

Tests (`tests/python/test_workspace.py`; GPU tests skip without a device):
-   Philox vectors; CPU/GPU identical `q`.
-   Planar 2R (`l1 = l2 = 1`, limits ±π): reachable region is the annulus
    `|l1 − l2| ≤ r ≤ l1 + l2` → with `l1 = l2` a full disk of radius 2. Make
    the robot with `l1 = 1`, `l2 = 0.5` (annulus 0.5 to 1.5, area `π(1.5² −
    0.5²) = 2π`). 2e6 samples, voxel 0.02: occupied-voxel area (voxels ×
    voxel²) within 1 % of `2π` (GPU and CPU).
-   Different chunk sizes (forced small chunk) → identical `counts` grids on
    the GPU; GPU grid vs CPU grid: at most 0.1 % of occupied voxels differ.
-   Memory: the logged VRAM peak never exceeds the budget (the `info` value),
    a tiny budget forces many chunks and still gives the same grid; a budget
    too small for the grid → `ValueError` or CPU fallback (document which).
-   Cancel: cancel from the progress callback after the first chunk →
    `cancelled = True`, `n_samples_done < n_samples`, returns promptly.
-   `want_points`: the first `max_points` points equal the CPU TCP of the
    first samples (tol per Decisions 12). Grid mode: counts the expected
    number of samples. Wrong inputs → `ValueError`.

Done when: tests green, graphify, commit `R5.3: workspace sampling`.

---------------------------------------------------------------------

## R5.4 --- CPU batch IK, GPU batch IK (go / no-go)

Goal: batch DLS IK for many targets. The CPU multi-thread version is built
first (the fallback needs it). The GPU version is built only if the go rule
holds.

Files: `core/kin/ik_batch.hpp/.cpp` (new, CPU), `cuda/ik_kernel.cuh/.cu` (new,
only if go), bindings, `tools/bench_ik.py`, `docs/gpu-ik-decision.md`, tests.

1.  `kin::ik_batch_cpu(cm, lo, hi, targets (N x 4x4 row-major doubles), q0
    (N x dof or one q0 broadcast), IKOptions, n_threads)` → per item
    `q`, `status` (CONVERGED, MAX_ITER, STALLED, NAN), `pos_error`,
    `rot_error`, `iterations`. Uses `kin::solve_ik` per item with a thread
    pool (static chunking). Python `cb.solve_ik_batch(cm, lo, hi, targets
    (N,4,4), q0, options=None, n_threads=0)` → dict of arrays. Results equal
    N calls of `cb.solve_ik` (same options) exactly.
2.  Prototype GPU IK (build it even if the rule later says no-go, as a
    measured prototype): per thread, build `J[6][NMAX]` (template NMAX ≤ 12,
    ≤ 72 floats), accumulate `A = J J^T` (21 unique entries), add `λ²`, solve
    with an unrolled fp32 6x6 Cholesky, `dq = J^T y`. Rules (from Opus):
    `λ_min = 5e-3` in fp32; pivot guard (pivot ≤ 1e-12 · trace → `λ *= 10`,
    retry); step clamp 0.2 rad (0.05 m for prismatic), then clamp to joint
    limits; adaptive λ (error up → reject step, `λ *= 10`; error down →
    `λ /= 2`); iteration cap 100; stall (error reduced by less than 10 % over
    10 iterations) → STALLED; NaN or inf → NAN; a `done` mask inside a fixed
    loop; 128 threads/block, `__launch_bounds__(128)`; launches sized for
    ≤ 50 ms (Decisions 9). GPU tolerance 1e-4 m / 1e-4 rad.
3.  CPU polish: after the GPU, run CPU fp64 `solve_ik` from the GPU `q` for
    every item (at most 10 iterations, damping start 1e-6 adaptive, tolerance
    1e-6 m / rad, multi-threaded). The final status comes from the polish.
    `gpu_bridge.solve_ik_batch(arrays, lo, hi, targets, q0, options)` does
    GPU + polish and returns the same dict as the CPU version plus `info`
    (`gpu_ms`, `polish_ms`, `n_polish_converged`).
4.  GO / NO-GO RULE: GPU IK is kept in the product only if BOTH hold:
    (a) a prototype run at 1e5 targets (random reachable targets on the
    6-DOF fixture, start within 0.5 rad of `q_true`), end-to-end including the
    CPU polish, is at least 5x faster than `cb.solve_ik_batch` on all CPU
    threads; (b) its final convergence rate is within 1 percentage point of
    the CPU rate. If not, the GPU IK code stays in the repository behind the
    flag `GPU_IK_EXPERIMENTAL` (not used by the backend) and the decision is
    logged. Record both numbers, the machine and versions in
    `docs/gpu-ik-decision.md` and in `PROGRESS.md` "Decisions". The "real
    use case ≥ 1e4 targets per call" criterion is assumed true for workspace
    studies.
5.  `tools/bench_ik.py` writes `benchmarks/r5_ik_<date>.json` with versions
    (same fields as R5.2).

Tests (`tests/python/test_ik_batch.py`): CPU batch equals per-item
`cb.solve_ik` (exact `q`); thread count independent results; 100 per robot
(6-DOF fixture, random 6R, random origin_axis 6R) reachable targets: CPU
convergence ≥ 90 %; statuses present; wrong shapes → `ValueError`; a single
unreachable target → status MAX_ITER or STALLED, no crash. GPU tests (skip
without a device): after polish the converged items have FK error < 1e-6
(independent numpy FK), the rate is within 1 point of CPU on a 1e4 subset
(the 1e5 benchmark is in the tool, not in the test), NaN inputs give status
NAN not a crash, small forced chunks give the same results, the polish flag
reports consistent counts. If the rule says no-go, the GPU tests still run
against the prototype.

Done when: tests green, graphify, commit `R5.4: batch IK (CPU, GPU go/no-go)`.

---------------------------------------------------------------------

## R5.5 --- Python compute backend, fallback, Workspace panel

Goal: one Python API, automatic fallback, a UI to run a workspace study.

Files: `app/compute/__init__.py`, `app/compute/backend.py`,
`app/compute/budget.py`, `app/panels/workspace_panel.py` (new),
`app/main_window.py`, `app/viewport/robot_view.py` (point cloud actor),
`app/settings.py` (keys), tests.

1.  `app/compute/backend.py`: `class Backend` with methods `name`,
    `device_info()`, `fk_batch(model_or_arrays, q, precision, frame)`,
    `sample_workspace(model, options, cancel, progress)`,
    `solve_ik_batch(...)` (returns the polished result), all with the same
    signatures and result dicts on both backends. `CpuBackend` uses
    `core_bridge`; `GpuBackend` uses `gpu_bridge` and the R5.1 to R5.4 code.
    `get_backend(preference=None)`: preference from the argument, else
    `AppSettings` key `compute/backend` (`"auto"` default), env
    `ANALYZER_FORCE_CPU=1` always wins. `auto` → GPU when `gpu_bridge`
    imports, `device_info()["available"]` and not `is_dead()`, else CPU. The
    result of `get_backend` includes why (`backend.reason`: e.g. "gpu_bridge
    not importable", "no device", "forced by env").
2.  Automatic fallback inside the GPU backend: on OOM below the minimum chunk
    or a sticky error, the same call is re-run on the CPU backend, a warning
    goes to the logger (`app.logger`), the result `info["fallback"] = True`
    with the reason. Test with the fake OOM hook (R5.1) on this machine and
    with `ANALYZER_FORCE_CPU=1`.
3.  `app/compute/budget.py`: `vram_budget_bytes()` from settings key
    `compute/vram_budget_mb` (default 6144, max 7168, min 256) and
    `max_batch(bytes_per_item)` wrapper over `cb.chunk_size` with live free
    memory from `gpu_bridge.mem_info()` (CPU backend: returns a fixed 1e6).
4.  `WorkspacePanel` dock "Workspace" (View-menu toggle, hidden at start):
    sample count (default 1e6), voxel size (default from reach / 100), seed,
    backend combo (auto / cpu / cuda), "Show points" and "Show voxels"
    checkboxes, "Run", "Cancel", a progress bar, a status line (samples/s,
    backend, fallback reason, VRAM peak). Runs through `TaskRunner.submit_fn`;
    progress updates come from the progress callback via a Qt signal; Cancel
    uses the `CancelToken`. `RobotView.set_workspace_points(points | None)`
    draws a point cloud actor (cap 2e6 points); `set_workspace_voxels(
    centers, voxel_size | None)` draws occupied voxels as small cubes via
    glyphs (cap 200 000 voxels, with a note when capped). Results clear when
    the project closes or the robot changes.
5.  Settings: `AppSettings` keys `compute/backend`, `compute/vram_budget_mb`.
    A small "Compute" section in the panel or a settings dialog edits them.

Tests (`tests/python/test_compute_backend.py`, `test_ui_workspace.py`): both
backends return equal results (fp32 tolerance) on a small FK batch and a
workspace run (GPU part skipped without a device); `auto` falls back to CPU
when `gpu_bridge` import is monkeypatched to fail; env force-CPU; fake OOM
fallback sets `info["fallback"]`; settings persist; UI: run on a 2R robot with
2e5 samples, progress reaches 100 %, the cloud actor appears, Cancel stops the
run, the UI thread is not blocked (the Run handler returns before the result),
panel clears on project close.

Done when: tests green, graphify, commit `R5.5: compute backend and workspace panel`.

---------------------------------------------------------------------

## R5.6 --- Performance and memory dashboard

Files: `app/panels/performance_panel.py` (new), `app/compute/benchmark.py`
(new), `app/main_window.py`, tests.

1.  `app/compute/benchmark.py`: `run_benchmark(backend, model, n=1_000_000)`
    measures FK samples/s and workspace samples/s (and batch IK items/s with
    n = 1e4) for the chosen backend and for the CPU, returns a dict;
    `write_benchmark(result, path)` writes JSON including versions (GPU name,
    SM, driver, toolkit, runtime, numpy, PySide6, Python, CPU model, thread
    count, git commit if available).
2.  `PerformancePanel` dock "Performance" (View toggle, hidden at start):
    device summary, live VRAM (used/free/total, refreshed by a 1 s `QTimer`
    only while visible), last run settings and VRAM peak (from the last
    workspace `info`), "Run benchmark" (via `TaskRunner`) showing samples/s CPU
    vs GPU and the speedup, "Save benchmark..." writes the JSON.
3.  The "last run" values come from the `info` dicts of R5.2/R5.3; no global
    state outside a small `app/compute/last_run.py` holder.

Tests: benchmark function returns the documented keys; JSON contains the
version fields; panel shows values after a run (CPU only path works without a
GPU; GPU rows skipped); the timer stops when the dock is hidden; no regression.

Done when: tests green, graphify, commit `R5.6: performance dashboard`.

---------------------------------------------------------------------

## R5.7 --- R5 milestone and stage gate

Builder does this after R5.6. `tests/python/test_milestone_r5.py` (offscreen
`MainWindow`, real `core_bridge` and `gpu_bridge`; GPU parts skip with a clear
message if no device, the CPU parts still run):
1.  Import `six_dof_with_gripper.urdf`, TCP link `l6`.
2.  Workspace on the GPU backend: 1e6 samples, voxel grid and a capped point
    cloud, shown in the viewport (cloud actor present, point count capped).
3.  A 1e4-sample subset equals the CPU result within the tolerances of
    Decisions 12 (positions of the first 1e4 samples) and the grid differs in
    at most 0.1 % of voxels on the subset.
4.  The `info` VRAM peak is below the configured budget and below 8 GB; the
    log records chunk size and peak.
5.  Cancel works mid-run; the CPU backend gives the same workspace on a 1e5
    subset (forced by the setting); the fallback path (fake OOM) completes and
    reports `fallback`.
6.  FK batch of 1e6 samples fp32 meets the Decisions 12 tolerances.
7.  The benchmark JSON is written and has the version fields.

Stage gate = `docs/MASTER_PLAN.md` section 1.3. In addition:
-   Docs: `docs/gpu-compute.md` (manager, budget rule, error policy,
    backends), `docs/gpu-kernels.md` (layouts, precision, Philox, TDR rule),
    `docs/workspace-panels.md` (R5.5 and R5.6 panels). One page each (gate
    item 8). `docs/gpu-precision-decision.md` and `docs/gpu-ik-decision.md`
    already exist from R5.2 and R5.4.
-   Gate item 5 (GPU/CPU equivalence) = the R5.2 to R5.4 tests, recorded in
    `PROGRESS.md` with the numbers.
-   Record results in `PROGRESS.md` → "Current". Commit `R5: stage milestone
    test and docs`. Tag `stage-R5-done` locally (no remote: do not push). Set
    stage status `stage-ready-for-review` (non-blocking review: the next stage
    may start at once if its spec pack exists; R6 spec is not written yet, so
    STOP and tell the user).
