# GPU FK precision decision (R5.2)

Status: applied mechanically from the measured medians. Reviewer (Sonnet) confirms.

## Rule (R5 spec, R5.2 item 8)

The FK default precision of `gpu_bridge.fk_batch` is **fp64 only if**
`e2e_fp64 <= 2 * e2e_fp32` **and** `e2e_fp64 <= 50 ms` at N = 1e6 samples,
12-joint robot (medians of 5 runs). Otherwise the default is fp32 and fp64
stays an option (`precision="fp64"`). Workspace sampling and GPU IK always use
fp32.

## Measurement

`python tools/bench_fk.py` -> `benchmarks/r5_fk_2026-11-11.json`.
Robot: `random_robot(seed=12, n=12)`, 12 revolute origin_axis joints, N = 1e6,
`frame="world"`, 2 warm-up calls then 5 timed calls. e2e = Python call to
return, including q transpose/convert into pinned buffers, copies, and the
host `base` multiply and output arrays (position (N,3), rotation (N,3,3)).

| Variant | Kernel median (ms) | End-to-end median (ms) | Chunks |
|---|---|---|---|
| GPU fp32 | 0.58 | 37.6 | 1 |
| GPU fp64 | 15.03 | 68.5 | 1 |
| CPU double, 32 threads | n/a | 127.3 | n/a |

(All 5 runs per variant are in the JSON; spread is about 10%: fp32 35.0 to 40.8,
fp64 67.5 to 69.2 ms.)

Result:

-   `e2e_fp64 / e2e_fp32` = 1.82 -> `<= 2` holds.
-   `e2e_fp64` = 68.5 ms -> `<= 50 ms` FAILS.
-   Rule output: **default precision = fp32**. fp64 remains selectable.
    `gpu_bridge.DEFAULT_FK_PRECISION == "fp32"` and the `precision` argument of
    `gpu_bridge.fk_batch` defaults to it.

Observation: end-to-end time is dominated by the host (transposing
12 x 1e6 doubles, unpacking 12 x 1e6 results, filling 96 MB of numpy output),
not by the fp32 kernel. The fp64 kernel is 26x slower than the fp32 kernel
(consumer GPU fp64 rate) and it is what pushes fp64 above 50 ms.

## Hardware and software

RTX 5060 Laptop GPU (SM 12.0, 8151 MiB), driver 592.19 (reports CUDA 13.1),
CUDA toolkit 13.5 (nvcc 13.5.37), CUDA runtime as reported 13010, MSVC
19.44.35229, AMD Ryzen 9 8940HX (32 threads), Windows 11 (10.0.26300), Python
3.13.15, numpy 2.5.4, git base commit 83dc126 (full record in the JSON).

## fp32 test tolerances (R5 Decisions 12)

1e6 samples in the joint limits (seed 42), GPU result vs CPU double engine run on
`(double)(float)q` (q rounded to float, else q quantisation gives false
failures). `n` = joints on the active path, `R` = reach = sum of `|pre.p| +
|post.p|` over the path plus `|tool.p|`.

-   position: max abs error <= `8 n 1.19e-7 R` (m)
-   rotation: max geodesic angle <= `8 n 1.19e-7` (rad)
-   RMS of both <= tolerance / 10
-   orthonormality `|R^T R - I|_F <= 4 n 1.19e-7`
-   fp64 GPU vs CPU: <= 1e-12 (position and rotation)

## Observed errors (1e6 samples, `tests/python/test_gpu_fk.py`)

| Robot | n | pos max / tol (m) | pos RMS | rot max / tol (rad) | rot RMS | ortho max / tol |
|---|---|---|---|---|---|---|
| six_dof_fixture | 6 | 2.04e-7 / 6.17e-6 | 2.85e-8 | 4.68e-7 / 5.71e-6 | 1.72e-7 | 1.29e-6 / 2.86e-6 |
| dh_6r | 6 | 4.38e-7 / 1.35e-5 | 6.46e-8 | 4.33e-7 / 5.71e-6 | 1.68e-7 | 1.16e-6 / 2.86e-6 |
| origin_axis_12r | 12 | 1.56e-6 / 7.29e-5 | 1.86e-7 | 7.13e-7 / 1.14e-5 | 2.93e-7 | 2.01e-6 / 5.71e-6 |
| tree_mimic_fixed | 8 | 7.00e-7 / 2.47e-5 | 9.38e-8 | 4.60e-7 / 7.62e-6 | 1.91e-7 | 1.22e-6 / 3.81e-6 |

fp64 GPU vs CPU double: position <= 3.2e-15, rotation <= 1.7e-15 (limit 1e-12).

All fp32 errors are well inside the tolerances (margin 12x to 47x on max position and rotation error,
only about 2.2x on the orthonormality bound of the six_dof robot: the tightest metric).

## SASS check (fp32 kernel)

Built SASS only (`CMAKE_CUDA_ARCHITECTURES 120-real`, `sm_120`, no
`--use_fast_math`). `cuobjdump --dump-sass build/cuda/cuda_lib.dir/Release/fk_kernel.obj`,
function `fk_tcp_kernel<float>`:

-   No `DADD`, `DFMA`, `DSETP`. Main body is FFMA/FMUL/FADD only (221 float ops
    counted).
-   **One `DMUL`** (plus one `I2F.F64` and one `F2F.F32.F64`) is present. It is
    in the `sincosf` slow path (Payne-Hanek range reduction, taken only when
    `|angle| > 105615` rad). It is not reached for joint values in the limits
    (|q| <= pi, mimic and offset included). The strict "no DMUL" wording is
    therefore met for the normal path, not for the whole instruction listing.
    Removing it would need `__sinf` (fast-math), which is forbidden. Reviewer
    please accept or rule otherwise.
-   `fk_tcp_kernel<double>` has DFMA/DMUL/DADD as expected and one call of
    `__internal_trig_reduction_slowpathd`.

## Notes

-   The fp32 conversion of `mult`, `offset`, `pre`, `post` is done on the host
    in double and rounded once (fixed joints are folded `pre * post` in double
    first).
-   Pool: pinned host and device buffers are kept between calls (2 slots, at
    most 2^20 samples each); `gpu_bridge.release_buffers()` frees them. The
    benchmark therefore measures steady-state calls (after warm-up).
