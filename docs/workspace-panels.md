# Workspace and Performance panels (R5.5, R5.6)

Code: `app/panels/workspace_panel.py`, `app/panels/performance_panel.py`,
`app/compute/benchmark.py`, `app/compute/last_run.py`, viewport actors in
`app/viewport/robot_view.py` (`set_workspace_points`, `set_workspace_voxels`).
Both docks start hidden; the View menu toggles them (Workspace, Performance).

## Workspace dock

Fields: Samples (1000..2e9, default 1e6), Voxel size [m] (default reach / 100, from
`estimate_reach`), Seed (42), Backend (`auto` / `cpu` / `cuda`, stored in the setting
`compute/backend`), VRAM budget (MiB, setting `compute/vram_budget_mb`), checkboxes
Show points (default on) and Show voxels (default off), Run, Cancel, progress bar, status.

-   Run builds a backend via `get_backend`, then runs `backend.sample_workspace` in a
    `TaskRunner` worker (random mode, points on, `max_points` 2,000,000, counts off). The UI
    thread is not blocked. A shared `cb.CancelToken` implements Cancel (checked between
    chunks).
-   Result: status line with samples, M samples/s, backend, VRAM peak, voxel count, and
    `FALLBACK to CPU: <reason>` if the GPU could not finish. The log gets
    `Workspace run: ...` and `Workspace done: ...; chunk N, chunks M`.
-   Viewport: the point cloud is capped at 2,000,000 points, the voxel actor at 200,000
    centers (a note says when a cap applies). Points are in the base frame; the panel passes
    the robot base transform.
-   A robot change or project close bumps a generation counter: the display is cleared and
    a late result is dropped. Errors (`ValueError`) appear as `Failed: ...`, never raised.
-   `last_run.set_last_run(backend, options, info)` stores the last finished run for the
    Performance dock.

## Performance dock

-   Device summary, live VRAM used / free / total (1 s timer, only while the dock is visible),
    the budget, and the last workspace run (settings and VRAM peak).
-   Benchmark (`run_benchmark(backend, model, n=1e6, n_ik=1e4, budget_bytes)`): FK, workspace
    and batch IK throughput for the CPU and for the selected backend, with speedup. Batch IK
    always runs on the CPU (GPU IK NO-GO). Runs in a worker.
-   Save benchmark writes JSON (`write_benchmark`): keys `n, n_ik, backend, date, fk,
    workspace, ik, versions`. `versions` holds hardware and software: gpu, sm,
    driver_version, runtime_version, toolkit, numpy, pyside6, python, os, cpu, cpu_threads,
    git_commit (GPU fields are null when the CPU backend is used).
