# Stage A1 spec pack: Audit fixes

Source: `docs/audit-2026-11-13.md` (full code audit, 2026-11-13). IDs H1-H3, M1-M9 and the LOW list come from there.
Review: BLOCKING. A1 touches data safety and the plugin sandbox. Sonnet reviews each substage diff, then the whole stage.
Builder = Claude subagents launched by Sonnet. Workflow = stage-batch (`.agents/rules/dev-workflow.md`), commit `A1.<y>: <summary>`.

Order: A1.1 → A1.2 → A1.3 → A1.4 → A1.5 → A1.6 → A1.7 → A1.8 (STOP if root cause not found) → A1.9 (milestone).
Tier: A1.1-A1.6 and A1.8 Sonnet; A1.7 Haiku (mechanical) with Sonnet check; A1.9 Sonnet.

## Common rules (every substage)

-   Same as R8 spec "Common rules". Baseline: CTest 9/9, pytest 1187 passed, 4 skipped. Old tests stay green.
-   Each fixed item gets at least one test that FAILS on the old code and PASSES on the new code. Write the
    test first and run it red. Name the audit ID in the test docstring (`"""Audit H2: ..."""`).
-   Do not change behaviour beyond the item. No refactors. No new dependency.
-   `app/main_window.py` is cp1252 + CRLF: edit by bytes. Monkeypatch every `QMessageBox` in tests.
-   C++ change → `cmake --build build --config Release`, then CTest and pytest. CUDA change → also run the
    GPU tests on the real device (`test_gpu_fk.py`, `test_workspace.py`, `test_milestone_r5.py`).
-   Project file format stays 1.3.0. Old fixtures in `tests/python/fixtures/projects/` must still load.
-   Do not touch `tests/python/fixtures/robot_model_v1.json` (unrelated working-tree change, user's).

## Code facts (checked 2026-11-13)

-   `MainWindow.closeEvent` (`app/main_window.py:1249`) cancels the script, workspace, planning and sim, then
    `self._task_runner.shutdown()` (`app/task_runner.py:179`, `waitForDone()` with no timeout), then saves settings.
    It never calls `ProjectController.close_project(parent_widget)` (`project_controller.py:227`), which already
    calls `_confirm_discard` (`:385`) and stops the autosave timer.
-   `TaskRunner` has `wait(timeout_ms)` (`task_runner.py:177`) next to `shutdown()`.
-   `serializer.write_atomic` (`serializer.py:~124-150`): `os.replace(path, prev)` then `os.replace(tmp, path)`.
    `load` (`:160-222`) maps only `BadZipFile`, `JSONDecodeError`, `OSError` to `ProjectSerializationError`.
    `ProjectFormatError` is a subclass. `load_best` (recovery.py) and `open_project` catch only
    `ProjectSerializationError`.
-   Plugin network helper: `app/plugins/host.py:161` `_http_get` → `urllib.request.urlopen(url)`.
-   Planner cancel adapter: `app/planning/planner.py:51` `_cancel_fn`. `robotics_api.CancelToken.is_cancelled`
    is a method (`robotics_api/_cancel.py:31`); `analyze_workspace` calls `cancel.is_cancelled()`.
-   C++ mass matrix parent walk: `core/dyn/dynamics.cpp:190` (`pj[child_link] = i`) and `:207`
    (`j = pj[dm.joints[j].parent_link]`). `pj` starts at -1. `from_arrays`: `core/kin/compiled_arrays.cpp:91`.

## A1.1 --- Close flow (H1, M3, LOW: temp dirs, stale autosave)

Files: `app/main_window.py`, `app/project/project_controller.py`, `app/task_runner.py`, `app/project/project_data.py` (only if needed).

1.  `closeEvent` first calls `project_controller.close_project(self)`. A False return (user picked Cancel)
    → `event.ignore()` and return. Nothing else is stopped in that case.
2.  `close_project` (Discard or Save) removes the autosave of that project and cleans its temp asset dir
    (`robotproj_assets_*`, the dir from `_read_visuals`).
3.  `TaskRunner.shutdown(timeout_ms=5000)`: wait with a timeout. After the timeout, log a warning that names
    the running tasks and return. Do not block the GUI thread forever. A worker blocked in the plugin
    permission `ask()` must be released: `_GuiAsker` gets a `shutdown()` that answers every pending ask
    with "deny for this call" (NOT saved). `closeEvent` calls it before `task_runner.shutdown`.
4.  `_set_project` (new, open, import) removes the autosave and temp asset dir of the project it replaces.
5.  Tests (`tests/python/test_close_flow.py`): modified project + close → prompt shown (monkeypatched);
    Cancel → window stays open, project kept; Discard → closed, autosave file gone, temp dir gone; Save →
    file written. Fake task that never ends → close returns in < 6 s with a warning in the log. Pending
    permission ask during close → returns False and is not saved in settings. Open B after A → A's autosave gone.

## A1.2 --- Load and save safety (H2, M8, LOW: recover-No, backup, configs)

Files: `app/project/serializer.py`, `app/project/recovery.py`, `app/project/project_controller.py`,
`app/project/migrations.py`, `app/configs/store.py`.

1.  `load`: wrap the whole body. Map `KeyError`, `TypeError`, `ValueError` (includes `UnicodeDecodeError`),
    `AttributeError`, `zlib.error`, `EOFError`, `zipfile.LargeZipFile` → `ProjectFormatError` with the file
    name inside the archive when known. Keep the existing mappings. A `ProjectSerializationError` raised
    inside passes through unchanged.
2.  `write_atomic`: no gap where the target is missing. New order: if `keep_prev` and the target is a valid zip →
    `shutil.copy2(path, prev)`; then `os.replace(tmp, path)`; then remove `prev` on success (same rule as
    now). On `os.replace` failure the target is untouched; remove `tmp`. Update the docstring.
3.  Recover prompt "No": delete the autosave only AFTER the main file loaded with no error. If the main file
    fails, keep the autosave and offer it again (or show the error and keep it).
4.  `make_backup`: if `<stem>.v<ver>.bak.robotproj` exists and differs (size or sha256) from the source, write
    `<stem>.v<ver>.bak.<n>.robotproj` with the next free n. Same content → skip.
5.  `ConfigStore.from_dict`: skip a bad entry (missing key, wrong length, non-finite), log a warning with
    its index, load the rest.
6.  Tests: extend `test_recovery.py` / `test_migration.py`. One test per corruption kind: metadata without
    `"name"`, metadata is a list, bad sim value, non-UTF-8 robot.json, deflate stream truncated
    (zip entry bytes damaged), visuals entry without `link_index`. Each → `ProjectFormatError`, and
    `load_best` falls back to `.prev` when present. Monkeypatch `os.replace` to fail on the second call →
    target still exists and is the old content. Recover-No with a corrupt main file → autosave still on disk.
    Two different old files with the same stem → two backups. configs.json with one bad entry → others load.

## A1.3 --- Plugin sandbox and untrusted paths (M1, M2, M9, LOW plugin items, LOW mesh paths)

Files: `app/plugins/{host,permissions,loader,manifest}.py`, `app/settings.py`, `app/project/serializer.py`,
`app/reports/urdf_export.py`, `app/importers/visuals.py`, `app/planning/collision.py`.

1.  `_http_get`: allow only `http` and `https` (`urllib.parse.urlsplit`). Anything else → `PermissionError`.
    Also refuse redirects to another scheme (custom `HTTPRedirectHandler` or check `r.geturl()`).
2.  Project mesh paths: after load, a `mesh_path` that is not under the project's own temp asset dir is
    made relative-and-checked: absolute paths, UNC (`\\` or `//` start), drive-relative paths and any `..`
    part → the visual is kept but `mesh_path` set to `""` and a warning added to the load result
    (`ProjectData.load_warnings: list[str]`, shown in the log). `os.path.isfile` is never called on a UNC
    path. `_write_visuals` and `export_urdf` copy only files inside the asset dir.
3.  Importer and collision mesh reads: `collision._mesh_points` and `visuals` resolve the path, refuse UNC,
    require `isfile`, and require extension in `{.stl,.obj,.ply,.dae,.vtk,.vtp}` before `pv.read`.
    URDF `package://` and relative paths must resolve inside the URDF's folder tree or a user-given package
    root (`mesh_paths_dialog`); else report code `MESH_OUTSIDE_ROOT` (warning) and skip the mesh.
4.  `guarded_call` timeout: time spent inside a permission dialog does not count. `_GuiAsker` records dialog
    time per call; the timeout check subtracts it. An `ask()` that times out (no answer) returns False for
    this call only and is NOT stored as `"denied"`.
5.  `_allow_read`: add the picked file path itself, not its parent dir. Read check = exact file match or
    inside a dir the user picked with a folder dialog.
6.  Settings keys: lowercase the plugin name in `plugins/perm/...` and `plugins/enabled/...`. Manifest
    validation refuses a second plugin whose name differs only by case (second one → error state).
    `_uniq` → `"_plugin_" + name.lower()` plus a short sha1 of the original name, so `a-b`/`a.b`/`a_b` differ.
7.  `manifest._NAME_RE` / `_VER_RE`: use `re.fullmatch` (or `\Z`). Name with `\n` → invalid.
8.  `AppSettings.plugins_folder()`: empty or whitespace stored value → default folder. `recent_max()`: bad
    stored value → default.
9.  Tests: extend `test_plugins.py`. `file:///` and `ftp://` refused; http → redirect to file refused.
    Plugin "Foo" granted, plugin "foo" → refused at load. `a-b` and `a_b` both load, distinct modules.
    Name `"x\n"` invalid. Slow (simulated 15 s) dialog in `register()` → plugin not timed out. Unanswered
    ask → nothing saved. Picked file → sibling file not readable. Project with `C:\Windows\win.ini`,
    `..\x.stl` and `\\server\s\x.stl` mesh paths → loads, paths cleared, warnings, save does not embed them,
    and `os.path.isfile` is not called for the UNC one (monkeypatch spy).

## A1.4 --- C++ input validation (M4, M5, LOW C++ items)

Files: `core/kin/compiled_model.cpp`, `core/kin/compiled_arrays.cpp`, `core/dyn/dynamics.cpp`,
`core/compute/parallel.hpp`, `core/kin/ik_py.hpp`, `core/kin/ik.cpp`, `core/kin/ik_multi.cpp`, bindings as needed.

1.  `compile()` throws `std::invalid_argument` (mapped to Python `ValueError`) when: a link > 0 has no
    incoming joint (non-legacy models); duplicate joint names; mimic multiplier/offset non-finite; any
    origin, axis, DH value or joint limit non-finite. Legacy 0/0 serial fallback keeps working.
2.  `build_dyn_model` keeps a defensive check: `pj[...] == -1` for a non-base link → throw. `mass_matrix`
    walk stops at the base, never indexes -1.
3.  `from_arrays`: check `n_links >= 1`, `n_dof >= 0`, every parent/child in `[0, n_links)`, parent < child,
    `src_q_index` in `[0, n_dof)` for non-fixed and `-1` for fixed, `tcp_link` in range, all transforms finite.
    Throw `std::invalid_argument` with the field name and index.
4.  `parallel_for`: clamp `n_threads` to `[1, 4*hardware_concurrency]`. Thread create failure → join the
    threads already started, then rethrow (no `std::terminate`).
5.  IK options: `damping > 0`, `max_iter` in `[1, 100000]`, `max_step > 0`, tolerances > 0, else `ValueError`.
    `ik_multi`: build the distribution only inside the `hi > lo` guard.
6.  `forward_dynamics`: input q/qd/tau non-finite → throw; after LLT, check the solution is finite, else throw
    (Python `ArithmeticError` or the existing dynamics error type).
7.  Tests: C++ (`test_compiled.cpp`, `test_dynamics.cpp`) and Python (`test_compiled_arrays.py`,
    `test_dynamics.py`, `test_ik.py`): 3-link 1-joint model → ValueError (not crash); duplicate names →
    ValueError; NaN mimic → ValueError; arrays dict with `src_q_index=99` / `child=50` / `tcp_link=-2` →
    ValueError; `n_threads=10**6` → runs; `damping=0` → ValueError; NaN q in forward dynamics → error.
    All existing fixtures still compile.

## A1.5 --- CUDA error paths (LOW CUDA items)

Files: `cuda/workspace_gpu.cpp`, `cuda/workspace_kernel.cu`, `cuda/gpu_bridge.cpp`, `cuda/gpu_manager.cu`.

1.  `ws_count_bits`: use the same pooled/checked allocation as the rest, and an OOM there takes the existing
    host `bitset_to_voxels` path instead of throwing.
2.  `sample_q_gpu`: catch `Oom` → throw the bridge's normal OOM exception (Python `MemoryError` or the existing
    mapped type). Cap `n * n_dof * 4` by the budget; above → `ValueError`.
3.  `ensure_pool`: on any init failure destroy what was created (streams, events, device buffers) before
    rethrowing. A retry starts clean.
4.  `gpu_manager`: `cudaErrorNoKernelImageForDevice`, `cudaErrorUnsupportedPtxVersion`,
    `cudaErrorInsufficientDriver` → device marked unusable with that reason. `device_info` probes once with
    a tiny kernel launch (vector_add of 1 element) at init so a non-sm_120 GPU reports unusable up front.
5.  Tests: fault injection hook (env `ANALYZER_GPU_FAULT=ws_count_oom|pool_init|no_kernel`, test builds only, or
    a bridge debug function) → workspace still returns the correct result via host path; pool init failure
    twice → no handle growth (count via a debug counter); no_kernel → backend reports CPU with the reason.
    Real-device GPU tests stay green.

## A1.6 --- Numerics, planning, import (M6, M7, LOW numerics items)

Files: `app/compute/backend.py`, `app/planning/{planner,checks,cartesian}.py`, `app/importers/urdf.py`,
`app/kin_utils.py`, `app/sim/{mode,playback,engine}.py`.

1.  `estimate_reach`: add `max(|lo|,|hi|)` of every prismatic joint (same rule as C++ `reach_bound`,
    `core/compute/workspace.cpp`). Better: call the bound C++ function if it is bound; else bind it.
2.  PRM inner sample loop: call `sp.check()` every iteration (or every 64 iterations; it must honour cancel
    and time limit within 0.2 s).
3.  Joint limits `lo >= hi` mean ONE rule everywhere: "no limit" only when the joint type is continuous;
    a revolute/prismatic with `lo == hi` is a locked joint at that value. Planner samples it at that value;
    start/goal must equal it (tolerance 1e-9). URDF import: revolute/prismatic `<limit>` without
    `lower`/`upper` → report warning `LIMIT_MISSING` (default stays 0/0 per URDF).
4.  URDF `_vec`/`_num`: non-finite value → report error `NON_FINITE` with the element path; model not built.
    Zero-axis check: `not np.isfinite(n) or n < eps`.
5.  `check_trajectory`: any non-finite q/qd/qdd/tau → violation kind `non_finite` with the sample range.
6.  `_cancel_fn`: for `is_cancelled`, call it if callable, else read it.
7.  `finite_difference_torque`: use the central difference at t2: `qd = (q3 - q1)/(h1 + h2)` (non-uniform
    3-point formula for qd and qdd at the middle sample); ends use one-sided second-order formulas.
8.  Playback speed: dynamic mode uses `speed` only (do not multiply by `time_scale` twice). Kinematic and
    dynamic play the same trajectory in the same wall time for any `time_scale`.
9.  `engine.stop()`: reset `integrator.next_dt` like `reset()`.
10. `cartesian._solve`: skip the branch-jump check at k = 0.
11. Tests: prismatic 1 m gantry fixture → `n_outside == 0`, volume within bound; PRM in a fully blocked space
    with cancel set after 0.1 s → returns in < 0.5 s; locked joint planned at its value; URDF with
    `axis xyz="nan 0 0"` → error, no crash; NaN in a trajectory → `non_finite` violation; object with method
    `is_cancelled()` returning False → planner runs; FD torque on a known quintic vs RNEA of analytic
    q/qd/qdd → error O(h^2) (check halving h divides error by ~4); kinematic vs dynamic play time equal with
    `time_scale=2`; RK45 stop then run → same first step as a fresh engine; Cartesian line with seed far from
    T0 branch → succeeds.

## A1.7 --- Small app fixes (LOW app/API/packaging items) --- Haiku tier

Files: `app/main_window.py`, `app/panels/{ik_panel,workspace_panel}.py`, `app/logger.py`,
`app/reports/mesh_export.py`, `app/sim/export.py`, `robotics_api/{_util,analysis}.py`, `packaging/*`.

1.  Connect `signals.result`/`signals.error` BEFORE `submit_fn` in main_window import (`~:915`, `~:1134`),
    `ik_panel` (`~:266`), `workspace_panel` (`~:284`). Same pattern as `planning_panel.py:356`.
2.  `AppLogger`: remove handlers it added before (keep a module-level registry); guard log dir mkdir with
    fallback to `%TEMP%/RoboticsAnalyzer/logs`, and if that fails too, console only.
3.  `mesh_export`: key `counts` by the safe name.
4.  `export_csv` meta: replace `\r` and `\n` in keys/values with a space. Non-aligned time union keeps duplicate
    timestamps (use stable concatenation, not `np.unique`) so row counts match the aligned path.
5.  `robotics_api._util`: `vec` refuses strings / object dtype (`InvalidArgument`); `pose4` checks
    `R^T R ≈ I` (1e-6) and `det R ≈ 1`; `xyz_rpy` conversion inside the `try`.
6.  `generate_report(format="pdf")` from a script: run the PDF write on the GUI thread (via the AppHandle
    invoker, `QMetaObject.invokeMethod` blocking-queued) when a QApplication exists; with no QApplication
    (pure script, no GUI), create it on the calling thread only if that is the main thread, else raise
    `ExportError("PDF needs the GUI thread")`.
7.  `packaging/analyzer.spec`: missing `core_bridge*.pyd` → build fails with a clear message
    (`gpu_bridge` missing stays a warning). `install.ps1`: remove the old target folder first (after a
    check it is not the source and not a parent of it).
8.  Tests: each item one small test (`test_audit_small.py`); packaging items checked by a static test that
    parses the spec/ps1 text.

## A1.8 --- Exit crash 0xC0000409 (H3) --- STOP if root cause not found

1.  Reproduce: run full pytest, record the process exit code (expect -1073740791). Then bisect by test
    file (and halves of the file list) until the smallest set that crashes at exit is known. Record times.
2.  Likely suspects, check in order: VTK/PyVista render windows not closed before interpreter exit
    (`plotter.close()`), QApplication destroyed after VTK objects, daemon plugin/script threads still in
    C++ at exit, `gpu_bridge` CUDA context torn down after the driver unloads (static cudart), Qt timers.
3.  Fix the root cause in product code or in `conftest.py` teardown (whichever is the true owner). If the
    cause is in the app (not only tests), also check the frozen exe exit (`--smoke`) exit code.
4.  Test: `tests/python/test_exit_clean.py` runs the found crashing subset in a subprocess and asserts exit
    code 0.
5.  STOP rule: two bisect/fix attempts without a root cause → write findings under "Blockers" in
    `PROGRESS.md` and stop. Do not mask the exit code (no `os._exit`) as a "fix".

## A1.9 --- A1 milestone and stage gate

1.  `tests/python/test_milestone_a1.py`: one end-to-end flow per risk area:
    (a) open fixture project, edit, close with Cancel / Discard / Save via `closeEvent`;
    (b) corrupt the saved file (6 kinds from A1.2) → open falls back to `.prev` or shows an error dialog,
        never an unhandled exception (install a `sys.excepthook` spy);
    (c) hostile plugin: tries `file:///`, sibling file read, case-twin name → all refused, app alive;
    (d) hostile project: UNC + absolute mesh paths → loaded with warnings, nothing embedded on save;
    (e) bad arrays dict to GPU FK → ValueError, GPU not marked dead (skip if no GPU);
    (f) prismatic gantry workspace complete; PRM cancel < 0.5 s.
2.  Gate (MASTER_PLAN 1.3): CTest + full pytest green, process exit code 0 (from A1.8), GPU tests on the real
    device, `graphify update .`, `graph.json` committed, `docs/audit-2026-11-13.md` gets a "Status" column
    (fixed in A1.x / won't fix + reason), `PROGRESS.md` updated, tag `stage-A1-done`, status
    `stage-ready-for-review`.

## Not in A1 (won't fix now, reason)

-   Hung plugin/script thread is abandoned, not killed: needs process isolation (larger design, own stage).
-   Planner does not constrain torque, ghost shown-set not saved: carried product gaps, not defects.
-   `RingBuffer.view()` copy at cap: accepted in R7.
