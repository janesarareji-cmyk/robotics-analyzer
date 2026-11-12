# Stage R8 spec pack: Extensibility and release

Authority: `docs/MASTER_PLAN.md` section 7 (workflow, gate, push in section 1).
Review: non-blocking, then FINAL full review (Sonnet reads the whole stage; user decides release).
Builder = Claude subagents launched by Sonnet. Sonnet reviews each substage diff.

Order: R8.1 → R8.2 → R8.3 → R8.4 → R8.5 (STOP after: packaging decision) → R8.6 → R8.7 (milestone).
Suggested tier: R8.1 Sonnet; R8.2 Sonnet (isolation, permissions); R8.3 Sonnet; R8.4 Sonnet;
R8.5 Sonnet (proof builds); R8.6 Haiku (docs) with Sonnet check; R8.7 Sonnet.

## Common rules (every substage)

-   Same as R6 spec "Common rules" (`docs/specs/R6-spec.md` lines 13-36) and R7 spec "Common rules".
-   Baseline after R7: CTest 9/9, pytest 956 passed, 1 skipped. Old tests stay green.
-   `app/main_window.py` is cp1252 + CRLF: edit by bytes. Monkeypatch any `QMessageBox` in tests
    (modal dialogs hang offscreen). Tests that close/open a project set
    `project_controller._is_modified = False` first.
-   New dependency or tool (PDF library, PyInstaller, Nuitka, installer maker) needs a Windows
    proof build first, decision in `PROGRESS.md` "Decisions" with versions.
-   A plugin or script can never crash the app: all foreign code runs behind try/except and, where
    stated, in a worker. Tests include a hostile case per feature.

## Code facts (checked 2026-11-12, after R7)

-   Public building blocks the API wraps: `app/project/*` (`ProjectData`, serializer, format 1.3.0),
    `core_bridge` (compile, fk, jacobian_tcp, analyze_singularity, IK, dynamics), `app/compute/backend.py`
    (`get_backend`, `sample_workspace`, `solve_ik_batch`), `app/sim/*` (`SimEngine`, `TrajectoryPlayer`,
    `Recorder`, `export_csv/npz`, `load_csv/npz`), `app/planning/*` (`plan`, `shortcut`,
    `time_parametrize`, profiles, `check_trajectory`, `CollisionWorld`), `app/configs/store.py`,
    `app/analysis/workspace_analysis.py`, importers `app/urdf/*` (URDF → URM), `app/dyn_utils.py`.
-   Importer returns `ImportResult(model, report, visuals, collisions)`; `ImportReport` has severities.
-   Docks are created in `MainWindow` and toggled from the View menu, hidden at start.
-   `TaskRunner.submit_fn(fn)` → `FunctionTask` with signals; connect the signals BEFORE submit and
    keep a reference to the task (R7.4 bug).
-   Project format history: 1.0.0, 1.1.0 (visuals/assets), 1.2.0 (trajectory.json), 1.3.0
    (collision.json, configs.json). Old files must always load.
-   Libraries now in requirements: PySide6/Qt, numpy, PyVista, pyqtgraph>=0.13, python-fcl.
    matplotlib 3.11.2 is installed. Python 3.13, Windows 11.
-   Known carried gaps: planner does not constrain torque (checks report it); ghosts shown-set not
    saved in the project; `RingBuffer.view()` copy 10.5 ms at the 64 MB cap (accepted).

## R8.1 --- Python scripting API

Files: `robotics_api/__init__.py` (top-level package next to `app/`), `robotics_api/*.py`,
`app/panels/script_panel.py`, `app/script_runner.py`, `docs/api-reference.md` (generated or
written), `examples/*.py`, tests `test_api.py`, `test_script_runner.py`, `test_ui_script.py`.

1.  `robotics_api` is the stable layer. It imports from `app`/`core_bridge` but exposes only plain
    Python and numpy types. `robotics_api.__api_version__ = "1.0"`, semantic versioning rule
    written in the docs (minor = additive, major = breaking).
2.  Functions (all documented with units): `open_project(path)`, `new_project()`,
    `save_project(project, path)`, `import_urdf(path) -> Robot`, `Robot` object (`n_dof`,
    `joint_names`, `limits`, `fk(q)`, `jacobian(q)`, `ik(pose, seed=None, all_solutions=False)`,
    `inverse_dynamics(q, qd, qdd)`, `forward_dynamics(q, qd, tau)`, `mass_matrix(q)`,
    `energy(q, qd)`, `manipulability(q)`), `simulate(robot, trajectory_or_torque_fn, duration, dt,
    integrator)` → recorded arrays, `plan(robot, start, goal, obstacles=None, planner, seed)` →
    path, `time_parametrize`, `workspace(robot, n_samples, voxel)` → dict, `export_csv/npz`.
    Errors raise `RoboticsApiError` subclasses (no raw bridge exceptions leak).
3.  Script runner: runs a script file or an editor string in a worker thread with `robotics_api` and
    a `app` handle (read-only project view) pre-imported; stdout/stderr go to the log panel; a
    cancel flag is checked by long API calls (`simulate`, `plan`, `workspace`) via a cooperative
    `check_cancel()`; the UI stays responsive; a script exception is shown in the log with the
    traceback and never crashes the app. Time limit setting (default none).
4.  UI: script console dock (editor + Run/Cancel + output) hidden at start, View toggle; File →
    "Run script..." with a file dialog. Examples in `examples/` (FK sweep, plan + simulate, export).

Tests: each API function vs an independent numpy/closed-form result (FK of random q = `numpy_ref`,
2R inverse dynamics closed form, plan path validity re-check), error mapping (bad file, wrong
n_dof → `RoboticsApiError`), api version string, example scripts run headless and finish, runner:
script exception reported not raised, cancel stops a long simulate within 1 s, UI stays responsive
(event loop processed during a run), dock offscreen test. 

Done when: tests green, graphify, commit `R8.1: scripting API`.

## R8.2 --- Plugin interfaces and permissions

Files: `app/plugins/{manifest,loader,permissions,host,kinds}.py`, `app/panels/plugin_manager_panel.py`,
`docs/plugin-guide.md`, sample plugins in `plugins_examples/` (importer, planner, analysis panel),
tests `test_plugins.py`, `test_ui_plugins.py`.

1.  Manifest `plugin.json`: `name`, `version`, `api_version` (must be compatible with
    `robotics_api`), `entry_point` (`module:function`), `kind` in `importer | planner | analysis`,
    `permissions` list from `file_read`, `file_write`, `network`, `gpu`. Unknown key or kind →
    rejected with a clear error.
2.  Loader: scans a plugins folder (setting; default `%APPDATA%/<app>/plugins`), validates
    manifests, loads entry points with `importlib` inside try/except, one failing plugin never blocks
    the others or the app. Load result list: name, status (ok | rejected | error | disabled),
    message. Enable/disable per plugin (saved in AppSettings).
3.  Kinds: importer (`register(host)` adds a file filter + `import_file(path) -> ImportResult`-compatible
    model), planner (adds a name to the planner combo; function with the `plan` signature of
    R7.4), analysis panel (returns a Qt widget given a read-only project view).
4.  Permissions: a plugin only gets `host` capabilities it declared (the `host` object enforces
    this: file read/write helpers refuse outside the granted scope, network helper absent unless
    granted; network OFF by default). First use of a permission asks the user (dialog, remembered
    per plugin in settings, revocable in the manager). Denied → `PermissionError` inside the plugin,
    caught by the host. This is a cooperative guard (not an OS sandbox): state that in the docs
    and the dialog text.
5.  Isolation: plugin calls run with exceptions caught and logged; planner/importer calls run in
    the worker with a timeout; a plugin raising, hanging (timeout) or returning garbage is
    reported and disabled for the session.

Tests: sample plugins of each kind load and work; bad manifest, wrong API version, missing entry
point, raising import, raising call, returning wrong type, hang past timeout: app survives, status
shown; permission granted / denied / not declared paths (dialog monkeypatched), network helper
absent without permission; enable/disable persists; UI manager offscreen.

Done when: tests green, graphify, commit `R8.2: plugins`.

## R8.3 --- Reports and export

Files: `app/reports/{report,html_report,pdf_report,urdf_export,mesh_export}.py`,
`app/panels/` menu wiring in `main_window.py` (File → Export), `docs/reports.md`, tests
`test_report.py`, `test_urdf_export.py`.

0.  PDF proof (before code): try PyVista/Qt-free options in order: (1) Qt `QPdfWriter` +
    `QTextDocument` (already in PySide6, no new dependency) printing the HTML report; (2) matplotlib
    PDF backend for graph pages. Use (1) if the result is readable; record the choice in "Decisions".
    No new PDF dependency unless both fail.
1.  Report content (one `ReportData` dataclass, rendered to HTML and PDF): robot summary (name, DOF,
    joint table with limits), DH table (DH robots) or origin/axis table, mass and COM, workspace
    summary (volume, bounds, if computed), graphs as PNG (from the recorder via the graph backend or
    matplotlib offscreen), plan results (planner, time, length, violations), collision settings, app
    version and date. Missing sections are omitted with a note, never an error.
2.  URDF export: URM → URDF XML (links with inertial/visual/collision, joints with origin/axis/limits/
    mimic; DH joints converted to origin/axis on export only), meshes copied to a chosen folder,
    `package://` replaced by relative paths. Mesh export: STL/OBJ of visuals via PyVista.
3.  Round trip: `export_urdf(model)` → R3.3 importer → model equals the original (FK at 50 random q
    to 1e-9, masses/inertia 1e-12, joint limits, names, tree structure, TCP link).

Tests: report HTML contains the key numbers (joint names, limits, mass, DH values) and the
expected sections, PDF file is created and non-empty with page count ≥ 1, report with missing
workspace/plan data renders; URDF export → import equality as above on DH, origin/axis, tree
(gripper) and prismatic/mimic robots; exported XML is well-formed and parsed by an independent
`xml.etree` check (joint count, parent/child names); mesh export file loads back.

Done when: tests green, graphify, commit `R8.3: reports and export`.

## R8.4 --- Project migration and recovery

Files: `app/project/migrations.py`, `app/project/recovery.py`, `app/project/diagnose.py`,
serializer hooks, `tests/python/fixtures/projects/` (one stored file per old format version:
1.0.0, 1.1.0, 1.2.0, 1.3.0 + corrupt samples), tests `test_migration.py`, `test_recovery.py`.

1.  Registry `MIGRATIONS = {from_version: fn}`; every `PROJECT_FORMAT_VERSION` step has a function
    (no-op where the format only added optional files). Opening an older file: automatic backup
    `<name>.v<old>.bak.robotproj` next to it BEFORE migrating, migrate in memory, saved only on
    user save. A newer file than the app: refuse with a clear message (no data loss, no write).
2.  Stored fixtures of each old version are generated once and committed; the test opens each and
    checks content equality against a reference model.
3.  Autosave: timer (setting, default 120 s, min 15 s) writes `<name>.autosave.robotproj` atomically
    (temp file + rename) only when modified. Normal save is atomic too (write temp, fsync, replace;
    keep previous file as `.prev` until success). Crash recovery: on startup/open, a newer autosave
    than the project → prompt "Recover?" (dialog monkeypatched in tests).
4.  Diagnosis: `diagnose(path)` returns a report (not a zip, bad zip CRC, missing `project.json`,
    invalid JSON with line/col, wrong schema field, missing asset, unknown format version) with
    human-readable messages and a suggested action; partial salvage loads what it can with warnings.
5.  Kill-during-save test: a subprocess saves a large project in a loop and is killed at random
    moments (several runs); afterward the original or the `.prev` always opens, never a truncated file.

Tests: all stored versions open and equal the reference; backup file created exactly once; newer
version refused; atomic save leaves no partial file when the write raises midway (monkeypatched);
autosave created, removed after a clean save/close; recovery path picks the autosave; each corrupt
fixture gives the expected diagnosis message; kill-during-save subprocess test (keep it under 60 s).

Done when: tests green, graphify, commit `R8.4: migration and recovery`.

## R8.5 --- Installer and packaging  --- STOP after this substage

Files: `packaging/` (spec files, build script `packaging/build.ps1`), `docs/packaging.md`,
`docs/clean-machine-test.md`.

0.  Proof builds on this Windows machine, time-box one attempt per tool:
    (a) PyInstaller (one-folder) of the app with PySide6, PyVista/VTK, pyqtgraph, python-fcl,
    `core_bridge` and `gpu_bridge` extension modules, CUDA runtime DLLs;
    (b) Nuitka only if (a) fails.
    Installer maker: Inno Setup (if installed or portable available) or a zip distribution with a
    `install.ps1` fallback. Do not install system-wide software without telling the user: if the
    tool needs a user download/installation, write it in "Blockers" and stop with the PoC done as
    far as possible.
1.  The built app must start from the dist folder with a clean `PATH` (no Python, no repo on
    PATH), show the main window (smoke start with `--smoke` flag that creates `MainWindow`, loads
    the sample project, runs one FK and exits 0), and run with `ANALYZER_FORCE_CPU=1` when no CUDA
    DLLs are present (CPU fallback). Bundle only the CUDA runtime pieces that are redistributable
    (document which DLLs); the NVIDIA driver is the user's.
2.  Document the size, build time, versions, and the clean-user-profile test procedure (new Windows
    user or Sandbox) in `docs/clean-machine-test.md`. Do not claim a clean-machine pass that was not
    done: the doc states what was tested (dist folder with scrubbed env) and what is left for the user.
3.  Decision logged in "Decisions". Then STOP and tell the user what remains manual.

Tests: `tests/python/test_packaging.py` (spec file parses, hidden-imports list complete by an import
scan, `--smoke` flag works in the source tree). If the built exe exists, an opt-in test (env var)
runs the smoke start; skipped otherwise with a clear message.

Done when: decision written, smoke start works (or the Blocker is written), commit `R8.5: packaging`.

## R8.6 --- Documentation

Files: `docs/user-guide.md`, `docs/api-reference.md`, `docs/developer-notes.md`, `docs/index.md`,
`docs/schema/` (links to the URM schema and project format docs), `tools/check_docs.py`.

1.  User guide: install, first project (URDF import and DH editor), joints, IK, dynamics, GPU
    workspace, simulation and graphs, planning and ghosts, export and reports, scripting, plugins,
    troubleshooting (GPU driver, missing meshes). Screenshots optional: produced by an offscreen
    `grab()` script (`tools/make_screenshots.py`), never hand-edited.
2.  API reference for `robotics_api` (every public function: signature, units, errors, example);
    generated from docstrings by `tools/gen_api_docs.py` and checked in a test that every public name
    has a docstring.
3.  Developer notes: architecture (C++ core, bridges, app layers, workers), build, tests, workflow,
    the compiled-model layout, how to add a plugin, how to add a migration, decisions summary.
4.  `tools/check_docs.py`: walks `docs/**/*.md`, checks relative links and anchors resolve, code
    fences are closed; exit 1 on a broken link. Run in a test.

Tests: `test_docs.py` (no broken links, every public API name documented, index lists every doc).
Done when: tests green, graphify, commit `R8.6: documentation`.

## R8.7 --- R8 milestone and stage gate

`tests/python/test_milestone_r8.py` (offscreen, real `core_bridge`):
1.  A script run through the script runner (worker): imports the 6-DOF URDF, plans a path, simulates
    it, exports CSV; the CSV equals an independent numpy FK of the recorded q (1e-9). A second script
    raises: error logged, app alive. A third is cancelled within 1 s.
2.  A sample plugin (planner kind) is loaded, appears in the planner combo, plans a path; a bad plugin
    in the same folder is reported and isolated; a permission is denied (monkeypatched dialog).
3.  Report HTML and PDF generated for the project (sections present, files non-empty).
4.  URDF export → import equals the model (FK 1e-9).
5.  Open the stored 1.0.0, 1.1.0, 1.2.0 fixtures: all migrate, backup file exists, content equals reference.
6.  Autosave file created and recovery chosen after a simulated crash.
7.  `python tools/check_docs.py` exit 0. Packaging smoke start passes when the built exe exists (opt-in
    env var), otherwise skipped with a message.
8.  No running timers/workers/threads after close.

Stage gate = `docs/MASTER_PLAN.md` 1.3. Docs: `docs/api-reference.md`, `docs/plugin-guide.md`,
`docs/reports.md`, `docs/packaging.md` exist. Record gate numbers in `PROGRESS.md` "Current".
Commit `R8: stage milestone test and docs`, tag `stage-R8-done` locally, no push. Stage status
`stage-ready-for-review`; Sonnet does the FINAL FULL REVIEW (all stages) and reports to the user.
