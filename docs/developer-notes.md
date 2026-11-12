# Developer notes

For developers working on the code. The user-facing description is in [user-guide.md](user-guide.md);
the technical architecture from the design phase is in `ARCHITECTURE.md` at the repo root.

## Architecture

Three layers, one process, plus optional GPU code:

| Layer | Folder | Language | Role |
|---|---|---|---|
| Core | `core/` (`core_lib`, static) | C++20, Eigen | Unified Robot Model (`core/urm`), compiled kinematic model and FK/Jacobian/IK (`core/kin`), dynamics RNEA/CRBA/forward dynamics (`core/dyn`), batch compute and budget (`core/compute`) |
| CPU bridge | `bindings/` -> `core_bridge.pyd` | pybind11 | Exposes the core to Python (`urm_bindings.cpp`, `kin_bindings.cpp`, `dyn_bindings.cpp`). The CPU bridge never links CUDA, so the CPU fallback always imports |
| GPU | `cuda/` (`cuda_lib`) -> `gpu_bridge.pyd` | CUDA | Batch FK kernel, workspace kernel, experimental GPU IK, GPU manager. Optional module: when it is missing the backend uses the CPU (`app/compute/backend.py`) |
| Application | `app/` | Python 3.13, PySide6, PyVista, pyqtgraph | Main window and panels, project model and file format, importers (URDF), simulation, planning, analysis, graphs, reports, plugins, script runner, worker tasks |
| Scripting API | `robotics_api/` | Python | Stable, versioned layer over `app/` and `core_bridge`. Plain Python and numpy types only. See [api-reference.md](api-reference.md) |

### App layers (`app/`)

-   `main.py`: entry point, `create_app()`, `--smoke` start check.
-   `main_window.py`: menus, docks, wiring. Docks are created here, hidden at start.
-   `panels/`: Qt widgets only (joint, DH editor, IK, analysis, workspace, simulation, graphs, planning,
    ghosts, collision, scripts, plugins). Panels call `app/` services; they do not own the model.
-   `viewport/`: PyVista scene (robot view, ghosts, workspace point and voxel actors).
-   `project/`: `ProjectData` (in memory), `serializer.py` (archive read/write, atomic save),
    `project_controller.py` (open/save/close, autosave timer, recovery prompt), `migrations.py`,
    `recovery.py`, `diagnose.py`.
-   `importers/`: URDF to URM (`urdf.py`), mesh resolution, import report. See [importers.md](importers.md).
-   `sim/`, `planning/`, `analysis/`, `configs/`, `graphs/`, `reports/`: engines and their data
    products. `physics/`, `kin_utils.py`, `dyn_utils.py`: thin Python helpers over the bridge.
-   `compute/`: backend selection (`get_backend`), GPU/CPU batch calls, benchmark.
-   `plugins/`: manifest, loader, permissions, host. See [plugin-guide.md](plugin-guide.md).
-   `script_runner.py`: runs scripts in a worker with thread-aware output capture.
-   `task_runner.py`: worker threads. `TaskRunner.submit_fn(fn)` returns a `FunctionTask` with signals.
    Connect the signals before submitting, and keep a reference to the task.
-   `logger.py`, `settings.py` (`AppSettings`, a typed wrapper over `QSettings`).

### Threading rules

-   Qt widgets are touched on the GUI thread only.
-   Long computations (IK, workspace, planning, reports, scripts, plugin calls) run through
    `TaskRunner`. A result for a stale robot or generation is dropped, not applied.
-   Simulation runs on a `QTimer` in the GUI thread, one step batch per tick. It never runs a long loop.
-   Plugin and script code runs behind try/except. A hung plugin call is abandoned after its timeout
    (documented limit).

### Model data

-   URM v2 is a tree of links and joints (`docs/urm-schema-v2.md`). The URM is the only model all
    engines read.
-   One precompile step turns it into the frozen compiled layout (`docs/compiled-model-layout.md`,
    `docs/compiled-model.md`). FK, Jacobian, dynamics and GPU code read only the compiled form.
-   Never convert origin/axis joints to DH. The DH editor refuses them.

## Build

Requirements: Windows 11, Visual Studio Build Tools (MSVC, C++20), CMake, Python 3.13, CUDA Toolkit
(only for the GPU module; see [hardware-audit.md](hardware-audit.md) for the versions used).

```powershell
# Python packages
python -m pip install -r requirements.txt

# C++ core, bridge and (optional) GPU module
cmake -S . -B build
cmake --build build --config Release
```

The built `core_bridge` and `gpu_bridge` modules (`.pyd`) are git-ignored; the Python code imports them
from `bindings/` and the repo root. `build/` and binaries are never committed.

The packaged build is made by `packaging/build.ps1` (PyInstaller, one folder). See [packaging.md](packaging.md).

## Tests

Two test suites, both must pass before a commit:

```powershell
ctest --test-dir build -C Release     # C++ tests (tests/cpp, 9 tests)
python -m pytest tests/python -q      # Python tests (pytest.ini, testpaths tests/python)
```

`run_tests.ps1` runs both. Conventions used by the test suite:

-   Offscreen Qt: `QT_QPA_PLATFORM=offscreen` is set by `tests/python/conftest.py`.
-   Modal dialogs are monkeypatched (`QMessageBox`), otherwise tests hang.
-   Numerical references are independent: `tests/python/numpy_ref.py` (FK, Jacobian, dynamics in numpy),
    `closed_form.py` (planar 2R), `robots.py` (fixture robots).
-   Milestone tests `test_milestone_r<N>.py` run each stage end to end (R2 to R8).
-   Tests that close and reopen a project clear `project_controller._is_modified` first.
-   Fixtures: `tests/python/fixtures/` (URDF, meshes, projects in every format version,
    corrupt projects).
-   `test_docs.py` checks the docs: links (`tools/check_docs.py`), API docstrings, the docs index, and
    that `docs/api-reference.md` matches `tools/gen_api_docs.py`.

Benchmarks: `tools/bench_fk.py`, `tools/bench_ik.py` write JSON to `benchmarks/` with the hardware and
software versions. Benchmark before optimizing, and record the versions with every number.

## Docs

-   Write docs in `docs/`, one file per feature, with the stage id in the title (for example `(R7.5)`).
-   Index: [index.md](index.md) lists every doc. Add new files there.
-   Check: `python tools/check_docs.py` (exit 1 on a broken link, anchor or unclosed code fence).
-   API reference: edit the docstring in `robotics_api/` or `tools/api_docs/*.md`, then run
    `python tools/gen_api_docs.py`. Never edit `docs/api-reference.md` by hand.

## Adding a plugin kind or a sample plugin

Read [plugin-guide.md](plugin-guide.md). In short: a plugin is a folder with `plugin.json` (required keys:
`name`, `version`, `api_version`, `entry_point`, `kind`; optional `permissions`, `description`) and one entry-point module. Samples are in
`plugins_examples/`. Kinds: file importer, path planner, analysis panel. The host in
`app/plugins/host.py` and the kind definitions in `app/plugins/kinds.py` are the only places that call plugin code.

## Adding a project format migration

Project files are versioned (`metadata.format_version`). The current version is
`PROJECT_FORMAT_VERSION` in `app/project/project_data.py`. Migrations are in `app/project/migrations.py`.

1.  Bump `PROJECT_FORMAT_VERSION` in `app/project/project_data.py` (semantic: new optional archive members
    are a MINOR step, a changed meaning is a MAJOR step).
2.  Append the new version to `VERSIONS` in `migrations.py` (the last entry must equal the current version).
3.  Add `_m_<old>_to_<new>(docs)`. It works in memory on the parsed JSON members (`docs` maps member name to
    parsed JSON). It must not touch the file. Document what it adds or converts in its docstring.
4.  Register it in `MIGRATIONS` (old version -> function) and `_NEXT` (old version -> next version).
5.  Save a project in the old format as a fixture under `tests/python/fixtures/projects/`
    (`make_fixtures.py` generates them) and add it to the parametrised tests in `tests/python/test_migration.py`.
    The migrated content must equal the reference.
6.  Add the version and its change to the history in [schema/README.md](schema/README.md) and to
    `docs/project-recovery.md` if the save or backup behaviour changes.

Rules that hold for every migration: old files must always load; a newer file than the app is refused;
the backup `<name>.v<old>.bak.robotproj` is created once, before the first save in the new format.

## Decisions summary

Full log: `PROGRESS.md`, section "Decisions". The decisions that shape the code:

-   **Own core, not Pinocchio.** Custom C++20 / Eigen engine (R0.8). Pinocchio validator not done.
-   **Tree model.** URM is a tree with origin/axis joints; DH is a per-robot convention (R2.1, R3.1).
-   **Featherstone RNEA and CRBA** for dynamics, mimic handled by multipliers (R4.6, `docs/dynamics.md`).
-   **GPU design.** Float32 default for FK with a CPU double reference. Thread per sample, shared joint
    tables, memory chunked to the VRAM budget, CPU fallback on OOM or a sticky error (R5,
    `docs/gpu-compute.md`, `docs/gpu-precision-decision.md`).
-   **GPU IK: no-go.** 1.26x against a 5x rule; CPU batch IK is the product path
    (`docs/gpu-ik-decision.md`).
-   **Simulation.** Integrators in Python on the core's forward dynamics; no C++ step (R6.2).
-   **Plot library.** pyqtgraph 0.14 (R6.6). **Collision.** python-fcl 0.7.0.11 (R7.2,
    `docs/collision-decision.md`). **Planner.** Own RRT-Connect, RRT*, PRM; OMPL has no wheel for this
    Python (R7.3, `docs/planner-decision.md`).
-   **Scripting.** `robotics_api` 1.x, semantic versioning; the namespace is not a security sandbox (R8.1).
-   **Plugins.** Cooperative guard in-process, permissions stored per plugin; not an OS sandbox (R8.2,
    `docs/plugin-guide.md`).
-   **PDF.** Qt `QTextDocument` + `QPdfWriter`, no new dependency (R8.3, `docs/reports.md`).
-   **Packaging.** PyInstaller one folder, zip and `install.ps1`; no Inno Setup yet (R8.5,
    `docs/packaging.md`).

## Workflow

Stage-batch workflow, roles and handoff rules: `docs/MASTER_PLAN.md` section 1,
`.agents/rules/dev-workflow.md` and `PROGRESS.md` (the handoff file). Commits follow `R<x>.<y>: <summary>`.
Nothing is pushed without the user's decision.
