# Documentation index

Start with the [user guide](user-guide.md) if you use the program, or the [developer notes](developer-notes.md)
if you change the code. The project overview is in the [README](../README.md).

Every document in `docs/` is listed below. `tests/python/test_docs.py` fails when a file is missing here.

## Guides

-   [User guide](user-guide.md): install, first project, joints, IK, dynamics, GPU, simulation, planning,
    export, scripting, plugins, troubleshooting.
-   [Developer notes](developer-notes.md): architecture, build, tests, docs workflow, migrations, decisions.
-   [Scripting API reference](api-reference.md): `robotics_api` functions, classes, errors (generated).
-   [Plugin guide](plugin-guide.md): writing, installing and permissions of plugins.
-   [Packaging](packaging.md): one-folder build, smoke start, installer status and limits.
-   [Clean machine test](clean-machine-test.md): what was tested on a clean profile, and what is left.
-   [Reports and export](reports.md): HTML and PDF report, URDF and mesh export.
-   [Project migration and recovery](project-recovery.md): atomic save, autosave, migrations, diagnosis.

## Data formats

-   [Data formats overview](schema/README.md): URM, project file versions, exports, plugin manifest.
-   [URM schema v2](urm-schema-v2.md): the robot model as JSON.
-   [Compiled model layout](compiled-model-layout.md): frozen flat layout (`layout_version` 1).
-   [Compiled kinematic model](compiled-model.md): data structures and compile rules.
-   [Trajectory and playback](trajectory.md): trajectory model, interpolation, project format 1.2.0, 1.3.0 and 1.4.0.
-   [Collision](collision.md): shapes, allowed collision matrix, project format 1.3.0.
-   [Targets and ranked IK](targets.md): 3D target, ranked solutions, ghost preview, play to target (U4a).
-   [Curves, follow and live matrices](curves.md): line / arc / spline, path check, coloured tube, matrix panel (U4b).
-   [User interface](user-interface.md): start screen, work modes, joint poses, Cartesian jog, 3D view buttons, undo, help (U5).
-   [UI review U5](ui-review-u5.md): findings of the GUI skills, what was fixed or skipped.
-   [Analysis overlays](overlays.md): reach map, singularity zones, joint load, timeline and video export (U6).

## Import

-   [Importers](importers.md): URDF to URM, stages, mesh resolution, project archive.
-   [Import report](import-report.md): report format, dock, severities, code table.
-   [Physical properties](physical-properties.md): mass, COM, inertia and their checks.

## Kinematics and dynamics

-   [Jacobian, IK and singularity](jacobian-ik.md): method, multiple solutions, singularity measure.
-   [Rigid-body dynamics](dynamics.md): RNEA, mass matrix, forward dynamics, friction, energy.
-   [IK and Analysis panels](analysis-panels.md): the two docks and their controls.

## Simulation, graphs and planning

-   [Simulation](simulation.md): core, state machine, integrators, modes, settings.
-   [Graph suite](graphs.md): recorded signals, units, export formats.
-   [Planning](planning.md): profiles, Cartesian paths, checks, planner interface, workflow.
-   [Path planning](path-planning.md): planner API, time parametrisation, panel, numbers.

## Workspace and GPU compute

-   [Workspace sampling](workspace-sampling.md): sampling method and bounds.
-   [Workspace analysis](workspace-analysis.md): volume, error bound, singularity regions, views.
-   [Workspace and Performance panels](workspace-panels.md): dock controls and benchmark.
-   [GPU compute](gpu-compute.md): modules, budget rule, error policy, backends.
-   [GPU kernels](gpu-kernels.md): FK and workspace kernels, precision, launch time rule, GPU IK.

## Decisions and audits

-   [Hardware audit](hardware-audit.md): GPU, CUDA, compiler and tool versions.
-   [Code audit 2026-11-13](audit-2026-11-13.md): full code audit, findings H1-H3, M1-M9.
-   [Master plan](MASTER_PLAN.md): stages R3 to R8, workflow, stage gate.
-   [Collision library decision](collision-decision.md): python-fcl choice (R7.2).
-   [Planner decision](planner-decision.md): own planners instead of OMPL (R7.3).
-   [GPU FK precision decision](gpu-precision-decision.md): float32 default (R5.2).
-   [GPU batch IK: go / no-go](gpu-ik-decision.md): no-go, CPU batch IK is the product path (R5.4).
-   [Dynamics validation](validation/dynamics-validation.md): independent check of the dynamics against MuJoCo (U2.8).

## Stage specifications

Specs in `docs/specs/` are the work orders for each stage. They are kept for the record.

-   [R2 spec](specs/R2-spec.md)
-   [R3 spec](specs/R3-spec.md)
-   [R3.1 spec](specs/R3.1-spec.md)
-   [R4 spec](specs/R4-spec.md)
-   [R5 spec](specs/R5-spec.md)
-   [R6 spec](specs/R6-spec.md)
-   [R7 spec](specs/R7-spec.md)
-   [R8 spec](specs/R8-spec.md)
-   [A1 spec](specs/A1-spec.md)
-   [F1 spec](specs/F1-spec.md)
-   [U2 spec](specs/U2-spec.md)
-   [U3 spec](specs/U3-spec.md)
-   [U4a spec](specs/U4a-spec.md)
-   [U4b spec](specs/U4b-spec.md)
-   [U5 spec](specs/U5-spec.md)
-   [U6 spec](specs/U6-spec.md)

## Plans

-   [U3-U6 plan](plans/U3-U6-plan.md): real collision, target and curve motion, usability, analysis overlays.
