# Technical Architecture

## 1. Architectural principles

-   One standalone desktop application.
-   Modular internals; one integrated user experience.
-   One Unified Robot Model (URM) shared by all engines.
-   Preserve source robot descriptions.
-   Separate computation, simulation state and rendering.
-   CPU reference implementations for GPU validation.
-   Resource-aware execution; no arbitrary software-imposed workload
    caps.

## 2. High-level data flow

`Custom / URDF / CAD` → `Import adapters` →
`Validation and normalization` → `Unified Robot Model` →
`Kinematics / Dynamics / Planning / Analysis` →
`Simulation state manager` → `Results and visualization`

## 3. Main subsystems

1.  Application shell and project manager.
2.  Import adapters: custom, URDF and CAD.
3.  Unified Robot Model and model validator.
4.  Geometry and coordinate-frame managers.
5.  Kinematics engine.
6.  Dynamics engine.
7.  Numerical integration and simulation coordinator.
8.  Motion-planning and trajectory engines.
9.  Workspace and advanced-analysis engines.
10. GPU Compute Manager.
11. Visualization and graph managers.
12. Results, export and reporting.
13. Python scripting and plugin manager.

## 4. Unified Robot Model

Represent: - Links and joint tree. - Joint type, axis, origin and
limits. - Native URDF transforms and optional DH representation. - Base,
link, joint, tool, TCP and user frames. - Visual/collision geometry
references. - Physical properties and provenance. - Simulation
configuration.

Use explicit SI units internally. Track source units and convert on
import.

## 5. Simulation state

Maintain a timestamped state containing: - `q`: joint positions. -
`q_dot`: joint velocities. - `q_ddot`: joint accelerations. - `tau`:
applied/calculated torques. - Link transforms and TCP pose. - Simulation
time and status.

The simulation engine owns state updates. Rendering reads snapshots and
must not determine numerical integration accuracy.

## 6. CPU/GPU execution

Create a backend interface with: - CPU reference backend. - CUDA
backend. - Workload selection and batching. - Memory monitoring and
chunk processing. - Explicit precision/tolerance configuration. -
Numerical comparison tests.

Use GPU for large parallel workloads when benchmarks justify it. Keep
small or latency-sensitive calculations on CPU when more efficient. Do
not assume third-party robotics libraries automatically execute on CUDA.

## 7. CAD and URDF pipeline

URDF: parse hierarchy, joints, axes, limits, geometry and inertial data.

CAD: import assembly/geometry, identify components, propose a kinematic
tree and possible joint definitions, then require user confirmation in
guided mode. CAD geometry alone does not guarantee joint semantics or
physical properties.

## 8. Project storage

Versioned `.robotproj` archive containing: - `robot_model.json` -
`physical_properties.json` - `simulation_settings.json` -
`project_metadata.json` - `geometry/` - `trajectories/` -
`analysis_results/`

Preserve original source files where practical. Include schema version
and migration support.

## 9. Threading and responsiveness

-   Keep UI thread responsive.
-   Run expensive calculations in worker threads/processes as
    appropriate.
-   Use asynchronous result delivery.
-   Keep simulation clock and rendering clock independent.
-   Support cancellation and progress reporting for long jobs.

## 10. Validation and testing

-   Unit tests for transforms, DH, FK, IK and Jacobians.
-   Finite-difference Jacobian checks.
-   Dynamics and mass-matrix reference tests.
-   Integration and trajectory constraint tests.
-   Import regression tests.
-   GPU/CPU equivalence tests.
-   Project serialization tests.
-   Performance benchmarks with recorded hardware/software versions.
