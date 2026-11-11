# Development Roadmap

## R0 --- Feasibility and setup

-   Confirm exact GPU, VRAM, driver and CUDA versions.
-   Verify Windows compatibility of candidate dependencies.
-   Build proof-of-compatibility projects.
-   Establish repository, CMake, Python environment and CI tests.
-   Define URM schema and interfaces.

**Exit criteria:** Reproducible Windows build with UI, C++ core, Python
bridge, CUDA sample and 3D viewport.

## R1 --- Desktop foundation

-   Main window and navigation.
-   Project create/save/load.
-   3D viewport and camera controls.
-   Logging, settings and background task framework.

## R2 --- Custom robot modelling

-   URM implementation.
-   Link/joint tree editor.
-   Standard/Modified DH editor.
-   R/P/fixed joints.
-   Frames, base and TCP.
-   Interactive joint controls.

**First end-to-end milestone:** Create a 3-DOF robot, visualize it,
change joint values, calculate FK and display frames/TCP pose.

## R3 --- Import

-   URDF parser and asset resolution.
-   CAD geometry/assembly import.
-   Guided joint assignment.
-   Physical-property import, estimation and manual editor.
-   Import validation and error reporting.

## R4 --- Kinematics and dynamics

-   FK and IK.
-   Multiple IK solutions and validation.
-   Jacobians and singularity indicators.
-   Custom C++20/Eigen kinematics/dynamics engine (Pinocchio deferred;
    optional validator in R4-late via Miniconda if needed, see
    docs/hardware-audit.md).
-   FD, ID, torque and energy calculations.

## R5 --- GPU compute

-   GPU Compute Manager.
-   CUDA/CuPy backend.
-   Batch FK and workspace sampling.
-   Batch Jacobian/IK where appropriate.
-   CPU/GPU correctness tests.
-   Performance and memory dashboard.

## R6 --- Simulation and graphs

-   Manual and automatic modes.
-   Numerical integrators.
-   Trajectory playback and timeline.
-   Joint/TCP state tracking.
-   Complete graph suite and data export.

## R7 --- Planning and advanced analysis

-   OMPL integration and additional planners.
-   Trajectory generation and constraint checking.
-   Workspace, singularity and manipulability analysis.
-   Collision/self-collision analysis.
-   Ghost configurations and comparisons.

## R8 --- Extensibility and release

-   Python scripting API.
-   Plugin interfaces and permissions.
-   Reports and export.
-   Project migration and recovery.
-   Installer, packaging and documentation.
-   Performance, reliability and regression testing.

## Development rules

-   Implement one vertical slice at a time.
-   Each feature requires tests and documentation.
-   Keep the application runnable after every milestone.
-   Do not add all dependencies at once.
-   Benchmark before optimizing.
-   Do not treat a feature as complete until numerical and integration
    tests pass.
