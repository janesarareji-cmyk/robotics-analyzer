# Robotics Analyzer (Robotics Engineering & Simulation Platform)

A Windows desktop application for robotic manipulator modelling, engineering
simulation, motion planning and analysis, with optional GPU acceleration.

It is an academic project. The release tag `v1.0-academic` is the academic
version; the usable version `v1.1-usable` adds real collision, 3D targets and
curves, six work modes and analysis overlays (see "Usability (U1 to U6)" below).

Stack: PySide6 and PyVista for the GUI and 3D view, a C++20 / Eigen core
exposed to Python with pybind11, and an optional CUDA module for batch
computation on the GPU. Python scripting (`robotics_api`) and plugins are
built in.

For users, start with the [user guide](docs/user-guide.md). The full list of
documents is in the [documentation index](docs/index.md).

## Features

-   **Robot models:** DH and origin/axis (tree) joints, a DH editor, physical
    properties (import, estimate or manual entry), TCP link selection.
-   **Import and export:** URDF import with meshes (STL, OBJ, PLY), import
    report, URDF export, mesh export, HTML and PDF reports.
-   **Kinematics:** forward kinematics, Jacobian, numerical inverse kinematics,
    multiple IK solutions, singularity indicators.
-   **Dynamics:** inverse dynamics (RNEA), mass matrix, forward dynamics,
    energy and torque tools.
-   **Workspace:** workspace sampling with GPU (CUDA, RTX) or CPU, volume,
    manipulability and singularity views, performance panel.
-   **Simulation:** manual (kinematic) and dynamic modes, integrators, trajectory
    playback, teach waypoints.
-   **Graphs and data:** engineering graph suite, export to CSV and NPZ.
-   **Collision:** shapes, allowed collision matrix, checks with python-fcl.
-   **Path planning:** RRT-Connect, RRT* and PRM planners, shortcut smoothing,
    trajectory checks against limits.
-   **Time-optimal timing:** trajectory timing with torque limits (TOPP,
    reachability analysis).
-   **Ghosts and comparison:** saved configurations shown as ghosts in the 3D
    view (the shown set is saved in the project), comparison of robots.
-   **Scripting:** Python API `robotics_api` (see [api reference](docs/api-reference.md)),
    script console, examples in `examples/`.
-   **Plugins:** Python plugins with a manifest and a permission prompt
    (see [plugin guide](docs/plugin-guide.md)).
-   **Projects:** `.robotproj` files with format versions and migration,
    atomic save, autosave and recovery (see [project recovery](docs/project-recovery.md)).

## Usability (U1 to U6)

Work after the academic release, aimed at everyday use:

-   **Viewport-first layout:** the 3D view is the main area. Panels are dockable
    and tabbed. A side bar has one button per panel.
-   **Focus 3D (F11)** hides all panels; press again to restore them.
    **View -> Reset layout** restores the default layout.
-   **File -> Open Sample Robot** opens one of the bundled robots (see below).
-   **Drag and drop:** drop a `.urdf` file to import it, or a `.robotproj` file to
    open it, on the window.
-   **Floor grid** at the base height, with world axes at the origin.
-   **Gripper:** robots with gripper joints get an Open / Close control and a slider.
-   **Teach waypoints** in the Simulation dock: add the current pose as a waypoint.
-   **Drag TCP (key T):** drag the TCP in 3D; IK moves the robot.
-   **Compare dock:** compares robots by reach, volume, mass, gravity torque and
    payload; export to CSV.
-   **Dynamics validator:** `tools/validate_dynamics.py` checks the dynamics against
    MuJoCo (`mujoco==4.0.1`, in `requirements-dev.txt`). On the bundled robots the
    two agree to about 1e-15. Report: [docs/validation/dynamics-validation.md](docs/validation/dynamics-validation.md).

-   **Collision (U3):** convex hull shapes, automatic allowed-collision matrix, floor
    and scene objects, live blocking in every motion input, red highlight, grasp.
-   **Targets and curves (U4):** 3D target with ranked IK solutions and ghost preview,
    line / arc / spline curves with a reach, collision and singularity check, a
    live matrix panel (tool pose, Jacobian, manipulability).
-   **Work modes (U5):** Model, Move, Target, Plan, Simulate, Analyze (Ctrl+1..6),
    start screen, joint limit bars, Cartesian jog, undo / redo, F1 help.
-   **Analysis overlays (U6):** reach map, singularity zones (red / orange, by kind,
    CSV export) and joint load bars in the 3D view; playback bar under the view,
    screenshot button, playback video export (GIF, PNG frames, MP4 when ffmpeg exists).
    See [overlays](docs/overlays.md).

## Sample robots

The `robots/` folder holds URDF models with meshes. Each folder has a
`SOURCE.txt` with the source and the licence. The first line of each is the
title shown in the File menu:

| Folder | Title |
|--------|-------|
| `robots/kuka_iiwa` | KUKA LBR iiwa 14 R820 |
| `robots/panda` | Franka Emika Panda + hand |
| `robots/ur3` | Universal Robots UR3 |
| `robots/ur5` | Universal Robots UR5 |
| `robots/ur10` | Universal Robots UR10 |
| `robots/xarm5` | UFACTORY xArm 5 |
| `robots/xarm5_gripper` | UFACTORY xArm 5 + xArm Gripper |
| `robots/xarm6` | UFACTORY xArm 6 |
| `robots/xarm7` | UFACTORY xArm 7 |

Licences are given in each `SOURCE.txt`.

## Requirements

-   Windows 11, Python 3.13.
-   Visual Studio Build Tools (MSVC, C++20), CMake, CUDA Toolkit (for the GPU module).
-   NVIDIA GPU with CUDA is optional. Without it, all computation runs on the CPU.
    Reference hardware: RTX 5060 Laptop (8 GB VRAM), 24 GB RAM.

## Install and run

```powershell
pip install -r requirements.txt
cmake -S . -B build
cmake --build build --config Release
python app/main.py
```

`requirements-dev.txt` adds the MuJoCo validator package (`mujoco==4.0.1`).
The packaged one-folder build is described in [docs/packaging.md](docs/packaging.md).

## Tests

```powershell
.\run_tests.ps1                              # C++ (CTest) and Python (pytest) together
python -m pytest tests/python -q             # Python tests only
ctest --test-dir build -C Release            # C++ tests only
```

Latest full suite: 1625 passed, 6 skipped (run in five foreground chunks of test files).

## Documentation

-   [User guide](docs/user-guide.md): install, first project, features, troubleshooting.
-   [Documentation index](docs/index.md): every document in `docs/`.
-   [Developer notes](docs/developer-notes.md): architecture, build, tests, decisions.
-   [Scripting API reference](docs/api-reference.md) and [plugin guide](docs/plugin-guide.md).

## Fixed decisions

-   **Platform:** Windows 11 desktop; standalone application.
-   **Target hardware:** NVIDIA RTX 5060 Laptop GPU, 24 GB system RAM,
    AMD Ryzen 9 HX series.
-   **Robot scope:** One active robot per project; arbitrary practical
    N-DOF configurations.
-   **Inputs:** Custom robot builder, URDF import, CAD import.
-   **CAD workflow:** Guided joint configuration by default; manual and
    automatic modes also available.
-   **Simulation:** Engineering-focused; robot-only environment.
-   **Interaction:** Manual joint control and automatic trajectory
    playback.
-   **Graphs:** Full engineering graph suite.
-   **Physical properties:** Hybrid---import, calculate/estimate, or
    enter manually.
-   **Extensibility:** Built-in algorithms plus Python scripting and
    plugins.

## Core capabilities

1.  Robot modelling and import.
2.  Forward/inverse kinematics and Jacobian analysis.
3.  Forward/inverse dynamics.
4.  Path planning and trajectory generation.
5.  Interactive and automatic simulation.
6.  Workspace, singularity and manipulability analysis.
7.  GPU-accelerated batch computation.
8.  3D visualization, graphs, numerical results and export.

## First milestone

Build a custom 3-DOF robot → configure DH parameters → visualize in 3D →
control joints → calculate FK → display frames and TCP pose.

See [SRS.md](SRS.md), [ARCHITECTURE.md](ARCHITECTURE.md),
[TECH_STACK.md](TECH_STACK.md), and [ROADMAP.md](ROADMAP.md).
