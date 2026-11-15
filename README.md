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

## Easy install (wizard)

Double-click `install.bat`. A console wizard asks for the install folder, gets
the source (git clone, or a zip if git is missing), checks Python 3.13, CMake
and the Visual Studio C++ tools (it offers `winget` installs and says honestly
how big they are), lets you choose Auto / CPU only / GPU, runs `setup.ps1`,
and can add Desktop and Start-menu shortcuts and start the app. Enter accepts
the default, Q quits, and it is safe to run again. The log is `install.log`
in the install folder.

```powershell
powershell -ExecutionPolicy Bypass -File install.ps1 -Yes -CpuOnly -InstallDir C:\RoboticsAnalyzer
```

Options: `-InstallDir <path>`, `-Yes` (no questions, take the defaults),
`-CpuOnly`, `-NoShortcuts`, `-SkipLaunch`, `-DryRun` (only prints what it
would do; nothing is installed and no winget, git or cmake is run).

To remove it: double-click `uninstall.bat` (or run `uninstall.ps1`). It deletes
only a folder that holds the marker file `.robotics-analyzer-install` written
by the wizard, asks first, and removes the shortcuts. A clone you made yourself
is never deleted: only `.venv`, `build/`, the built `.pyd` files and the log go.

## Install and run (manual)

Windows 11 only. Everything is installed by one script and started by another.

### Prerequisites

| Need | Version | Where | Required |
|------|---------|-------|----------|
| Python (64-bit) | 3.13 | https://www.python.org/downloads/windows/ | yes |
| CMake | 3.25 or newer | https://cmake.org/download/ (`winget install Kitware.CMake`) | yes |
| Visual Studio Build Tools | 2022 or newer, workload "Desktop development with C++" | https://visualstudio.microsoft.com/downloads/ | yes |
| Git | any | https://git-scm.com/ | to clone |
| CUDA Toolkit + NVIDIA GPU | 12 or newer | https://developer.nvidia.com/cuda-downloads | no (GPU module only) |

No administrator rights are needed by the scripts. The first build downloads
Eigen, nlohmann_json and pybind11, so it needs internet.

### Three commands

```powershell
git clone https://github.com/janesarareji-cmyk/robotics-analyzer.git
cd robotics-analyzer
powershell -ExecutionPolicy Bypass -File setup.ps1     # install + build (once)
.\run.ps1                                              # start the app (or double-click run.bat)
```

`setup.ps1` checks the tools, creates `.venv`, installs `requirements.txt`,
builds the C++ core (`core_bridge`) and, when it finds CUDA and an NVIDIA GPU,
the GPU module (`gpu_bridge`), then runs `python -m app.main --smoke`. The
built `.pyd` files are copied to the repo root, where the app imports them.
You can run it again at any time.

Options: `-CpuOnly` (skip the GPU module), `-SkipBuild` (venv and packages
only), `-Dev` (also installs pytest, pytest-qt and mujoco from
`requirements-dev.txt`), `-Clean` (delete `.venv`, `build/` and the `.pyd`
files first).

### CPU only or GPU

-   **GPU:** needs the CUDA Toolkit (`nvcc`) and an NVIDIA GPU with a driver.
    `setup.ps1` reads the GPU architecture with `nvidia-smi` and builds
    `gpu_bridge` for it. Workspace analysis then runs on the GPU.
-   **CPU only:** if CUDA or the GPU is missing (or you pass `-CpuOnly`), no
    `gpu_bridge` is built. The app falls back to the CPU on its own. All
    features work; very large workspace runs are slower.
-   Manual CMake: `cmake -S . -B build -DRA_ENABLE_CUDA=OFF` (CPU only), or
    `-DRA_CUDA_ARCH=89-real` to choose the GPU architecture by hand.

### Update

```powershell
git pull
powershell -ExecutionPolicy Bypass -File setup.ps1
```

### Troubleshooting

| Problem | Fix |
|---------|-----|
| "running scripts is disabled" | Start with `powershell -ExecutionPolicy Bypass -File setup.ps1`, or use `run.bat`. |
| "Python 3.13 was not found" | Install 64-bit Python 3.13; other versions do not work with the built modules. `py -0` lists what is installed. |
| "CMake was not found" or too old | Install CMake 3.25 or newer, tick "Add to PATH", open a new terminal. |
| "Visual Studio Build Tools were not found" / "C++ tools are missing" | Install the Build Tools and tick "Desktop development with C++". |
| "CUDA Toolkit not found" | Not an error: the build is CPU only. Install the CUDA Toolkit and run `setup.ps1` again for the GPU module. |
| Configure with CUDA fails | The script retries CPU only by itself. For the GPU, reinstall the CUDA Toolkit with its Visual Studio integration. |
| Download fails during configure | Check internet access (gitlab.com and github.com), then `setup.ps1 -Clean`. |
| Smoke test fails | Run `setup.ps1 -Clean`. If it still fails, run `.\.venv\Scripts\python.exe -m app.main --smoke` and read the message. |
| App says to run setup first | `.venv` or `core_bridge` is missing: run `setup.ps1`. |

### Run the tests

```powershell
powershell -ExecutionPolicy Bypass -File setup.ps1 -Dev    # once, adds pytest
.\.venv\Scripts\Activate.ps1
.\run_tests.ps1                                            # CTest and pytest together
python -m pytest tests/python -q                          # Python tests only
ctest --test-dir build -C Release                         # C++ tests only
```

The packaged one-folder build is described in [docs/packaging.md](docs/packaging.md).

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
