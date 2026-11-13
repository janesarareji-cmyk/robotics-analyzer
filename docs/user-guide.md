# User guide

Robotics Analyzer is a Windows desktop application for serial and tree manipulators: model or import a
robot, check kinematics and dynamics, run simulations and graphs, plan paths, analyse the workspace and
extend the program with scripts and plugins.

This guide describes what each menu item does and links to the detailed document for each feature.
Menu names are as shown in the application. For the scripting interface see
[api-reference.md](api-reference.md); for plugins see [plugin-guide.md](plugin-guide.md).

## Contents

1.  [Install](#1-install)
2.  [First project](#2-first-project)
3.  [Joints and the TCP](#3-joints-and-the-tcp)
4.  [Inverse kinematics](#4-inverse-kinematics)
5.  [Dynamics and analysis](#5-dynamics-and-analysis)
6.  [Workspace and GPU compute](#6-workspace-and-gpu-compute)
7.  [Simulation and graphs](#7-simulation-and-graphs)
8.  [Planning and ghosts](#8-planning-and-ghosts)
9.  [Export and reports](#9-export-and-reports)
10. [Scripting](#10-scripting)
11. [Plugins](#11-plugins)
12. [Projects, autosave and recovery](#12-projects-autosave-and-recovery)
13. [Troubleshooting](#13-troubleshooting)

## 1. Install

Two ways to run the program:

-   **Packaged build (end users).** A one-folder PyInstaller build with a zip and a per-user
    `install.ps1` (no administrator rights, no PATH change). The build steps, the smoke start and the
    known limits are in [packaging.md](packaging.md). A clean-profile test is described in
    [clean-machine-test.md](clean-machine-test.md). A Windows installer (Inno Setup) is not part of the
    delivered build yet.
-   **From source (developers).** Install the Python packages in `requirements.txt` (PySide6, PyVista,
    pyvistaqt, pyqtgraph, python-fcl, numpy and the test tools). Build the C++ core with CMake
    (`cmake --build build --config Release`). The main entry point is `app/main.py`.

Hardware the program is written for: Windows 11, an NVIDIA GPU with CUDA support (RTX 5060 Laptop,
8 GB VRAM, used as the reference) and 24 GB RAM. The GPU is optional: without it every computation
runs on the CPU (see [troubleshooting](#13-troubleshooting)).

## 2. First project

### Start a project

-   **File -> New Project** starts an empty project. It is not saved until **File -> Save Project As...**.
-   **File -> Open Project...** opens a `.robotproj` file. Older project formats are migrated on
    open; a backup of the old file is kept (see [project-recovery.md](project-recovery.md)).

### Open a sample robot

**File -> Open Sample Robot** (or the **Sample robot** button on the tool bar) opens one of the robots
in the `robots/` folder next to the program. Each is a URDF with meshes; the entry text is the first line
of the robot's `SOURCE.txt`, which also names the source and licence.

| Entry | Source |
|-------|--------|
| KUKA LBR iiwa 14 R820 | bullet3 `data/kuka_iiwa` (zlib) |
| Franka Emika Panda + hand | Gepetto `example-robot-data`, panda_description (Apache-2.0) |
| Universal Robots UR3, UR5, UR10 | Gepetto `example-robot-data`, ur_description (BSD) |
| UFACTORY xArm 5, xArm 6, xArm 7 | `xarm_ros`, xarm_description (BSD-3) |
| UFACTORY xArm 5 + xArm Gripper | `xarm_ros`, xarm_description (BSD-3) |

The xArm 5 + Gripper has 6 degrees of freedom: five arm joints and the gripper `drive_joint`; the
other finger joints follow it (mimic). A sample opens like any URDF import: the import report is shown,
and the project is new and unsaved.

### Import a URDF

1.  **File -> Import -> URDF...** and choose the `.urdf` file.
2.  If meshes are missing, set the search folders with **File -> Import -> Mesh search paths...**
    and import again.
3.  The **Import Report** dock (View menu) lists warnings and errors. Severities and codes are in
    [import-report.md](import-report.md). The URDF rules (joint types, mimic, inertial data,
    `tcp_link`) are in [importers.md](importers.md).

Example robot for a first try: `examples/planar_2r.urdf` (two revolute joints, planar).

### Build or edit a DH model

**View -> DH Parameter Editor** shows the Denavit-Hartenberg table of a DH robot: one row per joint
with theta, d, a, alpha (and the convention, standard or modified, set per robot). The editor works
only on DH joints; joints given by origin and axis (for example from a URDF) are shown but cannot be
edited here. Both kinds of joint can be in one robot tree. The model is shown in the
**Robot Model** dock (View -> Robot Model Panel) and the **Properties** dock (View -> Properties Panel).

The internal data model is described in [urm-schema-v2.md](urm-schema-v2.md) and
[compiled-model.md](compiled-model.md).

### Physical properties

Mass, centre of mass and inertia come from the import, from an estimate or from manual entry.
**View -> Physical Properties** edits them and runs the physical checks. See
[physical-properties.md](physical-properties.md).

### Layout: side bar, Focus 3D (F11), Reset layout

The 3D view is the main area. At first start only the Robot Model (with Properties as a second tab), the
Joints (with the DH editor as a second tab) and the Log are open.

-   The **side bar** on the left edge has one button per panel (Model, Joints, DH, IK, Analysis, Workspace,
    Sim, Graphs, Planning, Ghosts, Collision, Physical, Script, Log). A click opens the panel and brings
    it to the front of its tab group; a second click closes it.
-   **View -> Focus 3D view** (**F11**, also a tool bar button) hides all panels so the 3D view uses the
    whole window. Press it again to bring back exactly the panels that were open. Opening any panel
    while in Focus 3D leaves the mode.
-   **View -> Reset layout** restores the default layout and panel sizes.
-   Panels can still be dragged, tabbed and floated. The layout is saved when the program closes. When a
    new version changes the default layout, the old saved layout is ignored once.

## 3. Joints and the TCP

-   **View -> Joints Panel** has one control per DOF (slider and number field). Moving a
    joint updates the 3D view and the TCP readout.
-   The **TCP** is the frame used by FK, the Jacobian, IK and the graphs. By default it is the last
    link. **Robot -> Set TCP link...** chooses another link.
-   **View -> Robot** toggles frames, links, the TCP marker and visual meshes in the 3D view.
-   Joint values are in radians for revolute joints and metres for prismatic joints.
-   **Gripper:** a robot with gripper DOFs (joint name with `drive_joint`, `finger` or `gripper`, not a
    mimic joint; for example xArm 5 + Gripper and Panda) gets a Gripper group in the Joints panel with Open,
    Close and an opening slider (0 % open, 100 % closed). Arm-only robots have no group.

Forward kinematics, the Jacobian and singularity indicators are described in
[jacobian-ik.md](jacobian-ik.md) and [compiled-model.md](compiled-model.md).

## 4. Inverse kinematics

**View -> Inverse Kinematics** opens the IK dock:

-   Enter a target pose (x, y, z in metres; roll, pitch, yaw in degrees), or use the current TCP pose.
-   **Position only** ignores orientation.
-   **Solve** finds all solutions it can find (several starts, damped least squares). The table lists
    them sorted by distance from the current joint vector.
-   Selecting a row previews it in the model. **Apply** keeps it; **Cancel preview** restores the
    old joint vector. Preview does not mark the project as modified.

Details: [analysis-panels.md](analysis-panels.md) (IK dock) and [jacobian-ik.md](jacobian-ik.md)
(method, multiple solutions, limits).

**Drag TCP (View -> Robot -> Drag TCP, key T, toolbar button):** a sphere handle appears at the TCP.
Drag it with the left mouse button; the robot follows (IK, seed = current joints, position plus the
current orientation, position only for robots with fewer than 6 arm DOF such as xArm 5). Gripper DOFs
do not move. If the target cannot be reached the robot stays at the last good pose, the handle turns
red and the status bar says "Target not reachable". On release the pose is kept (project modified)
and the handle snaps to the TCP. Camera rotation with the mouse works as before when you do not
press on the handle. Switch the action off to remove the handle.

## 5. Dynamics and analysis

**View -> Analysis** shows the results at the current joint vector:

-   **Jacobian and singularity:** sigma_min, sigma_max, condition number and manipulability for the
    chosen mode (full, position or angular). A red "NEAR SINGULAR" label appears below the threshold.
-   **Dynamics:** you enter qd and qdd per DOF. The torque table splits the torque into gravity,
    Coriolis, M*qdd and total, with the limit ratio per joint. "Forward dynamics" fills qdd with the
    free acceleration (tau = 0).
-   Gravity is the project simulation setting (default `0 0 -9.81`), not a value in the panel.

Formulation, friction model and energy checks: [dynamics.md](dynamics.md). Panel layout:
[analysis-panels.md](analysis-panels.md).

## 6. Workspace and GPU compute

**View -> Workspace** samples the reachable workspace: samples (default 1e6), voxel size (default reach / 100),
seed, backend (`auto`, `cpu`, `cuda`) and VRAM budget. It shows the point cloud and voxels, with a volume
and an error bound. Run and Cancel control a worker task.

-   Batch forward kinematics and workspace sampling run on the GPU when it is available
    (float32 by default; float64 is available for FK). Otherwise they run on the CPU. A GPU run that
    cannot finish shows `FALLBACK to CPU: <reason>` in the status line.
-   **View -> Performance** shows the device summary, live VRAM use, the last workspace run and a
    benchmark (FK, workspace and batch IK throughput, CPU against the selected backend).
-   Batch inverse kinematics runs on the CPU. The GPU version was measured and not used
    (see [gpu-ik-decision.md](gpu-ik-decision.md)).

Panels: [workspace-panels.md](workspace-panels.md). Method and numbers: [workspace-sampling.md](workspace-sampling.md),
[workspace-analysis.md](workspace-analysis.md). GPU design: [gpu-compute.md](gpu-compute.md),
[gpu-kernels.md](gpu-kernels.md), [gpu-precision-decision.md](gpu-precision-decision.md).

## 7. Simulation and graphs

-   **View -> Simulation** runs the engine. Integrators: Euler, RK4 (default) and RK45 (adaptive).
    Modes: kinematic (you set q with the joint panel or trajectory playback, qd and qdd are zero) and
    dynamic (state evolves by forward dynamics from the torques you set, with gravity and friction
    switches). Details in [simulation.md](simulation.md).
-   **View -> Graphs** shows the recorded signals (joint positions, velocities, torques, TCP pose,
    energy, power, singularity measures) with a time axis. The list of signals and their units is in
    [graphs.md](graphs.md).
-   Trajectories: a trajectory is a list of timed waypoints with cubic or linear interpolation. It is
    stored in the project (`trajectory.json`, see [trajectory.md](trajectory.md)).
-   Teach waypoints: in the Simulation dock, move the robot with the Joints panel and press "Add current pose as
    waypoint" for each pose (cubic interpolation, segment time = largest joint distance / 1 rad/s, at least 0.5 s).
    "Undo last" removes the last waypoint, "Clear waypoints" all of them, "Play" runs the trajectory.
    Taught waypoints are the project trajectory and can be edited in the table below.

## 8. Planning and ghosts

-   **View -> Planning** plans a joint-space path from the current configuration to a goal (joint
    vector or pose) around obstacles. Planners: RRT-Connect (default), RRT*, PRM. The result is time
    parametrised with velocity and acceleration limits and then checked. Method and limits:
    [planning.md](planning.md), [path-planning.md](path-planning.md).
-   **View -> Collision** edits the collision shapes and the allowed-collision matrix (which link
    pairs may touch). See [collision.md](collision.md).
-   **View -> Ghosts** keeps saved configurations (for example a planned goal) and shows them as
    ghost robots next to the live model; a comparison table shows the differences. Stored in
    `configs.json` (see [planning.md](planning.md)).

## 9. Export and reports

-   **File -> Export -> Whole run (CSV)...** and **Whole run (NPZ)...** write the recorded run.
-   **File -> Export -> Shown graph...** writes the graph currently shown.
-   **File -> Export -> Report (HTML/PDF)...** writes a report: robot summary, DH or origin/axis table,
    mass and centre of mass, workspace summary, graphs as PNG, plan results, collision settings. A
    section without data is left out and named in an "Omitted sections" note, not an error.
-   **File -> Export -> URDF...** writes the robot as a URDF file. **Meshes...** copies the mesh files
    next to it.

Formats and limits: [reports.md](reports.md), [graphs.md](graphs.md) (export formats).

## 10. Scripting

-   **File -> Run script...** runs a Python file. **View -> Script console** is a small editor with Run,
    Cancel and an output pane.
-   Scripts use the `robotics_api` package (versioned, plain Python and numpy types only). A script
    runs in a worker thread, so the window stays responsive; Cancel stops it at the next API call.
-   Scripts run with full Python rights in the application process. Run only scripts you trust.

Start with the examples in `examples/` and the reference in [api-reference.md](api-reference.md).

## 11. Plugins

-   **View -> Plugins** lists the installed plugins, their permissions and their state. A plugin can add
    a file importer (File -> Import), a path planner (planner list in the Planning dock) or an analysis
    panel (View -> Plugin panels).
-   A permission (file, network, GPU) is asked the first time a plugin needs it. The answer is stored.
-   Plugins are a cooperative guard, not a sandbox: install only plugins you trust.

Write your own: [plugin-guide.md](plugin-guide.md). Samples: `plugins_examples/`.

## 12. Projects, autosave and recovery

-   A project is one `.robotproj` archive: robot, visuals, trajectory, collision setup, configurations.
-   Saves are atomic: the data is written to a temporary file, flushed to disk, then moved into place. The
    previous file is kept as `.prev` during the save and removed after a successful save.
-   While the project is modified it is autosaved to `name.autosave.robotproj` (interval in the
    settings, default 120 s). After a crash the next start offers to recover from that file.

Details: [project-recovery.md](project-recovery.md). Format history: [schema/README.md](schema/README.md).

## 13. Troubleshooting

### The GPU is not used

-   Check **View -> Performance** (device summary) and the status line of **View -> Workspace**: they show
    the backend in use, and `FALLBACK to CPU: <reason>` when the CPU was used.
-   Setting `ANALYZER_FORCE_CPU=1` in the environment always forces the CPU. The backend setting
    (`auto`, `cpu`, `cuda`) is in the settings.
-   A GPU out-of-memory error makes the program halve the batch and retry, then fall back to the CPU for
    that call. A CUDA error that corrupts the context disables the GPU for the rest of the session. The
    run still completes on the CPU; the log says why.
-   If the log reports an insufficient driver or an unsupported PTX version, update the NVIDIA driver
    to one that supports the CUDA runtime in the build (see [gpu-compute.md](gpu-compute.md) and
    [hardware-audit.md](hardware-audit.md)).

### Meshes are missing or shown as cubes

-   The import report gives `URDF_MESH_MISSING` for a visual mesh that was not found. The mesh is then
    drawn as a placeholder cube.
-   Add the folder that holds the meshes with **File -> Import -> Mesh search paths...**, then import
    again. `package://` names are looked up in the search folders and in the URDF folder and its parents.
-   Supported formats: `.stl`, `.obj`, `.ply`. Meshes with more than 2 million triangles are not drawn
    (`URDF_MESH_TOO_LARGE`). Rules: [importers.md](importers.md#mesh-resolution-resolve_mesh).

### A plan or simulation is slow or stops

-   Planning stops at its time limit and reports no path when none was found in time. Raise the time
    limit or change the obstacles.
-   A running script can be cancelled from the Script console. Pure Python code that does not call the
    API cannot be stopped (see [api-reference.md](api-reference.md#cancel-behaviour)).

### A plugin is disabled

-   A plugin that raises an error, times out or returns invalid data is disabled for the session and
    reported in the log and in the Plugins dock. Restart the program to try it again.

### The program will not start from the packaged build

-   Run `RoboticsAnalyzer.exe --smoke` from a console to see the failing step. If the message names a
    missing DLL, install the Microsoft Visual C++ redistributable. See [packaging.md](packaging.md).

## Import behaviour (U2.1)

- The Import Report dock opens only when the import has errors. Warnings are
  listed in the dock (View > Import Report); the status bar shows the count.
- Start pose after an import: each joint starts at 0 when 0 is inside its
  limits, else at the middle of its range. Saved projects keep their saved pose.
- Models with visual meshes start with View > Robot > Show Frames off (the TCP
  triad stays). Models without visuals start with frames on.
- Floor grid: a grid (0.1 m cells) at the base height with short world axes
  (X red, Y green, Z blue) at the origin. Toggle with View > Robot > Show Floor
  Grid (default on, remembered). Size follows the robot reach.
- Drag and drop: drop a `.urdf` file on the window to import it, or a
  `.robotproj` file to open it (asks about unsaved changes). Other files are
  ignored with a status bar message.
- Project name after an import: the sample title for a sample robot; the file
  name when the URDF robot name is generic (`UF_ROBOT`, `robot`, empty).
