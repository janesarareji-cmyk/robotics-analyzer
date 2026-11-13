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
14. [Collision while moving (U3)](#14-collision-while-moving-u3)

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

### Compare robots

**View -> Compare robots** (side bar: Compare) compares two or more robots. Tick bundled samples and/or
add URDF files, press **Compare** (runs in the background with progress and Cancel), then **Export CSV...**.
All robots use the same sample count (default 20 000), voxel size (0.05 m) and seed. Columns:

-   DOF, total mass (sum of link masses).
-   Reach: largest TCP distance from the base origin over the samples (plus a small local refinement). The TCP
    is the tool point, so it lies beyond the data-sheet reach of a robot (for UR arms the sheet value is the
    wrist-3 centre).
-   Workspace volume from the voxel study (`analyze_workspace`, CPU).
-   Gravity torque per joint at the stretched pose, next to the effort limit of each joint. The stretched pose is
    the sample with the largest horizontal TCP distance (gravity along -z), so the arm is not pointing straight up.
-   Payload: largest point mass at the TCP at that pose such that every joint torque stays within its effort
    limit (0 if the robot cannot hold itself there; n/a when a joint that carries the load has no effort limit).
-   Joint limit ranges (degrees, metres for prismatic joints).

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

## 14. Collision while moving (U3)

Link collision shapes follow the real link meshes: each mesh becomes a convex hull of at most 64
vertices (Collision dock > Mesh fit: Convex hull, the default; Box, Sphere and Capsule stay
available). Projects saved earlier keep their saved fit. Hulls are cached per mesh file.

After an import the program computes an automatic allowed-collision matrix in the background
(status bar "Auto collision matrix ..."): link pairs that always touch, never touch or touch at the
start pose are not checked. The Collision dock lists them with source `auto`; your own edits
(source `user`) always win. "Recompute automatic matrix" runs it again.

Floor: the Collision dock has "Floor collision" (default on). The floor is a plane 1 mm under the
robot base; parts fixed to the base do not touch it. Some arms start with a part under that plane
(the UR zero pose puts the wrist there); they can still move away from it.

Live blocking: with the toolbar button "Collision" on (default), the robot cannot be moved into
itself, the floor or an object. A slider that would enter a collision stops at the last free value
and goes back; the status bar says "Blocked: <link A> touches <link B>". Drag TCP stops the same way
and its handle turns red. The gripper slider stops closing at the first contact. IK "Apply" refuses a
solution that collides. Teach "Add pose" refuses a pose that touches the robot or an object.
Playback stops at the last free sample ("Playback stopped: collision at t = ... s"). Switch the
button off to pass through shapes. If a pose already touches something (floor, for example), moves
that do not add a contact or push an existing one deeper than 1 mm are allowed, so you can always
move away.

Colliding links turn red in the viewport while a pose touches something (it can happen at a start
pose or with blocking off). A robot whose imported start pose collides starts at the nearest free
pose instead (see the log).

Scene objects: toolbar "Add object" puts a box, cylinder or sphere (0.1 m) on the floor in front of
the robot. Drag the yellow box widget to move it, or select it in the Collision panel to type name,
size, position and rotation ("Update selected"). Objects block the robot like the floor does, turn
red when touched, and are saved in the project.

Grasp: close the gripper (Joints panel) with an object between the fingers. The object is grasped
("Grasped <name>", drawn green) and moves with the hand; opening the gripper releases it where it is
(it does not fall). A grasped object still blocks against the floor, other objects and the arm, but
not against the gripper itself. Planning ignores grasped objects.

## 15. Target, ranked IK and Play to target (U4a)

Open the Target dock (side bar "Target", View -> Target).

1. Make a target: "New at TCP" (starts at the current tool pose), or "Pick on surface" and click the
   floor, an object or a robot surface in the 3D view (the tool z axis points into the surface; the target is lifted 10 mm off it,
   see "Lift off surface"). Drag
   the small sphere to move the target; use the X Y Z (mm) and Roll Pitch Yaw (deg) fields, or the
   Rotate buttons (5 degrees about the target's own axis).
2. The target is drawn as an RGB triad with a cone (the tool). Cone colour: blue = not solved, green =
   a valid IK solution exists, orange = valid but near a singularity, red = no valid solution.
3. "Solve" (also automatic after each move) lists all IK solutions ranked best first: score bar,
   motion, limit margin, manipulability, clearance (0 to 100). Colliding solutions and solutions
   outside the limits come last in red. "Weights..." changes the weight of each part (saved).
   Robots with fewer than 6 arm joints are solved for position only.
4. Click a row: a ghost arm shows that solution (green valid, red invalid). "Show all solutions as
   ghosts" shows up to 8.
5. "Play to target": plans a collision-free motion from the current pose to the selected solution and
   plays it in the timeline. "Approach from" first goes to a pose 100 mm back along the tool z axis,
   then moves in a straight line to the target; "Retreat after" comes back. Playback stops at a
   collision like any other motion. Targets are saved in the project (targets.json).

Details: [targets](targets.md).

## 16. Curve, follow and live matrices (U4b)

Open the Curve dock (side bar "Curve", it sits next to Target).

1. Make a target in the Target dock. The Curve dock draws a curve from the current arm pose to it.
2. Type: Line, Arc (through one via point) or Spline (through any number of via points). "Add via
   point" puts one in the middle of the longest part; drag the yellow sphere in the 3D view or type X Y Z.
3. The 3D tube shows problems before you play: green = fine, orange = near a singularity or tight
   to an obstacle, red = unreachable, colliding or a joint flip. "Colour by" switches to TCP speed,
   collision clearance or singularity closeness. Play and Generate are off while a part is red; the
   tooltip says which sample and when.
4. Timing: constant TCP speed, or time-optimal (uses the joint speed, acceleration and torque limits).
   "Chain several targets" visits the ticked targets in order and shows the total cycle time.
5. "Generate" puts the curve into the timeline (Simulation dock); "Play" also plays it. A pink
   marker runs along the tube, a purple ghost shows the end pose. Use the timeline Play / Pause / Stop,
   speed and scrub slider to follow it.

Details: [curves](curves.md).

### Matrices dock

Side bar "Matrices": the 4 x 4 tool pose, the transform of a joint you pick, the joint values, the
Jacobian (tick "Show the Jacobian") and the manipulability bar (red near a singularity). The numbers
follow every move of the arm, also while a curve plays. "Pause updates" freezes them; "Copy as" puts
all matrices on the clipboard as text, numpy or LaTeX.
