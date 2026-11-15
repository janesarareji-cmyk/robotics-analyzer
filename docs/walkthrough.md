# Walkthrough: every part of the window

A guided tour in the order you meet things. All pictures are real window grabs made by
`tools/walkthrough_shots.py` (UR5 and Franka Panda, dark theme, 1600 x 950). For the short version see the
[user interface](user-interface.md) page; for details see the [user guide](user-guide.md).

Contents: [Start screen](#1-start-screen) - [Open a robot](#2-open-a-robot) - [Six work modes](#3-the-six-work-modes) -
[Panels](#4-the-panels-one-by-one) - [3D view tools](#5-the-3d-view) - [Overlays](#6-analysis-overlays) -
[Demo tour](#7-the-demo-tour) - [Menus](#8-menus) - [Help and shortcuts](#9-help-focus-mode-and-shortcuts) -
[Tables](#10-menu-and-shortcut-table) - [10 minute script](#11-ten-minute-first-run-script)

## 1. Start screen

**What it is.** What you see when no robot is loaded. On the very first start a four step tour card also sits
over the 3D view; **Skip tour** ends it for good.

![First run tour](screens/walkthrough/00_first_run_tour.png)

**How to reach it.** Start the program, or close the project (File -> Close Project).

![Start screen](screens/walkthrough/01_start_screen.png)

**Controls.**

-   Nine **sample robot cards** (UR3, UR5, UR10, xArm 5, xArm 5 + Gripper, xArm 6, xArm 7, KUKA iiwa 14, Franka
    Panda). Each shows a picture, full name, maker and "N DOF - kind". One click loads the robot.
-   **Import URDF...** loads your own robot file. **Open project...** opens a saved `.robotproj`. Recent projects
    are listed under the buttons. You can also drop a `.urdf` or `.robotproj` file on the window.

**Try.** Click **UR5**.

Note: while no robot is loaded the right-hand dock (Joints) is still visible and empty. That is expected, it
fills as soon as a robot loads.

## 2. Open a robot

After the click the robot appears on a floor grid, the Joints dock fills with sliders, the log tells you what
was imported (here: 10 joints, 6 DOF, 0 errors) and the status bar shows "Compute: GPU (CUDA)" or CPU.

![UR5 loaded](screens/walkthrough/02_ur5_loaded.png)

Parts of the window, top to bottom:

-   **Menu bar**: File, Edit, View, Robot, Help (section 8).
-   **Toolbar**, four groups: *File* (New, Open, Save), *Move* (Sample robot menu, Undo, Redo, Drag TCP), *Scene*
    (Collision on/off, Add object), *View* (Fit view, Focus 3D, Screenshot). Hover any button for a tip that
    names its shortcut.
-   **Mode bar**: Model, Move, Target, Plan, Simulate, Analyze.
-   **3D view** in the middle with the view buttons at its top right and the playback bar at the bottom.
-   **Dock area** at the right (and left in Model mode): the panels of the current mode.
-   **Log** under the 3D view and the **status bar** at the very bottom.

**Try.** Drag in the 3D view to orbit, mouse wheel to zoom, then press **Fit**.

## 3. The six work modes

A mode shows only the panels needed for one job. Every other panel is still reachable from the **View** menu.
Switch with the bar or **Ctrl+1 ... Ctrl+6**. The mode is remembered for each saved project.

| Mode | Keys | Job | Panels shown |
|------|------|-----|--------------|
| Model | Ctrl+1 | Look at the robot structure | Robot Model tree, Properties, DH editor, Physical |
| Move | Ctrl+2 | Move the robot by hand | Joints (sliders, poses, Cartesian jog), Matrices, Collision |
| Target | Ctrl+3 | Give the tool a 3D goal or curve | Target, Curve |
| Plan | Ctrl+4 | Find a path around objects | Planning, Collision, Ghosts |
| Simulate | Ctrl+5 | Run a motion, look at graphs | Simulation, Graphs |
| Analyze | Ctrl+6 | Reach, singularity, IK, compare | Overlays, Workspace, Inverse Kinematics, Analysis, Compare |

### Model

The tree of links and joints on the left, with Properties beside it (select a node to see its values).
Physical shows mass and inertia per link with **Estimate from visual** and **Scale all masses to total...**.

![Model mode](screens/walkthrough/03_mode_1_model.png)

### Move

Joint sliders (degrees; a thin bar under each slider shows where the joint sits in its range and turns red near
a limit), a TCP pose read-out, and a Cartesian jog pad.

![Move mode](screens/walkthrough/03_mode_2_move.png)

### Target

A target is a 3D goal for the tool: a sphere to drag, a cone showing the tool direction. The panel solves
inverse kinematics and lists every solution with scores. See section 4.

![Target mode](screens/walkthrough/03_mode_3_target.png)

### Plan

Planning table, planner choice and Plan button. Entering Plan mode sets the start to the current pose.

![Plan mode](screens/walkthrough/03_mode_4_plan.png)

### Simulate

Simulation controls and live graphs.

![Simulate mode](screens/walkthrough/03_mode_5_simulate.png)

### Analyze

Five tabs: Inverse Kinematics, Analysis, Workspace, Compare, Overlays.

![Analyze mode](screens/walkthrough/03_mode_6_analyze.png)

**Try.** Press Ctrl+1 to Ctrl+6 and watch the docks change.

## 4. The panels, one by one

Each panel below was grabbed alone. Open any of them from **View**, or by its mode.

### Robot Model tree (View -> Robot Model Panel)

Every link and joint of the robot. Click a node to select it; clicking a link in the 3D view selects it here
too.

![Model tree](screens/walkthrough/10_dock_model_tree.png)

### Properties

Name, type, limits and frames of the selected node. Empty until you select something.

![Properties](screens/walkthrough/11_dock_properties.png)

### DH Parameter Editor

The Denavit-Hartenberg table of the robot (standard or modified). It is for robots made from DH rows; robots
imported from URDF use origin and axis joints and the editor refuses to touch them.

![DH editor](screens/walkthrough/12_dock_dh_editor.png)

### Physical Properties

Mass, centre of mass and inertia per link, with a status per row and a total-mass line.

![Physical](screens/walkthrough/13_dock_physical.png)

### Joints and Cartesian jog

![Joints](screens/walkthrough/14_dock_joints_jog.png)

-   **Go to pose...** list: *Home*, *Ready* (middle of every range) and your saved poses. **Save** stores the
    current joint values under a name inside the project; **Delete** removes one.
-   One row per joint: slider plus a number box in degrees (mm for sliding joints). **Zero all** and **Reset**.
-   **TCP Pose (World Frame)**: position in mm, roll/pitch/yaw in degrees. **Copy pose** copies it.
-   **Move the tool**: X-/X+, Y-/Y+, Z-/Z+ translate, Rx/Ry/Rz rotate. Steps 1 / 10 / 50 mm and 1 / 5 / 15 deg,
    in the base or tool frame. Hold a button to repeat. The collision check stops the robot at objects; switch
    **Collision** off in the toolbar to pass through.

**Try.** Click Z+ a few times, then press Ctrl+Z.

### Matrices

Live numbers: T_base_tcp, T_world_tcp, a chosen joint transform, joint values, the Jacobian and the
manipulability / sigma_min bar. Buttons copy as text, numpy or LaTeX; **Pause** freezes the numbers.

![Matrices](screens/walkthrough/15_dock_matrices.png)

### Collision

Mesh fit (convex hull or box), **Floor collision** switch, a table of environment shapes with an editor (name, kind,
position, rotation, size; **Add shape**, **Update selected**, **Remove selected**), and the allowed-collision
matrix: link pairs that are never checked (the automatic one has 1 pair for UR5, source "auto"), with
**Disable pairs always colliding at zero pose** and **Recompute automatic matrix**. The toolbar **Collision**
button switches the whole check on or off.

![Collision](screens/walkthrough/16_dock_collision.png)

### Target

![Target](screens/walkthrough/17_dock_target.png)

-   **Demo tour**: Run demo / Stop demo (section 7).
-   **Target** list, **Delete**, **New at TCP** (a target where the tool is now), **Pick on surface** (click a
    surface in the 3D view; the target sits 10 mm above it, the **Lift off surface** value).
-   **Pose**: X Y Z in mm, R P Y in degrees, rotate buttons in 5 degree steps.
-   **Inverse kinematics**: **Solve** (or **Solve after each move**), **Weights...** for the four scores, a
    table of solutions (score, motion, limits, manipulability, clearance, state), **Show all solutions as
    ghosts**.
-   **Move to the target**: approach and retreat distances along the tool axis, then buttons that send a
    straight or planned motion to the timeline.

### Curve

Draw a line, arc or spline for the tool to follow, with via points you can drag. Samples are checked in the
background and coloured: green fine, orange warning (near singularity), red bad (unreachable or colliding).

![Curve](screens/walkthrough/18_dock_curve.png)

### Planning

![Planning](screens/walkthrough/19_dock_planning.png)

Start and Goal joint tables (**Start = current q**, **Goal = current q**, or named poses), planner
(RRT-Connect, RRT*, PRM), time limit, seed, edge resolution, timing (scale, or time-optimal with torque
limits), **Shortcut path**. **Plan** runs in the background, **Cancel** stops it, **Send to trajectory** puts the
result on the timeline.

Note: the table shows radians while the Joints panel shows degrees.

### Ghosts

Save a robot pose under a name and show it as a transparent copy (up to 8, with colours). The shown set is saved
in the project.

![Ghosts](screens/walkthrough/20_dock_ghosts.png)

### Simulation

**Teach waypoints** (Add pose, Undo last, Clear, Play), run buttons (Start, Pause, Resume, Stop), mode
(kinematic or dynamic), integrator, dt, time scale, Gravity, Friction, Estimate torque, applied torque per joint
(dynamic mode), a state line (STOPPED, t, steps), and the Trajectory table with interpolation. The playback bar
under the 3D view drives trajectory playback.

![Simulation](screens/walkthrough/21_dock_simulation.png)

### Graphs

Ten kinds of live plot (joint position, velocity, torque, TCP, energy ...), auto range, pause, clear and export
to CSV / NPZ / image. Fills while a simulation or playback runs.

![Graphs](screens/walkthrough/22_dock_graphs.png)

Note: at the default 380 px dock width the right edge of this panel is clipped (the Export button reads
"Expo"). Scroll the panel sideways or widen the dock.

### Workspace

Sampling-based reach: number of samples, voxel size, backend (auto / GPU / CPU), **Run**, **Cancel**; below it
the volume / manipulability analysis with a slice plot and colour bar.

![Workspace](screens/walkthrough/23_dock_workspace.png)

### Inverse Kinematics

Type a target pose, solve, list all solutions sorted, apply one to the robot.

![IK](screens/walkthrough/24_dock_ik.png)

### Analysis

Jacobian, singularity indicators, gravity torques, energy.

![Analysis](screens/walkthrough/25_dock_analysis.png)

### Compare robots

Tick two or more robots (the nine samples are listed; **Add URDF...** adds your own), set samples and voxel size,
press **Compare** to get a table (DOF, mass, reach and more) and **Export CSV...**.

![Compare](screens/walkthrough/26_dock_compare.png)

### Overlays

Three switches: Reach map, Singularity zones, Joint load (section 6).

![Overlays](screens/walkthrough/27_dock_overlays.png)

### Performance (View menu only)

Timings and compute backend of the heavy jobs.

![Performance](screens/walkthrough/28_dock_performance.png)

### Script console (View menu only)

Run Python against the scripting API (`robotics_api`) with output and a Cancel button. Same as File -> Run
script....

![Script](screens/walkthrough/29_dock_script.png)

### Plugins (View menu only)

Lists plugins found in the plugins folder, with enable switches and the permissions each asked for.

![Plugins](screens/walkthrough/30_dock_plugins.png)

### Log

Everything the program tells you. Always at the bottom under the 3D view.

![Log](screens/walkthrough/31_dock_log.png)

### Import Report

Errors, warnings and notes from the last URDF import. It opens by itself only when there are errors.

![Import report](screens/walkthrough/32_dock_import_report.png)

## 5. The 3D view

A clean Move-mode view of the UR5 in a made-up pose:

![Move mode, clean](screens/walkthrough/40_move_mode_clean.png)

**View buttons** (top right): **Front**, **Top**, **Side**, **Iso**, **Fit**.

| Front | Top |
|---|---|
| ![Front](screens/walkthrough/41_view_front.png) | ![Top](screens/walkthrough/41_view_top.png) |
| **Side** | **Iso** |
| ![Side](screens/walkthrough/41_view_side.png) | ![Iso](screens/walkthrough/41_view_iso.png) |

Other 3D behaviour: hover a link to see its name and a tint, click it to select it in the tree, drag to orbit,
wheel to zoom. **Drag TCP** (toolbar or key **T**) lets you pull the tool with the mouse; IK follows and the
collision check holds it back.

**Playback bar** (bottom of the 3D view, appears when a motion is loaded): step back, play/pause, step forward,
a time slider with "0.50 / 1.20 s" and a **Loop** box.

![Playback bar](screens/walkthrough/42_playback_bar.png)

## 6. Analysis overlays

Switch on in Analyze mode -> Overlays tab, or View -> Analysis overlays. Results are cached until the robot or
its tool changes.

**Reach map.** Voxels coloured by dexterity (purple hard to move, yellow free). Options: quality, cut open (hides
the front half), slice at a height, opacity. Shows voxel count, volume and compute time.

![Reach map](screens/walkthrough/70_overlay_reach.png)

**Singularity zones.** Red voxels: the arm loses a direction of motion. Orange: close to it. A threshold slider
(default 0.050), kinds Wrist / Elbow / Shoulder / General, **Export CSV...**, a statistic "unavoidable (best tool
orientation)" or "any".

![Singularity](screens/walkthrough/71_overlay_singularity.png)

**Joint load.** Next to each joint a **T** (torque) bar and an **S** (speed) bar as percent of the limit: green
below 70 %, orange up to 100 %, red above. At rest it shows the gravity load; move the robot and it updates live.

![Joint load](screens/walkthrough/72_overlay_load.png)

## 7. The demo tour

Opening a sample robot from the start screen (or File -> Open Sample Robot) adds five targets: **Pick A**,
**Place B**, **Home-side C** (reachable, green) and **Too far**, **Too close** (unreachable, red). A hint card
at the top of the 3D view offers **Run demo**.

![Sample targets and hint](screens/walkthrough/80_panda_targets_hint.png)

**Run demo** (Target tab or the hint card) visits the targets in order with planned, collision-free motion,
skips the red ones with a message, and returns to Pick A. The status line says "Step k of 6". **Stop demo**
cancels at once and stops the motion. It also works when the Simulation dock is hidden.

| Step | Picture |
|---|---|
| Heading to Pick A | ![demo 0](screens/walkthrough/81_demo_step_0.png) |
| Pick A reached, Too far skipped | ![demo 1](screens/walkthrough/81_demo_step_1.png) |
| Place B | ![demo 2](screens/walkthrough/81_demo_step_2.png) |
| Next target | ![demo 3](screens/walkthrough/81_demo_step_3.png) |
| Later step | ![demo 4](screens/walkthrough/81_demo_step_4.png) |

## 8. Menus

Pictures show each menu open. Items with a dimmed look need a loaded robot or project.

**File**

![File menu](screens/walkthrough/50_menu_1_file.png)

Submenus: **Open Sample Robot**, **Import**, **Export**.

| Open Sample Robot | Import | Export |
|---|---|---|
| ![Samples](screens/walkthrough/55_menu_file_samples.png) | ![Import](screens/walkthrough/56_menu_file_import.png) | ![Export](screens/walkthrough/57_menu_file_export.png) |

**Edit** (Undo, Redo)

![Edit menu](screens/walkthrough/50_menu_2_edit.png)

**View** (every panel, Focus 3D, Reset layout, Robot and Analysis overlays submenus)

![View menu](screens/walkthrough/50_menu_3_view.png)

| View -> Robot | View -> Analysis overlays |
|---|---|
| ![Robot submenu](screens/walkthrough/58_menu_view_robot.png) | ![Overlays submenu](screens/walkthrough/59_menu_view_overlays.png) |

**Robot** (Set TCP)

![Robot menu](screens/walkthrough/50_menu_4_robot.png)

**Help** (Shortcuts and panels, About)

![Help menu](screens/walkthrough/50_menu_5_help.png)

## 9. Help, Focus mode and shortcuts

**F1** shows the shortcut list and what each panel of the current mode does.

![Help dialog](screens/walkthrough/60_help_dialog.png)

The first-run tour can be replayed from Help:

![Tour card](screens/walkthrough/61_tour_card.png)

**Focus 3D** (F11, or toolbar) hides every panel so the 3D view fills the window; press F11 again to return.

![Focus mode](screens/walkthrough/62_focus_3d.png)

**View -> Reset layout** returns to Move mode with default dock sizes.

## 10. Menu and shortcut table

| Action | Where | Shortcut |
|---|---|---|
| New project | File | Ctrl+N |
| Open project | File | Ctrl+O |
| Open sample robot | File | (menu) |
| Save / Save As | File | Ctrl+S / Ctrl+Shift+S |
| Close project | File | (menu) |
| Import URDF / mesh paths | File -> Import | (menu) |
| Export CSV, NPZ, graph, report, URDF, meshes, video, screenshot | File -> Export | (menu) |
| Run script | File | (menu) |
| Quit | File | system quit key |
| Undo / Redo | Edit | Ctrl+Z / Ctrl+Y (or Ctrl+Shift+Z) |
| Focus 3D | View | F11 |
| Reset layout | View | (menu) |
| Show or hide any panel | View | (menu; the mode bar shows the common ones) |
| Analysis overlays | View | (menu) |
| Robot display: frames, floor, links, TCP, visual meshes, Drag TCP, Collision | View -> Robot | T for Drag TCP |
| Set TCP | Robot | (menu) |
| Shortcuts and panels | Help | F1 |
| Switch to Model, Move, Target, Plan, Simulate, Analyze | Mode bar | Ctrl+1 ... Ctrl+6 |

## 11. Ten minute first-run script

1.  (1 min) Start the program. Read the tour card, press **Skip tour**. Click **UR5**.
2.  (1 min) Orbit with the mouse, zoom with the wheel, click **Iso**, then **Fit**.
3.  (1 min) **Ctrl+1** Model: click a link in the tree, read its Properties. **Ctrl+2** Move.
4.  (2 min) Drag two sliders. Click **Z+** four times in the jog pad. Press **Ctrl+Z** twice. Save a pose called
    "mine" with **Save**, then pick *Home* and go back with *mine*.
5.  (1 min) **Ctrl+3** Target: press **New at TCP**, drag the sphere, watch the IK table fill in. Look at the ranked rows (tick
    **Show all solutions as ghosts** to see them).
6.  (1 min) Toolbar **Add object** -> Box, drag it in the way. **Ctrl+4** Plan: **Start = current q**, move the
    robot, **Goal = current q**, **Plan**, **Send to trajectory**.
7.  (1 min) **Ctrl+5** Simulate: press play on the playback bar; open Graphs to see the curves.
8.  (1 min) **Ctrl+6** Analyze -> Overlays: switch on **Reach map**, then **Singularity zones**, then **Joint
    load**.
9.  (1 min) File -> Close Project (discard), pick **Franka Panda**, **Ctrl+3**, press **Run demo** and watch
    the tour skip "Too far" and "Too close".
10. Finish with **F11** (Focus 3D) and a **Screenshot** from the toolbar.
