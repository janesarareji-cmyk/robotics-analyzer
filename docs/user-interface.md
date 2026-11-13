# The user interface (U5)

How the window is organised and how to do the common things with the fewest clicks. Measured with
`tools/ux_tasks.py`; the review is in [ui-review-u5.md](ui-review-u5.md).

## Start screen

With no robot loaded the 3D view shows the start screen: nine sample robot cards (picture and name),
**Import URDF...**, **Open project...** and the last five projects. One click on a card loads the robot.
You can also drop a `.urdf` or `.robotproj` file on the window.

## Work modes

The bar under the toolbar has six modes. A mode shows only the panels for that job; every other panel
stays reachable from the **View** menu.

| Mode | Shortcut | Panels |
|------|----------|--------|
| Model | Ctrl+1 | Robot Model, Properties, DH editor, Physical |
| Move | Ctrl+2 | Joints (sliders, poses, Cartesian jog), Matrices, Collision |
| Target | Ctrl+3 | Target, Curve |
| Plan | Ctrl+4 | Planning, Collision, Ghosts |
| Simulate | Ctrl+5 | Simulation (with timeline), Graphs |
| Analyze | Ctrl+6 | Workspace, Inverse Kinematics, Analysis, Compare (Performance: View menu) |

The Target mode makes a first target at the tool pose when the project has none. The mode is remembered
for each saved project. Log, Script console and Plugins keep their own state. **View -> Reset layout**
goes back to Move mode with the default sizes.

## Joints panel

-   Values are degrees (mm for sliding joints). Under each slider a thin bar shows the position inside
    the range; the two end zones are red and the marker turns red when the joint is near a limit.
-   **Pose** list: *Home* (the start pose), *Ready* (middle of every range) and your saved poses.
    **Save** stores the current joint values under a name in the project (they also appear in the Ghosts
    and Planning panels). **Delete** removes a saved pose.
-   **Move the tool**: X-, X+, Y-, Y+, Z-, Z+ move the tool; Rx, Ry, Rz turn it. Steps 1 / 10 / 50 mm and
    1 / 5 / 15 deg. The axes are those of the robot base or of the tool. Hold a button to repeat. The
    robot is held back by the collision check (switch **Collision** off to pass through objects), and a
    plain message says when a joint limit or the reach stops the tool.

## 3D view

Buttons **Front**, **Top**, **Side**, **Iso**, **Fit** at the top right. Hover a link to see its name; it is
tinted. Click a link to select it in the Robot Model tree (Model mode shows the tree and Properties).
A drag orbits the view and is not a click.

## Undo and redo

**Ctrl+Z** and **Ctrl+Y** (Edit menu, toolbar) undo joint moves (slider, jog, drag TCP, IK Apply, poses),
target edits and scene object edits. A slider drag or a held jog button is one step. The history is
cleared when another robot or project is opened.

## Help

**F1** shows the shortcuts and what each panel of the current mode does. The first time the program
starts, a four step tour appears over the 3D view; **Skip tour** ends it and it does not come back. Help
-> Shortcuts and panels shows the help again.

## Look

One dark theme (Fusion style, one blue accent) on every machine, independent of the Windows accent
colour. Text and controls meet WCAG AA contrast (4.5:1); the Tab key reaches every control and shows a
focus frame. The toolbar has icon and text in groups (File, Move, Scene, View) and every tooltip names
its shortcut.
