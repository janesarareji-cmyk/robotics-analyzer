# UI review U5: what was fixed, what was skipped

Skills used on real-window screenshots (`docs/screens/u5/before`, `docs/screens/u5/after`):
`ui-design-review:review` (ranked finding list), `design:accessibility-review` (WCAG 2.1 AA),
`design:design-critique` principles (hierarchy, consistency) for the fixes. The UXKIN server behind
`ui-design-review:compare` failed to connect (certificate error), so `compare` was not used. Full
finding text: [ux/review-U5.md](ux/review-U5.md). Numbers: `docs/ux/baseline.json`, `docs/ux/after.json`.

## Usability numbers (user actions per task, `tools/ux_tasks.py`)

| Task | Before | After |
|------|--------|-------|
| 1 Load a sample robot | 2 | 1 |
| 2 Jog joint 2 by 30 deg | 1 | 1 |
| 3 Set a target and reach it | 4 | 3 |
| 4 Add a box and plan around it | 6 | 5 |
| 5 Dynamic simulation | 4 | 4 |
| 6 Compare ur3 / ur5 | 4 | 4 |
| Task 1 + 3 | 6 | 4 |

Goal (task 1 + 3 at most 4 clicks, no task worse) met. A combo choice counts 2 (open, pick).

## Findings

| # | Finding | Result |
|---|---------|--------|
| 1 | Blank start screen | Fixed: start screen with 9 thumbnail cards, Import URDF, Open project, recent projects (U5.5) |
| 2 | 18 side bar buttons, random highlights | Fixed: six work modes (Ctrl+1..6) replace the side bar (U5.4) |
| 3 | Right dock tabs truncated | Fixed for Move, Target, Plan, Simulate (2-3 tabs). Analyze: Performance moved to the View menu; four tabs remain, the long title "Inverse Kinematics" is kept (tests and menu use it) |
| 4 | Planning table q1..q6 in rad | Half fixed: row headers are the joint names. Skipped: degrees in the table (19 tests and the planner API use radians; the header says rad or m) |
| 5 | Text-only ungrouped toolbar, orange Collision pill | Fixed: icon + text, groups, shortcut tooltips, one accent colour (U5.10) |
| 6 | No limit info, Home, poses | Fixed: limit bar with red zones, Home / Ready / saved poses (U5.6) |
| 7 | No Cartesian jog | Fixed: X/Y/Z/Rx/Ry/Rz, steps, frame, hold to repeat, IK + collision (U5.7) |
| 8 | No view buttons, links not pickable | Fixed: Front/Top/Side/Iso/Fit, hover name + tint, click selects in tree (U5.8) |
| 9 | Model tree text clipped | Skipped: horizontal scroll exists, the tree is only shown in Model mode now |
| 10 | Hard coded "GPU: RTX 5060" in the status bar | Fixed: real backend ("Compute: GPU (CUDA)" / "Compute: CPU") |
| 11 | Ambiguous side bar "Reset" | Fixed by removing the side bar; Reset layout stays in the View menu |
| 12 | Developer wording in messages | Fixed for the start pose and collision check messages; others left |
| 13 | Version 0.1.0 | Fixed: `app.__version__` 1.1.0 in title, About, reports, CSV, plugins |
| 14 | Tour, F1 help | Fixed: four step tour (first run), F1 overlay per mode (U5.9) |
| 15 | Undo / redo | Fixed: Ctrl+Z / Ctrl+Y for joints, targets, objects (U5.9) |
| 16 | Dock title duplicates the tab title | Skipped: Qt dock design, low value |

Found while reading the after screenshots and fixed (U5-fix): an object added to the scene logged "Updated
'box 1'" ten times (the drag widget calls back when it is created; now ignored when the position
did not change), the Undo button in the toolbar grew with the step name (text stays "Undo", the tool tip
names the step), the status text "Collision check ready" was drawn red (reworded), and the New at TCP
button crashed on click (found by the measurement in U5.1).

## Accessibility (WCAG 2.1 AA)

-   Contrast: body text 11:1, muted text 7:1, white on accent 5:1, mode bar, cards and tour card
    at least 4.5:1 (computed in `tests/python/test_ui_theme.py`).
-   Focus: Fusion draws a focus frame on every control; mode bar and cards add a white 2 px frame.
-   Names: mode buttons, jog buttons, view buttons, cards, pose list and toolbar groups have accessible
    names and descriptions.
-   Keyboard: modes Ctrl+1..6, F1, Ctrl+Z / Ctrl+Y, F11, T; Tab reaches every control.
-   Target size: jog buttons at least 28 px high, mode buttons 32 px.
-   Not done: a screen reader test (NVDA) and 200 % zoom check; the layout was only checked at 100 % on
    1600x950 and 1400x900.

## Deferred (not in U5)

-   Degrees in the Planning table.
-   Hide the docks while no robot is loaded (start screen only).
-   Timeline in the Target mode (the Simulation dock holds it; Target mode shows Target and Curve).
-   Screen reader and 200 % scaling pass.
