# U5 review (2026-11-14)

Evidence: `docs/ux/baseline.json` (actions per task), real-window screenshots in
`docs/screens/u5/before/` (1600x950, 1400x900) and `docs/ux/baseline/`. Skills used:
`ui-design-review:review` (finding list), `design:accessibility-review` (WCAG 2.1 AA checks),
`design:design-critique` principles (hierarchy, consistency). The UXKIN comparison server failed to
connect (certificate), so `compare` was not used.

## Baseline (actions per task)

| Task | Baseline |
|------|----------|
| 1 Load a sample robot | 2 (toolbar button, menu item) |
| 2 Jog joint 2 by 30 deg | 1 |
| 3 Set a target and reach it | 4 (side bar Target, New at TCP, type X, Play) |
| 4 Box + plan | 6 |
| 5 Dynamic simulation | 4 |
| 6 Compare ur3/ur5 | 4 |
| Task 1 + 3 | 6 |

Bug found by the measurement: the "New at TCP" button passed the Qt `checked` flag as the pose
(`ValueError: target pose must be a finite 4x4 matrix`). Fixed in U5.1.

## Findings (ranked)

### Must fix
1. Empty state is a blank dark viewport (01_empty). No hint what to do. Fix: start screen with
   9 sample robot cards, Import URDF, Open project, recent files.
2. Side bar has 18 text buttons in one column, with three random buttons highlighted (Model,
   Joints, Log) that mean "dock is open". No order, no grouping, no task meaning. Fix: 6 mode bar
   (Model, Move, Target, Plan, Simulate, Analyze), each mode shows only its docks; other docks via
   View menu.
3. Right dock tab strip carries up to 8 tabs and truncates names ("Simulati...", "Tar...",
   06_simulation). Fix: modes show 2-4 docks only, tab names fit.
4. Planning table: columns "q1..q6" and "[rad|m]" while the Joints panel shows names in deg
   (05_planning). Fix: joint names as row headers, degrees/mm in the table.
5. Toolbar is text only, ungrouped, "Collision" is an unexplained orange pill. Fix: icons + text,
   groups File | Move | Collision | View, tooltips with shortcuts, checked state readable.

### Should fix
6. Joint rows: no limit information, no Home, no named poses (02_loaded). Fix: limit bar per row,
   red near limit; Home / Ready / saved poses.
7. No Cartesian jogging: only joint sliders or dragging the TCP. Fix: X/Y/Z, Rx/Ry/Rz buttons with
   step and frame, hold to repeat, IK + collision blocking.
8. Viewport has no view buttons (front/top/side) and links cannot be picked. Fix: overlay
   Front/Top/Side/Iso/Fit, hover name, click selects link in the tree.
9. Model tree text clipped at the right ("shoulder_lift_joint [revolute" ...). Fix: elide + tooltip,
   already has horizontal scroll; widen default.
10. Status bar shows a hard coded "GPU: RTX 5060 Laptop". Fix: real backend text.
11. Side bar "Reset" is ambiguous (reset what?). Fix: "Reset layout" in View menu and tooltip.
12. Plain-language messages: "Start pose collided: moved to a free pose (distance 5.03 rad)",
    "Auto collision matrix ready" are developer text. Fix: wording pass.
13. Version shows 0.1.0 (title bar). Fix: one `__version__`.

### Nice to have
14. First-run tour (4 steps), F1 shortcut overlay.
15. Undo / redo for joint, target and object edits.
16. Dock title bars duplicate the tab title (Robot Model / Robot Model).

## Accessibility (WCAG 2.1 AA, desktop reading)

| Element | Result |
|---------|--------|
| Body text #cccccc on #1a1a1a | about 11:1, passes 1.4.3 |
| Grey unit labels #888888 on #1e1e1e | about 4.7:1, passes narrowly; keep, never go darker |
| Disabled buttons (Start/Pause/...) | low contrast by design, but state is also in text; ok |
| Checked side bar button white on #3d6fcc | about 4.9:1 passes |
| Orange "Collision" toggle, white text | needs check, replace by the same checked style as others (1.4.11) |
| Keyboard | Tab reaches every field; focus ring not visible on buttons and sliders (2.4.7): add a 2 px focus outline in the stylesheet. Mode shortcuts Ctrl+1..6, F1 help |
| Names for screen readers (4.1.2) | icon-only buttons need accessible names; mode and jog buttons get `setAccessibleName` |
| Targets (2.5.5, desktop) | side bar and jog buttons at least 28 px high |
