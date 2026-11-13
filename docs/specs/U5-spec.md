# U5 spec pack: usability check and GUI redesign

Plan: `docs/plans/U3-U6-plan.md`. Common rules: `docs/specs/U3-spec.md` "Common rules" (prefix
`U5.<n>`, tag `stage-U5-done`). Needs U3, U4a, U4b (redesign covers their panels too).

Goal: a new user can load a robot and reach a target in <= 4 clicks and understands each screen
without reading docs. Evidence based: measure before, review with the installed GUI skills, redesign,
measure again.

## Skills (installed claude.ai plugins, 2026-11-14)

- "UI Design Review": `ui-review` (fix list, most important first), `ui-polish`, `ui-compare`.
- "Design": `design-critique`, `ux-copy`, `accessibility-review`, `design-system`.
The orchestrator (not the builder) runs the skills on the screenshots and turns the output into the
U5.3 fix list. Skills target web UIs: use them for layout, hierarchy, wording, contrast; Qt behaviour
is verified by pytest. If the skills are not loaded in the session, start a fresh session for U5.2.

## Code facts

- `app/main_window.py` (cp1252, ~1700 lines): ~30 docks, ~100 actions, `_wire_dock_actions`,
  `_resync_dock_actions`, side bar `tool_bar_side`, Focus 3D (F11), `reset_layout`,
  `LAYOUT_VERSION` in `app/settings.py` (bump to 3 when the layout changes).
- `APP_VERSION = "0.1.0"` in `main_window.py:67` and `app/main.py:29` (wrong, real is 1.x).
- Joint panel `app/panels/joint_panel.py` (`JointRow`, rad display today).

## U5.1 Measure baseline (sonnet)

`tools/ux_tasks.py`: drives the real window (QTest clicks on actual widgets, not internal methods)
through 6 tasks and counts user actions (click, key, typed field):
1 load a sample robot, 2 jog joint 2 by 30 deg, 3 set a target and reach it (U4a), 4 add a box and plan
around it, 5 run a dynamic simulation, 6 compare ur3/ur5. Writes `docs/ux/baseline.json` (actions per
task, time) and real-window screenshots `docs/ux/baseline/*.png` (1600x950 and 1400x900, 150 % scaling).
Tests `tests/python/test_ux_tasks.py` (script runs offscreen, counts stable).

## U5.2 Skill review (orchestrator)

Run `ui-review`, `design-critique`, `ux-copy`, `accessibility-review` on the baseline screenshots
with the task list as context. Write `docs/ux/review-U5.md`: findings ranked (severity, screen, fix).
Opus reviews the fix list. Out: the U5.3 checklist below is final after this step (add/remove items).

## U5.3 Redesign (sonnet, split into substages as needed: U5.3a, U5.3b, ...)

1. Mode bar at the top: Model | Move | Target | Plan | Simulate | Analyze. Each mode = a saved dock
   layout showing only its panels (Model: tree, properties, DH; Move: joints, matrices, collision;
   Target: target + curve, timeline; Plan: planning, collision, objects; Simulate: sim, graphs,
   timeline; Analyze: workspace, IK/analysis, compare, overlays). Other docks reachable via View menu.
   Ctrl+1..6 shortcuts. Mode saved per project.
2. Start screen in the viewport area when no robot: Open sample robot (grid of 9 thumbnails),
   Import URDF, Open project, recent files.
3. Toolbar: icon + text, grouped (File | Move | Collision | View), tooltips with shortcuts.
4. Joints panel: degrees / mm default (rad toggle), limit bar per row (position inside range, red near
   limit), Home and named poses (Home, Ready, user-saved).
5. Cartesian jog: X+/X-/Y+/Y-/Z+/Z- and Rx/Ry/Rz buttons, step (1/10/50 mm, 1/5/15 deg), base/tool frame,
   hold-to-repeat; uses IK + collision blocking.
6. Viewport: view buttons Front/Top/Side/Iso/Fit (overlay toolbar), hover link -> name tooltip +
   outline, click link selects it in the tree.
7. Help: F1 overlay of shortcuts and "what is this panel"; short first-run tour (4 steps, skippable,
   shown once); plain-language messages (from `ux-copy` output).
8. Undo / redo (Ctrl+Z / Ctrl+Y) for q changes, target edits, object edits (QUndoStack).
9. Theme pass: one stylesheet, spacing and font sizes that work at 150 % scaling; contrast fixes from
   `accessibility-review`.
10. Version: `APP_VERSION` from one place (`app/__init__.py __version__ = "1.1.0"`), used by title,
    about, reports.
11. `LAYOUT_VERSION = 3` (old layouts dropped once).
Tests: `tests/python/test_ui_modes.py`, `test_start_screen.py`, `test_cartesian_jog.py`, `test_undo.py`,
update layout tests.

## U5.4 Measure again (sonnet + orchestrator)

Re-run `tools/ux_tasks.py` -> `docs/ux/after.json` + screenshots. Table before/after per task in
`docs/ux/review-U5.md`. Target: task 1 + 3 together <= 4 clicks; no task worse than baseline.
Second skill pass (`ui-review`) on the after screenshots; remaining high items -> U5-fix.

## U5.5 Stage end

Milestone `tests/python/test_milestone_u5.py` (each mode shows its docks, start screen flow, jog,
undo, version string). Full gate. Docs: rewrite `docs/user-guide.md` "Getting started" around modes.
