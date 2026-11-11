# Progress and Handoff

Shared status file between the Builder (Gemini, Antigravity) and the
Reviewer (Claude Sonnet, Claude Code). Read this file first. Update it at
every handoff. Keep it short.

## Current

-   **Stage:** R2 --- Custom robot modelling
-   **Substage:** R2.4 (next, not started)
-   **Owner now:** Reviewer --- confirm R2.1--R2.3 before R2.4 starts
-   **Blockers:** none known

## Handoff protocol

1.  Builder finishes a substage → runs tests → runs `graphify update .`
    → fills a row in the log below with status `ready-for-review`.
2.  Reviewer reads this file + graph → reviews → sets status `approved`
    or `changes-requested` and writes findings under "Open review items".
3.  Builder fixes → sets status `ready-for-review` again.
4.  Failure that blocks a substage → add `<id>-fix` row right after it.
5.  Advisor (Opus) consulted → record it in "Decisions".

Status values: `planned` · `in-progress` · `ready-for-review` ·
`changes-requested` · `approved` · `blocked`

## Substage log

| ID    | Summary                                   | Tests                              | Status   |
|-------|-------------------------------------------|------------------------------------|----------|
| R0.1  | Hardware audit                            | docs/hardware-audit.md             | approved |
| R0.2  | C++20 core_lib + CMake                    | tests/cpp/test_placeholder.cpp     | approved |
| R0.3  | (not tagged in code --- fill in)          | ?                                  | ?        |
| R0.4  | pybind11 bridge                           | tests/python/test_bridge.py        | approved |
| R0.5  | Eigen 3.4.0                               | tests/cpp/test_eigen.cpp           | approved |
| R0.6  | CUDA vector-add on SM 12.0                | tests/cpp/test_cuda.cpp            | approved |
| R0.7  | PyVista viewport in Qt                    | tests/python/test_viewport_smoke.py| approved |
| R0.8  | Pinocchio decision (deferred)             | docs/hardware-audit.md             | approved |
| R1.1  | Main window shell                         | tests/python/test_ui_smoke.py      | approved |
| R1.2  | AppSettings                               | tests/python/test_settings.py      | approved |
| R1.3  | Logger + log panel                        | tests/python/test_logger.py        | approved |
| R1.4  | TaskRunner                                | tests/python/test_task_runner.py   | approved |
| R1.5  | ProjectData + serializer                  | tests/python/test_project.py       | approved |
| R1.6  | ProjectController                         | tests/python/test_project_controller.py | approved |
| R1.7  | Viewport in main window                   | tests/python/test_viewport_in_main_window.py | approved |
| R2.1  | URM schema + JSON round-trip              | tests/cpp/test_urm.cpp             | ready-for-review |
| R2.2  | URM pybind11 bindings                     | tests/python/test_urm_bridge.py    | ready-for-review |
| R2.3  | DH editor widget                          | tests/python/test_dh_editor.py     | ready-for-review |

R0--R1 rows marked `approved` from existing code and tests. Reviewer has
not re-run them in this file's first version.

## Next up (R2 remaining, per ROADMAP)

-   Modified DH editor
-   R/P/fixed joints in UI
-   Frames, base and TCP
-   Interactive joint controls
-   FK + frame/TCP display → **first end-to-end milestone** (3-DOF robot)

## Open review items

-   none

## Decisions

-   2026-11-11: Pinocchio deferred; custom C++20/Eigen engine (R0.8).
-   2026-11-11: Roles --- Gemini 3.8 Medium builds, Sonnet 5.5 reviews,
    Opus advises only when absolutely necessary.

## Risks

-   CUDA Toolkit 13.5 vs driver max 13.1. Update driver before R5.
