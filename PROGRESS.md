# Progress and Handoff

Shared status file between the Builder (Gemini, Antigravity) and the
Reviewer (Claude Sonnet, Claude Code). Read this file first. Update it at
every handoff. Keep it short.

## Current

-   **Stage:** R2 --- Custom robot modelling
-   **Substage:** R2.4 --- URM hardening (ready for review)
-   **Owner now:** Reviewer
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
| R2.1  | URM schema + JSON round-trip              | tests/cpp/test_urm.cpp             | approved (follow-ups → R2.4) |
| R2.2  | URM pybind11 bindings                     | tests/python/test_urm_bridge.py    | approved |
| R2.3  | DH editor widget                          | tests/python/test_dh_editor.py     | approved (wiring → R2.6) |
| R2.4  | URM hardening                             | test_urm.cpp, test_urm_bridge.py   | ready-for-review |
| R2.5  | FK engine (DH + MDH), C++ + bindings      | test_fk.cpp, test_fk_bridge.py     | planned |
| R2.6  | Wire model + DH editor into app/project   | test_robot_in_project.py           | planned |
| R2.7  | Viewport: links, frames, TCP              | test_viewport_robot.py             | planned |
| R2.8  | Joint control panel + TCP pose readout    | test_joint_panel.py                | planned |
| R2.9  | 3-DOF end-to-end milestone test           | test_milestone_r2.py               | planned |

R0--R1 rows marked `approved` from existing code and tests. Reviewer has
not re-run them in this file's first version.

## Substage specs

### R2.4 --- URM hardening (C++ core + bindings + tests)

Goal: make the URM ready for FK, frames and TCP. No UI work.

1.  `RobotModel` gains `int schema_version = 1`. Write it to JSON as
    `"schema_version"`. Reading JSON without it → treat as 1. Reading
    `schema_version > 1` → throw `std::runtime_error("unsupported
    schema_version N")`. Do not half-load.
2.  DH convention moves from joint to robot. Add
    `enum class DHConvention { Standard, Modified }` and
    `RobotModel::convention = DHConvention::Standard`. JSON key
    `"convention"`: `"standard"` | `"modified"`. **Remove**
    `Joint::use_mdh` (C++, JSON write, bindings). Joints keep both `dh` and
    `mdh` structs; `convention` picks which one FK reads.
    Legacy read (no `"convention"` key): if every joint has
    `"use_mdh": true` → Modified; if none → Standard; mixed → throw.
    Bind `DHConvention` + `convention` to Python.
3.  Add `struct Pose { std::array<double,3> xyz{}; std::array<double,3> rpy{}; }`
    (metres, radians). Rotation is fixed-axis XYZ:
    **`R = Rz(yaw) · Ry(pitch) · Rx(roll)`**, rpy = (roll, pitch, yaw).
    Write this formula in the `Pose` doc comment.
    `RobotModel` gains `Pose base` (world → frame 0) and `Pose tool`
    (last frame → TCP). Serialize both. Missing in JSON → identity. Bind
    both to Python.
4.  Joint variable convention (document in `joint.hpp` comments):
    revolute → θ_eff = θ + q; prismatic → d_eff = d + q; fixed → q ignored.
    Same rule for MDH.
5.  Default limits by type: add a free function
    `JointLimits default_limits(JointType)`. Revolute ±π rad, prismatic
    0..0.5 m, fixed 0..0 (vel/effort keep current defaults). Used in
    exactly two places: (a) `from_json` when a joint has no `"limits"` key
    → `default_limits(type)`; (b) bound to Python as `urm.default_limits`
    so the DH editor (R2.6) can apply it when the user changes joint type.
    `Joint()` C++/Python default stays revolute ±π. No setter magic.
6.  `RobotModel::set_q(q)`: throw `std::invalid_argument` if
    `q.size() != dof()` (Python sees `ValueError`). No silent partial set.
7.  Add `std::vector<std::string> RobotModel::validate() const` returning
    human-readable errors (empty = valid). Checks: `links.size() ==
    joints.size() + 1` (when joints non-empty); `lower <= upper`; `q`
    inside limits for non-fixed joints; unique joint names; finite numbers.
    Bind to Python. `from_json` does **not** call `validate()` —
    load and validate stay separate (UI must be able to open a bad model
    and show the errors).
8.  `from_json` must not crash on bad input: catch nlohmann exceptions and
    throw one clear `std::runtime_error` (Python sees `RuntimeError`).
    Unknown joint `"type"` string → throw (no silent fallback to revolute).
9.  Fix doc comment in `robot_model.hpp`: `joints` includes fixed joints;
    `dof()` counts non-fixed only.

Tests (all must pass, plus all old tests):
-   C++: schema_version round-trip, missing-version default, version 2
    throws; convention round-trip, legacy use_mdh all/none/mixed;
    base/tool round-trip and missing → identity; `default_limits` per type
    and applied when JSON lacks `"limits"`; `set_q` wrong length throws;
    each validate() rule has one passing and one failing case; malformed
    JSON throws; unknown joint type throws.
-   Python: base/tool read/write via bindings; `convention` read/write;
    `default_limits` callable; `set_q` wrong length → `ValueError`;
    `validate()` returns list of str; `RuntimeError` on bad JSON.
-   Remove/adjust any old test that uses `Joint.use_mdh`.

Done when: `.\run_tests.ps1` all green, `graphify update .` run, row set
to `ready-for-review`, commit `R2.4: URM hardening`.

Changed files:
- `core/urm/joint.hpp`
- `core/urm/robot_model.hpp`
- `core/urm/robot_model.cpp`
- `bindings/urm_bindings.cpp`
- `tests/cpp/test_urm.cpp`
- `tests/python/test_urm_bridge.py`

### R2.5 onward

Spec written by Reviewer when R2.4 is approved.

## Open review items

-   (info) DH editor is not shown in MainWindow yet → handled in R2.6.
-   (info) URM is a serial chain (vector), not a tree. OK for R2. Revisit
    before R3 URDF import (branching robots).

## Decisions

-   2026-11-11: Pinocchio deferred; custom C++20/Eigen engine (R0.8).
-   2026-11-11: Roles --- Gemini 3.8 Medium builds, Sonnet 5.5 reviews,
    Opus advises only when absolutely necessary.
-   2026-11-11: DH/MDH convention is per robot, not per joint (R2.4).
    `from_json` loads without validating; `validate()` is explicit.

## Risks

-   CUDA Toolkit 13.5 vs driver max 13.1. Update driver before R5.
