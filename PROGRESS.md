# Progress and Handoff

Shared status file between the Builder (Gemini, Antigravity) and the
Reviewer (Claude Sonnet, Claude Code). Read this file first. Update it at
every handoff. Keep it short.

## Current

-   **Stage:** R2 --- Custom robot modelling
-   **Substage:** R2.8 --- Joint control panel + TCP pose readout (ready-for-review)
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
| R2.4  | URM hardening                             | test_urm.cpp, test_urm_bridge.py   | approved |
| R2.5  | FK engine (DH + MDH), C++ + bindings      | test_fk.cpp, test_fk_bridge.py     | approved (follow-ups → R2.6) |
| R2.6  | Wire model + DH editor into app/project   | test_robot_in_project.py           | approved |
| R2.7  | Viewport: links, frames, TCP              | test_viewport_robot.py             | approved (follow-ups → R2.8) |
| R2.8  | Joint control panel + TCP pose readout    | test_joint_panel.py                | ready-for-review |
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

### R2.5 --- FK engine (DH + MDH), C++ + bindings

Goal: forward kinematics from `RobotModel` → all frame poses + TCP. No UI.

Files: new `core/kin/fk.hpp`, `core/kin/fk.cpp` (namespace `kin`), add
`kin/fk.cpp` to `core_lib` in `core/CMakeLists.txt`. Bindings in new
`bindings/kin_bindings.cpp`, registered in `core_bridge.cpp` like
`bind_urm`. Use `#include <pybind11/eigen.h>`. New CTest target `test_fk`
in `tests/cpp/CMakeLists.txt` (copy `test_urm` pattern).

1.  `Eigen::Isometry3d pose_to_iso(const urm::Pose&)`:
    translation = xyz; rotation `R = Rz(yaw)·Ry(pitch)·Rx(roll)`,
    rpy = (roll, pitch, yaw). Use `Eigen::AngleAxisd`, nothing hand-rolled.
2.  `Eigen::Isometry3d dh_transform(a, alpha, d, theta)` --- standard DH:
    `A = Rz(θ) · Tz(d) · Tx(a) · Rx(α)`.
3.  `Eigen::Isometry3d mdh_transform(a, alpha, d, theta)` --- modified DH
    (Craig): `A = Rx(α) · Tx(a) · Rz(θ) · Tz(d)`. Here `a`, `alpha` stored
    on joint i are Craig's a_{i-1}, α_{i-1}. Write this in `fk.hpp` and
    update the `MDHParams` doc comment in `joint.hpp` to say it.
4.  Joint variable (from R2.4): revolute θ_eff = θ + q; prismatic
    d_eff = d + q; fixed → q ignored, transform still applied.
    `robot.convention` picks `dh` or `mdh` for every joint.
5.  `struct FKResult { std::vector<Eigen::Isometry3d> frames; Eigen::Isometry3d tcp; }`.
    `frames.size() == joints.size() + 1`. `frames[0] = base`.
    `frames[i+1] = frames[i] · A_i`. `tcp = frames.back() · tool`.
    All poses in world frame.
6.  `FKResult forward_kinematics(const urm::RobotModel&)` uses each
    `joint.q`. Overload `forward_kinematics(const urm::RobotModel&,
    const std::vector<double>& q)` takes `q` of size `dof()` (non-fixed
    joints, in order); wrong size → `std::invalid_argument`. Overload must
    NOT modify the model (take const ref, copy q internally).
7.  No `validate()` call inside FK. No allocation beyond the result vector.
8.  Python: `core_bridge.fk(model)` and `core_bridge.fk(model, q)` return
    dict `{"frames": [np 4x4, ...], "tcp": np 4x4}` (float64, homogeneous
    `.matrix()`). Also bind `pose_to_iso` → np 4x4. Wrong-size q →
    `ValueError`.

Tests C++ `tests/cpp/test_fk.cpp` (tol 1e-12 unless noted):
-   1R: a=1, q=π/2 → TCP xyz = (0, 1, 0).
-   2R planar a1=a2=1: q=(0,0) → (2,0,0); q=(π/2,−π/2) → (1,1,0).
-   3R planar closed form: a=(1, 0.8, 0.5), 100 deterministic q samples
    (fixed seed, std::mt19937 seed 42, uniform [−π, π]).
    x = Σ a_i cos(q1+…+qi), y = Σ a_i sin(…), yaw = Σ q. Match.
-   Prismatic: d=0.2, q=0.3 → TCP z = 0.5.
-   Fixed joint: contributes transform, `dof()` unchanged, q ignored
    (set q=99 → same result).
-   DH vs MDH equivalence: same 2R planar arm in MDH (j1 a=0, j2 a=1,
    tool xyz=(1,0,0)) gives same TCP as DH version for 20 q samples.
-   Base + tool: base yaw π/2 rotates TCP of 1R arm; tool xyz offset adds
    along last frame x.
-   `pose_to_iso` order: rpy=(0.3, −0.7, 1.1) equals explicit
    `Rz(1.1)·Ry(−0.7)·Rx(0.3)` built from AngleAxis.
-   Every frame rotation orthonormal, det = +1 (tol 1e-12).
-   `frames.size() == joints.size()+1`; wrong-size q throws
    `std::invalid_argument`; overload leaves `model.joints[i].q` unchanged.

Tests Python `tests/python/test_fk_bridge.py`:
-   `fk(model)` returns dict; frames list len = joints+1; each ndarray
    shape (4,4), dtype float64; tcp shape (4,4).
-   2R planar result equals C++ case above.
-   `fk(model, q)` wrong length → `ValueError`.
-   `pose_to_iso` returns (4,4), last row `[0,0,0,1]`.

Done when: `.\run_tests.ps1` all green (5 CTest targets), `graphify
update .` run, row set to `ready-for-review`, commit `R2.5: FK engine`.

Changed files:
- `core/kin/fk.hpp`
- `core/kin/fk.cpp`
- `core/CMakeLists.txt`
- `core/urm/joint.hpp`
- `bindings/kin_bindings.cpp`
- `bindings/core_bridge.cpp`
- `bindings/CMakeLists.txt`
- `tests/cpp/test_fk.cpp`
- `tests/cpp/CMakeLists.txt`
- `tests/python/test_fk_bridge.py`

### R2.5 follow-ups (do first, inside R2.6 commit, small)

Reviewer found tests weak. Add to `tests/cpp/test_fk.cpp`:
-   Known-answer 3D case, standard DH: 2-link arm, link1 a=0 alpha=π/2 d=0.5, link2 a=0.4 alpha=0 d=0; q=(π/2, 0). Compute expected TCP by hand and hard-code it (document the derivation in a comment). Must NOT call `dh_transform`/`mdh_transform` to build the expected value.
-   Same arm in MDH with nonzero alpha (j1 alpha=0 a=0 d=0.5; j2 alpha=π/2 a=0 d=0, tool xyz=(0.4,0,0)): hard-coded expected value, different from DH one if physically different arm — state in comment which arm it is.
-   Prismatic joint under MDH: d=0.2, q=0.3 → z=0.5.
-   Wrong-size q too LONG also throws `std::invalid_argument`.

Also Python: `fk(model, q)` with q too long → `ValueError`.

### R2.6 --- Wire model + DH editor into app/project

Goal: RobotModel lives in the project; DH editor is visible in MainWindow and edits it. No 3D drawing yet (R2.7).

Builder: run `graphify query` on ProjectData, ProjectController, MainWindow, DH editor first. Reuse existing classes.

1.  `ProjectData` holds one `urm.RobotModel` (default empty, convention Standard). Serializer saves/loads it via URM JSON (`to_json`/`from_json`) inside the project file. Old project files without robot → empty model. Bad robot JSON → project load fails with clear error, no crash.
2.  `ProjectController`: `robot` getter; `set_robot(model)`; `robot_changed` Qt signal; marks project dirty; undo not required.
3.  MainWindow shows DH editor in a dock (right side, toggle in View menu). Editor loads from controller on project open/new, and writes back to controller on every edit.
4.  Editor applies `urm.default_limits(type)` when user changes joint type. Editor assigns whole lists to `Pose.xyz`/`rpy` (see Open review items).
5.  Editor shows `model.validate()` errors in a label under the table (empty = hidden). Invalid model is still allowed to be edited and saved.
6.  Editor shows/edit robot `convention` (Standard/Modified combobox) and base/tool pose (6 spinboxes each).
7.  Log panel gets one line on robot load and on validate errors.

Tests `tests/python/test_robot_in_project.py`: save/load round-trip with 3-joint robot; old file without robot loads empty; bad robot JSON → error not crash; editing in editor changes controller robot + dirty flag; `robot_changed` emitted; joint type change applies default limits; validate errors shown; convention + base/tool edits persist through save/load; dock toggle in View menu.

Done when: `.\run_tests.ps1` green, `graphify update .` run, row R2.6 set to `ready-for-review`, commit `R2.6: wire robot model into project`.

Changed files:
- `app/project/project_data.py`
- `app/project/serializer.py`
- `app/project/project_controller.py`
- `app/panels/dh_editor.py`
- `app/main_window.py`
- `tests/python/test_robot_in_project.py`
- `tests/python/test_project.py`
- `tests/cpp/test_fk.cpp`

### R2.7 --- Viewport: links, frames, TCP

Goal: draw the robot in the PyVista viewport from FK. Read-only view. No joint sliders yet (R2.8).

Builder: `graphify query` on the viewport widget, ProjectController, `kin_bindings`/`core_bridge.fk` first. Reuse the existing viewport class from R1.7. Do not touch C++ unless a binding is missing.

1.  New `app/viewport/robot_view.py` (or next to the existing viewport code): class `RobotView` that owns the PyVista actors for one robot and has `update(model, fk_result)`. It must NOT call FK itself, caller passes `core_bridge.fk(model)` result. Pure draw logic.
2.  Draw: (a) one coordinate-frame triad per `frames[i]` (RGB = XYZ axes, length 0.05 m default); (b) a polyline through the origins of all frames, then to TCP; (c) a joint marker at each non-fixed joint frame origin (sphere radius 0.015 m for revolute, small cube for prismatic); (d) a larger triad at TCP. Base frame (frames[0]) drawn with the world triad style distinct from link frames.
3.  Update in place: `update()` re-uses actors (set new transforms/points). No remove-all/re-add per call. Actor count constant while joint count is constant. Joint count change → rebuild.
4.  Empty model (0 joints): draw only base triad + TCP triad (TCP == base · tool). No crash.
5.  Toggles on `RobotView`: `set_show_frames(bool)`, `set_show_links(bool)`, `set_show_tcp(bool)`. Add a View-menu submenu "Robot" with 3 checkable actions wired to them (default all on).
6.  MainWindow: on `project_opened` and `robot_changed`, compute `core_bridge.fk(model)` and call `RobotView.update`. If model fails FK (exception) → log warning, keep last drawing. Camera reset only on project_opened, not on every robot_changed.
7.  Triad length and marker sizes scale: default from `max(0.05, 0.05 * total_reach)` where total_reach = sum of |a|+|d| over joints; fall back to 0.05.

Tests `tests/python/test_viewport_robot.py` (offscreen, follow `test_viewport_in_main_window.py` pattern):
-   3R planar model: actor count equals expected (triads + polyline + markers); polyline points == frame origins + TCP (assert numerically, tol 1e-9).
-   `update()` with new q moves polyline points to new FK result, actor count unchanged.
-   Empty model draws base + TCP only, no exception.
-   Toggles hide/show each group (assert actor visibility).
-   MainWindow integration: set_robot on controller → polyline updates; FK failure path logs warning and keeps last points.
-   Camera not reset on robot_changed (record camera position before/after).
-   View > Robot submenu has 3 checkable actions, default checked.

Done when: `.\run_tests.ps1` green, `graphify update .` run, row R2.7 set to `ready-for-review`, commit `R2.7: robot viewport`.

Changed files:
- `app/viewport/viewport_widget.py`
- `app/viewport/robot_view.py`
- `app/viewport/__init__.py`
- `app/main_window.py`
- `app/panels/log_panel.py`
- `tests/python/test_viewport_robot.py`

### R2.8 --- Joint control panel + TCP pose readout

Goal: user moves joints with sliders/spinboxes, viewport and TCP readout update live. No new C++.

Builder: `graphify query` on MainWindow, ProjectController, RobotView, DH editor first. Reuse.

R2.7 follow-ups (do first, small):
-   `RobotView`: rebuild actors when computed scale (spec R2.7 item 7) changes by more than 1% (store last scale). Test: change a link `a` from 0.1 to 2.0 → triad size grows.
-   Connect `project_closed` to `RobotView.clear()` (viewport empty after close). Test it.

R2.8 items:
1.  New `app/panels/joint_panel.py`: `JointPanel(QWidget)` in a new right-side dock "Joints" (View-menu toggle, default on, tabbed with the DH editor dock). One row per NON-fixed joint: name label, QSlider + QDoubleSpinBox (synced), unit label (deg for revolute, mm for prismatic; internal values stay rad / m). Range = joint limits. Row for fixed joints not shown.
2.  Slider/spinbox change → writes `joint.q` on the controller's model and emits `q_changed`. MainWindow recomputes FK and updates RobotView. Use whole-object assignment rules from R2.4 notes where needed. Must not rebuild the panel on each change (only on joint count/type/limits change).
3.  Out-of-limit q (e.g. loaded from file): panel shows it clamped in the widget but does NOT silently change the model; show a small warning icon/label on that row. Slider drag never produces q outside limits.
4.  Buttons: "Zero all" (q=0 clamped to limits) and "Reset" (restore q values from when the project/robot was loaded or last edited in DH editor).
5.  TCP readout (same dock, below sliders): x, y, z in mm; roll, pitch, yaw in deg (use R = Rz·Ry·Rx extraction consistent with `pose_to_iso`; write helper `rotation_to_rpy(R)` in Python with gimbal-lock handling at pitch=±90°). Readout updates on every FK recompute. Also a "Copy pose" button that puts the readout text on the clipboard.
6.  q edits mark project dirty. q is saved in the URM JSON already.
7.  DH editor edit (joint count, type, limits, convention) → JointPanel rebuilds itself. JointPanel ↔ DH editor never fight (use update guards like `_updating`).
8.  Log: no log line per slider tick. One log line on Zero all / Reset.

Tests `tests/python/test_joint_panel.py`:
-   3R model: panel has 3 rows; fixed joint adds no row; prismatic row uses mm.
-   Setting spinbox deg value changes `model.joints[i].q` in rad; slider and spinbox stay in sync.
-   Slider change → RobotView polyline points == `cb.fk(model)` origins (tol 1e-9).
-   TCP readout of 2R planar at q=(90°,−90°) → x=1000mm, y=1000mm, z=0, yaw=0 (a1=a2=1 m).
-   `rotation_to_rpy` round-trip against `pose_to_iso` for 50 random rpy (pitch within ±80°), tol 1e-9; gimbal-lock case does not return NaN.
-   Out-of-limit q: model unchanged, warning shown.
-   Zero all and Reset behave as specified.
-   DH editor type change/joint add → panel rebuilds, row count correct.
-   Dirty flag set on q edit.
-   Follow-ups: scale rebuild test; project close clears viewport.

Done when: `.\run_tests.ps1` green, `graphify update .` run, row R2.8 set to `ready-for-review`, commit `R2.8: joint panel + TCP readout`.

Changed files: `app/panels/joint_panel.py`, `app/main_window.py`, `app/project/project_controller.py`, `app/viewport/robot_view.py`, `tests/python/test_joint_panel.py`, `tests/python/test_viewport_robot.py`.

### R2.9 onward

Spec written by Reviewer when R2.8 is approved.

## Open review items

-   (info) R2.6 DH editor: assign whole lists to `Pose.xyz`/`rpy`
    (`model.base.xyz = [...]`); element assignment is silently lost.
-   (info) R2.5 weak tests (no 3D known-answer) → follow-ups at top of R2.6.
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
