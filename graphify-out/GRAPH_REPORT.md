# Graph Report - Analyzer Project  (2026-11-11)

## Corpus Check
- 61 files · ~27,634 words
- Verdict: corpus is large enough that graph structure adds value.
- Unclassified: 3 file(s) not represented in the graph (top: (none) 2, .ini 1)

## Summary
- 961 nodes · 1680 edges · 72 communities (38 shown, 34 thin omitted)
- Extraction: 89% EXTRACTED · 11% INFERRED · 0% AMBIGUOUS · INFERRED: 188 edges (avg confidence: 0.88)
- Token cost: 0 input · 0 output

## Graph Freshness
- Built from commit: `28498c16`
- Run `git rev-parse HEAD` and compare to check if the graph is stale.
- Run `graphify update .` after code changes (no API cost).

## Community Hubs (Navigation)
- Software Requirements Specification
- ProjectController
- MainWindow
- Technical Architecture
- test_bridge.py
- Development Workflow
- Agent Communication Style Rules
- Hardware Audit — R0.1
- test_robot_in_project.py
- TaskRunner
- ProjectData
- test_viewport_smoke.py
- .set_model
- .add_recent_project
- ._on_project_opened
- Task
- robot_model.cpp
- from_json
- ._setup_dock_panels
- LogPanel
- test_urm_bridge.py
- test_logger.py
- test_task_runner.py
- test_project_controller.py
- RobotModel
- test_ui_smoke.py
- fixture
- .viewport
- .show_grid
- Joint
- FunctionTask
- .show_coordinate_frames
- Link
- test_task_error_signal
- fk.cpp
- .save_project
- test_main_window_has_task_runner
- DHEditorWidget
- test_settings.py
- test_fk.cpp
- test_project.py
- AddTask
- RobotMetadata
- .closeEvent
- core_bridge.cpp
- ._on_add_joint
- Development Roadmap
- README.md
- ProjectController
- Technology Stack and Compatibility Plan
- .dh_dock
- MDHParams
- ._on_remove_joint
- test_task_runner_creation
- 2. Robot inputs
- CLAUDE.md
- continue-protocol.md
- .log_level
- .viewport_bg_color

## God Nodes (most connected - your core abstractions)
1. `MainWindow` - 65 edges
2. `DHEditorWidget` - 39 edges
3. `ProjectData` - 37 edges
4. `AppSettings` - 34 edges
5. `ProjectController` - 31 edges
6. `load()` - 25 edges
7. `RobotModel` - 24 edges
8. `TaskRunner` - 22 edges
9. `save()` - 20 edges
10. `ViewportWidget` - 19 edges

## Surprising Connections (you probably didn't know these)
- `R2.4 --- URM hardening (C++ core + bindings + tests)` --references--> `Joint`  [INFERRED]
  PROGRESS.md → core/urm/joint.hpp
- `R2.4 --- URM hardening (C++ core + bindings + tests)` --references--> `DHConvention`  [INFERRED]
  PROGRESS.md → core/urm/robot_model.hpp
- `R2.4 --- URM hardening (C++ core + bindings + tests)` --references--> `RobotModel`  [INFERRED]
  PROGRESS.md → core/urm/robot_model.hpp
- `R2.5 --- FK engine (DH + MDH), C++ + bindings` --references--> `pose_to_iso()`  [INFERRED]
  PROGRESS.md → core/kin/fk.cpp
- `R2.5 --- FK engine (DH + MDH), C++ + bindings` --references--> `MDHParams`  [INFERRED]
  PROGRESS.md → core/urm/joint.hpp

## Import Cycles
- None detected.

## Communities (72 total, 34 thin omitted)

### Community 0 - "Software Requirements Specification"
Cohesion: 0.15
Nodes (13): 10. Visualization and results, 11. GPU and performance, 12. Project and extensibility, 13. Acceptance criteria, 1. Product scope, 3. Unified Robot Model, 4. Physical properties, 5. Kinematics (+5 more)

### Community 1 - "ProjectController"
Cohesion: 0.14
Nodes (4): test_new_project_cancelled(), test_new_project_creates_project(), test_new_project_emits_project_opened(), test_new_project_empty_name_cancelled()

### Community 2 - "MainWindow"
Cohesion: 0.08
Nodes (3): MainWindow, test_main_window_has_project_controller(), test_main_window_title_updates_on_new_project()

### Community 3 - "Technical Architecture"
Cohesion: 0.18
Nodes (11): 10. Validation and testing, 1. Architectural principles, 2. High-level data flow, 3. Main subsystems, 4. Unified Robot Model, 5. Simulation state, 6. CPU/GPU execution, 7. CAD and URDF pipeline (+3 more)

### Community 4 - "test_bridge.py"
Cohesion: 0.20
Nodes (4): test_bridge_add(), test_bridge_add_type(), test_bridge_get_version(), test_bridge_import()

### Community 6 - "Development Workflow"
Cohesion: 0.17
Nodes (11): Agent Roles, Claude Model Tiering (token saving), Development Workflow, Error Substage Rule, Hardware Context (OMEN 16), Key Rules, Progress File, Stage Completion Gate (+3 more)

### Community 7 - "Agent Communication Style Rules"
Cohesion: 0.50
Nodes (3): Agent Communication Style Rules, Rule 1 — Claude Agents: Caveman English, Rule 2 — Gemini Agents: ASD-STE100 Simplified Technical English

### Community 8 - "Hardware Audit — R0.1"
Cohesion: 0.22
Nodes (8): Build Tools, Compiler, CUDA Toolkit, GPU, Hardware Audit — R0.1, Notes, Pinocchio Decision (R0.8), R0 Final Tool Versions

### Community 9 - "test_robot_in_project.py"
Cohesion: 0.17
Nodes (5): make_valid_3joint_robot(), test_convention_and_base_tool_edits_persist_save_load(), test_editing_in_editor_changes_controller_robot_and_dirty_flag(), test_save_load_round_trip_3_joint_robot(), test_validate_errors_shown()

### Community 12 - "TaskRunner"
Cohesion: 0.16
Nodes (6): TaskRunner, test_task_error_signal(), test_task_finished_signal(), test_task_progress_signal(), test_task_runner_function_task(), test_task_runner_submit_and_result()

### Community 13 - "ProjectData"
Cohesion: 0.08
Nodes (13): ProjectData, save(), test_open_project_loads_file(), test_load_round_trip(), test_project_data_new(), test_project_data_touch(), test_save_creates_file(), test_save_load_with_robot_model() (+5 more)

### Community 14 - "test_viewport_smoke.py"
Cohesion: 0.07
Nodes (6): ViewportWidget, test_pyvista_import(), test_pyvista_offscreen_sphere(), test_pyvista_sphere_mesh_properties(), test_viewport_widget_add_sphere(), test_viewport_widget_creation()

### Community 16 - ".set_model"
Cohesion: 0.15
Nodes (7): make_robot(), test_dh_editor_a_values_in_table(), test_dh_editor_edit_updates_model(), test_dh_editor_invalid_float_reverts(), test_dh_editor_joint_names_in_table(), test_dh_editor_model_changed_signal(), test_dh_editor_set_model_populates_table()

### Community 17 - ".add_recent_project"
Cohesion: 0.20
Nodes (3): test_settings_add_recent_project(), test_settings_recent_project_deduplication(), test_settings_recent_projects_empty()

### Community 21 - "robot_model.cpp"
Cohesion: 0.17
Nodes (12): compiletime_version(), get_version(), JointType, Fixed, Prismatic, Revolute, joint_type_from_str(), joint_type_to_str() (+4 more)

### Community 22 - "from_json"
Cohesion: 0.14
Nodes (17): default_limits(), from_json(), parse_pose(), Pose, rpy, xyz, Current, Decisions (+9 more)

### Community 24 - "LogPanel"
Cohesion: 0.13
Nodes (3): LogPanel, test_main_window_has_log_panel(), test_main_window_log_panel_accepts_messages()

### Community 26 - "test_urm_bridge.py"
Cohesion: 0.05
Nodes (24): test_fk_2r_planar(), test_fk_return_dict_structure(), test_fk_wrong_length_raises_value_error(), test_pose_to_iso(), test_joint_type_change_applies_default_limits(), test_default_limits_callable(), test_dh_convention_enum(), test_dh_params_readwrite() (+16 more)

### Community 27 - "test_logger.py"
Cohesion: 0.05
Nodes (12): AppLogger, QtLogHandler, temp_logger(), test_app_logger_creates_log_file(), test_app_logger_creation(), test_app_logger_writes_to_file(), test_logger_log_path_is_in_appdata(), test_logger_set_level() (+4 more)

### Community 28 - "test_task_runner.py"
Cohesion: 0.24
Nodes (4): WorkerSignals, SlowCancellableTask, test_task_cancellation(), test_worker_signals_creation()

### Community 29 - "test_project_controller.py"
Cohesion: 0.14
Nodes (3): _read_json(), _write_json(), test_controller_creation()

### Community 30 - "RobotModel"
Cohesion: 0.18
Nodes (11): bind_urm(), RobotModel, base, convention, RobotModel::dof(), joints, links, metadata (+3 more)

### Community 31 - "test_ui_smoke.py"
Cohesion: 0.10
Nodes (6): create_app(), test_main_window_creation(), test_main_window_has_menu_bar(), test_main_window_has_status_bar(), test_main_window_minimum_size(), test_pyside6_import()

### Community 33 - "fixture"
Cohesion: 0.15
Nodes (6): clean_settings(), controller(), mock_widget(), test_settings(), clean_settings(), settings()

### Community 34 - ".viewport"
Cohesion: 0.16
Nodes (6): test_main_window_has_viewport(), test_main_window_viewport_property(), test_viewport_add_sphere(), test_viewport_import_does_not_load_vtk_at_module_level(), test_viewport_plotter_accessible(), test_viewport_reset_camera()

### Community 36 - "Joint"
Cohesion: 0.12
Nodes (17): DHParams, a, alpha, d, theta, Joint, dh, limits (+9 more)

### Community 39 - "Link"
Cohesion: 0.18
Nodes (8): Inertia, com, inertia_tensor, mass, Link, geometry_file, inertia, name

### Community 41 - "fk.cpp"
Cohesion: 0.29
Nodes (9): compute_joint_transform(), dh_transform(), forward_kinematics(), mdh_transform(), pose_to_iso(), DHConvention, Modified, Standard (+1 more)

### Community 47 - "test_fk.cpp"
Cohesion: 0.05
Nodes (45): FKResult, frames, tcp, cuda_device_info(), gpu_vector_add(), main(), nearly_equal_f(), test_device_info() (+37 more)

### Community 48 - "test_project.py"
Cohesion: 0.08
Nodes (15): ProjectMetadata, SimulationSettings, _check_format_version(), load(), ProjectFormatError, ProjectSerializationError, test_load_file_not_found(), test_load_invalid_zip() (+7 more)

### Community 50 - "AddTask"
Cohesion: 0.29
Nodes (3): AddTask, test_task_cancel_before_start(), test_task_creation()

### Community 52 - "RobotMetadata"
Cohesion: 0.33
Nodes (5): RobotMetadata, author, description, name, version

### Community 54 - "core_bridge.cpp"
Cohesion: 0.20
Nodes (3): PYBIND11_MODULE(), bind_kin(), fk_result_to_dict()

### Community 58 - "Development Roadmap"
Cohesion: 0.18
Nodes (11): Development Roadmap, Development rules, R0 --- Feasibility and setup, R1 --- Desktop foundation, R2 --- Custom robot modelling, R3 --- Import, R4 --- Kinematics and dynamics, R5 --- GPU compute (+3 more)

### Community 59 - "README.md"
Cohesion: 0.22
Nodes (5): Core capabilities, First milestone, Fixed decisions, Robotics Engineering & Simulation Platform, Vision

### Community 61 - "Technology Stack and Compatibility Plan"
Cohesion: 0.29
Nodes (6): Compatibility gates, Dependency policy, GPU design notes, GPU target, Proposed stack, Technology Stack and Compatibility Plan

### Community 63 - "MDHParams"
Cohesion: 0.40
Nodes (5): MDHParams, a, alpha, d, theta

### Community 67 - "2. Robot inputs"
Cohesion: 0.50
Nodes (4): 2. Robot inputs, CAD, Custom builder, URDF

## Knowledge Gaps
- **44 isolated node(s):** `Fixed`, `Prismatic`, `Revolute`, `rpy`, `xyz` (+39 more)
  These have ≤1 connection - possible missing edges or undocumented components. (Counts symbols only; 488 node(s) total have ≤1 connection when file, concept and rationale nodes are included.)
- **34 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `MainWindow` connect `MainWindow` to `.viewport`, `main_window.py`, `test_robot_in_project.py`, `test_main_window_has_task_runner`, `DHEditorWidget`, `test_viewport_smoke.py`, `test_settings.py`, `ProjectData`, `test_task_runner.py`, `._on_project_opened`, `.closeEvent`, `._on_robot_changed`, `._setup_dock_panels`, `LogPanel`, `ProjectController`, `test_project_controller.py`, `.dh_dock`, `test_ui_smoke.py`?**
  _High betweenness centrality (0.203) - this node is a cross-community bridge._
- **Are the 3 inferred relationships involving `MainWindow` (e.g. with `DHEditorWidget` and `ProjectController`) actually correct?**
  _`MainWindow` has 3 INFERRED edges - model-reasoned connections that need verification._
- **What connects `Fixed`, `Prismatic`, `Revolute` to the rest of the system?**
  _44 weakly-connected nodes found - possible documentation gaps or missing edges._
- **Should `ProjectController` be split into smaller, more focused modules?**
  _Cohesion score 0.13970588235294118 - nodes in this community are weakly interconnected._
- **Why does `R2.6 --- Wire model + DH editor into app/project` connect `from_json` to `robot_model.cpp`, `ProjectController`, `ProjectData`, `RobotModel`?**
  _High betweenness centrality (0.188) - this node is a cross-community bridge._
- **Are the 18 inferred relationships involving `ProjectData` (e.g. with `MainWindow` and `ProjectController`) actually correct?**
  _`ProjectData` has 18 INFERRED edges - model-reasoned connections that need verification._
- **Should `MainWindow` be split into smaller, more focused modules?**
  _Cohesion score 0.08387096774193549 - nodes in this community are weakly interconnected._