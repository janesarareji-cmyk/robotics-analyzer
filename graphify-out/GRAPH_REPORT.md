# Graph Report - Analyzer Project  (2026-11-11)

## Corpus Check
- 55 files · ~21,277 words
- Verdict: corpus is large enough that graph structure adds value.
- Unclassified: 3 file(s) not represented in the graph (top: (none) 2, .ini 1)

## Summary
- 845 nodes · 1391 edges · 75 communities (39 shown, 36 thin omitted)
- Extraction: 90% EXTRACTED · 10% INFERRED · 0% AMBIGUOUS · INFERRED: 145 edges (avg confidence: 0.87)
- Token cost: 0 input · 0 output

## Graph Freshness
- Built from commit: `3b1c04f1`
- Run `git rev-parse HEAD` and compare to check if the graph is stale.
- Run `graphify update .` after code changes (no API cost).

## Community Hubs (Navigation)
- Software Requirements Specification
- ProjectController
- .__init__
- Technical Architecture
- test_bridge.py
- main_window.py
- Development Workflow
- Agent Communication Style Rules
- Hardware Audit — R0.1
- test_cuda.cpp
- TaskRunner
- ProjectData
- test_viewport_smoke.py
- DHEditorWidget
- .add_recent_project
- ._on_project_opened
- Task
- RobotModel
- from_json
- ._setup_dock_panels
- LogPanel
- test_urm_bridge.py
- QtLogHandler
- test_task_runner.py
- test_project_controller.py
- ViewportWidget
- test_ui_smoke.py
- test_project.py
- fixture
- .viewport
- .show_grid
- Joint
- FunctionTask
- AppSettings
- Link
- test_task_error_signal
- .save_project
- test_main_window_has_task_runner
- serializer.py
- SimulationSettings
- test_urm.cpp
- ProjectMetadata
- task_runner.py
- test_logger.py
- RobotMetadata
- core_bridge.cpp
- MDHParams
- test_eigen.cpp
- AppLogger
- Development Roadmap
- README.md
- .project_controller
- Technology Stack and Compatibility Plan
- .wire_to_panel
- .set_level
- temp_logger
- test_mark_modified_emits_signal
- test_task_runner_creation
- 2. Robot inputs
- CLAUDE.md
- continue-protocol.md
- .log_level
- .viewport_bg_color
- string
- DHParams

## God Nodes (most connected - your core abstractions)
1. `MainWindow` - 55 edges
2. `AppSettings` - 31 edges
3. `ProjectData` - 25 edges
4. `DHEditorWidget` - 23 edges
5. `TaskRunner` - 22 edges
6. `RobotModel` - 21 edges
7. `ProjectController` - 21 edges
8. `ViewportWidget` - 20 edges
9. `load()` - 19 edges
10. `save()` - 17 edges

## Surprising Connections (you probably didn't know these)
- `R2.4 --- URM hardening (C++ core + bindings + tests)` --references--> `Joint`  [INFERRED]
  PROGRESS.md → core/urm/joint.hpp
- `R2.4 --- URM hardening (C++ core + bindings + tests)` --references--> `default_limits()`  [INFERRED]
  PROGRESS.md → core/urm/robot_model.cpp
- `R2.4 --- URM hardening (C++ core + bindings + tests)` --references--> `RobotModel`  [INFERRED]
  PROGRESS.md → core/urm/robot_model.hpp
- `Open review items` --references--> `from_json()`  [INFERRED]
  PROGRESS.md → core/urm/robot_model.cpp
- `Decisions` --references--> `from_json()`  [INFERRED]
  PROGRESS.md → core/urm/robot_model.cpp

## Import Cycles
- None detected.

## Communities (75 total, 36 thin omitted)

### Community 0 - "Software Requirements Specification"
Cohesion: 0.15
Nodes (13): 10. Visualization and results, 11. GPU and performance, 12. Project and extensibility, 13. Acceptance criteria, 1. Product scope, 3. Unified Robot Model, 4. Physical properties, 5. Kinematics (+5 more)

### Community 1 - "ProjectController"
Cohesion: 0.08
Nodes (5): ProjectController, test_new_project_cancelled(), test_new_project_creates_project(), test_new_project_emits_project_opened(), test_new_project_empty_name_cancelled()

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

### Community 9 - "test_cuda.cpp"
Cohesion: 0.22
Nodes (7): cuda_device_info(), gpu_vector_add(), main(), nearly_equal_f(), test_device_info(), test_vector_add_large(), test_vector_add_small()

### Community 12 - "TaskRunner"
Cohesion: 0.16
Nodes (6): TaskRunner, test_task_error_signal(), test_task_finished_signal(), test_task_progress_signal(), test_task_runner_function_task(), test_task_runner_submit_and_result()

### Community 13 - "ProjectData"
Cohesion: 0.12
Nodes (9): ProjectData, save(), test_open_project_loads_file(), test_project_data_new(), test_project_data_touch(), test_save_creates_file(), test_save_load_with_robot_model_placeholder(), test_save_sets_file_path() (+1 more)

### Community 14 - "test_viewport_smoke.py"
Cohesion: 0.11
Nodes (5): test_pyvista_import(), test_pyvista_offscreen_sphere(), test_pyvista_sphere_mesh_properties(), test_viewport_widget_add_sphere(), test_viewport_widget_creation()

### Community 16 - "DHEditorWidget"
Cohesion: 0.08
Nodes (12): DHEditorWidget, make_robot(), test_dh_editor_a_values_in_table(), test_dh_editor_add_joint(), test_dh_editor_creation(), test_dh_editor_edit_updates_model(), test_dh_editor_invalid_float_reverts(), test_dh_editor_joint_names_in_table() (+4 more)

### Community 17 - ".add_recent_project"
Cohesion: 0.20
Nodes (3): test_settings_add_recent_project(), test_settings_recent_project_deduplication(), test_settings_recent_projects_empty()

### Community 21 - "RobotModel"
Cohesion: 0.14
Nodes (13): joint_type_to_str(), RobotModel, base, convention, RobotModel::get_q(), joints, links, metadata (+5 more)

### Community 22 - "from_json"
Cohesion: 0.12
Nodes (19): DHConvention, Modified, Standard, from_json(), joint_type_from_str(), parse_pose(), Pose, rpy (+11 more)

### Community 24 - "LogPanel"
Cohesion: 0.13
Nodes (3): LogPanel, test_main_window_has_log_panel(), test_main_window_log_panel_accepts_messages()

### Community 26 - "test_urm_bridge.py"
Cohesion: 0.07
Nodes (19): test_default_limits_callable(), test_dh_convention_enum(), test_dh_params_readwrite(), test_from_json_bad_json_raises_runtime_error(), test_joint_creation(), test_joint_type_enum(), test_link_creation(), test_pose_readwrite() (+11 more)

### Community 27 - "QtLogHandler"
Cohesion: 0.20
Nodes (4): QtLogHandler, test_qt_handler_creation(), test_qt_handler_emits_signal(), test_qt_handler_formats_level()

### Community 28 - "test_task_runner.py"
Cohesion: 0.19
Nodes (5): AddTask, SlowCancellableTask, test_task_cancel_before_start(), test_task_cancellation(), test_task_creation()

### Community 31 - "test_ui_smoke.py"
Cohesion: 0.13
Nodes (6): create_app(), test_main_window_creation(), test_main_window_has_menu_bar(), test_main_window_has_status_bar(), test_main_window_minimum_size(), test_pyside6_import()

### Community 32 - "test_project.py"
Cohesion: 0.15
Nodes (10): _check_format_version(), load(), ProjectFormatError, ProjectSerializationError, test_load_file_not_found(), test_load_invalid_zip(), test_load_missing_metadata_raises_format_error(), test_load_round_trip() (+2 more)

### Community 33 - "fixture"
Cohesion: 0.15
Nodes (6): clean_settings(), controller(), mock_widget(), test_settings(), clean_settings(), settings()

### Community 34 - ".viewport"
Cohesion: 0.19
Nodes (5): test_main_window_viewport_property(), test_viewport_add_sphere(), test_viewport_import_does_not_load_vtk_at_module_level(), test_viewport_plotter_accessible(), test_viewport_reset_camera()

### Community 36 - "Joint"
Cohesion: 0.12
Nodes (17): Joint, dh, limits, mdh, name, q, type, JointLimits (+9 more)

### Community 38 - "AppSettings"
Cohesion: 0.14
Nodes (3): AppSettings, test_settings_generic_get_set(), test_settings_show_coordinate_frames_default()

### Community 39 - "Link"
Cohesion: 0.17
Nodes (8): Inertia, com, inertia_tensor, mass, Link, geometry_file, inertia, name

### Community 47 - "test_urm.cpp"
Cohesion: 0.24
Nodes (12): main(), test_base_tool_poses(), test_convention_and_legacy(), test_create_3dof(), test_default_limits(), test_fixed_joint_not_counted(), test_json_round_trip(), test_malformed_json_and_unknown_type() (+4 more)

### Community 48 - "ProjectMetadata"
Cohesion: 0.18
Nodes (4): ProjectMetadata, test_metadata_creation(), test_metadata_to_from_dict(), test_metadata_touch_updates_modified_at()

### Community 50 - "test_logger.py"
Cohesion: 0.18
Nodes (4): test_app_logger_creates_log_file(), test_app_logger_creation(), test_app_logger_writes_to_file(), test_logger_log_path_is_in_appdata()

### Community 52 - "RobotMetadata"
Cohesion: 0.33
Nodes (5): RobotMetadata, author, description, name, version

### Community 54 - "core_bridge.cpp"
Cohesion: 0.25
Nodes (3): PYBIND11_MODULE(), bind_urm(), RobotModel::dof()

### Community 55 - "MDHParams"
Cohesion: 0.40
Nodes (5): MDHParams, a, alpha, d, theta

### Community 56 - "test_eigen.cpp"
Cohesion: 0.36
Nodes (7): main(), matrix_nearly_equal(), nearly_equal(), test_identity(), test_rotation_z90(), test_transform_composition(), test_transform_inverse()

### Community 58 - "Development Roadmap"
Cohesion: 0.18
Nodes (11): Development Roadmap, Development rules, R0 --- Feasibility and setup, R1 --- Desktop foundation, R2 --- Custom robot modelling, R3 --- Import, R4 --- Kinematics and dynamics, R5 --- GPU compute (+3 more)

### Community 59 - "README.md"
Cohesion: 0.22
Nodes (5): Core capabilities, First milestone, Fixed decisions, Robotics Engineering & Simulation Platform, Vision

### Community 61 - "Technology Stack and Compatibility Plan"
Cohesion: 0.29
Nodes (6): Compatibility gates, Dependency policy, GPU design notes, GPU target, Proposed stack, Technology Stack and Compatibility Plan

### Community 67 - "2. Robot inputs"
Cohesion: 0.50
Nodes (4): 2. Robot inputs, CAD, Custom builder, URDF

### Community 72 - "string"
Cohesion: 0.43
Nodes (3): compiletime_version(), get_version(), main()

### Community 73 - "DHParams"
Cohesion: 0.40
Nodes (5): DHParams, a, alpha, d, theta

## Knowledge Gaps
- **42 isolated node(s):** `base`, `convention`, `joints`, `links`, `metadata` (+37 more)
  These have ≤1 connection - possible missing edges or undocumented components. (Counts symbols only; 444 node(s) total have ≤1 connection when file, concept and rationale nodes are included.)
- **36 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `MainWindow` connect `.__init__` to `ProjectController`, `.viewport`, `main_window.py`, `.save_project`, `test_main_window_has_task_runner`, `test_task_runner.py`, `._on_project_opened`, `.closeEvent`, `._setup_dock_panels`, `LogPanel`, `.project_controller`, `test_project_controller.py`, `ViewportWidget`, `test_ui_smoke.py`?**
  _High betweenness centrality (0.153) - this node is a cross-community bridge._
- **Are the 2 inferred relationships involving `AppSettings` (e.g. with `settings()` and `test_main_window_has_settings()`) actually correct?**
  _`AppSettings` has 2 INFERRED edges - model-reasoned connections that need verification._
- **What connects `base`, `convention`, `joints` to the rest of the system?**
  _42 weakly-connected nodes found - possible documentation gaps or missing edges._
- **Should `ProjectController` be split into smaller, more focused modules?**
  _Cohesion score 0.07956989247311828 - nodes in this community are weakly interconnected._
- **Why does `AppSettings` connect `AppSettings` to `ProjectController`, `.__init__`, `.show_grid`, `fixture`, `main_window.py`, `.log_level`, `.viewport_bg_color`, `.save_project`, `.add_recent_project`, `.closeEvent`, `test_project_controller.py`?**
  _High betweenness centrality (0.081) - this node is a cross-community bridge._
- **Are the 9 inferred relationships involving `ProjectData` (e.g. with `save()` and `test_load_round_trip()`) actually correct?**
  _`ProjectData` has 9 INFERRED edges - model-reasoned connections that need verification._
- **Should `.__init__` be split into smaller, more focused modules?**
  _Cohesion score 0.12857142857142856 - nodes in this community are weakly interconnected._