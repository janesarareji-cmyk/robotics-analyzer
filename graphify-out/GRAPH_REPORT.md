# Graph Report - Analyzer Project  (2026-11-11)

## Corpus Check
- 67 files · ~35,396 words
- Verdict: corpus is large enough that graph structure adds value.
- Unclassified: 3 file(s) not represented in the graph (top: (none) 2, .ini 1)

## Summary
- 1139 nodes · 2114 edges · 95 communities (49 shown, 46 thin omitted)
- Extraction: 87% EXTRACTED · 13% INFERRED · 0% AMBIGUOUS · INFERRED: 267 edges (avg confidence: 0.87)
- Token cost: 0 input · 0 output

## Graph Freshness
- Built from commit: `9b7c0a0b`
- Run `git rev-parse HEAD` and compare to check if the graph is stale.
- Run `graphify update .` after code changes (no API cost).

## Community Hubs (Navigation)
- Software Requirements Specification
- .new_project
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
- RobotView
- ._on_project_opened
- Task
- robot_model.cpp
- Progress and Handoff
- LogPanel
- ._on_project_opened
- test_urm_bridge.py
- test_logger.py
- test_task_runner.py
- ._on_add_joint
- RobotModel
- test_ui_smoke.py
- Any
- fixture
- .viewport
- .show_grid
- Joint
- AppSettings
- ._validate_and_notify
- Link
- task_runner.py
- fk.cpp
- .__init__
- TaskRunner
- test_viewport_robot.py
- project_data.py
- test_fk.cpp
- ProjectSerializationError
- .project_controller
- serializer.py
- RobotMetadata
- core_bridge.cpp
- ViewportWidget
- test_milestone_r2.py
- load
- Development Roadmap
- README.md
- ProjectController
- Technology Stack and Compatibility Plan
- .dh_dock
- MDHParams
- JointPanel
- test_settings.py
- test_fk_bridge.py
- 2. Robot inputs
- CLAUDE.md
- continue-protocol.md
- .add_recent_project
- .viewport_bg_color
- .mark_modified
- test_tcp_readout_2r_planar
- ._setup_dock_panels
- DHEditorWidget
- test_joint_panel.py
- .show_coordinate_frames
- .log_panel
- .set_model
- test_main_window_title_updates_on_new_project
- test_main_window_minimum_size
- test_view_robot_submenu_actions

## God Nodes (most connected - your core abstractions)
1. `MainWindow` - 82 edges
2. `DHEditorWidget` - 40 edges
3. `ProjectData` - 37 edges
4. `RobotView` - 36 edges
5. `AppSettings` - 34 edges
6. `ProjectController` - 29 edges
7. `JointPanel` - 26 edges
8. `load()` - 25 edges
9. `RobotModel` - 24 edges
10. `TaskRunner` - 22 edges

## Surprising Connections (you probably didn't know these)
- `Decisions` --references--> `from_json()`  [INFERRED]
  PROGRESS.md → core/urm/robot_model.cpp
- `R2.4 --- URM hardening (C++ core + bindings + tests)` --references--> `Joint`  [INFERRED]
  PROGRESS.md → core/urm/joint.hpp
- `R2.4 --- URM hardening (C++ core + bindings + tests)` --references--> `DHConvention`  [INFERRED]
  PROGRESS.md → core/urm/robot_model.hpp
- `R2.4 --- URM hardening (C++ core + bindings + tests)` --references--> `RobotModel`  [INFERRED]
  PROGRESS.md → core/urm/robot_model.hpp
- `R2.5 --- FK engine (DH + MDH), C++ + bindings` --references--> `pose_to_iso()`  [INFERRED]
  PROGRESS.md → core/kin/fk.cpp

## Import Cycles
- None detected.

## Communities (95 total, 46 thin omitted)

### Community 0 - "Software Requirements Specification"
Cohesion: 0.15
Nodes (13): 10. Visualization and results, 11. GPU and performance, 12. Project and extensibility, 13. Acceptance criteria, 1. Product scope, 3. Unified Robot Model, 4. Physical properties, 5. Kinematics (+5 more)

### Community 1 - ".new_project"
Cohesion: 0.13
Nodes (6): test_new_project_cancelled(), test_new_project_creates_project(), test_new_project_emits_project_opened(), test_new_project_empty_name_cancelled(), test_save_project_emits_project_saved(), test_save_project_saves_to_file()

### Community 2 - "MainWindow"
Cohesion: 0.08
Nodes (6): MainWindow, test_main_window_has_project_controller(), test_main_window_creation(), test_main_window_has_menu_bar(), test_main_window_has_status_bar(), test_main_window_has_viewport()

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
Cohesion: 0.16
Nodes (7): make_valid_3joint_robot(), test_bad_robot_json_error_not_crash(), test_convention_and_base_tool_edits_persist_save_load(), test_editing_in_editor_changes_controller_robot_and_dirty_flag(), test_joint_type_change_applies_default_limits(), test_old_file_without_robot_loads_empty(), test_save_load_round_trip_3_joint_robot()

### Community 12 - "TaskRunner"
Cohesion: 0.16
Nodes (5): ErrorTask, test_task_error_signal(), test_task_finished_signal(), test_task_progress_signal(), test_task_runner_submit_and_result()

### Community 13 - "ProjectData"
Cohesion: 0.11
Nodes (10): ProjectData, save(), test_load_round_trip(), test_project_data_new(), test_project_data_touch(), test_save_creates_file(), test_save_load_with_robot_model(), test_save_sets_file_path() (+2 more)

### Community 14 - "test_viewport_smoke.py"
Cohesion: 0.17
Nodes (5): test_pyvista_import(), test_pyvista_offscreen_sphere(), test_pyvista_sphere_mesh_properties(), test_viewport_widget_add_sphere(), test_viewport_widget_creation()

### Community 16 - ".set_model"
Cohesion: 0.16
Nodes (8): make_robot(), test_dh_editor_a_values_in_table(), test_dh_editor_edit_updates_model(), test_dh_editor_invalid_float_reverts(), test_dh_editor_joint_names_in_table(), test_dh_editor_model_changed_signal(), test_dh_editor_remove_joint(), test_dh_editor_set_model_populates_table()

### Community 17 - "RobotView"
Cohesion: 0.12
Nodes (4): RobotView, test_3r_planar_model_actors_and_polyline(), test_empty_model_draws_base_and_tcp_only(), test_visibility_toggles()

### Community 21 - "robot_model.cpp"
Cohesion: 0.18
Nodes (14): compiletime_version(), get_version(), JointType, Fixed, Prismatic, Revolute, default_limits(), from_json() (+6 more)

### Community 22 - "Progress and Handoff"
Cohesion: 0.25
Nodes (7): Current, Decisions, Handoff protocol, Open review items, Progress and Handoff, Risks, Substage log

### Community 26 - "test_urm_bridge.py"
Cohesion: 0.07
Nodes (19): test_default_limits_callable(), test_dh_convention_enum(), test_dh_params_readwrite(), test_from_json_bad_json_raises_runtime_error(), test_joint_creation(), test_joint_type_enum(), test_link_creation(), test_pose_readwrite() (+11 more)

### Community 27 - "test_logger.py"
Cohesion: 0.05
Nodes (12): AppLogger, QtLogHandler, temp_logger(), test_app_logger_creates_log_file(), test_app_logger_creation(), test_app_logger_writes_to_file(), test_logger_log_path_is_in_appdata(), test_logger_set_level() (+4 more)

### Community 28 - "test_task_runner.py"
Cohesion: 0.19
Nodes (5): AddTask, SlowCancellableTask, test_task_cancel_before_start(), test_task_cancellation(), test_task_creation()

### Community 30 - "RobotModel"
Cohesion: 0.15
Nodes (14): joint_type_to_str(), RobotModel, base, convention, RobotModel::get_q(), joints, links, metadata (+6 more)

### Community 32 - "Any"
Cohesion: 0.12
Nodes (4): R2.7 --- Viewport: links, frames, TCP, R2.8 --- Joint control panel + TCP pose readout, R3 onward, Substage specs

### Community 33 - "fixture"
Cohesion: 0.19
Nodes (5): clean_settings(), controller(), mock_widget(), test_controller_creation(), test_settings()

### Community 34 - ".viewport"
Cohesion: 0.17
Nodes (5): test_main_window_viewport_property(), test_viewport_add_sphere(), test_viewport_import_does_not_load_vtk_at_module_level(), test_viewport_plotter_accessible(), test_viewport_reset_camera()

### Community 36 - "Joint"
Cohesion: 0.12
Nodes (17): DHParams, a, alpha, d, theta, Joint, dh, limits (+9 more)

### Community 37 - "AppSettings"
Cohesion: 0.15
Nodes (3): AppSettings, test_main_window_has_settings(), test_settings_generic_get_set()

### Community 39 - "Link"
Cohesion: 0.18
Nodes (8): Inertia, com, inertia_tensor, mass, Link, geometry_file, inertia, name

### Community 40 - "task_runner.py"
Cohesion: 0.15
Nodes (3): FunctionTask, WorkerSignals, test_worker_signals_creation()

### Community 41 - "fk.cpp"
Cohesion: 0.29
Nodes (9): compute_joint_transform(), dh_transform(), forward_kinematics(), mdh_transform(), pose_to_iso(), DHConvention, Modified, Standard (+1 more)

### Community 43 - "TaskRunner"
Cohesion: 0.14
Nodes (4): TaskRunner, test_main_window_has_task_runner(), test_task_runner_creation(), test_task_runner_function_task()

### Community 45 - "test_viewport_robot.py"
Cohesion: 0.14
Nodes (6): make_3r_planar_robot(), test_camera_not_reset_on_robot_changed(), test_main_window_integration_and_fk_failure_path(), test_project_closed_clears_viewport(), test_scale_rebuild_when_link_length_changes(), test_update_in_place_new_q_moves_polyline()

### Community 46 - "project_data.py"
Cohesion: 0.20
Nodes (4): ProjectMetadata, test_metadata_creation(), test_metadata_to_from_dict(), test_metadata_touch_updates_modified_at()

### Community 47 - "test_fk.cpp"
Cohesion: 0.05
Nodes (45): FKResult, frames, tcp, cuda_device_info(), gpu_vector_add(), main(), nearly_equal_f(), test_device_info() (+37 more)

### Community 48 - "ProjectSerializationError"
Cohesion: 0.29
Nodes (3): ProjectSerializationError, test_load_file_not_found(), test_load_invalid_zip()

### Community 49 - ".project_controller"
Cohesion: 0.17
Nodes (5): make_test_arm(), test_dh_editor_edits_rebuild_joint_panel(), test_q_edit_sets_dirty_flag(), test_row_counts_and_units_fixed_and_prismatic(), test_slider_drag_does_not_rebuild_dh_table_widgets()

### Community 50 - "serializer.py"
Cohesion: 0.23
Nodes (4): _check_format_version(), ProjectFormatError, _read_json(), _write_json()

### Community 52 - "RobotMetadata"
Cohesion: 0.33
Nodes (5): RobotMetadata, author, description, name, version

### Community 54 - "core_bridge.cpp"
Cohesion: 0.16
Nodes (6): PYBIND11_MODULE(), bind_kin(), fk_result_to_dict(), bind_urm(), RobotModel::dof(), R2.5 --- FK engine (DH + MDH), C++ + bindings

### Community 56 - "test_milestone_r2.py"
Cohesion: 0.21
Nodes (6): test_slider_change_updates_robot_view_polyline(), independent_dh_modified(), independent_dh_standard(), independent_pose_to_matrix(), test_fk_200_random_q_vs_independent_numpy(), test_milestone_r2_end_to_end()

### Community 57 - "load"
Cohesion: 0.15
Nodes (5): SimulationSettings, load(), test_load_missing_metadata_raises_format_error(), test_simulation_settings_defaults(), test_simulation_settings_to_from_dict()

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

### Community 64 - "JointPanel"
Cohesion: 0.16
Nodes (3): JointPanel, test_out_of_limit_q_shows_warning_and_does_not_modify_model(), test_zero_all_and_reset()

### Community 65 - "test_settings.py"
Cohesion: 0.13
Nodes (5): clean_settings(), settings(), test_settings_creation(), test_settings_log_level_default(), test_settings_log_level_set()

### Community 66 - "test_fk_bridge.py"
Cohesion: 0.20
Nodes (4): test_fk_2r_planar(), test_fk_return_dict_structure(), test_fk_wrong_length_raises_value_error(), test_pose_to_iso()

### Community 67 - "2. Robot inputs"
Cohesion: 0.50
Nodes (4): 2. Robot inputs, CAD, Custom builder, URDF

### Community 70 - ".add_recent_project"
Cohesion: 0.20
Nodes (3): test_settings_add_recent_project(), test_settings_recent_project_deduplication(), test_settings_recent_projects_empty()

### Community 80 - "test_joint_panel.py"
Cohesion: 0.21
Nodes (3): rotation_to_rpy(), test_rotation_to_rpy_round_trip_and_gimbal_lock(), test_spinbox_and_slider_sync_and_model_q()

## Knowledge Gaps
- **44 isolated node(s):** `Fixed`, `Prismatic`, `Revolute`, `rpy`, `xyz` (+39 more)
  These have ≤1 connection - possible missing edges or undocumented components. (Counts symbols only; 564 node(s) total have ≤1 connection when file, concept and rationale nodes are included.)
- **46 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `MainWindow` connect `MainWindow` to `main_window.py`, `test_robot_in_project.py`, `._on_project_opened`, `._on_project_opened`, `test_task_runner.py`, `test_ui_smoke.py`, `fixture`, `.viewport`, `AppSettings`, `.__init__`, `TaskRunner`, `test_viewport_robot.py`, `.project_controller`, `.closeEvent`, `test_milestone_r2.py`, `ProjectController`, `.dh_dock`, `test_settings.py`, `.viewport_bg_color`, `._setup_dock_panels`, `DHEditorWidget`, `test_joint_panel.py`, `.log_panel`, `._on_robot_changed`, `._on_new_project`, `._on_open_project`, `._on_reset_camera`, `._setup_toolbar`, `test_main_window_title_updates_on_new_project`, `test_main_window_minimum_size`, `test_view_robot_submenu_actions`?**
  _High betweenness centrality (0.210) - this node is a cross-community bridge._
- **Are the 2 inferred relationships involving `MainWindow` (e.g. with `DHEditorWidget` and `ProjectController`) actually correct?**
  _`MainWindow` has 2 INFERRED edges - model-reasoned connections that need verification._
- **What connects `Fixed`, `Prismatic`, `Revolute` to the rest of the system?**
  _44 weakly-connected nodes found - possible documentation gaps or missing edges._
- **Should `.new_project` be split into smaller, more focused modules?**
  _Cohesion score 0.13333333333333333 - nodes in this community are weakly interconnected._
- **Why does `R2.6 --- Wire model + DH editor into app/project` connect `RobotModel` to `Any`, `robot_model.cpp`, `ProjectController`, `ProjectData`?**
  _High betweenness centrality (0.124) - this node is a cross-community bridge._
- **Are the 16 inferred relationships involving `ProjectData` (e.g. with `save()` and `R2.6 --- Wire model + DH editor into app/project`) actually correct?**
  _`ProjectData` has 16 INFERRED edges - model-reasoned connections that need verification._
- **Should `MainWindow` be split into smaller, more focused modules?**
  _Cohesion score 0.07692307692307693 - nodes in this community are weakly interconnected._