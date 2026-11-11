# Graph Report - Analyzer Project  (2026-11-11)

## Corpus Check
- 69 files · ~39,903 words
- Verdict: corpus is large enough that graph structure adds value.
- Unclassified: 3 file(s) not represented in the graph (top: (none) 2, .ini 1)

## Summary
- 1187 nodes · 2209 edges · 101 communities (50 shown, 51 thin omitted)
- Extraction: 87% EXTRACTED · 13% INFERRED · 0% AMBIGUOUS · INFERRED: 293 edges (avg confidence: 0.87)
- Token cost: 0 input · 0 output

## Graph Freshness
- Built from commit: `91a53a0d`
- Run `git rev-parse HEAD` and compare to check if the graph is stale.
- Run `graphify update .` after code changes (no API cost).

## Community Hubs (Navigation)
- Software Requirements Specification
- .new_project
- test_ui_smoke.py
- Technical Architecture
- test_bridge.py
- Development Workflow
- Agent Communication Style Rules
- Hardware Audit — R0.1
- .set_model
- TaskRunner
- test_project.py
- test_viewport_smoke.py
- .set_model
- .update
- ._on_project_opened
- Task
- test_urm.cpp
- test_urm_bridge.py
- LogPanel
- .set_model
- core_bridge
- test_logger.py
- test_task_runner.py
- ._validate_and_notify
- RobotModel
- PropertiesPanel
- robot_model.cpp
- fixture
- .viewport
- .show_grid
- Joint
- test_joint_panel.py
- RobotView
- Link
- task_runner.py
- fk.cpp
- TaskRunner
- pytest
- test_viewport_robot.py
- load
- test_fk.cpp
- test_cuda.cpp
- .project_controller
- test_3r_planar_model_actors_and_polyline
- RobotMetadata
- URM Schema v2 Specification
- ViewportWidget
- test_milestone_r2.py
- project_data.py
- Development Roadmap
- README.md
- ProjectController
- Technology Stack and Compatibility Plan
- AppLogger
- kin_bindings.cpp
- JointPanel
- .get
- ProjectMetadata
- 2. Robot inputs
- CLAUDE.md
- continue-protocol.md
- .add_recent_project
- .viewport_bg_color
- test_settings.py
- _read_json
- .actors
- ProjectData
- ModelPanel
- DHParams
- test_robot_model_convention_readwrite
- test_set_q_wrong_length_raises_value_error
- ._setup_dock_panels
- test_robot_model_validate_new_rules
- test_pose_readwrite
- QtLogHandler
- Substage specs
- DHEditorWidget
- .save_project
- ._on_remove_joint
- .wire_to_panel
- .set_level
- ._on_add_joint

## God Nodes (most connected - your core abstractions)
1. `MainWindow` - 82 edges
2. `DHEditorWidget` - 40 edges
3. `RobotView` - 36 edges
4. `AppSettings` - 34 edges
5. `ProjectData` - 31 edges
6. `ProjectController` - 29 edges
7. `Joint` - 27 edges
8. `RobotModel` - 27 edges
9. `JointPanel` - 26 edges
10. `load()` - 25 edges

## Surprising Connections (you probably didn't know these)
- `R2.4 --- URM hardening (C++ core + bindings + tests)` --references--> `Joint`  [INFERRED]
  PROGRESS.md → core/urm/joint.hpp
- `R2.4 --- URM hardening (C++ core + bindings + tests)` --references--> `DHConvention`  [INFERRED]
  PROGRESS.md → core/urm/robot_model.hpp
- `R2.4 --- URM hardening (C++ core + bindings + tests)` --references--> `RobotModel`  [INFERRED]
  PROGRESS.md → core/urm/robot_model.hpp
- `R2.5 --- FK engine (DH + MDH), C++ + bindings` --references--> `pose_to_iso()`  [INFERRED]
  PROGRESS.md → core/kin/fk.cpp
- `R2.5 --- FK engine (DH + MDH), C++ + bindings` --references--> `RobotModel`  [INFERRED]
  PROGRESS.md → core/urm/robot_model.hpp

## Import Cycles
- None detected.

## Communities (101 total, 51 thin omitted)

### Community 0 - "Software Requirements Specification"
Cohesion: 0.15
Nodes (13): 10. Visualization and results, 11. GPU and performance, 12. Project and extensibility, 13. Acceptance criteria, 1. Product scope, 3. Unified Robot Model, 4. Physical properties, 5. Kinematics (+5 more)

### Community 1 - ".new_project"
Cohesion: 0.14
Nodes (5): test_mark_modified_emits_signal(), test_new_project_cancelled(), test_new_project_creates_project(), test_new_project_emits_project_opened(), test_new_project_empty_name_cancelled()

### Community 2 - "test_ui_smoke.py"
Cohesion: 0.10
Nodes (6): create_app(), test_main_window_creation(), test_main_window_has_menu_bar(), test_main_window_has_status_bar(), test_main_window_minimum_size(), test_pyside6_import()

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

### Community 12 - "TaskRunner"
Cohesion: 0.16
Nodes (5): ErrorTask, test_task_error_signal(), test_task_finished_signal(), test_task_progress_signal(), test_task_runner_submit_and_result()

### Community 13 - "test_project.py"
Cohesion: 0.12
Nodes (10): save(), test_open_project_loads_file(), test_load_round_trip(), test_project_data_new(), test_project_data_touch(), test_save_creates_file(), test_save_load_with_robot_model(), test_save_sets_file_path() (+2 more)

### Community 14 - "test_viewport_smoke.py"
Cohesion: 0.17
Nodes (5): test_pyvista_import(), test_pyvista_offscreen_sphere(), test_pyvista_sphere_mesh_properties(), test_viewport_widget_add_sphere(), test_viewport_widget_creation()

### Community 16 - ".set_model"
Cohesion: 0.13
Nodes (7): make_robot(), test_dh_editor_a_values_in_table(), test_dh_editor_edit_updates_model(), test_dh_editor_invalid_float_reverts(), test_dh_editor_joint_names_in_table(), test_dh_editor_model_changed_signal(), test_dh_editor_set_model_populates_table()

### Community 19 - "._on_project_opened"
Cohesion: 0.10
Nodes (3): MainWindow, test_dock_toggle_in_view_menu(), test_main_window_set_status()

### Community 21 - "test_urm.cpp"
Cohesion: 0.19
Nodes (17): check_throws_runtime_error(), main(), test_base_tool_poses(), test_branched_tree_validates_clean(), test_convention_and_legacy(), test_create_3dof(), test_default_limits(), test_disconnected_link_fails() (+9 more)

### Community 22 - "test_urm_bridge.py"
Cohesion: 0.07
Nodes (14): test_dh_convention_enum(), test_dh_params_readwrite(), test_from_json_bad_json_raises_runtime_error(), test_joint_creation_and_schema_v2_fields(), test_joint_kinematics_enum(), test_joint_type_enum(), test_link_creation(), test_pose_whole_list_assignment() (+6 more)

### Community 26 - "core_bridge"
Cohesion: 0.10
Nodes (10): test_fk_2r_planar(), test_fk_return_dict_structure(), test_fk_wrong_length_raises_value_error(), test_pose_to_iso(), test_default_limits_callable(), test_mimic_struct_and_joint_mimic(), test_robot_model_add_joints(), test_robot_model_base_tool_poses() (+2 more)

### Community 27 - "test_logger.py"
Cohesion: 0.16
Nodes (5): test_app_logger_creates_log_file(), test_app_logger_creation(), test_app_logger_writes_to_file(), test_logger_log_path_is_in_appdata(), test_qt_handler_formats_level()

### Community 28 - "test_task_runner.py"
Cohesion: 0.19
Nodes (5): AddTask, SlowCancellableTask, test_task_cancel_before_start(), test_task_cancellation(), test_task_creation()

### Community 30 - "RobotModel"
Cohesion: 0.18
Nodes (13): RobotModel, base, convention, joints, links, metadata, schema_version, tcp_link (+5 more)

### Community 32 - "robot_model.cpp"
Cohesion: 0.19
Nodes (15): JointType, Fixed, Prismatic, Revolute, Pose, rpy, xyz, default_limits() (+7 more)

### Community 33 - "fixture"
Cohesion: 0.15
Nodes (6): clean_settings(), controller(), mock_widget(), test_settings(), clean_settings(), settings()

### Community 34 - ".viewport"
Cohesion: 0.16
Nodes (6): test_main_window_has_viewport(), test_main_window_viewport_property(), test_viewport_add_sphere(), test_viewport_import_does_not_load_vtk_at_module_level(), test_viewport_plotter_accessible(), test_viewport_reset_camera()

### Community 36 - "Joint"
Cohesion: 0.09
Nodes (21): Joint, axis, child_link, dh, kinematics, limits, mdh, mimic (+13 more)

### Community 37 - "test_joint_panel.py"
Cohesion: 0.16
Nodes (7): rotation_to_rpy(), make_test_arm(), test_out_of_limit_q_shows_warning_and_does_not_modify_model(), test_rotation_to_rpy_round_trip_and_gimbal_lock(), test_row_counts_and_units_fixed_and_prismatic(), test_spinbox_and_slider_sync_and_model_q(), test_zero_all_and_reset()

### Community 39 - "Link"
Cohesion: 0.14
Nodes (12): Mimic, joint, multiplier, offset, Inertia, com, inertia_tensor, mass (+4 more)

### Community 40 - "task_runner.py"
Cohesion: 0.15
Nodes (3): FunctionTask, WorkerSignals, test_worker_signals_creation()

### Community 41 - "fk.cpp"
Cohesion: 0.26
Nodes (9): compute_joint_transform(), dh_transform(), forward_kinematics(), mdh_transform(), pose_to_iso(), DHConvention, Modified, Standard (+1 more)

### Community 43 - "TaskRunner"
Cohesion: 0.14
Nodes (4): TaskRunner, test_main_window_has_task_runner(), test_task_runner_creation(), test_task_runner_function_task()

### Community 44 - "pytest"
Cohesion: 0.18
Nodes (6): make_valid_3joint_robot(), test_convention_and_base_tool_edits_persist_save_load(), test_editing_in_editor_changes_controller_robot_and_dirty_flag(), test_joint_type_change_applies_default_limits(), test_old_file_without_robot_loads_empty(), test_save_load_round_trip_3_joint_robot()

### Community 45 - "test_viewport_robot.py"
Cohesion: 0.12
Nodes (7): make_3r_planar_robot(), test_camera_not_reset_on_robot_changed(), test_main_window_integration_and_fk_failure_path(), test_project_closed_clears_viewport(), test_scale_rebuild_when_link_length_changes(), test_update_in_place_new_q_moves_polyline(), test_view_robot_submenu_actions()

### Community 46 - "load"
Cohesion: 0.18
Nodes (7): _check_format_version(), load(), ProjectFormatError, ProjectSerializationError, test_load_file_not_found(), test_load_invalid_zip(), test_load_missing_metadata_raises_format_error()

### Community 47 - "test_fk.cpp"
Cohesion: 0.08
Nodes (28): FKResult, frames, tcp, compiletime_version(), get_version(), main(), matrix_nearly_equal(), nearly_equal() (+20 more)

### Community 48 - "test_cuda.cpp"
Cohesion: 0.20
Nodes (7): cuda_device_info(), gpu_vector_add(), main(), nearly_equal_f(), test_device_info(), test_vector_add_large(), test_vector_add_small()

### Community 49 - ".project_controller"
Cohesion: 0.14
Nodes (6): test_dh_editor_edits_rebuild_joint_panel(), test_q_edit_sets_dirty_flag(), test_slider_change_updates_robot_view_polyline(), test_slider_drag_does_not_rebuild_dh_table_widgets(), test_main_window_has_project_controller(), test_main_window_title_updates_on_new_project()

### Community 52 - "RobotMetadata"
Cohesion: 0.33
Nodes (5): RobotMetadata, author, description, name, version

### Community 54 - "URM Schema v2 Specification"
Cohesion: 0.20
Nodes (9): 1. Overview, 2. Key Changes & Schema Fields, 3. Degree of Freedom (DOF) and `q_index_map`, 4. Active Chain Definition, 5. Schema v1 Migration Rule, 6. JSON Examples, Example A: 2-Joint Serial DH Arm, Example B: 3-Link Branched Tree (OriginAxis) (+1 more)

### Community 56 - "test_milestone_r2.py"
Cohesion: 0.23
Nodes (5): independent_dh_modified(), independent_dh_standard(), independent_pose_to_matrix(), test_fk_200_random_q_vs_independent_numpy(), test_milestone_r2_end_to_end()

### Community 57 - "project_data.py"
Cohesion: 0.15
Nodes (4): SimulationSettings, test_metadata_to_from_dict(), test_simulation_settings_defaults(), test_simulation_settings_to_from_dict()

### Community 58 - "Development Roadmap"
Cohesion: 0.18
Nodes (11): Development Roadmap, Development rules, R0 --- Feasibility and setup, R1 --- Desktop foundation, R2 --- Custom robot modelling, R3 --- Import, R4 --- Kinematics and dynamics, R5 --- GPU compute (+3 more)

### Community 59 - "README.md"
Cohesion: 0.22
Nodes (5): Core capabilities, First milestone, Fixed decisions, Robotics Engineering & Simulation Platform, Vision

### Community 60 - "ProjectController"
Cohesion: 0.11
Nodes (3): ProjectController, test_bad_robot_json_error_not_crash(), test_robot_changed_emitted()

### Community 61 - "Technology Stack and Compatibility Plan"
Cohesion: 0.29
Nodes (6): Compatibility gates, Dependency policy, GPU design notes, GPU target, Proposed stack, Technology Stack and Compatibility Plan

### Community 63 - "kin_bindings.cpp"
Cohesion: 0.12
Nodes (10): PYBIND11_MODULE(), bind_kin(), fk_result_to_dict(), bind_urm(), MDHParams, a, alpha, d (+2 more)

### Community 66 - "ProjectMetadata"
Cohesion: 0.29
Nodes (3): ProjectMetadata, test_metadata_creation(), test_metadata_touch_updates_modified_at()

### Community 67 - "2. Robot inputs"
Cohesion: 0.50
Nodes (4): 2. Robot inputs, CAD, Custom builder, URDF

### Community 70 - ".add_recent_project"
Cohesion: 0.13
Nodes (4): AppSettings, test_main_window_has_settings(), test_settings_add_recent_project(), test_settings_recent_project_deduplication()

### Community 72 - "test_settings.py"
Cohesion: 0.12
Nodes (5): test_settings_creation(), test_settings_log_level_default(), test_settings_log_level_set(), test_settings_recent_projects_empty(), test_settings_show_coordinate_frames_default()

### Community 78 - "DHParams"
Cohesion: 0.40
Nodes (5): DHParams, a, alpha, d, theta

### Community 89 - "QtLogHandler"
Cohesion: 0.18
Nodes (3): QtLogHandler, test_qt_handler_creation(), test_qt_handler_emits_signal()

### Community 90 - "Substage specs"
Cohesion: 0.18
Nodes (10): Current, Decisions, Handoff protocol, Open review items, Progress and Handoff, R3.2 onward, R3 plan (outline), Risks (+2 more)

## Knowledge Gaps
- **56 isolated node(s):** `Modified`, `Standard`, `base`, `convention`, `joints` (+51 more)
  These have ≤1 connection - possible missing edges or undocumented components. (Counts symbols only; 593 node(s) total have ≤1 connection when file, concept and rationale nodes are included.)
- **51 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `MainWindow` connect `._on_project_opened` to `.new_project`, `test_ui_smoke.py`, `main_window.py`, `.set_model`, `test_task_runner.py`, `PropertiesPanel`, `.viewport`, `test_joint_panel.py`, `.__init__`, `TaskRunner`, `pytest`, `test_viewport_robot.py`, `.project_controller`, `.closeEvent`, `test_milestone_r2.py`, `ProjectController`, `.add_recent_project`, `.viewport_bg_color`, `test_settings.py`, `ProjectData`, `ModelPanel`, `._setup_dock_panels`, `.__init__`, `DHEditorWidget`, `.save_project`?**
  _High betweenness centrality (0.167) - this node is a cross-community bridge._
- **Are the 2 inferred relationships involving `MainWindow` (e.g. with `DHEditorWidget` and `ProjectController`) actually correct?**
  _`MainWindow` has 2 INFERRED edges - model-reasoned connections that need verification._
- **What connects `Modified`, `Standard`, `base` to the rest of the system?**
  _56 weakly-connected nodes found - possible documentation gaps or missing edges._
- **Should `.new_project` be split into smaller, more focused modules?**
  _Cohesion score 0.14285714285714285 - nodes in this community are weakly interconnected._
- **Why does `R2.6 --- Wire model + DH editor into app/project` connect `robot_model.cpp` to `Substage specs`, `ProjectData`, `ProjectController`, `RobotModel`?**
  _High betweenness centrality (0.129) - this node is a cross-community bridge._
- **Are the 3 inferred relationships involving `RobotView` (e.g. with `ViewportWidget` and `R2.7 --- Viewport: links, frames, TCP`) actually correct?**
  _`RobotView` has 3 INFERRED edges - model-reasoned connections that need verification._
- **Should `test_ui_smoke.py` be split into smaller, more focused modules?**
  _Cohesion score 0.09956709956709957 - nodes in this community are weakly interconnected._