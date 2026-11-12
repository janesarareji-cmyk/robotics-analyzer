# Graph Report - Analyzer Project  (2026-11-12)

## Corpus Check
- 247 files · ~206,220 words
- Verdict: corpus is large enough that graph structure adds value.
- Unclassified: 23 file(s) not represented in the graph (top: .urdf 16, (none) 2, .ini 1)

## Summary
- 4638 nodes · 11660 edges · 195 communities (136 shown, 59 thin omitted)
- Extraction: 90% EXTRACTED · 10% INFERRED · 0% AMBIGUOUS · INFERRED: 1163 edges (avg confidence: 0.88)
- Token cost: 0 input · 0 output

## Graph Freshness
- Built from commit: `019bf644`
- Run `git rev-parse HEAD` and compare to check if the graph is stale.
- Run `graphify update .` after code changes (no API cost).

## Community Hubs (Navigation)
- Software Requirements Specification
- DynModel
- IKPanel
- Technical Architecture
- test_compute_backend.py
- test_ui_kin_dyn.py
- Development Workflow
- Agent Communication Style Rules
- CollisionPanel
- import_urdf
- .project_controller
- ImportReport
- test_collision.py
- DHEditorWidget
- RobotView
- test_import_report.py
- urdf.py
- random_robot
- numpy
- test_milestone_r4.py
- PhysicalPropertiesPanel
- .robot
- test_milestone_r3.py
- TaskRunner
- test_logger.py
- serializer.py
- test_import_report.py
- GraphPanel
- parametrize
- test_viewport_in_main_window.py
- Recorder
- compiled_model.hpp
- test_workspace_analysis.py
- test_urdf_meshes.py
- test_performance.py
- test_milestone_r5.py
- test_compute_backend.py
- Slot
- workspace_kernel.cu
- compile
- test_compiled_arrays.py
- ProjectController
- CompiledArrays
- test_fk.cpp
- SimPanel
- MainWindow
- JointPanel
- test_planner.py
- test_dynamics.cpp
- dof_limits
- ViewportWidget
- test_compiled_arrays.cpp
- TimelinePanel
- Development Roadmap
- README.md
- test_export.py
- Technology Stack and Compatibility Plan
- core_bridge
- GhostPanel
- main_window.py
- PlanningPanel
- WorkspacePanel
- 2. Robot inputs
- CLAUDE.md
- Continue Protocol (Sonnet orchestrator + Builder subagents)
- math
- test_gpu_manager.py
- test_sim_panel.py
- NumField
- test_compiled.cpp
- test_cuda.cpp
- test_integrators.py
- WsKernelArgs
- test_milestone_r7.py
- ConfigStore
- WorkspaceResult
- Trajectory
- SimulationSettings
- SimulationSettings
- test_bridge.py
- test_viewport_robot.py
- AnalysisPanel
- TrajectoryPlayer
- test_ik_multi.py
- test_ui_workspace.py
- ._render
- RobotModel
- PropertiesPanel
- check_trajectory
- test_milestone_r6.py
- LinkVisual
- PerformancePanel
- .set_shown
- .update
- gpu_manager.cu
- benchmark.py
- GPU FK precision decision (R5.2)
- planar_2r
- CollisionConfig
- Slot
- gpu_manager.cuh
- _Space
- SimEngine
- test_physical_props.py
- JointProfile
- DevMem
- test_cartesian_traj.py
- Stage R2 specs (archive, R2.4 to R2.9)
- WorkspaceOptions
- backend.py
- test_ui_trajectory.py
- RobotModel
- gpu_bridge.cpp
- forward_kinematics
- sim/__init__.py
- ._setup_dock_panels
- profiles.py
- Task
- IkGpuIn
- graph_data
- Stage R5 spec pack: GPU compute
- IkGpuOut
- time
- robot_view.py
- DeviceInfo
- AppSettings
- Master Plan: Stages R3 to R8
- Stage R3 spec pack: Import
- ModelPanel
- IkParams
- gp
- 1. How work runs (stage-batch workflow)
- CompiledCache
- IkStatus
- spec_from_model
- importers/__init__.py
- ._on_project_opened
- test_ui_ghosts.py
- cmath
- test_milestone_r2_end_to_end
- Hardware Audit — R0.1
- _runs
- test_ui_graphs.py
- test_workspace.py
- test_viewport_smoke.py
- gpu_bridge.cpp
- Trajectory and playback (R6.4)
- .set_model
- AddTask
- Workspace analysis (R7.5)
- planner.py
- URM Schema v2 Specification
- GlobalQ
- env_id
- .viewport_bg_color

## God Nodes (most connected - your core abstractions)
1. `MainWindow` - 178 edges
2. `random_robot()` - 89 edges
3. `RobotView` - 79 edges
4. `AppSettings` - 72 edges
5. `SimPanel` - 68 edges
6. `dof_limits()` - 66 edges
7. `spec_from_model()` - 63 edges
8. `Trajectory` - 62 edges
9. `ProjectData` - 53 edges
10. `save()` - 50 edges

## Surprising Connections (you probably didn't know these)
- `Workflow in the app` --references--> `TaskRunner`  [INFERRED]
  docs/planning.md → app/task_runner.py
- `Decisions` --references--> `check_trajectory()`  [INFERRED]
  docs/planning.md → app/planning/checks.py
- `Playback (`TrajectoryPlayer`)` --references--> `TrajectoryPlayer`  [INFERRED]
  docs/trajectory.md → app/sim/playback.py
- `Decision` --references--> `CollisionWorld`  [INFERRED]
  docs/collision-decision.md → app/planning/collision.py
- `R5.7 --- R5 milestone and stage gate` --references--> `MainWindow`  [INFERRED]
  docs/specs/R5-spec.md → app/main_window.py

## Import Cycles
- None detected.

## Communities (195 total, 59 thin omitted)

### Community 0 - "Software Requirements Specification"
Cohesion: 0.15
Nodes (13): 10. Visualization and results, 11. GPU and performance, 12. Project and extensibility, 13. Acceptance criteria, 1. Product scope, 3. Unified Robot Model, 4. Physical properties, 5. Kinematics (+5 more)

### Community 1 - "DynModel"
Cohesion: 0.06
Nodes (67): SimState, bias_torque(), build_dyn_model(), check_size(), coriolis_torque(), crossf(), crossm(), dyn_frames() (+59 more)

### Community 3 - "Technical Architecture"
Cohesion: 0.18
Nodes (11): 10. Validation and testing, 1. Architectural principles, 2. High-level data flow, 3. Main subsystems, 4. Unified Robot Model, 5. Simulation state, 6. CPU/GPU execution, 7. CAD and URDF pipeline (+3 more)

### Community 4 - "test_compute_backend.py"
Cohesion: 0.09
Nodes (24): get_backend(), GpuBackend, FakeGb, _gpu_or_skip(), isolated_settings(), _q(), robot(), test_auto_falls_back_when_gpu_bridge_import_fails() (+16 more)

### Community 5 - "test_ui_kin_dyn.py"
Cohesion: 0.16
Nodes (29): current_q(), _expected_tau(), _fill_from_pose(), _load(), _pump(), _set_q(), _solve(), _tcp() (+21 more)

### Community 6 - "Development Workflow"
Cohesion: 0.15
Nodes (12): Agent Roles, Claude Model Tiering (token saving), Development Workflow, Error Substage Rule, Hardware Context (OMEN 16), Key Rules, Progress File, Stage-Batch Workflow (active from Stage R3) (+4 more)

### Community 7 - "Agent Communication Style Rules"
Cohesion: 0.50
Nodes (3): Agent Communication Style Rules, Rule 1 — Claude Agents: Caveman English, Rule 2 — Gemini Agents: ASD-STE100 Simplified Technical English

### Community 8 - "CollisionPanel"
Cohesion: 0.11
Nodes (5): CollisionPanel, _fmt(), _params_text(), test_collision_panel_edit_list(), test_collision_zero_pose_button()

### Community 9 - "import_urdf"
Cohesion: 0.15
Nodes (30): import_urdf(), _assert_canonical(), _codes(), _fx(), test_continuous_joint_limits_and_warning(), test_error_fixture_reports_code(), test_gripper_fk_both_fingers(), test_gripper_mimic_tree_structure() (+22 more)

### Community 12 - ".project_controller"
Cohesion: 0.08
Nodes (19): win(), win(), win(), win(), win(), win(), win(), clean_settings() (+11 more)

### Community 14 - "test_collision.py"
Cohesion: 0.09
Nodes (24): EnvShape, fit_mesh(), _mesh_points(), pose_matrix(), Shape, shape_distance(), shapes_from_geometry(), shapes_from_project() (+16 more)

### Community 16 - "DHEditorWidget"
Cohesion: 0.05
Nodes (13): DHEditorWidget, make_robot(), test_dh_editor_a_values_in_table(), test_dh_editor_add_joint(), test_dh_editor_creation(), test_dh_editor_edit_updates_model(), test_dh_editor_invalid_float_reverts(), test_dh_editor_joint_names_in_table() (+5 more)

### Community 18 - "test_import_report.py"
Cohesion: 0.12
Nodes (23): find_element(), _click_row(), _code_constants(), _doc_codes(), _import(), _row_severities(), test_dock_rows_per_fixture(), test_docs_code_table_matches_constants() (+15 more)

### Community 19 - "urdf.py"
Cohesion: 0.09
Nodes (24): ImportReport, _build(), _build_joint(), _Fatal, _joint_limits(), _make_shape(), _named_materials(), _num() (+16 more)

### Community 20 - "random_robot"
Cohesion: 0.09
Nodes (47): inertias_from_model(), pose(), random_inertias(), random_q(), random_robot(), make(), _about(), _link() (+39 more)

### Community 21 - "numpy"
Cohesion: 0.09
Nodes (32): _make_static(), set_kinematic_q(), _z(), sample_q(), test_base_frame_is_world_without_base(), test_empty_batch(), test_errors(), test_matches_fk_compiled_1000() (+24 more)

### Community 22 - "test_milestone_r4.py"
Cohesion: 0.12
Nodes (20): integrate_rk4_forward_dynamics(), write_q(), rotation_to_rpy(), _fill(), _gravity_setting(), _load(), _numpy_sigma(), _pose_err() (+12 more)

### Community 23 - "PhysicalPropertiesPanel"
Cohesion: 0.08
Nodes (3): _fmt(), PhysicalPropertiesPanel, Physical panel

### Community 25 - "test_milestone_r3.py"
Cohesion: 0.15
Nodes (13): _axis_angle(), _drive(), _floats(), _hom(), _import(), NumpyRobot, _q_now(), _rpy() (+5 more)

### Community 26 - "TaskRunner"
Cohesion: 0.12
Nodes (8): TaskRunner, test_task_cancellation(), test_task_error_signal(), test_task_finished_signal(), test_task_progress_signal(), test_task_runner_creation(), test_task_runner_function_task(), test_task_runner_submit_and_result()

### Community 27 - "test_logger.py"
Cohesion: 0.06
Nodes (12): AppLogger, QtLogHandler, temp_logger(), test_app_logger_creates_log_file(), test_app_logger_creation(), test_app_logger_writes_to_file(), test_logger_log_path_is_in_appdata(), test_logger_set_level() (+4 more)

### Community 28 - "serializer.py"
Cohesion: 0.04
Nodes (37): ProjectData, ProjectMetadata, _check_format_version(), load(), ProjectFormatError, ProjectSerializationError, _read_collision(), _read_configs() (+29 more)

### Community 29 - "test_import_report.py"
Cohesion: 0.12
Nodes (24): is_plain_chain(), _import(), _make_dh_arm(), _model_json(), test_branch_segments_move_with_q(), test_dh_editor_convention_combo_still_works_on_tree(), test_dh_editor_origin_axis_rows_read_only(), test_dh_editor_refuses_add_remove_on_tree() (+16 more)

### Community 30 - "GraphPanel"
Cohesion: 0.07
Nodes (3): PlotView, GraphPanel, test_no_robot_is_safe()

### Community 31 - "parametrize"
Cohesion: 0.17
Nodes (23): test_shapes_and_dtypes(), test_chunk_size_invalid(), _gpu_or_skip(), _gpu_run(), _robots(), _setup(), test_equals_per_item_solve_ik(), test_gpu_converged_items_fk_error() (+15 more)

### Community 32 - "test_viewport_in_main_window.py"
Cohesion: 0.17
Nodes (5): test_main_window_viewport_property(), test_viewport_add_sphere(), test_viewport_import_does_not_load_vtk_at_module_level(), test_viewport_plotter_accessible(), test_viewport_reset_camera()

### Community 33 - "Recorder"
Cohesion: 0.09
Nodes (23): capacity_for(), _quat_wxyz(), Recorder, RingBuffer, signal_layout(), _engine(), test_60s_1khz_6dof_under_cap(), test_advance_records_every_step() (+15 more)

### Community 35 - "compiled_model.hpp"
Cohesion: 0.06
Nodes (29): CompiledJoint, axis, child_link, joint_index, mult, offset, parent_link, post (+21 more)

### Community 36 - "test_workspace_analysis.py"
Cohesion: 0.06
Nodes (35): analyze_workspace(), _boundary_count(), _chunk_stats(), field(), _merge(), _opts(), sample_metrics(), slice_image() (+27 more)

### Community 37 - "test_urdf_meshes.py"
Cohesion: 0.23
Nodes (18): _codes(), _import(), _mesh_urdf(), test_clear_removes_visual_actors(), test_dae_mesh_warning_and_fallback(), test_import_package_mesh_catkin_tree(), test_import_package_mesh_with_search_roots_arg(), test_mesh_too_large_warning_and_not_drawn() (+10 more)

### Community 38 - "test_performance.py"
Cohesion: 0.11
Nodes (17): _gpu_bridge_or_reason(), _import_gpu_bridge(), clear_last_run(), get_last_run(), set_last_run(), _gpu_backend(), _load(), _model() (+9 more)

### Community 39 - "test_milestone_r5.py"
Cohesion: 0.26
Nodes (14): _tolerances(), _gpu_or_skip(), _load(), _pump(), test_item_3_subset_gpu_equals_cpu(), test_item_5a_cancel_mid_run_gpu(), test_item_5b_cpu_by_setting_matches_gpu_subset(), test_item_5c_fake_oom_falls_back_and_reports() (+6 more)

### Community 40 - "test_compute_backend.py"
Cohesion: 0.14
Nodes (7): Backend, CpuBackend, Python backends (`app/compute/backend.py`), test_cpu_fk_bad_precision_and_arrays(), test_cpu_fk_batch_matches_numpy_and_info(), test_cpu_workspace_info_keys(), test_benchmark_rejects_no_robot()

### Community 41 - "Slot"
Cohesion: 0.12
Nodes (15): Pool, d_itab, d_tab, init, slot, Slot, d_in, d_out (+7 more)

### Community 42 - "workspace_kernel.cu"
Cohesion: 0.07
Nodes (18): sincos_t(), tcp_walk(), Oom, what, sample_q_gpu(), workspace_gpu(), BitFlag, __device__ (+10 more)

### Community 43 - "compile"
Cohesion: 0.08
Nodes (34): active_chain(), compile(), effective_links(), is_legacy_serial(), resolve_tcp(), rx(), rz(), tx() (+26 more)

### Community 44 - "test_compiled_arrays.py"
Cohesion: 0.11
Nodes (14): dh_chain(), fk_from_arrays(), mimic_model(), _motion(), test_compiled_order_is_child_sorted(), test_contents_match_compiled_fk(), test_effective_links_legacy(), test_effective_links_normal_chain() (+6 more)

### Community 45 - "ProjectController"
Cohesion: 0.03
Nodes (19): ProjectController, R2.6 --- Wire model + DH editor into app/project, controller(), test_controller_creation(), test_main_window_has_project_controller(), test_mark_modified_emits_signal(), test_new_project_cancelled(), test_new_project_creates_project() (+11 more)

### Community 46 - "CompiledArrays"
Cohesion: 0.07
Nodes (33): check_size(), CompiledArrays, axis, base, child_link, joint_index, layout_version, mult (+25 more)

### Community 47 - "test_fk.cpp"
Cohesion: 0.17
Nodes (16): main(), test_1r(), test_2r_planar(), test_3d_known_answer_mdh(), test_3d_known_answer_standard_dh(), test_3r_planar_random(), test_base_and_tool(), test_dh_vs_mdh() (+8 more)

### Community 48 - "SimPanel"
Cohesion: 0.05
Nodes (7): SimPanel, Graph suite (R6.6), Recorded signals (R6.5), Panel, panel(), rig(), panel()

### Community 49 - "MainWindow"
Cohesion: 0.06
Nodes (11): MainWindow, win(), test_docks_exist_hidden_and_toggle(), test_main_window_graphs_dock_hidden_and_wired(), test_path_3d_goes_to_viewport_polyline(), win(), test_main_window_creation(), test_main_window_has_menu_bar() (+3 more)

### Community 50 - "JointPanel"
Cohesion: 0.05
Nodes (12): JointPanel, JointRow, make_test_arm(), test_dh_editor_edits_rebuild_joint_panel(), test_out_of_limit_q_shows_warning_and_does_not_modify_model(), test_q_edit_sets_dirty_flag(), test_row_counts_and_units_fixed_and_prismatic(), test_slider_change_updates_robot_view_polyline() (+4 more)

### Community 52 - "test_planner.py"
Cohesion: 0.14
Nodes (29): plan(), path_length(), planar_2r(), assert_valid(), centreline_hits_box(), _circle_edge_free(), _circle_free(), densify() (+21 more)

### Community 53 - "test_dynamics.cpp"
Cohesion: 0.12
Nodes (22): IKResult, converged, iterations, message, pos_error, q, rot_error, compiletime_version() (+14 more)

### Community 54 - "dof_limits"
Cohesion: 0.22
Nodes (18): _ik_inputs(), dof_limits(), _err(), planar_3r(), _run(), _target(), test_deterministic(), test_dof_limits_mimic_and_branch() (+10 more)

### Community 56 - "test_compiled_arrays.cpp"
Cohesion: 0.33
Nodes (12): check_round_trip(), main(), make_links(), max_diff(), oa(), test_bad_arrays_throw(), test_effective_links_legacy(), test_layout_values() (+4 more)

### Community 57 - "TimelinePanel"
Cohesion: 0.08
Nodes (8): _fmt(), TimelinePanel, test_panel_plan_and_send(), test_delete_move_capture(), test_deleting_all_rows_clears_trajectory(), test_model_n_dof_change_drops_trajectory(), test_table_add_from_current_q_and_trajectory_signal(), tl()

### Community 58 - "Development Roadmap"
Cohesion: 0.18
Nodes (11): Development Roadmap, Development rules, R0 --- Feasibility and setup, R1 --- Desktop foundation, R2 --- Custom robot modelling, R3 --- Import, R4 --- Kinematics and dynamics, R5 --- GPU compute (+3 more)

### Community 59 - "README.md"
Cohesion: 0.22
Nodes (5): Core capabilities, First milestone, Fixed decisions, Robotics Engineering & Simulation Platform, Vision

### Community 60 - "test_export.py"
Cohesion: 0.14
Nodes (21): arrays_from_series(), build_meta(), _check(), _col_labels(), export_csv(), export_npz(), _fmt(), git_commit() (+13 more)

### Community 61 - "Technology Stack and Compatibility Plan"
Cohesion: 0.29
Nodes (6): Compatibility gates, Dependency policy, GPU design notes, GPU target, Proposed stack, Technology Stack and Compatibility Plan

### Community 62 - "core_bridge"
Cohesion: 0.03
Nodes (59): target_iso(), ik_goals(), finite_difference_torque(), test_joint_power_equals_tau_times_qd(), test_joint_power_size_mismatch_raises(), test_fk_2r_planar(), test_fk_return_dict_structure(), test_fk_wrong_length_raises_value_error() (+51 more)

### Community 64 - "main_window.py"
Cohesion: 0.05
Nodes (13): compare_configs(), joint_is_prismatic(), _rotvec(), wrap_pi(), dof_joint_types(), singularity_at(), create_app(), joint_type_name() (+5 more)

### Community 65 - "PlanningPanel"
Cohesion: 0.07
Nodes (9): _fmt(), PlanningPanel, progress(), work(), TimedPath, FunctionTask, _pump_until(), test_panel_capture_named_and_model_change() (+1 more)

### Community 66 - "WorkspacePanel"
Cohesion: 0.08
Nodes (5): SliceView, WorkspacePanel, work(), progress(), work()

### Community 67 - "2. Robot inputs"
Cohesion: 0.50
Nodes (4): 2. Robot inputs, CAD, Custom builder, URDF

### Community 69 - "Continue Protocol (Sonnet orchestrator + Builder subagents)"
Cohesion: 0.40
Nodes (4): Continue Protocol (Sonnet orchestrator + Builder subagents), Legacy per-substage protocol (R0 to R2 only, do not use), Read order (every new session), Stage-batch loop

### Community 70 - "math"
Cohesion: 0.23
Nodes (15): scurve(), trapezoid(), _bounds(), _ends(), _ends_1d(), _scan(), test_quintic_closed_form(), test_scurve_boundaries_limits_and_phases() (+7 more)

### Community 71 - "test_gpu_manager.py"
Cohesion: 0.20
Nodes (7): _device_or_skip(), _gpu(), test_chunk_size_invalid_headroom(), test_device_info_is_rtx_5060_sm120(), test_fake_oom_hook_toggles(), test_gpu_bridge_imports_and_reports(), test_mem_info_free_le_total()

### Community 72 - "test_sim_panel.py"
Cohesion: 0.13
Nodes (20): check_mode(), _load(), _loaded(), _model(), _pump(), _settings(), test_buttons_follow_state(), test_engine_lifecycle_open_edit_close() (+12 more)

### Community 74 - "test_compiled.cpp"
Cohesion: 0.34
Nodes (16): Iso, legacy_model(), main(), make_links(), oa(), test_legacy_fallback(), test_mimic(), test_misc() (+8 more)

### Community 75 - "test_cuda.cpp"
Cohesion: 0.22
Nodes (7): cuda_device_info(), gpu_vector_add(), main(), nearly_equal_f(), test_device_info(), test_vector_add_large(), test_vector_add_small()

### Community 76 - "test_integrators.py"
Cohesion: 0.07
Nodes (40): RK45, _accel_pendulum(), _engine(), _err_at(), _pendulum(), _ref_accel(), _ref_solution(), _run_to() (+32 more)

### Community 77 - "WsKernelArgs"
Cohesion: 0.07
Nodes (27): ws_kernel(), WsKernelArgs, count, d_a, d_counts, d_div, d_itab, d_lo (+19 more)

### Community 78 - "test_milestone_r7.py"
Cohesion: 0.13
Nodes (14): _box_axes_overlap(), DenseCheck, _load(), _plan(), planned(), _play(), _pump(), _pump_until() (+6 more)

### Community 79 - "ConfigStore"
Cohesion: 0.14
Nodes (5): ConfigStore, test_store_basics_and_validation(), test_store_dict_round_trip(), test_validate_dof_drops_and_logs(), test_model_change_drops_configs()

### Community 80 - "WorkspaceResult"
Cohesion: 0.09
Nodes (12): ProgressFn, CancelToken, flag, parse_workspace_options(), RunControl, error, local, progress (+4 more)

### Community 81 - "Trajectory"
Cohesion: 0.10
Nodes (14): acceptable(), build(), Trajectory, Common rules (every substage), R7.1 --- Trajectory generation and constraint checks, R7.2 --- Collision geometry (feasibility first), R7.3 --- Planner feasibility (OMPL) --- STOP after this substage, R7.4 --- Path planning (+6 more)

### Community 82 - "SimulationSettings"
Cohesion: 0.31
Nodes (3): test_settings_round_trip_old_file(), test_simulation_settings_to_from_dict(), test_settings_gains_roundtrip()

### Community 84 - "SimulationSettings"
Cohesion: 0.12
Nodes (12): SimulationSettings, Core, Integrators (`integrators.py`), Modes (`mode.py`), Settings (`SimulationSettings`), Simulation (R6.1 - R6.3), State machine, run() (+4 more)

### Community 85 - "test_bridge.py"
Cohesion: 0.20
Nodes (4): test_bridge_add(), test_bridge_add_type(), test_bridge_get_version(), test_bridge_import()

### Community 86 - "test_viewport_robot.py"
Cohesion: 0.08
Nodes (7): R3.4 --- UI for tree and origin_axis joints (Python UI), make_3r_planar_robot(), test_3r_planar_model_actors_and_polyline(), test_empty_model_draws_base_and_tcp_only(), test_update_in_place_new_q_moves_polyline(), test_view_robot_submenu_actions(), test_visibility_toggles()

### Community 88 - "AnalysisPanel"
Cohesion: 0.07
Nodes (3): CompiledCache, AnalysisPanel, mass_not_set()

### Community 89 - "TrajectoryPlayer"
Cohesion: 0.12
Nodes (17): TrajectoryPlayer, _engine(), test_clamped_and_natural_ends(), test_cubic_c1_c2_continuity_at_knots(), test_cubic_matches_independent_polynomial_solve(), test_derivatives_match_finite_differences(), test_dynamic_feedback_removes_initial_offset(), test_dynamic_speed_scales_playback_time() (+9 more)

### Community 91 - "test_ik_multi.py"
Cohesion: 0.18
Nodes (16): fk_2r(), fk_3r(), ik_2r(), ik_3r(), wrap(), so3_log(), planar(), run() (+8 more)

### Community 92 - "test_ui_workspace.py"
Cohesion: 0.23
Nodes (11): _cpu_panel(), _load(), _pump(), test_cancel_stops_run(), test_clear_on_project_close(), test_clear_on_robot_change_and_stale_result_dropped(), test_error_is_reported_not_raised(), test_fallback_reason_shown() (+3 more)

### Community 94 - "RobotModel"
Cohesion: 0.05
Nodes (64): Pose, rpy, xyz, Inertia, com, inertia_tensor, mass, Link (+56 more)

### Community 95 - "PropertiesPanel"
Cohesion: 0.13
Nodes (3): _fmt(), PropertiesPanel, 2. Stage R3: Import   (BLOCKING review)

### Community 96 - "check_trajectory"
Cohesion: 0.23
Nodes (11): _arr(), check_trajectory(), _samples(), quintic(), _heavy_2r(), test_accel_too_high_flagged_with_dof_and_time(), test_bad_inputs(), test_feasible_profile_passes() (+3 more)

### Community 97 - "test_milestone_r6.py"
Cohesion: 0.26
Nodes (15): _export(), _install(), _kinematic_run(), _load(), _pump(), _pump_until(), _show_dock(), test_item_3_kinematic_playback() (+7 more)

### Community 98 - "LinkVisual"
Cohesion: 0.07
Nodes (27): fallback_cube(), _first_file(), LinkVisual, load_visual_mesh(), mesh_format_ok(), read_mesh_file(), resolve_mesh(), triangle_count() (+19 more)

### Community 99 - "PerformancePanel"
Cohesion: 0.12
Nodes (3): _fmt_rate(), PerformancePanel, work()

### Community 107 - "gpu_manager.cu"
Cohesion: 0.16
Nodes (14): check_alloc(), checked_malloc(), cuda_check(), err_text(), is_oom(), is_sticky(), mark_dead(), query_device() (+6 more)

### Community 108 - "benchmark.py"
Cohesion: 0.14
Nodes (13): estimate_reach(), _measure(), _row(), _run(), run_benchmark(), _time(), versions(), write_benchmark() (+5 more)

### Community 109 - "GPU FK precision decision (R5.2)"
Cohesion: 0.22
Nodes (8): fp32 test tolerances (R5 Decisions 12), GPU FK precision decision (R5.2), Hardware and software, Measurement, Notes, Observed errors (1e6 samples, `tests/python/test_gpu_fk.py`), Rule (R5 spec, R5.2 item 8), SASS check (fp32 kernel)

### Community 110 - "planar_2r"
Cohesion: 0.13
Nodes (12): CollisionWorld, env_id(), Consequences and limits, planar_shapes(), test_adjacent_links_excluded_by_default(), test_check_batch_matches_single_queries_and_speed(), test_disable_always_colliding_at_zero_pose(), test_environment_box_distance_and_hit() (+4 more)

### Community 111 - "CollisionConfig"
Cohesion: 0.23
Nodes (3): CollisionConfig, _key(), _cfg()

### Community 112 - "Slot"
Cohesion: 0.13
Nodes (12): Slot, cn, d_pts, e0, e1, h_pts, pt_n, st (+4 more)

### Community 113 - "gpu_manager.cuh"
Cohesion: 0.18
Nodes (9): CudaError, GpuState, dead, dead_reason, MemInfo, error, free_bytes, ok (+1 more)

### Community 114 - "_Space"
Cohesion: 0.10
Nodes (6): _goal_of(), _prm(), _rrt_connect(), _rrt_star(), _Space, _Tree

### Community 120 - "SimEngine"
Cohesion: 0.09
Nodes (5): SimClock, SimEngine, UI driving, test_item_4_energy_balance_free_motion(), test_clock_legal_transitions()

### Community 121 - "test_physical_props.py"
Cohesion: 0.09
Nodes (31): _check_density_mass(), _closed_mesh_inertial(), estimate_inertial(), _mass_properties(), _origin_matrix(), _pack(), primitive_inertial(), visual_inertial() (+23 more)

### Community 122 - "JointProfile"
Cohesion: 0.15
Nodes (5): JointProfile, Decisions, Planning (R7.1, R7.4, R7.7), Profiles (`profiles.py`), Workflow in the app

### Community 123 - "DevMem"
Cohesion: 0.17
Nodes (6): pool_free_buffers(), PYBIND11_MODULE(), DevMem, cur, m, peak

### Community 124 - "test_cartesian_traj.py"
Cohesion: 0.19
Nodes (9): arc(), CartesianPath, line(), rot_exp(), rot_log(), _s_values(), _skew(), _solve() (+1 more)

### Community 125 - "Stage R2 specs (archive, R2.4 to R2.9)"
Cohesion: 0.50
Nodes (3): R2.7 --- Viewport: links, frames, TCP, R2.8 --- Joint control panel + TCP pose readout, Stage R2 specs (archive, R2.4 to R2.9)

### Community 126 - "WorkspaceOptions"
Cohesion: 0.04
Nodes (60): fill_q_grid(), fill_q_random(), fma32(), philox4x32_10(), philox_mulhilo(), philox_u01(), bitset_to_voxels(), make_sampler() (+52 more)

### Community 127 - "backend.py"
Cohesion: 0.22
Nodes (8): _gpu_mem_info(), max_batch(), set_vram_budget_mb(), vram_budget_bytes(), vram_budget_mb(), test_budget_default_and_clamp(), test_max_batch_cpu_is_fixed(), test_max_batch_uses_live_free_memory()

### Community 128 - "test_ui_trajectory.py"
Cohesion: 0.14
Nodes (20): Waypoint, waypoint_problems(), test_validation_errors(), _load(), _loaded(), _model(), _pump(), _set() (+12 more)

### Community 129 - "RobotModel"
Cohesion: 0.04
Nodes (49): PYBIND11_MODULE(), bind_dyn(), to_vec3(), bind_kin(), compiled_arrays_to_dict(), fk_result_to_dict(), to_np(), bind_urm() (+41 more)

### Community 131 - "gpu_bridge.cpp"
Cohesion: 0.09
Nodes (23): FkPathTables, axis, mult, n, offset, post, pre, src_q (+15 more)

### Community 132 - "forward_kinematics"
Cohesion: 0.04
Nodes (43): CompiledModel, FKResult, frames, tcp, forward_kinematics(), Err, e, pos (+35 more)

### Community 133 - "sim/__init__.py"
Cohesion: 0.27
Nodes (3): euler_step(), make_integrator(), rk4_step()

### Community 137 - "profiles.py"
Cohesion: 0.18
Nodes (3): _limit(), _Quintic, _Scalar

### Community 141 - "Task"
Cohesion: 0.11
Nodes (5): Task, R5.3 --- Workspace sampling (C++/CUDA + bindings), ErrorTask, ProgressTask, SlowCancellableTask

### Community 142 - "IkGpuIn"
Cohesion: 0.18
Nodes (10): IkGpuIn, base, forced_chunk, N, n_threads, prm, q0, q0_broadcast (+2 more)

### Community 145 - "graph_data"
Cohesion: 0.12
Nodes (18): decimate_minmax(), graph_data(), ser(), _joint_unit(), _rpy_from_quat(), Series, _stride(), Code facts (checked 2026-11-12, after R6) (+10 more)

### Community 146 - "Stage R5 spec pack: GPU compute"
Cohesion: 0.18
Nodes (12): is_dead(), mem_info(), set_fake_oom(), R5.1 GPU verification (2026-11-11), Common rules (every substage), Decisions fixed for R5 (from the Opus consult, 2026-11-11; do not reopen), R5.1 --- GPU Compute Manager (C++ + bindings), R5.4 --- CPU batch IK, GPU batch IK (go / no-go) (+4 more)

### Community 147 - "IkGpuOut"
Cohesion: 0.20
Nodes (9): IkGpuOut, chunk, chunks, iters, kernel_ms, perr, q, rerr (+1 more)

### Community 148 - "time"
Cohesion: 0.10
Nodes (23): Consequence for R7.4, Planner decision (R7.3), PoC evidence (own RRT-Connect, 2-DOF, circle obstacle), Proof attempt (one try, time-boxed), _edge_free(), _extend(), _free(), _nearest() (+15 more)

### Community 150 - "DeviceInfo"
Cohesion: 0.20
Nodes (10): DeviceInfo, available, driver_version, error, free_bytes, name, runtime_version, sm_major (+2 more)

### Community 151 - "AppSettings"
Cohesion: 0.04
Nodes (14): MeshPathsDialog, AppSettings, R3.5 --- Meshes and assets (Python), test_settings_persist(), test_settings(), test_main_window_has_settings(), test_settings_generic_get_set(), test_settings_log_level_default() (+6 more)

### Community 152 - "Master Plan: Stages R3 to R8"
Cohesion: 0.20
Nodes (9): 10. Summary timeline (order only, no dates), 3. Stage R4: Kinematics and dynamics   (BLOCKING review), 4. Stage R5: GPU compute   (precision decision is a blocking point), 5. Stage R6: Simulation and graphs   (non-blocking review), 6. Stage R7: Planning and advanced analysis   (non-blocking review), 7. Stage R8: Extensibility and release   (non-blocking review, then final full review), 8. Opus decision points (rare, logged in PROGRESS.md "Decisions"), 9. Risks and mitigations (+1 more)

### Community 154 - "Stage R3 spec pack: Import"
Cohesion: 0.25
Nodes (4): R3.6 --- Import validation and error report (Python UI), R3 stage milestone and gate (Builder does this after R3.7), Stage R3 spec pack: Import, test_dock_toggle_in_view_menu()

### Community 156 - "IkParams"
Cohesion: 0.11
Nodes (18): chol_solve6(), ik_kernel(), ik_launch(), IkParams, cap, hi, lam0, lam_min (+10 more)

### Community 158 - "1. How work runs (stage-batch workflow)"
Cohesion: 0.25
Nodes (8): 1.1 Cycle for every stage, 1.2 Review cadence (decision, can be changed by the user), 1.3 Stage gate (all must be true before push), 1.4 Push rule, 1.5 Failure rules (unchanged, restated), 1.5a Role split in every stage, 1.6 Standing technical rules (every stage), 1. How work runs (stage-batch workflow)

### Community 159 - "CompiledCache"
Cohesion: 0.33
Nodes (4): check_torque_limits(), Analysis dock, IK and Analysis panels (R4.9), Inverse Kinematics dock

### Community 160 - "IkStatus"
Cohesion: 0.40
Nodes (5): IkStatus, IK_CONVERGED, IK_MAX_ITER, IK_NAN, IK_STALLED

### Community 166 - "spec_from_model"
Cohesion: 0.05
Nodes (51): arc_geometry(), _enum_name(), fk_numpy(), hom(), _mass_matrix_numpy(), numeric_jacobian(), numeric_jacobian4(), pair() (+43 more)

### Community 171 - "test_ui_ghosts.py"
Cohesion: 0.12
Nodes (28): load(), save(), test_project_roundtrip_acm_and_environment(), test_project_without_collision_has_no_file_and_old_file_loads(), _project(), test_comparison_rejects_wrong_length(), test_empty_configs_not_written_and_old_file_loads(), test_invalid_configs_file_ignored() (+20 more)

### Community 175 - "cmath"
Cohesion: 0.07
Nodes (22): all_finite(), ik_batch_cpu(), IkBatchResult, iterations, n, n_dof, pos_error, q (+14 more)

### Community 176 - "test_milestone_r2_end_to_end"
Cohesion: 0.11
Nodes (8): independent_dh_modified(), independent_dh_standard(), independent_pose_to_matrix(), test_fk_200_random_q_vs_independent_numpy(), test_milestone_r2_end_to_end(), test_main_window_has_log_panel(), test_main_window_log_panel_accepts_messages(), test_main_window_integration_and_fk_failure_path()

### Community 177 - "Hardware Audit — R0.1"
Cohesion: 0.22
Nodes (8): Build Tools, Compiler, CUDA Toolkit, GPU, Hardware Audit — R0.1, Notes, Pinocchio Decision (R0.8), R0 Final Tool Versions

### Community 179 - "test_ui_graphs.py"
Cohesion: 0.12
Nodes (12): button_states(), switch_mode(), resolve_gains(), test_gain_defaults_and_settings(), _pump(), _run(), test_all_kinds_draw_after_a_run(), test_clear_empties_plot_and_recorder() (+4 more)

### Community 180 - "test_workspace.py"
Cohesion: 0.09
Nodes (50): fk_test_robots(), annulus_voxel_count(), bits(), _cancel_case(), _check_annulus(), gb(), _gpu_or_skip(), limits() (+42 more)

### Community 181 - "test_viewport_smoke.py"
Cohesion: 0.17
Nodes (5): test_pyvista_import(), test_pyvista_offscreen_sphere(), test_pyvista_sphere_mesh_properties(), test_viewport_widget_add_sphere(), test_viewport_widget_creation()

### Community 182 - "gpu_bridge.cpp"
Cohesion: 0.04
Nodes (30): chunk_size(), default_threads(), parallel_for(), register_cancel_token(), active_path(), fk_batch_cpu(), FkBatchResult, n (+22 more)

### Community 184 - "Trajectory and playback (R6.4)"
Cohesion: 0.25
Nodes (7): Interpolation, Model, Playback (`TrajectoryPlayer`), Project format 1.2.0, Project format 1.3.0 (R7), Trajectory and playback (R6.4), UI

### Community 186 - ".set_model"
Cohesion: 0.40
Nodes (3): active_link_set(), add_link(), style()

### Community 188 - "AddTask"
Cohesion: 0.29
Nodes (3): AddTask, test_task_cancel_before_start(), test_task_creation()

### Community 189 - "Workspace analysis (R7.5)"
Cohesion: 0.29
Nodes (6): Method, Numbers (Ryzen 9 8940HX, one thread, Python loop over `cb.jacobian_tcp` + `cb.analyze_singularity`), Singularity regions, Views, Volume and error bound, Workspace analysis (R7.5)

### Community 192 - "planner.py"
Cohesion: 0.08
Nodes (18): _model_limits(), ViolationReport, _cancel_fn(), _Cancelled, _limits_arrays(), PlanResult, _Timeout, _segment_times() (+10 more)

### Community 194 - "URM Schema v2 Specification"
Cohesion: 0.20
Nodes (9): 1. Overview, 2. Key Changes & Schema Fields, 3. Degree of Freedom (DOF) and `q_index_map`, 4. Active Chain Definition, 5. Schema v1 Migration Rule, 6. JSON Examples, Example A: 2-Joint Serial DH Arm, Example B: 3-Link Branched Tree (OriginAxis) (+1 more)

### Community 195 - "GlobalQ"
Cohesion: 0.33
Nodes (5): GlobalQ, __forceinline__, i, ld, q

### Community 196 - "env_id"
Cohesion: 0.33
Nodes (5): Collision library decision (R7.2), Decision, Project format, Proof build (one attempt, time-boxed), Speed (R7.2, `check_batch`, CPU, Python loop)

## Knowledge Gaps
- **314 isolated node(s):** `coulomb`, `eps`, `viscous`, `rpy`, `xyz` (+309 more)
  These have ≤1 connection - possible missing edges or undocumented components. (Counts symbols only; 1776 node(s) total have ≤1 connection when file, concept and rationale nodes are included.)
- **59 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `MainWindow` connect `MainWindow` to `test_ui_trajectory.py`, `DynModel`, `.closeEvent`, `IKPanel`, `test_ui_kin_dyn.py`, `CollisionPanel`, `._setup_dock_panels`, `.project_controller`, `ImportReport`, `Stage R5 spec pack: GPU compute`, `test_import_report.py`, `test_milestone_r4.py`, `AppSettings`, `.robot`, `PhysicalPropertiesPanel`, `Stage R3 spec pack: Import`, `ModelPanel`, `serializer.py`, `test_milestone_r3.py`, `GraphPanel`, `test_import_report.py`, `test_viewport_in_main_window.py`, `test_workspace_analysis.py`, `test_urdf_meshes.py`, `test_performance.py`, `test_milestone_r5.py`, `._on_project_opened`, `._add_tool_dock`, `test_ui_ghosts.py`, `ProjectController`, `test_milestone_r2_end_to_end`, `SimPanel`, `JointPanel`, `test_ui_graphs.py`, `test_export.py`, `GhostPanel`, `main_window.py`, `test_sim_panel.py`, `.viewport_bg_color`, `test_milestone_r7.py`, `Trajectory`, `test_viewport_robot.py`, `AnalysisPanel`, `.log`, `test_ui_workspace.py`, `._render`, `PropertiesPanel`, `test_milestone_r6.py`, `PerformancePanel`, `.update`, `test_physical_props.py`?**
  _High betweenness centrality (0.145) - this node is a cross-community bridge._
- **Are the 22 inferred relationships involving `MainWindow` (e.g. with `CollisionPanel` and `GhostPanel`) actually correct?**
  _`MainWindow` has 22 INFERRED edges - model-reasoned connections that need verification._
- **What connects `coulomb`, `eps`, `viscous` to the rest of the system?**
  _314 weakly-connected nodes found - possible documentation gaps or missing edges._
- **Should `DynModel` be split into smaller, more focused modules?**
  _Cohesion score 0.056962025316455694 - nodes in this community are weakly interconnected._
- **Why does `RobotView` connect `RobotView` to `.update`, `robot_view.py`, `test_viewport_robot.py`, `ViewportWidget`, `Stage R2 specs (archive, R2.4 to R2.9)`, `._render`?**
  _High betweenness centrality (0.030) - this node is a cross-community bridge._
- **Are the 3 inferred relationships involving `random_robot()` (e.g. with `Code facts (checked 2026-11-11, after R4)` and `R5.2 --- Batch FK kernel + precision decision (C++/CUDA + bindings)`) actually correct?**
  _`random_robot()` has 3 INFERRED edges - model-reasoned connections that need verification._
- **Should `IKPanel` be split into smaller, more focused modules?**
  _Cohesion score 0.0707070707070707 - nodes in this community are weakly interconnected._