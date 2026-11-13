# Graph Report - Analyzer Project  (2026-11-13)

## Corpus Check
- 346 files · ~293,251 words
- Verdict: corpus is large enough that graph structure adds value.
- Unclassified: 165 file(s) not represented in the graph (top: .stl 108, .urdf 26, .robotproj 16)
- WARNING: 165 of 528 files seen (31%) have no supported extractor and are missing from this graph entirely. It may look complete while representing only part of the codebase.

## Summary
- 6496 nodes · 16899 edges · 262 communities (179 shown, 83 thin omitted)
- Extraction: 90% EXTRACTED · 10% INFERRED · 0% AMBIGUOUS · INFERRED: 1749 edges (avg confidence: 0.89)
- Token cost: 0 input · 0 output

## Graph Freshness
- Built from commit: `c858d6e9`
- Run `git rev-parse HEAD` and compare to check if the graph is stale.
- Run `graphify update .` after code changes (no API cost).

## Community Hubs (Navigation)
- Software Requirements Specification
- DynModel
- IKPanel
- Technical Architecture
- ._update_robot_view
- test_ui_kin_dyn.py
- Development Workflow
- Agent Communication Style Rules
- CollisionPanel
- import_urdf
- test_recovery.py
- CompiledArrays
- dof_limits
- DHEditorWidget
- AppSettings
- RobotView
- urdf.py
- random_robot
- test_milestone_a1.py
- TcpDragController
- PhysicalPropertiesPanel
- math
- test_milestone_r3.py
- TaskRunner
- test_sim_panel.py
- PerformancePanel
- unittest_mock
- GraphPanel
- robots.py
- numpy
- CollisionWorld
- test_viewport_robot.py
- Current
- ._render
- test_urdf_meshes.py
- test_performance.py
- urdf_export.py
- RobotModel
- kinds.py
- compile
- core_bridge
- kin_bindings.cpp
- CompiledJoint
- test_fk.cpp
- SimPanel
- subprocess
- JointPanel
- _prm_cancel_latency
- planar_2r
- test_dynamics.cpp
- ComparePanel
- ViewportWidget
- math
- test_ui_trajectory.py
- Development Roadmap
- README.md
- test_export.py
- Technology Stack and Compatibility Plan
- Robot
- GhostPanel
- main_window.py
- PlanningPanel
- WorkspacePanel
- 2. Robot inputs
- CLAUDE.md
- Continue Protocol (Sonnet orchestrator + Builder subagents)
- test_import_report.py
- .restore_window
- test_milestone_r8.py
- User guide
- test_compiled.cpp
- Workspace analysis (R7.5)
- test_integrators.py
- WsKernelArgs
- test_logger.py
- ConfigStore
- test_plugins.py
- gpu_bridge.cpp
- test_milestone_r4.py
- IkBatchResult
- workspace_kernel.cu
- test_bridge.py
- test_milestone_r5.py
- AnalysisPanel
- test_ui_trajectory.py
- test_compiled_arrays.cpp
- Any
- test_ui_workspace.py
- compare.py
- PluginManager
- motion.py
- ._setup_central_widget
- test_milestone_r6.py
- graph_data
- Stage A1 spec pack: Audit fixes
- test_sim_panel.py
- test_gpu_manager.py
- re
- save
- test_gripper.py
- budget.py
- RoboticsApiError
- GPU FK precision decision (R5.2)
- test_collision.py
- Rigid-body dynamics (R4.6 to R4.8)
- test_exit_clean.py
- Trajectory
- _Space
- current_q
- U2 spec pack: usability fixes, interaction features, validation gaps
- test_compute_backend.py
- test_report.py
- bench_fk.py
- ModelPanel
- test_physical_props.py
- ImportReportPanel
- test_script_runner.py
- MeshPathsDialog
- Slot
- test_u2_fix.py
- gpu_manager.cu
- PluginHost
- test_urdf_export.py
- GPU kernels (R5)
- WorkspaceResult
- 2. First project
- ._setup_dock_panels
- FkPathTables
- stdexcept
- report.py
- linear.py
- isolated_settings
- Simulation (R6.1 - R6.3)
- SimEngine
- .log
- test_workspace_analysis.py
- test_cuda.cpp
- test_logger.py
- test_teach.py
- test_gpu_errors.py
- validate_dynamics.py
- store.py
- test_docs.py
- Trajectory
- Project
- Stage R8 spec pack: Extensibility and release
- test_milestone_r8.py
- .wire_to_panel
- QDockWidget
- index.md
- CollisionConfig
- Developer notes
- SyncRunner
- Slot
- Stage R7 spec pack: Planning and advanced analysis
- export_csv
- test_ui_ghosts.py
- test_workspace.py
- string
- Trajectory and playback (R6.4)
- test_recorder.py
- test_ui_script.py
- Packaging (R8.5)
- test_milestone_r2_end_to_end
- check_docs.py
- test_viewport_in_main_window.py
- ZipFile
- ProjectData
- test_audit_numerics.py
- Master Plan: Stages R3 to R8
- GPU batch IK: go / no-go (R5.4)
- test_urdf_meshes.py
- robot_model.hpp
- Documentation index
- visual_origin_iso
- test_ui_layout.py
- load_robot
- test_milestone_r2_end_to_end
- json
- Compiled model layout (frozen, layout_version 1)
- info_panel.py
- cartesian.py
- Data formats
- .set_target_pose
- urm_importer.py
- ViolationReport
- test_pdf_report_runs_on_gui_thread
- Dynamics validation
- Collision library decision (R7.2)
- .set_store
- test_u2_floor_drop.py
- planar_2r
- clean-machine-test.md
- Hardware Audit — R0.1
- test_urm_bridge.py
- AppSettings
- generate_report
- test_sample_q_matches_independent_numpy_philox
- MainWindow
- .set_level
- .set_collision_shapes

## God Nodes (most connected - your core abstractions)
1. `MainWindow` - 252 edges
2. `random_robot()` - 113 edges
3. `AppSettings` - 109 edges
4. `RobotView` - 89 edges
5. `planar_2r()` - 82 edges
6. `save()` - 81 edges
7. `dof_limits()` - 81 edges
8. `spec_from_model()` - 80 edges
9. `SimPanel` - 68 edges
10. `import_urdf()` - 68 edges

## Surprising Connections (you probably didn't know these)
- `Current` --references--> `fault_active()`  [INFERRED]
  PROGRESS.md → cuda/gpu_manager.cu
- `Enum values` --references--> `JointType`  [INFERRED]
  docs/compiled-model-layout.md → core/urm/joint.hpp
- `Core` --references--> `SimState`  [INFERRED]
  docs/simulation.md → app/sim/state.py
- ``time_parametrize`` --references--> `PlanningError`  [INFERRED]
  docs/api-reference.md → robotics_api/errors.py
- `Playback (`TrajectoryPlayer`)` --references--> `TrajectoryPlayer`  [INFERRED]
  docs/trajectory.md → app/sim/playback.py

## Import Cycles
- None detected.

## Communities (262 total, 83 thin omitted)

### Community 0 - "Software Requirements Specification"
Cohesion: 0.15
Nodes (13): 10. Visualization and results, 11. GPU and performance, 12. Project and extensibility, 13. Acceptance criteria, 1. Product scope, 3. Unified Robot Model, 4. Physical properties, 5. Kinematics (+5 more)

### Community 1 - "DynModel"
Cohesion: 0.07
Nodes (60): SimState, bias_torque(), build_dyn_model(), check_size(), coriolis_torque(), crossf(), crossm(), dyn_frames() (+52 more)

### Community 2 - "IKPanel"
Cohesion: 0.06
Nodes (3): write_q(), IKPanel, target_iso()

### Community 3 - "Technical Architecture"
Cohesion: 0.18
Nodes (11): 10. Validation and testing, 1. Architectural principles, 2. High-level data flow, 3. Main subsystems, 4. Unified Robot Model, 5. Simulation state, 6. CPU/GPU execution, 7. CAD and URDF pipeline (+3 more)

### Community 5 - "test_ui_kin_dyn.py"
Cohesion: 0.15
Nodes (30): current_q(), rotation_to_rpy(), _expected_tau(), _fill_from_pose(), _load(), _pump(), _set_q(), _solve() (+22 more)

### Community 6 - "Development Workflow"
Cohesion: 0.15
Nodes (12): Agent Roles, Claude Model Tiering (token saving), Development Workflow, Error Substage Rule, Hardware Context (OMEN 16), Key Rules, Progress File, Stage-Batch Workflow (active from Stage R3) (+4 more)

### Community 7 - "Agent Communication Style Rules"
Cohesion: 0.50
Nodes (3): Agent Communication Style Rules, Rule 1 — Claude Agents: Caveman English, Rule 2 — Gemini Agents: ASD-STE100 Simplified Technical English

### Community 8 - "CollisionPanel"
Cohesion: 0.12
Nodes (5): CollisionPanel, _fmt(), _params_text(), test_collision_panel_edit_list(), test_collision_zero_pose_button()

### Community 9 - "import_urdf"
Cohesion: 0.16
Nodes (30): import_urdf(), _assert_canonical(), _codes(), _fx(), test_continuous_joint_limits_and_warning(), test_error_fixture_reports_code(), test_gripper_fk_both_fingers(), test_gripper_mimic_tree_structure() (+22 more)

### Community 12 - "test_recovery.py"
Cohesion: 0.09
Nodes (30): autosave_path(), load_best(), newer_autosave(), remove_autosave(), write_autosave(), prev_path(), save(), test_item6_autosave_and_recovery_after_crash() (+22 more)

### Community 13 - "CompiledArrays"
Cohesion: 0.07
Nodes (34): at(), check_finite(), check_size(), CompiledArrays, axis, base, child_link, joint_index (+26 more)

### Community 14 - "dof_limits"
Cohesion: 0.06
Nodes (56): _ik_inputs(), dof_limits(), _enum_name(), fk_numpy(), hom(), _mass_matrix_numpy(), numeric_jacobian4(), pair() (+48 more)

### Community 16 - "DHEditorWidget"
Cohesion: 0.06
Nodes (14): DHEditorWidget, make_robot(), test_dh_editor_a_values_in_table(), test_dh_editor_add_joint(), test_dh_editor_creation(), test_dh_editor_edit_updates_model(), test_dh_editor_invalid_float_reverts(), test_dh_editor_joint_names_in_table() (+6 more)

### Community 17 - "AppSettings"
Cohesion: 0.04
Nodes (23): ProjectController, Atomic save, Autosave and crash recovery, Diagnosis, Files next to a project `name.robotproj`, Fixtures and tests, Migration, Project migration and recovery (R8.4) (+15 more)

### Community 19 - "urdf.py"
Cohesion: 0.14
Nodes (20): ImportReport, _build(), _build_joint(), _Fatal, _joint_limits(), _make_shape(), _named_materials(), _num() (+12 more)

### Community 20 - "random_robot"
Cohesion: 0.06
Nodes (63): inertias_from_model(), numeric_jacobian(), numeric_jacobian_tcp(), random_inertias(), random_q(), random_robot(), test_fk_jacobian_vs_numpy_ref(), test_manipulability_vs_numpy_svd() (+55 more)

### Community 21 - "test_milestone_a1.py"
Cohesion: 0.08
Nodes (22): backup_path(), make_backup(), _same_file(), _corrupt_kinds(), _good(), _meta(), _rewrite(), test_corruption_maps_to_format_error_and_prev_fallback() (+14 more)

### Community 24 - "math"
Cohesion: 0.09
Nodes (17): _segment_times(), _time_parametrize_topp(), TimedPath, _dedupe(), _infeasible(), _lp2d(), _norm_row(), solution_trajectory() (+9 more)

### Community 25 - "test_milestone_r3.py"
Cohesion: 0.15
Nodes (14): _axis_angle(), _drive(), _floats(), _hom(), _import(), NumpyRobot, _q_now(), _rpy() (+6 more)

### Community 26 - "TaskRunner"
Cohesion: 0.04
Nodes (29): CompareTask, _Token, Task, TaskRunner, WorkerSignals, Common rules (every substage), Decisions fixed for R5 (from the Opus consult, 2026-11-11; do not reopen), R5.3 --- Workspace sampling (C++/CUDA + bindings) (+21 more)

### Community 27 - "test_sim_panel.py"
Cohesion: 0.13
Nodes (14): check_mode(), switch_mode(), _loaded(), _pump(), test_buttons_follow_state(), test_error_pauses_and_shows(), test_event_loop_not_blocked(), test_kinematic_estimate_torque_default_off_and_on() (+6 more)

### Community 28 - "PerformancePanel"
Cohesion: 0.12
Nodes (3): _fmt_rate(), PerformancePanel, work()

### Community 29 - "unittest_mock"
Cohesion: 0.12
Nodes (26): is_plain_chain(), _import(), _make_dh_arm(), _model_json(), test_branch_segments_move_with_q(), test_dh_editor_convention_combo_still_works_on_tree(), test_dh_editor_origin_axis_rows_read_only(), test_dh_editor_refuses_add_remove_on_tree() (+18 more)

### Community 30 - "GraphPanel"
Cohesion: 0.05
Nodes (11): PlotView, GraphPanel, _pump(), _run(), test_all_kinds_draw_after_a_run(), test_clear_empties_plot_and_recorder(), test_data_survives_stop_and_graph_still_drawn(), test_joint_checklist_filters_curves() (+3 more)

### Community 31 - "robots.py"
Cohesion: 0.05
Nodes (51): _arr(), check_trajectory(), _model_limits(), _runs(), _samples(), Violation, time_parametrize(), numpy_dynamics() (+43 more)

### Community 32 - "numpy"
Cohesion: 0.05
Nodes (65): find_stretched_pose(), _positions(), arc_geometry(), resolve_gains(), _z(), fk_2r(), fk_3r(), ik_2r() (+57 more)

### Community 33 - "CollisionWorld"
Cohesion: 0.09
Nodes (17): CollisionConfig, CollisionWorld, env_id(), _key(), Consequences and limits, seg_hits_rect(), test_plan_path_is_valid_and_avoids_box(), _cfg() (+9 more)

### Community 34 - "test_viewport_robot.py"
Cohesion: 0.08
Nodes (7): make_3r_planar_robot(), test_3r_planar_model_actors_and_polyline(), test_camera_not_reset_on_robot_changed(), test_empty_model_draws_base_and_tcp_only(), test_update_in_place_new_q_moves_polyline(), test_view_robot_submenu_actions(), test_visibility_toggles()

### Community 35 - "Current"
Cohesion: 0.07
Nodes (24): Cancel behaviour, `check_cancel`, Error notes, `ScriptCancelled`, `ScriptTimeLimit`, Current, New agent? Read in this order, _arrays() (+16 more)

### Community 37 - "test_urdf_meshes.py"
Cohesion: 0.09
Nodes (22): fallback_cube(), _first_file(), LinkVisual, load_visual_mesh(), mesh_format_ok(), read_mesh_file(), triangle_count(), Canonical order (+14 more)

### Community 38 - "test_performance.py"
Cohesion: 0.08
Nodes (28): estimate_reach(), _measure(), _row(), _run(), run_benchmark(), _time(), versions(), write_benchmark() (+20 more)

### Community 39 - "urdf_export.py"
Cohesion: 0.21
Nodes (15): _enum(), _f(), _geometry(), _matrix_xyz_rpy(), _origin(), _pose_matrix(), _pose_matrix_rot(), _pre_post_axis() (+7 more)

### Community 41 - "RobotModel"
Cohesion: 0.04
Nodes (84): Pose, rpy, xyz, default_limits(), DHConvention, Modified, Standard, from_json() (+76 more)

### Community 42 - "kinds.py"
Cohesion: 0.13
Nodes (6): ImportResult, _garbage(), PluginAnalysis, PluginPlanner, validate_import_result(), validate_plan_result()

### Community 43 - "compile"
Cohesion: 0.08
Nodes (36): active_chain(), check_joint_inputs(), compile(), effective_links(), is_legacy_serial(), require_finite(), resolve_tcp(), rx() (+28 more)

### Community 44 - "core_bridge"
Cohesion: 0.04
Nodes (70): singularity_at(), _mass_section(), dh_chain(), fk_from_arrays(), mimic_model(), _motion(), test_a14_compile_rejects_bad_models(), test_a14_from_arrays_range_checks() (+62 more)

### Community 45 - "kin_bindings.cpp"
Cohesion: 0.03
Nodes (54): bind_kin(), compiled_arrays_to_dict(), fk_result_to_dict(), to_np(), CompiledModel, base, joints, n_dof (+46 more)

### Community 46 - "CompiledJoint"
Cohesion: 0.12
Nodes (17): CompiledJoint, axis, child_link, joint_index, mult, offset, parent_link, post (+9 more)

### Community 47 - "test_fk.cpp"
Cohesion: 0.22
Nodes (15): main(), test_1r(), test_2r_planar(), test_3d_known_answer_mdh(), test_3d_known_answer_standard_dh(), test_3r_planar_random(), test_base_and_tool(), test_dh_vs_mdh() (+7 more)

### Community 48 - "SimPanel"
Cohesion: 0.05
Nodes (8): SimPanel, button_states(), Panel, panel(), rig(), panel(), test_pause_resume_seek_kinematic(), test_seek_while_stopped_shows_exact_q()

### Community 49 - "subprocess"
Cohesion: 0.21
Nodes (8): run_smoke(), scrubbed_env(), _spec_literal(), _spec_tree(), test_built_exe_smoke_scrubbed_env(), test_hidden_imports_cover_third_party(), test_smoke_flag_in_source_tree(), test_spec_parses_and_has_parts()

### Community 50 - "JointPanel"
Cohesion: 0.04
Nodes (15): JointPanel, JointRow, make_test_arm(), test_all_rows_visible_7_joints(), test_dh_editor_edits_rebuild_joint_panel(), test_out_of_limit_q_shows_warning_and_does_not_modify_model(), test_q_edit_sets_dirty_flag(), test_rotation_to_rpy_round_trip_and_gimbal_lock() (+7 more)

### Community 51 - "_prm_cancel_latency"
Cohesion: 0.14
Nodes (4): _CpuLoad, _prm_cancel_latency(), _SlowCollision, test_u2_prm_cancel_latency_under_load()

### Community 52 - "planar_2r"
Cohesion: 0.09
Nodes (36): plan(), path_length(), shortcut(), _Blocked, test_low_cancel_object_with_method_style_token(), test_m7_prm_cancel_in_blocked_space(), test_m7_prm_time_limit_in_blocked_space(), planar_2r() (+28 more)

### Community 53 - "test_dynamics.cpp"
Cohesion: 0.25
Nodes (13): a14_missing_link_and_nonfinite(), case_A(), case_B(), case_C(), case_C_model(), dmodel(), link_with(), main() (+5 more)

### Community 56 - "math"
Cohesion: 0.09
Nodes (20): JointProfile, _limit(), quintic(), _Quintic, _Scalar, scurve(), trapezoid(), _bounds() (+12 more)

### Community 57 - "test_ui_trajectory.py"
Cohesion: 0.07
Nodes (7): _fmt(), TimelinePanel, test_delete_move_capture(), test_deleting_all_rows_clears_trajectory(), test_model_n_dof_change_drops_trajectory(), test_table_add_from_current_q_and_trajectory_signal(), tl()

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

### Community 62 - "Robot"
Cohesion: 0.05
Nodes (27): Members, `Robot`, `Robot.collision_shapes`, `Robot.copy`, `Robot.energy`, `Robot.fk`, `Robot.forward_dynamics`, `Robot.from_json` (+19 more)

### Community 63 - "GhostPanel"
Cohesion: 0.09
Nodes (5): GhostPanel, _qcolor(), test_ik_solutions_added_and_shown(), test_max_eight_ghosts_message(), test_panel_save_show_compare()

### Community 64 - "main_window.py"
Cohesion: 0.05
Nodes (12): create_app(), _destroy_window(), _sample_urdf(), smoke(), resolved_tcp_link(), FunctionTask, App layers (`app/`), test_main_window_creation() (+4 more)

### Community 65 - "PlanningPanel"
Cohesion: 0.07
Nodes (11): _fmt(), PlanningPanel, progress(), work(), src(), panel(), _pump_until(), test_panel_capture_named_and_model_change() (+3 more)

### Community 66 - "WorkspacePanel"
Cohesion: 0.07
Nodes (6): SliceView, work(), WorkspacePanel, work(), progress(), work()

### Community 67 - "2. Robot inputs"
Cohesion: 0.50
Nodes (4): 2. Robot inputs, CAD, Custom builder, URDF

### Community 69 - "Continue Protocol (Sonnet orchestrator + Builder subagents)"
Cohesion: 0.40
Nodes (4): Continue Protocol (Sonnet orchestrator + Builder subagents), Legacy per-substage protocol (R0 to R2 only, do not use), Read order (every new session), Stage-batch loop

### Community 70 - "test_import_report.py"
Cohesion: 0.14
Nodes (18): find_element(), _click_row(), _import(), _row_severities(), test_dock_rows_per_fixture(), test_errors_keep_old_project(), test_errors_raise_dock_info_only_does_not(), test_filters_do_not_change_saved_report() (+10 more)

### Community 72 - "test_milestone_r8.py"
Cohesion: 0.08
Nodes (10): guarded_call(), _on_gui_thread(), ProjectView, load_manifest(), Manifest, ManifestError, clock(), dialog_time() (+2 more)

### Community 73 - "User guide"
Cohesion: 0.10
Nodes (21): 10. Scripting, 11. Plugins, 12. Projects, autosave and recovery, 13. Troubleshooting, 1. Install, 3. Joints and the TCP, 4. Inverse kinematics, 5. Dynamics and analysis (+13 more)

### Community 74 - "test_compiled.cpp"
Cohesion: 0.26
Nodes (17): Iso, legacy_model(), main(), make_links(), oa(), test_a14_validation(), test_legacy_fallback(), test_mimic() (+9 more)

### Community 75 - "Workspace analysis (R7.5)"
Cohesion: 0.33
Nodes (6): Method, Numbers (Ryzen 9 8940HX, one thread, Python loop over `cb.jacobian_tcp` + `cb.analyze_singularity`), Singularity regions, Views, Volume and error bound, Workspace analysis (R7.5)

### Community 76 - "test_integrators.py"
Cohesion: 0.08
Nodes (36): SimulationSettings, RK45, _link(), _oa(), robot(), test_pendulum_potential_closed_form(), test_potential_energy_free_mass_height(), _accel_pendulum() (+28 more)

### Community 77 - "WsKernelArgs"
Cohesion: 0.10
Nodes (20): WsKernelArgs, count, d_a, d_counts, d_div, d_itab, d_lo, d_occ (+12 more)

### Community 78 - "test_logger.py"
Cohesion: 0.15
Nodes (3): AppLogger, test_logger_handlers_not_duplicated_and_dir_fallback(), temp_logger()

### Community 79 - "ConfigStore"
Cohesion: 0.12
Nodes (8): ConfigStore, test_shown_bad_entries_skipped_with_warning(), test_shown_follows_remove_rename_clear(), test_shown_round_trip_order_colours_and_limits(), test_store_basics_and_validation(), test_store_dict_round_trip(), test_validate_dof_drops_and_logs(), test_configs_bad_entry_skipped_others_load()

### Community 80 - "test_plugins.py"
Cohesion: 0.08
Nodes (51): PluginCallError, api_compatible(), parse_manifest(), test_c_slow_permission_dialog_and_unanswered_ask(), call_plan(), imp_plugin(), manifest(), mgr() (+43 more)

### Community 81 - "gpu_bridge.cpp"
Cohesion: 0.10
Nodes (20): build_tables(), ensure_pool(), fk_batch_py(), get_vec(), parse_arrays(), Pending, cn, start (+12 more)

### Community 82 - "test_milestone_r4.py"
Cohesion: 0.15
Nodes (19): integrate_rk4_forward_dynamics(), _fill(), _gravity_setting(), _load(), _numpy_sigma(), _numpy_tau(), _pose_err(), _pump() (+11 more)

### Community 83 - "IkBatchResult"
Cohesion: 0.09
Nodes (18): all_finite(), ik_batch_cpu(), IkBatchResult, iterations, n, n_dof, pos_error, q (+10 more)

### Community 84 - "workspace_kernel.cu"
Cohesion: 0.10
Nodes (15): check_sample_q_size(), sample_q_gpu(), workspace_gpu(), BitFlag, __device__, occ, LocalQ, __forceinline__ (+7 more)

### Community 85 - "test_bridge.py"
Cohesion: 0.20
Nodes (4): test_bridge_add(), test_bridge_add_type(), test_bridge_get_version(), test_bridge_import()

### Community 86 - "test_milestone_r5.py"
Cohesion: 0.26
Nodes (14): _tolerances(), _gpu_or_skip(), _load(), _pump(), test_item_3_subset_gpu_equals_cpu(), test_item_5a_cancel_mid_run_gpu(), test_item_5b_cpu_by_setting_matches_gpu_subset(), test_item_5c_fake_oom_falls_back_and_reports() (+6 more)

### Community 88 - "AnalysisPanel"
Cohesion: 0.05
Nodes (9): check_torque_limits(), CompiledCache, AnalysisPanel, mass_not_set(), NumField, Analysis dock, IK and Analysis panels (R4.9), Inverse Kinematics dock (+1 more)

### Community 89 - "test_ui_trajectory.py"
Cohesion: 0.24
Nodes (14): _load(), _loaded(), _model(), _pump(), _set(), test_dynamic_playback_shows_tracking_error(), test_editing_table_during_playback_discards_player(), test_invalid_rows_are_marked_not_dropped() (+6 more)

### Community 90 - "test_compiled_arrays.cpp"
Cohesion: 0.28
Nodes (12): check_round_trip(), main(), make_links(), max_diff(), oa(), test_bad_arrays_throw(), test_effective_links_legacy(), test_layout_values() (+4 more)

### Community 91 - "Any"
Cohesion: 0.13
Nodes (4): joint_links(), R2.7 --- Viewport: links, frames, TCP, Code facts (checked 2026-11-11, after R3), test_half_size_rounding()

### Community 92 - "test_ui_workspace.py"
Cohesion: 0.23
Nodes (11): _cpu_panel(), _load(), _pump(), test_cancel_stops_run(), test_clear_on_project_close(), test_clear_on_robot_change_and_stale_result_dropped(), test_error_is_reported_not_raised(), test_fallback_reason_shown() (+3 more)

### Community 93 - "compare.py"
Cohesion: 0.15
Nodes (13): compare_robot(), dof_efforts(), dof_master_joints(), dof_names(), _limit_text(), _norm3(), _norm_xy(), _num() (+5 more)

### Community 94 - "PluginManager"
Cohesion: 0.07
Nodes (14): PluginManagerPanel, PluginImporter, default_plugins_folder(), LoadedPlugin, LoadResult, PluginManager, DictSettings, test_unanswered_ask_is_not_saved_as_denied() (+6 more)

### Community 95 - "motion.py"
Cohesion: 0.10
Nodes (18): LOW (fix when nearby), A1.7 --- Small app fixes (LOW app/API/packaging items) --- Haiku tier, InvalidArgument, _cancel_or_raise_if(), _obstacle(), plan(), simulate(), tau_of() (+10 more)

### Community 97 - "test_milestone_r6.py"
Cohesion: 0.23
Nodes (15): _export(), _install(), _kinematic_run(), _load(), _pump(), _pump_until(), _show_dock(), test_item_3_kinematic_playback() (+7 more)

### Community 98 - "graph_data"
Cohesion: 0.08
Nodes (28): wrap_pi(), decimate_minmax(), graph_data(), ser(), _joint_unit(), _rpy_from_quat(), Series, _stride() (+20 more)

### Community 99 - "Stage A1 spec pack: Audit fixes"
Cohesion: 0.13
Nodes (7): _GuiAsker, A1.1 --- Close flow (H1, M3, LOW: temp dirs, stale autosave), A1.8 --- Exit crash 0xC0000409 (H3) --- STOP if root cause not found, A1.9 --- A1 milestone and stage gate, Common rules (every substage), Not in A1 (won't fix now, reason), Stage A1 spec pack: Audit fixes

### Community 100 - "test_sim_panel.py"
Cohesion: 0.29
Nodes (6): _load(), _settings(), test_engine_lifecycle_open_edit_close(), test_free_fall_matches_numpy_rk4(), test_hide_dock_stops_timer(), test_joint_slider_in_kinematic_updates_engine()

### Community 101 - "test_gpu_manager.py"
Cohesion: 0.18
Nodes (8): _device_or_skip(), _gpu(), test_chunk_size_invalid(), test_chunk_size_invalid_headroom(), test_device_info_is_rtx_5060_sm120(), test_fake_oom_hook_toggles(), test_gpu_bridge_imports_and_reports(), test_mem_info_free_le_total()

### Community 103 - "re"
Cohesion: 0.32
Nodes (5): _safe(), test_api_version_string(), _code_constants(), _doc_codes(), test_docs_code_table_matches_constants()

### Community 105 - "save"
Cohesion: 0.07
Nodes (30): load(), _open_modified(), test_close_discard_removes_autosave_and_temp(), test_close_modified_prompts_and_cancel_keeps_window(), test_close_save_writes_file(), test_open_other_project_removes_old_autosave_and_temp(), test_project_roundtrip_acm_and_environment(), test_project_without_collision_has_no_file_and_old_file_loads() (+22 more)

### Community 106 - "test_gripper.py"
Cohesion: 0.20
Nodes (8): gripper_dofs(), GripperDof, _load(), _model(), test_arm_only_has_no_group(), test_detection_on_all_samples(), test_finger_link_moves_with_close(), test_open_close_buttons_move_dof_and_fk()

### Community 107 - "budget.py"
Cohesion: 0.16
Nodes (13): _gpu_mem_info(), max_batch(), set_vram_budget_mb(), vram_budget_bytes(), vram_budget_mb(), Budget rule, Error policy, GPU compute (R5) (+5 more)

### Community 108 - "RoboticsApiError"
Cohesion: 0.10
Nodes (23): `AnalysisError`, `ComputationError`, Errors, `ExportError`, `import_urdf`, `ImportFailed`, `InvalidArgument`, `new_project` (+15 more)

### Community 109 - "GPU FK precision decision (R5.2)"
Cohesion: 0.25
Nodes (8): fp32 test tolerances (R5 Decisions 12), GPU FK precision decision (R5.2), Hardware and software, Measurement, Notes, Observed errors (1e6 samples, `tests/python/test_gpu_fk.py`), Rule (R5 spec, R5.2 item 8), SASS check (fp32 kernel)

### Community 110 - "test_collision.py"
Cohesion: 0.09
Nodes (24): EnvShape, fit_mesh(), _mesh_points(), pose_matrix(), Shape, shape_distance(), shapes_from_geometry(), shapes_from_project() (+16 more)

### Community 111 - "Rigid-body dynamics (R4.6 to R4.8)"
Cohesion: 0.25
Nodes (8): Energy and tools (R4.8), Errors and hooks, Forward dynamics, Friction, Mass matrix (CRBA), Model, Rigid-body dynamics (R4.6 to R4.8), RNEA (inverse dynamics)

### Community 112 - "test_exit_clean.py"
Cohesion: 0.46
Nodes (6): _msg(), _run(), test_main_window_close_script_exits_zero(), test_module_start_smoke_exits_zero(), test_script_start_smoke_exits_zero_three_times(), test_ui_ghosts_subprocess_exits_zero_three_times()

### Community 113 - "Trajectory"
Cohesion: 0.08
Nodes (27): build(), TrajectoryPlayer, Trajectory, Waypoint, test_low_playback_same_wall_time_any_time_scale(), excepthook_spy(), test_hermite_trajectory_exact_and_roundtrip(), _engine() (+19 more)

### Community 114 - "_Space"
Cohesion: 0.07
Nodes (13): _cancel_fn(), _Cancelled, _goal_of(), ik_goals(), fail(), PlanResult, _prm(), _rrt_connect() (+5 more)

### Community 115 - "current_q"
Cohesion: 0.29
Nodes (9): _fake_clock(), _load(), _tcp(), test_drag_reaches_target(), test_gripper_dofs_unchanged(), test_throttle_drops_events_and_release_solves_last(), test_unreachable_keeps_q_and_flags(), test_vtk_event_calls_handler() (+1 more)

### Community 116 - "U2 spec pack: usability fixes, interaction features, validation gaps"
Cohesion: 0.15
Nodes (12): Common rules, Original U2.8 text (Pinocchio), Stage end (U2.10, orchestrator), U2.1 Small fixes (sonnet), U2.2 PRM cancel latency (sonnet), U2.3 Floor grid + drag-and-drop URDF (sonnet), U2.4 Gripper open/close (sonnet), U2.5 Teach waypoints (sonnet) (+4 more)

### Community 117 - "test_compute_backend.py"
Cohesion: 0.07
Nodes (30): Backend, CpuBackend, get_backend(), _gpu_bridge_or_reason(), GpuBackend, _import_gpu_bridge(), Python backends (`app/compute/backend.py`), FakeGb (+22 more)

### Community 118 - "test_report.py"
Cohesion: 0.14
Nodes (19): work(), _data_uri(), _e(), _g(), render_html(), _table(), write_html(), write_pdf() (+11 more)

### Community 119 - "bench_fk.py"
Cohesion: 0.23
Nodes (11): _cpu_name(), main(), _median(), _msvc_version(), _run(), _toolkit(), _versions(), _cpu_name() (+3 more)

### Community 120 - "ModelPanel"
Cohesion: 0.06
Nodes (9): active_link_set(), joint_type_name(), ModelPanel, add_link(), style(), _fmt(), PropertiesPanel, 2. Stage R3: Import   (BLOCKING review) (+1 more)

### Community 121 - "test_physical_props.py"
Cohesion: 0.06
Nodes (36): _check_density_mass(), _closed_mesh_inertial(), estimate_inertial(), _mass_properties(), _origin_matrix(), _pack(), primitive_inertial(), visual_inertial() (+28 more)

### Community 123 - "test_script_runner.py"
Cohesion: 0.06
Nodes (36): AppHandle, _flush_thread(), _install(), ScriptRunner, sink(), work(), _ThreadStream, _uninstall() (+28 more)

### Community 125 - "Slot"
Cohesion: 0.13
Nodes (12): Slot, cn, d_pts, e0, e1, h_pts, pt_n, st (+4 more)

### Community 126 - "test_u2_fix.py"
Cohesion: 0.39
Nodes (6): _load(), test_compare_table_scrolls_and_elides(), test_drag_off_resets_status(), test_import_xarm5_gripper_keeps_report_dock_hidden(), test_sim_panel_fits_380_and_teach_on_top(), test_tree_root_shows_display_name()

### Community 127 - "gpu_manager.cu"
Cohesion: 0.05
Nodes (44): check_alloc(), checked_malloc(), cuda_check(), CudaError, DeviceInfo, available, driver_version, error (+36 more)

### Community 128 - "PluginHost"
Cohesion: 0.08
Nodes (4): PluginHost, _key(), PermissionManager, Host capabilities

### Community 129 - "test_urdf_export.py"
Cohesion: 0.11
Nodes (24): export_meshes(), export_urdf(), pose(), make(), one_dof(), check_fk(), check_structure(), dof_index() (+16 more)

### Community 130 - "GPU kernels (R5)"
Cohesion: 0.29
Nodes (7): ws_kernel(), FK kernel (`fk_tcp_kernel<T>`, T = float or double), GPU IK (experimental), GPU kernels (R5), Launch time (WDDM TDR) rule, Precision, Workspace kernel (`ws_kernel`)

### Community 131 - "WorkspaceResult"
Cohesion: 0.03
Nodes (73): fill_q_grid(), fill_q_random(), fma32(), philox4x32_10(), philox_mulhilo(), philox_u01(), bitset_to_voxels(), make_sampler() (+65 more)

### Community 132 - "2. First project"
Cohesion: 0.29
Nodes (7): 2. First project, Build or edit a DH model, Import a URDF, Layout: side bar, Focus 3D (F11), Reset layout, Open a sample robot, Physical properties, Start a project

### Community 136 - "FkPathTables"
Cohesion: 0.04
Nodes (49): sincos_t(), tcp_walk(), fk_launch(), fk_pack_tables(), FkPathTables, axis, mult, n (+41 more)

### Community 137 - "stdexcept"
Cohesion: 0.05
Nodes (21): chunk_size(), default_threads(), parallel_for(), active_path(), fk_batch_cpu(), FkBatchResult, n, position (+13 more)

### Community 138 - "report.py"
Cohesion: 0.23
Nodes (7): _collision_section(), _enum(), graphs_from_arrays(), _joint_rows(), _param_rows(), _plan_section(), render_graph_png()

### Community 145 - "Simulation (R6.1 - R6.3)"
Cohesion: 0.33
Nodes (6): Core, Integrators (`integrators.py`), Modes (`mode.py`), Settings (`SimulationSettings`), Simulation (R6.1 - R6.3), State machine

### Community 146 - "SimEngine"
Cohesion: 0.07
Nodes (16): SimClock, SimEngine, UI driving, test_item_4_energy_balance_free_motion(), _engine(), test_advance_only_when_running(), test_advance_time_scale_two(), test_clock_illegal_transitions() (+8 more)

### Community 147 - ".log"
Cohesion: 0.06
Nodes (7): done(), done(), test_export_actions_without_robot_show_info_only(), test_export_cancelled_dialog_writes_nothing(), test_export_report_urdf_meshes_from_menu(), wait_for(), test_main_window_set_status()

### Community 149 - "test_workspace_analysis.py"
Cohesion: 0.08
Nodes (33): analyze_workspace(), _boundary_count(), _chunk_stats(), field(), _merge(), _opts(), sample_metrics(), slice_image() (+25 more)

### Community 150 - "test_cuda.cpp"
Cohesion: 0.24
Nodes (7): cuda_device_info(), gpu_vector_add(), main(), nearly_equal_f(), test_device_info(), test_vector_add_large(), test_vector_add_small()

### Community 151 - "test_logger.py"
Cohesion: 0.10
Nodes (8): QtLogHandler, test_app_logger_creates_log_file(), test_app_logger_creation(), test_app_logger_writes_to_file(), test_logger_log_path_is_in_appdata(), test_qt_handler_creation(), test_qt_handler_emits_signal(), test_qt_handler_formats_level()

### Community 152 - "test_teach.py"
Cohesion: 0.10
Nodes (22): import_display_name(), list_sample_robots(), sample_robots_dir(), _sample_menu(), test_file_menu_has_nine_enabled_entries(), test_list_sample_robots_finds_nine(), test_list_sample_robots_missing_folder(), test_list_sample_robots_title_fallback() (+14 more)

### Community 154 - "test_gpu_errors.py"
Cohesion: 0.18
Nodes (8): _clean(), gb(), test_sample_q_oom_is_memory_error(), test_sample_q_over_budget_is_value_error(), test_unknown_fault_name_rejected(), test_d_bad_arrays_gpu_alive(), gb(), _gpu_or_skip()

### Community 155 - "validate_dynamics.py"
Cohesion: 0.18
Nodes (12): test_robot_matches_mujoco(), compare(), dof_info(), fmt(), load_app(), main(), mujoco_eval(), passes() (+4 more)

### Community 156 - "store.py"
Cohesion: 0.18
Nodes (5): compare_configs(), joint_is_prismatic(), _rotvec(), dof_joint_types(), _fmt()

### Community 157 - "test_docs.py"
Cohesion: 0.12
Nodes (22): _api(), _index_targets(), _load_check_docs(), _run(), test_api_reference_is_generated_and_current(), test_api_reference_names_every_public_name(), test_checker_cli_exit_codes(), test_checker_reports_broken_link_and_anchor() (+14 more)

### Community 158 - "Trajectory"
Cohesion: 0.08
Nodes (12): Members, `Trajectory`, `Trajectory.duration`, `Trajectory.interpolation`, `Trajectory.n_dof`, `Trajectory.q`, `Trajectory.qd`, `Trajectory.qdd` (+4 more)

### Community 159 - "Project"
Cohesion: 0.12
Nodes (9): Members, `Project`, `Project.description`, `Project.name`, `Project.path`, `Project.robot`, new_project(), open_project() (+1 more)

### Community 164 - "Stage R8 spec pack: Extensibility and release"
Cohesion: 0.20
Nodes (9): Common rules (every substage), R8.1 --- Python scripting API, R8.2 --- Plugin interfaces and permissions, R8.3 --- Reports and export, R8.4 --- Project migration and recovery, R8.5 --- Installer and packaging  --- STOP after this substage, R8.6 --- Documentation, R8.7 --- R8 milestone and stage gate (+1 more)

### Community 165 - "test_milestone_r8.py"
Cohesion: 0.09
Nodes (24): assert_matches(), controller(), _no_dialogs(), reference(), _same_visual(), test_backup_created_exactly_once_and_original_untouched(), test_current_format_file_gets_no_backup(), test_fixture_layouts_match_their_era() (+16 more)

### Community 167 - "QDockWidget"
Cohesion: 0.05
Nodes (8): _scrollable(), R3.5 --- Meshes and assets (Python), R3.6 --- Import validation and error report (Python UI), R3 stage milestone and gate (Builder does this after R3.7), Stage R3 spec pack: Import, test_dock_wiring_side_bar_and_view_menu(), test_dock_toggle_in_view_menu(), test_docks_exist_hidden_and_toggle()

### Community 169 - "CollisionConfig"
Cohesion: 0.22
Nodes (9): main(), bad_json(), bad_robot(), bad_schema(), members_of(), reference_project(), _rewrite(), _set_meta() (+1 more)

### Community 170 - "Developer notes"
Cohesion: 0.18
Nodes (11): Adding a plugin kind or a sample plugin, Adding a project format migration, Architecture, Build, Decisions summary, Developer notes, Docs, Model data (+3 more)

### Community 171 - "SyncRunner"
Cohesion: 0.17
Nodes (4): SyncRunner, test_ik_panel_connects_before_submit(), test_main_window_import_connects_before_submit(), test_workspace_panel_connects_before_submit()

### Community 172 - "Slot"
Cohesion: 0.12
Nodes (15): Pool, d_itab, d_tab, init, slot, Slot, d_in, d_out (+7 more)

### Community 174 - "Stage R7 spec pack: Planning and advanced analysis"
Cohesion: 0.10
Nodes (15): Code table, Dock behaviour, Import report, Output format, Severity policy, Common rules (every substage), R7.1 --- Trajectory generation and constraint checks, R7.2 --- Collision geometry (feasibility first) (+7 more)

### Community 176 - "export_csv"
Cohesion: 0.50
Nodes (3): export_csv(), f(), test_csv_round_trip()

### Community 179 - "test_ui_ghosts.py"
Cohesion: 0.29
Nodes (10): _clean(), _load(), _pump(), _pump_until(), _reopen(), test_ghost_count_and_tcp_vs_numpy(), test_ik_solution_list_feeds_ghosts(), test_planning_wiring_plan_and_send() (+2 more)

### Community 180 - "test_workspace.py"
Cohesion: 0.12
Nodes (38): fk_test_robots(), test_ws_count_bits_oom_takes_host_path(), robots(), bits(), _cancel_case(), limits(), _persistent_estimate(), philox_py() (+30 more)

### Community 181 - "string"
Cohesion: 0.11
Nodes (18): analyze_singularity(), SingularityInfo, condition, manipulability, near_singular, sigma_max, sigma_min, singular_values (+10 more)

### Community 184 - "Trajectory and playback (R6.4)"
Cohesion: 0.25
Nodes (8): Interpolation, Model, Playback (`TrajectoryPlayer`), Project format 1.2.0, Project format 1.3.0 (R7), Project format 1.4.0 (F1.4), Trajectory and playback (R6.4), UI

### Community 185 - "test_recorder.py"
Cohesion: 0.05
Nodes (35): euler_step(), make_integrator(), rk4_step(), _make_static(), set_kinematic_q(), capacity_for(), _quat_wxyz(), Recorder (+27 more)

### Community 187 - "test_ui_script.py"
Cohesion: 0.31
Nodes (8): test_cancel_button_stops_long_script(), test_close_with_running_script_cancels_it(), test_exception_shown_in_log_and_app_survives(), test_file_menu_run_script_uses_file_dialog(), test_gui_event_loop_runs_while_script_runs(), test_run_editor_text_output_in_panel_and_log(), test_script_sees_open_project_through_app_handle(), wait()

### Community 188 - "Packaging (R8.5)"
Cohesion: 0.33
Nodes (6): Build, CUDA runtime, Installer, Known limits, Packaging (R8.5), Smoke start

### Community 192 - "test_milestone_r2_end_to_end"
Cohesion: 0.28
Nodes (4): independent_dh_modified(), independent_dh_standard(), independent_pose_to_matrix(), test_fk_200_random_q_vs_independent_numpy()

### Community 193 - "check_docs.py"
Cohesion: 0.25
Nodes (4): check(), main(), parse_file(), slugify()

### Community 195 - "test_viewport_in_main_window.py"
Cohesion: 0.16
Nodes (6): test_main_window_has_viewport(), test_main_window_viewport_property(), test_viewport_add_sphere(), test_viewport_import_does_not_load_vtk_at_module_level(), test_viewport_plotter_accessible(), test_viewport_reset_camera()

### Community 197 - "ZipFile"
Cohesion: 0.07
Nodes (18): salvage(), ProjectMetadata, load(), _read_collision(), _read_configs(), _read_json(), _read_trajectory(), _read_visuals() (+10 more)

### Community 199 - "ProjectData"
Cohesion: 0.04
Nodes (24): ProjectData, _check_format_version(), ProjectFormatError, ProjectSerializationError, _write_archive(), write_atomic(), A1.2 --- Load and save safety (H2, M8, LOW: recover-No, backup, configs), Code facts (checked 2026-11-13) (+16 more)

### Community 200 - "test_audit_numerics.py"
Cohesion: 0.08
Nodes (18): _limits_arrays(), finite_difference_torque(), gantry(), _locked(), _quintic(), test_low_check_trajectory_flags_non_finite(), test_low_engine_stop_resets_rk45_step(), fresh() (+10 more)

### Community 202 - "Master Plan: Stages R3 to R8"
Cohesion: 0.11
Nodes (17): 10. Summary timeline (order only, no dates), 1.1 Cycle for every stage, 1.2 Review cadence (decision, can be changed by the user), 1.3 Stage gate (all must be true before push), 1.4 Push rule, 1.5 Failure rules (unchanged, restated), 1.5a Role split in every stage, 1.6 Standing technical rules (every stage) (+9 more)

### Community 204 - "GPU batch IK: go / no-go (R5.4)"
Cohesion: 0.29
Nodes (7): GPU batch IK: go / no-go (R5.4), Machine and versions, Numbers (1e5 targets, median of 5 runs after warm-up), Prototype summary (for a later revisit), Rule (R5 spec, R5.4 item 4), Tests, Why no-go

### Community 206 - "test_urdf_meshes.py"
Cohesion: 0.15
Nodes (25): resolve_mesh(), Mesh resolution (`resolve_mesh`), _codes(), _import(), _mesh_urdf(), test_clear_removes_visual_actors(), test_dae_mesh_warning_and_fallback(), test_import_package_mesh_catkin_tree() (+17 more)

### Community 207 - "robot_model.hpp"
Cohesion: 0.04
Nodes (49): PYBIND11_MODULE(), bind_dyn(), to_vec3(), bind_urm(), DHParams, a, alpha, d (+41 more)

### Community 209 - "Documentation index"
Cohesion: 0.22
Nodes (9): Data formats, Decisions and audits, Documentation index, Guides, Import, Kinematics and dynamics, Simulation, graphs and planning, Stage specifications (+1 more)

### Community 211 - "test_ui_layout.py"
Cohesion: 0.32
Nodes (15): _close(), isolated_settings(), _make(), test_click_raises_tabbed_dock(), test_close_in_focus_mode_saves_normal_layout(), test_current_layout_version_is_restored(), test_default_viewport_is_large(), test_focus_action_toggle_and_shortcut() (+7 more)

### Community 212 - "load_robot"
Cohesion: 0.19
Nodes (7): load_robot(), studies(), test_dock_cancel(), test_dock_runs_comparison_and_exports(), test_gravity_torque_and_payload(), test_reach_is_independent_check(), _wait()

### Community 213 - "test_milestone_r2_end_to_end"
Cohesion: 0.15
Nodes (4): test_milestone_r2_end_to_end(), test_main_window_has_log_panel(), test_main_window_log_panel_accepts_messages(), test_main_window_integration_and_fk_failure_path()

### Community 219 - "json"
Cohesion: 0.07
Nodes (20): _check_meta(), _check_schema(), diagnose(), DiagnosisReport, Issue, is_newer(), is_older(), _m_1_0_to_1_1() (+12 more)

### Community 220 - "Compiled model layout (frozen, layout_version 1)"
Cohesion: 0.40
Nodes (5): Compiled model layout (frozen, layout_version 1), Enum values, Example: 2-joint planar arm, Fields, Order and meaning

### Community 222 - "cartesian.py"
Cohesion: 0.18
Nodes (9): arc(), CartesianPath, line(), rot_exp(), rot_log(), _s_values(), _skew(), _solve() (+1 more)

### Community 223 - "Data formats"
Cohesion: 0.40
Nodes (5): Data formats, Plugin manifest, Project file (`.robotproj`), Robot model (URM), Simulation and graph exports

### Community 230 - "ViolationReport"
Cohesion: 0.15
Nodes (11): ViolationReport, API, Numbers (planar 2R, one box obstacle, Ryzen 9 8940HX), Path planning (R7.4), Time parametrisation, Cancel latency (U2.2), Checks (`checks.py`), Decisions (+3 more)

### Community 235 - "test_pdf_report_runs_on_gui_thread"
Cohesion: 0.29
Nodes (3): test_pdf_report_runs_on_gui_thread(), work(), test_pdf_report_without_app_off_main_thread_raises()

### Community 236 - "Dynamics validation"
Cohesion: 0.29
Nodes (6): Dynamics validation, How to run, Pinocchio (optional), Results, Setup notes, What is compared

### Community 237 - "Collision library decision (R7.2)"
Cohesion: 0.33
Nodes (5): Collision library decision (R7.2), Decision, Project format, Proof build (one attempt, time-boxed), Speed (R7.2, `check_batch`, CPU, Python loop)

### Community 239 - ".set_store"
Cohesion: 0.13
Nodes (11): Code facts (checked 2026-11-13), Common rules, F1.1 --- Pinocchio validator: proof build --- STOP after (report result to user), F1.2 --- Pinocchio validator: tool, tests, report, F1.4 --- Save the shown ghost set, F1.5 --- F1 milestone and stage gate, Not in F1 (decided OUT, 2026-11-13), Stage F1 spec pack: Feature completion (academic version) (+3 more)

### Community 240 - "test_u2_floor_drop.py"
Cohesion: 0.25
Nodes (6): _drop(), _load(), test_drop_other_ignored(), test_drop_robotproj(), test_drop_urdf(), test_floor_exists_toggle_rebuild()

### Community 243 - "planar_2r"
Cohesion: 0.13
Nodes (12): volume_from_occupancy(), test_cancel_and_progress(), test_volume_helper_matches_r5_occupancy(), annulus_voxel_count(), _check_annulus(), planar_2r(), test_bad_inputs_are_value_error_cpu(), test_bad_limits_and_cancel_type() (+4 more)

### Community 245 - "clean-machine-test.md"
Cohesion: 0.67
Nodes (3): Clean machine test (R8.5), Done (dev machine, scrubbed environment), Left for the user

### Community 246 - "Hardware Audit — R0.1"
Cohesion: 0.25
Nodes (8): Build Tools, Compiler, CUDA Toolkit, GPU, Hardware Audit — R0.1, Notes, Pinocchio Decision (R0.8), R0 Final Tool Versions

### Community 248 - "test_urm_bridge.py"
Cohesion: 0.04
Nodes (23): test_default_limits_callable(), test_dh_convention_enum(), test_dh_params_readwrite(), test_from_json_bad_json_raises_runtime_error(), test_joint_creation_and_schema_v2_fields(), test_joint_kinematics_enum(), test_joint_type_enum(), test_link_creation() (+15 more)

### Community 252 - "AppSettings"
Cohesion: 0.04
Nodes (20): AppSettings, A1.3 --- Plugin timeout and settings (M9, plugins_folder), test_preference_from_settings(), test_settings_persist(), test_empty_plugins_folder_setting_uses_default(), test_settings_default_folder(), test_settings(), clean_settings() (+12 more)

### Community 255 - "generate_report"
Cohesion: 0.06
Nodes (31): Analysis and export, Cancel, `cancel_scope`, `CancelToken`, `CancelToken.cancel`, `CancelToken.is_cancelled`, Examples, `export_csv` (+23 more)

### Community 267 - "test_sample_q_matches_independent_numpy_philox"
Cohesion: 0.40
Nodes (3): philox_np(), q_numpy(), test_sample_q_matches_independent_numpy_philox()

### Community 270 - "MainWindow"
Cohesion: 0.03
Nodes (42): MainWindow, _no_modal_unsaved_prompt(), win(), win(), win(), test_ui_export_actions(), win(), win() (+34 more)

### Community 274 - ".set_collision_shapes"
Cohesion: 0.29
Nodes (5): Motion, `plan`, `Robot.set_collision_shapes`, `simulate`, `time_parametrize`

## Knowledge Gaps
- **314 isolated node(s):** `axis`, `child_link`, `mult`, `offset`, `parent_link` (+309 more)
  These have ≤1 connection - possible missing edges or undocumented components. (Counts symbols only; 2400 node(s) total have ≤1 connection when file, concept and rationale nodes are included.)
- **83 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `MainWindow` connect `MainWindow` to `DynModel`, `IKPanel`, `._update_robot_view`, `._setup_dock_panels`, `test_ui_kin_dyn.py`, `CollisionPanel`, `._apply_default_sizes`, `AppSettings`, `.log`, `test_workspace_analysis.py`, `TcpDragController`, `PhysicalPropertiesPanel`, `test_teach.py`, `test_milestone_r3.py`, `TaskRunner`, `PerformancePanel`, `unittest_mock`, `GraphPanel`, `robots.py`, `test_viewport_robot.py`, `Stage R8 spec pack: Extensibility and release`, `test_milestone_r8.py`, `test_performance.py`, `QDockWidget`, `Stage R7 spec pack: Planning and advanced analysis`, `SimPanel`, `JointPanel`, `test_ui_ghosts.py`, `ComparePanel`, `test_recorder.py`, `test_ui_script.py`, `test_export.py`, `GhostPanel`, `main_window.py`, `test_viewport_in_main_window.py`, `._on_project_opened`, `.restore_window`, `ProjectData`, `test_import_report.py`, `test_urdf_meshes.py`, `test_plugins.py`, `test_milestone_r4.py`, `test_ui_layout.py`, `load_robot`, `test_milestone_r2_end_to_end`, `test_milestone_r5.py`, `AnalysisPanel`, `test_ui_trajectory.py`, `Any`, `test_ui_workspace.py`, `PluginManager`, `._setup_central_widget`, `.set_target_pose`, `test_milestone_r6.py`, `test_sim_panel.py`, `save`, `test_gripper.py`, `test_u2_floor_drop.py`, `current_q`, `AppSettings`, `test_report.py`, `ModelPanel`, `test_physical_props.py`, `ImportReportPanel`, `MeshPathsDialog`, `test_u2_fix.py`?**
  _High betweenness centrality (0.125) - this node is a cross-community bridge._
- **Are the 31 inferred relationships involving `MainWindow` (e.g. with `ComparePanel` and `SimPanel`) actually correct?**
  _`MainWindow` has 31 INFERRED edges - model-reasoned connections that need verification._
- **What connects `axis`, `child_link`, `mult` to the rest of the system?**
  _314 weakly-connected nodes found - possible documentation gaps or missing edges._
- **Should `DynModel` be split into smaller, more focused modules?**
  _Cohesion score 0.06621004566210045 - nodes in this community are weakly interconnected._
- **Why does `GraphPanel` connect `GraphPanel` to `main_window.py`, `SimPanel`, `._setup_dock_panels`, `MainWindow`?**
  _High betweenness centrality (0.041) - this node is a cross-community bridge._
- **Are the 4 inferred relationships involving `random_robot()` (e.g. with `Code facts (checked 2026-11-11, after R4)` and `R5.2 --- Batch FK kernel + precision decision (C++/CUDA + bindings)`) actually correct?**
  _`random_robot()` has 4 INFERRED edges - model-reasoned connections that need verification._
- **Should `IKPanel` be split into smaller, more focused modules?**
  _Cohesion score 0.0596078431372549 - nodes in this community are weakly interconnected._