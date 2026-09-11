from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
RUNNER = (ROOT / "scripts/run_formal_single_episode_cleaning_mission.sh").read_text(encoding="utf-8")
COLLECTOR = (ROOT / "scripts/collect_formal_single_episode_cleaning_mission.py").read_text(encoding="utf-8")
PRODUCT_LAUNCH = (
    ROOT / "starter_ws/src/sanitation_product_demo_integration/launch/product_demo.launch.py"
).read_text(encoding="utf-8")
HIGH_FIDELITY_CLEANING_XACRO = (
    ROOT / "starter_ws/src/sanitation_vehicle_description/urdf/high_fidelity/cleaning_mechanism.xacro"
).read_text(encoding="utf-8")
FORMAL_VEHICLE_LAUNCH = (
    ROOT / "starter_ws/src/sanitation_vehicle_description/launch/formal_vehicle_sim.launch.py"
).read_text(encoding="utf-8")
FORMAL_VEHICLE_XACRO = (
    ROOT / "starter_ws/src/sanitation_vehicle_description/urdf/formal_competition_vehicle.urdf.xacro"
).read_text(encoding="utf-8")
FORMAL_CAMPUS_LAUNCH = (
    ROOT / "starter_ws/src/sanitation_formal_campus_integration/launch/formal_campus.launch.py"
).read_text(encoding="utf-8")
GROUND_TRUTH_ADAPTER = (
    ROOT / "starter_ws/src/sanitation_tasks/sanitation_tasks/ground_truth_adapter.py"
).read_text(encoding="utf-8")
SAFETY_MANAGER = (
    ROOT / "starter_ws/src/sanitation_safety/sanitation_safety/whole_vehicle_safety_manager.py"
).read_text(encoding="utf-8")
CLEANING_ACTUATOR_MIRROR = (
    ROOT / "starter_ws/src/sanitation_gazebo_control/scripts/cleaning_actuator_command_mirror.py"
).read_text(encoding="utf-8")


def test_runner_owns_exactly_one_product_gazebo_launch() -> None:
    assert RUNNER.count("ros2 launch sanitation_product_demo_integration product_demo.launch.py") == 1
    assert "run_formal_ground_dirt_cleaning_runtime.sh" not in RUNNER
    assert "run_formal_grasp_executor_runtime.sh" not in RUNNER
    assert "run_formal_water_recovery_runtime.sh" not in RUNNER
    assert "run_formal_dynamic_obstacle_avoidance.sh" not in RUNNER
    assert "gz service" not in RUNNER and "set_entity_pose" not in RUNNER


def test_evaluator_topics_are_one_way_and_not_product_commands() -> None:
    assert "/evaluation/single_episode/ground_dirt/status_json" in RUNNER
    assert "/evaluation/single_episode/water_recovery/status_json" in RUNNER
    assert "/evaluation/single_episode/dry_bin/status_json" in RUNNER
    assert "get_subscriptions_info_by_topic" in COLLECTOR
    assert "control_prohibited_truth_topic_subscribers" in COLLECTOR
    assert '"ground_truth_odom_allowed_subscribers"' in COLLECTOR
    assert '"--trusted-gt-recorder-pgid"' in COLLECTOR
    assert "create_publisher" not in COLLECTOR


def test_runner_binds_seed_session_process_and_hash_chain() -> None:
    for token in ("--session-id", "--episode-id", "--episode-seed", "--runtime-id",
                  "--gazebo-process-id", "--session-start-epoch-ns", "--evaluator-ground-truth",
                  "--evaluator-episode-manifest", "--policy-checkpoint", "--saved-map",
                  "--perception-artifacts", "--input-binding"):
        assert token in RUNNER
    assert "--prepare-input-binding" in RUNNER
    assert "collector_ready.json" in RUNNER
    assert "seeds.get('dirt')" in RUNNER
    assert "aggregate_formal_single_episode_cleaning_mission.py" in RUNNER
    assert "validate_formal_end_to_end_cleaning_mission.py" in RUNNER
    assert "generate_formal_same_map_baseline.py\" validate" in RUNNER
    assert "formal_vehicle_snapshot_manifest.json" in RUNNER
    assert "return_distance_included" in RUNNER
    assert "formal_runtime_gate_binding.py" in RUNNER
    assert "FORMAL_FINAL_RUNTIME_CLOSURE_MANIFEST" in RUNNER
    assert "--runtime-binding" in RUNNER
    assert 'RUNTIME_BINDING="${FORMAL_E2E_RUNTIME_BINDING:-${FORMAL_OUTPUT}.runtime_binding.json}"' in RUNNER
    assert "--snapshot-manifest" in RUNNER


def test_collector_records_initial_terminal_grasp_and_runtime_bindings() -> None:
    for token in (
        '"initial": self.first', '"terminal": self.latest', '"grasp_results": self.grasp_results',
        '"runtime_parameters": self.runtime_parameters', 'directory_descriptor',
        '"trajectory_evidence": self.trajectory_evidence',
        '"planner_status_samples": self.planner_status_samples',
        '"task_odom_trajectory_xy_m"',
        '"replay_metric_capture"', '"brush_on_ground_truth_stream"',
        '"localization_pairing"', '"/brush_controller/commands"',
        '"/localization/fused_odom"', '"/ground_truth/odom"',
        'synchronized_xy_errors', 'JsonlMetricStream',
        'header_stamp_window_filter', 'no_overwrite_jsonl_streams_with_rolling_sha256',
    ):
        assert token in COLLECTOR
    assert "--require-replay-metric-capture" in RUNNER
    assert "CONTROL_PROHIBITED_TRUTH_TOPICS" in COLLECTOR
    assert '"/safety/command/brush"' not in COLLECTOR


def test_product_launch_does_not_start_coverage_probe_and_uses_current_brush_geometry() -> None:
    assert '"start_coverage": "false"' in PRODUCT_LAUNCH
    assert "coverage_probe" not in PRODUCT_LAUNCH
    assert '<origin xyz="0.385 ${lateral_sign * 0.545} 0.0244"' in HIGH_FIDELITY_CLEANING_XACRO
    assert 'formal_model_odometry_evaluation_bridge' in FORMAL_VEHICLE_LAUNCH
    assert 'sanitation_ground_truth_adapter' in FORMAL_VEHICLE_LAUNCH
    assert '/ground_truth/model_odom_raw@nav_msgs/msg/Odometry[gz.msgs.Odometry' in FORMAL_VEHICLE_LAUNCH
    assert '/ground_truth/odom' not in PRODUCT_LAUNCH


def test_truth_chain_is_model_odom_only_and_manifest_bound() -> None:
    assert 'gz::sim::systems::OdometryPublisher' in FORMAL_VEHICLE_XACRO
    assert '<odom_topic>/ground_truth/model_odom_raw</odom_topic>' in FORMAL_VEHICLE_XACRO
    assert 'TFMessage' not in FORMAL_VEHICLE_LAUNCH
    assert 'dynamic_pose' not in FORMAL_VEHICLE_LAUNCH
    assert 'world_to_map = _world_to_map_from_source_start(manifest_start_pose)' in FORMAL_CAMPUS_LAUNCH
    assert 'episode_manifest_sha256 = _sha256_file(episode_manifest)' in FORMAL_CAMPUS_LAUNCH
    assert 'rejects spawn overrides' in FORMAL_CAMPUS_LAUNCH
    assert 'Odometry, "/ground_truth/model_odom_raw"' in GROUND_TRUTH_ADAPTER
    assert 'TFMessage' not in GROUND_TRUTH_ADAPTER
    assert 're.fullmatch(r"[0-9a-f]{64}", manifest_sha256)' in GROUND_TRUTH_ADAPTER
    assert 'stamp_ns > 0' in GROUND_TRUTH_ADAPTER
    assert 'normalized_quaternion' in GROUND_TRUTH_ADAPTER


def test_coverage_observes_only_post_safety_brush_commands() -> None:
    assert '"brush_command_input_topic", "/safety/command/brush"' in SAFETY_MANAGER
    assert '"brush_command_output_topic", "/brush_controller/commands"' in SAFETY_MANAGER
    assert '"/brush_controller/commands"' in CLEANING_ACTUATOR_MIRROR
    assert '"/safety/command/brush"' not in COLLECTOR
    assert 'full_width_coverage_active' in COLLECTOR
    assert 'canonical_localization_pairing_from_streams' in COLLECTOR


def test_truth_subscription_boundary_is_collector_plus_fixed_recorder_only() -> None:
    assert 'TRUSTED_GT_RECORDER_NODE = "/a12_trusted_gt_recorder"' in COLLECTOR
    assert 'terminal_gt_subscribers == self._allowed_gt_subscribers()' in COLLECTOR
    assert 'trusted_identity["pid_pgid_match"] is True' in COLLECTOR
    assert 'truth_subscription_audit_enabled' in COLLECTOR
    assert 'args.require_replay_metric_capture' in COLLECTOR
    assert 'RAW_GROUND_TRUTH_TOPIC = "/ground_truth/model_odom_raw"' in COLLECTOR
    assert 'RAW_GROUND_TRUTH_ADAPTER_NODE = "/formal_model_ground_truth_adapter"' in COLLECTOR
    assert 'terminal_raw_gt_subscribers == [RAW_GROUND_TRUTH_ADAPTER_NODE]' in COLLECTOR


def test_runner_attests_the_only_trusted_gt_recorder_before_operator_start() -> None:
    for token in (
        'A12_TRUSTED_GT_RECORDER_NODE="/a12_trusted_gt_recorder"',
        'formal_a12_single_execution_capture.py',
        '--bag-output "a12_execution.mcap"',
        '--video-ready-file "${A12_VIDEO_WORKER_READY}"',
        '--trusted-gt-recorder-node "${A12_TRUSTED_GT_RECORDER_NODE}"',
        '--trusted-gt-recorder-pid "${A12_RECORDER_IDENTITY[0]}"',
        '--trusted-gt-recorder-pgid "${A12_RECORDER_IDENTITY[1]}"',
        'A12_CAPTURE_SUPERVISOR_READY',
        'A12_CAPTURE_SUPERVISOR_BLOCKED',
    ):
        assert token in RUNNER
    assert RUNNER.index("a12_video_worker_ready.json") < RUNNER.index("ros2 topic pub --once /product_demo/operator_start")
    assert RUNNER.index('wait "${A12_SUPERVISOR_PID}"') < RUNNER.index("aggregate_formal_single_episode_cleaning_mission.py")
