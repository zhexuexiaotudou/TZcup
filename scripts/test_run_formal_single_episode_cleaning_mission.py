from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
RUNNER = (ROOT / "scripts/run_formal_single_episode_cleaning_mission.sh").read_text(encoding="utf-8")
COLLECTOR = (ROOT / "scripts/collect_formal_single_episode_cleaning_mission.py").read_text(encoding="utf-8")


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
    assert 'rows == ["/formal_single_episode_cleaning_collector"]' in COLLECTOR
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
    ):
        assert token in COLLECTOR
