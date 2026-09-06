from __future__ import annotations

from pathlib import Path

import json

import pytest

from validate_formal_dynamic_obstacle_avoidance import (
    PASSED_STATUS,
    attach_evaluator_dynamic_proximity,
    evaluate,
    frozen_session_binding,
    load_public_mission_contract,
    point_in_polygon,
)


def _telemetry() -> dict:
    return {
        "vehicle_profile": "formal_transport_stowed",
        "runtime_build_manifest": {
            "current_source_build_completed": True,
            "source_install_bindings": [{"matches": True}] * 11,
            "source_only_runtime_files": [
                {"source": source, "source_sha256": "c" * 64}
                for source in (
                    "starter_ws/src/sanitation_formal_campus_integration/setup.py",
                    "scripts/collect_formal_map_lifecycle_runtime.py",
                    "scripts/run_formal_dynamic_obstacle_avoidance.sh",
                    "scripts/collect_formal_dynamic_obstacle_avoidance_runtime.py",
                    "scripts/collect_formal_dynamic_environment_runtime.py",
                    "scripts/validate_formal_dynamic_obstacle_avoidance.py",
                    "scripts/generate_formal_dynamic_runtime_build_manifest.py",
                    "scripts/prepare_formal_dynamic_obstacle_schedule.py",
                    "scripts/prepare_formal_dynamic_runtime_world.py",
                )
            ],
            "required_plugin_libraries": {"a": True},
        },
        "runtime_world_manifest": {
            "source_world_sha256": "a" * 64,
            "runtime_world_sha256": "b" * 64,
            "world_preserved_except_contact_system": True,
            "contact_system_plugin_count": 1,
            "pedestrian_model_count": 8,
            "pedestrian_model_ids": [f"walker_{index}" for index in range(8)],
        },
        "command_chain": [
            "/cmd_vel_nav",
            "/cmd_vel_smoothed",
            "/cmd_vel_gate",
            "/base_controller/cmd_vel",
        ],
        "active_pedestrian_count": 8,
        "expected_pedestrian_count": 8,
        "dynamic_environment_contract": {
            "seed": 81422,
            "randomized_each_run_unless_seed_pinned": True,
            "mission_corridor_crossing_count": 3,
            "pedestrian_model_ids": [f"walker_{index}" for index in range(8)],
            "product_control_access_prohibited": True,
        },
        "mission_goal_source": (
            "public_manifest_fixed_start_transformed_to_saved_map_local_plus_nominal_leg"
        ),
        "source_fixed_start_pose": [-98.0, 0.0, 0.0],
        "product_control_reads_pedestrian_truth": False,
        "evaluator_truth_process_isolated": True,
        "pedestrian_velocity_estimation_used": False,
        "collision_monitor_node_count": 1,
        "final_command_publisher_count": 1,
        "final_command_publisher_node": "/whole_vehicle_safety_manager",
        "command_topic_publishers": {
            "/cmd_vel_gate": ["/collision_monitor"],
        },
        "safety_permit_sample_count": 100,
        "mission_safety_sample_count": 100,
        "mission_safety_inhibit_sample_count": 0,
        "bms_fault_clear_sample_count": 100,
        "traction_permitted_sample_count": 100,
        "runtime_node_graph": [
            "/a300_bms_simulator",
            "/a300_drivetrain_command_adapter",
            "/whole_vehicle_safety_manager",
        ],
        "selected_odom_publishers": ["/local_ekf"],
        "raw_odom_publishers": ["/a300_drivetrain_bridge"],
        "topic_sample_counts": {
            "/odom": 100,
            "/odom/unfiltered": 100,
            "/tf:odom->base_footprint": 100,
            "/amcl_pose": 100,
            "/scan/navigation": 100,
            "/sensors/lidar_3d/points": 100,
            "/cmd_vel_nav": 100,
            "/cmd_vel_smoothed": 100,
            "/cmd_vel_gate": 100,
            "/base_controller/cmd_vel": 100,
            "/collision_monitor_state": 100,
            "/safety/front_bumper/contact": 100,
            "/safety/rear_bumper/contact": 100,
            "/safety/status": 100,
            "/scenario/environment/pedestrian_driver/status": 100,
            "/model/tzcup_formal_sanitation_vehicle/a300_drivetrain/status": 100,
        },
        "nav2_goal_succeeded": True,
        "mission_metrics_begin_at_goal_submission": True,
        "physical_travel_distance_m": 12.0,
        "mission_odom_trajectory_xy_m": [[0.0, 0.0], [3.0, 0.0], [6.0, 0.0], [9.0, 0.0], [12.0, 0.0]],
        "mission_map_trajectory_xy_m": [[0.0, 0.0], [3.0, 0.3], [6.0, 1.2], [9.0, 0.4], [12.0, 0.0]],
        "maximum_cross_track_detour_m": 1.2,
        "verified_dynamic_interaction_count": 1,
        "evaluator_verified_dynamic_interaction_count": 1,
        "collision_monitor_intervention_count": 2,
        "collision_count": 0,
        "geofence_violation_count": 0,
        "map_pose_sample_count": 5,
        "odom_pose_sample_count": 5,
        "control_truth_topics_subscribed": [],
        "control_prohibited_truth_topic_subscriber_audit": {
            "/scenario/environment/pedestrian_driver/status": [
                "/formal_dynamic_environment_truth_collector"
            ]
        },
    }


def test_complete_runtime_evidence_can_pass() -> None:
    report = evaluate(_telemetry(), saved_map_valid=True)
    assert report["status"] == PASSED_STATUS
    assert report["passed"] is True


def test_missing_saved_map_or_dynamic_interaction_fails_closed() -> None:
    telemetry = _telemetry()
    telemetry["evaluator_verified_dynamic_interaction_count"] = 0
    report = evaluate(
        telemetry,
        saved_map_valid=False,
        saved_map_error="map_lifecycle_manifest.json missing",
    )
    assert report["passed"] is False
    assert "saved_map_lifecycle_artifact_valid" in report["blockers"]
    assert "dynamic_interaction_observed" in report["blockers"]


def test_missing_frozen_session_fails_closed() -> None:
    report = evaluate(
        _telemetry(),
        saved_map_valid=True,
        frozen_session_valid=False,
    )
    assert report["passed"] is False
    assert "frozen_snapshot_session_valid" in report["blockers"]


def test_missing_or_stale_bms_evidence_fails_closed() -> None:
    telemetry = _telemetry()
    telemetry["bms_fault_clear_sample_count"] = 0
    telemetry["traction_permitted_sample_count"] = 0
    telemetry["runtime_node_graph"] = ["/whole_vehicle_safety_manager"]
    report = evaluate(telemetry, saved_map_valid=True)
    assert report["passed"] is False
    assert "bms_fault_feed_fresh_and_clear" in report["blockers"]
    assert "traction_permit_feed_fresh_and_true" in report["blockers"]
    assert "a300_bms_node_observed" in report["blockers"]


def test_any_mission_safety_inhibit_fails_continuous_permit_gate() -> None:
    telemetry = _telemetry()
    telemetry["mission_safety_inhibit_sample_count"] = 1
    report = evaluate(telemetry, saved_map_valid=True)
    assert report["passed"] is False
    assert "safety_manager_continuously_permitted" in report["blockers"]


def test_reported_mileage_must_recompute_from_recorded_odom_trajectory() -> None:
    telemetry = _telemetry()
    telemetry["physical_travel_distance_m"] = 11.0
    report = evaluate(telemetry, saved_map_valid=True)
    assert report["passed"] is False
    assert (
        "physical_mileage_recomputes_from_recorded_odom_trajectory"
        in report["blockers"]
    )


def test_runner_never_bypasses_saved_map_or_writes_direct_base_command() -> None:
    source = (
        Path(__file__).with_name("run_formal_dynamic_obstacle_avoidance.sh")
        .read_text(encoding="utf-8")
    )
    assert "--preflight-only" in source
    assert "validate_formal_dynamic_obstacle_avoidance.py" in source
    assert "/base_controller/cmd_vel" not in source
    assert "FORMAL_DYNAMIC_SAVED_MAP_ROOT" in source
    assert "formal_campus_map_lifecycle.launch.py" in source
    assert "collect_formal_dynamic_obstacle_avoidance_runtime.py" in source
    assert "prepare_formal_dynamic_obstacle_schedule.py" in source
    assert "prepare_formal_dynamic_runtime_world.py" in source
    assert "generate_formal_dynamic_runtime_build_manifest.py" in source
    assert "FORMAL_DYNAMIC_BUILD_CURRENT" not in source
    assert "colcon --log-base" not in source
    assert 'rm -f "${output}" "${telemetry}"' not in source
    assert "pedestrian_schedule:=\"${runtime_schedule}\"" in source
    assert "world:=\"${runtime_world}\"" in source


def test_runner_uses_one_final_overlay_legal_domain_and_fresh_outputs() -> None:
    source = Path(__file__).with_name(
        "run_formal_dynamic_obstacle_avoidance.sh"
    ).read_text(encoding="utf-8")
    assert "FORMAL_VEHICLE_RUNTIME_WS:?" in source
    assert 'runtime_install="${runtime_ws}/install"' in source
    assert 'source "${runtime_install}/setup.bash"' in source
    assert "project package resolved outside the one frozen overlay" in source
    assert 'realpath "${runtime_install}"' in source
    assert "stage1_20260826_023716" not in source
    assert "tzcup_integrated_build_20260826_v3" not in source
    assert ".work/formal_dynamic_build/install" not in source
    assert "FORMAL_DYNAMIC_BUILD_CURRENT" not in source
    assert "colcon --log-base" not in source
    assert 'domain="${ROS_DOMAIN_ID:-73}"' in source
    assert 'formal_runtime_configure "${domain}"' in source
    assert "Refusing stale dynamic-obstacle evidence" in source
    assert "session_started_epoch_ns" in source
    assert "snapshot_preflight.json" in source
    assert "snapshot_postflight.json" in source
    assert "formal_runtime_gate_binding.py" in source
    assert "FORMAL_FINAL_RUNTIME_CLOSURE_MANIFEST" in source
    assert 'runtime_binding="${FORMAL_DYNAMIC_RUNTIME_BINDING:-${output}.runtime_binding.json}"' in source


def test_runner_disables_ros2_daemon_and_proves_partition_cleanup() -> None:
    source = Path(__file__).with_name(
        "run_formal_dynamic_obstacle_avoidance.sh"
    ).read_text(encoding="utf-8")
    helper = Path(__file__).with_name("run_formal_runtime_isolation.sh").read_text(
        encoding="utf-8"
    )
    assert "formal_runtime_install_traps cleanup" in source
    assert "formal_runtime_cleanup_groups" in source
    assert "export ROS2CLI_DISABLE_DAEMON=1" in helper
    assert 'needle = ("GZ_PARTITION=" + sys.argv[1]).encode()' in helper
    assert "signal.SIGINT, signal.SIGTERM, signal.SIGKILL" in helper
    assert 'excluded = {os.getpid(), int(sys.argv[2])}' in helper
    assert "runtime_partition_cleanup_complete" in source
    assert "FORMAL_DYNAMIC_OBSTACLE_AVOIDANCE_ACCEPTANCE_BLOCKED" in source
    assert "trap 'exit 130' INT" in helper
    assert "trap 'exit 143' TERM" in helper
    assert "FORMAL_GAZEBO_LOCK_FILE:-/tmp/tzcup_formal_gazebo.lock" in helper
    assert "flock -n 9" in helper
    assert "run the matrix serially" in helper


def test_frozen_session_binding_requires_exact_running_snapshot(tmp_path: Path) -> None:
    snapshot = tmp_path / "snapshot.json"
    session = tmp_path / "session.json"
    snapshot.write_text(
        json.dumps(
            {
                "source_inventory_sha256": "a" * 64,
                "outputs": {
                    "reports/engineering/formal_competition_vehicle.urdf": {
                        "sha256": "b" * 64
                    }
                },
            }
        ),
        encoding="utf-8",
    )
    import hashlib

    identity = {
        "snapshot_manifest_sha256": hashlib.sha256(snapshot.read_bytes()).hexdigest(),
        "source_inventory_sha256": "a" * 64,
        "expanded_urdf_sha256": "b" * 64,
    }
    session.write_text(
        json.dumps(
            {
                "status": "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING",
                "started_epoch_ns": 42,
                "snapshot": identity,
            }
        ),
        encoding="utf-8",
    )
    binding = frozen_session_binding(snapshot, session)
    assert binding["session_running"] is True
    assert binding["session_started_epoch_ns"] == 42
    assert binding["expanded_urdf_sha256"] == "b" * 64

    bad = json.loads(session.read_text(encoding="utf-8"))
    bad["snapshot"]["source_inventory_sha256"] = "c" * 64
    session.write_text(json.dumps(bad), encoding="utf-8")
    try:
        frozen_session_binding(snapshot, session)
    except ValueError as exc:
        assert "another snapshot" in str(exc)
    else:
        raise AssertionError("mismatched snapshot session must fail closed")


def test_dynamic_gate_contract_binds_both_snapshot_hashes() -> None:
    contract = (
        Path(__file__).parents[1]
        / "config/high_fidelity_vehicle/formal_functional_acceptance_contract.yaml"
    ).read_text(encoding="utf-8")
    dynamic = contract.split("  dynamic_obstacle_avoidance:", 1)[1].split(
        "  rl_cross_map_policy:", 1
    )[0]
    assert "session_bound: true" in dynamic
    assert "snapshot_urdf_hash_field: source_binding.expanded_urdf_sha256" in dynamic
    assert (
        "snapshot_source_hash_field: source_binding.source_inventory_sha256"
        in dynamic
    )
    assert "runtime_binding:" in dynamic
    assert "report_field: runtime_gate_binding" in dynamic


def test_runtime_collector_uses_scan_gate_and_not_pedestrian_truth() -> None:
    source = (
        Path(__file__).with_name(
            "collect_formal_dynamic_obstacle_avoidance_runtime.py"
        )
        .read_text(encoding="utf-8")
    )
    assert '"/navigate_to_pose"' in source
    assert '"/collision_monitor_state"' in source
    assert '"/scan/navigation"' in source
    assert '"/sensors/lidar_3d/points"' in source
    assert '"/odom"' in source
    assert '"/amcl_pose"' in source
    assert '"/safety/status"' in source
    assert "pedestrian_schedule" not in source
    assert '"/scenario/environment/pedestrian_driver/status"' not in source
    assert '"/safety/front_bumper/contact"' not in source
    assert "ros_gz_interfaces.msg" not in source
    assert "velocity_estimation_used\": False" in source
    tick = source.split("def _tick", 1)[1].split("def _goal_response", 1)[0]
    assert "active_pedestrian_count" not in tick
    monitor = source.split("def _monitor", 1)[1].split("def _front_contact", 1)[0]
    assert "active_pedestrian_count" not in monitor


def test_runtime_builds_and_audits_the_bms_that_gates_vehicle_safety() -> None:
    runner = Path(__file__).with_name(
        "run_formal_dynamic_obstacle_avoidance.sh"
    ).read_text(encoding="utf-8")
    manifest = Path(__file__).with_name(
        "generate_formal_dynamic_runtime_build_manifest.py"
    ).read_text(encoding="utf-8")
    assert "sanitation_power_system" in runner
    assert "a300_bms_core.py" in manifest
    assert "a300_bms_node.py" in manifest
    assert "a300_40ah_bms.yaml" in manifest


def test_public_default_goal_uses_start_pose_and_real_geofence(tmp_path: Path) -> None:
    manifest = tmp_path / "episode_manifest.json"
    manifest.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "episode_id": "mission",
                "map_id": "map",
                "vehicle_start_pose_map": {
                    "x_m": -98.0,
                    "y_m": 0.0,
                    "yaw_rad": 0.0,
                },
                "field": {
                    "geofence_frame": "map",
                    "geofence_polygon_m": [
                        [-100.0, -50.0],
                        [100.0, -50.0],
                        [100.0, 50.0],
                        [-100.0, 50.0],
                    ]
                },
                "counts": {"pedestrians": 8},
            }
        ),
        encoding="utf-8",
    )
    contract = load_public_mission_contract(manifest)
    assert contract["start_pose_map"] == [0.0, 0.0, 0.0]
    assert contract["goal_pose_map"][:2] == [30.0, 0.0]
    assert contract["goal_pose_source_world"][:2] == [-68.0, 0.0]
    assert contract["geofence_polygon_m"] == [
        [-2.0, -50.0],
        [198.0, -50.0],
        [198.0, 50.0],
        [-2.0, 50.0],
    ]
    assert contract["expected_pedestrian_count"] == 8
    diagnostic = load_public_mission_contract(
        manifest, goal_x=10.0, goal_y=2.0
    )
    assert diagnostic["goal_source"] == "explicit_map_goal_diagnostic_only"


def test_public_explicit_geofence_rejects_repeated_transform(tmp_path: Path) -> None:
    manifest = tmp_path / "episode_manifest.json"
    source = [[-100.0, -50.0], [100.0, -50.0], [100.0, 50.0], [-100.0, 50.0]]
    manifest.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "episode_id": "mission",
                "map_id": "map",
                "vehicle_start_pose_source_world": {"x_m": -98.0, "y_m": 0.0, "yaw_rad": 0.0},
                "vehicle_start_pose_map": {"x_m": -98.0, "y_m": 0.0, "yaw_rad": 0.0},
                "field": {
                    "geofence_frame": "source_world",
                    "geofence_polygon_m": source,
                    "source_world_geofence": {"frame_id": "source_world", "polygon_m": source},
                    "localization_map_geofence": {"frame_id": "map", "polygon_m": source},
                },
                "counts": {"pedestrians": 8},
            }
        ),
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="exactly once"):
        load_public_mission_contract(manifest)


def test_point_in_polygon_includes_boundary_and_rejects_old_wrong_frame() -> None:
    polygon = [(-2.0, -50.0), (198.0, -50.0), (198.0, 50.0), (-2.0, 50.0)]
    assert point_in_polygon((0.0, 0.0), polygon, boundary_is_inside=True)
    assert point_in_polygon((198.0, 0.0), polygon, boundary_is_inside=True)
    assert not point_in_polygon((199.0, 0.0), polygon, boundary_is_inside=True)


def test_evaluator_verifies_pedestrian_proximity_only_after_run(tmp_path: Path) -> None:
    schedule = tmp_path / "pedestrians.json"
    schedule.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "access": "environment_driver_only_not_robot_control",
                "world_name": "campus_formal",
                "loop": True,
                "pedestrians": [
                    {
                        "object_id": f"walker_{index}",
                        "radius_m": 0.25,
                        "waypoints": [
                            [0.0, 1.0 + 10.0 * index, -1.0],
                            [2.0, 1.0 + 10.0 * index, 1.0],
                            [4.0, 1.0 + 10.0 * index, -1.0],
                        ],
                    }
                    for index in range(8)
                ],
            }
        ),
        encoding="utf-8",
    )
    telemetry = {
        "source_fixed_start_pose": [0.0, 0.0, 0.0],
        "dynamic_interaction_candidates": [
            {
                "observation_ros_time_ns": 1_000_000_000,
                "vehicle_pose_map": [0.0, 0.0],
                "nearest_scan_range_m": 0.75,
            }
        ]
    }
    environment = {
        "collector_role": "evaluator_only_no_robot_control",
        "active_pedestrian_count": 8,
        "pedestrian_status_samples": [
            {
                "observation_ros_time_ns": 1_000_000_000,
                "schedule_elapsed_s": 1.0,
                "pedestrian_count": 8,
            }
        ],
        "collision_count": 0,
        "topic_sample_counts": {
            "/scenario/environment/pedestrian_driver/status": 1,
            "/safety/front_bumper/contact": 1,
            "/safety/rear_bumper/contact": 1,
        },
        "control_topics_published": [],
        "product_actions_created": [],
    }
    attach_evaluator_dynamic_proximity(telemetry, schedule, environment)
    assert telemetry["evaluator_verified_dynamic_interaction_count"] == 1
    assert telemetry["minimum_evaluator_pedestrian_clearance_m"] == 0.75
    assert telemetry["evaluator_truth_process_isolated"] is True


def test_environment_truth_collector_cannot_control_the_product() -> None:
    source = (
        Path(__file__).with_name(
            "collect_formal_dynamic_environment_runtime.py"
        )
        .read_text(encoding="utf-8")
    )
    assert '"/scenario/environment/pedestrian_driver/status"' in source
    assert '"/safety/front_bumper/contact"' in source
    assert "NavigateToPose" not in source
    assert "ActionClient" not in source
    assert "create_publisher" not in source
    assert '"control_topics_published": []' in source
