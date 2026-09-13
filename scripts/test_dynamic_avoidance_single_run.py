from __future__ import annotations

import json
from pathlib import Path

from evaluate_dynamic_avoidance_single_run import (
    SINGLE_RUN_FAIL,
    SINGLE_RUN_INVALID,
    SINGLE_RUN_PASS,
    evaluate_run,
)
from prepare_dynamic_avoidance_single_run import (
    _sha256,
    freeze_route_from_schedule,
    validate_protocol,
)


ROOT = Path(__file__).resolve().parents[1]
PROTOCOL_PATH = ROOT / "config/dynamic_avoidance_single_run_protocol.json"


def _write_json(path: Path, value: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )


def _episode_manifest(tmp_path: Path) -> Path:
    path = tmp_path / "episode_manifest.json"
    _write_json(
        path,
        {
            "schema_version": 1,
            "episode_id": "single-run-fixture",
            "map_id": "fixture-map",
            "vehicle_start_pose_source_world": {
                "x_m": 0.0,
                "y_m": 0.0,
                "yaw_rad": 0.0,
            },
            "field": {
                "source_world_geofence": {
                    "frame_id": "source_world",
                    "polygon_m": [
                        [-10.0, -20.0],
                        [50.0, -20.0],
                        [50.0, 20.0],
                        [-10.0, 20.0],
                    ],
                },
                "localization_map_geofence": {
                    "frame_id": "map",
                    "polygon_m": [
                        [-10.0, -20.0],
                        [50.0, -20.0],
                        [50.0, 20.0],
                        [-10.0, 20.0],
                    ],
                },
            },
            "counts": {"pedestrians": 8},
        },
    )
    return path


def _schedule() -> dict:
    pedestrians = []
    for index in range(8):
        if index == 0:
            waypoints = [
                [0.0, 12.0, -4.0],
                [2.0, 12.0, 0.0],
                [4.0, 12.0, 4.0],
                [8.0, 12.0, -4.0],
            ]
        else:
            y = 20.0 + index
            waypoints = [
                [0.0, 30.0, y],
                [2.0, 31.0, y],
                [4.0, 30.0, y],
            ]
        pedestrians.append(
            {
                "object_id": f"walker_{index}",
                "radius_m": 0.25,
                "height_m": 1.7,
                "speed_mps": 0.7,
                "waypoints": waypoints,
            }
        )
    return {
        "schema_version": 1,
        "access": "environment_driver_only_not_robot_control",
        "world_name": "fixture_world",
        "loop": True,
        "pedestrians": pedestrians,
        "acceptance_environment": {
            "schema_version": 1,
            "seed": 2026091401,
            "randomized_each_run_unless_seed_pinned": True,
            "mission_corridor_crossing_count": 3,
            "mission_corridor_crossing_ids": [
                "walker_0",
                "walker_1",
                "walker_2",
            ],
            "pedestrian_model_ids": [f"walker_{index}" for index in range(8)],
            "product_control_access_prohibited": True,
        },
    }


def _product_telemetry() -> dict:
    topic_counts = {
        topic: 10
        for topic in (
            "/odom",
            "/odom/unfiltered",
            "/tf:odom->base_footprint",
            "/amcl_pose",
            "/scan/navigation",
            "/sensors/lidar_3d/points",
            "/cmd_vel_nav",
            "/cmd_vel_smoothed",
            "/cmd_vel_gate",
            "/base_controller/cmd_vel",
            "/collision_monitor_state",
            "/safety/status",
        )
    }
    return {
        "command_chain": [
            "/cmd_vel_nav",
            "/cmd_vel_smoothed",
            "/cmd_vel_gate",
            "/base_controller/cmd_vel",
        ],
        "collision_monitor_node_count": 1,
        "command_topic_publishers": {"/cmd_vel_gate": ["/collision_monitor"]},
        "topic_sample_counts": topic_counts,
        "odom_pose_sample_count": 5,
        "map_pose_sample_count": 5,
        "dynamic_interaction_candidates": [
            {
                "observation_ros_time_ns": 1_000_000_000,
                "vehicle_pose_map": [10.0, 0.0],
                "nearest_scan_range_m": 1.5,
            }
        ],
        "maximum_cross_track_detour_m": 0.6,
        "collision_monitor_intervention_count": 1,
        "goal_accepted": True,
        "nav2_goal_succeeded": True,
        "completion_reason": "action_result",
        "feedback_sample_count": 2,
        "mission_goal_source": (
            "public_manifest_fixed_start_transformed_to_saved_map_local_plus_nominal_leg"
        ),
        "geofence_violation_count": 0,
        "physical_travel_distance_m": 12.0,
        "mission_odom_trajectory_xy_m": [
            [0.0, 0.0],
            [3.0, 0.0],
            [6.0, 0.0],
            [9.0, 0.0],
            [12.0, 0.0],
        ],
        "mission_map_trajectory_xy_m": [
            [0.0, 0.0],
            [3.0, 0.1],
            [6.0, 0.6],
            [9.0, 0.2],
            [12.0, 0.0],
        ],
    }


def _environment_telemetry(schedule_sha256: str) -> dict:
    return {
        "schema_version": 1,
        "collector_role": "evaluator_only_no_robot_control",
        "pedestrian_schedule_sha256": schedule_sha256,
        "active_pedestrian_count": 8,
        "pedestrian_status_samples": [
            {
                "observation_ros_time_ns": 1_000_000_000 + index * 100_000_000,
                "schedule_elapsed_s": 2.0,
                "pedestrian_count": 8,
            }
            for index in range(5)
        ],
        "collision_count": 0,
        "control_topics_published": [],
        "product_actions_created": [],
        "walker_pose_sampling_sufficient": True,
        "walker_pose_source_fresh_at_window_end": True,
        "walker_pose_invalid_frame_count": 0,
        "walker_pose_stale_frame_count": 0,
        "native_pose_transport_error_count": 0,
        "native_pose_transport_timeout_count": 0,
        "walker_pose_complete_frame_count": 4,
        "walker_pose_maximum_complete_frame_gap_s": 0.5,
        "walker_peer_gate_passed": True,
    }


def _fixture(tmp_path: Path) -> dict[str, Path]:
    protocol = json.loads(PROTOCOL_PATH.read_text(encoding="utf-8"))
    episode_manifest = _episode_manifest(tmp_path)
    schedule = _schedule()
    run_root = tmp_path / "run"
    runtime_root = run_root / "runtime"
    runtime_root.mkdir(parents=True)
    schedule_path = (
        runtime_root / "pedestrian_schedule.seed.2026091401.json"
    )
    _write_json(schedule_path, schedule)
    schedule_sha256 = _sha256(schedule_path)
    route_manifest = freeze_route_from_schedule(
        schedule=schedule,
        episode_manifest=episode_manifest,
        protocol=protocol,
        protocol_sha256=_sha256(PROTOCOL_PATH),
        schedule_sha256=schedule_sha256,
        input_sha256={
            "episode_manifest": _sha256(episode_manifest),
            "public_world": "a" * 64,
            "base_schedule": "b" * 64,
        },
        declared_epoch_ns=100,
    )
    route_path = run_root / "predeclared_obstacle_route.json"
    _write_json(route_path, route_manifest)
    _write_json(
        run_root / "dynamic_obstacle_acceptance.json",
        {
            "report_id": "tzcup_formal_dynamic_obstacle_avoidance_acceptance_v1",
            "status": "FORMAL_DYNAMIC_OBSTACLE_AVOIDANCE_ACCEPTANCE_PASSED",
            "passed": True,
        },
    )
    _write_json(run_root / "dynamic.runtime_binding.json", {"bound": True})
    _write_json(runtime_root / "runtime_telemetry.json", _product_telemetry())
    _write_json(
        runtime_root / "environment_truth_telemetry.json",
        _environment_telemetry(schedule_sha256),
    )
    for name in ("dynamic.launch.log", "collector.log", "environment_collector.log"):
        (runtime_root / name).write_text(f"{name}\n", encoding="utf-8")
    timeline_path = run_root / "single_run_timeline.json"
    _write_json(
        timeline_path,
        {
            "schema_version": 1,
            "run_id": "single-run-fixture",
            "status": "RUNNER_FINISHED",
            "run_start_epoch_ns": 1_000,
            "run_end_epoch_ns": 11_000_000_000,
            "schedule_seed": 2026091401,
            "runner_exit_code": 0,
            "evaluator_exit_code": None,
        },
    )
    return {
        "run_root": run_root,
        "route": route_path,
        "timeline": timeline_path,
        "schedule": schedule_path,
    }


def _evaluate(fixture: dict[str, Path]) -> dict:
    return evaluate_run(
        protocol=json.loads(PROTOCOL_PATH.read_text(encoding="utf-8")),
        protocol_path=PROTOCOL_PATH,
        run_root=fixture["run_root"],
        route_manifest_path=fixture["route"],
        run_timeline_path=fixture["timeline"],
    )


def test_protocol_preserves_official_rate_as_not_measured() -> None:
    protocol = json.loads(PROTOCOL_PATH.read_text(encoding="utf-8"))
    validate_protocol(protocol)
    assert protocol["official_metric"]["threshold"] == 0.95
    assert protocol["official_metric"]["status"] == "NOT_MEASURED"
    assert protocol["official_metric"]["single_run_can_prove_threshold"] is False
    assert protocol["denominator"]["single_run_denominator"] == 1


def test_valid_single_run_fixture_passes_without_a_rate_claim(
    tmp_path: Path,
) -> None:
    report = _evaluate(_fixture(tmp_path))
    assert report["status"] == SINGLE_RUN_PASS
    assert report["single_run"]["numerator"] == 1
    assert report["single_run"]["denominator"] == 1
    assert report["single_run"]["observed_fraction"] == 1.0
    assert report["official_metric"]["status"] == "NOT_MEASURED"
    assert report["official_metric"]["measured_value"] is None
    assert report["event"]["minimum_surface_gap_m"] >= 0.12


def test_collision_is_a_failed_started_trial_not_an_exclusion(
    tmp_path: Path,
) -> None:
    fixture = _fixture(tmp_path)
    environment_path = (
        fixture["run_root"] / "runtime/environment_truth_telemetry.json"
    )
    environment = json.loads(environment_path.read_text(encoding="utf-8"))
    environment["collision_count"] = 1
    _write_json(environment_path, environment)
    report = _evaluate(fixture)
    assert report["status"] == SINGLE_RUN_FAIL
    assert report["single_run"]["numerator"] == 0
    assert report["single_run"]["denominator"] == 1
    assert "physical_contact_collision_free" in report["blockers"]
    assert report["official_metric"]["status"] == "NOT_MEASURED"


def test_vehicle_envelope_overlap_fails_minimum_distance(tmp_path: Path) -> None:
    fixture = _fixture(tmp_path)
    product_path = fixture["run_root"] / "runtime/runtime_telemetry.json"
    product = json.loads(product_path.read_text(encoding="utf-8"))
    product["dynamic_interaction_candidates"][0]["vehicle_pose_map"] = [12.0, 0.0]
    _write_json(product_path, product)
    report = _evaluate(fixture)
    assert report["status"] == SINGLE_RUN_FAIL
    assert "minimum_distance_satisfied" in report["blockers"]
    assert "vehicle_envelope_collision_free" in report["blockers"]


def test_wait_only_response_passes_when_collision_monitor_intervenes(
    tmp_path: Path,
) -> None:
    fixture = _fixture(tmp_path)
    product_path = fixture["run_root"] / "runtime/runtime_telemetry.json"
    product = json.loads(product_path.read_text(encoding="utf-8"))
    product["maximum_cross_track_detour_m"] = 0.1
    product["collision_monitor_intervention_count"] = 1
    _write_json(product_path, product)
    report = _evaluate(fixture)
    assert report["status"] == SINGLE_RUN_PASS
    assert report["checks"]["reroute_or_wait_observed"] is True


def test_timeout_and_missing_task_recovery_fail(tmp_path: Path) -> None:
    fixture = _fixture(tmp_path)
    product_path = fixture["run_root"] / "runtime/runtime_telemetry.json"
    product = json.loads(product_path.read_text(encoding="utf-8"))
    product["nav2_goal_succeeded"] = False
    product["completion_reason"] = "timeout"
    _write_json(product_path, product)
    timeline = json.loads(fixture["timeline"].read_text(encoding="utf-8"))
    timeline["run_end_epoch_ns"] = 400_000_000_000
    _write_json(fixture["timeline"], timeline)
    report = _evaluate(fixture)
    assert report["status"] == SINGLE_RUN_FAIL
    assert "task_recovered_after_obstacle" in report["blockers"]
    assert "wall_time_within_protocol_limit" in report["blockers"]


def test_insufficient_environment_sampling_fails(tmp_path: Path) -> None:
    fixture = _fixture(tmp_path)
    environment_path = (
        fixture["run_root"] / "runtime/environment_truth_telemetry.json"
    )
    environment = json.loads(environment_path.read_text(encoding="utf-8"))
    environment["pedestrian_status_samples"] = environment[
        "pedestrian_status_samples"
    ][:1]
    _write_json(environment_path, environment)
    report = _evaluate(fixture)
    assert report["status"] == SINGLE_RUN_FAIL
    assert "eight_pedestrians_active" in report["blockers"]


def test_post_run_route_mismatch_is_invalid_evidence(tmp_path: Path) -> None:
    fixture = _fixture(tmp_path)
    schedule_path = fixture["schedule"]
    schedule = json.loads(schedule_path.read_text(encoding="utf-8"))
    schedule["pedestrians"][0]["waypoints"][1][1] = 13.0
    _write_json(schedule_path, schedule)
    report = _evaluate(fixture)
    assert report["status"] == SINGLE_RUN_INVALID
    assert report["single_run"]["denominator"] == 1
    assert "schedule_hash_matches_predeclaration" in report["blockers"]
    assert "live_schedule_uses_predeclared_route" in report["blockers"]


def test_missing_raw_log_invalidates_a_started_trial(tmp_path: Path) -> None:
    fixture = _fixture(tmp_path)
    (
        fixture["run_root"] / "runtime/dynamic.launch.log"
    ).unlink()
    report = _evaluate(fixture)
    assert report["status"] == SINGLE_RUN_INVALID
    assert report["single_run"]["numerator"] == 0
    assert report["single_run"]["denominator"] == 1
    assert "all_required_evidence_files_present" in report["blockers"]


def test_route_must_be_declared_before_run_start(tmp_path: Path) -> None:
    fixture = _fixture(tmp_path)
    route = json.loads(fixture["route"].read_text(encoding="utf-8"))
    route["declared_epoch_ns"] = 2_000
    _write_json(fixture["route"], route)
    report = _evaluate(fixture)
    assert report["status"] == SINGLE_RUN_INVALID
    assert "route_predeclared_before_run_start" in report["blockers"]


def test_protocol_hash_change_invalidates_the_run(tmp_path: Path) -> None:
    fixture = _fixture(tmp_path)
    route = json.loads(fixture["route"].read_text(encoding="utf-8"))
    route["protocol_sha256"] = "0" * 64
    _write_json(fixture["route"], route)
    report = _evaluate(fixture)
    assert report["status"] == SINGLE_RUN_INVALID
    assert "protocol_hash_matches_predeclaration" in report["blockers"]


def test_campaign_rule_is_explicit_single_run_denominator_is_exact() -> None:
    protocol = json.loads(PROTOCOL_PATH.read_text(encoding="utf-8"))
    denominator = protocol["denominator"]
    assert denominator["single_run_denominator"] == 1
    assert denominator["post_launch_exclusions"] == "none"
    assert denominator["minimum_runs_per_scenario"] == 20
    assert denominator["recommended_total_runs"] == 100
