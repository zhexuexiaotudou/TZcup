#!/usr/bin/env python3
"""Deterministically evaluate one completed dynamic-avoidance trial."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
from typing import Any

from prepare_dynamic_avoidance_single_run import (
    FUNCTIONAL_SMOKE_MODE,
    OFFICIAL_MODE,
    ROUTE_MANIFEST_KIND,
    _source_to_map,
    first_centerline_crossing_time,
    protocol_mode,
    validate_protocol,
)


SINGLE_RUN_PASS = "SINGLE_RUN_FUNCTIONAL_PASS"
SINGLE_RUN_FAIL = "SINGLE_RUN_FUNCTIONAL_FAIL"
SINGLE_RUN_INVALID = "SINGLE_RUN_EVIDENCE_INVALID"
FORMAL_REPORT_ID = "tzcup_formal_dynamic_obstacle_avoidance_acceptance_v1"
FORMAL_PASS_STATUS = "FORMAL_DYNAMIC_OBSTACLE_AVOIDANCE_ACCEPTANCE_PASSED"
LIVE_SLAM_MAP_SOURCE = "LIVE_SLAM"
OFFLINE_RAYCAST_MAPPING = "OFFLINE_RAYCAST_MAPPING"
OFFLINE_MAP_SOURCE_LABEL = "OFFLINE_MAP_SOURCE"
MAP_SOURCE_MODES = (LIVE_SLAM_MAP_SOURCE, OFFLINE_RAYCAST_MAPPING)


def _read_object(path: Path, label: str) -> dict[str, Any]:
    if not path.is_file() or path.is_symlink():
        raise ValueError(f"{label} must be a regular file: {path}")
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"{label} is not valid JSON: {path}") from exc
    if not isinstance(value, dict):
        raise ValueError(f"{label} JSON root must be an object")
    return value


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _atomic_write_json(path: Path, value: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    pending = path.with_suffix(path.suffix + f".pending.{os.getpid()}")
    pending.write_text(
        json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    pending.replace(path)


def _finite_number(value: Any) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    number = float(value)
    return number if math.isfinite(number) else None


def _integer(value: Any) -> int | None:
    if isinstance(value, bool) or not isinstance(value, int):
        return None
    return value


def _interpolate_route(
    route_map: list[list[float]], elapsed_s: float
) -> tuple[float, float]:
    if len(route_map) < 2:
        raise ValueError("route_map needs at least two waypoints")
    period = float(route_map[-1][0])
    if period <= 0.0:
        raise ValueError("route_map period must be positive")
    phase = elapsed_s % period
    for first, second in zip(route_map, route_map[1:]):
        if phase <= float(second[0]):
            duration = float(second[0]) - float(first[0])
            if duration <= 0.0:
                raise ValueError("route_map times must increase")
            ratio = (phase - float(first[0])) / duration
            return (
                float(first[1])
                + ratio * (float(second[1]) - float(first[1])),
                float(first[2])
                + ratio * (float(second[2]) - float(first[2])),
            )
    raise AssertionError("validated route interpolation fell through")


def _trigger_alignment_s(
    elapsed_s: float, trigger_time_s: float, period_s: float
) -> float:
    if period_s <= 0.0:
        raise ValueError("route period must be positive")
    delta = abs(elapsed_s - trigger_time_s) % period_s
    return min(delta, period_s - delta)


def _evidence_file(root: Path, relative: str, role: str) -> dict[str, Any]:
    path = root / relative
    exists = path.is_file() and not path.is_symlink()
    return {
        "role": role,
        "path": str(path),
        "relative_path": relative,
        "exists": exists,
        "size_bytes": path.stat().st_size if exists else None,
        "sha256": _sha256(path) if exists else None,
    }


def _all_raw_logs_present(files: list[dict[str, Any]]) -> bool:
    return bool(files) and all(
        row["exists"] is True
        and isinstance(row["size_bytes"], int)
        and row["size_bytes"] > 0
        and isinstance(row["sha256"], str)
        and len(row["sha256"]) == 64
        for row in files
    )


def _exact_route_match(
    expected: list[list[float]], actual: list[list[float]]
) -> bool:
    if len(expected) != len(actual):
        return False
    return all(
        all(
            math.isclose(float(left), float(right), rel_tol=0.0, abs_tol=1e-9)
            for left, right in zip(expected_row, actual_row)
        )
        for expected_row, actual_row in zip(expected, actual)
    )


def _numeric_route(value: Any) -> list[list[float]] | None:
    if not isinstance(value, list):
        return None
    route: list[list[float]] = []
    for row in value:
        if not isinstance(row, list) or len(row) != 3:
            return None
        try:
            numbers = [float(item) for item in row]
        except (TypeError, ValueError):
            return None
        if not all(math.isfinite(item) for item in numbers):
            return None
        route.append(numbers)
    return route


def evaluate_run(
    *,
    protocol: dict[str, Any],
    protocol_path: Path,
    run_root: Path,
    route_manifest_path: Path,
    run_timeline_path: Path,
    map_source_mode: str = LIVE_SLAM_MAP_SOURCE,
) -> dict[str, Any]:
    """Evaluate one started trial and never infer a campaign success rate."""

    validate_protocol(protocol)
    mode = protocol_mode(protocol)
    if map_source_mode not in MAP_SOURCE_MODES:
        raise ValueError(f"unsupported map_source_mode: {map_source_mode}")
    root = run_root.resolve()
    runtime_root = root / "runtime"
    schedule_relative = (
        f"runtime/pedestrian_schedule.seed.{int(protocol['schedule']['seed'])}.json"
    )
    evidence_files = [
        _evidence_file(root, "dynamic_obstacle_acceptance.json", "formal_report"),
        _evidence_file(root, "dynamic.runtime_binding.json", "runtime_binding"),
        _evidence_file(root, "runtime/runtime_telemetry.json", "product_telemetry"),
        _evidence_file(
            root,
            "runtime/environment_truth_telemetry.json",
            "environment_telemetry",
        ),
        _evidence_file(root, schedule_relative, "pedestrian_schedule"),
        _evidence_file(root, "runtime/dynamic.launch.log", "dynamic_launch_log"),
        _evidence_file(root, "runtime/collector.log", "collector_log"),
        _evidence_file(
            root, "runtime/environment_collector.log", "environment_collector_log"
        ),
    ]
    route_file = _evidence_file(
        route_manifest_path.parent,
        route_manifest_path.name,
        "predeclared_obstacle_route",
    )
    timeline_file = _evidence_file(
        run_timeline_path.parent,
        run_timeline_path.name,
        "run_timeline",
    )
    protocol_file = _evidence_file(protocol_path.parent, protocol_path.name, "protocol")
    all_files = [*evidence_files, route_file, timeline_file, protocol_file]

    timeline: dict[str, Any] = {}
    route_manifest: dict[str, Any] = {}
    formal_report: dict[str, Any] = {}
    product: dict[str, Any] = {}
    environment: dict[str, Any] = {}
    schedule: dict[str, Any] = {}
    structural_errors: list[str] = []
    for path, label, target in (
        (run_timeline_path, "run timeline", timeline),
        (route_manifest_path, "route manifest", route_manifest),
        (root / "dynamic_obstacle_acceptance.json", "formal report", formal_report),
        (root / "runtime/runtime_telemetry.json", "product telemetry", product),
        (
            root / "runtime/environment_truth_telemetry.json",
            "environment telemetry",
            environment,
        ),
        (root / schedule_relative, "pedestrian schedule", schedule),
    ):
        try:
            target.update(_read_object(path, label))
        except ValueError as exc:
            structural_errors.append(str(exc))

    started = timeline.get("status") in {"STARTED", "RUNNER_FINISHED"}
    denominator = 1 if started else 0
    run_start_ns = timeline.get("run_start_epoch_ns")
    run_end_ns = timeline.get("run_end_epoch_ns")
    duration_s: float | None = None
    if (
        isinstance(run_start_ns, int)
        and isinstance(run_end_ns, int)
        and run_start_ns > 0
        and run_end_ns >= run_start_ns
    ):
        duration_s = (run_end_ns - run_start_ns) / 1.0e9

    checks: dict[str, bool] = {}
    checks["run_timeline_started"] = started
    checks["all_required_evidence_files_present"] = _all_raw_logs_present(all_files)
    checks["protocol_hash_matches_predeclaration"] = (
        route_manifest.get("protocol_sha256") == _sha256(protocol_path)
    )
    checks["route_manifest_kind_expected"] = (
        route_manifest.get("kind") == ROUTE_MANIFEST_KIND
    )
    checks["route_manifest_mode_matches_protocol"] = (
        route_manifest.get("mode", OFFICIAL_MODE) == mode
    )
    checks["run_timeline_mode_matches_protocol"] = (
        timeline.get("mode", OFFICIAL_MODE) == mode
    )
    declared_ns = route_manifest.get("declared_epoch_ns")
    checks["route_predeclared_before_run_start"] = (
        isinstance(declared_ns, int)
        and isinstance(run_start_ns, int)
        and 0 < declared_ns <= run_start_ns
    )
    checks["schedule_hash_matches_predeclaration"] = (
        isinstance(schedule, dict)
        and bool(schedule)
        and route_manifest.get("schedule_sha256")
        == _sha256(root / schedule_relative)
        if evidence_files[4]["exists"]
        else False
    )
    environment_contract = schedule.get("acceptance_environment", {})
    checks["schedule_seed_matches_protocol"] = (
        isinstance(environment_contract, dict)
        and environment_contract.get("seed") == protocol["schedule"]["seed"]
    )
    crossing_ids = (
        environment_contract.get("mission_corridor_crossing_ids", [])
        if isinstance(environment_contract, dict)
        else []
    )
    selected = route_manifest.get("selected_obstacle", {})
    selected_id = selected.get("object_id") if isinstance(selected, dict) else None
    checks["predeclared_obstacle_is_selected_crossing"] = (
        isinstance(crossing_ids, list)
        and bool(crossing_ids)
        and selected_id == crossing_ids[0]
    )

    route_source = (
        selected.get("route_source_world") if isinstance(selected, dict) else None
    )
    route_map = selected.get("route_map") if isinstance(selected, dict) else None
    actual_source: list[list[float]] | None = None
    pedestrians = schedule.get("pedestrians")
    if isinstance(pedestrians, list):
        rows = [
            row
            for row in pedestrians
            if isinstance(row, dict) and row.get("object_id") == selected_id
        ]
        if len(rows) == 1 and isinstance(rows[0].get("waypoints"), list):
            actual_source = _numeric_route(rows[0]["waypoints"])
    checks["live_schedule_uses_predeclared_route"] = (
        isinstance(route_source, list)
        and actual_source is not None
        and _exact_route_match(route_source, actual_source)
    )
    source_start = route_manifest.get("source_fixed_start_pose")
    actual_map: list[list[float]] | None = None
    if (
        isinstance(source_start, list)
        and len(source_start) == 3
        and actual_source is not None
    ):
        try:
            start = tuple(float(value) for value in source_start)
        except (TypeError, ValueError):
            structural_errors.append("source_fixed_start_pose is not numeric")
        else:
            if all(math.isfinite(value) for value in start):
                actual_map = [
                    [row[0], *_source_to_map((row[1], row[2]), start)]
                    for row in actual_source
                ]
            else:
                structural_errors.append("source_fixed_start_pose is not finite")
    checks["live_schedule_route_map_matches"] = (
        isinstance(route_map, list)
        and actual_map is not None
        and _exact_route_match(route_map, actual_map)
    )
    trigger_recomputed: float | None = None
    if actual_map is not None:
        try:
            trigger_recomputed = first_centerline_crossing_time(actual_map)
        except ValueError as exc:
            structural_errors.append(str(exc))
    declared_trigger = selected.get(
        "trigger_time_from_schedule_start_s"
    ) if isinstance(selected, dict) else None
    checks["trigger_time_matches_predeclaration"] = (
        isinstance(declared_trigger, (int, float))
        and not isinstance(declared_trigger, bool)
        and trigger_recomputed is not None
        and math.isclose(
            float(declared_trigger),
            trigger_recomputed,
            rel_tol=0.0,
            abs_tol=1.0e-9,
        )
    )
    checks["wall_time_within_protocol_limit"] = (
        duration_s is not None
        and duration_s <= float(protocol["task"]["wall_time_limit_s"])
    )
    checks["formal_dynamic_gate_passed"] = (
        formal_report.get("report_id") == FORMAL_REPORT_ID
        and formal_report.get("status") == FORMAL_PASS_STATUS
        and formal_report.get("passed") is True
    )
    expected_map_source_label = (
        LIVE_SLAM_MAP_SOURCE
        if map_source_mode == LIVE_SLAM_MAP_SOURCE
        else OFFLINE_MAP_SOURCE_LABEL
    )
    checks["formal_report_map_source_label_matches"] = (
        formal_report.get("map_source", {}).get("mode") == map_source_mode
        and formal_report.get("map_source", {}).get("label")
        == expected_map_source_label
        and formal_report.get("map_source", {}).get("live_slam")
        is (map_source_mode == LIVE_SLAM_MAP_SOURCE)
    )

    environment_collector = (
        product.get("environment_truth_collector", {})
        if isinstance(product, dict)
        else {}
    )
    env_status_samples = environment.get("pedestrian_status_samples")
    env_status_valid = env_status_samples if isinstance(env_status_samples, list) else []
    checks["environment_truth_is_evaluator_only"] = (
        environment.get("collector_role") == "evaluator_only_no_robot_control"
        and environment.get("control_topics_published") == []
        and environment.get("product_actions_created") == []
    )
    checks["environment_schedule_hash_matches"] = (
        environment.get("pedestrian_schedule_sha256")
        == route_manifest.get("schedule_sha256")
    )
    checks["eight_pedestrians_active"] = (
        environment.get("active_pedestrian_count") == 8
        and len(env_status_valid)
        >= int(protocol["sampling"]["minimum_environment_status_samples"])
    )
    pose_gap_s = _finite_number(
        environment.get("walker_pose_maximum_complete_frame_gap_s")
    )
    pose_invalid_count = _integer(environment.get("walker_pose_invalid_frame_count"))
    pose_stale_count = _integer(environment.get("walker_pose_stale_frame_count"))
    pose_transport_error_count = _integer(
        environment.get("native_pose_transport_error_count")
    )
    pose_transport_timeout_count = _integer(
        environment.get("native_pose_transport_timeout_count")
    )
    pose_complete_frame_count = _integer(
        environment.get("walker_pose_complete_frame_count")
    )
    checks["environment_sampling_integrity"] = (
        environment.get("walker_pose_sampling_sufficient") is True
        and environment.get("walker_pose_source_fresh_at_window_end") is True
        and pose_invalid_count == 0
        and pose_stale_count == 0
        and pose_transport_error_count == 0
        and pose_transport_timeout_count == 0
        and pose_complete_frame_count is not None
        and pose_complete_frame_count
        >= int(protocol["sampling"]["minimum_walker_pose_frames"])
        and pose_gap_s is not None
        and pose_gap_s <= float(protocol["sampling"]["maximum_walker_pose_gap_s"])
        and environment.get("walker_peer_gate_passed") is True
    )
    checks["product_command_chain_observed"] = (
        product.get("command_chain")
        == [
            "/cmd_vel_nav",
            "/cmd_vel_smoothed",
            "/cmd_vel_gate",
            "/base_controller/cmd_vel",
        ]
    )
    monitor_node_count = _integer(product.get("collision_monitor_node_count"))
    checks["collision_monitor_is_single_command_gate"] = (
        monitor_node_count == 1
        and isinstance(product.get("command_topic_publishers"), dict)
        and product["command_topic_publishers"].get("/cmd_vel_gate")
        == ["/collision_monitor"]
    )
    minimum_samples = protocol["sampling"]["minimum_product_topic_samples"]
    product_counts = product.get("topic_sample_counts")
    odom_pose_sample_count = _integer(product.get("odom_pose_sample_count"))
    map_pose_sample_count = _integer(product.get("map_pose_sample_count"))
    checks["product_topic_sampling_sufficient"] = (
        isinstance(product_counts, dict)
        and all(
            _integer(product_counts.get(topic)) is not None
            and int(product_counts[topic]) >= int(minimum)
            for topic, minimum in minimum_samples.items()
        )
        and odom_pose_sample_count is not None
        and odom_pose_sample_count >= 5
        and map_pose_sample_count is not None
        and map_pose_sample_count >= 5
    )

    candidates = product.get("dynamic_interaction_candidates")
    candidate_rows = candidates if isinstance(candidates, list) else []
    selected_candidate_count = 0
    minimum_surface_gap_m: float | None = None
    minimum_trigger_alignment_s: float | None = None
    envelope_overlap_count = 0
    if (
        isinstance(route_map, list)
        and len(route_map) >= 2
        and isinstance(selected, dict)
    ):
        walker_radius = _finite_number(selected.get("radius_m"))
        vehicle_radius = _finite_number(
            protocol["obstacle"]["minimum_distance"]["vehicle_envelope_radius_m"]
        )
        if walker_radius is None or vehicle_radius is None:
            structural_errors.append("route or protocol radii are invalid")
        else:
            period_s = float(route_map[-1][0])
            for raw in candidate_rows:
                if not isinstance(raw, dict):
                    continue
                observation_ns = raw.get("observation_ros_time_ns")
                vehicle_pose = raw.get("vehicle_pose_map")
                if (
                    not isinstance(observation_ns, int)
                    or not isinstance(vehicle_pose, list)
                    or len(vehicle_pose) != 2
                ):
                    continue
                aligned_samples = [
                    sample
                    for sample in env_status_valid
                    if isinstance(sample, dict)
                    and isinstance(sample.get("observation_ros_time_ns"), int)
                    and isinstance(sample.get("schedule_elapsed_s"), (int, float))
                    and not isinstance(sample.get("schedule_elapsed_s"), bool)
                ]
                if not aligned_samples:
                    continue
                nearest = min(
                    aligned_samples,
                    key=lambda sample: abs(
                        int(sample["observation_ros_time_ns"]) - observation_ns
                    ),
                )
                alignment_s = (
                    abs(int(nearest["observation_ros_time_ns"]) - observation_ns)
                    / 1.0e9
                )
                if alignment_s > 0.5:
                    continue
                elapsed_s = float(nearest["schedule_elapsed_s"])
                trigger_delta = _trigger_alignment_s(
                    elapsed_s, float(declared_trigger), period_s
                )
                if trigger_delta > float(protocol["obstacle"]["trigger"]["window_s"]):
                    continue
                try:
                    walker_x, walker_y = _interpolate_route(route_map, elapsed_s)
                except ValueError as exc:
                    structural_errors.append(str(exc))
                    continue
                center_distance = math.dist(
                    (walker_x, walker_y),
                    (float(vehicle_pose[0]), float(vehicle_pose[1])),
                )
                surface_gap = center_distance - walker_radius - vehicle_radius
                minimum_surface_gap_m = (
                    surface_gap
                    if minimum_surface_gap_m is None
                    else min(minimum_surface_gap_m, surface_gap)
                )
                minimum_trigger_alignment_s = (
                    trigger_delta
                    if minimum_trigger_alignment_s is None
                    else min(minimum_trigger_alignment_s, trigger_delta)
                )
                envelope_overlap_count += int(surface_gap <= 0.0)
                selected_candidate_count += 1
    checks["predeclared_obstacle_interaction_observed"] = (
        selected_candidate_count >= 1
    )
    checks["trigger_window_observed"] = (
        minimum_trigger_alignment_s is not None
        and minimum_trigger_alignment_s
        <= float(protocol["sampling"]["maximum_trigger_alignment_s"])
    )
    checks["minimum_distance_satisfied"] = (
        minimum_surface_gap_m is not None
        and minimum_surface_gap_m
        >= float(
            protocol["obstacle"]["minimum_distance"]["threshold_m"]
        )
    )
    physical_collision_count = _integer(environment.get("collision_count"))
    monitor_intervention_count = _integer(
        product.get("collision_monitor_intervention_count")
    )
    checks["physical_contact_collision_free"] = physical_collision_count == 0
    checks["vehicle_envelope_collision_free"] = envelope_overlap_count == 0
    checks["collision_monitor_intervened"] = (
        monitor_intervention_count is not None
        and monitor_intervention_count >= 1
    )
    detour = _finite_number(product.get("maximum_cross_track_detour_m"))
    reroute_pass = (
        detour is not None
        and detour
        >= float(protocol["response"]["reroute_min_cross_track_detour_m"])
    )
    wait_pass = monitor_intervention_count is not None and monitor_intervention_count >= 1
    feedback_sample_count = _integer(product.get("feedback_sample_count"))
    checks["reroute_or_wait_observed"] = reroute_pass or wait_pass
    checks["task_recovered_after_obstacle"] = (
        product.get("goal_accepted") is True
        and product.get("nav2_goal_succeeded") is True
        and product.get("completion_reason") == "action_result"
        and feedback_sample_count is not None
        and feedback_sample_count >= 1
        and product.get("mission_goal_source")
        == protocol["task"]["required_goal_source"]
    )
    geofence_violation_count = _integer(product.get("geofence_violation_count"))
    checks["zero_geofence_violations"] = geofence_violation_count == 0
    physical_travel_m = _finite_number(product.get("physical_travel_distance_m"))
    odom_trajectory = product.get("mission_odom_trajectory_xy_m", [])
    map_trajectory = product.get("mission_map_trajectory_xy_m", [])
    checks["mission_progress_recorded"] = (
        physical_travel_m is not None
        and physical_travel_m >= 5.0
        and isinstance(odom_trajectory, list)
        and len(odom_trajectory) >= 5
        and isinstance(map_trajectory, list)
        and len(map_trajectory) >= 5
    )

    if structural_errors:
        checks["no_structural_evidence_errors"] = False
    else:
        checks["no_structural_evidence_errors"] = True
    evidence_check_names = {
        "run_timeline_started",
        "all_required_evidence_files_present",
        "protocol_hash_matches_predeclaration",
        "route_manifest_kind_expected",
        "route_manifest_mode_matches_protocol",
        "run_timeline_mode_matches_protocol",
        "route_predeclared_before_run_start",
        "schedule_hash_matches_predeclaration",
        "schedule_seed_matches_protocol",
        "predeclared_obstacle_is_selected_crossing",
        "live_schedule_uses_predeclared_route",
        "live_schedule_route_map_matches",
        "trigger_time_matches_predeclaration",
        "no_structural_evidence_errors",
    }
    evidence_valid = all(checks[name] for name in evidence_check_names)
    functional_pass = evidence_valid and all(checks.values())
    numerator = 1 if functional_pass else 0
    status = (
        SINGLE_RUN_PASS
        if functional_pass
        else SINGLE_RUN_INVALID
        if not evidence_valid
        else SINGLE_RUN_FAIL
    )
    blockers = [name for name, passed in checks.items() if not passed]
    report = {
        "schema_version": 1,
        "report_id": "tzcup_dynamic_avoidance_single_run_evaluation_v1",
        "status": status,
        "passed": functional_pass,
        "mode": mode,
        "run_id": timeline.get("run_id"),
        "protocol_id": protocol["protocol_id"],
        "map_source": {
            "mode": map_source_mode,
            "label": expected_map_source_label,
            "live_slam": map_source_mode == LIVE_SLAM_MAP_SOURCE,
        },
        "single_run": {
            "started": started,
            "numerator": numerator,
            "denominator": denominator,
            "observed_fraction": (
                None if denominator == 0 else numerator / denominator
            ),
            "functional_success": functional_pass,
            "post_launch_exclusions": 0,
        },
        "official_metric": {
            "metric_id": protocol["official_metric"]["metric_id"],
            "formula": protocol["official_metric"]["formula"],
            "threshold": protocol["official_metric"]["threshold"],
            "measured_value": None,
            "status": "NOT_MEASURED",
            "independent_run_count": denominator,
            "minimum_runs_per_scenario": protocol["denominator"][
                "minimum_runs_per_scenario"
            ],
            "recommended_total_runs": protocol["denominator"][
                "recommended_total_runs"
            ],
            "reason": (
                "One started trial yields a single-run numerator and denominator "
                "only. It cannot measure or establish the >=95% campaign rate."
            ),
        },
        "event": {
            "obstacle_object_id": selected_id,
            "trigger_time_from_schedule_start_s": declared_trigger,
            "trigger_window_s": protocol["obstacle"]["trigger"]["window_s"],
            "selected_interaction_candidate_count": selected_candidate_count,
            "minimum_trigger_alignment_s": minimum_trigger_alignment_s,
            "minimum_surface_gap_m": minimum_surface_gap_m,
            "minimum_surface_gap_threshold_m": protocol["obstacle"][
                "minimum_distance"
            ]["threshold_m"],
            "vehicle_envelope_overlap_count": envelope_overlap_count,
            "physical_collision_count": physical_collision_count,
            "collision_monitor_intervention_count": monitor_intervention_count,
            "maximum_cross_track_detour_m": detour,
            "task_completion_reason": product.get("completion_reason"),
            "wall_time_s": duration_s,
        },
        "checks": checks,
        "blockers": blockers,
        "structural_errors": structural_errors,
        "evidence_files": all_files,
        "claim_boundary": (
            (
                "FUNCTIONAL_SMOKE_NOT_OFFICIAL_95 proves only that this one "
                "6.0 m short mission completed the retained functional checks "
                "without collision. It is not the official 30.0 m protocol, "
                "does not measure the >=95% rate, and leaves the official "
                "metric NOT_MEASURED."
            )
            if mode == FUNCTIONAL_SMOKE_MODE
            else (
                "A PASS proves only that this one predeclared trial satisfied "
                "the functional checks and retained the required raw evidence. "
                "It does not measure the official >=95% dynamic-avoidance "
                "success rate, establish statistical confidence, validate "
                "real-vehicle braking, or justify changing the official metric "
                "from NOT_MEASURED."
            )
        ),
    }
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--protocol", type=Path, required=True)
    parser.add_argument("--run-root", type=Path, required=True)
    parser.add_argument("--route-manifest", type=Path, required=True)
    parser.add_argument("--run-timeline", type=Path, required=True)
    parser.add_argument(
        "--map-source-mode",
        choices=MAP_SOURCE_MODES,
        default=LIVE_SLAM_MAP_SOURCE,
    )
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise SystemExit(f"fresh single-run evaluation output required: {args.output}")
    if not args.run_root.is_dir() or args.run_root.is_symlink():
        raise SystemExit(f"run root must be a real directory: {args.run_root}")
    protocol = _read_object(args.protocol, "protocol")
    report = evaluate_run(
        protocol=protocol,
        protocol_path=args.protocol,
        run_root=args.run_root,
        route_manifest_path=args.route_manifest,
        run_timeline_path=args.run_timeline,
        map_source_mode=args.map_source_mode,
    )
    _atomic_write_json(args.output, report)
    print(args.output)
    if report["status"] == SINGLE_RUN_PASS:
        return 0
    if report["status"] == SINGLE_RUN_INVALID:
        return 3
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
