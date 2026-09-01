#!/usr/bin/env python3
"""Fail-closed validator for the formal saved-map/Nav2 pedestrian run."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
from typing import Any

from formal_runtime_gate_binding import load_binding


PASSED_STATUS = "FORMAL_DYNAMIC_OBSTACLE_AVOIDANCE_ACCEPTANCE_PASSED"
BLOCKED_STATUS = "FORMAL_DYNAMIC_OBSTACLE_AVOIDANCE_ACCEPTANCE_BLOCKED"
CONTROL_PROHIBITED_TRUTH_TOPICS = (
    "/scenario/environment/pedestrian_driver/status",
)


def _trajectory(value: Any, label: str) -> list[tuple[float, float]]:
    if not isinstance(value, list):
        return []
    points: list[tuple[float, float]] = []
    for row in value:
        if not isinstance(row, list) or len(row) != 2:
            return []
        try:
            point = (float(row[0]), float(row[1]))
        except (TypeError, ValueError):
            return []
        if not all(math.isfinite(item) for item in point):
            return []
        points.append(point)
    return points


def _read(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"JSON root must be an object: {path}")
    return value


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def frozen_session_binding(
    snapshot_manifest: Path,
    session_status: Path,
) -> dict[str, Any]:
    """Validate and describe the currently running frozen acceptance session."""

    snapshot = _read(snapshot_manifest)
    session = _read(session_status)
    outputs = snapshot.get("outputs")
    if not isinstance(outputs, dict):
        raise ValueError("formal vehicle snapshot has no outputs mapping")
    urdf = outputs.get("reports/engineering/formal_competition_vehicle.urdf")
    if not isinstance(urdf, dict) or not isinstance(urdf.get("sha256"), str):
        raise ValueError("formal vehicle snapshot has no expanded URDF hash")
    source_hash = snapshot.get("source_inventory_sha256")
    if not isinstance(source_hash, str) or len(source_hash) != 64:
        raise ValueError("formal vehicle snapshot has no source inventory hash")
    identity = {
        "snapshot_manifest_sha256": _sha256(snapshot_manifest),
        "source_inventory_sha256": source_hash,
        "expanded_urdf_sha256": urdf["sha256"],
    }
    if session.get("status") != "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING":
        raise ValueError("formal final acceptance session is not RUNNING")
    if session.get("snapshot") != identity:
        raise ValueError("formal acceptance session is bound to another snapshot")
    started_ns = session.get("started_epoch_ns")
    if not isinstance(started_ns, int) or started_ns <= 0:
        raise ValueError("formal acceptance session start time is invalid")
    return {
        **identity,
        "snapshot_manifest": str(snapshot_manifest.resolve()),
        "session_status": str(session_status.resolve()),
        "session_started_epoch_ns": started_ns,
        "session_running": True,
    }


def load_public_mission_contract(
    episode_manifest: Path,
    *,
    goal_x: float | None = None,
    goal_y: float | None = None,
    nominal_leg_m: float = 30.0,
) -> dict[str, Any]:
    """Load the public-only mission geometry used by product control.

    The dynamic environment schedule is deliberately not an argument.  The
    default goal is a deterministic leg from the public fixed start pose into
    the campus, so neither a pedestrian waypoint nor evaluator truth can
    select the command sent to Nav2.
    """

    manifest = _read(episode_manifest)
    if manifest.get("schema_version") != 1:
        raise ValueError("public episode manifest schema_version must equal 1")
    start_raw = manifest.get("vehicle_start_pose_map")
    field_raw = manifest.get("field")
    counts_raw = manifest.get("counts")
    if not isinstance(start_raw, dict):
        raise ValueError("public vehicle_start_pose_map is missing")
    if not isinstance(field_raw, dict):
        raise ValueError("public field contract is missing")
    if not isinstance(counts_raw, dict):
        raise ValueError("public object counts are missing")
    source_start = (
        float(start_raw["x_m"]),
        float(start_raw["y_m"]),
        float(start_raw.get("yaw_rad", 0.0)),
    )
    polygon_raw = field_raw.get("geofence_polygon_m")
    if not isinstance(polygon_raw, list) or len(polygon_raw) < 3:
        raise ValueError("public geofence polygon needs at least three vertices")
    source_polygon = [(float(row[0]), float(row[1])) for row in polygon_raw]
    cosine, sine = math.cos(source_start[2]), math.sin(source_start[2])

    def to_local(point: tuple[float, float]) -> tuple[float, float]:
        dx, dy = point[0] - source_start[0], point[1] - source_start[1]
        return cosine * dx + sine * dy, -sine * dx + cosine * dy

    def to_source(point: tuple[float, float]) -> tuple[float, float]:
        return (
            source_start[0] + cosine * point[0] - sine * point[1],
            source_start[1] + sine * point[0] + cosine * point[1],
        )

    polygon = [to_local(point) for point in source_polygon]
    if not math.isfinite(nominal_leg_m) or nominal_leg_m < 5.0:
        raise ValueError("nominal dynamic acceptance leg must be at least 5 m")
    goal = (
        nominal_leg_m if goal_x is None else float(goal_x),
        0.0 if goal_y is None else float(goal_y),
    )
    if not all(math.isfinite(value) for value in (*source_start, *goal)):
        raise ValueError("public start and mission goal must be finite")
    if not point_in_polygon(goal, polygon, boundary_is_inside=True):
        raise ValueError(f"public mission goal is outside the geofence: {goal}")
    default_goal = goal_x is None and goal_y is None
    return {
        "episode_id": manifest.get("episode_id"),
        "map_id": manifest.get("map_id"),
        "source_fixed_start_pose": list(source_start),
        "start_pose_map": [0.0, 0.0, 0.0],
        "goal_pose_map": [goal[0], goal[1], 0.0],
        "goal_pose_source_world": [*to_source(goal), source_start[2]],
        "goal_source": (
            "public_manifest_fixed_start_transformed_to_saved_map_local_plus_nominal_leg"
            if default_goal
            else "explicit_map_goal_diagnostic_only"
        ),
        "geofence_polygon_m": [list(point) for point in polygon],
        "geofence_polygon_source_world": [
            list(point) for point in source_polygon
        ],
        "expected_pedestrian_count": int(counts_raw.get("pedestrians", -1)),
    }


def point_in_polygon(
    point: tuple[float, float],
    polygon: list[tuple[float, float]],
    *,
    boundary_is_inside: bool,
) -> bool:
    """Return whether a point is in a simple polygon, including its boundary."""

    x, y = point
    inside = False
    for first, second in zip(polygon, polygon[1:] + polygon[:1]):
        x1, y1 = first
        x2, y2 = second
        cross = (x - x1) * (y2 - y1) - (y - y1) * (x2 - x1)
        if (
            abs(cross) <= 1.0e-9
            and min(x1, x2) - 1.0e-9 <= x <= max(x1, x2) + 1.0e-9
            and min(y1, y2) - 1.0e-9 <= y <= max(y1, y2) + 1.0e-9
        ):
            return boundary_is_inside
        if (y1 > y) != (y2 > y):
            intersection_x = (x2 - x1) * (y - y1) / (y2 - y1) + x1
            if x < intersection_x:
                inside = not inside
    return inside


def _interpolate_schedule(
    waypoints: list[list[float]], elapsed_s: float
) -> tuple[float, float]:
    if len(waypoints) < 2:
        raise ValueError("pedestrian schedule needs at least two waypoints")
    parsed = [tuple(float(value) for value in row) for row in waypoints]
    if parsed[0][0] != 0.0 or any(
        second[0] <= first[0] for first, second in zip(parsed, parsed[1:])
    ):
        raise ValueError("pedestrian schedule times are invalid")
    phase = elapsed_s % parsed[-1][0]
    for first, second in zip(parsed, parsed[1:]):
        if phase <= second[0]:
            ratio = (phase - first[0]) / (second[0] - first[0])
            return (
                first[1] + ratio * (second[1] - first[1]),
                first[2] + ratio * (second[2] - first[2]),
            )
    raise AssertionError("validated schedule interpolation fell through")


def attach_evaluator_dynamic_proximity(
    telemetry: dict[str, Any],
    pedestrian_schedule: Path,
    environment_telemetry: dict[str, Any],
) -> None:
    """Verify interaction candidates using environment truth after the run.

    This function is called only by the evaluator. Its results annotate an
    already completed Nav2 run; neither the goal nor any product command can
    depend on the schedule.
    """

    if environment_telemetry.get("collector_role") != (
        "evaluator_only_no_robot_control"
    ):
        raise ValueError("environment telemetry lacks evaluator-only role boundary")
    if environment_telemetry.get("control_topics_published") != []:
        raise ValueError("environment truth collector published a control topic")
    if environment_telemetry.get("product_actions_created") != []:
        raise ValueError("environment truth collector created a product action")
    samples = environment_telemetry.get("pedestrian_status_samples")
    if not isinstance(samples, list) or not samples:
        raise ValueError("environment telemetry has no active pedestrian samples")
    valid_samples = [
        row
        for row in samples
        if isinstance(row, dict)
        and isinstance(row.get("observation_ros_time_ns"), int)
        and isinstance(row.get("schedule_elapsed_s"), (int, float))
        and row.get("pedestrian_count") == 8
    ]
    if not valid_samples:
        raise ValueError("environment telemetry has no valid eight-pedestrian sample")

    schedule = _read(pedestrian_schedule)
    if schedule.get("access") != "environment_driver_only_not_robot_control":
        raise ValueError("pedestrian schedule lacks evaluator-only access boundary")
    pedestrians = schedule.get("pedestrians")
    if not isinstance(pedestrians, list) or len(pedestrians) != 8:
        raise ValueError("formal dynamic acceptance requires eight scheduled pedestrians")
    verified = 0
    minimum_clearance = math.inf
    annotated: list[dict[str, Any]] = []
    candidates = telemetry.get("dynamic_interaction_candidates", [])
    source_start = telemetry.get("source_fixed_start_pose")
    if not isinstance(source_start, list) or len(source_start) != 3:
        raise ValueError("runtime telemetry lacks the public-to-map start transform")
    start_x, start_y, start_yaw = (float(value) for value in source_start)
    cosine, sine = math.cos(start_yaw), math.sin(start_yaw)
    if not isinstance(candidates, list):
        candidates = []
    for raw in candidates:
        if not isinstance(raw, dict):
            continue
        observation_ns = raw.get("observation_ros_time_ns")
        pose = raw.get("vehicle_pose_map")
        if not isinstance(observation_ns, int) or not isinstance(pose, list):
            continue
        if len(pose) != 2:
            continue
        nearest_sample = min(
            valid_samples,
            key=lambda row: abs(row["observation_ros_time_ns"] - observation_ns),
        )
        alignment_error_s = abs(
            nearest_sample["observation_ros_time_ns"] - observation_ns
        ) / 1.0e9
        if alignment_error_s > 0.5:
            continue
        elapsed = float(nearest_sample["schedule_elapsed_s"])
        distances = []
        for pedestrian in pedestrians:
            source_x, source_y = _interpolate_schedule(
                pedestrian.get("waypoints", []), float(elapsed)
            )
            dx, dy = source_x - start_x, source_y - start_y
            x = cosine * dx + sine * dy
            y = -sine * dx + cosine * dy
            radius = float(pedestrian.get("radius_m", 0.25))
            clearance = max(
                0.0,
                math.hypot(x - float(pose[0]), y - float(pose[1])) - radius,
            )
            distances.append((clearance, pedestrian.get("object_id")))
        clearance, object_id = min(distances)
        minimum_clearance = min(minimum_clearance, clearance)
        is_verified = clearance <= 3.0
        verified += int(is_verified)
        annotated.append(
            {
                **raw,
                "schedule_elapsed_s_evaluator_only": elapsed,
                "schedule_alignment_error_s_evaluator_only": alignment_error_s,
                "nearest_pedestrian_id_evaluator_only": object_id,
                "nearest_pedestrian_clearance_m_evaluator_only": clearance,
                "evaluator_verified": is_verified,
            }
        )
    telemetry["dynamic_interaction_candidates"] = annotated
    telemetry["evaluator_verified_dynamic_interaction_count"] = verified
    telemetry["minimum_evaluator_pedestrian_clearance_m"] = (
        minimum_clearance if math.isfinite(minimum_clearance) else None
    )
    telemetry["evaluator_truth_used_after_run_only"] = True
    telemetry["evaluator_truth_process_isolated"] = True
    telemetry["active_pedestrian_count"] = environment_telemetry.get(
        "active_pedestrian_count"
    )
    telemetry["collision_count"] = environment_telemetry.get("collision_count")
    telemetry["environment_truth_collector"] = environment_telemetry
    product_counts = telemetry.setdefault("topic_sample_counts", {})
    environment_counts = environment_telemetry.get("topic_sample_counts", {})
    if isinstance(product_counts, dict) and isinstance(environment_counts, dict):
        product_counts.update(environment_counts)
    telemetry["dynamic_environment_contract"] = schedule.get(
        "acceptance_environment"
    )


def saved_map_preflight(
    episode_manifest: Path,
    saved_map_artifact_dir: Path,
) -> tuple[bool, str | None, dict[str, Any] | None]:
    try:
        from sanitation_formal_campus_integration.map_lifecycle_core import (
            load_campus_map_contract,
            validate_saved_map_artifact,
        )

        contract = load_campus_map_contract(episode_manifest)
        evidence = validate_saved_map_artifact(saved_map_artifact_dir, contract)
    except Exception as exc:  # exact package/runtime errors belong in evidence
        return False, str(exc), None
    return True, None, evidence


def evaluate(
    telemetry: dict[str, Any],
    *,
    saved_map_valid: bool,
    saved_map_error: str | None = None,
    frozen_session_valid: bool = True,
    runtime_closure_valid: bool = True,
) -> dict[str, Any]:
    topic_samples = telemetry.get("topic_sample_counts", {})
    command_publishers = telemetry.get("command_topic_publishers", {})
    dynamic_environment = telemetry.get("dynamic_environment_contract", {})
    runtime_build = telemetry.get("runtime_build_manifest", {})
    runtime_world = telemetry.get("runtime_world_manifest", {})
    source_only_files = {
        item.get("source")
        for item in runtime_build.get("source_only_runtime_files", [])
        if isinstance(item, dict)
        and len(str(item.get("source_sha256", ""))) == 64
    } if isinstance(runtime_build, dict) else set()
    odom_trajectory = _trajectory(
        telemetry.get("mission_odom_trajectory_xy_m"), "mission odom trajectory"
    )
    map_trajectory = _trajectory(
        telemetry.get("mission_map_trajectory_xy_m"), "mission map trajectory"
    )
    recomputed_travel = sum(
        math.dist(left, right)
        for left, right in zip(odom_trajectory, odom_trajectory[1:])
    )
    checks = {
        "frozen_snapshot_session_valid": frozen_session_valid,
        "unified_non_symlink_runtime_closure_valid": runtime_closure_valid,
        "saved_map_lifecycle_artifact_valid": saved_map_valid,
        "runtime_bound_to_current_checkout": isinstance(runtime_build, dict)
        and runtime_build.get("current_source_build_completed") is True
        and len(runtime_build.get("source_install_bindings", [])) >= 11
        and all(
            item.get("matches") is True
            for item in runtime_build.get("source_install_bindings", [])
        )
        and all(runtime_build.get("required_plugin_libraries", {}).values())
        and {
            "starter_ws/src/sanitation_formal_campus_integration/setup.py",
            "scripts/collect_formal_map_lifecycle_runtime.py",
            "scripts/run_formal_dynamic_obstacle_avoidance.sh",
            "scripts/collect_formal_dynamic_obstacle_avoidance_runtime.py",
            "scripts/collect_formal_dynamic_environment_runtime.py",
            "scripts/validate_formal_dynamic_obstacle_avoidance.py",
            "scripts/generate_formal_dynamic_runtime_build_manifest.py",
            "scripts/prepare_formal_dynamic_obstacle_schedule.py",
            "scripts/prepare_formal_dynamic_runtime_world.py",
        }.issubset(source_only_files),
        "cleaning_world_preserved_and_contact_instrumented": isinstance(
            runtime_world, dict
        )
        and runtime_world.get("world_preserved_except_contact_system") is True
        and runtime_world.get("contact_system_plugin_count") == 1
        and runtime_world.get("pedestrian_model_count") == 8
        and len(runtime_world.get("source_world_sha256", "")) == 64
        and len(runtime_world.get("runtime_world_sha256", "")) == 64,
        "formal_vehicle_used": telemetry.get("vehicle_profile")
        == "formal_transport_stowed",
        "saved_map_nav2_command_chain_used": telemetry.get("command_chain")
        == [
            "/cmd_vel_nav",
            "/cmd_vel_smoothed",
            "/cmd_vel_gate",
            "/base_controller/cmd_vel",
        ],
        "exactly_eight_random_pedestrians_active": telemetry.get(
            "active_pedestrian_count"
        )
        == 8
        and telemetry.get("expected_pedestrian_count") == 8,
        "runtime_randomized_pedestrian_environment_recorded": isinstance(
            dynamic_environment, dict
        )
        and isinstance(dynamic_environment.get("seed"), int)
        and dynamic_environment.get("randomized_each_run_unless_seed_pinned") is True
        and int(dynamic_environment.get("mission_corridor_crossing_count", 0)) >= 3
        and dynamic_environment.get("pedestrian_model_ids")
        == runtime_world.get("pedestrian_model_ids")
        and dynamic_environment.get("product_control_access_prohibited") is True,
        "goal_uses_public_manifest_not_environment_truth": telemetry.get(
            "mission_goal_source"
        )
        == (
            "public_manifest_fixed_start_transformed_to_saved_map_local_plus_nominal_leg"
        ),
        "product_control_truth_free": telemetry.get(
            "product_control_reads_pedestrian_truth"
        )
        is False
        and telemetry.get("control_truth_topics_subscribed") == [],
        "live_truth_subscriber_audit_isolated_to_evaluator": telemetry.get(
            "control_prohibited_truth_topic_subscriber_audit"
        )
        == {
            "/scenario/environment/pedestrian_driver/status": [
                "/formal_dynamic_environment_truth_collector"
            ]
        },
        "evaluator_truth_process_isolated": telemetry.get(
            "evaluator_truth_process_isolated"
        )
        is True,
        "no_pedestrian_velocity_estimation": telemetry.get(
            "pedestrian_velocity_estimation_used"
        )
        is False,
        "single_collision_monitor": telemetry.get("collision_monitor_node_count")
        == 1,
        "single_final_command_publisher": telemetry.get(
            "final_command_publisher_count"
        )
        == 1,
        "final_publisher_is_safety_manager": telemetry.get(
            "final_command_publisher_node"
        )
        == "/whole_vehicle_safety_manager",
        "collision_monitor_owns_checked_command": command_publishers.get(
            "/cmd_vel_gate"
        )
        == ["/collision_monitor"],
        "safety_manager_continuously_permitted": int(
            telemetry.get("mission_safety_sample_count", 0)
        )
        >= 5
        and int(telemetry.get("mission_safety_inhibit_sample_count", -1)) == 0,
        "bms_fault_feed_fresh_and_clear": int(
            telemetry.get("bms_fault_clear_sample_count", 0)
        )
        >= 5,
        "traction_permit_feed_fresh_and_true": int(
            telemetry.get("traction_permitted_sample_count", 0)
        )
        >= 5,
        "a300_bms_node_observed": "/a300_bms_simulator"
        in telemetry.get("runtime_node_graph", []),
        "a300_drivetrain_adapter_observed": "/a300_drivetrain_command_adapter"
        in telemetry.get("runtime_node_graph", []),
        "local_ekf_is_unique_selected_odom_publisher": telemetry.get(
            "selected_odom_publishers"
        )
        == ["/local_ekf"],
        "a300_bridge_is_unique_raw_odom_publisher": telemetry.get(
            "raw_odom_publishers"
        )
        == ["/a300_drivetrain_bridge"],
        "odom_to_base_footprint_tf_observed": int(
            topic_samples.get("/tf:odom->base_footprint", 0)
        )
        >= 5,
        "saved_map_localization_observed": int(
            topic_samples.get("/amcl_pose", 0)
        )
        >= 5,
        "required_runtime_topics_observed": all(
            int(topic_samples.get(topic, 0)) > 0
            for topic in (
                "/odom",
                "/odom/unfiltered",
                "/scan/navigation",
                "/sensors/lidar_3d/points",
                "/cmd_vel_nav",
                "/cmd_vel_smoothed",
                "/cmd_vel_gate",
                "/base_controller/cmd_vel",
                "/collision_monitor_state",
                "/safety/status",
                "/model/tzcup_formal_sanitation_vehicle/a300_drivetrain/status",
            )
        ),
        "nav2_goal_completed": telemetry.get("nav2_goal_succeeded") is True,
        "motion_metrics_exclude_pre_goal_spawn_and_startup": telemetry.get(
            "mission_metrics_begin_at_goal_submission"
        )
        is True,
        "vehicle_physically_travelled": float(
            telemetry.get("physical_travel_distance_m", 0.0)
        )
        >= 5.0,
        "physical_mileage_recomputes_from_recorded_odom_trajectory": len(
            odom_trajectory
        )
        >= 5
        and int(telemetry.get("odom_pose_sample_count", 0)) == len(odom_trajectory)
        and math.isclose(
            recomputed_travel,
            float(telemetry.get("physical_travel_distance_m", math.nan)),
            rel_tol=1.0e-9,
            abs_tol=1.0e-6,
        ),
        "map_trajectory_is_recorded_for_geofence_and_detour_evidence": len(
            map_trajectory
        )
        >= 5
        and int(telemetry.get("map_pose_sample_count", 0)) == len(map_trajectory),
        "vehicle_performed_avoidance_detour": float(
            telemetry.get("maximum_cross_track_detour_m", 0.0)
        )
        >= 0.5,
        "dynamic_interaction_observed": int(
            telemetry.get("evaluator_verified_dynamic_interaction_count", 0)
        )
        >= 1,
        "collision_monitor_intervened": int(
            telemetry.get("collision_monitor_intervention_count", 0)
        )
        >= 1,
        "zero_physical_collisions": int(telemetry.get("collision_count", -1)) == 0,
        "zero_geofence_violations": int(
            telemetry.get("geofence_violation_count", -1)
        )
        == 0,
        "map_pose_remained_inside_public_geofence": int(
            telemetry.get("map_pose_sample_count", 0)
        )
        >= 5,
    }
    passed = all(checks.values())
    blockers = [name for name, value in checks.items() if not value]
    if saved_map_error:
        blockers.insert(0, f"saved_map_preflight: {saved_map_error}")
    return {
        "report_id": "tzcup_formal_dynamic_obstacle_avoidance_acceptance_v1",
        "status": PASSED_STATUS if passed else BLOCKED_STATUS,
        "passed": passed,
        "checks": checks,
        "blockers": blockers,
        "metrics": telemetry,
        "claim_boundary": (
            "This gate requires the formal transport-stowed vehicle to execute a "
            "saved-map Nav2 goal in Gazebo with eight moving pedestrians. Product "
            "control may use current scan observations only, treats a detected person "
            "as a current static obstacle, and performs no pedestrian velocity "
            "estimation. Pedestrian schedule/entity truth is evaluator-only and cannot "
            "select goals or commands. Static tests or a startup-only graph cannot pass."
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episode-manifest", type=Path, required=True)
    parser.add_argument("--saved-map-artifact-dir", type=Path, required=True)
    parser.add_argument("--telemetry", type=Path)
    parser.add_argument("--pedestrian-schedule", type=Path)
    parser.add_argument("--environment-telemetry", type=Path)
    parser.add_argument("--snapshot-manifest", type=Path, required=True)
    parser.add_argument("--session-status", type=Path, required=True)
    parser.add_argument("--runtime-binding", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--preflight-only", action="store_true")
    args = parser.parse_args()
    valid, error, saved_map_evidence = saved_map_preflight(
        args.episode_manifest,
        args.saved_map_artifact_dir,
    )
    try:
        source_binding = frozen_session_binding(
            args.snapshot_manifest,
            args.session_status,
        )
        session_valid = True
        session_error = None
    except Exception as exc:
        source_binding = None
        session_valid = False
        session_error = str(exc)
    try:
        runtime_binding = load_binding(args.runtime_binding)
        bound_snapshot = runtime_binding["acceptance_session_binding"].get("snapshot")
        expected_snapshot = (
            {
                key: source_binding[key]
                for key in (
                    "snapshot_manifest_sha256",
                    "source_inventory_sha256",
                    "expanded_urdf_sha256",
                )
            }
            if source_binding is not None
            else None
        )
        if bound_snapshot != expected_snapshot:
            raise ValueError("runtime closure gate is bound to another snapshot")
        bound_session = runtime_binding["acceptance_session_binding"]
        if (
            bound_session.get("session_started_epoch_ns")
            != source_binding["session_started_epoch_ns"]
            or bound_session.get("session_manifest")
            != str(args.session_status.resolve())
            or bound_session.get("session_manifest_sha256")
            != _sha256(args.session_status)
            or bound_session.get("snapshot_current_source_verified") is not True
        ):
            raise ValueError("runtime closure gate is bound to another acceptance session")
        closure_valid = True
        closure_error = None
    except Exception as exc:
        runtime_binding = None
        closure_valid = False
        closure_error = str(exc)
    if args.preflight_only and valid and session_valid and closure_valid:
        return 0
    telemetry = _read(args.telemetry) if args.telemetry and args.telemetry.is_file() else {}
    if telemetry and args.pedestrian_schedule and args.environment_telemetry:
        try:
            attach_evaluator_dynamic_proximity(
                telemetry,
                args.pedestrian_schedule,
                _read(args.environment_telemetry),
            )
        except Exception as exc:
            telemetry["evaluator_verified_dynamic_interaction_count"] = 0
            telemetry["dynamic_interaction_evaluator_error"] = str(exc)
    report = evaluate(
        telemetry,
        saved_map_valid=valid,
        saved_map_error=error,
        frozen_session_valid=session_valid,
        runtime_closure_valid=closure_valid,
    )
    if session_error:
        report["blockers"].insert(0, f"frozen_session_preflight: {session_error}")
    report["source_binding"] = source_binding
    report["runtime_gate_binding"] = runtime_binding
    if closure_error:
        report["blockers"].insert(0, f"runtime_closure_preflight: {closure_error}")
    report["saved_map_lifecycle_evidence"] = saved_map_evidence
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(args.output)
    return 0 if report["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
