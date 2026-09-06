#!/usr/bin/env python3
"""Aggregate one immutable live episode; reject defaults and inferred proof."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

from formal_runtime_gate_binding import RuntimeGateError, load_binding

from collect_formal_single_episode_cleaning_mission import (
    CONTROL_PROHIBITED_TRUTH_TOPICS,
    REQUIRED_RUNTIME_NODES,
    directory_descriptor,
    file_descriptor,
    sha256_file,
)
from generate_formal_same_map_baseline import (
    BaselineError,
    REPORT_ID as BASELINE_REPORT_ID,
    validate as validate_same_map_baseline,
)


class AggregateError(RuntimeError):
    pass


COMPETITION_EFFICIENCY_THRESHOLD_M2_H = 3500.0
COMPETITION_EFFICIENCY_KEYS = {
    "threshold_m2_h",
    "covered_area_m2",
    "actual_duration_sec",
    "measured_net_efficiency_m2_h",
    "recomputed_net_efficiency_m2_h",
    "return_distance_included",
    "passed",
}


def _object(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise AggregateError(f"JSON root must be object: {path}")
    return value


def _mapping(value: Any, label: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise AggregateError(f"{label} must be an object")
    return value


def _list(value: Any, label: str) -> list[Any]:
    if not isinstance(value, list):
        raise AggregateError(f"{label} must be an array")
    return value


def _source_geofence_polygon(field: dict[str, Any]) -> list[Any]:
    """Read only the source-world geofence, with strict legacy semantics."""

    source_geofence = field.get("source_world_geofence")
    if source_geofence is not None:
        source_geofence = _mapping(
            source_geofence, "episode.field.source_world_geofence"
        )
        if source_geofence.get("frame_id") != "source_world":
            raise AggregateError("episode source-world geofence frame is invalid")
        return _list(
            source_geofence.get("polygon_m"),
            "episode.field.source_world_geofence.polygon_m",
        )
    if field.get("geofence_frame") not in {"map", "source_world"}:
        raise AggregateError("episode legacy geofence frame is invalid")
    return _list(
        field.get("geofence_polygon_m"), "episode.field.geofence_polygon_m"
    )


def _number(row: dict[str, Any], key: str, label: str) -> float:
    if key not in row or isinstance(row[key], bool):
        raise AggregateError(f"{label}.{key} missing")
    try:
        value = float(row[key])
    except (TypeError, ValueError) as exc:
        raise AggregateError(f"{label}.{key} is not numeric") from exc
    if not math.isfinite(value):
        raise AggregateError(f"{label}.{key} is not finite")
    return value


def _strict_number(row: dict[str, Any], key: str, label: str) -> float:
    value = row.get(key)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise AggregateError(f"{label}.{key} must be a numeric JSON scalar")
    return _number(row, key, label)


def _integer(row: dict[str, Any], key: str, label: str) -> int:
    if key not in row or isinstance(row[key], bool) or not isinstance(row[key], int):
        raise AggregateError(f"{label}.{key} must be an integer")
    return row[key]


def _true(row: dict[str, Any], key: str, label: str) -> None:
    if row.get(key) is not True:
        raise AggregateError(f"{label}.{key} must be explicitly true")


def _same_path(observed: Any, expected: str, label: str) -> None:
    if not isinstance(observed, str) or Path(observed).resolve() != Path(expected).resolve():
        raise AggregateError(f"runtime path binding mismatch: {label}")


def canonical_session_id(session: dict[str, Any]) -> str:
    snapshot = session.get("snapshot", {})
    digest = snapshot.get("snapshot_manifest_sha256") if isinstance(snapshot, dict) else None
    started = session.get("started_epoch_ns")
    if not isinstance(digest, str) or len(digest) != 64 or not isinstance(started, int):
        raise AggregateError("formal acceptance session has no frozen identity")
    return f"{digest}:{started}"


def _verify_input_binding(raw: dict[str, Any]) -> dict[str, dict[str, Any]]:
    ledger = _mapping(raw.get("input_binding"), "input_binding")
    binding_path = Path(str(ledger.get("path", "")))
    if not binding_path.is_file() or sha256_file(binding_path) != ledger.get("sha256"):
        raise AggregateError("immutable input binding file hash mismatch")
    binding = _object(binding_path)
    if binding.get("artifact_kind") != "single_episode_immutable_input_binding":
        raise AggregateError("invalid immutable input binding kind")
    artifacts = _mapping(binding.get("artifacts"), "input_binding.artifacts")
    if artifacts != ledger.get("artifacts"):
        raise AggregateError("raw input ledger differs from frozen binding")
    required = {
        "episode_manifest", "evaluator_episode_manifest", "evaluator_ground_truth",
        "world", "pedestrian_schedule", "session_status", "same_map_baseline",
        "policy_checkpoint", "runtime_binding", "saved_map", "perception_artifacts",
    }
    if set(artifacts) != required:
        raise AggregateError("immutable input ledger has missing or unexpected artifacts")
    for name, expected in artifacts.items():
        if not isinstance(expected, dict):
            raise AggregateError(f"invalid descriptor: {name}")
        path = Path(str(expected.get("path", "")))
        if expected.get("kind") == "file":
            actual = file_descriptor(path)
        elif expected.get("kind") == "directory":
            actual = directory_descriptor(path)
        else:
            raise AggregateError(f"invalid descriptor kind: {name}")
        if actual != expected:
            raise AggregateError(f"input hash mismatch: {name}")
    return artifacts


def _polygon_area(points: list[Any]) -> float:
    if len(points) < 3:
        raise AggregateError("field geofence polygon has fewer than three vertices")
    parsed: list[tuple[float, float]] = []
    for point in points:
        if not isinstance(point, list) or len(point) != 2:
            raise AggregateError("field geofence vertex is malformed")
        x, y = float(point[0]), float(point[1])
        if not math.isfinite(x) or not math.isfinite(y):
            raise AggregateError("field geofence vertex is not finite")
        parsed.append((x, y))
    return abs(sum(
        parsed[i][0] * parsed[(i + 1) % len(parsed)][1]
        - parsed[(i + 1) % len(parsed)][0] * parsed[i][1]
        for i in range(len(parsed))
    )) / 2.0


def _trajectory(value: Any, label: str) -> list[tuple[float, float]]:
    rows = _list(value, label)
    points: list[tuple[float, float]] = []
    for row in rows:
        if not isinstance(row, list) or len(row) != 2:
            raise AggregateError(f"{label} contains a malformed point")
        try:
            point = (float(row[0]), float(row[1]))
        except (TypeError, ValueError) as exc:
            raise AggregateError(f"{label} contains a nonnumeric point") from exc
        if not all(math.isfinite(item) for item in point):
            raise AggregateError(f"{label} contains a nonfinite point")
        points.append(point)
    if len(points) < 2:
        raise AggregateError(f"{label} has fewer than two poses")
    return points


def _trajectory_distance(points: list[tuple[float, float]]) -> float:
    return sum(math.dist(left, right) for left, right in zip(points, points[1:]))


def _validate_grasp_result(
    result: dict[str, Any], truth_cube: dict[str, Any], session_start_ns: int,
) -> dict[str, Any]:
    target_id = str(truth_cube["object_id"])
    if result.get("schema_version") != 2 or result.get("target_id") != target_id:
        raise AggregateError(f"invalid grasp result identity/schema: {target_id}")
    if result.get("verified_in_bin") is not True or result.get("reason") != "physical_cube_verified_in_bin":
        raise AggregateError(f"grasp was not physically verified: {target_id}")
    if _integer(result, "collector_received_epoch_ns", f"grasp[{target_id}]") <= session_start_ns:
        raise AggregateError(f"grasp evidence predates session: {target_id}")
    evidence = _mapping(result.get("evidence"), f"grasp[{target_id}].evidence")
    if evidence.get("truth_used_for_control") is not False:
        raise AggregateError(f"grasp truth boundary missing: {target_id}")
    if evidence.get("simulator_entity_identity_in_request") is not False:
        raise AggregateError(f"simulator identity entered grasp request: {target_id}")
    if evidence.get("planning_backend") != "MoveGroup_action_GetPositionIK_GetCartesianPath":
        raise AggregateError(f"unexpected grasp planning backend: {target_id}")
    perceived = _mapping(evidence.get("perceived_target"), f"grasp[{target_id}].perceived_target")
    if perceived.get("target_id") != target_id or perceived.get("material") != "unknown":
        raise AggregateError(f"grasp was not conditioned on the truth-free target: {target_id}")
    wrist = _mapping(evidence.get("wrist_near_field_recheck"), f"grasp[{target_id}].wrist")
    _true(wrist, "accepted", f"grasp[{target_id}].wrist")
    sequence = _list(evidence.get("sequence"), f"grasp[{target_id}].sequence")
    steps = {
        str(row.get("step")): row for row in sequence if isinstance(row, dict) and row.get("step")
    }
    required_plans = {
        "TARGET_CONDITIONED_PREGRASP",
        "WRIST_REFINED_PREGRASP",
        "LINEAR_CONTACT_APPROACH",
        "LINEAR_COLLISION_CHECKED_LIFT",
        "COLLISION_CHECKED_DEPOSIT",
        "COLLISION_CHECKED_BIN_RETREAT",
    }
    if not required_plans.issubset(steps):
        raise AggregateError(f"grasp trajectory proof incomplete: {target_id}")
    for step in required_plans:
        row = _mapping(steps[step], f"grasp[{target_id}].{step}")
        if row.get("collision_checked") is not True or row.get("ik_validated") is not True:
            raise AggregateError(f"IK/collision proof missing for {target_id}:{step}")
        if not isinstance(row.get("target_pose"), dict):
            raise AggregateError(f"target-conditioned pose missing for {target_id}:{step}")
    hold = _mapping(evidence.get("physical_hold_after_lift"), f"grasp[{target_id}].hold")
    if hold.get("attachment_state_ack") is not True and hold.get("persistent_dual_finger_contact") is not True:
        raise AggregateError(f"physical grasp hold missing: {target_id}")
    dry = _mapping(evidence.get("dry_bin_verification"), f"grasp[{target_id}].dry_bin")
    _true(dry, "physical_monitor_confirmed", f"grasp[{target_id}].dry_bin")
    _true(dry, "dynamic_payload_increment_confirmed", f"grasp[{target_id}].dry_bin")
    measured = _number(dry, "measured_increment_kg", f"grasp[{target_id}].dry_bin")
    expected = _number(truth_cube, "mass_kg", f"truth_cube[{target_id}]")
    if abs(measured - expected) > 1.0e-5:
        raise AggregateError(f"grasp mass increment differs from episode truth: {target_id}")
    if dry.get("pre_grasp_material") != "unknown":
        raise AggregateError(f"pre-grasp material truth leak: {target_id}")
    return {
        "target_id": target_id,
        "measured_increment_kg": measured,
        "truth_mass_kg": expected,
        "truth_material": truth_cube["material"],
        "contained_object_count": _integer(
            dry, "contained_object_count", f"grasp[{target_id}].dry_bin"
        ),
        "target_conditioned_moveit_ik_collision_verified": True,
        "wrist_recheck_verified": True,
    }


def aggregate(raw_path: Path) -> dict[str, Any]:
    raw = _object(raw_path)
    if raw.get("artifact_kind") != "single_live_episode_raw_collection" or raw.get("schema_version") != 2:
        raise AggregateError("raw artifact is not schema-v2 single live episode evidence")
    identity = _mapping(raw.get("run_identity"), "run_identity")
    required_identity = (
        "session_id", "episode_id", "episode_seed", "runtime_id",
        "gazebo_process_id", "ros_domain_id", "gz_partition",
    )
    if any(identity.get(key) in (None, "") for key in required_identity):
        raise AggregateError("raw run identity is incomplete")
    artifacts = _verify_input_binding(raw)

    sources = _list(raw.get("metric_sources"), "metric_sources")
    expected_metrics = set((*{
        "planner", "mission_complete", "trajectory", "grasp_result", "odometry"
    }, *{
        "ground_dirt", "water", "dry_bin", "pedestrians", "collision",
        "front_bumper", "rear_bumper"
    }))
    observed_metrics: set[str] = set()
    for row in sources:
        row = _mapping(row, "metric_source")
        for key in required_identity:
            if row.get(key) != identity.get(key):
                raise AggregateError(f"metric source identity mismatch: {row.get('metric')}:{key}")
        if row.get("source_class") not in {"product", "evaluator_truth"}:
            raise AggregateError("historical or unclassified metric source prohibited")
        if _integer(row, "sample_count", f"source[{row.get('metric')}]") <= 0:
            raise AggregateError(f"metric source has no live samples: {row.get('metric')}")
        metric = str(row.get("metric", ""))
        if metric in observed_metrics:
            raise AggregateError(f"duplicate metric source ledger row: {metric}")
        observed_metrics.add(metric)
    if observed_metrics != expected_metrics:
        raise AggregateError("live metric source ledger is incomplete or unexpected")

    def load(name: str) -> dict[str, Any]:
        return _object(Path(artifacts[name]["path"]))

    episode = load("episode_manifest")
    evaluator_manifest = load("evaluator_episode_manifest")
    ground_truth = load("evaluator_ground_truth")
    session = load("session_status")
    baseline = load("same_map_baseline")
    policy = load("policy_checkpoint")
    try:
        runtime_binding = load_binding(Path(artifacts["runtime_binding"]["path"]))
    except RuntimeGateError as exc:
        raise AggregateError(f"runtime gate binding invalid: {exc}") from exc
    bound_session = _mapping(
        runtime_binding.get("acceptance_session_binding"),
        "runtime_binding.acceptance_session_binding",
    )
    if (
        bound_session.get("snapshot") != session.get("snapshot")
        or bound_session.get("session_started_epoch_ns")
        != session.get("started_epoch_ns")
    ):
        raise AggregateError("runtime gate is bound to another acceptance session")
    schedule = load("pedestrian_schedule")

    episode_id = identity["episode_id"]
    map_id = episode.get("map_id")
    for label, row in (
        ("public manifest", episode), ("evaluator manifest", evaluator_manifest),
        ("ground truth", ground_truth),
    ):
        if row.get("episode_id") != episode_id or row.get("map_id") != map_id:
            raise AggregateError(f"{label} identity differs from live collector")
    if evaluator_manifest.get("truth_boundary", {}).get("control_use_prohibited") is not True:
        raise AggregateError("evaluator manifest does not prohibit product truth use")
    if ground_truth.get("control_use_prohibited") is not True:
        raise AggregateError("ground truth does not prohibit product control use")
    if evaluator_manifest.get("world_sha256") != artifacts["world"]["sha256"]:
        raise AggregateError("world is not the evaluator-manifest-bound episode world")
    seeds = _mapping(evaluator_manifest.get("seeds"), "evaluator_manifest.seeds")
    if set(seeds) != {"layout", "dirt", "cubes", "pedestrians", "sensor"}:
        raise AggregateError("episode seed ledger is incomplete or unexpected")
    if any(isinstance(value, bool) or not isinstance(value, int) or value <= 0 for value in seeds.values()):
        raise AggregateError("episode seed ledger contains an invalid seed")
    if identity["episode_seed"] != seeds["dirt"]:
        raise AggregateError("runtime mission seed is not evaluator seeds.dirt")

    if canonical_session_id(session) != identity["session_id"]:
        raise AggregateError("formal session ID differs from live collector")
    if session.get("status") not in {
        "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING", "FORMAL_FINAL_ACCEPTANCE_SESSION_COMPLETE"
    }:
        raise AggregateError("formal acceptance session is not active/complete")
    session_start = _integer(session, "started_epoch_ns", "session")
    if identity.get("session_start_epoch_ns") != session_start:
        raise AggregateError("collector session-start binding mismatch")
    if _integer(raw, "created_epoch_ns", "raw") <= session_start or raw_path.stat().st_mtime_ns < session_start:
        raise AggregateError("raw collection predates frozen session")
    if raw.get("collector_ready_before_operator_start") is not True or raw.get("timed_out") is not False:
        raise AggregateError("collector did not freeze initial state before a completed mission")

    baseline_evidence = _mapping(baseline.get("evidence"), "same_map_baseline.evidence")
    snapshot_descriptor = _mapping(
        baseline_evidence.get("snapshot"), "same_map_baseline.evidence.snapshot"
    )
    try:
        validate_same_map_baseline(
            Path(artifacts["same_map_baseline"]["path"]),
            Path(artifacts["session_status"]["path"]),
            Path(str(snapshot_descriptor.get("path", ""))),
        )
    except BaselineError as exc:
        raise AggregateError(f"same-map FullCoverage baseline failed source validation: {exc}") from exc
    if baseline.get("report_id") != BASELINE_REPORT_ID or baseline.get("session_bound") is not True:
        raise AggregateError("FullCoverage baseline is not a session-bound formal report")
    if baseline.get("map_id") != map_id:
        raise AggregateError("FullCoverage baseline is for a different map")
    if (
        baseline.get("episode_id") != episode_id
        or baseline_evidence.get("episode_manifest", {}).get("sha256")
        != artifacts["episode_manifest"]["sha256"]
    ):
        raise AggregateError("FullCoverage baseline is not for this exact episode manifest")
    if baseline.get("status") != "FORMAL_FULL_COVERAGE_BASELINE_PASSED":
        raise AggregateError("FullCoverage baseline has no explicit passing status")
    for key in ("fixed_start_verified", "first_map_ignored_dirt", "saved_map_hard_restart_verified"):
        _true(baseline, key, "same_map_baseline")
    baseline_distance = _number(baseline, "successful_distance_m", "same_map_baseline")
    if baseline_distance <= 0.0:
        raise AggregateError("FullCoverage baseline distance must be positive")
    if baseline.get("planner") != "full_coverage" or baseline.get("truth_used_for_control") is not False:
        raise AggregateError("FullCoverage baseline planner/truth boundary is invalid")
    if baseline.get("return_distance_included") is not False:
        raise AggregateError("FullCoverage baseline must exclude return-home distance")
    competition_efficiency = _mapping(
        baseline.get("competition_efficiency"),
        "same_map_baseline.competition_efficiency",
    )
    if set(competition_efficiency) != COMPETITION_EFFICIENCY_KEYS:
        raise AggregateError("FullCoverage competition efficiency mapping is incomplete or unexpected")
    threshold = _strict_number(
        competition_efficiency,
        "threshold_m2_h",
        "same_map_baseline.competition_efficiency",
    )
    covered_area = _strict_number(
        competition_efficiency,
        "covered_area_m2",
        "same_map_baseline.competition_efficiency",
    )
    duration = _strict_number(
        competition_efficiency,
        "actual_duration_sec",
        "same_map_baseline.competition_efficiency",
    )
    measured_efficiency = _strict_number(
        competition_efficiency,
        "measured_net_efficiency_m2_h",
        "same_map_baseline.competition_efficiency",
    )
    reported_recomputed_efficiency = _strict_number(
        competition_efficiency,
        "recomputed_net_efficiency_m2_h",
        "same_map_baseline.competition_efficiency",
    )
    _true(competition_efficiency, "passed", "same_map_baseline.competition_efficiency")
    if competition_efficiency.get("return_distance_included") is not False:
        raise AggregateError("FullCoverage competition efficiency must exclude return-home distance")
    if not math.isclose(threshold, COMPETITION_EFFICIENCY_THRESHOLD_M2_H, abs_tol=1.0e-9):
        raise AggregateError("FullCoverage competition efficiency threshold must equal 3500 m2/h")
    if covered_area <= 0.0 or duration <= 0.0:
        raise AggregateError("FullCoverage competition coverage area/duration must be positive")
    recomputed_efficiency = covered_area / duration * 3600.0
    if not (
        math.isclose(measured_efficiency, recomputed_efficiency, rel_tol=1.0e-9, abs_tol=1.0e-6)
        and math.isclose(reported_recomputed_efficiency, recomputed_efficiency, rel_tol=1.0e-9, abs_tol=1.0e-6)
    ):
        raise AggregateError("FullCoverage competition efficiency differs from area/duration recomputation")
    if measured_efficiency < COMPETITION_EFFICIENCY_THRESHOLD_M2_H:
        raise AggregateError("FullCoverage competition efficiency is below 3500 m2/h")
    comparison = _mapping(baseline.get("planner_comparison"), "same_map_baseline.planner_comparison")
    if (
        comparison.get("candidate_planner") != "q_learning"
        or comparison.get("metric") != "task_trajectory_length_m_excluding_return"
        or comparison.get("successful_distance_m") != baseline_distance
    ):
        raise AggregateError("FullCoverage planner comparison contract is invalid")

    if policy.get("policy") != "q_learning" or policy.get("truth_access_used") is not False:
        raise AggregateError("checkpoint is not the truth-free Q-learning policy")
    if not isinstance(policy.get("q_table"), dict) or not policy["q_table"]:
        raise AggregateError("Q-learning checkpoint has no learned state table")

    runtime_parameters = _mapping(raw.get("runtime_parameters"), "runtime_parameters")
    planner_parameters = _mapping(
        runtime_parameters.get("/formal_active_cleaning_policy_planner"),
        "planner runtime parameters",
    )
    _same_path(planner_parameters.get("policy_checkpoint"), artifacts["policy_checkpoint"]["path"], "policy_checkpoint")
    if planner_parameters.get("episode_seed") != identity["episode_seed"]:
        raise AggregateError("live planner episode_seed mismatch")
    if float(planner_parameters.get("maximum_task_distance_m", math.nan)) != baseline_distance:
        raise AggregateError("live planner distance budget differs from same-map baseline")
    perception_parameters = _mapping(
        runtime_parameters.get("/pc_open_vocab_product_adapter"),
        "perception runtime parameters",
    )
    _same_path(perception_parameters.get("artifact_root"), artifacts["perception_artifacts"]["path"], "perception artifacts")
    map_parameters = _mapping(
        runtime_parameters.get("/formal_map_lifecycle_manager"),
        "map lifecycle runtime parameters",
    )
    if map_parameters.get("mode") != "cleaning":
        raise AggregateError("live map lifecycle is not saved-map cleaning mode")
    _same_path(map_parameters.get("episode_manifest"), artifacts["episode_manifest"]["path"], "episode manifest")
    _same_path(map_parameters.get("artifact_directory"), artifacts["saved_map"]["path"], "saved map")

    graph = _mapping(raw.get("runtime_graph"), "runtime_graph")
    nodes = set(_list(graph.get("nodes"), "runtime_graph.nodes"))
    if graph.get("required_nodes_present") is not True or not REQUIRED_RUNTIME_NODES.issubset(nodes):
        raise AggregateError("required executable product node graph was not present")
    subscribers = _mapping(
        graph.get("control_prohibited_truth_topic_subscribers"),
        "runtime_graph.control_prohibited_truth_topic_subscribers",
    )
    if set(subscribers) != set(CONTROL_PROHIBITED_TRUTH_TOPICS):
        raise AggregateError("runtime truth-subscription audit is incomplete")
    for topic, rows in subscribers.items():
        if rows != ["/formal_single_episode_cleaning_collector"]:
            raise AggregateError(f"evaluator truth has a non-collector subscriber: {topic}:{rows}")

    field = _mapping(episode.get("field"), "episode.field")
    width = _number(field, "width_m", "episode.field")
    height = _number(field, "height_m", "episode.field")
    area = _number(field, "area_m2", "episode.field")
    polygon = _source_geofence_polygon(field)
    if episode.get("profile") != "formal" or area < 20000.0:
        raise AggregateError("episode is not a formal >=20000 m2 field")
    if abs(width * height - area) > 1.0e-6 or abs(_polygon_area(polygon) - area) > 1.0e-6:
        raise AggregateError("field dimensions/area/geofence disagree")

    counts = _mapping(episode.get("counts"), "episode.counts")
    cube_contract = _mapping(episode.get("cube_contract"), "episode.cube_contract")
    cubes = _list(ground_truth.get("discrete_cubes"), "ground_truth.discrete_cubes")
    if _integer(counts, "discrete_cubes", "episode.counts") != 20 or len(cubes) != 20:
        raise AggregateError("episode must explicitly contain exactly 20 cubes")
    if _number(cube_contract, "edge_m", "episode.cube_contract") != 0.03:
        raise AggregateError("episode cube contract is not exactly 3 cm")
    cube_by_id: dict[str, dict[str, Any]] = {}
    for index, value in enumerate(cubes):
        cube = _mapping(value, f"ground_truth.discrete_cubes[{index}]")
        target_id = str(cube.get("object_id", ""))
        if not target_id or target_id in cube_by_id:
            raise AggregateError("ground-truth cube IDs are missing or duplicated")
        if _number(cube, "edge_m", f"cube[{target_id}]") != 0.03:
            raise AggregateError(f"ground-truth cube is not 3 cm: {target_id}")
        if cube.get("material") not in {"paperboard", "PP", "PET", "aluminum"}:
            raise AggregateError(f"invalid randomized material: {target_id}")
        cube_by_id[target_id] = cube
    if {cube["material"] for cube in cubes} != {"paperboard", "PP", "PET", "aluminum"}:
        raise AggregateError("all four episode material classes must occur")

    schedule_pedestrians = _list(schedule.get("pedestrians"), "pedestrian_schedule.pedestrians")
    truth_pedestrians = _list(ground_truth.get("pedestrians"), "ground_truth.pedestrians")
    public_pedestrian_count = _integer(counts, "pedestrians", "episode.counts")
    if public_pedestrian_count <= 0 or len(schedule_pedestrians) != public_pedestrian_count or len(truth_pedestrians) != public_pedestrian_count:
        raise AggregateError("random pedestrian manifests/counts are absent or inconsistent")
    if episode.get("dynamic_pedestrians_present") is not True:
        raise AggregateError("public episode does not explicitly declare dynamic pedestrians")
    schedule_ids = {
        str(row.get("object_id", "")) for row in schedule_pedestrians if isinstance(row, dict)
    }
    truth_pedestrian_ids = {
        str(row.get("object_id", "")) for row in truth_pedestrians if isinstance(row, dict)
    }
    if (
        len(schedule_ids) != public_pedestrian_count
        or schedule_ids != truth_pedestrian_ids or "" in schedule_ids
    ):
        raise AggregateError("pedestrian schedule IDs differ from evaluator truth IDs")
    runtime_environment = _mapping(
        evaluator_manifest.get("runtime_environment"),
        "evaluator_manifest.runtime_environment",
    )
    if runtime_environment.get("pedestrian_schedule") != "environment/pedestrian_schedule.json":
        raise AggregateError("evaluator manifest does not bind the pedestrian schedule")

    product = _mapping(raw.get("product"), "product")
    planner = _mapping(product.get("planner_status"), "product.planner_status")
    if (
        planner.get("diagnostic_name") != "formal_active_cleaning_policy_planner"
        or planner.get("hardware_id") != "frozen_truth_free_q_policy"
        or planner.get("truth_used_for_control") != "false"
        or planner.get("product_inputs_fresh") != "true"
    ):
        raise AggregateError("live planner diagnostic does not prove truth-free fresh operation")
    if planner.get("state") != "COMPLETE" or product.get("mission_complete") is not True:
        raise AggregateError("live planner mission did not complete")
    if _integer(product, "trajectory_publish_count", "product") <= 0:
        raise AggregateError("planner published no trajectory")
    trajectory_evidence = _list(
        product.get("trajectory_evidence"), "product.trajectory_evidence"
    )
    if len(trajectory_evidence) != _integer(
        product, "trajectory_publish_count", "product"
    ):
        raise AggregateError("trajectory evidence count differs from published count")
    for index, evidence in enumerate(trajectory_evidence):
        evidence = _mapping(evidence, f"trajectory_evidence[{index}]")
        points = _trajectory(
            evidence.get("trajectory_xy_m"), f"trajectory_evidence[{index}].points"
        )
        if evidence.get("frame_id") != "map" or evidence.get("pose_count") != len(points):
            raise AggregateError("planner trajectory has invalid frame or pose count")
    planner_samples = _list(
        product.get("planner_status_samples"), "product.planner_status_samples"
    )
    observed_progress = [
        _number(
            _mapping(row, "planner_status_sample"),
            "observed_ratio",
            "planner_status_sample",
        )
        for row in planner_samples
    ]
    if (
        len(observed_progress) < 2
        or observed_progress[0] >= 0.95
        or observed_progress[-1] < 0.95
        or any(
            right + 1.0e-6 < left
            for left, right in zip(observed_progress, observed_progress[1:])
        )
    ):
        raise AggregateError(
            "planner observed-area progress evidence is incomplete or nonmonotonic"
        )
    observed_ratio = _number(planner, "observed_ratio", "planner_status")
    if not math.isclose(observed_ratio, observed_progress[-1], abs_tol=1.0e-6):
        raise AggregateError("terminal observed ratio differs from progress evidence")
    task_trajectory = _trajectory(
        product.get("task_odom_trajectory_xy_m"), "product.task_odom_trajectory"
    )
    return_trajectory = _trajectory(
        product.get("return_odom_trajectory_xy_m"), "product.return_odom_trajectory"
    )
    mission_distance = _trajectory_distance(task_trajectory)
    return_distance = _trajectory_distance(return_trajectory)
    reported_mission_distance = _number(
        planner, "task_distance_m_excluding_return", "planner_status"
    )
    reported_return_distance = _number(
        planner, "return_distance_m", "planner_status"
    )
    for measured, reported, label in (
        (mission_distance, reported_mission_distance, "task"),
        (return_distance, reported_return_distance, "return"),
    ):
        if not math.isclose(measured, reported, rel_tol=0.01, abs_tol=0.5):
            raise AggregateError(
                f"{label} mileage differs from recorded odom trajectory"
            )

    raw_grasps = _list(product.get("grasp_results"), "product.grasp_results")
    successful_by_id: dict[str, list[dict[str, Any]]] = {target_id: [] for target_id in cube_by_id}
    for item in raw_grasps:
        result = _mapping(item, "grasp_result")
        target_id = str(result.get("target_id", ""))
        if target_id not in cube_by_id:
            raise AggregateError(f"grasp result target is outside episode truth IDs: {target_id}")
        if result.get("verified_in_bin") is True:
            successful_by_id[target_id].append(result)
    if any(len(rows) != 1 for rows in successful_by_id.values()):
        raise AggregateError("each episode cube must have exactly one successful physical result")
    grasp_evidence = [
        _validate_grasp_result(rows[0], cube_by_id[target_id], session_start)
        for target_id, rows in sorted(successful_by_id.items())
    ]
    if set(product.get("successful_grasp_target_ids", [])) != set(cube_by_id):
        raise AggregateError("collector successful target IDs differ from episode cube IDs")
    if sorted(row["contained_object_count"] for row in grasp_evidence) != list(range(1, 21)):
        raise AggregateError("per-grasp dry-bin count progression is not exactly 1..20")

    evaluator = _mapping(raw.get("evaluator"), "evaluator")
    initial = _mapping(evaluator.get("initial"), "evaluator.initial")
    terminal = _mapping(evaluator.get("terminal"), "evaluator.terminal")
    initial_dirt = _mapping(initial.get("ground_dirt"), "initial.ground_dirt")
    terminal_dirt = _mapping(terminal.get("ground_dirt"), "terminal.ground_dirt")
    initial_dirt_area = _number(initial_dirt, "initial_area_m2", "initial.ground_dirt")
    initial_cleaned_area = _number(initial_dirt, "cleaned_area_m2", "initial.ground_dirt")
    if abs(initial_cleaned_area) > 1.0e-9:
        raise AggregateError("ground dirt was already cleaned before operator start")
    dirt_delta = _number(terminal_dirt, "cleaned_area_m2", "terminal.ground_dirt") - initial_cleaned_area
    truth_dirt_area = _number(ground_truth, "dirt_union_area_m2", "ground_truth")
    if abs(initial_dirt_area - truth_dirt_area) > 1.0e-6 or initial_dirt_area <= 0.0:
        raise AggregateError("ground-dirt evaluator initial area differs from episode truth")
    dirt_fraction = dirt_delta / initial_dirt_area
    brush_chain = all(
        terminal_dirt.get(key) is True for key in ("left_ready", "right_ready", "roller_ready")
    ) and _integer(terminal_dirt, "rigid_litter_entities_modified", "terminal.ground_dirt") == 0

    initial_bin = _mapping(initial.get("dry_bin"), "initial.dry_bin")
    terminal_bin = _mapping(terminal.get("dry_bin"), "terminal.dry_bin")
    if (
        _integer(initial_bin, "contained_object_count", "initial.dry_bin") != 0
        or abs(_number(initial_bin, "physical_contained_mass_kg", "initial.dry_bin")) > 1.0e-9
    ):
        raise AggregateError("dry bin was not empty before operator start")
    dry_count_delta = _integer(terminal_bin, "contained_object_count", "terminal.dry_bin") - _integer(initial_bin, "contained_object_count", "initial.dry_bin")
    dry_mass_delta = _number(terminal_bin, "physical_contained_mass_kg", "terminal.dry_bin") - _number(initial_bin, "physical_contained_mass_kg", "initial.dry_bin")
    expected_dry_mass = sum(_number(cube, "mass_kg", "ground_truth.cube") for cube in cubes)
    if dry_count_delta != 20 or abs(dry_mass_delta - expected_dry_mass) > 2.0e-5:
        raise AggregateError("terminal dry-bin count/mass increment differs from the 20 episode cubes")
    if abs(sum(row["measured_increment_kg"] for row in grasp_evidence) - dry_mass_delta) > 2.0e-5:
        raise AggregateError("per-grasp mass increments do not reconcile to terminal dry-bin mass")

    initial_water = _mapping(initial.get("water"), "initial.water")
    terminal_water = _mapping(terminal.get("water"), "terminal.water")
    initial_ground_l = _number(initial_water, "ground_volume_l", "initial.water")
    if abs(_number(initial_water, "recovered_volume_l", "initial.water")) > 1.0e-9:
        raise AggregateError("water recovery had already advanced before operator start")
    recovered_delta_l = _number(terminal_water, "recovered_volume_l", "terminal.water") - _number(initial_water, "recovered_volume_l", "initial.water")
    ground_delta_l = initial_ground_l - _number(terminal_water, "ground_volume_l", "terminal.water")
    tank_mass_delta = _number(terminal_water, "tank_mass_kg", "terminal.water") - _number(initial_water, "tank_mass_kg", "initial.water")
    if initial_ground_l <= 0.0 or recovered_delta_l <= 0.0:
        raise AggregateError("water evaluator has no positive same-run recovery increment")
    if abs(recovered_delta_l - ground_delta_l) > max(1.0e-4, initial_ground_l * 0.01):
        raise AggregateError("ground-water decrement and recovered-volume increment disagree")
    if abs(tank_mass_delta - recovered_delta_l) > max(1.0e-4, recovered_delta_l * 0.01):
        raise AggregateError("wet-tank mass increment and recovered volume disagree")
    water_chain = all(
        terminal_water.get(key) is True
        for key in ("brush_ready", "squeegee_ready", "nozzle_ready", "pump_ready")
    )

    terminal_pedestrians = _mapping(terminal.get("pedestrians"), "terminal.pedestrians")
    if terminal_pedestrians.get("state") != "ACTIVE" or _integer(terminal_pedestrians, "pedestrian_count", "terminal.pedestrians") != public_pedestrian_count:
        raise AggregateError("live pedestrian driver differs from the frozen episode")

    return_state = _mapping(product.get("return_start_state"), "product.return_start_state")
    return_eval = _mapping(return_state.get("evaluator"), "return_start_state.evaluator")
    return_dirt = _mapping(return_eval.get("ground_dirt"), "return_start_state.ground_dirt")
    return_water = _mapping(return_eval.get("water"), "return_start_state.water")
    return_bin = _mapping(return_eval.get("dry_bin"), "return_start_state.dry_bin")
    return_water_recovered = _number(return_water, "recovered_volume_l", "return_start_state.water") - _number(initial_water, "recovered_volume_l", "initial.water")
    return_started_after_complete = (
        product.get("return_started_seen") is True
        and _number(return_dirt, "cleaned_area_m2", "return_start_state.ground_dirt") / initial_dirt_area >= 0.95
        and _integer(return_bin, "contained_object_count", "return_start_state.dry_bin") - _integer(initial_bin, "contained_object_count", "initial.dry_bin") == 20
        and return_water_recovered / initial_ground_l >= 0.95
        and set(return_state.get("successful_grasp_target_ids", [])) == set(cube_by_id)
    )

    return {
        "schema_version": 3,
        "field": {
            "profile": episode["profile"], "width_m": width, "length_m": height,
            "area_m2": area, "geofence_polygon_m": polygon,
            "source": "episode.public.field",
        },
        "mission": {
            "fixed_start_verified": baseline["fixed_start_verified"],
            "first_map_ignored_dirt": baseline["first_map_ignored_dirt"],
            "saved_map_hard_restart_verified": baseline["saved_map_hard_restart_verified"],
            "episode_seed": identity["episode_seed"], "episode_seed_ledger": seeds,
            "episode_id": episode_id, "session_id": identity["session_id"], "map_id": map_id,
        },
        "perception": {
            "world_truth_used_for_control": False,
            "truth_boundary_source": "live_ros_subscription_graph",
            "observed_field_fraction": observed_ratio,
            "artifact_tree_sha256": artifacts["perception_artifacts"]["tree_sha256"],
        },
        "ground_dirt": {
            "actual_brushed_area_fraction": dirt_fraction,
            "cleaned_area_increment_m2": dirt_delta,
            "episode_truth_area_m2": truth_dirt_area,
            "physical_brush_contact_verified": brush_chain,
        },
        "discrete_litter": {
            "spawned_count": len(cubes), "physically_deposited_count": dry_count_delta,
            "cube_edge_m": cube_contract["edge_m"],
            "episode_target_ids": sorted(cube_by_id),
            "successful_target_ids": sorted(product["successful_grasp_target_ids"]),
            "materials_observed": sorted({cube["material"] for cube in cubes}),
            "target_pose_conditioned_planning": all(row["target_conditioned_moveit_ik_collision_verified"] for row in grasp_evidence),
            "ik_and_collision_checked": all(row["target_conditioned_moveit_ik_collision_verified"] for row in grasp_evidence),
            "wrist_recheck_verified": all(row["wrist_recheck_verified"] for row in grasp_evidence),
            "dry_bin_physical_mass_increment_kg": dry_mass_delta,
            "episode_truth_total_mass_kg": expected_dry_mass,
            "dynamic_bin_mass_increment_verified": True,
            "per_target_grasp_evidence": grasp_evidence,
        },
        "water_recovery": {
            "recovered_fraction": recovered_delta_l / initial_ground_l,
            "ground_volume_decrement_l": ground_delta_l,
            "recovered_volume_increment_l": recovered_delta_l,
            "tank_mass_increment_kg": tank_mass_delta,
            "brush_squeegee_pump_chain_verified": water_chain,
            "dynamic_tank_mass_increment_verified": True,
        },
        "dynamic_obstacles": {
            "random_pedestrian_count": public_pedestrian_count,
            "collision_count": _integer(evaluator, "collision_count", "evaluator"),
            "replan_or_stop_verified": _integer(evaluator, "collision_monitor_intervention_count", "evaluator") > 0,
        },
        "planning": {
            "planner": policy["policy"],
            "policy_checkpoint_sha256": artifacts["policy_checkpoint"]["sha256"],
            "policy_truth_access_used": policy["truth_access_used"],
            "cleaning_distance_m": mission_distance,
            "full_coverage_baseline_distance_m": baseline_distance,
            "baseline_map_id": baseline["map_id"],
            "same_map_full_coverage_efficiency": dict(competition_efficiency),
            "trajectory_publish_count": product["trajectory_publish_count"],
        },
        "return_home": {
            "distance_excluded_from_efficiency_m": return_distance,
            "started_only_after_task_complete": return_started_after_complete,
            "fixed_home_reached": (
                planner.get("reason") == "task_complete_and_fixed_start_pose_reached"
                and planner.get("state") == "COMPLETE"
            ),
        },
        "evidence": {
            "single_snapshot_session": True, "single_episode": True,
            "raw_collection_path": str(raw_path.resolve()),
            "raw_collection_sha256": sha256_file(raw_path),
            "run_identity": identity, "metric_sources": sources,
            "immutable_input_artifacts": artifacts,
            "runtime_parameters": runtime_parameters,
            "runtime_graph": graph,
            "initial_evaluator_state": initial,
            "terminal_evaluator_state": terminal,
            "runtime_gate_binding": runtime_binding,
            "acceptance_session_binding": runtime_binding[
                "acceptance_session_binding"
            ],
            "runtime_closure_binding": runtime_binding["runtime_closure_binding"],
            "planner_trajectory_evidence": trajectory_evidence,
            "planner_observed_ratio_progress": observed_progress,
            "task_odom_trajectory_xy_m": [list(point) for point in task_trajectory],
            "return_odom_trajectory_xy_m": [
                list(point) for point in return_trajectory
            ],
            "truth_boundary": {
                "evaluator_topics": sorted(CONTROL_PROHIBITED_TRUTH_TOPICS),
                "control_truth_topics_subscribed": [],
                "subscriber_audit": subscribers,
            },
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--raw", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    try:
        result = aggregate(args.raw)
    except (OSError, ValueError, TypeError, json.JSONDecodeError, AggregateError) as exc:
        print(json.dumps({"status": "BLOCKED", "error": str(exc)}, sort_keys=True))
        return 3
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
