#!/usr/bin/env python3
"""Fail-closed validator for one complete random campus cleaning mission."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
from typing import Any

from aggregate_formal_single_episode_cleaning_mission import AggregateError, aggregate
from collect_formal_single_episode_cleaning_mission import (
    CONTROL_PROHIBITED_TRUTH_TOPICS, REQUIRED_RUNTIME_NODES, sha256_file,
)
from formal_runtime_gate_binding import RuntimeGateError, load_binding


class EndToEndMissionError(RuntimeError):
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


def validate(
    payload: dict[str, Any], *, runtime_gate_binding: dict[str, Any] | None = None
) -> dict[str, Any]:
    errors: list[str] = []

    def require(condition: bool, message: str) -> None:
        if not condition:
            errors.append(message)

    require(payload.get("schema_version") == 3, "aggregate schema_version must equal 3")
    sections: dict[str, dict[str, Any]] = {}
    for name in (
        "field", "mission", "perception", "ground_dirt", "discrete_litter",
        "water_recovery", "dynamic_obstacles", "planning", "return_home", "evidence",
    ):
        value = payload.get(name)
        require(isinstance(value, dict), f"{name} must be an object")
        sections[name] = value if isinstance(value, dict) else {}
    if errors:
        return _result(errors)

    evidence = sections["evidence"]
    embedded_runtime_binding = evidence.get("runtime_gate_binding")
    require(
        isinstance(embedded_runtime_binding, dict),
        "complete runtime gate binding is missing from aggregate evidence",
    )
    if runtime_gate_binding is not None:
        require(
            embedded_runtime_binding == runtime_gate_binding,
            "aggregate runtime gate binding differs from the final sidecar",
        )
    raw_path = Path(str(evidence.get("raw_collection_path", "")))
    require(raw_path.is_file(), "raw single-episode collection missing")
    if raw_path.is_file():
        require(
            sha256_file(raw_path) == evidence.get("raw_collection_sha256"),
            "raw single-episode collection hash mismatch",
        )
        try:
            recomputed = aggregate(raw_path)
            require(
                recomputed == payload,
                "aggregate differs from deterministic recomputation of raw evidence",
            )
        except (OSError, ValueError, TypeError, json.JSONDecodeError, AggregateError) as exc:
            errors.append(f"raw evidence cannot be re-aggregated: {exc}")

    field = sections["field"]
    width = _finite(field.get("width_m"))
    length = _finite(field.get("length_m"))
    area = _finite(field.get("area_m2"))
    require(field.get("profile") == "formal", "field is not the formal episode profile")
    require(field.get("source") == "episode.public.field", "field is not manifest-derived")
    require(width > 0.0 and length > 0.0 and area >= 20000.0, "formal field dimensions/area invalid")
    require(abs(width * length - area) <= 1.0e-6, "field dimensions do not match area")

    mission = sections["mission"]
    require(mission.get("fixed_start_verified") is True, "fixed start was not explicitly verified")
    require(mission.get("first_map_ignored_dirt") is True, "mapping did not explicitly ignore dirt")
    require(mission.get("saved_map_hard_restart_verified") is True, "saved-map hard restart missing")
    require(isinstance(mission.get("episode_seed"), int), "random episode seed missing")
    seeds = mission.get("episode_seed_ledger")
    require(
        isinstance(seeds, dict)
        and set(seeds) == {"layout", "dirt", "cubes", "pedestrians", "sensor"}
        and mission.get("episode_seed") == seeds.get("dirt"),
        "complete episode seed ledger/runtime binding missing",
    )

    perception = sections["perception"]
    require(perception.get("world_truth_used_for_control") is False, "truth entered perception control")
    require(perception.get("truth_boundary_source") == "live_ros_subscription_graph", "truth boundary is not graph-derived")
    require(_finite(perception.get("observed_field_fraction")) >= 0.95, "less than 95% of field observed")
    require(_sha(perception.get("artifact_tree_sha256")), "perception artifact directory hash missing")

    dirt = sections["ground_dirt"]
    require(_finite(dirt.get("actual_brushed_area_fraction")) >= 0.95, "ground dirt cleaning below 95%")
    require(_finite(dirt.get("cleaned_area_increment_m2")) > 0.0, "no same-run brushed-area increment")
    require(dirt.get("physical_brush_contact_verified") is True, "brush contact chain not verified")

    litter = sections["discrete_litter"]
    spawned = litter.get("spawned_count")
    deposited = litter.get("physically_deposited_count")
    episode_ids = litter.get("episode_target_ids")
    success_ids = litter.get("successful_target_ids")
    require(spawned == 20 and deposited == 20, "exactly 20 physical cubes were not deposited")
    require(litter.get("cube_edge_m") == 0.03, "episode cubes are not exactly 3 cm")
    require(
        isinstance(episode_ids, list) and len(episode_ids) == 20
        and len(set(episode_ids)) == 20 and success_ids == episode_ids,
        "successful grasp IDs do not equal the 20 episode truth cube IDs",
    )
    require(
        set(litter.get("materials_observed", [])) == {"paperboard", "PP", "PET", "aluminum"},
        "all four randomized material classes did not occur",
    )
    require(litter.get("target_pose_conditioned_planning") is True, "target-conditioned MoveIt planning missing")
    require(litter.get("ik_and_collision_checked") is True, "per-target IK/collision proof missing")
    require(litter.get("wrist_recheck_verified") is True, "per-target wrist recheck proof missing")
    dry_mass = _finite(litter.get("dry_bin_physical_mass_increment_kg"))
    truth_mass = _finite(litter.get("episode_truth_total_mass_kg"))
    require(dry_mass > 0.0 and abs(dry_mass - truth_mass) <= 2.0e-5, "dry-bin mass increment does not match episode truth")
    require(litter.get("dynamic_bin_mass_increment_verified") is True, "dry-bin dynamic mass increment missing")
    per_target = litter.get("per_target_grasp_evidence")
    require(
        isinstance(per_target, list) and len(per_target) == 20
        and {row.get("target_id") for row in per_target if isinstance(row, dict)} == set(episode_ids or []),
        "20 complete per-target grasp evidence rows missing",
    )

    water = sections["water_recovery"]
    recovered = _finite(water.get("recovered_volume_increment_l"))
    ground_delta = _finite(water.get("ground_volume_decrement_l"))
    tank_delta = _finite(water.get("tank_mass_increment_kg"))
    require(_finite(water.get("recovered_fraction")) >= 0.95, "water recovery below 95%")
    require(recovered > 0.0 and ground_delta > 0.0 and tank_delta > 0.0, "same-run water/tank increments missing")
    require(abs(recovered - ground_delta) <= max(1.0e-4, recovered * 0.01), "water volume decrement/recovery mismatch")
    require(abs(recovered - tank_delta) <= max(1.0e-4, recovered * 0.01), "wet-tank mass increment/recovery mismatch")
    require(water.get("brush_squeegee_pump_chain_verified") is True, "physical brush/squeegee/pump chain missing")
    require(water.get("dynamic_tank_mass_increment_verified") is True, "wet-tank dynamic mass increment missing")

    obstacles = sections["dynamic_obstacles"]
    require(isinstance(obstacles.get("random_pedestrian_count"), int) and obstacles["random_pedestrian_count"] > 0, "random pedestrians missing")
    require(obstacles.get("collision_count") == 0, "mission had a collision")
    require(obstacles.get("replan_or_stop_verified") is True, "obstacle stop/replan not verified")

    planning = sections["planning"]
    mission_distance = _finite(planning.get("cleaning_distance_m"), default=math.inf)
    baseline_distance = _finite(planning.get("full_coverage_baseline_distance_m"))
    require(planning.get("planner") == "q_learning", "runtime checkpoint was not Q-learning")
    require(planning.get("policy_truth_access_used") is False, "RL checkpoint used evaluator truth")
    require(_sha(planning.get("policy_checkpoint_sha256")), "policy checkpoint hash missing")
    require(mission_distance <= baseline_distance, "mission path exceeds same-map FullCoverage baseline")
    require(planning.get("baseline_map_id") == mission.get("map_id"), "baseline map differs from mission map")
    require(isinstance(planning.get("trajectory_publish_count"), int) and planning["trajectory_publish_count"] > 0, "planner published no trajectory")
    competition_efficiency = planning.get("same_map_full_coverage_efficiency")
    require(
        isinstance(competition_efficiency, dict),
        "same-map FullCoverage competition efficiency is missing",
    )
    if isinstance(competition_efficiency, dict):
        require(
            set(competition_efficiency) == COMPETITION_EFFICIENCY_KEYS,
            "same-map FullCoverage competition efficiency mapping is incomplete or unexpected",
        )
        threshold = _strict_finite(
            competition_efficiency.get("threshold_m2_h")
        )
        covered_area = _strict_finite(
            competition_efficiency.get("covered_area_m2")
        )
        duration = _strict_finite(
            competition_efficiency.get("actual_duration_sec")
        )
        measured_efficiency = _strict_finite(
            competition_efficiency.get("measured_net_efficiency_m2_h")
        )
        reported_recomputed_efficiency = _strict_finite(
            competition_efficiency.get("recomputed_net_efficiency_m2_h")
        )
        require(
            all(value is not None for value in (
                threshold, covered_area, duration, measured_efficiency,
                reported_recomputed_efficiency,
            )),
            "same-map FullCoverage competition efficiency must use finite numeric values",
        )
        require(
            competition_efficiency.get("passed") is True,
            "same-map FullCoverage competition efficiency has no explicit pass",
        )
        require(
            competition_efficiency.get("return_distance_included") is False,
            "same-map FullCoverage competition efficiency includes return-home distance",
        )
        if all(value is not None for value in (
            threshold, covered_area, duration, measured_efficiency,
            reported_recomputed_efficiency,
        )):
            assert threshold is not None and covered_area is not None and duration is not None
            assert measured_efficiency is not None and reported_recomputed_efficiency is not None
            require(
                math.isclose(threshold, COMPETITION_EFFICIENCY_THRESHOLD_M2_H, abs_tol=1.0e-9),
                "same-map FullCoverage competition threshold must equal 3500 m2/h",
            )
            require(
                covered_area > 0.0 and duration > 0.0,
                "same-map FullCoverage competition area/duration must be positive",
            )
            if covered_area > 0.0 and duration > 0.0:
                recomputed_efficiency = covered_area / duration * 3600.0
                require(
                    math.isclose(measured_efficiency, recomputed_efficiency, rel_tol=1.0e-9, abs_tol=1.0e-6)
                    and math.isclose(reported_recomputed_efficiency, recomputed_efficiency, rel_tol=1.0e-9, abs_tol=1.0e-6),
                    "same-map FullCoverage competition efficiency formula mismatch",
                )
                require(
                    measured_efficiency >= COMPETITION_EFFICIENCY_THRESHOLD_M2_H,
                    "same-map FullCoverage competition efficiency is below 3500 m2/h",
                )

    return_home = sections["return_home"]
    require(_finite(return_home.get("distance_excluded_from_efficiency_m"), default=-1.0) >= 0.0, "return distance accounting missing")
    require(return_home.get("started_only_after_task_complete") is True, "vehicle returned before all cleaning was complete")
    require(return_home.get("fixed_home_reached") is True, "fixed home was not reached")

    require(evidence.get("single_snapshot_session") is True, "evidence is not from one frozen session")
    require(evidence.get("single_episode") is True, "evidence is not from one episode")
    session_binding = evidence.get("acceptance_session_binding")
    closure_binding = evidence.get("runtime_closure_binding")
    require(
        isinstance(session_binding, dict)
        and session_binding.get("session_status_at_gate")
        == "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING",
        "fresh acceptance-session runtime binding missing",
    )
    require(
        isinstance(closure_binding, dict)
        and closure_binding.get("status")
        == "FORMAL_FINAL_RUNTIME_CLOSURE_VERIFIED",
        "verified non-symlink runtime closure binding missing",
    )
    trajectory_evidence = evidence.get("planner_trajectory_evidence")
    require(
        isinstance(trajectory_evidence, list)
        and len(trajectory_evidence) == planning.get("trajectory_publish_count")
        and all(
            isinstance(row, dict)
            and row.get("frame_id") == "map"
            and isinstance(row.get("pose_count"), int)
            and row["pose_count"] >= 2
            for row in trajectory_evidence
        ),
        "complete map-frame planner trajectory evidence missing",
    )
    progress = evidence.get("planner_observed_ratio_progress")
    require(
        isinstance(progress, list)
        and len(progress) >= 2
        and _finite(progress[0], default=1.0) < 0.95
        and _finite(progress[-1]) >= 0.95,
        "observed-area progress did not cross the 95% gate",
    )
    task_trajectory = _points(evidence.get("task_odom_trajectory_xy_m"))
    return_trajectory = _points(evidence.get("return_odom_trajectory_xy_m"))
    require(
        len(task_trajectory) >= 2
        and math.isclose(
            _distance(task_trajectory), mission_distance, rel_tol=1.0e-9, abs_tol=1.0e-6
        ),
        "cleaning mileage does not recompute from recorded odometry",
    )
    require(
        len(return_trajectory) >= 2
        and math.isclose(
            _distance(return_trajectory),
            _finite(return_home.get("distance_excluded_from_efficiency_m")),
            rel_tol=1.0e-9,
            abs_tol=1.0e-6,
        ),
        "return mileage does not recompute from recorded odometry",
    )
    identity = evidence.get("run_identity")
    required_identity = (
        "session_id", "episode_id", "episode_seed", "runtime_id", "gazebo_process_id",
        "ros_domain_id", "gz_partition",
    )
    require(isinstance(identity, dict), "run identity missing")
    if isinstance(identity, dict):
        require(all(identity.get(key) not in (None, "") for key in required_identity), "run identity incomplete")
        require(identity.get("session_id") == mission.get("session_id"), "mission/session identity mismatch")
        require(identity.get("episode_id") == mission.get("episode_id"), "mission/episode identity mismatch")
        require(identity.get("episode_seed") == mission.get("episode_seed"), "mission/seed identity mismatch")
    sources = evidence.get("metric_sources")
    require(isinstance(sources, list) and len(sources) == 12, "complete live metric source ledger missing")
    if isinstance(sources, list) and isinstance(identity, dict):
        for row in sources:
            require(isinstance(row, dict), "metric source row must be an object")
            if isinstance(row, dict):
                require(row.get("source_class") in {"product", "evaluator_truth"}, "historical/unclassified metric source prohibited")
                require(isinstance(row.get("sample_count"), int) and row["sample_count"] > 0, f"metric source has no live samples: {row.get('metric')}")
                require(all(row.get(key) == identity.get(key) for key in required_identity), f"metric source identity mismatch: {row.get('metric')}")
    artifacts = evidence.get("immutable_input_artifacts")
    require(
        isinstance(artifacts, dict)
        and set(artifacts) == {
            "episode_manifest", "evaluator_episode_manifest", "evaluator_ground_truth",
            "world", "pedestrian_schedule", "session_status", "same_map_baseline",
            "policy_checkpoint", "runtime_binding", "saved_map", "perception_artifacts",
        },
        "complete immutable input artifact ledger missing",
    )
    graph = evidence.get("runtime_graph")
    require(isinstance(graph, dict), "live executable runtime graph missing")
    if isinstance(graph, dict):
        require(graph.get("required_nodes_present") is True, "required product runtime nodes absent")
        require(REQUIRED_RUNTIME_NODES.issubset(set(graph.get("nodes", []))), "required runtime node names absent")
    truth_boundary = evidence.get("truth_boundary")
    require(isinstance(truth_boundary, dict), "truth boundary ledger missing")
    if isinstance(truth_boundary, dict):
        require(truth_boundary.get("control_truth_topics_subscribed") == [], "evaluator truth entered product control subscriptions")
        audit = truth_boundary.get("subscriber_audit")
        require(
            isinstance(audit, dict) and set(audit) == set(CONTROL_PROHIBITED_TRUTH_TOPICS)
            and all(rows == ["/formal_single_episode_cleaning_collector"] for rows in audit.values()),
            "live ROS truth-subscriber audit failed",
        )
    require(isinstance(evidence.get("initial_evaluator_state"), dict), "initial evaluator state missing")
    require(isinstance(evidence.get("terminal_evaluator_state"), dict), "terminal evaluator state missing")
    result = _result(
        errors,
        payload if not errors else None,
        runtime_gate_binding=embedded_runtime_binding
        if isinstance(embedded_runtime_binding, dict)
        else None,
    )
    if isinstance(session_binding, dict):
        result["acceptance_session_binding"] = session_binding
    if isinstance(closure_binding, dict):
        result["runtime_closure_binding"] = closure_binding
    return result


def _finite(value: Any, *, default: float = 0.0) -> float:
    try:
        converted = float(value)
    except (TypeError, ValueError):
        return default
    return converted if math.isfinite(converted) else default


def _strict_finite(value: Any) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    converted = float(value)
    return converted if math.isfinite(converted) else None


def _sha(value: Any) -> bool:
    return isinstance(value, str) and len(value) == 64 and all(char in "0123456789abcdef" for char in value)


def _points(value: Any) -> list[tuple[float, float]]:
    if not isinstance(value, list):
        return []
    result: list[tuple[float, float]] = []
    for row in value:
        if not isinstance(row, list) or len(row) != 2:
            return []
        point = (_finite(row[0], default=math.nan), _finite(row[1], default=math.nan))
        if not all(math.isfinite(item) for item in point):
            return []
        result.append(point)
    return result


def _distance(points: list[tuple[float, float]]) -> float:
    return sum(math.dist(left, right) for left, right in zip(points, points[1:]))


def _result(
    errors: list[str],
    payload: dict[str, Any] | None = None,
    *,
    runtime_gate_binding: dict[str, Any] | None = None,
) -> dict[str, Any]:
    passed = not errors
    result = {
        "report_id": "tzcup_formal_end_to_end_cleaning_mission_v3",
        "status": (
            "FORMAL_END_TO_END_CLEANING_MISSION_PASSED"
            if passed else "FORMAL_END_TO_END_CLEANING_MISSION_BLOCKED"
        ),
        "passed": passed,
        "errors": errors,
    }
    if passed and payload is not None:
        # These are a compact, type-stable projection of the values already
        # recomputed above.  They let the final acceptance session bind the
        # closed-loop facts without copying the large raw collector artifact.
        mission = payload["mission"]
        perception = payload["perception"]
        dirt = payload["ground_dirt"]
        litter = payload["discrete_litter"]
        water = payload["water_recovery"]
        obstacles = payload["dynamic_obstacles"]
        planning = payload["planning"]
        return_home = payload["return_home"]
        evidence = payload["evidence"]
        result["validated_closed_loop"] = {
            "fixed_start_verified": mission["fixed_start_verified"] is True,
            "first_map_ignored_dirt": mission["first_map_ignored_dirt"] is True,
            "saved_map_hard_restart_verified": mission["saved_map_hard_restart_verified"] is True,
            "truth_isolated_from_product_control": perception["world_truth_used_for_control"] is False,
            "observation_coverage_at_least_95_percent": _finite(perception["observed_field_fraction"]) >= 0.95,
            "actual_brushed_area_at_least_95_percent": _finite(dirt["actual_brushed_area_fraction"]) >= 0.95,
            "all_20_discrete_targets_physically_deposited": (
                litter["spawned_count"] == 20
                and litter["physically_deposited_count"] == 20
                and litter["successful_target_ids"] == litter["episode_target_ids"]
            ),
            "water_recovery_at_least_95_percent": _finite(water["recovered_fraction"]) >= 0.95,
            "zero_collisions": obstacles["collision_count"] == 0,
            "trajectory_output_verified": planning["trajectory_publish_count"] > 0,
            "task_distance_not_above_full_coverage_baseline": (
                _finite(planning["cleaning_distance_m"], default=math.inf)
                <= _finite(planning["full_coverage_baseline_distance_m"])
            ),
            "same_map_full_coverage_efficiency_at_least_3500": (
                planning["same_map_full_coverage_efficiency"]["passed"] is True
                and planning["same_map_full_coverage_efficiency"]["return_distance_included"] is False
                and _strict_finite(
                    planning["same_map_full_coverage_efficiency"]["measured_net_efficiency_m2_h"]
                ) is not None
                and _strict_finite(
                    planning["same_map_full_coverage_efficiency"]["measured_net_efficiency_m2_h"]
                ) >= COMPETITION_EFFICIENCY_THRESHOLD_M2_H
            ),
            "return_distance_excluded_from_efficiency": _finite(
                return_home["distance_excluded_from_efficiency_m"], default=-1.0
            ) >= 0.0,
            "return_started_only_after_task_complete": return_home["started_only_after_task_complete"] is True,
            "single_snapshot_single_episode": (
                evidence["single_snapshot_session"] is True
                and evidence["single_episode"] is True
            ),
        }
        if runtime_gate_binding is not None:
            result["runtime_gate_binding"] = runtime_gate_binding
    return result


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _current_runtime_binding(
    snapshot_manifest: Path, session_status: Path, sidecar: Path
) -> dict[str, Any]:
    try:
        snapshot = json.loads(snapshot_manifest.read_text(encoding="utf-8"))
        session = json.loads(session_status.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise EndToEndMissionError(f"cannot read current runtime identity: {exc}") from exc
    if not isinstance(snapshot, dict) or not isinstance(session, dict):
        raise EndToEndMissionError("current runtime identity must be JSON objects")
    outputs = snapshot.get("outputs")
    urdf = outputs.get("reports/engineering/formal_competition_vehicle.urdf") if isinstance(outputs, dict) else None
    source_hash = snapshot.get("source_inventory_sha256")
    if not isinstance(urdf, dict) or not isinstance(urdf.get("sha256"), str) or not isinstance(source_hash, str):
        raise EndToEndMissionError("current vehicle snapshot is incomplete")
    identity = {
        "snapshot_manifest_sha256": _sha256(snapshot_manifest),
        "source_inventory_sha256": source_hash,
        "expanded_urdf_sha256": urdf["sha256"],
    }
    if session.get("status") != "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING" or session.get("snapshot") != identity:
        raise EndToEndMissionError("current acceptance session is not bound to the snapshot")
    binding = load_binding(sidecar)
    bound_session = binding["acceptance_session_binding"]
    if (
        bound_session.get("snapshot") != identity
        or bound_session.get("session_started_epoch_ns") != session.get("started_epoch_ns")
        or bound_session.get("session_manifest") != str(session_status.resolve())
        or bound_session.get("session_manifest_sha256") != _sha256(session_status)
        or bound_session.get("snapshot_current_source_verified") is not True
    ):
        raise EndToEndMissionError("runtime binding is not bound to the current snapshot/session")
    return binding


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--snapshot-manifest", type=Path, required=True)
    parser.add_argument("--session-status", type=Path, required=True)
    parser.add_argument("--runtime-binding", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    try:
        payload = json.loads(args.input.read_text(encoding="utf-8"))
        if not isinstance(payload, dict):
            raise EndToEndMissionError("input root is not an object")
        runtime_binding = _current_runtime_binding(
            args.snapshot_manifest, args.session_status, args.runtime_binding
        )
        result = validate(payload, runtime_gate_binding=runtime_binding)
    except (OSError, UnicodeError, json.JSONDecodeError, ValueError, TypeError, RuntimeGateError, EndToEndMissionError) as exc:
        result = _result([str(exc)])
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result["passed"] else 3


if __name__ == "__main__":
    raise SystemExit(main())
