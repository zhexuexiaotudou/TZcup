#!/usr/bin/env python3
"""Generate and revalidate the session-bound same-map FullCoverage baseline."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
from typing import Any

import yaml

from collect_formal_safety_speed_readback import (
    CaptureError,
    validate_capture_order,
    validate_capture_timeout,
    validate_status_capture,
    validate_topic_info_capture,
)


REPORT_ID = "tzcup_formal_same_map_full_coverage_baseline_v1"
PASS_STATUS = "FORMAL_FULL_COVERAGE_BASELINE_PASSED"
SESSION_STATUS = "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING"
PLANNER = "OpenNav Coverage + Fields2Cover"
COMPETITION_EFFICIENCY_THRESHOLD_M2_H = 3500.0


class BaselineError(RuntimeError):
    pass


def _json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise BaselineError(f"cannot read JSON object {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise BaselineError(f"JSON root must be an object: {path}")
    return value


def _yaml(path: Path) -> dict[str, Any]:
    try:
        value = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        raise BaselineError(f"cannot read YAML object {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise BaselineError(f"YAML root must be an object: {path}")
    return value


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _number(row: dict[str, Any], key: str, label: str) -> float:
    value = row.get(key)
    if isinstance(value, bool):
        raise BaselineError(f"{label}.{key} must be numeric")
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise BaselineError(f"{label}.{key} must be numeric") from exc
    if not math.isfinite(result):
        raise BaselineError(f"{label}.{key} must be finite")
    return result


def _strict_number(row: dict[str, Any], key: str, label: str) -> float:
    value = row.get(key)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise BaselineError(f"{label}.{key} must be a numeric JSON scalar")
    return _number(row, key, label)


def _true(row: dict[str, Any], key: str, label: str) -> None:
    if row.get(key) is not True:
        raise BaselineError(f"{label}.{key} must be explicitly true")


def _false(row: dict[str, Any], key: str, label: str) -> None:
    if row.get(key) is not False:
        raise BaselineError(f"{label}.{key} must be explicitly false")


def _snapshot_identity(path: Path) -> dict[str, str]:
    snapshot = _json(path)
    outputs = snapshot.get("outputs")
    if not isinstance(outputs, dict):
        raise BaselineError("snapshot manifest has no outputs mapping")
    urdf = outputs.get("reports/engineering/formal_competition_vehicle.urdf")
    source_hash = snapshot.get("source_inventory_sha256")
    if not isinstance(urdf, dict) or not isinstance(urdf.get("sha256"), str):
        raise BaselineError("snapshot manifest has no expanded URDF hash")
    if not isinstance(source_hash, str) or len(source_hash) != 64:
        raise BaselineError("snapshot manifest has no source inventory hash")
    return {
        "snapshot_manifest_sha256": _sha256(path),
        "source_inventory_sha256": source_hash,
        "expanded_urdf_sha256": urdf["sha256"],
    }


def _evidence(
    path: Path, started_ns: int, label: str, *, require_session_fresh: bool
) -> dict[str, Any]:
    resolved = path.resolve()
    if not resolved.is_file():
        raise BaselineError(f"missing {label}: {resolved}")
    mtime = resolved.stat().st_mtime_ns
    if require_session_fresh and mtime < started_ns:
        raise BaselineError(f"{label} predates formal session start")
    return {"path": str(resolved), "sha256": _sha256(resolved), "mtime_epoch_ns": mtime}


def _hashes_valid(root: Path, manifest: dict[str, Any]) -> None:
    hashes = manifest.get("sha256")
    if not isinstance(hashes, dict) or not hashes:
        raise BaselineError("saved-map manifest has no artifact hash ledger")
    for name, expected in hashes.items():
        path = root / str(name)
        if not isinstance(expected, str) or not path.is_file() or _sha256(path) != expected:
            raise BaselineError(f"saved-map artifact hash mismatch: {name}")


def _triple(values: Any, label: str) -> list[float]:
    if not isinstance(values, list) or len(values) != 3:
        raise BaselineError(f"{label} must be a three-value pose")
    try:
        result = [float(value) for value in values]
    except (TypeError, ValueError) as exc:
        raise BaselineError(f"{label} contains a non-numeric value") from exc
    if not all(math.isfinite(value) for value in result):
        raise BaselineError(f"{label} contains a non-finite value")
    return result


def build_report(
    *,
    episode_manifest: Path,
    map_root: Path,
    mapping_runtime: Path,
    cleaning_runtime: Path,
    lifecycle_acceptance: Path,
    coverage_runtime: Path,
    session_path: Path,
    snapshot_path: Path,
    safety_manager_readback: Path | None = None,
    runtime_binding: Path | None = None,
    runtime_closure: Path | None = None,
    runtime_install: Path | None = None,
    expected_safety_cap: float = 0.45,
) -> dict[str, Any]:
    episode = _json(episode_manifest)
    manifest_path = map_root / "map_lifecycle_manifest.json"
    mission_path = map_root / "mission_geometry.yaml"
    manifest = _json(manifest_path)
    mission = _yaml(mission_path)
    mapping = _json(mapping_runtime)
    cleaning = _json(cleaning_runtime)
    lifecycle = _json(lifecycle_acceptance)
    coverage = _json(coverage_runtime)
    session = _json(session_path)
    snapshot = _snapshot_identity(snapshot_path)
    safety_readback = None
    if safety_manager_readback is not None:
        if runtime_binding is None or runtime_closure is None or runtime_install is None:
            raise BaselineError("safety-manager readback requires current runtime binding, closure and install")
        safety_readback = _json(safety_manager_readback)
        bound_runtime = _json(runtime_binding)
        closure = _json(runtime_closure)
        bound_session = bound_runtime.get("acceptance_session_binding")
        bound_closure = bound_runtime.get("runtime_closure_binding")
        if (
            bound_runtime.get("status") != "FORMAL_RUNTIME_GATE_BOUND"
            or not isinstance(bound_session, dict)
            or not isinstance(bound_closure, dict)
            or bound_session.get("session_manifest") != str(session_path.resolve())
            or bound_session.get("session_manifest_sha256") != _sha256(session_path)
            or bound_session.get("session_started_epoch_ns") != session.get("started_epoch_ns")
            or bound_session.get("session_status_at_gate") != SESSION_STATUS
            or bound_session.get("snapshot") != snapshot
            or bound_session.get("snapshot_current_source_verified") is not True
            or bound_closure.get("status")
            != "FORMAL_FINAL_RUNTIME_CLOSURE_VERIFIED"
            or bound_closure.get("manifest_sha256") != _sha256(runtime_closure)
            or bound_closure.get("runtime_install_root") != str(runtime_install.resolve())
            or not runtime_install.is_dir()
            or session.get("runtime_closure_binding") != bound_closure
        ):
            raise BaselineError("safety-manager runtime binding does not match current inputs")
        if safety_readback.get("schema_version") != 2:
            raise BaselineError("safety-manager readback has an unsupported schema")
        try:
            capture_timeout_sec = validate_capture_timeout(
                safety_readback.get("capture_timeout_sec")
            )
        except CaptureError as exc:
            raise BaselineError(f"safety-manager capture timeout is invalid: {exc}") from exc
        if safety_readback.get("capture_status") != "PASSED":
            raise BaselineError("safety-manager readback is not a passing live capture")
        if safety_readback.get("runtime_gate_binding_sha256") != _sha256(runtime_binding):
            raise BaselineError("safety-manager readback is bound to another runtime receipt")
        if safety_readback.get("runtime_gate_binding") != bound_runtime:
            raise BaselineError("safety-manager readback does not retain the current runtime binding")
        producer = safety_readback.get("producer_identity")
        producer_before = safety_readback.get("producer_capture_before")
        status_capture = safety_readback.get("status_capture")
        producer_capture = safety_readback.get("producer_capture")
        if (
            not isinstance(producer, dict)
            or producer.get("node_name") != "whole_vehicle_safety_manager"
            or producer.get("topic") != "/safety/status_json"
            or producer.get("message_type") != "std_msgs/msg/String"
            or producer.get("publisher_count") != "1"
            or not isinstance(producer_before, dict)
            or not isinstance(status_capture, dict)
            or not isinstance(producer_capture, dict)
        ):
            raise BaselineError("safety-manager producer receipt is incomplete or unexpected")
        try:
            before_identity, before_window = validate_topic_info_capture(
                producer_before, "producer-before", capture_timeout_sec
            )
            captured_status, status_window = validate_status_capture(
                status_capture, capture_timeout_sec
            )
            after_identity, after_window = validate_topic_info_capture(
                producer_capture, "producer-after", capture_timeout_sec
            )
            validate_capture_order(
                before_window, status_window, after_window, capture_timeout_sec
            )
        except CaptureError as exc:
            raise BaselineError(f"safety-manager producer receipt is invalid: {exc}") from exc
        if before_identity != producer or after_identity != producer:
            raise BaselineError("safety-manager retained topic-info identity differs from receipt")
        cap = _strict_number(safety_readback, "effective_max_linear_velocity_mps", "safety_manager_readback")
        if captured_status.get("effective_max_linear_velocity_mps") != cap:
            raise BaselineError("safety-manager receipt cap differs from captured status")
        if captured_status.get("operation_speed_profile") != safety_readback.get("operation_speed_profile"):
            raise BaselineError("safety-manager receipt profile differs from captured status")
        if captured_status.get("speed_qualification_state") != safety_readback.get("speed_qualification_state"):
            raise BaselineError("safety-manager receipt state differs from captured status")
        if not math.isclose(cap, expected_safety_cap, abs_tol=1.0e-12):
            raise BaselineError("safety-manager effective cap differs from runner requirement")
        if expected_safety_cap == 1.0 and safety_readback.get("speed_qualification_state") != "isolated_same_map_dry_coverage":
            raise BaselineError("1.0 m/s baseline lacks isolated dry-only safety state")
        if expected_safety_cap == 1.0 and safety_readback.get("operation_speed_profile") != "dry_cleaning_competition_candidate":
            raise BaselineError("1.0 m/s baseline lacks dry-cleaning speed profile")

    if session.get("status") != SESSION_STATUS:
        raise BaselineError("formal acceptance session is not RUNNING")
    started_ns = session.get("started_epoch_ns")
    if isinstance(started_ns, bool) or not isinstance(started_ns, int) or started_ns <= 0:
        raise BaselineError("formal acceptance session has no valid start time")
    if session.get("snapshot") != snapshot:
        raise BaselineError("formal session snapshot identity differs from current snapshot")

    episode_id, map_id = episode.get("episode_id"), episode.get("map_id")
    if not isinstance(episode_id, str) or not episode_id or not isinstance(map_id, str) or not map_id:
        raise BaselineError("episode has no stable episode_id/map_id")
    if episode.get("profile") != "formal":
        raise BaselineError("baseline episode is not formal")
    field = episode.get("field")
    if not isinstance(field, dict) or _number(field, "area_m2", "episode.field") < 20000.0:
        raise BaselineError("baseline episode field is smaller than 20000 square metres")
    if manifest.get("episode_id") != episode_id or manifest.get("map_id") != map_id:
        raise BaselineError("saved-map identity differs from the episode")
    if manifest.get("status") != "ready_for_localization_cleaning":
        raise BaselineError("saved-map manifest is not ready for localization cleaning")
    if _number(manifest, "observed_fraction", "map_manifest") < 0.95:
        raise BaselineError("saved-map observed fraction is below 95 percent")
    _true(manifest, "fixed_start_verified", "map_manifest")
    _true(manifest, "mapping_ignored_dirt", "map_manifest")
    _false(manifest, "world_truth_used_for_control", "map_manifest")
    _hashes_valid(map_root, manifest)

    public_pose = episode.get("vehicle_start_pose_map")
    if not isinstance(public_pose, dict):
        raise BaselineError("episode has no fixed vehicle_start_pose_map")
    expected_source = [
        _number(public_pose, "x_m", "episode.vehicle_start_pose_map"),
        _number(public_pose, "y_m", "episode.vehicle_start_pose_map"),
        _number(public_pose, "yaw_rad", "episode.vehicle_start_pose_map"),
    ]
    source_pose = _triple(mission.get("source_fixed_start_pose"), "mission.source_fixed_start_pose")
    if any(abs(left - right) > 1.0e-9 for left, right in zip(source_pose, expected_source)):
        raise BaselineError("mission source fixed start differs from episode fixed start")
    local_pose = mission.get("vehicle_start_pose_map")
    if not isinstance(local_pose, dict) or any(
        abs(_number(local_pose, key, "mission.vehicle_start_pose_map")) > 1.0e-9
        for key in ("x_m", "y_m", "yaw_rad")
    ):
        raise BaselineError("saved map frame is not anchored at the fixed start")
    truth_boundary = mission.get("truth_boundary")
    if not isinstance(truth_boundary, dict):
        raise BaselineError("mission geometry has no truth boundary")
    for key in ("world_geometry_used_for_product_map", "evaluator_truth_used", "dirt_truth_used"):
        _false(truth_boundary, key, "mission.truth_boundary")

    expected_urdf = snapshot["expanded_urdf_sha256"]
    _true(mapping, "passed", "mapping_runtime")
    _false(mapping, "truth_used_for_control", "mapping_runtime")
    if mapping.get("robot_description_sha256") != expected_urdf:
        raise BaselineError("mapping runtime robot description differs from frozen snapshot")
    _true(cleaning, "passed", "cleaning_runtime")
    _false(cleaning, "truth_used_for_control", "cleaning_runtime")
    if cleaning.get("localization_backend") != "amcl":
        raise BaselineError("baseline cleaning did not use saved-map AMCL")
    if cleaning.get("robot_description_sha256") != expected_urdf:
        raise BaselineError("cleaning runtime robot description differs from frozen snapshot")
    for key in ("saved_map_sha256_verified", "hard_restart_verified", "cleaning_stack_ready", "coverage_server_ready"):
        _true(cleaning, key, "cleaning_runtime")
    _false(cleaning, "world_derived_map_fallback", "cleaning_runtime")
    restart = cleaning.get("hard_restart_record")
    if not isinstance(restart, dict):
        raise BaselineError("cleaning runtime has no hard restart record")
    _true(restart, "mapping_stopped_before_cleaning", "hard_restart_record")
    if restart.get("mapping_process_count_before_cleaning") != 0 or restart.get("restart_type") != "separate_process_hard_restart":
        raise BaselineError("cleaning was not a separate-process hard restart")

    if lifecycle.get("status") != "FORMAL_FIRST_MAP_THEN_SAVED_MAP_CLEANING_PASSED":
        raise BaselineError("map lifecycle acceptance has no passing status")
    _true(lifecycle, "passed", "lifecycle_acceptance")
    _false(lifecycle, "truth_used_for_control", "lifecycle_acceptance")
    checks = lifecycle.get("checks")
    if not isinstance(checks, dict) or not checks or any(value is not True for value in checks.values()):
        raise BaselineError("map lifecycle acceptance checks are incomplete")

    if coverage.get("schema_version") != 2 or coverage.get("planner") != PLANNER:
        raise BaselineError("coverage evidence is not the live OpenNav/Fields2Cover planner")
    if coverage.get("mission_id") != mission.get("mission_id"):
        raise BaselineError("coverage mission differs from saved-map mission geometry")
    for key in ("success", "planning_success", "full_execution_success", "coverage_quality_success", "safety_success", "localization_success"):
        _true(coverage, key, "coverage_runtime")
    injection = coverage.get("evaluation_injection")
    if not isinstance(injection, dict):
        raise BaselineError("coverage report has no evaluation truth boundary")
    _false(injection, "ground_truth_used_for_control", "coverage_runtime.evaluation_injection")
    empirical = coverage.get("empirical_metrics")
    planned = coverage.get("planned_metrics")
    if not isinstance(empirical, dict) or not isinstance(planned, dict):
        raise BaselineError("coverage report lacks planned/empirical metrics")
    actual_distance = _number(empirical, "actual_path_length_m", "coverage_runtime.empirical_metrics")
    planned_distance = _number(planned, "path_length_m", "coverage_runtime.planned_metrics")
    if actual_distance <= 0.0 or planned_distance <= 0.0:
        raise BaselineError("FullCoverage distances must be positive")
    covered_area = _strict_number(empirical, "covered_area_m2", "coverage_runtime.empirical_metrics")
    actual_duration = _strict_number(empirical, "actual_duration_sec", "coverage_runtime.empirical_metrics")
    measured_efficiency = _strict_number(
        empirical, "net_efficiency_m2_h", "coverage_runtime.empirical_metrics"
    )
    _true(coverage, "competition_efficiency_pass", "coverage_runtime")
    if covered_area <= 0.0 or actual_duration <= 0.0:
        raise BaselineError("FullCoverage competition coverage area/duration must be positive")
    recomputed_efficiency = covered_area / actual_duration * 3600.0
    if not math.isclose(
        measured_efficiency,
        recomputed_efficiency,
        rel_tol=1.0e-9,
        abs_tol=1.0e-6,
    ):
        raise BaselineError("FullCoverage competition efficiency differs from area/duration recomputation")
    if measured_efficiency < COMPETITION_EFFICIENCY_THRESHOLD_M2_H:
        raise BaselineError("FullCoverage competition efficiency is below 3500 m2/h")

    evidence_paths = {
        "episode_manifest": episode_manifest,
        "map_manifest": manifest_path,
        "mission_geometry": mission_path,
        "mapping_runtime": mapping_runtime,
        "cleaning_runtime": cleaning_runtime,
        "lifecycle_acceptance": lifecycle_acceptance,
        "coverage_runtime": coverage_runtime,
        "session": session_path,
        "snapshot": snapshot_path,
    }
    if safety_manager_readback is not None:
        evidence_paths["safety_manager_readback"] = safety_manager_readback
        evidence_paths["runtime_binding"] = runtime_binding
        evidence_paths["runtime_closure"] = runtime_closure
    session_fresh = {
        "map_manifest", "mission_geometry", "mapping_runtime", "cleaning_runtime",
        "lifecycle_acceptance", "coverage_runtime",
    }
    if safety_manager_readback is not None:
        session_fresh.add("safety_manager_readback")
        session_fresh.add("runtime_binding")
        session_fresh.add("runtime_closure")
    evidence = {
        name: _evidence(
            path, started_ns, name, require_session_fresh=name in session_fresh
        )
        for name, path in sorted(evidence_paths.items())
    }
    return {
        "schema_version": 1,
        "report_id": REPORT_ID,
        "status": PASS_STATUS,
        "session_bound": True,
        "episode_id": episode_id,
        "map_id": map_id,
        "fixed_start_verified": True,
        "first_map_ignored_dirt": True,
        "saved_map_hard_restart_verified": True,
        "truth_used_for_control": False,
        "planner": "full_coverage",
        "planner_implementation": PLANNER,
        "successful_distance_m": actual_distance,
        "return_distance_included": False,
        "competition_efficiency": {
            "threshold_m2_h": COMPETITION_EFFICIENCY_THRESHOLD_M2_H,
            "covered_area_m2": covered_area,
            "actual_duration_sec": actual_duration,
            "measured_net_efficiency_m2_h": measured_efficiency,
            "recomputed_net_efficiency_m2_h": recomputed_efficiency,
            "return_distance_included": False,
            "passed": True,
        },
        "planner_comparison": {
            "baseline_planner": "full_coverage",
            "candidate_planner": "q_learning",
            "metric": "task_trajectory_length_m_excluding_return",
            "acceptance_rule": "candidate_successful_distance_m <= baseline_successful_distance_m",
            "planned_distance_m": planned_distance,
            "successful_distance_m": actual_distance,
            "execution_over_plan_ratio": actual_distance / planned_distance,
        },
        "source_binding": {"started_epoch_ns": started_ns, **snapshot},
        "safety_manager_speed": safety_readback,
        "evidence": evidence,
    }


def generate(args: argparse.Namespace) -> dict[str, Any]:
    if args.output.exists():
        raise BaselineError(f"refusing to overwrite retained baseline: {args.output}")
    report = build_report(
        episode_manifest=args.episode_manifest,
        map_root=args.map_root,
        mapping_runtime=args.mapping_runtime,
        cleaning_runtime=args.cleaning_runtime,
        lifecycle_acceptance=args.lifecycle_acceptance,
        coverage_runtime=args.coverage_runtime,
        session_path=args.session,
        snapshot_path=args.snapshot,
        safety_manager_readback=getattr(args, "safety_manager_readback", None),
        runtime_binding=getattr(args, "runtime_binding", None),
        runtime_closure=getattr(args, "runtime_closure", None),
        runtime_install=getattr(args, "runtime_install", None),
        expected_safety_cap=getattr(args, "expected_safety_cap", 0.45),
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    pending = args.output.with_suffix(args.output.suffix + f".pending.{os.getpid()}")
    if pending.exists():
        raise BaselineError(f"refusing stale pending output: {pending}")
    pending.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    pending.replace(args.output)
    return report


def validate(input_path: Path, session_path: Path, snapshot_path: Path) -> dict[str, Any]:
    stored = _json(input_path)
    if stored.get("report_id") != REPORT_ID or stored.get("status") != PASS_STATUS:
        raise BaselineError("input is not a passing formal same-map baseline")
    evidence = stored.get("evidence")
    if not isinstance(evidence, dict):
        raise BaselineError("baseline has no evidence ledger")
    required = {
        "episode_manifest", "map_manifest", "mission_geometry", "mapping_runtime",
        "cleaning_runtime", "lifecycle_acceptance", "coverage_runtime", "session", "snapshot",
    }
    if stored.get("safety_manager_speed") is not None:
        required.add("safety_manager_readback")
        required.add("runtime_binding")
        required.add("runtime_closure")
    if set(evidence) != required:
        raise BaselineError("baseline evidence ledger is incomplete or unexpected")
    for name, row in evidence.items():
        if not isinstance(row, dict):
            raise BaselineError(f"invalid evidence descriptor: {name}")
        path = Path(str(row.get("path", "")))
        if not path.is_file() or _sha256(path) != row.get("sha256") or path.stat().st_mtime_ns != row.get("mtime_epoch_ns"):
            raise BaselineError(f"baseline evidence changed after generation: {name}")
    if Path(str(evidence["session"]["path"])).resolve() != session_path.resolve():
        raise BaselineError("baseline is bound to a different formal session file")
    if Path(str(evidence["snapshot"]["path"])).resolve() != snapshot_path.resolve():
        raise BaselineError("baseline is bound to a different snapshot file")
    rebuilt = build_report(
        episode_manifest=Path(evidence["episode_manifest"]["path"]),
        map_root=Path(evidence["map_manifest"]["path"]).parent,
        mapping_runtime=Path(evidence["mapping_runtime"]["path"]),
        cleaning_runtime=Path(evidence["cleaning_runtime"]["path"]),
        lifecycle_acceptance=Path(evidence["lifecycle_acceptance"]["path"]),
        coverage_runtime=Path(evidence["coverage_runtime"]["path"]),
        session_path=session_path,
        snapshot_path=snapshot_path,
        safety_manager_readback=(Path(evidence["safety_manager_readback"]["path"])
            if "safety_manager_readback" in evidence else None),
        runtime_binding=(Path(evidence["runtime_binding"]["path"])
            if "runtime_binding" in evidence else None),
        runtime_closure=(Path(evidence["runtime_closure"]["path"])
            if "runtime_closure" in evidence else None),
        runtime_install=(Path(stored["safety_manager_speed"]["runtime_gate_binding"]
            ["runtime_closure_binding"]["runtime_install_root"])
            if stored.get("safety_manager_speed") is not None else None),
        expected_safety_cap=_strict_number(stored.get("safety_manager_speed", {}), "effective_max_linear_velocity_mps", "baseline.safety_manager_speed")
            if stored.get("safety_manager_speed") is not None else 0.45,
    )
    if rebuilt != stored:
        raise BaselineError("baseline contents do not equal recomputed source evidence")
    return rebuilt


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    generate_parser = subparsers.add_parser("generate")
    for name in (
        "episode_manifest", "map_root", "mapping_runtime", "cleaning_runtime",
        "lifecycle_acceptance", "coverage_runtime", "session", "snapshot", "output",
    ):
        generate_parser.add_argument("--" + name.replace("_", "-"), type=Path, required=True)
    generate_parser.add_argument("--safety-manager-readback", type=Path)
    generate_parser.add_argument("--runtime-binding", type=Path)
    generate_parser.add_argument("--runtime-closure", type=Path)
    generate_parser.add_argument("--runtime-install", type=Path)
    generate_parser.add_argument("--expected-safety-cap", type=float, default=0.45)
    validate_parser = subparsers.add_parser("validate")
    validate_parser.add_argument("--input", type=Path, required=True)
    validate_parser.add_argument("--session", type=Path, required=True)
    validate_parser.add_argument("--snapshot", type=Path, required=True)
    args = parser.parse_args()
    try:
        report = generate(args) if args.command == "generate" else validate(args.input, args.session, args.snapshot)
    except (BaselineError, OSError, UnicodeError, yaml.YAMLError) as exc:
        print(json.dumps({"status": "INVALID", "error": str(exc)}, indent=2))
        return 2
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
