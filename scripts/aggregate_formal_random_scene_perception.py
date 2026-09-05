#!/usr/bin/env python3
"""Aggregate disjoint live Gazebo perception episode reports fail-closed."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
from pathlib import Path
import sys

from formal_runtime_gate_binding import RuntimeGateError, load_binding


FORMAL_MINIMUM_EPISODE_COUNT = 30
FORMAL_REQUIRED_SPLIT = "val"
FORMAL_REQUIRED_VALIDATION_MAP_INDICES = tuple(range(8))
FORMAL_MINIMUM_EPISODES_PER_VALIDATION_MAP = 3
_EPISODE_ID = re.compile(r"(?P<split>[a-z]+)-map-(?P<map_index>\d{3})-mission-(?P<mission_index>\d{3})$")


def _sha256(path: Path) -> str | None:
    try:
        return hashlib.sha256(path.read_bytes()).hexdigest()
    except OSError:
        return None


def _atomic_write_json(path: Path, value: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    pending = path.with_name(path.name + f".pending.{os.getpid()}")
    try:
        pending.write_text(
            json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        pending.replace(path)
    finally:
        if pending.exists():
            pending.unlink()


def write_bound_report(output: Path, report: dict, runtime_binding_path: Path) -> None:
    """Publish the exact pre-runtime binding beside and inside the matrix report."""
    binding = load_binding(runtime_binding_path)
    report["runtime_gate_binding"] = binding
    report["acceptance_session_binding"] = binding["acceptance_session_binding"]
    report["runtime_closure_binding"] = binding["runtime_closure_binding"]
    sidecar = output.with_name(output.name + ".runtime_binding.json")
    # Keep the sidecar first: final acceptance requires it not to postdate the report.
    _atomic_write_json(sidecar, binding)
    _atomic_write_json(output, report)


def _diagnostic_evidence(episode_root: Path) -> dict:
    diagnostic_path = episode_root / "dosod_raw_diagnostic.json"
    source_manifest_path = episode_root / "product_source_manifest.sha256"
    evidence = {
        "artifact_manifest_sha256": _sha256(episode_root / "artifact_manifest.sha256"),
        "product_source_manifest_sha256": _sha256(source_manifest_path),
        "best_front_frame_sha256": _sha256(episode_root / "best_front_frame.png"),
        "dosod_raw_diagnostic_sha256": _sha256(diagnostic_path),
    }
    try:
        evidence["product_source_manifest_entries"] = [
            line.strip() for line in source_manifest_path.read_text(encoding="utf-8").splitlines()
            if line.strip()
        ]
    except OSError:
        evidence["product_source_manifest_entries"] = []
    try:
        diagnostic = json.loads(diagnostic_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return evidence
    evidence["dosod_raw_score_max_by_class"] = {
        class_id: values.get("max")
        for class_id, values in diagnostic.get("raw_scores", {}).items()
    }
    evidence["diagnostic_image_sha256"] = diagnostic.get("input", {}).get("image_sha256")
    evidence["diagnostic_evaluator_only"] = diagnostic.get("claim_boundary", {}).get(
        "evaluator_only_offline_diagnostic"
    )
    return evidence


def _episode_identity(episode_id: object) -> dict | None:
    match = _EPISODE_ID.fullmatch(str(episode_id))
    if match is None:
        return None
    return {
        "split": match.group("split"),
        "map_index": int(match.group("map_index")),
        "mission_index": int(match.group("mission_index")),
    }


def aggregate(report_paths: list[Path], minimum_episode_count: int) -> dict:
    # This program produces the formal matrix report.  Callers cannot lower the
    # evidence scale to turn the retained three-episode smoke into a product
    # acceptance result.
    effective_minimum_episode_count = max(
        FORMAL_MINIMUM_EPISODE_COUNT, int(minimum_episode_count)
    )
    reports = []
    errors = []
    for path in report_paths:
        try:
            report = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            errors.append({"path": str(path), "error": str(exc)})
            continue
        if report.get("report_id") != "tzcup_formal_random_scene_perception_episode_v1":
            errors.append({"path": str(path), "error": "unexpected report_id"})
            continue
        reports.append((path, report))
    episode_ids = [str(report.get("episode_id")) for _, report in reports]
    duplicate_ids = sorted({item for item in episode_ids if episode_ids.count(item) > 1})
    episode_identities = [_episode_identity(episode_id) for episode_id in episode_ids]
    identity_errors = [
        episode_id
        for episode_id, identity in zip(episode_ids, episode_identities, strict=True)
        if identity is None
    ]
    valid_identities = [identity for identity in episode_identities if identity is not None]
    wrong_split_ids = [
        episode_id
        for episode_id, identity in zip(episode_ids, episode_identities, strict=True)
        if identity is not None and identity["split"] != FORMAL_REQUIRED_SPLIT
    ]
    map_episode_counts = {
        f"{FORMAL_REQUIRED_SPLIT}-map-{map_index:03d}": sum(
            1
            for identity in valid_identities
            if identity["split"] == FORMAL_REQUIRED_SPLIT
            and identity["map_index"] == map_index
        )
        for map_index in FORMAL_REQUIRED_VALIDATION_MAP_INDICES
    }
    out_of_scope_map_ids = [
        episode_id
        for episode_id, identity in zip(episode_ids, episode_identities, strict=True)
        if identity is not None
        and identity["split"] == FORMAL_REQUIRED_SPLIT
        and identity["map_index"] not in FORMAL_REQUIRED_VALIDATION_MAP_INDICES
    ]
    per_episode_pass = all(report.get("status") == "PASSED" for _, report in reports)
    truth_isolation_pass = all(
        report.get("truth_isolation", {}).get("truth_published_to_ros") is False
        and report.get("truth_isolation", {}).get("truth_used_by_product_control") is False
        and report.get("truth_isolation", {}).get("synthetic_offline_image_used") is False
        for _, report in reports
    )
    split_gate = len(reports) >= effective_minimum_episode_count and not duplicate_ids
    all_validation_maps_covered = not out_of_scope_map_ids and all(
        map_episode_counts[f"{FORMAL_REQUIRED_SPLIT}-map-{map_index:03d}"] > 0
        for map_index in FORMAL_REQUIRED_VALIDATION_MAP_INDICES
    )
    per_validation_map_sample_size = all(
        map_episode_counts[f"{FORMAL_REQUIRED_SPLIT}-map-{map_index:03d}"]
        >= FORMAL_MINIMUM_EPISODES_PER_VALIDATION_MAP
        for map_index in FORMAL_REQUIRED_VALIDATION_MAP_INDICES
    )
    gates = {
        "minimum_disjoint_episode_count": split_gate,
        "all_episode_reports_well_formed": not errors,
        "formal_validation_episode_identity_present": bool(reports) and not identity_errors,
        "formal_validation_split_only": bool(reports) and not wrong_split_ids,
        "all_required_validation_maps_covered": all_validation_maps_covered,
        "minimum_episodes_per_validation_map": per_validation_map_sample_size,
        "all_episode_metric_gates_passed": bool(reports) and per_episode_pass,
        "truth_isolation_passed": bool(reports) and truth_isolation_pass,
        "pc_gazebo_camera_evidence_present": bool(reports)
        and all(
            report.get("sensor_runtime", {}).get("real_camera_message_count", 0) > 0
            for _, report in reports
        ),
    }
    detection = [report.get("litter_cube_detection", {}) for _, report in reports]
    segmentation = [report.get("ground_dirt_segmentation", {}) for _, report in reports]
    projection = [report.get("map_projection", {}) for _, report in reports]
    projection_rmse = [float(item["rmse_m"]) for item in projection if item.get("rmse_m") is not None]
    projection_p95 = [float(item["p95_m"]) for item in projection if item.get("p95_m") is not None]
    accepted = all(gates.values())
    payload = {
        "schema_version": 1,
        "report_id": "tzcup_formal_random_scene_perception_matrix_v1",
        "status": (
            "FORMAL_DOSOD_EDGESAM_RANDOM_SCENE_ACCEPTANCE_PASSED"
            if accepted
            else "FORMAL_DOSOD_EDGESAM_RANDOM_SCENE_ACCEPTANCE_BLOCKED"
        ),
        "gates": gates,
        "episode_count": len(reports),
        "minimum_episode_count": effective_minimum_episode_count,
        "requested_minimum_episode_count": minimum_episode_count,
        "episode_ids": episode_ids,
        "duplicate_episode_ids": duplicate_ids,
        "episode_identity_errors": identity_errors,
        "wrong_split_episode_ids": wrong_split_ids,
        "out_of_scope_validation_map_episode_ids": out_of_scope_map_ids,
        "input_errors": errors,
        "statistical_scope": {
            "tier": "formal_pc_gazebo_validation_matrix",
            "required_split": FORMAL_REQUIRED_SPLIT,
            "required_validation_map_indices": list(FORMAL_REQUIRED_VALIDATION_MAP_INDICES),
            "minimum_unique_validation_map_count": len(FORMAL_REQUIRED_VALIDATION_MAP_INDICES),
            "minimum_episodes_per_validation_map": FORMAL_MINIMUM_EPISODES_PER_VALIDATION_MAP,
            "episode_count_by_validation_map": map_episode_counts,
            "smoke_episode_count": 3,
            "smoke_eligible_for_final_product_evidence": False,
            "statistical_generalization_claimed": False,
        },
        "metrics_summary": {
            "cube_precision_min": min((float(item.get("precision", 0.0)) for item in detection), default=0.0),
            "cube_recall_min": min((float(item.get("recall", 0.0)) for item in detection), default=0.0),
            "cube_f1_min": min((float(item.get("f1", 0.0)) for item in detection), default=0.0),
            "ground_dirt_iou_min": min((float(item.get("iou", 0.0)) for item in segmentation), default=0.0),
            "ground_dirt_recall_min": min((float(item.get("recall", 0.0)) for item in segmentation), default=0.0),
            "map_projection_rmse_max_m": max(projection_rmse, default=None),
            "map_projection_p95_max_m": max(projection_p95, default=None),
            "false_product_track_count": sum(int(item.get("false_product_track_count", 0)) for item in projection),
        },
        # Inline the acceptance evidence and content hashes. The formal matrix
        # remains auditable after ignored .work episode directories are pruned.
        "episodes": [
            {
                "source_directory": path.parent.name,
                "source_report": path.name,
                "report_sha256": _sha256(path),
                "episode_id": report.get("episode_id"),
                "status": report.get("status"),
                "blocked_checks": report.get("blocked_checks", {}),
                "litter_cube_detection": report.get("litter_cube_detection", {}),
                "ground_dirt_segmentation": report.get("ground_dirt_segmentation", {}),
                "map_projection": report.get("map_projection", {}),
                "sensor_runtime": report.get("sensor_runtime", {}),
                "truth_isolation": report.get("truth_isolation", {}),
                "artifact_evidence": _diagnostic_evidence(path.parent),
            }
            for path, report in reports
        ],
        "truth_isolation": {
            "truth_reader": "per_episode_evaluator_process_only",
            "truth_published_to_ros": False,
            "truth_used_by_product_control": False,
            "synthetic_offline_image_used": False,
        },
        "claim_boundary": {
            "pc_product_perception_accepted": accepted,
            "pc_gazebo_validation_matrix_accepted": accepted,
            "s100_board_accepted": False,
            "real_vehicle_accepted": False,
            "no_fine_tuning_performed": True,
            "real_world_accuracy_claimed": False,
        },
    }
    return payload


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--runtime-binding", type=Path, required=True)
    parser.add_argument("--minimum-episodes", type=int, default=FORMAL_MINIMUM_EPISODE_COUNT)
    args = parser.parse_args()
    report = aggregate(sorted(args.input_root.glob("episode-*/perception_acceptance.json")), args.minimum_episodes)
    try:
        write_bound_report(args.output, report, args.runtime_binding)
    except (OSError, RuntimeGateError, TypeError, ValueError, KeyError) as exc:
        print(f"FORMAL_RANDOM_SCENE_PERCEPTION_BINDING_BLOCKED: {exc}", file=sys.stderr)
        return 3
    print(json.dumps({"output": str(args.output), "status": report["status"]}, ensure_ascii=False))
    return (
        0
        if report["status"] == "FORMAL_DOSOD_EDGESAM_RANDOM_SCENE_ACCEPTANCE_PASSED"
        else 3
    )


if __name__ == "__main__":
    raise SystemExit(main())
