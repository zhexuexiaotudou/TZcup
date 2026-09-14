#!/usr/bin/env python3
"""Fail-closed composite-component evidence contract for simulation only.

It emits component evidence only. The source-bound components are not a causal
same-scene mission chain.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
from typing import Any


COMPLETE_STATUS = "COMPETITION_SIM_ONLY_COMPOSITE_COMPONENT_EVIDENCE_COMPLETE"
BLOCKED_STATUS = "COMPETITION_SIM_ONLY_COMPOSITE_COMPONENT_EVIDENCE_BLOCKED"

PERCEPTION_REPORT_ID = "tzcup_competition_sim_only_online_perception_v1"
PERCEPTION_STATUS = "COMPETITION_SIM_ONLY_ONLINE_PERCEPTION_PASSED"
EMERGENCY_REPORT_ID = "tzcup_competition_sim_only_emergency_braking_v1"
EMERGENCY_STATUS = "COMPETITION_SIM_ONLY_EMERGENCY_BRAKING_PASSED"
POST_CLEAN_REPORT_ID = "tzcup_competition_sim_only_post_clean_verification_v1"
POST_CLEAN_STATUS = "COMPETITION_SIM_ONLY_POST_CLEAN_VERIFICATION_PASSED"


def _read(path: Path | None) -> dict[str, Any] | None:
    if path is None:
        return None
    if path.is_symlink() or not path.is_file():
        return None
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError):
        return None
    return value if isinstance(value, dict) else None


def _artifact_identity(path: Path | None) -> dict[str, Any] | None:
    if path is None or path.is_symlink() or not path.is_file():
        return None
    return {
        "path": str(path.resolve()),
        "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "byte_size": path.stat().st_size,
    }


def _passed(report: dict[str, Any] | None, status: str) -> bool:
    return bool(report and report.get("status") == status and report.get("passed") is True)


def _source_identity(report: dict[str, Any] | None) -> tuple[str, str] | None:
    if not report:
        return None
    binding = report.get("acceptance_session_binding")
    if not isinstance(binding, dict):
        binding = (report.get("runtime_gate_binding") or {}).get("acceptance_session_binding")
    if not isinstance(binding, dict):
        return None
    session_hash = binding.get("session_manifest_sha256")
    snapshot = binding.get("snapshot")
    source_hash = snapshot.get("source_inventory_sha256") if isinstance(snapshot, dict) else None
    if not isinstance(session_hash, str) or len(session_hash) != 64:
        return None
    if not isinstance(source_hash, str) or len(source_hash) != 64:
        return None
    return session_hash, source_hash


def _component_report(report: dict[str, Any] | None, report_id: str, status: str) -> bool:
    return bool(report and report.get("report_id") == report_id and report.get("status") == status and report.get("passed") is True)


def _perception_passed(report: dict[str, Any] | None) -> bool:
    if not _component_report(report, PERCEPTION_REPORT_ID, PERCEPTION_STATUS):
        return False
    return (
        report.get("online_inference") is True
        and report.get("garbage_detection_and_localization") is True
        and report.get("truth_used_for_control") is False
        and _source_identity(report) is not None
    )


def _emergency_passed(report: dict[str, Any] | None) -> bool:
    if not _component_report(report, EMERGENCY_REPORT_ID, EMERGENCY_STATUS):
        return False
    delay = report.get("emergency_braking_s")
    return (
        isinstance(delay, (int, float)) and not isinstance(delay, bool) and delay <= 1.0
        and report.get("emergency_stop_command_observed") is True
        and report.get("safe_recovery_verified") is True
        and _source_identity(report) is not None
    )


def _post_clean_passed(report: dict[str, Any] | None) -> bool:
    return bool(
        _component_report(report, POST_CLEAN_REPORT_ID, POST_CLEAN_STATUS)
        and report.get("post_clean_verification") is True
        and report.get("truth_used_for_control") is False
        and _source_identity(report) is not None
    )


def evaluate(
    map_lifecycle: dict[str, Any] | None,
    ground_dirt: dict[str, Any] | None,
    dynamic_obstacle: dict[str, Any] | None,
    perception: dict[str, Any] | None,
    emergency: dict[str, Any] | None,
    post_clean: dict[str, Any] | None,
) -> dict[str, Any]:
    identities = [_source_identity(report) for report in (
        map_lifecycle, ground_dirt, dynamic_obstacle, perception, emergency, post_clean
    )]
    source_bound = all(identity is not None for identity in identities) and len(set(identities)) == 1
    checks = {
        "saved_map_lifecycle_passed": _passed(map_lifecycle, "FORMAL_FIRST_MAP_THEN_SAVED_MAP_CLEANING_PASSED"),
        "ground_dirt_physical_cleaning_passed": _passed(ground_dirt, "FORMAL_GROUND_DIRT_PHYSICAL_CLEANING_PASSED"),
        "dynamic_obstacle_avoidance_passed": _passed(dynamic_obstacle, "FORMAL_DYNAMIC_OBSTACLE_AVOIDANCE_ACCEPTANCE_PASSED"),
        "online_garbage_detection_and_localization_passed": _perception_passed(perception),
        "emergency_braking_and_recovery_passed": _emergency_passed(emergency),
        "post_clean_verification_passed": _post_clean_passed(post_clean),
        "all_simulation_reports_share_frozen_session_and_source": source_bound,
        "grasp_is_explicitly_not_executed": True,
        "a12_contract_not_used": True,
    }
    complete = all(checks.values())
    return {
        "schema_version": 1,
        "report_id": "tzcup_competition_sim_only_composite_component_evidence_v1",
        "status": COMPLETE_STATUS if complete else BLOCKED_STATUS,
        "component_evidence_complete": complete,
        "checks": checks,
        "blockers": [name for name, value in checks.items() if not value],
        "evidence_kind": "COMPOSITE_COMPONENT_EVIDENCE_ONLY",
        "grasp_status": "NOT_EXECUTED",
        "cube_target_count": 0,
        "a12_contract_used": False,
        "actual_cleaning_evidence": "GroundDirtCleaningSystem physical-cleaning report; saved-map centerline telemetry is planning-only.",
        "claim_boundary": "Separate source-bound component runs only; not a causal same-scene chain, A12 replacement, or grasp completion.",
    }


def _write(path: Path, report: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    pending = path.with_name(path.name + f".pending.{os.getpid()}")
    pending.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    pending.replace(path)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map-lifecycle-report", type=Path, required=True)
    parser.add_argument("--ground-dirt-report", type=Path, required=True)
    parser.add_argument("--dynamic-obstacle-report", type=Path, required=True)
    parser.add_argument("--perception-report", type=Path)
    parser.add_argument("--emergency-braking-report", type=Path)
    parser.add_argument("--post-clean-report", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise SystemExit("output must not exist")
    report = evaluate(
        _read(args.map_lifecycle_report), _read(args.ground_dirt_report),
        _read(args.dynamic_obstacle_report), _read(args.perception_report),
        _read(args.emergency_braking_report), _read(args.post_clean_report),
    )
    report["input_artifacts"] = {
        "map_lifecycle": _artifact_identity(args.map_lifecycle_report),
        "ground_dirt": _artifact_identity(args.ground_dirt_report),
        "dynamic_obstacle": _artifact_identity(args.dynamic_obstacle_report),
        "perception": _artifact_identity(args.perception_report),
        "emergency_braking": _artifact_identity(args.emergency_braking_report),
        "post_clean": _artifact_identity(args.post_clean_report),
    }
    _write(args.output, report)
    return 0 if report["component_evidence_complete"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
