#!/usr/bin/env python3
"""Adversarial fixtures: current main must never manufacture an A19 PASS."""

from __future__ import annotations

import json
import time
from pathlib import Path

from validate_formal_a19_reliability_fault import BLOCKED, ROOT, validate


def _write(path: Path, payload: object) -> None:
    path.write_text(json.dumps(payload), encoding="utf-8")


def _record(fault_id: str, stamp: int) -> dict:
    event = [{"state": "STOPPED", "wall_clock_epoch_ns": stamp}]
    return {"fault_id": fault_id, "safety_state_events": event, "recovery_state_events": [{"state": "RECOVERED", "wall_clock_epoch_ns": stamp + 1}], "perception_health_events": [{"state": "DEGRADED", "wall_clock_epoch_ns": stamp}], "safety_nav_operational": True, "unsafe_pending_clean_outcome": "CANCELLED"}


def _candidate(tmp_path: Path) -> tuple[Path, Path, Path, Path, Path]:
    evidence = tmp_path / "evidence"; evidence.mkdir()
    now = time.time_ns(); start = now - 7_201_000_000_000
    report = {"profiles": ["nominal", "transport_stress", "wet_surface", "degraded_drive"], "faults": [_record(name, start + 1) for name in ("rgb_freeze", "depth_freeze", "timestamp_skew", "camera_info_mismatch", "tf_unavailable", "invalid_depth", "proposal_flood", "proposal_dropout", "classifier_exception", "classifier_timeout", "action_verifier_failure", "reobserve_timeout", "cuda_provider_failure", "model_hash_mismatch", "corrupt_model", "sustained_slow_inference", "nav2_path_unavailable", "dynamic_obstacle_blocks_observation")], "stability_metrics": {"crash_count": 0, "deadlock_count": 0, "memory_growth_ratio": 0.05, "queue_growth_count": 0, "unexpected_model_reload_count": 0, "persistent_tf_failure_count": 0, "unrecoverable_watchdog_event_count": 0, "unsafe_cleaning_action_count": 0, "localization_xy_rmse_m": 0.05, "localization_xy_p95_m": 0.05, "estop_brake_latency_s": 1.0, "pipeline_components": ["Coverage", "Perception", "Tracking", "DynamicTrashMap", "Spot Cleaning", "Post-Clean Verification"]}, "wall_clock_collection": {"started_epoch_ns": start, "ended_epoch_ns": now}, "raw_evidence": {name: {} for name in ("wall_clock_time_series", "launch_command", "process_exit", "zero_survivor", "fault_injection_receipt")}}
    report_path = evidence / "report.json"; _write(report_path, report)
    snapshot = evidence / "snapshot.json"; _write(snapshot, {"source_inventory_sha256": "a" * 64})
    session = evidence / "session.json"; _write(session, {"started_epoch_ns": start - 1})
    closure = evidence / "closure.json"; _write(closure, {"closure_sha256": "b" * 64})
    return report_path, snapshot, session, closure, evidence


def _validate(paths: tuple[Path, Path, Path, Path, Path]) -> dict:
    return validate(*paths, repository_root=ROOT)


def test_complete_hand_authored_static_claim_is_still_blocked(tmp_path: Path) -> None:
    result = _validate(_candidate(tmp_path))
    assert result["status"] == BLOCKED and not result["passed"]
    assert any("canonical current-main runtime collector" in item for item in result["blockers"])


def test_duplicate_or_unknown_profile_is_rejected(tmp_path: Path) -> None:
    paths = _candidate(tmp_path); payload = json.loads(paths[0].read_text()); payload["profiles"] = ["nominal", "nominal", "wet_surface", "unknown"]; _write(paths[0], payload)
    assert any("profiles must be" in item for item in _validate(paths)["blockers"])


def test_missing_one_of_the_eighteen_faults_is_rejected(tmp_path: Path) -> None:
    paths = _candidate(tmp_path); payload = json.loads(paths[0].read_text()); payload["faults"] = payload["faults"][:-1]; _write(paths[0], payload)
    assert any("fault ids must be" in item for item in _validate(paths)["blockers"])


def test_each_fault_requires_a_nonempty_injection_parameter_record(tmp_path: Path) -> None:
    result = _validate(_candidate(tmp_path))
    assert any("rgb_freeze lacks injection parameters" in item for item in result["blockers"])


def test_51mm_localization_and_string_states_are_rejected(tmp_path: Path) -> None:
    paths = _candidate(tmp_path); payload = json.loads(paths[0].read_text()); payload["stability_metrics"]["localization_xy_p95_m"] = 0.051; payload["faults"][0]["safety_state_events"] = "STOPPED"; _write(paths[0], payload)
    result = _validate(paths)
    assert any("localization_xy_p95_m" in item for item in result["blockers"])
    assert any("timestamped STOPPED" in item for item in result["blockers"])


def test_future_and_pre_session_time_are_rejected(tmp_path: Path) -> None:
    paths = _candidate(tmp_path); payload = json.loads(paths[0].read_text()); payload["wall_clock_collection"]["ended_epoch_ns"] = time.time_ns() + 60_000_000_000; payload["wall_clock_collection"]["started_epoch_ns"] = 0; _write(paths[0], payload)
    result = _validate(paths)
    assert any("future" in item for item in result["blockers"])
    assert any("predates" in item for item in result["blockers"])
