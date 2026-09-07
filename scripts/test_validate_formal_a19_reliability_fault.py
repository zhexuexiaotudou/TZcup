#!/usr/bin/env python3
"""Focused fixtures for the A19 reliability/fault evidence boundary."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

from validate_formal_a19_reliability_fault import BLOCKED, PASS, validate


def _write(path: Path, value: object) -> str:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value), encoding="utf-8")
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _fixtures(tmp_path: Path, duration_s: int = 7200) -> tuple[Path, Path, Path, Path, Path]:
    evidence = tmp_path / "evidence"
    snapshot = tmp_path / "snapshot.json"
    _write(snapshot, {"source_inventory_sha256": "a" * 64, "outputs": {"reports/engineering/formal_competition_vehicle.urdf": {"sha256": "b" * 64}}})
    closure = tmp_path / "closure.json"
    _write(closure, {"kind": "tzcup_formal_final_runtime_closure", "status": "FORMAL_FINAL_RUNTIME_CLOSURE_FROZEN", "closure_sha256": "c" * 64})
    snapshot_binding = {"snapshot_manifest_sha256": hashlib.sha256(snapshot.read_bytes()).hexdigest(), "source_inventory_sha256": "a" * 64, "expanded_urdf_sha256": "b" * 64}
    closure_binding = {"runtime_closure_manifest_sha256": hashlib.sha256(closure.read_bytes()).hexdigest(), "runtime_closure_sha256": "c" * 64}
    session = tmp_path / "session.json"
    _write(session, {"report_id": "tzcup_formal_final_acceptance_session_v1", "status": "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING", "started_epoch_ns": 10, "snapshot": snapshot_binding, "runtime_closure_binding": {"manifest_sha256": closure_binding["runtime_closure_manifest_sha256"], "closure_sha256": "c" * 64}})
    session_binding = {"session_manifest_sha256": hashlib.sha256(session.read_bytes()).hexdigest(), "session_started_epoch_ns": 10, "snapshot": snapshot_binding}
    collection = {"started_epoch_ns": 20, "ended_epoch_ns": 20 + duration_s * 1_000_000_000}
    telemetry = {"collection": collection, "coverage_ratio": 0.9, "collision_count": 0, "localization_p95_m": 0.2, "max_estop_brake_latency_s": 1.0}
    telemetry_hash = _write(evidence / "telemetry.json", telemetry)
    fault_rows = [{"profile": profile, "injected_epoch_ns": 30, "safety_outcome": "STOPPED_SAFE", "recovery_outcome": "RECOVERED_SAFE"} for profile in ("transport_stress", "wet_surface", "degraded_drive")]
    fault_hash = _write(evidence / "faults.json", {"fault_injections": fault_rows})
    log_path = evidence / "launch.log"
    log_path.write_text("retained launch log\n", encoding="utf-8")
    log_hash = hashlib.sha256(log_path.read_bytes()).hexdigest()
    report = tmp_path / "candidate.json"
    _write(report, {"schema_version": 1, "report_id": "tzcup_formal_a19_reliability_fault_report_v1", "acceptance_session_binding": session_binding, "runtime_closure_binding": closure_binding, "collection": collection, "raw_evidence": {"telemetry": {"path": "telemetry.json", "sha256": telemetry_hash}, "launch_log": {"path": "launch.log", "sha256": log_hash}, "fault_injection": {"path": "faults.json", "sha256": fault_hash}}, "metrics": {key: telemetry[key] for key in ("coverage_ratio", "collision_count", "localization_p95_m", "max_estop_brake_latency_s")}, "fault_injections": fault_rows})
    return report, snapshot, session, closure, evidence


def test_current_two_hour_fixture_passes(tmp_path: Path) -> None:
    report, snapshot, session, closure, evidence = _fixtures(tmp_path)
    assert validate(report, snapshot, session, closure, evidence)["status"] == PASS


def test_short_soak_cannot_be_relabelled_as_a19_pass(tmp_path: Path) -> None:
    report, snapshot, session, closure, evidence = _fixtures(tmp_path, duration_s=65)
    result = validate(report, snapshot, session, closure, evidence)
    assert result["status"] == BLOCKED
    assert any("7200" in blocker for blocker in result["blockers"])


def test_missing_fault_profile_blocks_even_with_good_metrics(tmp_path: Path) -> None:
    report, snapshot, session, closure, evidence = _fixtures(tmp_path)
    payload = json.loads(report.read_text(encoding="utf-8"))
    payload["fault_injections"] = payload["fault_injections"][:-1]
    fault_hash = _write(evidence / "faults.json", {"fault_injections": payload["fault_injections"]})
    payload["raw_evidence"]["fault_injection"]["sha256"] = fault_hash
    _write(report, payload)
    result = validate(report, snapshot, session, closure, evidence)
    assert result["status"] == BLOCKED
    assert any("missing fault profiles" in blocker for blocker in result["blockers"])


def test_closure_mismatch_blocks_candidate(tmp_path: Path) -> None:
    report, snapshot, session, closure, evidence = _fixtures(tmp_path)
    payload = json.loads(report.read_text(encoding="utf-8"))
    payload["runtime_closure_binding"]["runtime_closure_sha256"] = "d" * 64
    _write(report, payload)
    result = validate(report, snapshot, session, closure, evidence)
    assert result["status"] == BLOCKED
    assert "candidate report does not bind the current runtime closure" in result["blockers"]
