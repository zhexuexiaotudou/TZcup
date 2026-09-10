#!/usr/bin/env python3
"""Contract, fixture and negative tests for the canonical A19 producer."""

from __future__ import annotations

import copy
import json
import os
import sys
from pathlib import Path

import pytest

SCRIPT_ROOT = Path(__file__).resolve().parent
if str(SCRIPT_ROOT) not in sys.path:
    sys.path.insert(0, str(SCRIPT_ROOT))

import produce_formal_a19_reliability_fault as producer
from produce_formal_a19_reliability_fault import (
    DEFAULT_CONTRACT,
    PROTOCOL,
    ROOT,
    A19ProducerError,
    _parse_adapter_argv,
    execute_capture,
    validate_contract,
)
from validate_formal_a19_reliability_fault import (
    _parse_events,
    _safe_output,
    _validate_semantics,
)


FIXTURE = ROOT / "scripts/fixtures/formal_a19_adapter_fixture.py"


def _contract() -> dict:
    return json.loads(DEFAULT_CONTRACT.read_text(encoding="utf-8"))


def _fast_contract() -> dict:
    contract = copy.deepcopy(_contract())
    contract["stability_gates"].update(
        minimum_duration_s=0.8,
        sample_period_s=0.005,
        maximum_sample_gap_s=0.1,
        minimum_sample_count=100,
        memory_comparison_window_s=0.05,
    )
    starts = [0.0, 0.2, 0.4, 0.6]
    for row, start in zip(contract["profile_schedule"], starts):
        row["start_offset_s"] = start
        row["minimum_observed_duration_s"] = 0.12
    for index, row in enumerate(contract["fault_schedule"], start=1):
        row["offset_s"] = index * 0.04
    contract["fault_timing"].update(
        maximum_injection_lateness_s=0.1,
        maximum_safe_stop_latency_s=0.1,
        maximum_recovery_latency_s=0.1,
    )
    return contract


def test_contract_preserves_two_hours_four_profiles_and_all_18_faults() -> None:
    contract = _contract()
    validate_contract(contract)
    assert contract["current_state"] == "READY_FOR_FRESH_TWO_HOUR_RUNTIME"
    assert contract["stability_gates"]["minimum_duration_s"] == 7200
    assert [row["profile"] for row in contract["profile_schedule"]] == [
        "nominal", "transport_stress", "wet_surface", "degraded_drive"
    ]
    assert len(contract["fault_schedule"]) == 18
    assert len({row["fault"] for row in contract["fault_schedule"]}) == 18
    assert contract["canonical_producer"]["available_on_current_source"] is True


def test_formal_cli_rejects_the_test_fixture_adapter() -> None:
    with pytest.raises(A19ProducerError, match="forbids the test fixture"):
        _parse_adapter_argv(
            json.dumps([sys.executable, str(FIXTURE)]), _contract()
        )
    assert _parse_adapter_argv(
        json.dumps([sys.executable, str(FIXTURE)]), _contract(), allow_fixture=True
    )[1] == str(FIXTURE)


@pytest.mark.skipif(os.name != "posix", reason="process-group supervisor is POSIX/WSL-only")
def test_fixture_exercises_supervisor_and_all_fault_semantics(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    contract = _fast_contract()
    contract_path = tmp_path / "contract.json"
    contract_path.write_text(json.dumps(contract), encoding="utf-8")
    evidence_root = tmp_path / "evidence"
    binding = {
        "schema_version": 1,
        "status": "FORMAL_RUNTIME_GATE_BOUND",
        "acceptance_session_binding": {"session_status_at_gate": "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING"},
        "runtime_closure_binding": {"status": "FORMAL_FINAL_RUNTIME_CLOSURE_VERIFIED"},
    }
    source = {"repository_commit": "a" * 40, "repository_tree": "b" * 40, "tracked_worktree_clean": True}

    def fake_preflight(**kwargs):
        del kwargs
        return {
            "evidence_root": str(evidence_root),
            "adapter_argv": [sys.executable, str(FIXTURE)],
            "contract_sha256": producer._sha256(contract_path),
            "source_binding": source,
            "runtime_gate_binding": binding,
            "adapter_file_identities": [],
        }

    monkeypatch.setattr(producer, "preflight", fake_preflight)
    monkeypatch.setattr(producer, "_git_identity", lambda root: source)
    monkeypatch.setattr(producer, "build_binding", lambda **kwargs: binding)
    receipt, status = execute_capture(
        repository_root=tmp_path,
        contract_path=contract_path,
        install_root=tmp_path,
        closure_manifest=tmp_path / "closure.json",
        session_path=tmp_path / "session.json",
        snapshot_path=tmp_path / "snapshot.json",
        evidence_root=evidence_root,
        adapter_argv_json=json.dumps([sys.executable, str(FIXTURE)]),
        allow_fixture=True,
    )
    assert status == 0
    assert receipt["execution"]["exit_code"] == 0
    assert receipt["execution"]["zero_survivors"] is True
    event_data = (evidence_root / "adapter_events.jsonl").read_bytes()
    events = _parse_events(event_data)
    errors, detail = _validate_semantics(receipt, events, contract)
    assert errors == []
    assert detail["metrics"]["fault_count"] == 18
    assert detail["metrics"]["passed_fault_count"] == 18
    assert detail["metrics"]["sample_count"] >= 100


@pytest.mark.skipif(os.name != "posix", reason="process-group supervisor is POSIX/WSL-only")
def test_validator_rejects_unsafe_action_and_missing_recovery(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    contract = _fast_contract()
    contract_path = tmp_path / "contract.json"
    contract_path.write_text(json.dumps(contract), encoding="utf-8")
    evidence_root = tmp_path / "evidence"
    binding = {
        "schema_version": 1, "status": "FORMAL_RUNTIME_GATE_BOUND",
        "acceptance_session_binding": {}, "runtime_closure_binding": {},
    }
    source = {"repository_commit": "a" * 40, "repository_tree": "b" * 40, "tracked_worktree_clean": True}
    monkeypatch.setattr(producer, "preflight", lambda **kwargs: {
        "evidence_root": str(evidence_root), "adapter_argv": [sys.executable, str(FIXTURE)],
        "contract_sha256": producer._sha256(contract_path), "source_binding": source,
        "runtime_gate_binding": binding,
        "adapter_file_identities": [],
    })
    monkeypatch.setattr(producer, "_git_identity", lambda root: source)
    monkeypatch.setattr(producer, "build_binding", lambda **kwargs: binding)
    receipt, _ = execute_capture(
        repository_root=tmp_path, contract_path=contract_path, install_root=tmp_path,
        closure_manifest=tmp_path / "closure", session_path=tmp_path / "session",
        snapshot_path=tmp_path / "snapshot", evidence_root=evidence_root,
        adapter_argv_json=json.dumps([sys.executable, str(FIXTURE)]), allow_fixture=True,
    )
    events = _parse_events((evidence_root / "adapter_events.jsonl").read_bytes())
    sample = next(row["adapter"] for row in events if row["adapter"].get("type") == "sample")
    sample["metrics"]["unsafe_cleaning_action_count"] = 1
    first_fault = contract["fault_schedule"][0]["fault"]
    events = [row for row in events if not (
        row["adapter"].get("type") == "fault_state"
        and row["adapter"].get("fault") == first_fault
        and row["adapter"].get("state") == "RECOVERED"
    )]
    for index, row in enumerate(events):
        row["producer_sequence"] = index
        row["adapter"]["adapter_sequence"] = index
    errors, _ = _validate_semantics(receipt, events, contract)
    assert any("unsafe_cleaning_action_count" in error for error in errors)
    assert any(first_fault in error and "RECOVERED" in error for error in errors)


def test_output_is_fresh_and_cannot_escape_repository(tmp_path: Path) -> None:
    root = tmp_path / "repo"; root.mkdir()
    parent = root / "artifacts"; parent.mkdir()
    output = parent / "receipt.json"
    _safe_output(output, root)
    output.write_text("{}", encoding="utf-8")
    with pytest.raises(ValueError, match="overwrite"):
        _safe_output(output, root)
    with pytest.raises(ValueError, match="parent traversal"):
        _safe_output(root / ".." / "outside.json", root)


def test_runner_requires_json_argv_and_never_uses_eval() -> None:
    source = (ROOT / "scripts/run_formal_a19_reliability_fault.sh").read_text(encoding="utf-8")
    assert "FORMAL_A19_ADAPTER_ARGV_JSON" in source
    assert "run_formal_runtime_isolation.sh" in source
    assert "formal_runtime_configure" in source
    assert "eval " not in source
    assert "--preflight" in source
