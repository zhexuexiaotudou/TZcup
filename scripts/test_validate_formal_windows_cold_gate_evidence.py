from __future__ import annotations

import json
from pathlib import Path

import pytest

from validate_formal_windows_cold_gate_evidence import EvidenceError, validate_evidence


NOW_NS = 1_788_000_000_000_000_000


def _payload() -> dict[str, object]:
    recorded = NOW_NS - 1_000_000_000
    return {
        "report_id": "tzcup_formal_windows_memory_start_gate_v1",
        "status": "FORMAL_WINDOWS_MEMORY_START_GATE_PASSED",
        "passed": True,
        "recorded_epoch_ns": recorded,
        "require_wsl_stopped": True,
        "require_wsl_running": False,
        "thresholds_bytes": {
            "min_commit_available": 13_421_772_800,
            "max_docker_private": 4_294_967_296,
        },
        "sample": {
            "epoch_ns": recorded - 10_000_000,
            "commit_available_bytes": 14_000_000_000,
            "docker_private_bytes": 0,
            "vmmem_wsl_private_bytes": 0,
        },
        "checks": {
            "windows_commit_available_at_least_configured_minimum": True,
            "docker_private_at_most_configured_maximum": True,
            "wsl_vm_stopped_when_required": True,
            "wsl_vm_running_when_required": True,
        },
        "docker_was_signalled_or_stopped": False,
    }


def _write(path: Path, payload: dict[str, object]) -> None:
    path.write_text(json.dumps(payload), encoding="utf-8")


def test_accepts_fresh_strict_cold_start_evidence(tmp_path: Path) -> None:
    path = tmp_path / "gate.json"
    _write(path, _payload())
    assert validate_evidence(path, now_ns=NOW_NS)["passed"] is True


def test_rejects_evidence_that_did_not_require_wsl_stopped(tmp_path: Path) -> None:
    payload = {**_payload(), "require_wsl_stopped": False}
    path = tmp_path / "gate.json"
    _write(path, payload)
    with pytest.raises(EvidenceError, match="did not require WSL"):
        validate_evidence(path, now_ns=NOW_NS)


def test_rejects_stale_evidence(tmp_path: Path) -> None:
    recorded = NOW_NS - 301_000_000_000
    payload = {
        **_payload(),
        "recorded_epoch_ns": recorded,
        "sample": {**_payload()["sample"], "epoch_ns": recorded - 10_000_000},
    }
    path = tmp_path / "gate.json"
    _write(path, payload)
    with pytest.raises(EvidenceError, match="stale"):
        validate_evidence(path, now_ns=NOW_NS)


def test_rejects_weakened_thresholds_and_running_wsl(tmp_path: Path) -> None:
    payload = _payload()
    payload["thresholds_bytes"] = {
        "min_commit_available": 10 * 1024**3,
        "max_docker_private": 8 * 1024**3,
    }
    payload["sample"] = {**payload["sample"], "vmmem_wsl_private_bytes": 1}
    path = tmp_path / "gate.json"
    _write(path, payload)
    with pytest.raises(EvidenceError, match="below 12.5 GiB"):
        validate_evidence(path, now_ns=NOW_NS)
