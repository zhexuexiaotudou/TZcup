#!/usr/bin/env python3
"""Focused negative tests for the current A19 producer boundary."""

from __future__ import annotations

import json
from pathlib import Path

from validate_formal_a19_reliability_fault import BLOCKED, ROOT, _safe_output, validate


def _write(path: Path, payload: object) -> None:
    path.write_text(json.dumps(payload), encoding="utf-8")


def _candidate(tmp_path: Path) -> tuple[Path, Path, Path, Path, Path]:
    evidence = tmp_path / "evidence"; evidence.mkdir()
    paths = tuple(evidence / name for name in ("report.json", "snapshot.json", "session.json", "closure.json"))
    for path in paths: _write(path, {})
    return *paths, evidence


def _validate(paths: tuple[Path, Path, Path, Path, Path]) -> dict:
    return validate(*paths, repository_root=ROOT, evidence_parent=paths[-1].parent)


def test_contract_preserves_all_product_standard_a19_requirements() -> None:
    contract = json.loads((ROOT / "config/high_fidelity_vehicle/formal_a19_reliability_fault_contract.json").read_text(encoding="utf-8"))
    assert contract["required_profiles"] == ["nominal", "transport_stress", "wet_surface", "degraded_drive"]
    assert len(contract["required_faults"]) == 18
    assert contract["stability_gates"]["maximum_localization_xy_p95_m"] == 0.05
    assert contract["canonical_producer"]["available_on_current_main"] is False


def test_hand_authored_json_is_blocked_without_a_canonical_producer(tmp_path: Path) -> None:
    result = _validate(_candidate(tmp_path))
    assert result["status"] == BLOCKED and not result["passed"]
    assert any("canonical current-main runtime collector" in item for item in result["blockers"])
    assert any("future canonical producer must parse" in item for item in result["blockers"])


def test_evidence_root_with_parent_traversal_is_rejected_before_normalization(tmp_path: Path) -> None:
    paths = _candidate(tmp_path); outside = tmp_path / "outside"; outside.mkdir()
    result = validate(paths[0], paths[1], paths[2], paths[3], paths[4] / ".." / "outside", repository_root=ROOT, evidence_parent=tmp_path)
    assert result["status"] == BLOCKED
    assert any("parent traversal is forbidden" in item for item in result["blockers"])


def test_report_and_output_escape_are_rejected(tmp_path: Path) -> None:
    paths = _candidate(tmp_path); outside = tmp_path / "outside.json"; _write(outside, {})
    result = validate(outside, paths[1], paths[2], paths[3], paths[4], repository_root=ROOT, evidence_parent=tmp_path)
    assert any("report: path escapes root" in item for item in result["blockers"])
    try:
        _safe_output(paths[4] / ".." / "outside.json", paths[4])
    except ValueError as exc:
        assert "parent traversal is forbidden" in str(exc)
    else:
        raise AssertionError("output traversal must fail")
