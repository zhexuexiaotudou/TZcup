from __future__ import annotations

import csv
import shutil
from pathlib import Path

import pytest
import yaml

from validate_formal_vehicle_mechanical_manufacturing_preparation_draft import (
    DEFAULT_DRAFT,
    ROOT,
    validate,
)


def _fixture(tmp_path: Path) -> Path:
    payload = yaml.safe_load(DEFAULT_DRAFT.read_text(encoding="utf-8"))
    for entry in payload["source_design_inputs"].values():
        source = ROOT / entry["path"]
        target = tmp_path / entry["path"]
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy(source, target)
    target = tmp_path / "mechanical_manufacturing_preparation_draft.yaml"
    target.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")
    return target


def _write(path: Path, payload: dict) -> None:
    path.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")


def test_valid_draft_remains_explicitly_not_ready(tmp_path: Path) -> None:
    draft = _fixture(tmp_path)
    result = validate(draft, root=tmp_path)
    assert result["valid"] is True
    assert result["status"] == "DRAFT_DESIGN_INPUT_NOT_RELEASED"
    assert result["ready_for_manufacturing_release"] is False
    assert result["computed"] == {"bom_row_count": 33, "subassembly_count": 18}


def test_rejects_any_promotion_to_manufacturing_ready(tmp_path: Path) -> None:
    draft = _fixture(tmp_path)
    payload = yaml.safe_load(draft.read_text(encoding="utf-8"))
    payload["status"] = "READY_FOR_MANUFACTURING_RELEASE"
    payload["ready_for_manufacturing_release"] = True
    _write(draft, payload)
    result = validate(draft, root=tmp_path)
    assert result["valid"] is False
    assert "status must equal DRAFT_DESIGN_INPUT_NOT_RELEASED" in result["errors"]
    assert "ready_for_manufacturing_release must be boolean false" in result["errors"]


def test_rejects_audit_claiming_release_evidence(tmp_path: Path) -> None:
    draft = _fixture(tmp_path)
    payload = yaml.safe_load(draft.read_text(encoding="utf-8"))
    payload["audit_outcome"]["release_evidence_created"] = True
    _write(draft, payload)
    result = validate(draft, root=tmp_path)
    assert result["valid"] is False
    assert "audit_outcome.release_evidence_created must be boolean false" in result["errors"]


def test_rejects_non_pending_fastener_torque(tmp_path: Path) -> None:
    draft = _fixture(tmp_path)
    payload = yaml.safe_load(draft.read_text(encoding="utf-8"))
    payload["fastener_schedule"]["records"][0]["release_torque_or_preload"] = "25 Nm"
    _write(draft, payload)
    result = validate(draft, root=tmp_path)
    assert result["valid"] is False
    assert any("release_torque_or_preload must retain a pending:// release boundary" in error for error in result["errors"])


def test_rejects_changed_source_bom_row_count(tmp_path: Path) -> None:
    draft = _fixture(tmp_path)
    payload = yaml.safe_load(draft.read_text(encoding="utf-8"))
    bom_path = tmp_path / payload["source_design_inputs"]["manufacturing_bom"]["path"]
    with bom_path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        fields = list(reader.fieldnames or [])
        rows = list(reader)
    with bom_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows[:-1])
    result = validate(draft, root=tmp_path)
    assert result["valid"] is False
    assert "manufacturing BOM row count does not match the frozen draft input" in result["errors"]


def test_rejects_promoted_mechanical_release_readiness(tmp_path: Path) -> None:
    draft = _fixture(tmp_path)
    payload = yaml.safe_load(draft.read_text(encoding="utf-8"))
    readiness = tmp_path / payload["source_design_inputs"]["mechanical_release_readiness"]["path"]
    readiness_payload = yaml.safe_load(readiness.read_text(encoding="utf-8"))
    readiness_payload["ready"] = True
    _write(readiness, readiness_payload)
    result = validate(draft, root=tmp_path)
    assert result["valid"] is False
    assert "mechanical release readiness must remain false" in result["errors"]


def test_rejects_removed_manufacturing_release_hold_point(tmp_path: Path) -> None:
    draft = _fixture(tmp_path)
    payload = yaml.safe_load(draft.read_text(encoding="utf-8"))
    payload["required_release_hold_points"].remove("native_step_or_stp_models")
    _write(draft, payload)
    result = validate(draft, root=tmp_path)
    assert result["valid"] is False
    assert "required_release_hold_points must preserve every blocked release gate" in result["errors"]
