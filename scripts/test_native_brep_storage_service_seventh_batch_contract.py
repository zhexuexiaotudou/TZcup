from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import pytest

from validate_native_brep_storage_service_seventh_batch_contract import (
    CONTRACT,
    ROOT,
    validate,
)


def _contract() -> dict:
    return json.loads((ROOT / CONTRACT).read_text(encoding="utf-8"))


def test_current_mapping_is_exact_and_fail_closed() -> None:
    report = validate(ROOT)
    assert report["valid"], report["errors"]
    assert report["summary"]["storage_part_count"] == 24
    assert report["summary"]["service_part_count"] == 10
    assert report["summary"]["total_exact_source_mesh_mappings"] == 34
    assert report["summary"]["cadquery_executed"] is False
    assert report["summary"]["runtime_accepted"] is False


def test_missing_or_duplicated_source_mapping_fails_closed(tmp_path: Path) -> None:
    payload = _contract()
    payload["part_mappings"].pop()
    broken = tmp_path / "contract.json"
    broken.write_text(json.dumps(payload), encoding="utf-8")
    report = validate(ROOT, contract_path=broken)
    assert not report["valid"]
    assert any("exactly 24 storage" in error or "mapping IDs" in error for error in report["errors"])

    payload = _contract()
    payload["part_mappings"][1]["source_mesh"] = payload["part_mappings"][0]["source_mesh"]
    broken.write_text(json.dumps(payload), encoding="utf-8")
    report = validate(ROOT, contract_path=broken)
    assert not report["valid"]
    assert any("source_mesh values must be unique" in error for error in report["errors"])


def test_no_mapping_can_promote_native_or_runtime_acceptance(tmp_path: Path) -> None:
    payload = _contract()
    payload["acceptance"]["runtime_accepted"] = True
    broken = tmp_path / "contract.json"
    broken.write_text(json.dumps(payload), encoding="utf-8")
    report = validate(ROOT, contract_path=broken)
    assert not report["valid"]
    assert any("acceptance flags" in error for error in report["errors"])


def test_cli_static_validation_is_machine_readable() -> None:
    completed = subprocess.run(
        [sys.executable, str(ROOT / "scripts/validate_native_brep_storage_service_seventh_batch_contract.py")],
        check=True,
        capture_output=True,
        text=True,
    )
    report = json.loads(completed.stdout)
    assert report["valid"] is True
    assert report["boundary"].startswith("Static")

