"""Regression tests for the static CadQuery serial-export contract audit."""

from __future__ import annotations

import copy
import importlib.util
import json
import sys
import tempfile
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "validate_native_cadquery_serial_export_contract.py"
SPEC = importlib.util.spec_from_file_location(
    "validate_native_cadquery_serial_export_contract", SCRIPT
)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
sys.path.insert(0, str(ROOT / "scripts"))
SPEC.loader.exec_module(MODULE)


def test_current_contract_is_complete_but_native_export_blocked() -> None:
    report = MODULE.validate(ROOT)
    assert report["status"] == "STATIC_SERIAL_EXPORT_CONTRACT_VALID_NATIVE_EXPORT_BLOCKED"
    assert report["contract_structurally_valid"] is True
    assert report["source_batch_count"] == 8
    assert report["provenance_only_batch_count"] == 4
    assert report["component_addressable_batch_count"] == 4
    assert report["component_addressable_count"] == 105
    assert report["component_ids_unique"] is True
    assert report["pending_batch_contract_count"] == 8
    assert report["source_digest_bindings_valid"] is True
    assert report["minimum_free_physical_memory_mib"] == 4096
    assert report["execution"] == "strictly_serial"
    assert report["formal_export_ready"] is False
    assert report["native_cad_delivery_accepted"] is False
    assert report["cadquery_imported"] is False
    assert report["source_modules_loaded"] is False
    assert report["errors"] == []


def test_preview_or_batch_count_drift_invalidates_contract() -> None:
    contract = json.loads(
        (ROOT / MODULE.CONTRACT_RELATIVE).read_text(encoding="utf-8")
    )
    altered = copy.deepcopy(contract)
    altered["preview"]["implemented"] = True
    altered["source_batches"][4]["component_export_count"] = 46
    report = MODULE.audit_serial_export_contract(ROOT, altered)
    assert report["contract_structurally_valid"] is False
    assert report["status"] == "STATIC_SERIAL_EXPORT_CONTRACT_INVALID"
    assert "preview_boundary_not_fail_closed" in report["errors"]
    assert any("component_export_count_mismatch" in error for error in report["errors"])


def test_missing_contract_is_a_structured_failure() -> None:
    with tempfile.TemporaryDirectory() as temporary:
        report = MODULE.validate(ROOT, Path(temporary) / "missing.json")
    assert report["contract_structurally_valid"] is False
    assert report["status"] == "STATIC_SERIAL_EXPORT_CONTRACT_INVALID"
    assert report["cadquery_imported"] is False
    assert report["source_modules_loaded"] is False
