"""Regression tests for the independent second-batch static validator."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
VALIDATOR_PATH = ROOT / "scripts/validate_native_brep_cleaning_recovery_second_batch_contract.py"
CONTRACT_PATH = ROOT / "config/high_fidelity_vehicle/native_brep_cleaning_recovery_second_batch_contract.json"
MANIFEST_PATH = ROOT / "starter_ws/src/sanitation_vehicle_description/cad/native_brep/formal_vehicle/native_brep_cleaning_recovery_second_batch_source_manifest.json"
SPEC = importlib.util.spec_from_file_location("second_batch_validator", VALIDATOR_PATH)
assert SPEC and SPEC.loader
VALIDATOR = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(VALIDATOR)


def test_current_second_batch_is_source_bound_and_pending_native_export() -> None:
    report = VALIDATOR.validate(ROOT)

    assert report["valid"], report["errors"]
    assert report["summary"] == {
        "status": "design_input_pending_native_export",
        "second_batch_component_count": 7,
        "source_manifest_paths_exact": True,
        "source_hash_verified": True,
        "cadquery_import_lazy": True,
        "mesh_import_absent": True,
        "native_or_step_artifacts_created": 0,
        "static_only": True,
    }


def test_hash_drift_or_created_native_output_fails_closed(tmp_path: Path) -> None:
    manifest = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
    manifest["source_files"][0]["sha256"] = "0" * 64
    invalid_manifest = tmp_path / "manifest.json"
    invalid_manifest.write_text(json.dumps(manifest), encoding="utf-8")

    report = VALIDATOR.validate(ROOT, manifest_path=invalid_manifest)

    assert not report["valid"]
    assert not report["summary"]["source_hash_verified"]
    assert any("SHA-256 mismatch" in error for error in report["errors"])

    contract = json.loads(CONTRACT_PATH.read_text(encoding="utf-8"))
    contract["items"][0]["planned_native_export"]["step_path"] = "README.md"
    invalid_contract = tmp_path / "contract.json"
    invalid_contract.write_text(json.dumps(contract), encoding="utf-8")
    report = VALIDATOR.validate(ROOT, contract_path=invalid_contract)

    assert not report["valid"]
    assert any("step_path" in error for error in report["errors"])
