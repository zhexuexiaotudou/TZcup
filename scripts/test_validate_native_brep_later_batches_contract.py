"""Regression tests for the third/fourth pending source-bound B-rep validators."""

from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))


def _load(name: str):
    path = SCRIPTS / f"{name}.py"
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


THIRD = _load("validate_native_brep_storage_service_third_batch_contract")
FOURTH = _load("validate_native_brep_body_sensor_power_fourth_batch_contract")


def test_current_later_batches_are_source_bound_and_pending_native_export() -> None:
    for validator, count_key, count in (
        (THIRD, "third_batch_component_count", 6),
        (FOURTH, "fourth_batch_component_count", 4),
    ):
        report = validator.validate(ROOT)
        assert report["valid"], report["errors"]
        assert report["summary"] == {
            "status": "design_input_pending_native_export",
            count_key: count,
            "source_manifest_paths_exact": True,
            "source_hash_verified": True,
            "cadquery_import_lazy": True,
            "mesh_import_absent": True,
            "native_or_step_artifacts_created": 0,
            "static_only": True,
        }


def test_later_batch_hash_or_future_output_drift_fails_closed(tmp_path: Path) -> None:
    manifest = json.loads((ROOT / THIRD.MANIFEST_RELATIVE_PATH).read_text(encoding="utf-8"))
    manifest["source_files"][0]["sha256"] = "0" * 64
    invalid_manifest = tmp_path / "third-manifest.json"
    invalid_manifest.write_text(json.dumps(manifest), encoding="utf-8")
    report = THIRD.validate(ROOT, manifest_path=invalid_manifest)
    assert not report["valid"]
    assert not report["summary"]["source_hash_verified"]
    assert any("SHA-256 mismatch" in error for error in report["errors"])

    contract = json.loads((ROOT / FOURTH.CONTRACT_RELATIVE_PATH).read_text(encoding="utf-8"))
    contract["items"][0]["planned_native_export"]["step_path"] = "README.md"
    invalid_contract = tmp_path / "fourth-contract.json"
    invalid_contract.write_text(json.dumps(contract), encoding="utf-8")
    report = FOURTH.validate(ROOT, contract_path=invalid_contract)
    assert not report["valid"]
    assert any("step_path" in error for error in report["errors"])
