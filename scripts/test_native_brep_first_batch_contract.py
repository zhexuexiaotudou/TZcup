#!/usr/bin/env python3
"""Regression tests for the static first native B-rep design-input contract."""

from __future__ import annotations

import importlib.util
import json
import tempfile
import unittest
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
CONTRACT_PATH = REPOSITORY_ROOT / "config" / "high_fidelity_vehicle" / "native_brep_first_batch_contract.json"
SCHEMA_PATH = REPOSITORY_ROOT / "config" / "high_fidelity_vehicle" / "native_brep_first_batch_contract.schema.json"
VALIDATOR_PATH = REPOSITORY_ROOT / "scripts" / "validate_native_brep_first_batch_contract.py"

SPEC = importlib.util.spec_from_file_location("validate_native_brep_first_batch_contract", VALIDATOR_PATH)
assert SPEC and SPEC.loader
VALIDATOR = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(VALIDATOR)


class NativeBrepFirstBatchContractTests(unittest.TestCase):
    def _contract(self) -> dict:
        return json.loads(CONTRACT_PATH.read_text(encoding="utf-8"))

    def test_current_contract_is_valid_and_static_only(self) -> None:
        report = VALIDATOR.validate(REPOSITORY_ROOT, CONTRACT_PATH, SCHEMA_PATH)
        self.assertTrue(report["valid"], report["errors"])
        self.assertEqual("design_input_pending_native_export", report["summary"]["status"])
        self.assertEqual(4, report["summary"]["first_batch_component_count"])
        self.assertEqual(0, report["summary"]["native_or_step_artifacts_created"])
        self.assertTrue(report["summary"]["static_only"])

    def test_exact_first_batch_and_non_released_statuses(self) -> None:
        contract = self._contract()
        self.assertEqual(
            {"arm_pedestal_adapter", "sensor_tower", "cleaning_head_brackets", "storage_frame"},
            {item["id"] for item in contract["items"]},
        )
        self.assertTrue(all(item["status"] == "design_input_pending_native_export" for item in contract["items"]))
        self.assertIn("mesh_to_step_conversion", contract["prohibited_methods"])
        self.assertIn("placeholder_fcstd_or_step_artifact", contract["prohibited_methods"])
        self.assertTrue(all(item["planned_native_export"]["must_not_exist_yet"] for item in contract["items"]))

    def test_status_or_fake_output_reference_fails_closed(self) -> None:
        contract = self._contract()
        with tempfile.TemporaryDirectory() as temporary_directory:
            temporary = Path(temporary_directory) / "contract.json"
            contract["items"][0]["status"] = "native_exported"
            temporary.write_text(json.dumps(contract), encoding="utf-8")
            report = VALIDATOR.validate(REPOSITORY_ROOT, temporary, SCHEMA_PATH)
            self.assertFalse(report["valid"])
            self.assertTrue(any("status" in error for error in report["errors"]))

            contract = self._contract()
            contract["items"][0]["planned_native_export"]["native_source_path"] = "README.md"
            temporary.write_text(json.dumps(contract), encoding="utf-8")
            report = VALIDATOR.validate(REPOSITORY_ROOT, temporary, SCHEMA_PATH)
            self.assertFalse(report["valid"])
            self.assertTrue(any("native_source_path" in error for error in report["errors"]))


if __name__ == "__main__":
    unittest.main()
