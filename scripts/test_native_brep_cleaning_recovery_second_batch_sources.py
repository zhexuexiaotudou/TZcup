#!/usr/bin/env python3
"""Static-only checks for project-authored cleaning/recovery CadQuery source."""

from __future__ import annotations

import importlib.util
import hashlib
import json
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SOURCE_PATH = ROOT / "starter_ws/src/sanitation_vehicle_description/cad/native_brep/formal_vehicle/native_brep_cleaning_recovery_second_batch.py"
CONTRACT_PATH = ROOT / "config/high_fidelity_vehicle/native_brep_cleaning_recovery_second_batch_contract.json"
MANIFEST_PATH = ROOT / "starter_ws/src/sanitation_vehicle_description/cad/native_brep/formal_vehicle/native_brep_cleaning_recovery_second_batch_source_manifest.json"
SPEC = importlib.util.spec_from_file_location("native_brep_cleaning_recovery_second_batch", SOURCE_PATH)
assert SPEC and SPEC.loader
SOURCE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = SOURCE
SPEC.loader.exec_module(SOURCE)


class NativeBrepCleaningRecoverySecondBatchTests(unittest.TestCase):
    def setUp(self) -> None:
        self.contract = json.loads(CONTRACT_PATH.read_text(encoding="utf-8"))

    def test_contract_is_complete_pending_design_input(self) -> None:
        self.assertEqual(SOURCE.DESIGN_INPUT_STATUS, self.contract["status"])
        self.assertEqual(set(SOURCE.COMPONENT_IDS), {item["id"] for item in self.contract["items"]})
        self.assertIn("mesh_to_step_conversion", self.contract["prohibited_methods"])
        for item in self.contract["items"]:
            self.assertEqual(SOURCE.DESIGN_INPUT_STATUS, item["status"])
            self.assertTrue(item["pending_manufacturing_inputs"])
            self.assertTrue(item["planned_native_export"]["must_not_exist_yet"])
            self.assertTrue(item["planned_native_export"]["export_preconditions"])
            self.assertEqual(str(SOURCE_PATH.relative_to(ROOT)).replace("\\", "/"), item["planned_native_export"]["authoritative_editable_source_path"])
            self.assertTrue(item["planned_native_export"]["optional_future_fcstd_path"].endswith(".FCStd"))
            self.assertTrue(item["planned_native_export"]["step_path"].endswith(".step"))
            self.assertEqual("starter_ws/src/sanitation_vehicle_description/cad/step/formal_vehicle/cleaning_recovery_second_batch.step", item["planned_native_export"]["assembly_step_path"])

    def test_source_is_real_parametric_and_never_imports_meshes(self) -> None:
        contents = SOURCE_PATH.read_text(encoding="utf-8")
        compile(contents, str(SOURCE_PATH), "exec")
        self.assertIn("def require_cadquery()", contents)
        self.assertNotIn("\nimport cadquery", contents)
        self.assertNotIn("importers.importStep", contents)
        self.assertNotIn("importMesh", contents)
        self.assertNotIn(".stl", contents.lower())
        for builder in ("_build_side_brush_drive", "_build_central_roller", "_build_squeegee_backing", "_build_suction_nozzle", "_build_quick_coupling", "_build_dry_deposit_gate_chute", "_build_wastewater_tank_pan_baffles"):
            self.assertIn(f"def {builder}", contents)
        self.assertIn("def build_design_input_assembly", contents)

    def test_source_manifest_binds_exact_editable_source_and_contract_hashes(self) -> None:
        manifest = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
        self.assertEqual(SOURCE.DESIGN_INPUT_STATUS, manifest["status"])
        self.assertEqual("sha256", manifest["verification"]["algorithm"])
        entries = [*manifest["source_files"], *manifest["design_inputs"]]
        self.assertEqual(2, len(entries))
        expected_paths = {
            str(SOURCE_PATH.relative_to(ROOT)).replace("\\", "/"),
            str(CONTRACT_PATH.relative_to(ROOT)).replace("\\", "/"),
        }
        self.assertEqual(expected_paths, {entry["path"] for entry in entries})
        for entry in entries:
            observed = hashlib.sha256((ROOT / entry["path"]).read_bytes()).hexdigest()
            self.assertEqual(entry["sha256"], observed, entry["path"])

    def test_runtime_bristles_are_not_claimed_as_a_manufacturing_part(self) -> None:
        roller = SOURCE.component_by_id(self.contract, "central_roller")
        model = SOURCE.geometry(roller)["runtime_bristle_model"]
        self.assertEqual("flexible_runtime_model_not_manufacturing_brep", model["status"])

    def test_export_is_blocked_before_cadquery_import(self) -> None:
        with self.assertRaisesRegex(SOURCE.ExportBlocked, "design-input contract forbids native export"):
            SOURCE.validate_release_authorization(self.contract)


if __name__ == "__main__":
    unittest.main()
