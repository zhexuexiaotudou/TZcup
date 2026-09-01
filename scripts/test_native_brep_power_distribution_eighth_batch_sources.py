#!/usr/bin/env python3
"""Low-memory AST/contract/hash checks for the eighth-batch PDU B-rep source."""

from __future__ import annotations

import ast
import hashlib
import importlib.util
import json
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SOURCE_PATH = ROOT / "starter_ws/src/sanitation_vehicle_description/cad/native_brep/formal_vehicle/native_brep_power_distribution_eighth_batch.py"
CONTRACT_PATH = ROOT / "config/high_fidelity_vehicle/native_brep_power_distribution_eighth_batch_contract.json"
MANIFEST_PATH = ROOT / "starter_ws/src/sanitation_vehicle_description/cad/native_brep/formal_vehicle/native_brep_power_distribution_eighth_batch_source_manifest.json"
SPEC = importlib.util.spec_from_file_location("native_brep_power_distribution_eighth_batch", SOURCE_PATH)
assert SPEC and SPEC.loader
SOURCE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = SOURCE
SPEC.loader.exec_module(SOURCE)


class NativeBrepPowerDistributionEighthBatchTests(unittest.TestCase):
    def setUp(self) -> None:
        self.contract = json.loads(CONTRACT_PATH.read_text(encoding="utf-8"))

    def test_exact_single_part_contract_is_fail_closed(self) -> None:
        self.assertEqual(SOURCE.PART_ID, self.contract["part_id"])
        self.assertEqual("generated/platform/power_distribution_box.stl", self.contract["source_mesh"])
        self.assertEqual(SOURCE.DESIGN_INPUT_STATUS, self.contract["status"])
        self.assertTrue(self.contract["pending_release_inputs"])
        self.assertTrue(self.contract["planned_native_export"]["must_not_exist_yet"])
        self.assertFalse((ROOT / self.contract["planned_native_export"]["optional_future_fcstd_path"]).exists())
        self.assertFalse((ROOT / self.contract["planned_native_export"]["step_path"]).exists())
        self.assertIn("creepage_clearance_touch_protection_grounding_and_bonding", self.contract["pending_release_inputs"])
        self.assertIn("rated_current_voltage_protection_coordination_and_short_circuit_analysis", self.contract["pending_release_inputs"])

    def test_ast_requires_real_serviceable_assembly_features_without_mesh_import(self) -> None:
        text = SOURCE_PATH.read_text(encoding="utf-8")
        tree = ast.parse(text, filename=str(SOURCE_PATH))
        functions = {node.name for node in ast.walk(tree) if isinstance(node, ast.FunctionDef)}
        self.assertTrue({"_enclosure_shell", "_removable_cover", "_din_rail", "_interface_blocks", "_cable_entry_bosses", "build_power_distribution_box", "build_design_input_assembly"} <= functions)
        self.assertNotIn("\nimport cadquery", text)
        self.assertNotIn("importMesh", text)
        self.assertNotIn("importers.importStep", text)
        self.assertNotIn(".stl", text.lower())
        for feature in (".cut(", ".fillet(", ".chamfer(", "cq.Assembly", "polyline(profile)"):
            self.assertIn(feature, text)
        self.assertIn(".extrude(length / 2.0, both=True)", text)

    def test_manifest_binds_exact_source_and_contract(self) -> None:
        manifest = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
        self.assertEqual(SOURCE.DESIGN_INPUT_STATUS, manifest["status"])
        entries = [*manifest["source_files"], *manifest["design_inputs"]]
        self.assertEqual(2, len(entries))
        for entry in entries:
            self.assertEqual(entry["sha256"], hashlib.sha256((ROOT / entry["path"]).read_bytes()).hexdigest())

    def test_export_is_blocked_before_kernel_import(self) -> None:
        with self.assertRaisesRegex(SOURCE.ExportBlocked, "design-input contract forbids native export"):
            SOURCE.validate_release_authorization(self.contract)
        self.assertFalse(SOURCE.summary(self.contract)["cadquery_imported"])


if __name__ == "__main__":
    unittest.main()
