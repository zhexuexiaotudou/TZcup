#!/usr/bin/env python3
"""Static-only tests for the storage/service third-batch CadQuery package.

No test imports CadQuery, starts a CAD kernel, writes an export, or starts
WSL, Gazebo, Docker, or a data-collection process.
"""

from __future__ import annotations

import hashlib
import importlib.util
import json
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SOURCE_PATH = ROOT / "starter_ws/src/sanitation_vehicle_description/cad/native_brep/formal_vehicle/native_brep_storage_service_third_batch.py"
CONTRACT_PATH = ROOT / "config/high_fidelity_vehicle/native_brep_storage_service_third_batch_contract.json"
MANIFEST_PATH = ROOT / "starter_ws/src/sanitation_vehicle_description/cad/native_brep/formal_vehicle/native_brep_storage_service_third_batch_source_manifest.json"
SPEC = importlib.util.spec_from_file_location("native_brep_storage_service_third_batch", SOURCE_PATH)
assert SPEC and SPEC.loader
SOURCE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = SOURCE
SPEC.loader.exec_module(SOURCE)


class NativeBrepStorageServiceThirdBatchTests(unittest.TestCase):
    def setUp(self) -> None:
        self.contract = json.loads(CONTRACT_PATH.read_text(encoding="utf-8"))

    def test_complete_pending_component_set_and_contract_boundary(self) -> None:
        expected = {
            "dry_bin_shell_lid_ribs",
            "wastewater_lid_vent_inlet",
            "dry_bin_latch_and_toggle_triplet",
            "level_sensor_and_probe_mounts",
            "wastewater_drain_service_train",
            "charge_port_interface",
        }
        self.assertEqual(expected, set(SOURCE.COMPONENT_IDS))
        self.assertEqual(expected, {item["id"] for item in self.contract["items"]})
        self.assertEqual(SOURCE.DESIGN_INPUT_STATUS, self.contract["status"])
        self.assertIn("mesh_to_step_conversion", self.contract["prohibited_methods"])
        self.assertIn("placeholder_fcstd_or_step_artifact", self.contract["prohibited_methods"])
        for item in self.contract["items"]:
            self.assertEqual(SOURCE.DESIGN_INPUT_STATUS, item["status"])
            self.assertTrue(item["pending_manufacturing_inputs"])
            planned = item["planned_native_export"]
            self.assertTrue(planned["must_not_exist_yet"])
            self.assertTrue(planned["export_preconditions"])
            self.assertEqual(str(SOURCE_PATH.relative_to(ROOT)).replace("\\", "/"), planned["authoritative_editable_source_path"])
            self.assertTrue(planned["optional_future_fcstd_path"].endswith(".FCStd"))
            self.assertTrue(planned["step_path"].endswith(".step"))
            self.assertEqual("starter_ws/src/sanitation_vehicle_description/cad/step/formal_vehicle/storage_service_third_batch.step", planned["assembly_step_path"])

    def test_source_is_parametric_lazy_and_never_mesh_backed(self) -> None:
        contents = SOURCE_PATH.read_text(encoding="utf-8")
        compile(contents, str(SOURCE_PATH), "exec")
        self.assertIn("def require_cadquery()", contents)
        self.assertNotIn("\nimport cadquery", contents)
        self.assertNotIn("importers.importStep", contents)
        self.assertNotIn("importMesh", contents)
        self.assertNotIn(".stl", contents.lower())
        for builder in (
            "_build_dry_bin_shell_lid_ribs",
            "_build_wastewater_lid_vent_inlet",
            "_build_dry_bin_latch_and_toggle_triplet",
            "_build_level_sensor_and_probe_mounts",
            "_build_wastewater_drain_service_train",
            "_build_charge_port_interface",
        ):
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

    def test_wet_pan_baffle_is_dependency_not_duplicate_and_interfaces_remain_pending(self) -> None:
        wet = SOURCE.component_by_id(self.contract, "wastewater_lid_vent_inlet")
        self.assertIn("wastewater_tank_pan_baffles", " ".join(wet["dependencies"]))
        self.assertNotIn("floor_size_m", SOURCE.geometry(wet))
        all_pending = " ".join(" ".join(item["pending_manufacturing_inputs"]) for item in self.contract["items"])
        for term in ("holes", "threads", "seal", "material", "tolerances"):
            self.assertIn(term, all_pending)

    def test_export_is_blocked_before_cadquery_import(self) -> None:
        with self.assertRaisesRegex(SOURCE.ExportBlocked, "design-input contract forbids native export"):
            SOURCE.validate_release_authorization(self.contract)


if __name__ == "__main__":
    unittest.main()
