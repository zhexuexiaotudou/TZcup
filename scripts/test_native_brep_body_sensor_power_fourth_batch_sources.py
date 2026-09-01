#!/usr/bin/env python3
"""Static-only regression checks for fourth-batch CadQuery B-rep inputs."""

from __future__ import annotations

import hashlib
import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SOURCE_PATH = ROOT / "starter_ws/src/sanitation_vehicle_description/cad/native_brep/formal_vehicle/native_brep_body_sensor_power_fourth_batch.py"
CONTRACT_PATH = ROOT / "config/high_fidelity_vehicle/native_brep_body_sensor_power_fourth_batch_contract.json"
MANIFEST_PATH = ROOT / "starter_ws/src/sanitation_vehicle_description/cad/native_brep/formal_vehicle/native_brep_body_sensor_power_fourth_batch_source_manifest.json"
SPEC = importlib.util.spec_from_file_location("native_brep_body_sensor_power_fourth_batch", SOURCE_PATH)
assert SPEC and SPEC.loader
SOURCE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = SOURCE
SPEC.loader.exec_module(SOURCE)


class NativeBrepBodySensorPowerFourthBatchTests(unittest.TestCase):
    def setUp(self) -> None:
        self.contract = json.loads(CONTRACT_PATH.read_text(encoding="utf-8"))

    def test_complete_pending_scope_and_explicit_make_buy_boundary(self) -> None:
        self.assertEqual(SOURCE.DESIGN_INPUT_STATUS, self.contract["status"])
        self.assertEqual(set(SOURCE.COMPONENT_IDS), {item["id"] for item in self.contract["items"]})
        self.assertIn("mesh_to_step_conversion", self.contract["prohibited_methods"])
        self.assertIn("reverse_engineering_vendor_sensor_or_electronics_bodies_from_visual_meshes", self.contract["prohibited_methods"])
        required_scope = {
            "bodywork_access_set": ("front nose", "four physical service-door panels"),
            "sensor_mast_and_installation_brackets": ("UTM", "MID360", "GNSS", "IMU"),
            "compute_and_control_cabinet_mounting": ("S100P", "UR5e"),
            "power_distribution_mounting_enclosures": ("DC-DC", "safety-relay"),
        }
        for item in self.contract["items"]:
            self.assertEqual(SOURCE.DESIGN_INPUT_STATUS, item["status"])
            self.assertTrue(item["pending_manufacturing_inputs"])
            self.assertTrue(item["planned_native_export"]["must_not_exist_yet"])
            self.assertTrue(item["planned_native_export"]["export_preconditions"])
            self.assertEqual(str(SOURCE_PATH.relative_to(ROOT)).replace("\\", "/"), item["planned_native_export"]["authoritative_editable_source_path"])
            self.assertFalse((ROOT / item["planned_native_export"]["optional_future_fcstd_path"]).exists())
            self.assertFalse((ROOT / item["planned_native_export"]["step_path"]).exists())
            for phrase in required_scope[item["id"]]:
                self.assertIn(phrase, item["scope"])

    def test_source_is_parametric_lazy_and_never_uses_meshes(self) -> None:
        contents = SOURCE_PATH.read_text(encoding="utf-8")
        compile(contents, str(SOURCE_PATH), "exec")
        self.assertIn("def require_cadquery()", contents)
        self.assertNotIn("\nimport cadquery", contents)
        self.assertNotIn("importers.importStep", contents)
        self.assertNotIn("importMesh", contents)
        self.assertNotIn(".stl", contents.lower())
        for builder in (
            "_build_bodywork_access_set",
            "_build_sensor_mast_and_installation_brackets",
            "_build_compute_and_control_cabinet_mounting",
            "_build_power_distribution_mounting_enclosures",
        ):
            self.assertIn(f"def {builder}", contents)

    def test_manifest_hashes_bind_exact_source_and_contract(self) -> None:
        manifest = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
        self.assertEqual(SOURCE.DESIGN_INPUT_STATUS, manifest["status"])
        entries = [*manifest["source_files"], *manifest["design_inputs"]]
        self.assertEqual(2, len(entries))
        self.assertEqual("sha256", manifest["verification"]["algorithm"])
        self.assertEqual(
            {str(SOURCE_PATH.relative_to(ROOT)).replace("\\", "/"), str(CONTRACT_PATH.relative_to(ROOT)).replace("\\", "/")},
            {entry["path"] for entry in entries},
        )
        for entry in entries:
            self.assertEqual(entry["sha256"], hashlib.sha256((ROOT / entry["path"]).read_bytes()).hexdigest())

    def test_export_fails_closed_before_cadquery_import(self) -> None:
        with self.assertRaisesRegex(SOURCE.ExportBlocked, "design-input contract forbids native export"):
            SOURCE.validate_release_authorization(self.contract)
        with tempfile.TemporaryDirectory() as temporary_directory:
            output = Path(temporary_directory) / "must-not-be-created"
            with self.assertRaisesRegex(SOURCE.ExportBlocked, "design-input contract forbids native export"):
                SOURCE.export_released_components(ROOT, output)
            self.assertFalse(output.exists())
        self.assertFalse(SOURCE.summary(self.contract)["cadquery_imported"])

    def test_functional_coverage_has_no_vendor_body_rebuild(self) -> None:
        expected_features = {
            "bodywork_access_set": {"front_nose_lower_packaging", "front_left_fender_envelope", "rear_right_fender_envelope", "power_door_panel", "rear_dry_door_panel"},
            "sensor_mast_and_installation_brackets": {"mast_left_column", "utm_shelf", "front_rgbd_upper_bezel", "mid360_isolator_plate", "gnss_boom", "imu_isolation_tray"},
            "compute_and_control_cabinet_mounting": {"control_box_lower_rail_pair", "s100_roof_mount_plate", "s100_protective_enclosure"},
            "power_distribution_mounting_enclosures": {"pdu_enclosure_body", "dc_dc_support_enclosure", "safety_relay_support_enclosure"},
        }
        for item in self.contract["items"]:
            features = {entry["feature"] for key in ("box_features", "cylinder_features") for entry in item["geometry"].get(key, [])}
            self.assertTrue(expected_features[item["id"]].issubset(features))
        self.assertIn("Vendor sensor, compute-board, power-converter and relay bodies are expressly not rebuilt.", self.contract["claim_boundary"])


if __name__ == "__main__":
    unittest.main()
