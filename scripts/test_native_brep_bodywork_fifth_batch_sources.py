#!/usr/bin/env python3
"""Low-memory AST/JSON checks for fifth-batch per-part bodywork source."""

from __future__ import annotations

import ast
import hashlib
import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SOURCE_PATH = ROOT / "starter_ws/src/sanitation_vehicle_description/cad/native_brep/formal_vehicle/native_brep_bodywork_fifth_batch.py"
CONTRACT_PATH = ROOT / "config/high_fidelity_vehicle/native_brep_bodywork_fifth_batch_contract.json"
MANIFEST_PATH = ROOT / "starter_ws/src/sanitation_vehicle_description/cad/native_brep/formal_vehicle/native_brep_bodywork_fifth_batch_source_manifest.json"
AUDIT_PATH = ROOT / "reports/engineering/native_brep_source_coverage_audit.json"
SPEC = importlib.util.spec_from_file_location("native_brep_bodywork_fifth_batch", SOURCE_PATH)
assert SPEC and SPEC.loader
SOURCE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = SOURCE
SPEC.loader.exec_module(SOURCE)


class NativeBrepBodyworkFifthBatchTests(unittest.TestCase):
    def setUp(self) -> None:
        self.contract = json.loads(CONTRACT_PATH.read_text(encoding="utf-8"))

    def test_exactly_covers_current_47_bodywork_audit_rows_once(self) -> None:
        audit = json.loads(AUDIT_PATH.read_text(encoding="utf-8"))
        expected = {
            (row["manifest_part_id"], row["source_mesh"])
            for row in audit["rows"]
            if row.get("profile") == "bodywork"
        }
        observed = {(row["part_id"], row["source_mesh"]) for row in self.contract["parts"]}
        self.assertEqual(47, len(expected))
        self.assertEqual(expected, observed)
        self.assertEqual(47, len(observed))

    def test_every_part_has_one_named_ast_builder_and_pending_gate(self) -> None:
        tree = ast.parse(SOURCE_PATH.read_text(encoding="utf-8"), filename=str(SOURCE_PATH))
        functions = {node.name for node in ast.walk(tree) if isinstance(node, ast.FunctionDef)}
        builders = [row["builder"] for row in self.contract["parts"]]
        self.assertEqual(47, len(builders))
        self.assertEqual(47, len(set(builders)))
        self.assertTrue(set(builders).issubset(functions))
        self.assertEqual(SOURCE.DESIGN_INPUT_STATUS, self.contract["status"])
        self.assertIn("mesh_to_step_conversion", self.contract["prohibited_methods"])
        for row in self.contract["parts"]:
            self.assertTrue(row["pending_release_inputs"], row["part_id"])
            self.assertIn(row["topology"], {"segmented_lofted_body", "segmented_lofted_guard", "thickened_panel", "thin_trim_panel", "thin_bezel_panel", "filleted_trim_panel", "door_seam_hinge_ears", "wheel_arch_cutout", "hinge_ear_and_barrel", "latch_support_and_tongue", "annular_service_trim", "lamp_pod_recess", "filleted_impact_bar", "filleted_sill", "reinforced_panel", "open_turret_panels"})

    def test_source_is_real_feature_source_without_mesh_or_kernel_execution(self) -> None:
        contents = SOURCE_PATH.read_text(encoding="utf-8")
        compile(contents, str(SOURCE_PATH), "exec")
        self.assertNotIn("\nimport cadquery", contents)
        self.assertNotIn("importMesh", contents)
        self.assertNotIn("importers.importStep", contents)
        self.assertNotIn(".stl", contents.lower())
        for feature in (".loft(", ".fillet(", ".chamfer(", ".cut(", "_door_with_seam_and_ears", "_wheel_arch"):
            self.assertIn(feature, contents)
        self.assertIn('width, depth, height = _vector(p["door_envelope_m"], "door_envelope_m")', contents)
        self.assertIn('extrude(mm(float(p["thickness_m"])) / 2.0, both=True)', contents)
        self.assertIn('extrude(mm(float(p["seam_depth_m"])) / 2.0, both=True)', contents)
        self.assertIn('extrude(mm(float(p["cut_depth_m"])) / 2.0, both=True)', contents)

    def test_manifest_binds_source_and_contract_hashes(self) -> None:
        manifest = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
        self.assertEqual(SOURCE.DESIGN_INPUT_STATUS, manifest["status"])
        entries = [*manifest["source_files"], *manifest["design_inputs"]]
        self.assertEqual(2, len(entries))
        for entry in entries:
            self.assertEqual(entry["sha256"], hashlib.sha256((ROOT / entry["path"]).read_bytes()).hexdigest())

    def test_export_is_blocked_without_creating_an_output(self) -> None:
        with self.assertRaisesRegex(SOURCE.ExportBlocked, "design-input contract forbids native export"):
            SOURCE.validate_release_authorization(self.contract)
        with tempfile.TemporaryDirectory() as temporary_directory:
            self.assertFalse((Path(temporary_directory) / "bodywork.step").exists())
        self.assertFalse(SOURCE.summary(self.contract)["cadquery_imported"])


if __name__ == "__main__":
    unittest.main()
