#!/usr/bin/env python3
"""Low-memory static tests for the CadQuery first-batch source module.

No test imports CadQuery, starts FreeCAD/WSL/Docker/Gazebo, or writes CAD/STEP
artifacts.  They exercise the source's JSON binding and its pre-kernel release
gate only.
"""

from __future__ import annotations

import importlib.util
import hashlib
import json
import sys
import tempfile
import unittest
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
SOURCE_PATH = (
    REPOSITORY_ROOT
    / "starter_ws/src/sanitation_vehicle_description/cad/native_brep/formal_vehicle/native_brep_first_batch.py"
)
CONTRACT_PATH = REPOSITORY_ROOT / "config/high_fidelity_vehicle/native_brep_first_batch_contract.json"
SOURCE_MANIFEST_PATH = (
    REPOSITORY_ROOT
    / "starter_ws/src/sanitation_vehicle_description/cad/native_brep/formal_vehicle/native_brep_first_batch_source_manifest.json"
)

SPEC = importlib.util.spec_from_file_location("native_brep_first_batch", SOURCE_PATH)
assert SPEC and SPEC.loader
SOURCE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = SOURCE
SPEC.loader.exec_module(SOURCE)


class NativeBrepFirstBatchSourceTests(unittest.TestCase):
    def setUp(self) -> None:
        self.contract = SOURCE.load_contract(REPOSITORY_ROOT)

    def test_source_compiles_and_keeps_cadquery_lazy(self) -> None:
        contents = SOURCE_PATH.read_text(encoding="utf-8")
        compile(contents, str(SOURCE_PATH), "exec")
        self.assertIn("def require_cadquery()", contents)
        self.assertNotIn("\nimport cadquery", contents)
        self.assertNotIn("importers.importStep", contents)
        self.assertNotIn("importMesh", contents)

    def test_source_manifest_lists_exact_editable_source_and_contract_hashes(self) -> None:
        manifest = json.loads(SOURCE_MANIFEST_PATH.read_text(encoding="utf-8"))
        self.assertEqual("design_input_pending_native_export", manifest["status"])
        self.assertEqual("sha256", manifest["verification"]["algorithm"])
        listed = [*manifest["source_files"], *manifest["design_inputs"]]
        self.assertEqual(2, len(listed))
        for entry in listed:
            path = REPOSITORY_ROOT / entry["path"]
            self.assertTrue(path.is_file(), entry["path"])
            observed = hashlib.sha256(path.read_bytes()).hexdigest()
            self.assertEqual(entry["sha256"], observed, entry["path"])

    def test_all_four_contract_items_have_real_source_builders_and_datums(self) -> None:
        self.assertEqual(set(SOURCE.COMPONENT_IDS), {item["id"] for item in self.contract["items"]})
        for component_id in SOURCE.COMPONENT_IDS:
            component = SOURCE.component_by_id(self.contract, component_id)
            self.assertEqual("design_input_pending_native_export", component["status"])
            locations = SOURCE._datum_locations(component)
            expected = {"arm_pedestal_adapter": 6, "sensor_tower": 4, "cleaning_head_brackets": 4, "storage_frame": 6}
            self.assertEqual(expected[component_id], len(locations))
        contents = SOURCE_PATH.read_text(encoding="utf-8")
        for builder in (
            "_build_arm_pedestal_adapter",
            "_build_sensor_tower",
            "_build_cleaning_head_brackets",
            "_build_storage_frame",
        ):
            self.assertIn(f"def {builder}", contents)
        self.assertIn("_triangular_gusset", contents)
        self.assertIn("build_released_shape", contents)

    def test_current_contract_fails_closed_before_cadquery_import(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            evidence_path = Path(temporary_directory) / "release.json"
            evidence_path.write_text("{}", encoding="utf-8")
            with self.assertRaisesRegex(SOURCE.ExportBlocked, "design-input contract forbids native export"):
                SOURCE.load_and_validate_release_evidence(self.contract, evidence_path, SOURCE.COMPONENT_IDS)

    def test_current_contract_cannot_create_an_output_directory(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            temporary = Path(temporary_directory)
            evidence_path = temporary / "release.json"
            output_directory = temporary / "must-not-exist"
            evidence_path.write_text("{}", encoding="utf-8")
            with self.assertRaisesRegex(SOURCE.ExportBlocked, "design-input contract forbids native export"):
                SOURCE.export_released_components_and_assembly(
                    REPOSITORY_ROOT, evidence_path, output_directory, SOURCE.COMPONENT_IDS
                )
            self.assertFalse(output_directory.exists())

    def test_release_evidence_requires_every_contract_gate_and_controlled_holes(self) -> None:
        released_contract = json.loads(json.dumps(self.contract))
        released_contract["status"] = SOURCE.RELEASED_STATUS
        components: dict[str, dict] = {}
        for component_id in SOURCE.COMPONENT_IDS:
            component = SOURCE.component_by_id(released_contract, component_id)
            record = {gate: True for gate in component["planned_native_export"]["export_preconditions"]}
            record["released_holes"] = [
                {
                    "feature": "controlled_pattern",
                    "datum_xy_m": [x, y],
                    "start_m": [x, y, -0.1],
                    "axis": "z",
                    "diameter_m": 0.006,
                    "depth_m": 0.2,
                }
                for x, y in SOURCE._datum_locations(component)
            ]
            components[component_id] = record
        evidence = {
            "schema_version": SOURCE.RELEASE_EVIDENCE_SCHEMA,
            "contract_document_id": released_contract["document_id"],
            "native_export_authorized": True,
            "components": components,
        }
        with tempfile.TemporaryDirectory() as temporary_directory:
            evidence_path = Path(temporary_directory) / "release.json"
            evidence_path.write_text(json.dumps(evidence), encoding="utf-8")
            parsed = SOURCE.load_and_validate_release_evidence(released_contract, evidence_path, SOURCE.COMPONENT_IDS)
            self.assertEqual({component_id: len(SOURCE._datum_locations(SOURCE.component_by_id(released_contract, component_id))) for component_id in SOURCE.COMPONENT_IDS}, {component_id: len(holes) for component_id, holes in parsed.items()})
            evidence["components"]["sensor_tower"].pop("native_parametric_feature_model_reviewed")
            evidence_path.write_text(json.dumps(evidence), encoding="utf-8")
            with self.assertRaisesRegex(SOURCE.ExportBlocked, "unclosed export preconditions"):
                SOURCE.load_and_validate_release_evidence(released_contract, evidence_path, SOURCE.COMPONENT_IDS)


if __name__ == "__main__":
    unittest.main()
