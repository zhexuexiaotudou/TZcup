#!/usr/bin/env python3
"""Windows-only regression tests for the native CAD honesty preflight."""

from __future__ import annotations

import importlib.util
import hashlib
import json
import tempfile
import unittest
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
AUDIT_PATH = REPOSITORY_ROOT / "scripts" / "audit_native_cad_readiness.py"
SPEC = importlib.util.spec_from_file_location("audit_native_cad_readiness", AUDIT_PATH)
assert SPEC and SPEC.loader
AUDIT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(AUDIT)


class NativeCadReadinessTests(unittest.TestCase):
    def test_current_vehicle_assets_fail_closed_without_native_cad(self) -> None:
        report = AUDIT.build_report(REPOSITORY_ROOT)
        self.assertEqual("blocked", report["outcome"])
        self.assertFalse(report["native_editable_step_assembly_ready"])
        # The audit discovers every valid manifest-bound source.  Require this
        # batch set without asserting that unrelated concurrent additions have
        # not also been recognised.
        required_sources = {
            "starter_ws/src/sanitation_vehicle_description/cad/native_brep/formal_vehicle/native_brep_first_batch.py",
            "starter_ws/src/sanitation_vehicle_description/cad/native_brep/formal_vehicle/native_brep_cleaning_recovery_second_batch.py",
            "starter_ws/src/sanitation_vehicle_description/cad/native_brep/formal_vehicle/native_brep_storage_service_third_batch.py",
            "starter_ws/src/sanitation_vehicle_description/cad/native_brep/formal_vehicle/native_brep_body_sensor_power_fourth_batch.py",
        }
        self.assertTrue(required_sources.issubset(report["inventory"]["cadquery_parametric_brep_sources"]))
        self.assertEqual(report["inventory"]["cadquery_parametric_brep_sources"], report["inventory"]["editable_native_brep_sources"])
        self.assertEqual([], report["inventory"]["step_artifacts"])
        self.assertGreater(report["inventory"]["mesh_asset_count"], 0)
        self.assertTrue(report["inventory"]["openscad_packaging_sources"])
        self.assertTrue(report["inventory"]["python_mesh_generators"])
        self.assertTrue(any(gap["code"] == "NATIVE_ASSEMBLY_MANIFEST_DRAFT_NOT_RELEASED" for gap in report["gaps"]))
        self.assertFalse(any(gap["code"] == "NO_NATIVE_ASSEMBLY_MANIFEST" for gap in report["gaps"]))
        self.assertEqual(
            {
                "path": "config/high_fidelity_vehicle/component_addressable_native_cad_assembly_manifest_draft.json",
                "valid": True,
                "status": "STATIC_COMPONENT_ADDRESSABLE_DRAFT_VALID_NATIVE_EXPORT_BLOCKED",
                "component_count": 105,
                "supplier_excluded_count": 21,
            },
            report["inventory"]["component_addressable_assembly_draft"],
        )
        self.assertTrue(any(gap["code"] == "NO_NATIVE_CAD_EXPORT_RECEIPT" for gap in report["gaps"]))

    def test_mesh_conversion_is_explicitly_disallowed(self) -> None:
        report = AUDIT.build_report(REPOSITORY_ROOT)
        prohibited = report["native_cad_definition"]["explicitly_not_accepted"]
        self.assertTrue(any("mesh-to-STEP" in item for item in prohibited))
        warning = next(gap for gap in report["gaps"] if gap["code"] == "LEGACY_STL_VISUAL_GENERATORS_PRESENT")
        self.assertEqual("warning", warning["severity"])

    def test_checked_in_preflight_tracks_static_native_inventory(self) -> None:
        """The retained report must not regress to the pre-native-BREP inventory."""

        checked_in = json.loads(
            (
                REPOSITORY_ROOT
                / "reports/engineering/native_cad_preflight.json"
            ).read_text(encoding="utf-8")
        )
        current = AUDIT.build_report(REPOSITORY_ROOT)
        for key in (
            "cadquery_parametric_brep_sources",
            "editable_native_brep_sources",
            "rejected_python_brep_candidates",
            "assembly_manifest",
            "component_addressable_assembly_draft",
        ):
            self.assertEqual(current["inventory"][key], checked_in["inventory"][key])
        self.assertEqual(
            [gap["code"] for gap in current["gaps"]],
            [gap["code"] for gap in checked_in["gaps"]],
        )
        self.assertEqual("blocked", checked_in["outcome"])
        self.assertFalse(checked_in["native_editable_step_assembly_ready"])

    def _write_candidate_source(self, root: Path, source: str, *, manifest: bool = True) -> Path:
        source_root = root / AUDIT.NATIVE_BREP_PYTHON_ROOT
        source_root.mkdir(parents=True)
        source_path = source_root / "candidate.py"
        source_path.write_text(source, encoding="utf-8")
        if manifest:
            relative_path = source_path.relative_to(root).as_posix()
            source_root.joinpath("candidate_source_manifest.json").write_text(
                json.dumps(
                    {
                        "verification": {"algorithm": "sha256"},
                        "source_files": [
                            {
                                "path": relative_path,
                                "sha256": hashlib.sha256(source_path.read_bytes()).hexdigest(),
                                "role": "editable_parametric_cadquery_brep_source",
                            }
                        ],
                    }
                ),
                encoding="utf-8",
            )
        return source_path

    def test_disguised_python_file_is_not_native_brep_source(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            source_path = self._write_candidate_source(root, "def build():\n    return 'pretend CAD'\n")
            rejection_codes = AUDIT.python_cadquery_source_rejection_codes(source_path, root)
            self.assertIn("CADQUERY_IMPORT_NOT_LAZY", rejection_codes)
            self.assertIn("CADQUERY_ENTITY_CONSTRUCTION_UNPROVEN", rejection_codes)

    def test_python_source_that_writes_stl_is_not_native_brep_source(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            source_path = self._write_candidate_source(
                root,
                "def build():\n    import cadquery as cq\n    shape = cq.Workplane('XY').box(1, 1, 1)\n    cq.exporters.export(shape, 'visual.stl')\n    return shape\n",
            )
            rejection_codes = AUDIT.python_cadquery_source_rejection_codes(source_path, root)
            self.assertIn("PYTHON_SOURCE_EMITS_OR_REFERENCES_MESH", rejection_codes)

    def test_python_source_without_manifest_hash_is_not_native_brep_source(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            source_path = self._write_candidate_source(
                root,
                "def build():\n    import cadquery as cq\n    return cq.Workplane('XY').box(1, 1, 1)\n",
                manifest=False,
            )
            rejection_codes = AUDIT.python_cadquery_source_rejection_codes(source_path, root)
            self.assertEqual(["CADQUERY_SOURCE_MANIFEST_OR_HASH_UNPROVEN"], rejection_codes)

    def test_faceted_step_text_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            step_path = Path(temporary_directory) / "mesh_disguised_as_step.step"
            step_path.write_text("ISO-10303-21; HEADER; FILE_SCHEMA(('AUTOMOTIVE_DESIGN')); FACETED_BREP; ENDSEC;", encoding="ascii")
            self.assertFalse(AUDIT.is_non_mesh_step(step_path))

    def test_json_output_is_machine_readable_and_strict_mode_fails(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            output = Path(temporary_directory) / "native_cad_preflight.json"
            self.assertEqual(0, AUDIT.main(["--root", str(REPOSITORY_ROOT), "--output", str(output)]))
            report = json.loads(output.read_text(encoding="utf-8"))
            self.assertEqual("native_cad_readiness", report["audit_name"])
            self.assertEqual(1, AUDIT.main(["--root", str(REPOSITORY_ROOT), "--strict", "--output", str(output)]))


if __name__ == "__main__":
    unittest.main()
