#!/usr/bin/env python3
"""Regression tests for the static native B-rep reconstruction manifest."""

from __future__ import annotations

import importlib.util
import json
import tempfile
import unittest
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
VALIDATOR_PATH = REPOSITORY_ROOT / "scripts" / "validate_native_brep_reconstruction_manifest.py"
MANIFEST_PATH = REPOSITORY_ROOT / "config" / "high_fidelity_vehicle" / "native_brep_reconstruction_manifest.json"
SCHEMA_PATH = REPOSITORY_ROOT / "config" / "high_fidelity_vehicle" / "native_brep_reconstruction_manifest.schema.json"

SPEC = importlib.util.spec_from_file_location("validate_native_brep_reconstruction_manifest", VALIDATOR_PATH)
assert SPEC and SPEC.loader
VALIDATOR = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(VALIDATOR)


class NativeBrepReconstructionManifestTests(unittest.TestCase):
    def test_current_manifest_is_valid_and_pending(self) -> None:
        report = VALIDATOR.validate(REPOSITORY_ROOT, MANIFEST_PATH, SCHEMA_PATH)
        self.assertTrue(report["valid"], report["errors"])
        self.assertEqual(105, report["summary"]["project_authored_parts_pending"])
        self.assertEqual(21, report["summary"]["excluded_vendor_reference_meshes"])
        self.assertEqual(126, report["summary"]["source_meshes_accounted_for"])
        self.assertEqual(0, report["summary"]["native_or_step_artifacts_created"])

    def test_mesh_conversion_is_explicitly_prohibited(self) -> None:
        manifest = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
        self.assertIn("mesh_to_step_conversion", manifest["prohibited_methods"])
        self.assertTrue(all(part["conversion_policy"] == "native_brep_feature_rebuild_only" for part in manifest["parts"]))
        self.assertTrue(all(part["status"] == "pending_native_brep_reconstruction" for part in manifest["parts"]))

    def test_every_generator_output_is_accounted_for_once(self) -> None:
        manifest = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
        sources = [(item["source_generator"], item["source_mesh"]) for item in manifest["parts"]]
        sources.extend((item["source_generator"], item["source_mesh"]) for item in manifest["excluded_generator_outputs"])
        self.assertEqual(126, len(sources))
        self.assertEqual(len(sources), len(set(sources)))
        self.assertEqual(
            {"product_bodywork": 47, "cleaning_storage": 61, "power_service_hardware": 18},
            {generator["id"]: generator["generated_mesh_count"] for generator in manifest["source_generators"]},
        )

    def test_missing_output_or_fake_native_artifact_fails_closed(self) -> None:
        manifest = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
        with tempfile.TemporaryDirectory() as temporary_directory:
            temporary = Path(temporary_directory)
            broken = temporary / "manifest.json"
            manifest["parts"].pop()
            broken.write_text(json.dumps(manifest), encoding="utf-8")
            report = VALIDATOR.validate(REPOSITORY_ROOT, broken, SCHEMA_PATH)
            self.assertFalse(report["valid"])
            self.assertTrue(any("coverage" in error for error in report["errors"]))

            manifest = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
            manifest["parts"][0]["target_native_source"] = "README.md"
            broken.write_text(json.dumps(manifest), encoding="utf-8")
            report = VALIDATOR.validate(REPOSITORY_ROOT, broken, SCHEMA_PATH)
            self.assertFalse(report["valid"])
            self.assertTrue(any("target_native_source" in error for error in report["errors"]))


if __name__ == "__main__":
    unittest.main()
