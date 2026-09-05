"""Low-memory tests for the fail-closed S100P formal board bundle audit."""

from __future__ import annotations

import copy
import importlib.util
import json
import tempfile
import unittest
from pathlib import Path
from unittest import mock


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "validate_s100p_formal_board_bundle", ROOT / "scripts" / "validate_s100p_formal_board_bundle.py"
)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class S100PFormalBoardBundleTests(unittest.TestCase):
    def test_current_manifest_is_copyable_but_deployment_blocked(self) -> None:
        report = MODULE.validate_manifest()
        self.assertEqual(report["status"], "BLOCKED")
        self.assertFalse(report["ready_to_deploy"])
        self.assertTrue(report["manifest_copyable"])
        self.assertFalse(report["payload_copy_authorized"])
        self.assertTrue(report["checks"]["bound_source_digests_valid"])
        self.assertTrue(report["checks"]["formal_snapshot_file_matches_declaration"])
        self.assertTrue(report["checks"]["formal_snapshot_content_matches_declaration"])
        self.assertTrue(report["checks"]["launch_binds_dosod_edgesam_and_project_adapters"])
        self.assertTrue(
            report["checks"][
                "product_bundle_overlay_inventory_digest_matches_contract"
            ]
        )
        self.assertTrue(report["checks"]["required_board_payload_roles_exact"])
        self.assertTrue(report["checks"]["payload_roles_match_product_artifact_bundle"])
        self.assertTrue(report["checks"]["payload_roles_match_launch_parameter_record"])
        self.assertTrue(report["checks"]["launch_binds_each_required_board_payload_role"])
        self.assertTrue(report["checks"]["overlay_runtime_dependency_closure_classified"])
        self.assertIn("project_dosod_hbm_missing_or_unhashed", report["blockers"])
        self.assertIn("thermal_and_power_measurement_not_collected", report["blockers"])

    def test_source_digest_tamper_is_detected_without_board_operation(self) -> None:
        manifest_path = ROOT / "config" / "s100p_formal_board_bundle_manifest.json"
        payload = json.loads(manifest_path.read_text(encoding="utf-8"))
        altered = copy.deepcopy(payload)
        altered["bound_sources"][0]["sha256"] = "0" * 64
        with tempfile.TemporaryDirectory() as temporary:
            candidate = Path(temporary) / "bundle.json"
            candidate.write_text(json.dumps(altered), encoding="utf-8")
            report = MODULE.validate_manifest(candidate)
        self.assertEqual(report["status"], "BLOCKED")
        self.assertFalse(report["checks"]["bound_source_digests_valid"])
        self.assertIn("bound_source_digest_mismatch:dosod_hbm_compile_contract", report["blockers"])
        self.assertFalse(report["board_operations_performed"])

    def test_required_payload_role_target_path_mismatch_is_blocked(self) -> None:
        manifest_path = ROOT / "config" / "s100p_formal_board_bundle_manifest.json"
        altered = json.loads(manifest_path.read_text(encoding="utf-8"))
        altered["required_board_payload_roles"][0]["target_relative_path"] = "dosod/wrong.hbm"
        with tempfile.TemporaryDirectory() as temporary:
            candidate = Path(temporary) / "bundle.json"
            candidate.write_text(json.dumps(altered), encoding="utf-8")
            report = MODULE.validate_manifest(candidate)
        self.assertEqual(report["status"], "BLOCKED")
        self.assertFalse(report["checks"]["required_board_payload_roles_exact"])
        self.assertIn("required_board_payload_roles_not_exact", report["blockers"])

    def test_required_payload_role_receipt_requirement_cannot_be_relaxed(self) -> None:
        manifest_path = ROOT / "config" / "s100p_formal_board_bundle_manifest.json"
        altered = json.loads(manifest_path.read_text(encoding="utf-8"))
        altered["required_board_payload_roles"][0]["source_receipt_required"] = False
        with tempfile.TemporaryDirectory() as temporary:
            candidate = Path(temporary) / "bundle.json"
            candidate.write_text(json.dumps(altered), encoding="utf-8")
            report = MODULE.validate_manifest(candidate)
        self.assertEqual(report["status"], "BLOCKED")
        self.assertFalse(report["checks"]["required_board_payload_roles_exact"])
        self.assertIn("required_board_payload_roles_not_exact", report["blockers"])

    def test_missing_bound_source_returns_structured_blocked_report(self) -> None:
        manifest_path = ROOT / "config" / "s100p_formal_board_bundle_manifest.json"
        altered = json.loads(manifest_path.read_text(encoding="utf-8"))
        altered["bound_sources"][0]["path"] = "config/missing-bound-source.json"
        with tempfile.TemporaryDirectory() as temporary:
            candidate = Path(temporary) / "bundle.json"
            candidate.write_text(json.dumps(altered), encoding="utf-8")
            report = MODULE.validate_manifest(candidate)
        self.assertEqual(report["status"], "BLOCKED")
        self.assertFalse(report["checks"]["bound_source_digests_valid"])
        self.assertIn("bound_source_missing:dosod_hbm_compile_contract", report["blockers"])

    def test_unreadable_bound_source_returns_structured_blocked_report(self) -> None:
        original_sha256 = MODULE._sha256

        def deny_compile_contract(path: Path) -> str:
            if path.name == "dosod_s100p_hbm_compile_contract.json":
                raise PermissionError("simulated read denial")
            return original_sha256(path)

        with mock.patch.object(MODULE, "_sha256", side_effect=deny_compile_contract):
            report = MODULE.validate_manifest()
        self.assertEqual(report["status"], "BLOCKED")
        self.assertFalse(report["checks"]["bound_source_digests_valid"])
        self.assertIn(
            "bound_source_unreadable:dosod_hbm_compile_contract:PermissionError", report["blockers"]
        )

    def test_validator_does_not_import_execution_or_network_modules(self) -> None:
        source = (ROOT / "scripts" / "validate_s100p_formal_board_bundle.py").read_text(encoding="utf-8")
        for forbidden in ("subprocess", "socket", "paramiko", "shutil", "ros2", "ssh"):
            self.assertNotIn(f"import {forbidden}", source)
            self.assertNotIn(f"from {forbidden}", source)


if __name__ == "__main__":
    unittest.main()
