#!/usr/bin/env python3
"""Static regression checks for the fail-closed serial CadQuery export route."""

from __future__ import annotations

import ast
import contextlib
import importlib.util
import io
import sys
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "run_native_cadquery_serial_export.py"
CONTRACT = ROOT / "config" / "high_fidelity_vehicle" / "native_cadquery_serial_export_contract.json"
BOOTSTRAP = ROOT / "scripts" / "bootstrap_native_cadquery_windows.ps1"

SPEC = importlib.util.spec_from_file_location("native_cadquery_serial_export", SCRIPT)
assert SPEC and SPEC.loader
SERIAL = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = SERIAL
SPEC.loader.exec_module(SERIAL)


class NativeCadQuerySerialExportTests(unittest.TestCase):
    @staticmethod
    def _sources() -> list[dict[str, object]]:
        return [
            {
                "batch_id": "batch_one",
                "component_ids": ["part_a", "part_b"],
                "source": "source.py",
                "source_sha256": "source",
                "contract": "contract.json",
                "contract_sha256": "contract",
                "source_manifest": "source_manifest.json",
                "source_manifest_sha256": "manifest",
                "adapter": "fixture",
            }
        ]

    def test_contract_has_eight_sources_and_exactly_105_component_targets(self) -> None:
        contract = SERIAL.load_contract(ROOT)
        batches = contract["source_batches"]
        self.assertEqual(8, len(batches))
        self.assertEqual(105, sum(int(batch["component_export_count"]) for batch in batches))
        self.assertEqual(4, sum(batch["role"] == "component_addressable" for batch in batches))
        self.assertFalse(contract["preview"]["implemented"])
        for batch in batches:
            self.assertTrue((ROOT / batch["source"]).is_file())
            self.assertTrue((ROOT / batch["contract"]).is_file())
            self.assertTrue((ROOT / batch["source_manifest"]).is_file())

    def test_current_pending_contract_blocks_before_source_loading_or_cadquery(self) -> None:
        invoked: list[str] = []
        cadquery_was_loaded = "cadquery" in sys.modules
        original = SERIAL._load_source_module
        SERIAL._load_source_module = lambda *args: invoked.append("source")  # type: ignore[method-assign]
        try:
            with self.assertRaises(SERIAL.ExportBlocked):
                SERIAL.check_release_authorization(ROOT, SERIAL.load_contract(ROOT))
        finally:
            SERIAL._load_source_module = original  # type: ignore[method-assign]
        self.assertEqual([], invoked)
        self.assertEqual(cadquery_was_loaded, "cadquery" in sys.modules)

    def test_script_has_no_top_level_cadquery_import_or_mesh_conversion(self) -> None:
        tree = ast.parse(SCRIPT.read_text(encoding="utf-8"), filename=str(SCRIPT))
        top_level_imports = [
            alias.name
            for node in tree.body
            if isinstance(node, (ast.Import, ast.ImportFrom))
            for alias in node.names
        ]
        self.assertNotIn("cadquery", top_level_imports)
        text = SCRIPT.read_text(encoding="utf-8")
        self.assertIn("_source_is_lazy_and_mesh_free", text)
        self.assertIn("MESH_STEP_MARKERS", text)
        self.assertIn("require_windows_native_resources", text)
        self.assertIn("component-logs", text)
        self.assertIn("sha256-receipt.json", text)
        self.assertIn("--resume", text)
        self.assertIn("CHECKPOINT_NAME", text)
        self.assertIn("gc.collect()", text)
        self.assertIn("os.replace", text)

    def test_all_pending_prefixes_fail_closed(self) -> None:
        self.assertTrue(
            SERIAL._has_open_pending_input(
                {"pending_native_export_inputs": ["controlled hole evidence"]}
            )
        )
        self.assertTrue(
            SERIAL._has_open_pending_input(
                {"nested": {"pending_supplier_release": {"status": "open"}}}
            )
        )

    def test_step_scan_rejects_late_faceted_marker(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            step = Path(temporary) / "late-faceted.step"
            step.write_bytes(
                b"ISO-10303-21;ADVANCED_BREP_SHAPE_REPRESENTATION;"
                + b" " * 300_000
                + b"FACETED_BREP;END-ISO-10303-21;"
            )
            self.assertFalse(SERIAL._step_header_ok(step))

    def test_checkpoint_resume_requires_exact_hash_bound_serial_prefix(self) -> None:
        sources = self._sources()
        with tempfile.TemporaryDirectory() as temporary:
            staging = Path(temporary) / ".export.incomplete"
            staging.mkdir()
            step = staging / "components" / "batch_one__part_a.step"
            step.parent.mkdir()
            step.write_bytes(
                b"ISO-10303-21;ADVANCED_BREP_SHAPE_REPRESENTATION;END-ISO-10303-21;"
            )
            row = {
                "outcome": "passed",
                "batch_id": "batch_one",
                "component_id": "part_a",
                "step": "components/batch_one__part_a.step",
                "step_sha256": SERIAL.sha256(step),
                "topology": {"solids": 1},
            }
            SERIAL._write_json(
                staging / SERIAL.CHECKPOINT_NAME,
                SERIAL._checkpoint_payload(sources, [row], state="failed"),
            )
            self.assertEqual(
                [row], SERIAL._completed_checkpoint_rows(staging, sources)
            )
            changed_sources = self._sources()
            changed_sources[0]["component_ids"] = ["part_b", "part_a"]
            with self.assertRaises(SERIAL.ExportBlocked):
                SERIAL._completed_checkpoint_rows(staging, changed_sources)

    def test_prepare_staging_requires_explicit_resume_and_does_not_overwrite(self) -> None:
        sources = self._sources()
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary) / "release"
            staging, completed = SERIAL._prepare_staging(output, sources, resume=False)
            self.assertEqual([], completed)
            self.assertTrue((staging / SERIAL.CHECKPOINT_NAME).is_file())
            with self.assertRaises(SERIAL.ExportBlocked):
                SERIAL._prepare_staging(output, sources, resume=False)
            resumed_staging, resumed = SERIAL._prepare_staging(
                output, sources, resume=True
            )
            self.assertEqual(staging, resumed_staging)
            self.assertEqual([], resumed)

    def test_atomic_json_failure_leaves_prior_file_intact(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            target = Path(temporary) / "checkpoint.json"
            target.write_text("prior", encoding="utf-8")
            original = SERIAL.os.replace
            SERIAL.os.replace = lambda *_args: (_ for _ in ()).throw(OSError("fail"))  # type: ignore[method-assign]
            try:
                with self.assertRaises(OSError):
                    SERIAL._write_json(target, {"new": "value"})
            finally:
                SERIAL.os.replace = original  # type: ignore[method-assign]
            self.assertEqual("prior", target.read_text(encoding="utf-8"))
            self.assertEqual([], list(target.parent.glob(".checkpoint.json.*.tmp")))

    def test_preflight_only_remains_before_cadquery_or_source_module_loading(self) -> None:
        cadquery_was_loaded = "cadquery" in sys.modules
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            self.assertEqual(
                3,
                SERIAL.main(["--repo-root", str(ROOT), "--preflight-only"]),
            )
        self.assertIn('"outcome": "blocked_or_failed"', output.getvalue())
        self.assertEqual(cadquery_was_loaded, "cadquery" in sys.modules)

    def test_output_is_restricted_to_work_and_bootstrap_preflights_formal_release(self) -> None:
        allowed = ROOT / ".work" / "native-cadquery-serial-release"
        self.assertEqual(allowed.resolve(), SERIAL.require_work_directory(ROOT, allowed))
        with self.assertRaises(SERIAL.ExportBlocked):
            SERIAL.require_work_directory(ROOT, ROOT / "reports" / "engineering")
        bootstrap = BOOTSTRAP.read_text(encoding="utf-8")
        self.assertIn("[switch]$FormalExport", bootstrap)
        self.assertIn("--preflight-only", bootstrap)
        self.assertLess(bootstrap.index("--preflight-only"), bootstrap.index("-m venv"))


if __name__ == "__main__":
    unittest.main()
