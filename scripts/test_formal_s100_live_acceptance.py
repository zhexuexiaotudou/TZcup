#!/usr/bin/env python3

from __future__ import annotations

import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import yaml

from formal_s100_live_acceptance_core import (
    FINAL_BLOCKED,
    FINAL_PASSED,
    RAW_BLOCKED,
    RAW_COLLECTED,
    RAW_REPORT_ID,
    acceptance_session_binding,
    active_session_identity,
    build_final_report,
    probe_hardware,
    runtime_closure_binding,
    sha256_path,
    snapshot_identity,
    validate_raw,
)
from validate_formal_s100_live_runtime import validate_schema


ROOT = Path(__file__).resolve().parents[1]
RAW_SCHEMA = ROOT / "config/high_fidelity_vehicle/formal_s100_live_runtime.schema.json"
FINAL_SCHEMA = ROOT / "config/high_fidelity_vehicle/formal_s100_live_acceptance.schema.json"


class FormalS100LiveAcceptanceTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.snapshot = self.root / "snapshot.json"
        self.snapshot.write_text(
            json.dumps({"source_inventory_sha256": "1" * 64}) + "\n", encoding="utf-8"
        )
        self.identity = snapshot_identity(self.snapshot)
        self.runtime_closure = self.root / "runtime-closure.json"
        self.runtime_closure.write_text(
            json.dumps(
                {
                    "schema_version": 6,
                    "kind": "tzcup_formal_final_runtime_closure",
                    "status": "FORMAL_FINAL_RUNTIME_CLOSURE_FROZEN",
                    "closure_sha256": "8" * 64,
                }
            ),
            encoding="utf-8",
        )
        self.closure_binding = runtime_closure_binding(self.runtime_closure)
        self.session = self.root / "session.json"
        self.session.write_text(
            json.dumps(
                {
                    "schema_version": 1,
                    "report_id": "tzcup_formal_final_acceptance_session_v1",
                    "status": "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING",
                    "started_epoch_ns": 123456789,
                    "snapshot": self.identity,
                    "runtime_closure_binding": {
                        "manifest_sha256": self.closure_binding[
                            "runtime_closure_manifest_sha256"
                        ],
                        "closure_sha256": self.closure_binding[
                            "runtime_closure_sha256"
                        ],
                    },
                }
            ),
            encoding="utf-8",
        )
        self.session_binding = acceptance_session_binding(
            self.session, self.identity, self.closure_binding
        )
        self.active_session = active_session_identity(
            self.session, self.identity, self.closure_binding
        )
        self.raw_path = self.root / "raw.json"

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def valid_raw(self) -> dict:
        models = {
            "dosod_hbm": "2" * 64,
            "dosod_vocabulary": "3" * 64,
            "edgesam_encoder_hbm": "4" * 64,
            "edgesam_decoder_hbm": "5" * 64,
        }
        return {
            "schema_version": 1,
            "report_id": RAW_REPORT_ID,
            "status": RAW_COLLECTED,
            "collection_complete": True,
            "hardware": {
                "attested": True,
                "architecture": "aarch64",
                "board": "RDK S100P",
                "soc": "Journey 6P",
                "sku": "D-Robotics RDK S100P Journey 6P",
                "device_tree_model": "D-Robotics RDK S100P Journey 6P",
                "device_tree_model_sha256": "6" * 64,
                "device_tree_compatible": "d-robotics,rdk-s100p horizon,journey6p",
                "device_tree_compatible_sha256": "7" * 64,
                "blockers": [],
            },
            "collector": {
                "script_sha256": sha256_path(ROOT / "scripts/collect_formal_s100_live_runtime.py"),
                "pid": 123,
            },
            "source_binding": dict(self.identity),
            "acceptance_session_binding": dict(self.session_binding),
            "runtime_closure_binding": dict(self.closure_binding),
            "system_image": {
                "os_release": {"ID": "ubuntu", "VERSION_ID": "22.04", "IMAGE_ID": "rdk-s100-4.1"},
                "os_release_sha256": "9" * 64,
                "kernel_release": "5.15.0-rdk-s100",
                "kernel_version": "#1 SMP",
                "uname": {"returncode": 0},
                "runtime_inventory": {
                    "ros2": {"returncode": 0},
                    "hbrt4": {"returncode": 0},
                    "hrt_model_exec": {"returncode": 127},
                },
            },
            "models": [
                {"role": role, "path": f"/opt/models/{role}", "sha256": digest, "byte_size": 1024}
                for role, digest in models.items()
            ],
            "dosod_hbm_evidence": {
                "compile": {
                    "path": "/opt/evidence/compile.json", "sha256": "a" * 64,
                    "report_id": "tzcup_s100p_dosod_hbm_compile_receipt_v1",
                    "status": "COMPILED_NOT_BOARD_ACCEPTED", "hbm_sha256": models["dosod_hbm"],
                },
                "parity": {
                    "path": "/opt/evidence/parity.json", "sha256": "b" * 64,
                    "report_id": "tzcup_dosod_hbm_x86_nash_parity_v1",
                    "status": "PARITY_PASSED", "hbm_sha256": models["dosod_hbm"],
                },
                "metric": {
                    "path": "/opt/evidence/metric.json", "sha256": "c" * 64,
                    "report_id": "tzcup_dosod_quantized_metric_regression_v1",
                    "status": "REGRESSION_PASSED", "hbm_sha256": models["dosod_hbm"],
                },
                "full_admission": {
                    "bundle_path": "/opt/evidence/bundle", "bundle_manifest_sha256": "d" * 64,
                    "compile_receipt_sha256": "a" * 64, "parity_report_sha256": "b" * 64,
                    "metric_report_sha256": "c" * 64, "hbm_sha256": models["dosod_hbm"],
                },
            },
            "ros_graph": {
                "nodes": ["/hobot_dosod", "/mono_edgesam", "/open_vocab_product_adapter"],
                "topics": {
                    "/perception/garbage/detections_2d": ["vision_msgs/msg/Detection2DArray"],
                    "/perception/ground_dirt/masks": ["sensor_msgs/msg/Image"],
                    "/perception/garbage/targets": ["sanitation_perception_interfaces/msg/GarbageTargetArray"],
                    "/perception/wrist/grasp_recheck": ["std_msgs/msg/String"],
                    "/perception/open_vocab/diagnostics": ["diagnostic_msgs/msg/DiagnosticArray"],
                    "/perception/open_vocab/dosod_boxes": ["vision_msgs/msg/Detection2DArray"],
                },
                "node_info": {},
            },
            "ros_graph_start": {},
            "inference_telemetry": {
                "dosod": {
                    "backend": "bpu",
                    "model_sha256": models["dosod_hbm"],
                    "vocabulary_sha256": models["dosod_vocabulary"],
                    "samples": 4000,
                    "fps": 2.2,
                    "latency_ms_p50": 90.0,
                    "latency_ms_p95": 130.0,
                    "latency_ms_p99": 170.0,
                    "latency_ms_max": 190.0,
                    "runtime_process_maps_backend_match": True,
                },
                "edgesam": {
                    "backend": "bpu",
                    "model_sha256": models["edgesam_encoder_hbm"],
                    "model_sha256s": [models["edgesam_encoder_hbm"], models["edgesam_decoder_hbm"]],
                    "samples": 2000,
                    "fps": 1.1,
                    "latency_ms_p50": 100.0,
                    "latency_ms_p95": 180.0,
                    "latency_ms_p99": 220.0,
                    "latency_ms_max": 250.0,
                    "runtime_process_maps_backend_match": True,
                },
                "product_output_counts": {
                    "/perception/garbage/detections_2d": 100,
                    "/perception/ground_dirt/masks": 100,
                    "/perception/garbage/targets": 100,
                    "/perception/wrist/grasp_recheck": 10,
                },
                "product_nonempty_counts": {
                    "/perception/garbage/detections_2d": 80,
                    "/perception/ground_dirt/masks": 70,
                    "/perception/garbage/targets": 50,
                    "/perception/wrist/grasp_recheck": 5,
                },
            },
            "sustained_run": {
                "duration_sec": 1801.0,
                "sample_period_sec": 1.0,
                "process_restarts": 0,
                "node_disappearances": 0,
                "inference_failures": 0,
            },
            "resources": {
                "system_memory_total_bytes": 8_000_000_000,
                "system_memory_available_min_bytes": 1_000_000_000,
                "perception_rss_peak_bytes": 1_500_000_000,
                "sample_count": 1801,
            },
            "thermal": {"samples": 1801, "peak_celsius": 72.5, "zones_last_sample": []},
            "truth_boundary": {"simulator_or_evaluator_truth_used": False},
            "blockers": [],
        }

    def test_device_tree_attestation_requires_architecture_sku_and_soc(self) -> None:
        model = self.root / "proc/device-tree/model"
        compatible = self.root / "proc/device-tree/compatible"
        model.parent.mkdir(parents=True)
        model.write_bytes(b"D-Robotics RDK S100P Journey 6P\x00")
        compatible.write_bytes(b"d-robotics,rdk-s100p\x00horizon,journey6p\x00")
        self.assertTrue(probe_hardware(self.root, machine="aarch64")["attested"])
        rejected = probe_hardware(self.root, machine="x86_64")
        self.assertFalse(rejected["attested"])
        self.assertTrue(any("architecture" in row for row in rejected["blockers"]))

    def test_real_s100p_v1p0_device_tree_signature_is_accepted(self) -> None:
        model = self.root / "proc/device-tree/model"
        compatible = self.root / "proc/device-tree/compatible"
        model.parent.mkdir(parents=True, exist_ok=True)
        model.write_bytes(b"D-Robotics RDK S100P V1P0\x00")
        compatible.write_bytes(b"drobot,s100-rdk\x00")
        observed = probe_hardware(self.root, machine="aarch64")
        self.assertTrue(observed["attested"])
        self.assertEqual(observed["board"], "RDK S100P")
        self.assertEqual(observed["soc"], "Journey 6P")
        self.assertEqual(
            observed["soc_identity_basis"],
            "s100p_model_plus_drobot_s100_rdk_compatible",
        )

    def test_generic_arm_host_cannot_pass_with_only_s100_family_compatible(self) -> None:
        model = self.root / "proc/device-tree/model"
        compatible = self.root / "proc/device-tree/compatible"
        model.parent.mkdir(parents=True, exist_ok=True)
        model.write_bytes(b"Generic ARM64 development board\x00")
        compatible.write_bytes(b"drobot,s100-rdk\x00")
        rejected = probe_hardware(self.root, machine="aarch64")
        self.assertFalse(rejected["attested"])
        self.assertTrue(any("S100P" in row for row in rejected["blockers"]))

    def test_legacy_generic_s100_identity_cannot_pass_the_s100p_gate(self) -> None:
        model = self.root / "proc/device-tree/model"
        compatible = self.root / "proc/device-tree/compatible"
        model.parent.mkdir(parents=True, exist_ok=True)
        model.write_bytes(b"D-Robotics RDK S100 Journey 6\x00")
        compatible.write_bytes(b"d-robotics,rdk-s100\x00horizon,journey6\x00")
        rejected = probe_hardware(self.root, machine="aarch64")
        self.assertFalse(rejected["attested"])
        self.assertTrue(any("S100P" in row for row in rejected["blockers"]))
        self.assertTrue(any("Journey 6P" in row for row in rejected["blockers"]))

    def test_valid_live_evidence_passes_semantics_and_both_schemas(self) -> None:
        raw = self.valid_raw()
        self.assertIn("DOSOD full admission bundle is required for core validation", validate_raw(raw, self.identity, self.active_session, self.closure_binding))
        self.assertEqual(validate_schema(raw, RAW_SCHEMA), [])
        self.raw_path.write_text(json.dumps(raw), encoding="utf-8")
        report = build_final_report(
            raw, self.identity, self.raw_path, self.active_session, self.closure_binding
        )
        self.assertNotEqual(report["status"], FINAL_PASSED)
        self.assertFalse(report["passed"])
        self.assertEqual(validate_schema(report, FINAL_SCHEMA), [])

    def test_handwritten_dosod_triad_cannot_pass_without_the_real_admission_bundle(self) -> None:
        failures = validate_raw(
            self.valid_raw(), self.identity, self.active_session, self.closure_binding
        )
        self.assertIn("DOSOD full admission bundle is required for core validation", failures)

    def test_rejects_old_session_or_drifted_runtime_closure(self) -> None:
        raw = self.valid_raw()
        raw["acceptance_session_binding"]["session_started_epoch_ns"] += 1
        self.assertTrue(validate_raw(raw, self.identity, self.active_session, self.closure_binding))

        raw = self.valid_raw()
        raw["runtime_closure_binding"]["runtime_closure_sha256"] = "a" * 64
        self.assertTrue(validate_raw(raw, self.identity, self.active_session, self.closure_binding))

    def test_pending_session_preserves_identity_without_reusing_a_different_session(self) -> None:
        raw = self.valid_raw()
        session = json.loads(self.session.read_text(encoding="utf-8"))
        session["status"] = "FORMAL_FINAL_ACCEPTANCE_SESSION_PENDING"
        self.session.write_text(json.dumps(session), encoding="utf-8")
        pending_identity = active_session_identity(
            self.session, self.identity, self.closure_binding
        )
        self.assertIn("DOSOD full admission bundle is required for core validation", validate_raw(raw, self.identity, pending_identity, self.closure_binding))

    def test_final_schema_rejects_pass_with_missing_or_false_admission_check(self) -> None:
        raw = self.valid_raw()
        self.raw_path.write_text(json.dumps(raw), encoding="utf-8")
        report = build_final_report(
            raw, self.identity, self.raw_path, self.active_session, self.closure_binding
        )
        report["passed"] = True
        report["status"] = FINAL_PASSED
        report["checks"]["runtime_closure_binding"] = False
        self.assertTrue(validate_schema(report, FINAL_SCHEMA))
        report = build_final_report(
            raw, self.identity, self.raw_path, self.active_session, self.closure_binding
        )
        del report["checks"]["acceptance_session_binding"]
        self.assertTrue(validate_schema(report, FINAL_SCHEMA))

    def test_offline_validator_refuses_evidence_from_another_session(self) -> None:
        raw = self.valid_raw()
        self.raw_path.write_text(json.dumps(raw), encoding="utf-8")
        other_session = self.root / "other-session.json"
        other_session.write_text(
            json.dumps(
                {
                    "schema_version": 1,
                    "report_id": "tzcup_formal_final_acceptance_session_v1",
                    "status": "FORMAL_FINAL_ACCEPTANCE_SESSION_PENDING",
                    "started_epoch_ns": 987654321,
                    "snapshot": self.identity,
                    "runtime_closure_binding": {
                        "manifest_sha256": self.closure_binding[
                            "runtime_closure_manifest_sha256"
                        ],
                        "closure_sha256": self.closure_binding[
                            "runtime_closure_sha256"
                        ],
                    },
                }
            ),
            encoding="utf-8",
        )
        output = self.root / "blocked-final.json"
        result = subprocess.run(
            [
                sys.executable,
                str(ROOT / "scripts/validate_formal_s100_live_runtime.py"),
                "--raw",
                str(self.raw_path),
                "--snapshot",
                str(self.snapshot),
                "--acceptance-session",
                str(other_session),
                "--runtime-closure",
                str(self.runtime_closure),
                "--dosod-admission-bundle",
                str(self.root),
                "--output",
                str(output),
            ],
            capture_output=True,
            text=True,
            check=False,
        )
        self.assertEqual(result.returncode, 4)
        final = json.loads(output.read_text(encoding="utf-8"))
        self.assertFalse(final["passed"])
        self.assertTrue(any("active session" in row for row in final["blockers"]))

    def test_session_runtime_closure_must_match_the_active_board_closure(self) -> None:
        session = json.loads(self.session.read_text(encoding="utf-8"))
        session["runtime_closure_binding"]["closure_sha256"] = "f" * 64
        self.session.write_text(json.dumps(session), encoding="utf-8")

        with self.assertRaisesRegex(ValueError, "runtime closure does not match"):
            acceptance_session_binding(self.session, self.identity, self.closure_binding)
        with self.assertRaisesRegex(ValueError, "runtime closure does not match"):
            active_session_identity(self.session, self.identity, self.closure_binding)

    def test_legacy_core_call_without_a_closure_fails_closed(self) -> None:
        with self.assertRaisesRegex(ValueError, "closure binding is required"):
            acceptance_session_binding(self.session, self.identity)
        with self.assertRaisesRegex(ValueError, "closure binding is required"):
            active_session_identity(self.session, self.identity)

    def test_offline_validator_rejects_session_runtime_closure_drift(self) -> None:
        raw = self.valid_raw()
        self.raw_path.write_text(json.dumps(raw), encoding="utf-8")
        session = json.loads(self.session.read_text(encoding="utf-8"))
        session["runtime_closure_binding"]["manifest_sha256"] = "f" * 64
        self.session.write_text(json.dumps(session), encoding="utf-8")
        output = self.root / "closure-drift-final.json"
        result = subprocess.run(
            [
                sys.executable,
                str(ROOT / "scripts/validate_formal_s100_live_runtime.py"),
                "--raw",
                str(self.raw_path),
                "--snapshot",
                str(self.snapshot),
                "--acceptance-session",
                str(self.session),
                "--runtime-closure",
                str(self.runtime_closure),
                "--dosod-admission-bundle",
                str(self.root),
                "--output",
                str(output),
            ],
            capture_output=True,
            text=True,
            check=False,
        )
        self.assertEqual(result.returncode, 2)
        self.assertIn("runtime closure does not match", result.stdout)
        self.assertFalse(output.exists())

    def test_collector_binds_closure_before_acceptance_session(self) -> None:
        source = (ROOT / "scripts/collect_formal_s100_live_runtime.py").read_text(
            encoding="utf-8"
        )
        closure_assignment = source.index('base["runtime_closure_binding"] = runtime_closure_binding')
        session_binding = source.index('base["acceptance_session_binding"] = acceptance_session_binding')
        assert closure_assignment < session_binding
        assert 'base["runtime_closure_binding"],' in source

    def test_non_board_and_blocked_raw_can_never_pass(self) -> None:
        raw = self.valid_raw()
        raw["status"] = RAW_BLOCKED
        raw["collection_complete"] = False
        raw["hardware"]["attested"] = False
        raw["hardware"]["board"] = None
        raw["hardware"]["soc"] = None
        raw["blockers"] = ["collector refuses non-RDK-S100 hardware"]
        self.raw_path.write_text(json.dumps(raw), encoding="utf-8")
        report = build_final_report(raw, self.identity, self.raw_path)
        self.assertEqual(report["status"], FINAL_BLOCKED)
        self.assertFalse(report["passed"])

    def test_rejects_model_backend_duration_thermal_memory_restart_and_truth_failures(self) -> None:
        mutations = {
            "model mismatch": lambda row: row["inference_telemetry"]["dosod"].__setitem__("model_sha256", "a" * 64),
            "vocabulary mismatch": lambda row: row["inference_telemetry"]["dosod"].__setitem__("vocabulary_sha256", "a" * 64),
            "backend absent": lambda row: row["inference_telemetry"]["edgesam"].__setitem__("backend", "unknown"),
            "cpu fallback": lambda row: row["inference_telemetry"]["dosod"].__setitem__("backend", "cpu"),
            "low fps": lambda row: row["inference_telemetry"]["edgesam"].__setitem__("fps", 0.9),
            "high latency": lambda row: row["inference_telemetry"]["dosod"].__setitem__("latency_ms_p95", 1000.1),
            "missing product output": lambda row: row["inference_telemetry"]["product_output_counts"].__setitem__("/perception/ground_dirt/masks", 0),
            "empty product output": lambda row: row["inference_telemetry"]["product_nonempty_counts"].__setitem__("/perception/ground_dirt/masks", 0),
            "short duration": lambda row: row["sustained_run"].__setitem__("duration_sec", 1799.0),
            "high thermal": lambda row: row["thermal"].__setitem__("peak_celsius", 85.1),
            "low memory": lambda row: row["resources"].__setitem__("system_memory_available_min_bytes", 1),
            "restart": lambda row: row["sustained_run"].__setitem__("process_restarts", 1),
            "truth topic": lambda row: row["ros_graph"]["topics"].__setitem__("/evaluator/ground_truth", ["std_msgs/msg/String"]),
        }
        for label, mutate in mutations.items():
            with self.subTest(label=label):
                raw = self.valid_raw()
                mutate(raw)
                self.assertTrue(validate_raw(raw, self.identity))

    def test_rejects_snapshot_drift_and_missing_product_node(self) -> None:
        raw = self.valid_raw()
        raw["source_binding"]["source_inventory_sha256"] = "f" * 64
        raw["ros_graph"]["nodes"].remove("/open_vocab_product_adapter")
        failures = validate_raw(raw, self.identity)
        self.assertTrue(any("source binding" in row for row in failures))
        self.assertTrue(any("required ROS nodes" in row for row in failures))

    def test_final_gate_rejects_hbm_hash_without_compile_parity_metric_chain(self) -> None:
        raw = self.valid_raw()
        del raw["dosod_hbm_evidence"]
        failures = validate_raw(raw, self.identity, self.active_session, self.closure_binding)
        self.assertTrue(any("compile/parity/metric" in row for row in failures))

    def test_collector_refuses_this_non_s100_host_and_writes_blocked_artifact(self) -> None:
        output = self.root / "blocked.json"
        result = subprocess.run(
            [sys.executable, str(ROOT / "scripts/collect_formal_s100_live_runtime.py"), "--output", str(output)],
            capture_output=True,
            text=True,
            check=False,
        )
        self.assertEqual(result.returncode, 4)
        payload = json.loads(output.read_text(encoding="utf-8"))
        self.assertEqual(payload["status"], RAW_BLOCKED)
        self.assertFalse(payload["collection_complete"])
        self.assertFalse(payload["hardware"]["attested"])
        self.assertEqual(validate_schema(payload, RAW_SCHEMA), [])

    def test_collector_and_validator_refuse_to_overwrite_retained_evidence(self) -> None:
        output = self.root / "retained.json"
        output.write_text('{"retained": true}\n', encoding="utf-8")
        before = output.read_bytes()
        collector = subprocess.run(
            [sys.executable, str(ROOT / "scripts/collect_formal_s100_live_runtime.py"), "--output", str(output)],
            capture_output=True,
            text=True,
            check=False,
        )
        self.assertEqual(collector.returncode, 2)
        self.assertEqual(output.read_bytes(), before)

        raw = self.valid_raw()
        self.raw_path.write_text(json.dumps(raw), encoding="utf-8")
        validator = subprocess.run(
            [
                sys.executable,
                str(ROOT / "scripts/validate_formal_s100_live_runtime.py"),
                "--raw",
                str(self.raw_path),
                "--snapshot",
                str(self.snapshot),
                "--acceptance-session",
                str(self.session),
                "--runtime-closure",
                str(self.runtime_closure),
                "--dosod-admission-bundle",
                str(self.root),
                "--output",
                str(output),
            ],
            capture_output=True,
            text=True,
            check=False,
        )
        self.assertEqual(validator.returncode, 2)
        self.assertEqual(output.read_bytes(), before)

    def test_contract_wires_schema_collector_validator_and_session_binding(self) -> None:
        contract = yaml.safe_load(
            (ROOT / "config/high_fidelity_vehicle/formal_functional_acceptance_contract.yaml").read_text(encoding="utf-8")
        )
        gate = contract["evidence_gates"]["s100_live_runtime"]
        self.assertEqual(gate["report_id"], "tzcup_formal_rdk_s100_live_product_runtime_v1")
        self.assertTrue(gate["session_bound"])
        self.assertTrue(gate["required_values"]["passed"])
        self.assertTrue(gate["required_values"]["checks.acceptance_session_binding"])
        self.assertTrue(gate["required_values"]["checks.runtime_closure_binding"])
        self.assertEqual(gate["required_values"]["blockers"], [])
        self.assertIn("acceptance_session_binding", gate["required_mapping_keys"])
        self.assertIn("runtime_closure_binding", gate["required_mapping_keys"])
        self.assertEqual(
            gate["snapshot_manifest_hash_field"],
            "source_binding.snapshot_manifest_sha256",
        )
        self.assertEqual(gate["snapshot_source_hash_field"], "source_binding.source_inventory_sha256")
        for field in ("raw_schema", "acceptance_schema", "collector", "offline_validator"):
            self.assertTrue((ROOT / gate[field]).is_file(), field)
        self.assertGreaterEqual(len(gate["required_physical_scope"]), 8)


if __name__ == "__main__":
    unittest.main()
