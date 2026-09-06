#!/usr/bin/env python3
"""Fail closed G4 evidence finalizer; never overwrites historical AUTO-05 evidence."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path


def under(path: Path, root: Path) -> bool:
    try:
        path.resolve().relative_to(root.resolve())
    except ValueError:
        return False
    return True


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read_object(path: Path) -> dict:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"JSON root is not an object: {path}")
    return value


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", required=True)
    parser.add_argument("--raw-root", required=True)
    parser.add_argument("--dataset-evidence", required=True)
    parser.add_argument("--g4-contract", required=True)
    parser.add_argument("--runtime-binding", required=True)
    parser.add_argument("--capture-receipt", required=True)
    parser.add_argument("--attempt-ledger", required=True)
    parser.add_argument("--test-lock", required=True)
    parser.add_argument("--cross-host-import")
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    repo = Path(args.repo).resolve()
    root = repo / ".work" / "auto05-g4"
    raw, dataset, contract, runtime_binding, capture_receipt, attempt_ledger, test_lock, output = map(
        lambda item: Path(item).resolve(),
        (args.raw_root, args.dataset_evidence, args.g4_contract,
         args.runtime_binding, args.capture_receipt, args.attempt_ledger, args.test_lock, args.output),
    )
    if not all(under(path, root) for path in (raw, dataset, runtime_binding, capture_receipt, attempt_ledger, test_lock, output)):
        parser.error("G4 inputs and output must remain below TZcup/.work/auto05-g4")
    if contract != (repo / "starter_ws/src/sanitation_learning/config/auto05_g4_screening.yaml").resolve():
        parser.error("G4 requires the checked-in frozen contract")
    report_path = raw / "auto05_screening_report.json"
    cross_host_import = Path(args.cross_host_import).resolve() if args.cross_host_import else None
    if cross_host_import and (not under(cross_host_import, root) or not cross_host_import.is_file() or cross_host_import.is_symlink()):
        parser.error("G4 cross-host import marker is not admissible")
    required = [
        report_path, raw / "auto05_direct_detector.onnx",
        raw / "auto05_rgbd_area_segmenter.onnx", raw / "environment.json", capture_receipt, attempt_ledger,
        dataset / "g3_dataset_qa.json", dataset / "split_manifest.json",
        dataset / "leakage_report.json", dataset / "g3_frame_manifest.jsonl",
    ]
    missing = [str(path) for path in required if not path.is_file()]
    if missing:
        parser.error("missing G4 evidence: " + ", ".join(missing))
    if any(path.is_symlink() for path in required) or contract.is_symlink():
        parser.error("G4 finalization rejects symbolic-link evidence")
    if output.exists():
        parser.error("refusing to overwrite retained G4 finalization")
    report = read_object(report_path)
    if report.get("attempt_id") != "AUTO-05-G3-SCREENING-V4":
        parser.error("only the pre-registered G4/V4 report may be finalized")
    selection = report.get("selection_policy", {})
    if selection.get("test_used_for_model_selection") is not False:
        parser.error("test-driven G4 selection is forbidden")
    if report.get("detector", {}).get("g4_quality_head_used") is not True:
        parser.error("G4 finalizer requires the preregistered detector quality head")
    if "independently parameterized" not in report.get("area_segmenter", {}).get("architecture", ""):
        parser.error("G4 finalizer requires independently parameterized area heads")
    if not isinstance(selection.get("g4_frozen_training"), dict):
        parser.error("G4 finalizer requires frozen training parameters in the report")
    dataset_report = read_object(dataset / "g3_dataset_qa.json")
    split = read_object(dataset / "split_manifest.json")
    leakage = read_object(dataset / "leakage_report.json")
    gate = read_object(runtime_binding)
    capture = read_object(capture_receipt)
    ledger = read_object(attempt_ledger)
    lock = read_object(test_lock)
    if dataset_report.get("dataset_gate_pass") is not True or split.get("test_used_for_model_selection") is not False or any(leakage.values()):
        parser.error("G4 QA, split, or leakage evidence is not admissible")
    if gate.get("status") != "AUTO05_G4_RUNTIME_GATE_BOUND":
        parser.error("G4 runtime binding was not formally bound")
    formal_gate = gate.get("formal_runtime_gate", {})
    if (
        formal_gate.get("status") != "FORMAL_RUNTIME_GATE_BOUND"
        or formal_gate.get("acceptance_session_binding", {}).get("session_status_at_gate") != "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING"
        or formal_gate.get("runtime_closure_binding", {}).get("status") != "FORMAL_FINAL_RUNTIME_CLOSURE_VERIFIED"
        or not gate.get("capture", {}).get("single_gazebo_lock")
    ):
        parser.error("G4 finalizer requires formal closure/session/single-lock proof")
    expected_data_root = (root / "data" / "g3_screening_native").relative_to(repo).as_posix()
    provenance = capture.get("capture_provenance", {})
    if (
        capture.get("status") != "AUTO05_G4_CAPTURE_COMPLETE"
        or capture.get("data_root_repository_relative") != expected_data_root
        or provenance.get("mode") != "fresh_native_gazebo_g3_capture"
        or provenance.get("replay_input_used") is not False
        or provenance.get("synthetic_substitution_used") is not False
    ):
        parser.error("G4 capture receipt is not bound to this raw data root")
    if capture.get("runtime_binding_sha256") != digest(runtime_binding):
        parser.error("G4 capture receipt does not bind the formal runtime gate")
    if report.get("implementation_commit") != gate.get("git", {}).get("head"):
        parser.error("report and runtime binding disagree on implementation commit")
    if report.get("implementation_tree") != gate.get("git", {}).get("tree"):
        parser.error("report and runtime binding disagree on implementation tree")
    if selection.get("g4_contract_sha256") != digest(contract) or gate.get("contract", {}).get("sha256") != digest(contract):
        parser.error("report/runtime binding disagree on frozen G4 contract")
    if (
        lock.get("status") != "G4_TEST_CONSUMED"
        or lock.get("output_repository_relative") != raw.relative_to(repo).as_posix()
    ):
        parser.error("G4 test lock does not bind this raw output")
    if lock.get("contract_sha256") != digest(contract) or lock.get("implementation_commit") != report.get("implementation_commit"):
        parser.error("G4 test lock identity mismatch")
    if (
        ledger.get("status") != "G4_ATTEMPT_RESERVED"
        or ledger.get("configuration_count") != 1
        or ledger.get("test_runs_allowed") != 1
        or ledger.get("implementation_commit") != report.get("implementation_commit")
        or ledger.get("implementation_tree") != report.get("implementation_tree")
        or ledger.get("contract_sha256") != digest(contract)
    ):
        parser.error("G4 attempt ledger does not prove the single preregistered configuration")
    actual_dataset_binding = {
        "data_root_repository_relative": (root / "data" / "g3_screening_native").relative_to(repo).as_posix(),
        "files": {
            name: digest(dataset / name)
            for name in ("g3_dataset_qa.json", "split_manifest.json", "leakage_report.json", "g3_frame_manifest.jsonl")
        },
    }
    report_dataset_binding = selection.get("g4_dataset_binding")
    if report_dataset_binding != actual_dataset_binding or lock.get("dataset_binding") != actual_dataset_binding:
        parser.error("G4 report/test lock does not bind the finalized dataset evidence")
    if ledger.get("dataset_binding") != actual_dataset_binding:
        parser.error("G4 attempt ledger does not bind the finalized dataset evidence")
    if cross_host_import:
        marker = read_object(cross_host_import)
        import_binding = {
            "handoff_receipt_sha256": marker.get("handoff_receipt_sha256"),
            "inventory_sha256": marker.get("inventory_sha256"),
        }
        if (
            marker.get("status") != "AUTO05_G4_CROSS_HOST_IMPORTED"
            or not all(isinstance(value, str) and len(value) == 64 for value in import_binding.values())
            or selection.get("g4_dataset_binding", {}).get("cross_host_import") != import_binding
            or lock.get("dataset_binding", {}).get("cross_host_import") != import_binding
            or ledger.get("dataset_binding", {}).get("cross_host_import") != import_binding
        ):
            parser.error("G4 finalizer cross-host import binding mismatch")
    runtime_digest = digest(runtime_binding)
    if selection.get("g4_runtime_binding", {}).get("git") != gate.get("git"):
        parser.error("G4 report does not preserve its formal runtime binding")
    if selection.get("g4_test_lock_sha256") != digest(test_lock) or lock.get("runtime_binding_sha256") != runtime_digest:
        parser.error("G4 report/test lock runtime evidence hash mismatch")
    if selection.get("g4_attempt_ledger_sha256") != digest(attempt_ledger) or lock.get("attempt_ledger_sha256") != digest(attempt_ledger) or ledger.get("runtime_binding_sha256") != runtime_digest:
        parser.error("G4 attempt ledger runtime evidence hash mismatch")
    for model in ("detector", "area_segmenter"):
        onnx = report.get(model, {}).get("onnx", {})
        filename = onnx.get("path")
        if not isinstance(filename, str) or onnx.get("sha256") != digest(raw / filename):
            parser.error(f"G4 {model} ONNX hash mismatch")
    status = "PENDING_REVIEW" if (
        dataset_report.get("dataset_gate_pass")
        and report.get("auto05_screening_gate_pass")
    ) else "BLOCKED"
    output.mkdir(parents=True, exist_ok=False)
    (output / "g4_finalization.json").write_text(json.dumps({
        "schema_version": 1, "stage": "AUTO-05", "attempt": "G4",
        "status": status, "historical_evidence_overwritten": False,
        "auto06_authorized": False,
        "reason": "manual acceptance review required even after a machine pass",
        "bindings": {
            "implementation_commit": report.get("implementation_commit"),
            "implementation_tree": report.get("implementation_tree"),
            "contract_sha256": digest(contract),
            "dataset_evidence": actual_dataset_binding,
            "runtime_binding_sha256": runtime_digest,
            "capture_receipt_sha256": digest(capture_receipt),
            "attempt_ledger_sha256": digest(attempt_ledger),
            "test_lock_sha256": digest(test_lock),
            "cross_host_import": import_binding if cross_host_import else None,
            "detector_onnx_sha256": report["detector"]["onnx"]["sha256"],
            "area_onnx_sha256": report["area_segmenter"]["onnx"]["sha256"],
        },
    }, indent=2) + "\n", encoding="utf-8")
    return 0 if status == "PENDING_REVIEW" else 2


if __name__ == "__main__":
    raise SystemExit(main())
