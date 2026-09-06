#!/usr/bin/env python3
"""End-to-end fail-closed regression for G4 host/container evidence identity."""

from __future__ import annotations

import hashlib
import json
import shutil
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_json(path: Path, value: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def test_finalizer_accepts_container_safe_repository_relative_binding(tmp_path: Path) -> None:
    """`/repo` container paths must normalize to the host repository identity."""
    repo = tmp_path / "TZcup"
    contract = repo / "starter_ws/src/sanitation_learning/config/auto05_g4_screening.yaml"
    contract.parent.mkdir(parents=True)
    shutil.copyfile(
        ROOT / "starter_ws/src/sanitation_learning/config/auto05_g4_screening.yaml",
        contract,
    )
    root = repo / ".work/auto05-g4"
    raw, dataset, evidence = root / "evidence/screening", root / "evidence/dataset", root / "evidence"
    data = root / "data/g3_screening_native"
    raw.mkdir(parents=True)
    data.mkdir(parents=True)
    for filename, contents in (
        ("auto05_direct_detector.onnx", b"detector"),
        ("auto05_rgbd_area_segmenter.onnx", b"area"),
        ("environment.json", b"{}\n"),
    ):
        (raw / filename).write_bytes(contents)
    write_json(dataset / "g3_dataset_qa.json", {"dataset_gate_pass": True})
    write_json(dataset / "split_manifest.json", {"test_used_for_model_selection": False})
    write_json(dataset / "leakage_report.json", {})
    (dataset / "g3_frame_manifest.jsonl").write_text("{}\n", encoding="utf-8")
    dataset_binding = {
        "data_root_repository_relative": ".work/auto05-g4/data/g3_screening_native",
        "files": {name: sha256(dataset / name) for name in (
            "g3_dataset_qa.json", "split_manifest.json", "leakage_report.json", "g3_frame_manifest.jsonl",
        )},
    }
    implementation = {"head": "a" * 40, "tree": "b" * 40}
    runtime = {
        "status": "AUTO05_G4_RUNTIME_GATE_BOUND", "git": implementation,
        "contract": {"repository_relative": "starter_ws/src/sanitation_learning/config/auto05_g4_screening.yaml", "sha256": sha256(contract)},
        "capture": {"data_root_repository_relative": ".work/auto05-g4/data/g3_screening_native", "single_gazebo_lock": "/tmp/tzcup_formal_gazebo.lock"},
        "formal_runtime_gate": {
            "status": "FORMAL_RUNTIME_GATE_BOUND",
            "acceptance_session_binding": {"session_status_at_gate": "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING"},
            "runtime_closure_binding": {"status": "FORMAL_FINAL_RUNTIME_CLOSURE_VERIFIED"},
        },
    }
    runtime_path = evidence / "runtime_gate_binding.json"
    write_json(runtime_path, runtime)
    write_json(evidence / "capture_complete.json", {
        "status": "AUTO05_G4_CAPTURE_COMPLETE", "data_root_repository_relative": ".work/auto05-g4/data/g3_screening_native",
        "capture_provenance": {"mode": "fresh_native_gazebo_g3_capture", "replay_input_used": False, "synthetic_substitution_used": False},
        "runtime_binding_sha256": sha256(runtime_path),
    })
    ledger_path = evidence / "g4_attempt_ledger.json"
    write_json(ledger_path, {
        "status": "G4_ATTEMPT_RESERVED", "configuration_count": 1, "test_runs_allowed": 1,
        "implementation_commit": implementation["head"], "implementation_tree": implementation["tree"],
        "contract_sha256": sha256(contract), "dataset_binding": dataset_binding,
        "runtime_binding_sha256": sha256(runtime_path),
    })
    lock_path = evidence / "g4_test_consumed_lock.json"
    write_json(lock_path, {
        "status": "G4_TEST_CONSUMED", "implementation_commit": implementation["head"],
        "contract_sha256": sha256(contract), "dataset_binding": dataset_binding,
        "runtime_binding_sha256": sha256(runtime_path), "attempt_ledger_sha256": sha256(ledger_path),
        "output_repository_relative": ".work/auto05-g4/evidence/screening",
    })
    write_json(raw / "auto05_screening_report.json", {
        "attempt_id": "AUTO-05-G3-SCREENING-V4", "implementation_commit": implementation["head"],
        "implementation_tree": implementation["tree"], "auto05_screening_gate_pass": False,
        "selection_policy": {
            "test_used_for_model_selection": False, "g4_contract_sha256": sha256(contract),
            "g4_frozen_training": {"seed": 20260730}, "g4_dataset_binding": dataset_binding,
            "g4_runtime_binding": runtime, "g4_attempt_ledger_sha256": sha256(ledger_path),
            "g4_test_lock_sha256": sha256(lock_path),
        },
        "detector": {"g4_quality_head_used": True, "onnx": {"path": "auto05_direct_detector.onnx", "sha256": sha256(raw / "auto05_direct_detector.onnx")}},
        "area_segmenter": {"architecture": "G4 independently parameterized binary heads", "onnx": {"path": "auto05_rgbd_area_segmenter.onnx", "sha256": sha256(raw / "auto05_rgbd_area_segmenter.onnx")}},
    })
    result = subprocess.run([
        sys.executable, str(ROOT / "scripts/finalize_auto05_g4.py"),
        "--repo", str(repo), "--raw-root", str(raw), "--dataset-evidence", str(dataset),
        "--g4-contract", str(contract), "--runtime-binding", str(runtime_path),
        "--capture-receipt", str(evidence / "capture_complete.json"),
        "--attempt-ledger", str(ledger_path), "--test-lock", str(lock_path),
        "--output", str(evidence / "finalization"),
    ], capture_output=True, text=True)
    assert result.returncode == 2, result.stderr
    final = json.loads((evidence / "finalization/g4_finalization.json").read_text(encoding="utf-8"))
    assert final["status"] == "BLOCKED"
    assert final["bindings"]["dataset_evidence"] == dataset_binding
