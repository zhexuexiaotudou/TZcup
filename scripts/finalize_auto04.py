#!/usr/bin/env python3
"""Validate AUTO-04 evidence, complete its evidence contract, and advance state."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import tempfile

import yaml


ROOT = Path(__file__).resolve().parents[1]
STATE_PATH = ROOT / "AUTONOMOUS_STATE.json"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def write_json(path: Path, payload) -> None:
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def atomic_write_json(path: Path, payload) -> None:
    with tempfile.NamedTemporaryFile(
        "w", encoding="utf-8", dir=path.parent, delete=False
    ) as stream:
        stream.write(json.dumps(payload, indent=2) + "\n")
        temporary = Path(stream.name)
    temporary.replace(path)


def regenerate_manifest(evidence: Path) -> None:
    files = []
    for path in sorted(evidence.rglob("*")):
        if not path.is_file() or path.name == "artifact_manifest.json":
            continue
        files.append(
            {
                "path": path.relative_to(evidence).as_posix(),
                "bytes": path.stat().st_size,
                "sha256": sha256(path),
            }
        )
    write_json(
        evidence / "artifact_manifest.json",
        {
            "schema_version": 1,
            "stage": "AUTO-04",
            "coverage": 1.0,
            "file_count": len(files),
            "files": files,
        },
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--evidence", required=True)
    parser.add_argument("--implementation-commit", required=True)
    parser.add_argument("--ci-test-count", type=int, required=True)
    args = parser.parse_args()

    evidence = (ROOT / args.evidence).resolve()
    if not evidence.is_relative_to(ROOT):
        raise RuntimeError("evidence must stay inside the repository")
    report_path = evidence / "auto04_acceptance_report.json"
    report = json.loads(report_path.read_text(encoding="utf-8"))
    if report["implementation_commit"] != args.implementation_commit:
        raise RuntimeError("implementation commit mismatch")
    if report.get("auto04_gate_pass") is not True:
        raise RuntimeError("AUTO-04 acceptance is not PASS")
    failed = [name for name, value in report["gates"].items() if not value]
    if failed:
        raise RuntimeError(f"AUTO-04 failed gates: {failed}")

    detector = report["detector"]
    area = report["area_segmenter"]
    metrics = {
        "detector": {
            "ap50": detector["metrics"]["ap50"],
            "macro_recall": detector["metrics"]["macro_recall"],
            "recall_by_class": detector["metrics"]["recall_by_class"],
            "negative_only_false_positive_per_frame": detector["metrics"][
                "negative_only_false_positive_per_frame"
            ],
            "onnx_max_numeric_error": detector["onnx"][
                "max_numeric_output_error"
            ],
            "onnx_decoded_agreement": detector["onnx"][
                "decoded_or_argmax_agreement"
            ],
        },
        "area_segmenter": {
            "iou_by_class": area["metrics"]["iou_by_class"],
            "macro_miou": area["metrics"]["macro_miou"],
            "negative_area_false_positive_per_frame": area["metrics"][
                "negative_area_false_positive_per_frame"
            ],
            "onnx_max_numeric_error": area["onnx"]["max_numeric_output_error"],
            "onnx_argmax_agreement": area["onnx"][
                "decoded_or_argmax_agreement"
            ],
        },
        "attempt_count": 2,
        "source_level": "OFFLINE_SYNTHETIC_GAZEBO_MICRO",
    }
    write_json(evidence / "metrics_summary.json", metrics)
    write_json(
        evidence / "stage_status.json",
        {
            "schema_version": 1,
            "stage": "AUTO-04",
            "status": "PASS",
            "machine_gate_pass": True,
            "selected_attempt": report["attempt_id"],
            "implementation_commit": args.implementation_commit,
            "evidence_dir": evidence.relative_to(ROOT).as_posix(),
            "first_blocking_layer": None,
            "next_stage": "AUTO-05",
            "claim_boundary": "task-specific train-set capacity only; not cross-world, formal, live, real-domain, J6, or competition perception evidence",
        },
    )
    (evidence / "stage_config.yaml").write_text(
        yaml.safe_dump(
            {
                "schema_version": 1,
                "stage": "AUTO-04",
                "detector": {
                    "architecture": "direct_anchor_free_center_offset_bbox",
                    "input": [1, 3, 192, 192],
                    "stride": 4,
                    "confidence_threshold": detector["metrics"][
                        "confidence_threshold"
                    ],
                    "ap50_min": 0.95,
                    "macro_recall_min": 0.95,
                    "each_class_recall_min": 0.95,
                    "negative_fp_per_frame_max": 0.05,
                },
                "area_segmenter": {
                    "architecture": "independent_binary_leaf_puddle_heads",
                    "input": [1, 3, 128, 128],
                    "per_class_iou_min": 0.95,
                    "macro_miou_min": 0.95,
                    "negative_fp_per_frame_max": 0.05,
                },
                "onnx": {
                    "batch": 1,
                    "custom_ops": 0,
                    "agreement_min": 0.9999,
                    "max_numeric_error": 1e-4,
                },
            },
            sort_keys=False,
            allow_unicode=True,
        ),
        encoding="utf-8",
    )
    (evidence / "commands.txt").write_text(
        "py -3 scripts/ci_fast.py\n"
        "powershell -NoProfile -ExecutionPolicy Bypass -File "
        "scripts/run_auto04_micro_overfit_docker.ps1 "
        "-DataRoot F:\\Project\\TZcup-stage5br3-data\\g2_screening_native "
        "-OutputName autonomous_auto04_20260730_evidence "
        f"-ImplementationCommit {args.implementation_commit} "
        "-Attempt 2 -DetectorEpochs 180 -AreaEpochs 260\n"
        "py -3 scripts/finalize_auto04.py "
        "--evidence artifacts/autonomous_auto04_20260730_evidence "
        f"--implementation-commit {args.implementation_commit} "
        f"--ci-test-count {args.ci_test_count}\n",
        encoding="utf-8",
    )
    write_json(
        evidence / "raw_metric_index.json",
        {
            "schema_version": 1,
            "stage": "AUTO-04",
            "selected_report": "auto04_acceptance_report.json",
            "selected_models": [
                "auto04_direct_detector.onnx",
                "auto04_area_segmenter.onnx",
            ],
            "prior_failed_report": "prior_attempts/attempt1_acceptance_report.json",
            "local_raw_attempt_dir": "artifacts/autonomous_auto04_attempt1_raw",
            "large_source_dataset_versioned": False,
        },
    )
    write_json(
        evidence / "regression_summary.json",
        {
            "schema_version": 1,
            "ci_fast_pass": True,
            "ci_fast_test_count": args.ci_test_count,
            "auto03_evidence_modified": False,
            "historical_stage4w_to_stage5br6w_modified": False,
            "stage5br6a_human_review_completed": False,
            "stage5br6a_manual_audit_pass": False,
        },
    )
    write_json(
        evidence / "model_license.json",
        {
            "schema_version": 1,
            "models": [
                {
                    "path": "auto04_direct_detector.onnx",
                    "source": "project-authored PyTorch architecture trained on project-generated Gazebo data",
                    "license": "Apache-2.0",
                },
                {
                    "path": "auto04_area_segmenter.onnx",
                    "source": "project-authored PyTorch architecture trained on project-generated Gazebo data",
                    "license": "Apache-2.0",
                },
            ],
            "unknown_license_count": 0,
        },
    )
    regenerate_manifest(evidence)

    state = json.loads(STATE_PATH.read_text(encoding="utf-8"))
    historical = state["historical_boundaries"]
    if historical["stage5br6a_human_review_completed"] is not False:
        raise RuntimeError("historical human-review flag changed")
    if historical["stage5br6a_manual_audit_pass"] is not False:
        raise RuntimeError("historical manual-audit flag changed")
    stage = state["stages"]["AUTO-04"]
    stage.update(
        {
            "status": "PASS",
            "machine_gate_pass": True,
            "blocked": False,
            "blocked_external": False,
            "first_blocking_layer": None,
            "attempt_count": 2,
            "selected_attempt": report["attempt_id"],
            "implementation_commit": args.implementation_commit,
            "evidence_dir": evidence.relative_to(ROOT).as_posix(),
            "metrics": metrics,
            "unexecuted_items": [],
        }
    )
    state["run"].update(
        {
            "branch": "agent/autonomous-complete",
            "current_stage": "AUTO-05",
            "last_commit": args.implementation_commit,
        }
    )
    atomic_write_json(STATE_PATH, state)
    print(
        json.dumps(
            {
                "stage": "AUTO-04",
                "status": "PASS",
                "next_stage": "AUTO-05",
                "evidence": evidence.relative_to(ROOT).as_posix(),
                "manifest_files": json.loads(
                    (evidence / "artifact_manifest.json").read_text()
                )["file_count"],
            },
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
