#!/usr/bin/env python3
"""Fail-closed regression gate for actual ONNX-vs-HBM perception metrics.

It consumes two independently retained evaluator reports over the *same*
frozen non-calibration holdout.  It does not infer labels, run a model, or
reinterpret historical AUTO-05 screening as HBM evidence.  This separation
prevents a model-export success from being misreported as metric acceptance.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path
from typing import Any

from hbm_evidence_common import atomic_json, load_object, normal_file, sha256_file


REPORT_ID = "tzcup_dosod_quantized_metric_regression_v1"
METRICS_REPORT_ID = "tzcup_dosod_perception_validation_metrics_v1"
THRESHOLDS_REPORT_ID = "tzcup_dosod_quantized_metric_thresholds_v1"
CLASS_ORDER = ["litter_cube", "fallen_leaves", "dust_or_soil", "puddle"]


def _digest(value: Any) -> bool:
    return isinstance(value, str) and len(value) == 64 and all(c in "0123456789abcdef" for c in value.lower())


def _block(report: dict[str, Any], value: str) -> None:
    if value not in report["blockers"]:
        report["blockers"].append(value)


def _number(value: Any, label: str) -> float:
    if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(float(value)):
        raise ValueError(f"metric_not_finite:{label}")
    return float(value)


def _metric(report: dict[str, Any], key: str) -> float:
    current: Any = report.get("metrics")
    for item in key.split("."):
        if not isinstance(current, dict) or item not in current:
            raise ValueError(f"metric_missing:{key}")
        current = current[item]
    return _number(current, key)


def _sources(report: dict[str, Any]) -> set[str]:
    dataset = report.get("dataset")
    if not isinstance(dataset, dict) or dataset.get("status") != "FROZEN" or not isinstance(dataset.get("dataset_id"), str) or not dataset["dataset_id"]:
        raise ValueError("validation_dataset_not_frozen")
    values = dataset.get("source_sha256")
    if not isinstance(values, list) or not values or not all(_digest(value) for value in values):
        raise ValueError("validation_dataset_source_sha_invalid")
    lowered = {str(value).lower() for value in values}
    if len(lowered) != len(values):
        raise ValueError("validation_dataset_duplicate_source_sha")
    return lowered


def _validate_metrics_report(report: dict[str, Any], *, expected_backend: str, hbm_sha: str | None) -> set[str]:
    if report.get("report_id") != METRICS_REPORT_ID or report.get("status") != "VERIFIED":
        raise ValueError("validation_metrics_report_not_verified")
    execution = report.get("execution")
    if not isinstance(execution, dict) or execution.get("backend") != expected_backend:
        raise ValueError("validation_metrics_backend_mismatch")
    if not isinstance(execution.get("command"), list) or not execution["command"] or execution.get("returncode") != 0:
        raise ValueError("validation_metrics_execution_not_retained")
    if expected_backend == "hbm":
        if execution.get("hbm_sha256") != hbm_sha:
            raise ValueError("validation_metrics_hbm_identity_mismatch")
    if report.get("class_order") != CLASS_ORDER:
        raise ValueError("validation_metrics_class_order_mismatch")
    return _sources(report)


def _calibration_sources(path: Path) -> set[str]:
    manifest = load_object(path)
    records = manifest.get("records")
    if manifest.get("status") != "FROZEN" or not isinstance(records, list):
        raise ValueError("calibration_manifest_not_frozen")
    values = {row.get("source_sha256") for row in records if isinstance(row, dict)}
    if not values or not all(_digest(value) for value in values):
        raise ValueError("calibration_manifest_source_sha_invalid")
    return {str(value).lower() for value in values}


def _validate_thresholds(value: dict[str, Any]) -> tuple[dict[str, float], dict[str, float], dict[str, float]]:
    if value.get("report_id") != THRESHOLDS_REPORT_ID or value.get("schema_version") != 1:
        raise ValueError("thresholds_report_identity_invalid")
    groups: list[dict[str, float]] = []
    for name in ("minimum", "maximum", "max_drop"):
        group = value.get(name, {})
        if not isinstance(group, dict):
            raise ValueError(f"thresholds_{name}_not_object")
        parsed = {str(key): _number(item, f"threshold:{name}:{key}") for key, item in group.items()}
        if any(number < 0 for number in parsed.values()):
            raise ValueError(f"thresholds_{name}_negative")
        groups.append(parsed)
    if not any(groups):
        raise ValueError("thresholds_empty")
    return tuple(groups)  # type: ignore[return-value]


def validate_regression(
    *, hbm: Path, compile_receipt_path: Path, calibration_manifest: Path,
    reference_report_path: Path, quantized_report_path: Path,
    thresholds_path: Path, output: Path,
) -> dict[str, Any]:
    for path, label in ((hbm, "hbm"), (compile_receipt_path, "compile_receipt"), (calibration_manifest, "calibration_manifest"),
                        (reference_report_path, "reference_metrics"), (quantized_report_path, "quantized_metrics"), (thresholds_path, "thresholds")):
        normal_file(path, label)
    if output.exists() and any(output.iterdir()):
        raise ValueError("evidence_output_must_be_empty")
    hbm_sha = sha256_file(hbm)
    report: dict[str, Any] = {
        "schema_version": 1, "report_id": REPORT_ID, "status": "BLOCKED",
        "claim_boundary": "metric comparison only; a pass does not claim board deployment or 1800-second runtime acceptance",
        "hbm": {"path": str(hbm.resolve()), "sha256": hbm_sha, "byte_size": hbm.stat().st_size},
        "checks": {}, "blockers": [], "started_epoch_ns": time.time_ns(), "ended_epoch_ns": None,
    }
    try:
        compile_receipt = load_object(compile_receipt_path)
        if compile_receipt.get("status") != "COMPILED_NOT_BOARD_ACCEPTED" or compile_receipt.get("returncode") != 0:
            raise ValueError("compile_receipt_not_successful")
        if compile_receipt.get("output_sha256") != hbm_sha or compile_receipt.get("output_byte_size") != hbm.stat().st_size:
            raise ValueError("compile_receipt_hbm_identity_mismatch")
        reference = load_object(reference_report_path)
        quantized = load_object(quantized_report_path)
        thresholds = load_object(thresholds_path)
        calibration_sources = _calibration_sources(calibration_manifest)
        reference_sources = _validate_metrics_report(reference, expected_backend="onnx", hbm_sha=None)
        quantized_sources = _validate_metrics_report(quantized, expected_backend="hbm", hbm_sha=hbm_sha)
        if reference_sources != quantized_sources:
            raise ValueError("reference_quantized_holdout_identity_mismatch")
        if reference_sources & calibration_sources:
            raise ValueError("validation_calibration_source_overlap")
        minimum, maximum, max_drop = _validate_thresholds(thresholds)
        report.update({"compile_receipt_sha256": sha256_file(compile_receipt_path),
                       "reference_report_sha256": sha256_file(reference_report_path),
                       "quantized_report_sha256": sha256_file(quantized_report_path),
                       "thresholds_sha256": sha256_file(thresholds_path),
                       "holdout_source_count": len(reference_sources)})
        checks: dict[str, dict[str, Any]] = {}
        for key, limit in minimum.items():
            value = _metric(quantized, key)
            checks[f"minimum:{key}"] = {"value": value, "limit": limit, "pass": value >= limit}
        for key, limit in maximum.items():
            value = _metric(quantized, key)
            checks[f"maximum:{key}"] = {"value": value, "limit": limit, "pass": value <= limit}
        for key, limit in max_drop.items():
            reference_value, quantized_value = _metric(reference, key), _metric(quantized, key)
            drop = reference_value - quantized_value
            checks[f"max_drop:{key}"] = {"reference": reference_value, "quantized": quantized_value,
                                            "drop": drop, "limit": limit, "pass": drop <= limit}
        report["checks"] = checks
        if checks and all(item["pass"] for item in checks.values()):
            report["status"] = "REGRESSION_PASSED"
        else:
            _block(report, "quantized_metric_threshold_failed")
    except Exception as exc:
        _block(report, f"metric_regression_precondition_or_validation_failed:{type(exc).__name__}")
    finally:
        report["ended_epoch_ns"] = time.time_ns()
        atomic_json(output / "dosod_quantized_metric_regression.json", report)
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--hbm", required=True, type=Path)
    parser.add_argument("--compile-receipt", required=True, type=Path)
    parser.add_argument("--calibration-manifest", required=True, type=Path)
    parser.add_argument("--reference-report", required=True, type=Path)
    parser.add_argument("--quantized-report", required=True, type=Path)
    parser.add_argument("--thresholds", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    try:
        report = validate_regression(hbm=args.hbm, compile_receipt_path=args.compile_receipt,
                                     calibration_manifest=args.calibration_manifest,
                                     reference_report_path=args.reference_report,
                                     quantized_report_path=args.quantized_report,
                                     thresholds_path=args.thresholds, output=args.output)
    except Exception as exc:
        print(f"quantized_metric_regression_blocked:{type(exc).__name__}:{exc}", file=sys.stderr)
        return 2
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report["status"] == "REGRESSION_PASSED" else 2


if __name__ == "__main__":
    raise SystemExit(main())
