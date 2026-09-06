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

from hbm_evidence_common import atomic_json, fresh_directory, load_object, normal_file, sha256_file


REPORT_ID = "tzcup_dosod_quantized_metric_regression_v1"
METRICS_REPORT_ID = "tzcup_dosod_perception_validation_metrics_v1"
THRESHOLDS_REPORT_ID = "tzcup_dosod_quantized_metric_thresholds_v1"
PARITY_REPORT_ID = "tzcup_dosod_hbm_x86_nash_parity_v1"
EVALUATOR_IDENTITY_ID = "tzcup_dosod_metric_evaluator_identity_v1"
COMPILE_RECEIPT_ID = "tzcup_s100p_dosod_hbm_compile_receipt_v1"
EVALUATOR_RECEIPT_ID = "tzcup_dosod_quantized_metric_evaluator_receipt_v1"
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


def _validate_metrics_report(
    report: dict[str, Any], *, expected_backend: str, hbm_sha: str | None,
    holdout_manifest_sha256: str, evaluator_identity_sha256: str, evaluator_binding: dict[str, Any], runner_identity_sha256: str | None,
) -> set[str]:
    if report.get("report_id") != METRICS_REPORT_ID or report.get("status") != "VERIFIED":
        raise ValueError("validation_metrics_report_not_verified")
    execution = report.get("execution")
    if not isinstance(execution, dict) or execution.get("backend") != expected_backend:
        raise ValueError("validation_metrics_backend_mismatch")
    if not isinstance(execution.get("command"), list) or not execution["command"] or execution.get("returncode") != 0:
        raise ValueError("validation_metrics_execution_not_retained")
    if execution.get("evaluator_identity_sha256") != evaluator_identity_sha256:
        raise ValueError("validation_metrics_evaluator_identity_mismatch")
    if execution.get("holdout_manifest_sha256") != holdout_manifest_sha256:
        raise ValueError("validation_metrics_holdout_manifest_mismatch")
    if execution["command"][0] != evaluator_binding["absolute_path"]:
        raise ValueError("validation_metrics_evaluator_path_mismatch")
    if execution.get("command_template") != evaluator_binding["command_template"]:
        raise ValueError("validation_metrics_command_template_mismatch")
    for stream in ("stdout", "stderr"):
        raw_path = execution.get(f"{stream}_path")
        if not isinstance(raw_path, str) or not Path(raw_path).is_absolute() or not _digest(execution.get(f"{stream}_sha256")):
            raise ValueError("validation_metrics_raw_output_identity_missing")
        candidate = Path(raw_path)
        normal_file(candidate, f"validation_metrics_{stream}")
        if sha256_file(candidate) != execution[f"{stream}_sha256"]:
            raise ValueError(f"validation_metrics_{stream}_identity_mismatch")
    if expected_backend == "hbm":
        if execution.get("hbm_sha256") != hbm_sha:
            raise ValueError("validation_metrics_hbm_identity_mismatch")
        if execution.get("runner_identity_sha256") != runner_identity_sha256:
            raise ValueError("validation_metrics_runner_identity_mismatch")
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


def _validate_parity_report(path: Path, *, hbm_sha: str, calibration_sha: str, holdout_sha: str) -> tuple[set[str], str]:
    parity = load_object(path)
    if parity.get("report_id") != PARITY_REPORT_ID or parity.get("status") != "PARITY_PASSED":
        raise ValueError("parity_report_not_passed")
    if parity.get("hbm", {}).get("sha256") != hbm_sha:
        raise ValueError("parity_report_hbm_identity_mismatch")
    if parity.get("calibration_manifest_sha256") != calibration_sha or parity.get("holdout_manifest_sha256") != holdout_sha:
        raise ValueError("parity_report_dataset_identity_mismatch")
    runner_sha = parity.get("runner_identity_sha256")
    if not _digest(runner_sha):
        raise ValueError("parity_report_runner_identity_missing")
    records = parity.get("records")
    if not isinstance(records, list) or not records or not all(isinstance(row, dict) and row.get("pass") is True and _digest(row.get("source_sha256")) for row in records):
        raise ValueError("parity_report_records_incomplete")
    sources = {str(row["source_sha256"]).lower() for row in records}
    if len(sources) != len(records):
        raise ValueError("parity_report_duplicate_sources")
    return sources, str(runner_sha)


def _validate_compile_receipt(path: Path, *, hbm: Path, calibration_manifest: Path) -> dict[str, Any]:
    """Consume only a structurally complete fresh-root compile producer receipt."""
    receipt = load_object(path)
    if receipt.get("receipt_id") != COMPILE_RECEIPT_ID or receipt.get("status") != "COMPILED_NOT_BOARD_ACCEPTED" or receipt.get("returncode") != 0:
        raise ValueError("compile_receipt_not_successful")
    if receipt.get("output_created_by_this_compile") is not True:
        raise ValueError("compile_receipt_does_not_prove_fresh_output")
    root = receipt.get("evidence_root")
    if not isinstance(root, str) or not Path(root).is_absolute() or Path(root).is_symlink() or Path(root).resolve() != path.parent.resolve():
        raise ValueError("compile_receipt_evidence_root_invalid")
    if receipt.get("receipt_path") != str(path.resolve()):
        raise ValueError("compile_receipt_path_binding_mismatch")
    for stream in ("stdout", "stderr"):
        name = receipt.get(f"raw_{stream}_path")
        digest = receipt.get(f"raw_{stream}_sha256")
        if not isinstance(name, str) or not _digest(digest):
            raise ValueError("compile_receipt_raw_output_identity_missing")
        candidate = (path.parent / name).resolve()
        if not candidate.is_relative_to(path.parent.resolve()):
            raise ValueError("compile_receipt_raw_output_path_escape")
        normal_file(candidate, f"compile_receipt_{stream}")
        if sha256_file(candidate) != digest:
            raise ValueError("compile_receipt_raw_output_identity_mismatch")
    inputs = receipt.get("inputs")
    if not isinstance(inputs, dict) or inputs.get("calibration_manifest_sha256") != sha256_file(calibration_manifest):
        raise ValueError("compile_receipt_calibration_binding_mismatch")
    if receipt.get("output_sha256") != sha256_file(hbm) or receipt.get("output_byte_size") != hbm.stat().st_size:
        raise ValueError("compile_receipt_hbm_identity_mismatch")
    return receipt


def _validate_evaluator_receipt(path: Path, *, backend: str, metrics_path: Path, holdout_sha: str) -> None:
    normal_file(path, "evaluator_receipt")
    value = load_object(path)
    if value.get("receipt_id") != EVALUATOR_RECEIPT_ID or value.get("status") != "EVALUATOR_EXECUTED" or value.get("backend") != backend:
        raise ValueError("evaluator_receipt_not_executed")
    if value.get("holdout_manifest_sha256") != holdout_sha or value.get("metrics_path") != str(metrics_path.resolve()) or value.get("metrics_sha256") != sha256_file(metrics_path):
        raise ValueError("evaluator_receipt_metrics_binding_mismatch")
    for stream in ("stdout", "stderr"):
        raw = value.get(f"{stream}_path")
        if not isinstance(raw, str) or not Path(raw).is_absolute() or not _digest(value.get(f"{stream}_sha256")):
            raise ValueError("evaluator_receipt_raw_output_missing")
        candidate = Path(raw); normal_file(candidate, f"evaluator_receipt_{stream}")
        if sha256_file(candidate) != value[f"{stream}_sha256"]:
            raise ValueError("evaluator_receipt_raw_output_mismatch")


def _validate_evaluator_identity(path: Path) -> tuple[str, dict[str, dict[str, Any]]]:
    identity = load_object(path)
    if identity.get("schema_version") != 1 or identity.get("report_id") != EVALUATOR_IDENTITY_ID or identity.get("status") != "VERIFIED":
        raise ValueError("evaluator_identity_not_verified")
    backends = identity.get("backends")
    if not isinstance(backends, dict) or set(backends) != {"onnx", "hbm"}:
        raise ValueError("evaluator_identity_backend_set_invalid")
    official = identity.get("official_compiler_identity")
    if not isinstance(official, dict) or not isinstance(official.get("path"), str) or not Path(official["path"]).is_absolute():
        raise ValueError("evaluator_identity_official_toolchain_missing")
    official_path = Path(official["path"])
    normal_file(official_path, "evaluator_official_compiler_identity")
    if official.get("sha256") != sha256_file(official_path):
        raise ValueError("evaluator_identity_official_toolchain_sha_mismatch")
    official_report = load_object(official_path)
    if official_report.get("report_id") != "tzcup_dosod_s100p_live_compiler_identity_v1" or official_report.get("identity_verified") is not True:
        raise ValueError("evaluator_identity_official_toolchain_unverified")
    bindings: dict[str, dict[str, Any]] = {}
    for backend, row in backends.items():
        if not isinstance(row, dict) or not isinstance(row.get("absolute_path"), str) or not Path(row["absolute_path"]).is_absolute():
            raise ValueError(f"evaluator_identity_path_invalid:{backend}")
        executable = Path(row["absolute_path"])
        normal_file(executable, f"evaluator_executable:{backend}")
        if row.get("sha256") != sha256_file(executable) or not isinstance(row.get("version"), str) or not row["version"].strip() or not isinstance(row.get("command_template"), list) or not row["command_template"]:
            raise ValueError(f"evaluator_identity_binding_invalid:{backend}")
        bindings[backend] = {"absolute_path": str(executable.resolve()), "command_template": row["command_template"]}
    return sha256_file(path), bindings


def _holdout_sources_and_adapter(path: Path) -> tuple[set[str], dict[str, Any]]:
    manifest = load_object(path)
    if manifest.get("schema_version") != 1 or manifest.get("status") != "FROZEN" or not isinstance(manifest.get("records"), list):
        raise ValueError("metric_holdout_manifest_not_frozen")
    adapter = manifest.get("hbm_input_adapter")
    if not isinstance(adapter, dict) or adapter.get("status") != "VERIFIED" or not isinstance(adapter.get("command"), list):
        raise ValueError("metric_holdout_adapter_unverified")
    values = {row.get("source_sha256") for row in manifest["records"] if isinstance(row, dict)}
    if not values or not all(_digest(value) for value in values):
        raise ValueError("metric_holdout_source_sha_invalid")
    lowered = {str(value).lower() for value in values}
    if len(lowered) != len(manifest["records"]):
        raise ValueError("metric_holdout_duplicate_or_invalid_records")
    return lowered, adapter


def _validate_runner_identity(path: Path, expected_sha: str, holdout_adapter: dict[str, Any]) -> None:
    """Bind metric acceptance to the runner identity used by passed parity."""

    identity = load_object(path)
    if sha256_file(path) != expected_sha:
        raise ValueError("metric_runner_identity_digest_mismatch")
    if identity.get("schema_version") != 1 or identity.get("report_id") != "tzcup_dosod_hbm_runner_identity_v1" or identity.get("status") != "VERIFIED":
        raise ValueError("metric_runner_identity_not_verified")
    runner = identity.get("runner")
    if not isinstance(runner, dict) or not isinstance(runner.get("absolute_path"), str) or not Path(runner["absolute_path"]).is_absolute():
        raise ValueError("metric_runner_identity_path_invalid")
    executable = Path(runner["absolute_path"])
    normal_file(executable, "metric_runner_executable")
    if runner.get("sha256") != sha256_file(executable) or not isinstance(runner.get("version"), str) or not runner["version"].strip():
        raise ValueError("metric_runner_identity_binding_invalid")
    if identity.get("command_template") != ["{runner}", "--model", "{hbm}", "--input", "{input}", "--output-path", "{output}"]:
        raise ValueError("metric_runner_identity_command_template_invalid")
    output_map = identity.get("output_map")
    if not isinstance(output_map, dict) or set(output_map) != {"scores", "boxes"} or not all(isinstance(value, str) and value.endswith(".npy") for value in output_map.values()):
        raise ValueError("metric_runner_identity_output_map_invalid")
    if identity.get("hbm_input_adapter") != holdout_adapter:
        raise ValueError("metric_runner_identity_adapter_mismatch")


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
    holdout_manifest: Path, parity_report_path: Path, runner_identity_path: Path, evaluator_identity_path: Path,
    reference_report_path: Path, quantized_report_path: Path,
    thresholds_path: Path, output: Path, reference_evaluator_receipt: Path | None = None, quantized_evaluator_receipt: Path | None = None,
) -> dict[str, Any]:
    for path, label in ((hbm, "hbm"), (compile_receipt_path, "compile_receipt"), (calibration_manifest, "calibration_manifest"),
                        (holdout_manifest, "holdout_manifest"), (parity_report_path, "parity_report"), (runner_identity_path, "runner_identity"), (evaluator_identity_path, "evaluator_identity"),
                        (reference_report_path, "reference_metrics"), (quantized_report_path, "quantized_metrics"), (thresholds_path, "thresholds")):
        normal_file(path, label)
    fresh_directory(output, "evidence_output")
    hbm_sha = sha256_file(hbm)
    report: dict[str, Any] = {
        "schema_version": 1, "report_id": REPORT_ID, "status": "BLOCKED",
        "claim_boundary": "metric comparison only; a pass does not claim board deployment or 1800-second runtime acceptance",
        "hbm": {"path": str(hbm.resolve()), "sha256": hbm_sha, "byte_size": hbm.stat().st_size},
        "checks": {}, "blockers": [], "started_epoch_ns": time.time_ns(), "ended_epoch_ns": None,
    }
    try:
        compile_receipt = _validate_compile_receipt(
            compile_receipt_path, hbm=hbm, calibration_manifest=calibration_manifest
        )
        reference = load_object(reference_report_path)
        quantized = load_object(quantized_report_path)
        thresholds = load_object(thresholds_path)
        calibration_sources = _calibration_sources(calibration_manifest)
        holdout_sha = sha256_file(holdout_manifest)
        if reference_evaluator_receipt is None or quantized_evaluator_receipt is None:
            raise ValueError("actual_evaluator_receipts_required")
        _validate_evaluator_receipt(reference_evaluator_receipt, backend="onnx", metrics_path=reference_report_path, holdout_sha=holdout_sha)
        _validate_evaluator_receipt(quantized_evaluator_receipt, backend="hbm", metrics_path=quantized_report_path, holdout_sha=holdout_sha)
        holdout_sources, holdout_adapter = _holdout_sources_and_adapter(holdout_manifest)
        parity_sources, runner_identity_sha256 = _validate_parity_report(
            parity_report_path, hbm_sha=hbm_sha, calibration_sha=sha256_file(calibration_manifest), holdout_sha=holdout_sha
        )
        _validate_runner_identity(runner_identity_path, runner_identity_sha256, holdout_adapter)
        evaluator_identity_sha256, evaluator_bindings = _validate_evaluator_identity(evaluator_identity_path)
        reference_sources = _validate_metrics_report(reference, expected_backend="onnx", hbm_sha=None,
            holdout_manifest_sha256=holdout_sha, evaluator_identity_sha256=evaluator_identity_sha256,
            evaluator_binding=evaluator_bindings["onnx"], runner_identity_sha256=None)
        quantized_sources = _validate_metrics_report(quantized, expected_backend="hbm", hbm_sha=hbm_sha,
            holdout_manifest_sha256=holdout_sha, evaluator_identity_sha256=evaluator_identity_sha256,
            evaluator_binding=evaluator_bindings["hbm"], runner_identity_sha256=runner_identity_sha256)
        if reference_sources != quantized_sources:
            raise ValueError("reference_quantized_holdout_identity_mismatch")
        if reference_sources != parity_sources:
            raise ValueError("evaluator_parity_holdout_identity_mismatch")
        if reference_sources != holdout_sources:
            raise ValueError("evaluator_holdout_coverage_incomplete")
        if reference_sources & calibration_sources:
            raise ValueError("validation_calibration_source_overlap")
        minimum, maximum, max_drop = _validate_thresholds(thresholds)
        report.update({"compile_receipt_sha256": sha256_file(compile_receipt_path),
                       "reference_report_sha256": sha256_file(reference_report_path),
                       "quantized_report_sha256": sha256_file(quantized_report_path),
                       "parity_report_sha256": sha256_file(parity_report_path),
                       "holdout_manifest_sha256": holdout_sha,
                       "evaluator_identity_sha256": evaluator_identity_sha256,
                       "runner_identity_sha256": runner_identity_sha256,
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
    parser.add_argument("--holdout-manifest", required=True, type=Path)
    parser.add_argument("--parity-report", required=True, type=Path)
    parser.add_argument("--runner-identity", required=True, type=Path)
    parser.add_argument("--evaluator-identity", required=True, type=Path)
    parser.add_argument("--reference-evaluator-receipt", required=True, type=Path)
    parser.add_argument("--quantized-evaluator-receipt", required=True, type=Path)
    parser.add_argument("--reference-report", required=True, type=Path)
    parser.add_argument("--quantized-report", required=True, type=Path)
    parser.add_argument("--thresholds", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    try:
        report = validate_regression(hbm=args.hbm, compile_receipt_path=args.compile_receipt,
                                     calibration_manifest=args.calibration_manifest, holdout_manifest=args.holdout_manifest,
                                     parity_report_path=args.parity_report, runner_identity_path=args.runner_identity,
                                     evaluator_identity_path=args.evaluator_identity,
                                     reference_evaluator_receipt=args.reference_evaluator_receipt, quantized_evaluator_receipt=args.quantized_evaluator_receipt,
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
