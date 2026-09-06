#!/usr/bin/env python3
"""Execute one contract-pinned evaluator and retain its raw-output receipt.

No operator command is accepted.  The frozen contract is the sole source of
the executable path, digest, argv template and output schema.
"""
from __future__ import annotations

import argparse
import json
import subprocess
import time
from pathlib import Path
from typing import Any

from hbm_evidence_common import atomic_json, fresh_directory, load_object, normal_file, sha256_file

RECEIPT_ID = "tzcup_dosod_quantized_metric_evaluator_receipt_v1"
METRICS_REPORT_ID = "tzcup_dosod_perception_validation_metrics_v1"
CLASS_ORDER = ["litter_cube", "fallen_leaves", "dust_or_soil", "puddle"]


def _block(report: dict[str, Any], reason: str) -> None:
    if reason not in report["blockers"]:
        report["blockers"].append(reason)


def execute(*, contract_path: Path, backend: str, holdout_manifest: Path, holdout_root: Path, output: Path) -> dict[str, Any]:
    for path, label in ((contract_path, "evaluator_contract"), (holdout_manifest, "holdout_manifest")):
        normal_file(path, label)
    fresh_directory(output, "evaluator_evidence_output")
    receipt: dict[str, Any] = {"schema_version": 1, "receipt_id": RECEIPT_ID, "status": "BLOCKED", "backend": backend,
        "evidence_root": str(output.resolve()), "contract_sha256": sha256_file(contract_path),
        "holdout_manifest_sha256": sha256_file(holdout_manifest), "blockers": [], "started_epoch_ns": time.time_ns()}
    try:
        contract = load_object(contract_path)
        rows = contract.get("backends")
        if contract.get("schema_version") != 1 or contract.get("status") != "FROZEN" or not isinstance(rows, dict) or backend not in {"onnx", "hbm"} or not isinstance(rows.get(backend), dict):
            raise ValueError("frozen_evaluator_contract_missing_backend")
        row = rows[backend]
        executable = Path(str(row.get("absolute_path", "")))
        template = row.get("argv_template")
        schema = row.get("output_schema")
        if not executable.is_absolute() or not isinstance(template, list) or not template or not isinstance(schema, dict):
            raise ValueError("evaluator_contract_binding_invalid")
        normal_file(executable, "contract_evaluator")
        if row.get("sha256") != sha256_file(executable) or not isinstance(row.get("version"), str) or not row["version"].strip():
            raise ValueError("evaluator_contract_executable_identity_mismatch")
        if schema.get("report_id") != METRICS_REPORT_ID or schema.get("class_order") != CLASS_ORDER:
            raise ValueError("evaluator_contract_output_schema_unknown")
        metrics = output / "metrics.json"
        values = {"{evaluator}": str(executable), "{holdout_manifest}": str(holdout_manifest.resolve()), "{holdout_root}": str(holdout_root.resolve()), "{metrics_output}": str(metrics)}
        if any(not isinstance(item, str) or item not in values for item in template):
            raise ValueError("evaluator_contract_argv_template_unknown")
        command = [values[item] for item in template]
        completed = subprocess.run(command, capture_output=True, text=True, check=False)
        stdout, stderr = output / "evaluator.stdout.txt", output / "evaluator.stderr.txt"
        stdout.write_text(completed.stdout or "", encoding="utf-8"); stderr.write_text(completed.stderr or "", encoding="utf-8")
        receipt.update({"command": command, "returncode": completed.returncode, "stdout_path": str(stdout.resolve()), "stdout_sha256": sha256_file(stdout), "stderr_path": str(stderr.resolve()), "stderr_sha256": sha256_file(stderr), "metrics_path": str(metrics.resolve()) if metrics.is_file() else None, "metrics_sha256": sha256_file(metrics) if metrics.is_file() else None})
        if completed.returncode != 0 or not metrics.is_file():
            raise ValueError("evaluator_execution_failed_or_metrics_missing")
        result = load_object(metrics)
        if result.get("report_id") != METRICS_REPORT_ID or result.get("status") != "VERIFIED" or result.get("class_order") != CLASS_ORDER:
            raise ValueError("evaluator_metrics_schema_mismatch")
        receipt["status"] = "EVALUATOR_EXECUTED"
    except Exception as exc:
        _block(receipt, f"evaluator_precondition_or_execution_failed:{type(exc).__name__}")
    receipt["ended_epoch_ns"] = time.time_ns(); receipt_path = output / "dosod_quantized_metric_evaluator_receipt.json"; receipt["receipt_path"] = str(receipt_path.resolve()); atomic_json(receipt_path, receipt)
    return receipt


def main() -> int:
    p = argparse.ArgumentParser(); p.add_argument("--contract", required=True, type=Path); p.add_argument("--backend", choices=("onnx", "hbm"), required=True); p.add_argument("--holdout-manifest", required=True, type=Path); p.add_argument("--holdout-root", required=True, type=Path); p.add_argument("--output", required=True, type=Path); a = p.parse_args()
    result = execute(contract_path=a.contract, backend=a.backend, holdout_manifest=a.holdout_manifest, holdout_root=a.holdout_root, output=a.output)
    print(json.dumps(result, indent=2)); return 0 if result["status"] == "EVALUATOR_EXECUTED" else 2

if __name__ == "__main__": raise SystemExit(main())
