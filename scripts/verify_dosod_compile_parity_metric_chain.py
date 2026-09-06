#!/usr/bin/env python3
"""Re-validate a retained DOSOD compile/parity/metric bundle without running models.

The bundle is intentionally explicit and relative to one ordinary artifact
directory.  This is the same verifier used before a board collection and by
the offline finalizer; a handwritten three-row summary is never admission.
"""
from __future__ import annotations

import json
import tempfile
from pathlib import Path
from typing import Any

from hbm_evidence_common import load_object, normal_file, path_under, sha256_file
from validate_dosod_quantized_metric_regression import validate_regression


REQUIRED = {
    "compile_receipt", "calibration_manifest", "holdout_manifest", "parity_report",
    "runner_identity", "evaluator_identity", "reference_evaluator_receipt",
    "quantized_evaluator_receipt", "reference_report", "quantized_report", "thresholds",
}


def verify_dosod_compile_parity_metric_chain(bundle_root: Path, hbm: Path) -> dict[str, Any]:
    """Return a compact bound summary or raise; never accept partial receipts."""
    normal_file(hbm, "dosod_hbm")
    if bundle_root.is_symlink() or not bundle_root.is_dir():
        raise ValueError("dosod_admission_bundle_root_invalid")
    manifest_path = bundle_root / "dosod_admission_bundle.json"
    normal_file(manifest_path, "dosod_admission_bundle_manifest")
    manifest = load_object(manifest_path)
    if manifest.get("schema_version") != 1 or set(manifest.get("artifacts", {})) != REQUIRED:
        raise ValueError("dosod_admission_bundle_manifest_invalid")
    artifacts = {
        name: path_under(bundle_root, value, f"dosod_admission_bundle_{name}")
        for name, value in manifest["artifacts"].items()
    }
    # validate_regression is the canonical complete admission implementation:
    # it rechecks compile producer/raw streams/config/preflight/identity,
    # calibration tensors, parity+runner+holdout, and both evaluator receipts.
    with tempfile.TemporaryDirectory(prefix="tzcup-dosod-recheck-") as scratch:
        report = validate_regression(
            hbm=hbm, compile_receipt_path=artifacts["compile_receipt"],
            calibration_manifest=artifacts["calibration_manifest"],
            holdout_manifest=artifacts["holdout_manifest"], parity_report_path=artifacts["parity_report"],
            runner_identity_path=artifacts["runner_identity"], evaluator_identity_path=artifacts["evaluator_identity"],
            reference_evaluator_receipt=artifacts["reference_evaluator_receipt"],
            quantized_evaluator_receipt=artifacts["quantized_evaluator_receipt"],
            reference_report_path=artifacts["reference_report"], quantized_report_path=artifacts["quantized_report"],
            thresholds_path=artifacts["thresholds"], output=Path(scratch) / "recheck",
        )
    if report.get("status") != "REGRESSION_PASSED":
        raise ValueError("dosod_admission_bundle_full_recheck_failed")
    return {
        "bundle_path": str(bundle_root.resolve()), "bundle_manifest_sha256": sha256_file(manifest_path),
        "compile_receipt_sha256": sha256_file(artifacts["compile_receipt"]),
        "parity_report_sha256": sha256_file(artifacts["parity_report"]),
        "metric_report_sha256": sha256_file(artifacts["quantized_report"]),
        "hbm_sha256": sha256_file(hbm),
    }
