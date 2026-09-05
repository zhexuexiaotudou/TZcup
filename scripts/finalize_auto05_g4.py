#!/usr/bin/env python3
"""Fail closed G4 evidence finalizer; never overwrites historical AUTO-05 evidence."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def under(path: Path, root: Path) -> bool:
    try:
        path.resolve().relative_to(root.resolve())
    except ValueError:
        return False
    return True


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", required=True)
    parser.add_argument("--raw-root", required=True)
    parser.add_argument("--dataset-evidence", required=True)
    parser.add_argument("--g4-contract", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    repo = Path(args.repo).resolve()
    root = repo / ".work" / "auto05-g4"
    raw, dataset, contract, output = map(
        lambda item: Path(item).resolve(),
        (args.raw_root, args.dataset_evidence, args.g4_contract, args.output),
    )
    if not all(under(path, root) for path in (raw, dataset, output)):
        parser.error("G4 inputs and output must remain below TZcup/.work/auto05-g4")
    if contract != repo / "starter_ws/src/sanitation_learning/config/auto05_g4_screening.yaml":
        parser.error("G4 requires the checked-in frozen contract")
    report_path = raw / "auto05_screening_report.json"
    required = [
        report_path, raw / "auto05_direct_detector.onnx",
        raw / "auto05_rgbd_area_segmenter.onnx", raw / "environment.json",
        dataset / "g3_dataset_qa.json", dataset / "split_manifest.json",
        dataset / "leakage_report.json",
    ]
    missing = [str(path) for path in required if not path.is_file()]
    if missing:
        parser.error("missing G4 evidence: " + ", ".join(missing))
    report = json.loads(report_path.read_text(encoding="utf-8"))
    if report.get("attempt_id") != "AUTO-05-G3-SCREENING-V4":
        parser.error("only the pre-registered G4/V4 report may be finalized")
    if report.get("selection_policy", {}).get("test_used_for_model_selection") is not False:
        parser.error("test-driven G4 selection is forbidden")
    dataset_report = json.loads((dataset / "g3_dataset_qa.json").read_text(encoding="utf-8"))
    status = "PENDING_REVIEW" if (
        dataset_report.get("dataset_gate_pass")
        and report.get("auto05_screening_gate_pass")
    ) else "BLOCKED"
    output.mkdir(parents=True, exist_ok=True)
    (output / "g4_finalization.json").write_text(json.dumps({
        "schema_version": 1, "stage": "AUTO-05", "attempt": "G4",
        "status": status, "historical_evidence_overwritten": False,
        "auto06_authorized": False,
        "reason": "manual acceptance review required even after a machine pass",
    }, indent=2) + "\n", encoding="utf-8")
    return 0 if status == "PENDING_REVIEW" else 2


if __name__ == "__main__":
    raise SystemExit(main())
