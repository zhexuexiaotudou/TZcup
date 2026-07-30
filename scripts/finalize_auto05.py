#!/usr/bin/env python3
"""Create compact AUTO-05 failure evidence and update autonomous state."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import shutil


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def write_json(path: Path, value: object) -> None:
    path.write_text(json.dumps(value, indent=2) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", required=True)
    parser.add_argument("--raw-root", required=True)
    parser.add_argument("--dataset-evidence", required=True)
    parser.add_argument("--implementation-commit", required=True)
    args = parser.parse_args()
    repo = Path(args.repo)
    raw_root = Path(args.raw_root)
    dataset = Path(args.dataset_evidence)
    output = repo / "artifacts" / "autonomous_auto05_20260730_evidence"
    output.mkdir(parents=True, exist_ok=True)

    reports = []
    for attempt in range(1, 4):
        source = (
            raw_root
            / f"autonomous_auto05_attempt{attempt}_raw"
            / "auto05_screening_report.json"
        )
        target = output / f"attempt{attempt}_screening_report.json"
        shutil.copy2(source, target)
        reports.append(json.loads(source.read_text(encoding="utf-8")))
    for name in ("g3_dataset_qa.json", "leakage_report.json", "split_manifest.json"):
        shutil.copy2(dataset / name, output / name)
    selected_raw = raw_root / "autonomous_auto05_attempt3_raw"
    for name in (
        "auto05_direct_detector.onnx",
        "auto05_rgbd_area_segmenter.onnx",
        "environment.json",
    ):
        shutil.copy2(selected_raw / name, output / name)

    best = reports[2]
    detector = best["detector"]["results"]
    area = best["area_segmenter"]["results"]
    failed = [name for name, passed in best["gates"].items() if not passed]
    summary = {
        "schema_version": 1,
        "stage": "AUTO-05",
        "status": "BLOCKED",
        "machine_gate_pass": False,
        "first_blocking_layer": "G3_split_model_screening_gates_failed_after_3_attempts",
        "attempt_count": 3,
        "best_attempt": "AUTO-05-G3-SCREENING-V3",
        "dataset_gate_pass": True,
        "best_metrics": {
            "in_domain_macro_f1": detector["in_domain"]["macro_f1"],
            "validation_cross_world_macro_f1": detector[
                "validation_cross_world"
            ]["macro_f1"],
            "test_cross_world_macro_f1": detector["test_cross_world"][
                "macro_f1"
            ],
            "validation_small_object_recall": detector[
                "validation_cross_world"
            ]["small_object_recall"],
            "test_small_object_recall": detector["test_cross_world"][
                "small_object_recall"
            ],
            "validation_negative_fp_per_frame": detector[
                "validation_cross_world"
            ]["negative_only_false_positive_per_frame"],
            "test_negative_fp_per_frame": detector["test_cross_world"][
                "negative_only_false_positive_per_frame"
            ],
            "validation_leaf_iou": area["validation_cross_world"][
                "iou_by_class"
            ]["leaf_pile"],
            "validation_puddle_iou": area["validation_cross_world"][
                "iou_by_class"
            ]["puddle"],
            "validation_macro_miou": area["validation_cross_world"][
                "macro_miou"
            ],
            "test_leaf_iou": area["test_cross_world"]["iou_by_class"][
                "leaf_pile"
            ],
            "test_puddle_iou": area["test_cross_world"]["iou_by_class"][
                "puddle"
            ],
            "test_macro_miou": area["test_cross_world"]["macro_miou"],
        },
        "failed_gates": failed,
        "claim_boundary": best["claim_boundary"],
    }
    write_json(output / "metrics_summary.json", summary)
    write_json(
        output / "attempt_ledger.json",
        {
            "attempts": [
                {
                    "attempt_id": report["attempt_id"],
                    "hypothesis": report["hypothesis"],
                    "gate_pass": report["auto05_screening_gate_pass"],
                    "failed_gates": [
                        name
                        for name, passed in report["gates"].items()
                        if not passed
                    ],
                }
                for report in reports
            ],
            "attempt_limit_reached": True,
            "further_blind_tuning_allowed": False,
        },
    )
    write_json(
        output / "stage_status.json",
        {
            "stage": "AUTO-05",
            "status": "BLOCKED",
            "machine_gate_pass": False,
            "first_blocking_layer": summary["first_blocking_layer"],
            "implementation_commit": args.implementation_commit,
            "evidence_dir": str(output.relative_to(repo)).replace("\\", "/"),
        },
    )
    (output / "commands.txt").write_text(
        "py -3 scripts/auto05_finalize_dataset.py --data-root <G3_ROOT> "
        "--output-dir <DATASET_EVIDENCE>\n"
        "powershell -NoProfile -ExecutionPolicy Bypass -File "
        "scripts/run_auto05_screening_docker.ps1 -Attempt <1|2|3>\n",
        encoding="utf-8",
    )
    (output / "README.md").write_text(
        "# AUTO-05 compact evidence\n\n"
        "The native Gazebo G3 dataset gate passed. Three bounded model "
        "screening attempts improved the metrics but did not pass all frozen "
        "gates. AUTO-05 is therefore BLOCKED; AUTO-06/07/08 must not run.\n",
        encoding="utf-8",
    )
    manifest = []
    for path in sorted(output.iterdir()):
        if path.name == "artifact_manifest.json" or not path.is_file():
            continue
        manifest.append(
            {
                "path": path.name,
                "bytes": path.stat().st_size,
                "sha256": sha256(path),
            }
        )
    write_json(
        output / "artifact_manifest.json",
        {"schema_version": 1, "files": manifest},
    )

    state_path = repo / "AUTONOMOUS_STATE.json"
    state = json.loads(state_path.read_text(encoding="utf-8"))
    stage = state["stages"]["AUTO-05"]
    stage.update(
        {
            "status": "BLOCKED",
            "machine_gate_pass": False,
            "blocked": True,
            "blocked_external": False,
            "first_blocking_layer": summary["first_blocking_layer"],
            "attempt_count": 3,
            "selected_attempt": None,
            "implementation_commit": args.implementation_commit,
            "evidence_dir": str(output.relative_to(repo)).replace("\\", "/"),
            "metrics": summary,
            "unexecuted_items": [
                "AUTO-06 formal dataset and perception",
                "AUTO-07 live learned perception",
                "AUTO-08 learned spot cleaning",
            ],
        }
    )
    for dependent in ("AUTO-06", "AUTO-07", "AUTO-08"):
        state["stages"][dependent].update(
            {
                "status": "BLOCKED",
                "machine_gate_pass": False,
                "blocked": True,
                "blocked_external": False,
                "first_blocking_layer": "dependency_AUTO-05_blocked",
                "unexecuted_items": ["dependency AUTO-05 did not pass"],
            }
        )
    state["run"].update(
        {
            "current_stage": "AUTO-14",
            "status": "RUNNING",
            "last_commit": args.implementation_commit,
        }
    )
    write_json(state_path, state)
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
