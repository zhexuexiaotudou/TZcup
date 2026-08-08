#!/usr/bin/env python3
"""Select the bounded FCOS teacher input scale from development pixels only."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np


def summarize(values: list[int]) -> dict:
    array = np.asarray(values, dtype=np.float64)
    if not len(array):
        raise RuntimeError("scale audit requires positive discrete instances")
    return {
        "count": int(len(array)),
        "p10_px": float(np.percentile(array, 10)),
        "p25_px": float(np.percentile(array, 25)),
        "p50_px": float(np.percentile(array, 50)),
        "p75_px": float(np.percentile(array, 75)),
        "fraction_below_4px": float(np.mean(array < 4)),
        "fraction_below_8px": float(np.mean(array < 8)),
        "fraction_below_12px": float(np.mean(array < 12)),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--instance-records", required=True, type=Path)
    parser.add_argument("--teacher-report", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    rows = [
        json.loads(line)
        for line in args.instance_records.read_text(encoding="utf-8").splitlines()
        if line
    ]
    development = [
        row
        for row in rows
        if row.get("split") in {"train", "val"}
        and int(row.get("semantic_id", 0)) in {1, 2, 3}
    ]
    by_split = {
        split: summarize(
            [
                int(row["bbox_shortest_side_px"])
                for row in development
                if row["split"] == split
            ]
        )
        for split in ("train", "val")
    }
    by_class = {
        str(class_id): summarize(
            [
                int(row["bbox_shortest_side_px"])
                for row in development
                if row["split"] == "val"
                and int(row["semantic_id"]) == class_id
            ]
        )
        for class_id in (1, 2, 3)
    }
    first = json.loads(args.teacher_report.read_text(encoding="utf-8"))
    median = by_split["val"]["p50_px"]
    selected_scale = 1 if median >= 12 else 2 if median * 2 >= 12 else None
    report = {
        "schema_version": 1,
        "stage": "PERCEPTION-P2_TEACHER_SCALE_AUDIT",
        "read_splits": ["train", "val"],
        "legacy_G4_D6_diagnostic_read": False,
        "G5_SEALED_FINAL_read": False,
        "shortest_side_distribution": by_split,
        "val_shortest_side_by_class": by_class,
        "first_teacher": {
            "input_scale": first["config"].get("input_scale", 1),
            "best_epoch": first["training"]["best_epoch"],
            "val_recall": first["cross_world_val_metrics"][
                "all_gt_candidate_recall"
            ],
            "val_ap50": first["cross_world_val_metrics"]["ap50"],
            "false_candidates_per_min": first["cross_world_val_metrics"][
                "false_candidates_per_min"
            ],
            "gate_pass": first["teacher_data_learnability_pass"],
        },
        "selection_rule": (
            "choose only scale 1 or 2; require median discrete shortest side "
            "at least 12 model-input pixels; otherwise return to capture scale"
        ),
        "selected_bounded_input_scale": selected_scale,
        "next_action": (
            "run_single_scale2_teacher_control"
            if selected_scale == 2
            else "return_to_camera_or_placement_scale"
            if selected_scale is None
            else "no_scale_change"
        ),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
