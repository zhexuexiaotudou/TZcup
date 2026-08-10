#!/usr/bin/env python3
"""Aggregate fixed VAL/D1-D5 Area evidence for the OPRV3-06 gate."""

from __future__ import annotations

import argparse
import hashlib
import json
import statistics
from pathlib import Path


SPLITS = ("VAL", "D1", "D2", "D3", "D4", "D5")
CLASSES = ("leaf_pile", "puddle")
MODEL_TASKS = ("leaf", "puddle")
THRESHOLDS = {
    "leaf_iou": 0.80,
    "puddle_iou": 0.80,
    "macro_miou": 0.80,
    "boundary_f1": 0.75,
    "negative_area_fp_per_frame": 0.05,
}


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def aggregate(report: dict) -> dict:
    if report.get("G5_SEALED_FINAL_read") is not False:
        raise ValueError("Area audit violates the G5 sealed-final boundary")
    if report.get("legacy_G4_D6_read") is not False:
        raise ValueError("Area audit violates the legacy D6 boundary")
    models = report.get("models", {})
    for task in MODEL_TASKS:
        record = models.get(task)
        if not isinstance(record, dict):
            raise ValueError(f"Area audit is missing {task} model provenance")
        if record.get("checkpoint_status") != "training_complete":
            raise ValueError(f"Area audit {task} checkpoint is not training_complete")
        digest = record.get("sha256")
        if not isinstance(digest, str) or len(digest) != 64:
            raise ValueError(f"Area audit {task} checkpoint SHA-256 is invalid")
    splits = report.get("splits", {})
    missing = [name for name in SPLITS if name not in splits]
    if missing:
        raise ValueError(f"Area audit is missing fixed splits: {missing}")

    totals = {
        "intersection": [0, 0],
        "union": [0, 0],
        "boundary_intersection": [0, 0],
        "boundary_union": [0, 0],
        "negative_frames": 0,
        "negative_fp_frames": 0,
    }
    split_metrics = {}
    for name in SPLITS:
        metrics = splits[name].get("development_selected_postprocess")
        if not metrics:
            raise ValueError(f"Area split {name} lacks selected postprocess evidence")
        pixels = metrics.get("pixel_totals")
        if not pixels:
            raise ValueError(f"Area split {name} lacks auditable pixel totals")
        for key in (
            "intersection",
            "union",
            "boundary_intersection",
            "boundary_union",
        ):
            values = pixels.get(key)
            if not isinstance(values, list) or len(values) != 2:
                raise ValueError(f"Area split {name} has invalid {key}")
            totals[key] = [
                totals[key][index] + int(values[index]) for index in range(2)
            ]
        totals["negative_frames"] += int(pixels.get("negative_frames", 0))
        totals["negative_fp_frames"] += int(
            pixels.get("negative_fp_frames", 0)
        )
        split_metrics[name] = {
            "iou_by_class": metrics["iou_by_class"],
            "boundary_f1": metrics["postprocessed_mask_boundary_f1"],
            "negative_area_fp_per_frame": metrics[
                "negative_area_fp_per_frame"
            ],
        }

    iou = [
        totals["intersection"][index] / max(totals["union"][index], 1)
        for index in range(2)
    ]
    boundary = [
        2 * totals["boundary_intersection"][index]
        / max(
            totals["boundary_intersection"][index]
            + totals["boundary_union"][index],
            1,
        )
        for index in range(2)
    ]
    area = {
        "iou_by_class": dict(zip(CLASSES, iou)),
        "macro_miou": statistics.fmean(iou),
        "boundary_f1_by_class": dict(zip(CLASSES, boundary)),
        "boundary_f1": statistics.fmean(boundary),
        "negative_only_frames": totals["negative_frames"],
        "negative_only_fp_frames": totals["negative_fp_frames"],
        "negative_area_fp_per_frame": totals["negative_fp_frames"]
        / max(totals["negative_frames"], 1),
        "pixel_totals": totals,
    }
    gates = {
        "leaf_iou": area["iou_by_class"]["leaf_pile"]
        >= THRESHOLDS["leaf_iou"],
        "puddle_iou": area["iou_by_class"]["puddle"]
        >= THRESHOLDS["puddle_iou"],
        "macro_miou": area["macro_miou"] >= THRESHOLDS["macro_miou"],
        "boundary_f1": area["boundary_f1"] >= THRESHOLDS["boundary_f1"],
        "negative_area_fp_per_frame": area["negative_area_fp_per_frame"]
        <= THRESHOLDS["negative_area_fp_per_frame"],
    }
    return {
        "schema_version": 1,
        "protocol": "OPRV3-06",
        "thresholds": THRESHOLDS,
        "models": models,
        "selected_config": report["development_selected_config"],
        "split_metrics": split_metrics,
        "cross_world_aggregate": {"area": area},
        "gates": gates,
        "OPRV3_06_AREA_PASS": all(gates.values()),
        "G5_SEALED_FINAL_read": False,
        "legacy_G4_D6_read": False,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--area-audit", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    source = json.loads(args.area_audit.read_text(encoding="utf-8"))
    output = aggregate(source)
    output["input"] = {
        "path": args.area_audit.as_posix(),
        "sha256": sha256(args.area_audit),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(output, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(output, indent=2))
    return 0 if output["OPRV3_06_AREA_PASS"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
