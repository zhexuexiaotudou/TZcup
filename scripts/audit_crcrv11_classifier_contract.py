#!/usr/bin/env python3
"""Forensic audit of the frozen V10 close-range classifier contract."""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import hashlib
import json
import math
from pathlib import Path
import statistics

import cv2
import numpy as np

from evaluate_trcrv10_proposals import iou


CLASSES = ("plastic_bottle", "metal_can", "paper_litter", "background_or_unknown")
TARGETS = CLASSES[:3]
COCO_CLASSES = {1: TARGETS[0], 2: TARGETS[1], 3: TARGETS[2]}
RUNTIME_POSITIVE_VIEWS = {"proposal", "context"}


def load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, payload: dict) -> None:
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def describe(values: list[float]) -> dict:
    if not values:
        return {"count": 0, "min": None, "p50": None, "mean": None, "p95": None, "max": None}
    array = np.asarray(values, dtype=np.float64)
    return {
        "count": int(array.size), "min": float(array.min()),
        "p50": float(np.percentile(array, 50)), "mean": float(array.mean()),
        "p95": float(np.percentile(array, 95)), "max": float(array.max()),
    }


def area(box: list[float]) -> float:
    return max(0.0, box[2] - box[0]) * max(0.0, box[3] - box[1])


def intersection_area(first: list[float], second: list[float]) -> float:
    return max(0.0, min(first[2], second[2]) - max(first[0], second[0])) * max(
        0.0, min(first[3], second[3]) - max(first[1], second[1])
    )


def candidate_key(row: dict) -> str:
    stem = Path(row["path"]).stem
    suffix = stem.rsplit("_", 1)[-1]
    return f"{row['scene']}:{row['frame_index']}:{suffix}"


def expected_sampler_stats(total: int, count: int) -> dict:
    if not count or not total:
        return {"expected_draws_per_unique_crop": None, "expected_unique_coverage": None}
    probability = 1.0 / (len(CLASSES) * count)
    return {
        "expected_draws_per_unique_crop": total * probability,
        "expected_unique_coverage": 1.0 - (1.0 - probability) ** total,
        "sampler": "WeightedRandomSampler(replacement=True,num_samples=total_train_rows)",
    }


def remap_path(value: str, mappings: list[tuple[str, Path]]) -> Path:
    normalized = value.replace("/", "\\")
    for source, destination in mappings:
        source_normalized = source.replace("/", "\\").rstrip("\\")
        if normalized.lower().startswith(source_normalized.lower()):
            remainder = normalized[len(source_normalized):].lstrip("\\")
            return destination.joinpath(*remainder.split("\\"))
    return Path(value)


def build_truth(coco: dict) -> tuple[dict, dict]:
    images = {int(row["id"]): row for row in coco["images"]}
    truth: dict[int, list[dict]] = defaultdict(list)
    for row in coco["annotations"]:
        x, y, width, height = row["bbox"]
        truth[int(row["image_id"])].append({
            "class_id": COCO_CLASSES[int(row["category_id"])],
            "bbox": [x, y, x + width, y + height],
        })
    by_key = {(row["scene"], int(row["frame_index"])): int(row["id"]) for row in coco["images"]}
    return {image_id: {"meta": meta, "truth": truth.get(image_id, [])} for image_id, meta in images.items()}, by_key


def audit_split(rows: list[dict], root: Path) -> dict:
    class_counts = Counter(row["class_id"] for row in rows)
    unique_hashes: dict[str, set[str]] = defaultdict(set)
    source_frames: dict[str, set[str]] = defaultdict(set)
    candidates: dict[str, set[str]] = defaultdict(set)
    missing = []
    for row in rows:
        class_id = row["class_id"]
        path = root / row["path"]
        if path.is_file():
            unique_hashes[class_id].add(sha256(path))
        else:
            missing.append(row["path"])
        source_frames[class_id].add(f"{row['scene']}:{row['frame_index']}")
        candidates[class_id].add(candidate_key(row))
    return {
        "samples_by_class": {name: class_counts[name] for name in CLASSES},
        "unique_crops_by_class": {name: len(unique_hashes[name]) for name in CLASSES},
        "unique_source_frames_by_class": {name: len(source_frames[name]) for name in CLASSES},
        "unique_candidates_by_class": {name: len(candidates[name]) for name in CLASSES},
        "unique_source_frames": len({value for values in source_frames.values() for value in values}),
        "missing_crop_count": len(missing),
        "missing_crop_examples": missing[:20],
    }


def background_label_records(coco: dict, raw: dict, threshold: float) -> list[dict]:
    frames, _ = build_truth(coco)
    predictions = {int(row["image_id"]): row["detections"] for row in raw["frames"]}
    records = []
    for image_id, frame in frames.items():
        targets = frame["truth"]
        for proposal_index, proposal in enumerate(predictions.get(image_id, [])):
            score = float(proposal["score"])
            if score < threshold:
                continue
            box = proposal["bbox_xyxy"]
            matches = sorted(
                ((iou(box, target["bbox"]), target) for target in targets),
                key=lambda item: item[0], reverse=True,
            )
            best_iou = float(matches[0][0]) if matches else 0.0
            nearest = matches[0][1] if matches else None
            if best_iou >= 0.5:
                continue
            occupancy = intersection_area(box, nearest["bbox"]) / max(area(box), 1e-12) if nearest else 0.0
            if 0.20 <= best_iou < 0.50 and occupancy >= 0.50:
                taxonomy = "CROPPED_TARGET_FRAGMENT"
            elif 0.20 <= best_iou < 0.50:
                taxonomy = "NEAR_MISS_TARGET"
            elif best_iou > 0.0 and occupancy >= 0.20:
                taxonomy = "PARTIAL_TARGET_BACKGROUND_LABEL"
            elif best_iou > 0.0 and proposal.get("source_class_label") not in (None, 0):
                taxonomy = "WRONG_CLASS_PROPOSAL"
            else:
                taxonomy = "TRUE_BACKGROUND"
            records.append({
                "scene": frame["meta"]["scene"], "frame_index": int(frame["meta"]["frame_index"]),
                "proposal_index": proposal_index, "proposal_score": score,
                "proposal_box": box, "best_gt_iou": best_iou,
                "nearest_gt_class": nearest["class_id"] if nearest else None,
                "object_occupancy": occupancy, "taxonomy": taxonomy,
            })
    return records


def geometry(rows: list[dict], coco: dict) -> dict:
    frames, by_key = build_truth(coco)
    fields: dict[str, list[float]] = defaultdict(list)
    by_view: dict[str, dict[str, list[float]]] = defaultdict(lambda: defaultdict(list))
    for row in rows:
        image_id = by_key[(row["scene"], int(row["frame_index"]))]
        meta, targets = frames[image_id]["meta"], frames[image_id]["truth"]
        box = [float(value) for value in row["source_bbox_xyxy"]]
        box_area = area(box)
        clipped = [max(0.0, box[0]), max(0.0, box[1]), min(float(meta["width"]), box[2]), min(float(meta["height"]), box[3])]
        clipping = 1.0 - area(clipped) / max(box_area, 1e-12)
        aspect = (box[2] - box[0]) / max(box[3] - box[1], 1e-12)
        occupancy = max((intersection_area(box, target["bbox"]) / max(box_area, 1e-12) for target in targets), default=0.0)
        boundary = float(box[0] <= 0 or box[1] <= 0 or box[2] >= meta["width"] or box[3] >= meta["height"])
        for name, value in (("clipping_fraction", clipping), ("aspect_ratio", aspect),
                            ("object_occupancy", occupancy), ("image_boundary_contact", boundary),
                            ("crop_area_px", box_area)):
            fields[name].append(value)
            by_view[row["view"]][name].append(value)
    return {
        "aggregate": {name: describe(values) for name, values in fields.items()},
        "by_view": {view: {name: describe(values) for name, values in metrics.items()} for view, metrics in by_view.items()},
    }


def pixel_parity(train_rows: list[dict], holdout_rows: list[dict], train_root: Path, holdout_root: Path,
                 train_coco: dict, holdout_coco: dict, mappings: list[tuple[str, Path]], sample_count: int) -> dict:
    try:
        from torchvision.io import read_image
        torchvision_available = True
    except Exception:
        read_image = None
        torchvision_available = False
    source_lookup = {}
    for split, coco in (("G10_TRAIN", train_coco), ("G10_HOLDOUT", holdout_coco)):
        for row in coco["images"]:
            source_lookup[(split, row["scene"], int(row["frame_index"]))] = row["file_name"]
    candidates = [("G10_TRAIN", row, train_root) for row in train_rows] + [
        ("G10_HOLDOUT", row, holdout_root) for row in holdout_rows
    ]
    candidates.sort(key=lambda item: hashlib.sha256(
        f"{item[0]}:{item[1]['path']}".encode("utf-8")
    ).hexdigest())
    if len(candidates) < sample_count:
        raise ValueError(f"pixel parity needs {sample_count} rows, found {len(candidates)}")
    failures = []
    channel_checks = 0
    for split, row, root in candidates[:sample_count]:
        source_path = remap_path(source_lookup[(split, row["scene"], int(row["frame_index"]))], mappings)
        crop_path = root / row["path"]
        source = cv2.imread(str(source_path), cv2.IMREAD_COLOR)
        actual = cv2.imread(str(crop_path), cv2.IMREAD_COLOR)
        if source is None or actual is None:
            failures.append({"path": row["path"], "reason": "unreadable_source_or_crop"})
            continue
        height, width = source.shape[:2]
        x1, y1, x2, y2 = row["source_bbox_xyxy"]
        x1 = max(0, min(width - 1, int(round(x1))))
        y1 = max(0, min(height - 1, int(round(y1))))
        x2 = max(1, min(width, int(round(x2))))
        y2 = max(1, min(height, int(round(y2))))
        expected = source[y1:y2, x1:x2]
        if expected.shape != actual.shape or not np.array_equal(expected, actual):
            failures.append({"path": row["path"], "reason": "cv2_round_trip_mismatch"})
            continue
        if read_image is not None:
            tensor_rgb = read_image(str(crop_path)).permute(1, 2, 0).numpy()
            expected_rgb = cv2.cvtColor(expected, cv2.COLOR_BGR2RGB)
            channel_checks += 1
            if tensor_rgb.shape != expected_rgb.shape or not np.array_equal(tensor_rgb, expected_rgb):
                failures.append({"path": row["path"], "reason": "torchvision_rgb_channel_mismatch"})
    return {
        "requested_samples": sample_count, "evaluated_samples": sample_count,
        "torchvision_available": torchvision_available, "torchvision_channel_checks": channel_checks,
        "failure_count": len(failures), "failures": failures[:20],
        "pass": torchvision_available and channel_checks == sample_count and not failures,
    }


def disagreement(report: dict) -> dict:
    pairs: dict[str, dict[str, dict]] = defaultdict(dict)
    for row in report.get("evaluated_rows", []):
        pairs[candidate_key(row)][row["view"]] = row
    complete = [pair for pair in pairs.values() if {"tight", "context"} <= set(pair)]
    disagreements = sum(pair["tight"]["predicted"] != pair["context"]["predicted"] for pair in complete)
    complementary = sum(
        (pair["tight"]["predicted"] == pair["tight"]["truth"]) !=
        (pair["context"]["predicted"] == pair["context"]["truth"])
        for pair in complete
    )
    return {
        "complete_candidates": len(complete), "prediction_disagreements": disagreements,
        "disagreement_rate": disagreements / max(len(complete), 1),
        "complementary_correctness_candidates": complementary,
        "complementary_correctness_rate": complementary / max(len(complete), 1),
    }


def decision(status: str, evidence: dict, severity: str) -> dict:
    return {"decision": status, "evidence": evidence, "severity": severity}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--train", type=Path, required=True)
    parser.add_argument("--holdout", type=Path, required=True)
    parser.add_argument("--train-coco", type=Path, required=True)
    parser.add_argument("--holdout-coco", type=Path, required=True)
    parser.add_argument("--train-raw", type=Path, required=True)
    parser.add_argument("--holdout-raw", type=Path, required=True)
    parser.add_argument("--c1-report", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--path-map", action="append", default=[])
    parser.add_argument("--pixel-samples", type=int, default=100)
    args = parser.parse_args()

    train_manifest = args.train / "CLASSIFIER_TRAIN_CROP_MANIFEST.json"
    holdout_manifest = args.holdout / "CLASSIFIER_HOLDOUT_CROP_MANIFEST.json"
    train_payload, holdout_payload = load_json(train_manifest), load_json(holdout_manifest)
    train_rows, holdout_rows = train_payload["rows"], holdout_payload["rows"]
    if {row["source_split"] for row in train_rows} != {"G10_TRAIN"}:
        raise ValueError("forensic TRAIN input must be G10_TRAIN only")
    if {row["source_split"] for row in holdout_rows} != {"G10_HOLDOUT"}:
        raise ValueError("forensic HOLDOUT input must be G10_HOLDOUT only")
    for payload in (train_payload, holdout_payload):
        if any(payload.get(name) is True for name in ("G10_DEV_VAL_SEALED_read", "VAL_NEW_read", "G5_V2_read")):
            raise RuntimeError("sealed boundary is already consumed")

    train_coco, holdout_coco = load_json(args.train_coco), load_json(args.holdout_coco)
    train_raw, holdout_raw = load_json(args.train_raw), load_json(args.holdout_raw)
    c1_report = load_json(args.c1_report)
    threshold = float(train_payload["threshold"])
    args.output.mkdir(parents=True, exist_ok=True)
    mappings = []
    for value in args.path_map:
        source, destination = value.split("=", 1)
        mappings.append((source, Path(destination)))

    train_audit, holdout_audit = audit_split(train_rows, args.train), audit_split(holdout_rows, args.holdout)
    balance = {
        "schema_version": 1, "protocol": "CRCRV11", "stage": "CRCRV11-01-CLASS-BALANCE",
        "train": train_audit, "holdout": holdout_audit,
    }
    background_train = train_audit["unique_crops_by_class"][CLASSES[-1]]
    sampler = {
        "schema_version": 1, "protocol": "CRCRV11", "stage": "CRCRV11-01-SAMPLER-REPEAT",
        "total_train_rows": len(train_rows),
        "by_class": {name: expected_sampler_stats(len(train_rows), train_audit["samples_by_class"][name]) for name in CLASSES},
    }
    background_repeat = sampler["by_class"][CLASSES[-1]]["expected_draws_per_unique_crop"]
    background_unique = {
        "schema_version": 1, "protocol": "CRCRV11", "stage": "CRCRV11-01-BACKGROUND-UNIQUENESS",
        "train_unique_crops": background_train,
        "train_unique_source_frames": train_audit["unique_source_frames_by_class"][CLASSES[-1]],
        "holdout_unique_crops": holdout_audit["unique_crops_by_class"][CLASSES[-1]],
        "sampler_expected_repeat_factor": background_repeat,
        "BACKGROUND_MEMORIZATION_RISK": background_train < 500 or (background_repeat or 0) > 5,
    }
    positive_rows = [row for row in train_rows if row["class_id"] in TARGETS]
    views = sorted({row["view"] for row in train_rows})
    view_distribution = {
        "schema_version": 1, "protocol": "CRCRV11", "stage": "CRCRV11-01-VIEW-DISTRIBUTION",
        "by_view": {
            view: {
                "samples": sum(row["view"] == view for row in train_rows),
                "unique_targets": len({candidate_key(row) for row in train_rows if row["view"] == view}),
                "class_counts": {name: sum(row["view"] == view and row["class_id"] == name for row in train_rows) for name in CLASSES},
            } for view in views
        },
        "positive_runtime_faithful_fraction": sum(row["view"] in RUNTIME_POSITIVE_VIEWS for row in positive_rows) / max(len(positive_rows), 1),
        "runtime_faithful_positive_views": sorted(RUNTIME_POSITIVE_VIEWS),
    }
    background_records = background_label_records(train_coco, train_raw, threshold)
    taxonomy = Counter(row["taxonomy"] for row in background_records)
    label_audit = {
        "schema_version": 1, "protocol": "CRCRV11", "stage": "CRCRV11-01-BACKGROUND-LABEL-AUDIT",
        "threshold": threshold, "records": background_records,
        "taxonomy_counts": dict(sorted(taxonomy.items())),
        "near_miss_020_050_count": sum(0.20 <= row["best_gt_iou"] < 0.50 for row in background_records),
    }
    crop_geometry = {
        "schema_version": 1, "protocol": "CRCRV11", "stage": "CRCRV11-01-CROP-GEOMETRY",
        "train": geometry(train_rows, train_coco), "holdout": geometry(holdout_rows, holdout_coco),
    }
    parity = pixel_parity(train_rows, holdout_rows, args.train, args.holdout, train_coco, holdout_coco,
                          mappings, args.pixel_samples)
    parity.update({"schema_version": 1, "protocol": "CRCRV11", "stage": "CRCRV11-01-CROP-PIXEL-PARITY"})
    pair_disagreement = disagreement(c1_report)
    target_confusion = sum(
        c1_report["aggregate"]["metrics"]["confusion"][row][column]
        for row in range(3) for column in range(3) if row != column
    )
    target_support = sum(sum(c1_report["aggregate"]["metrics"]["confusion"][row]) for row in range(3))
    aug = c1_report.get("targeted_recovery", {})
    strong_aug = aug.get("random_grayscale_probability", 0) > .10 or aug.get("color_jitter", {}).get("hue", 0) > .05
    root_cause = {
        "schema_version": 1, "protocol": "CRCRV11", "stage": "CRCRV11-01-ROOT-CAUSE-DECISION",
        "decisions": {
            "BACKGROUND_SAMPLE_SCARCITY": decision("supported" if background_train < 500 else "unsupported", {"unique_background_crops": background_train}, "critical" if background_train < 100 else "high"),
            "BACKGROUND_MEMORIZATION": decision("supported" if background_unique["BACKGROUND_MEMORIZATION_RISK"] else "unsupported", {"expected_repeat_factor": background_repeat}, "critical"),
            "NEAR_MISS_LABEL_NOISE": decision("supported" if label_audit["near_miss_020_050_count"] else "unsupported", {"near_miss_count": label_audit["near_miss_020_050_count"]}, "high"),
            "TRAIN_RUNTIME_VIEW_MISMATCH": decision("supported" if view_distribution["positive_runtime_faithful_fraction"] < .8 else "unsupported", {"runtime_faithful_fraction": view_distribution["positive_runtime_faithful_fraction"]}, "high"),
            "AUGMENTATION_TOO_STRONG": decision("supported" if strong_aug else "unsupported", aug, "high"),
            "CROP_CONTEXT_MISMATCH": decision("supported" if pair_disagreement["disagreement_rate"] >= .10 else "unsupported", pair_disagreement, "medium"),
            "PIXEL_CHANNEL_BUG": decision("unsupported" if parity["pass"] else "unknown", {"parity_pass": parity["pass"], "failure_count": parity["failure_count"]}, "critical"),
            "CLASS_INTRINSIC_CONFUSION": decision("supported" if target_confusion / max(target_support, 1) >= .05 else "unsupported", {"target_to_target_error_rate": target_confusion / max(target_support, 1)}, "medium"),
        },
        "sealed_boundary": {"G10_DEV_VAL_SEALED_read": False, "VAL_NEW_read": False, "G5_V2_read": False},
    }
    outputs = {
        "V11_CLASS_BALANCE_AUDIT.json": balance,
        "V11_BACKGROUND_UNIQUENESS_AUDIT.json": background_unique,
        "V11_SAMPLER_REPEAT_AUDIT.json": sampler,
        "V11_VIEW_DISTRIBUTION_AUDIT.json": view_distribution,
        "V11_BACKGROUND_LABEL_AUDIT.json": label_audit,
        "V11_CROP_GEOMETRY_AUDIT.json": crop_geometry,
        "V11_CROP_PIXEL_PARITY.json": parity,
        "V11_ROOT_CAUSE_DECISION.json": root_cause,
    }
    for name, payload in outputs.items():
        write_json(args.output / name, payload)
    print(json.dumps({
        "train": train_audit, "holdout": holdout_audit,
        "background": background_unique, "runtime_fraction": view_distribution["positive_runtime_faithful_fraction"],
        "near_miss_count": label_audit["near_miss_020_050_count"], "pixel_parity": parity["pass"],
    }, indent=2))
    return 0 if parity["pass"] and not train_audit["missing_crop_count"] and not holdout_audit["missing_crop_count"] else 3


if __name__ == "__main__":
    raise SystemExit(main())
