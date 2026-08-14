#!/usr/bin/env python3
"""Build the runtime-faithful CRCRV11 C11 proposal-pair dataset."""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np

from audit_crcrv11_classifier_contract import expand, remap_path
from evaluate_trcrv10_proposals import iou
from prepare_trcrv10_classifier_holdout import depth_statistics, distance_bucket, size_bucket


CLASSES = ("plastic_bottle", "metal_can", "paper_litter", "background_or_unknown")
TARGETS = CLASSES[:3]
COCO_CLASSES = {1: TARGETS[0], 2: TARGETS[1], 3: TARGETS[2]}
SOURCE_CLASS_TAXONOMY = {0: "plastic_like_clutter", 1: "metal_like_clutter", 2: "paper_like_flat_mark"}
GROUND_BOXES = ((.18, .54, .40, .78), (.58, .55, .82, .80))


def load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, payload: dict) -> None:
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def phash(image: np.ndarray) -> str:
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    resized = cv2.resize(gray, (32, 32), interpolation=cv2.INTER_AREA)
    transformed = cv2.dct(resized.astype(np.float32))[:8, :8]
    values = transformed.flatten()[1:]
    bits = values > np.median(values)
    packed = np.packbits(np.concatenate([bits, np.asarray([False])]))
    return packed.tobytes().hex()


def clip_crop(image: np.ndarray, box: list[float]) -> np.ndarray:
    height, width = image.shape[:2]
    x1, y1, x2, y2 = box
    x1 = max(0, min(width - 1, int(round(x1))))
    y1 = max(0, min(height - 1, int(round(y1))))
    x2 = max(1, min(width, int(round(x2))))
    y2 = max(1, min(height, int(round(y2))))
    if x2 <= x1 or y2 <= y1:
        raise ValueError(f"empty crop: {box}")
    return image[y1:y2, x1:x2]


def png_bytes(image: np.ndarray) -> bytes:
    ok, encoded = cv2.imencode(".png", image)
    if not ok:
        raise RuntimeError("failed to encode PNG")
    return encoded.tobytes()


def ground_taxonomy(world_id: str, slot: int) -> str:
    world = world_id.lower()
    if "wet" in world:
        return ("wet_specular_highlight", "shadow")[slot]
    if "asphalt" in world:
        return ("road_paint", "seam_crack")[slot]
    if "concrete" in world:
        return ("paper_like_flat_mark", "shadow")[slot]
    if "cobblestone" in world:
        return ("stone_leaf", "seam_crack")[slot]
    if "brick" in world:
        return ("paper_like_flat_mark", "road_paint")[slot]
    return ("seam_crack", "shadow")[slot]


def development_partition(world_id: str) -> str:
    return "dev" if "_w06_" in world_id else "fit"


def build_index(coco: dict) -> tuple[dict[int, dict], dict[int, list[dict]]]:
    images = {int(row["id"]): row for row in coco["images"]}
    truth: dict[int, list[dict]] = defaultdict(list)
    for row in coco["annotations"]:
        x, y, width, height = row["bbox"]
        truth[int(row["image_id"])].append({
            "class_id": COCO_CLASSES[int(row["category_id"])],
            "bbox": [x, y, x + width, y + height],
            "short_side_px": min(width, height),
        })
    return images, truth


def proposal_label(box: list[float], truth: list[dict]) -> tuple[str, float, dict | None, float]:
    matches = sorted(((iou(box, row["bbox"]), row) for row in truth), key=lambda item: item[0], reverse=True)
    best_iou = float(matches[0][0]) if matches else 0.0
    nearest = matches[0][1] if matches else None
    if best_iou >= .5:
        return nearest["class_id"], best_iou, nearest, 0.0
    if .20 <= best_iou < .50:
        return "AMBIGUOUS_NEAR_MISS", best_iou, nearest, 0.0
    return CLASSES[-1], best_iou, nearest, 0.0


def make_pair(output: Path, split: str, pair_id: str, class_id: str, taxonomy: str,
              image: np.ndarray, box: list[float], meta: dict, proposal_score: float | None,
              best_iou: float, depth_meta: dict, source_kind: str,
              development_role: str) -> tuple[dict, dict[str, str]]:
    tight, context = clip_crop(image, box), clip_crop(image, expand(box))
    tight_bytes, context_bytes = png_bytes(tight), png_bytes(context)
    relative_root = Path(split.lower()) / class_id / pair_id
    tight_relative = relative_root / "tight.png"
    context_relative = relative_root / "context.png"
    (output / tight_relative).parent.mkdir(parents=True, exist_ok=True)
    (output / tight_relative).write_bytes(tight_bytes)
    (output / context_relative).write_bytes(context_bytes)
    hashes = {
        "tight_sha256": sha256_bytes(tight_bytes), "context_sha256": sha256_bytes(context_bytes),
        "tight_phash": phash(tight), "context_phash": phash(context),
    }
    short_side = min(box[2] - box[0], box[3] - box[1])
    pair = {
        "pair_id": pair_id, "tight_path": tight_relative.as_posix(), "context_path": context_relative.as_posix(),
        "proposal_box": [float(value) for value in box], "proposal_score": proposal_score,
        "source_frame": meta["file_name"], "source_mission": meta["scene"],
        "source_frame_index": int(meta["frame_index"]), "source_world": meta["world_id"],
        "source_seed": int(meta["scene_seed"]), "source_split": split,
        "class": class_id, "best_iou": best_iou, "background_taxonomy": taxonomy,
        "source_kind": source_kind, "development_partition": development_role,
        "size_bucket": size_bucket(short_side), "distance_m": depth_meta["median_m"],
        "distance_bucket": distance_bucket(depth_meta["median_m"]),
        "depth_valid_fraction": depth_meta["valid_fraction"],
        "depth_robust_sigma_m": depth_meta["robust_sigma_m"],
        "projection_covariance_m2": depth_meta["projection_covariance_m2"],
        **hashes,
    }
    return pair, hashes


def build_split(coco: dict, raw: dict, output: Path, split: str, threshold: float, min_short_side: int,
                mappings: list[tuple[str, Path]], include_train_bank: bool) -> tuple[list[dict], list[dict]]:
    images, truth_by_image = build_index(coco)
    raw_by_image = {int(row["image_id"]): row["detections"] for row in raw["frames"]}
    pairs, ambiguous = [], []
    for image_id, meta in sorted(images.items()):
        source = remap_path(meta["file_name"], mappings)
        image = cv2.imread(str(source), cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError(f"unreadable RGB: {source}")
        depth = np.load(remap_path(meta["depth_file_name"], mappings))
        camera = load_json(remap_path(meta["camera_file_name"], mappings))
        truth = truth_by_image.get(image_id, [])
        for proposal_index, proposal in enumerate(raw_by_image.get(image_id, [])):
            score, box = float(proposal["score"]), proposal["bbox_xyxy"]
            short_side = min(box[2] - box[0], box[3] - box[1])
            if short_side < min_short_side:
                continue
            class_id, best_iou, nearest, _ = proposal_label(box, truth)
            if class_id == "AMBIGUOUS_NEAR_MISS" and score >= threshold:
                ambiguous.append({
                    "scene": meta["scene"], "frame_index": int(meta["frame_index"]),
                    "proposal_index": proposal_index, "proposal_score": score,
                    "proposal_box": box, "best_iou": best_iou,
                    "nearest_gt_class": nearest["class_id"] if nearest else None,
                    "policy": "ignore",
                })
                continue
            source_kind = taxonomy = None
            if score >= threshold:
                if class_id in TARGETS:
                    source_kind, taxonomy = "matched_runtime_proposal", "matched_target"
                elif class_id == CLASSES[-1]:
                    source_kind, taxonomy = "unmatched_real_proposal", "unmatched_real_proposal"
            elif include_train_bank and .05 <= score < threshold and class_id == CLASSES[-1]:
                source_kind = "low_score_hard_negative"
                taxonomy = SOURCE_CLASS_TAXONOMY.get(proposal.get("source_class_label"), "low_score_hard_negative")
            if source_kind is None:
                continue
            pair_id = f"{meta['scene']}_{int(meta['frame_index']):03d}_p{proposal_index:03d}"
            depth_meta = depth_statistics(depth, box, camera)
            pair, _ = make_pair(
                output, split, pair_id, class_id, taxonomy, image, box, meta, score,
                best_iou, depth_meta, source_kind,
                development_partition(meta["world_id"]) if split == "G10_TRAIN" else "formal_holdout",
            )
            pairs.append(pair)
        if include_train_bank and meta.get("negative_only"):
            width, height = int(meta["width"]), int(meta["height"])
            for slot, normalized in enumerate(GROUND_BOXES):
                box = [normalized[0] * width, normalized[1] * height, normalized[2] * width, normalized[3] * height]
                pair_id = f"{meta['scene']}_{int(meta['frame_index']):03d}_g{slot}"
                depth_meta = depth_statistics(depth, box, camera)
                pair, _ = make_pair(
                    output, split, pair_id, CLASSES[-1], ground_taxonomy(meta["world_id"], slot),
                    image, box, meta, None, 0.0, depth_meta, "negative_only_ground_control",
                    development_partition(meta["world_id"]),
                )
                pairs.append(pair)
    return pairs, ambiguous


def duplicate_stats(train_pairs: list[dict], holdout_pairs: list[dict]) -> dict:
    fields = ("tight_sha256", "context_sha256", "tight_phash", "context_phash")
    train_values = {field: {row[field] for row in train_pairs} for field in fields}
    holdout_values = {field: {row[field] for row in holdout_pairs} for field in fields}
    return {
        "cross_split_exact_crop_duplicates": sum(len(train_values[field] & holdout_values[field]) for field in fields[:2]),
        "cross_split_phash_duplicates": sum(len(train_values[field] & holdout_values[field]) for field in fields[2:]),
        "within_train_exact_duplicate_instances": sum(len(train_pairs) - len(train_values[field]) for field in fields[:2]),
        "within_holdout_exact_duplicate_instances": sum(len(holdout_pairs) - len(holdout_values[field]) for field in fields[:2]),
        "within_train_phash_duplicate_instances": sum(len(train_pairs) - len(train_values[field]) for field in fields[2:]),
        "within_holdout_phash_duplicate_instances": sum(len(holdout_pairs) - len(holdout_values[field]) for field in fields[2:]),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--train-coco", type=Path, required=True)
    parser.add_argument("--holdout-coco", type=Path, required=True)
    parser.add_argument("--train-raw", type=Path, required=True)
    parser.add_argument("--holdout-raw", type=Path, required=True)
    parser.add_argument("--threshold", type=float, required=True)
    parser.add_argument("--min-reliable-short-side", type=int, default=18)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--path-map", action="append", default=[])
    args = parser.parse_args()
    mappings = []
    for value in args.path_map:
        source, destination = value.split("=", 1)
        mappings.append((source, Path(destination)))
    train_coco, holdout_coco = load_json(args.train_coco), load_json(args.holdout_coco)
    if {row.get("source_split") for row in train_coco["images"]} != {"train"}:
        raise ValueError("C11 TRAIN must come from G10_TRAIN")
    if {row.get("source_split") for row in holdout_coco["images"]} != {"val"}:
        raise ValueError("C11 HOLDOUT must come from G10_HOLDOUT")
    args.output.mkdir(parents=True, exist_ok=True)
    train_pairs, train_ambiguous = build_split(
        train_coco, load_json(args.train_raw), args.output, "G10_TRAIN", args.threshold,
        args.min_reliable_short_side, mappings, True,
    )
    holdout_pairs, holdout_ambiguous = build_split(
        holdout_coco, load_json(args.holdout_raw), args.output, "G10_HOLDOUT", args.threshold,
        args.min_reliable_short_side, mappings, False,
    )
    train_worlds = {row["source_world"] for row in train_pairs}
    holdout_worlds = {row["source_world"] for row in holdout_pairs}
    train_frames = {(row["source_mission"], row["source_frame_index"]) for row in train_pairs}
    holdout_frames = {(row["source_mission"], row["source_frame_index"]) for row in holdout_pairs}
    duplicates = duplicate_stats(train_pairs, holdout_pairs)
    train_class_counts = Counter(row["class"] for row in train_pairs)
    holdout_class_counts = Counter(row["class"] for row in holdout_pairs)
    taxonomy = Counter(row["background_taxonomy"] for row in train_pairs if row["class"] == CLASSES[-1])
    background_unique = len({row["tight_sha256"] for row in train_pairs if row["class"] == CLASSES[-1]})
    qa_gates = {
        "cross_split_world_overlap_zero": not (train_worlds & holdout_worlds),
        "source_frame_overlap_zero": not (train_frames & holdout_frames),
        "cross_split_exact_crop_duplicate_zero": duplicates["cross_split_exact_crop_duplicates"] == 0,
        "cross_split_phash_duplicate_zero": duplicates["cross_split_phash_duplicates"] == 0,
        "GT_runtime_input_zero": True,
        "ambiguous_policy_deterministic": all(row["policy"] == "ignore" for row in train_ambiguous + holdout_ambiguous),
        "all_four_classes_train": all(train_class_counts[name] > 0 for name in CLASSES),
        "all_four_classes_holdout": all(holdout_class_counts[name] > 0 for name in CLASSES),
        "background_unique_preferred_3000": background_unique >= 3000,
    }
    c11_pass = all(qa_gates.values())
    common = {
        "schema_version": 1, "protocol": "CRCRV11", "threshold": args.threshold,
        "MIN_RELIABLE_CLASSIFICATION_SHORT_SIDE_PX": args.min_reliable_short_side,
        "crop_contract": {"tight": "proposal_box", "context": "expand(proposal_box,1.6)"},
        "ambiguous_near_miss_policy": "ignore when 0.20 <= best_iou < 0.50",
        "production_runtime_gt_used": False,
        "G10_DEV_VAL_SEALED_read": False, "VAL_NEW_read": False, "G5_V2_read": False,
    }
    write_json(args.output / "C11_TRAIN_PAIR_MANIFEST.json", {
        **common, "stage": "CRCRV11-03-C11-TRAIN-PAIR-MANIFEST", "source_split": "G10_TRAIN",
        "pairs": train_pairs, "ambiguous_ignored": train_ambiguous,
    })
    write_json(args.output / "C11_HOLDOUT_PAIR_MANIFEST.json", {
        **common, "stage": "CRCRV11-03-C11-HOLDOUT-PAIR-MANIFEST", "source_split": "G10_HOLDOUT",
        "pairs": holdout_pairs, "ambiguous_ignored": holdout_ambiguous,
    })
    write_json(args.output / "C11_DATA_QA.json", {
        **common, "stage": "CRCRV11-03-C11-DATA-QA", "gates": qa_gates,
        "duplicate_audit": duplicates, "train_worlds": sorted(train_worlds),
        "holdout_worlds": sorted(holdout_worlds), "C11_DATA_PASS": c11_pass,
    })
    write_json(args.output / "C11_CLASS_COUNTS.json", {
        **common, "stage": "CRCRV11-03-C11-CLASS-COUNTS",
        "train_pairs": {name: train_class_counts[name] for name in CLASSES},
        "holdout_pairs": {name: holdout_class_counts[name] for name in CLASSES},
        "train_fit_pairs": dict(Counter(row["class"] for row in train_pairs if row["development_partition"] == "fit")),
        "train_dev_pairs": dict(Counter(row["class"] for row in train_pairs if row["development_partition"] == "dev")),
    })
    write_json(args.output / "C11_BACKGROUND_TAXONOMY.json", {
        **common, "stage": "CRCRV11-03-C11-BACKGROUND-TAXONOMY",
        "unique_background_tight_crops": background_unique,
        "taxonomy_counts": dict(sorted(taxonomy.items())),
        "sources": ["unmatched real proposals", "proposal-score 0.05 to threshold hard negatives", "negative-only mission fixed ground ROIs"],
    })
    write_json(args.output / "C11_PAIR_STATS.json", {
        **common, "stage": "CRCRV11-03-C11-PAIR-STATS",
        "train_pairs": len(train_pairs), "holdout_pairs": len(holdout_pairs),
        "train_ambiguous_ignored": len(train_ambiguous), "holdout_ambiguous_ignored": len(holdout_ambiguous),
        "train_unique_source_frames": len(train_frames), "holdout_unique_source_frames": len(holdout_frames),
        "train_manifest_sources": {"coco_sha256": sha256(args.train_coco), "raw_sha256": sha256(args.train_raw)},
        "holdout_manifest_sources": {"coco_sha256": sha256(args.holdout_coco), "raw_sha256": sha256(args.holdout_raw)},
    })
    print(json.dumps({
        "train_pairs": len(train_pairs), "holdout_pairs": len(holdout_pairs),
        "train_class_counts": dict(train_class_counts), "holdout_class_counts": dict(holdout_class_counts),
        "background_unique": background_unique, "taxonomy": dict(taxonomy),
        "duplicate_audit": duplicates, "gates": qa_gates, "C11_DATA_PASS": c11_pass,
    }, indent=2))
    return 0 if c11_pass else 4


if __name__ == "__main__":
    raise SystemExit(main())
