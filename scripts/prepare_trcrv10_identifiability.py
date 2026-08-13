#!/usr/bin/env python3
"""Build TRAIN_DIAG/HOLDOUT_DIAG GT-crop datasets for asset identifiability.

This is evaluator-only development tooling.  It intentionally rejects sealed
or product-route captures and never emits a production runtime manifest.
"""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np


LABELS = {1: "plastic_bottle", 2: "metal_can", 3: "paper_litter"}
ALLOWED_SPLITS = {"TRAIN_DIAG", "HOLDOUT_DIAG"}
CONTEXT_SCALE = 1.6


def read(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def write(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def phash(image: np.ndarray) -> str:
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    spectrum = cv2.dct(np.float32(cv2.resize(gray, (32, 32), interpolation=cv2.INTER_AREA)))[:8, :8]
    bits = spectrum > np.median(spectrum[1:])
    return f"{sum(int(value) << index for index, value in enumerate(bits.flat)):016x}"


def bucket(short_side: int) -> str:
    if short_side < 12:
        return "lt12"
    if short_side < 18:
        return "12_18"
    if short_side < 32:
        return "18_32"
    if short_side < 48:
        return "32_48"
    if short_side < 64:
        return "48_64"
    if short_side < 96:
        return "64_96"
    return "ge96"


def bbox(mask: np.ndarray, label: int) -> tuple[int, int, int, int] | None:
    ys, xs = np.where(mask == label)
    if not len(xs):
        return None
    return int(xs.min()), int(ys.min()), int(xs.max() + 1), int(ys.max() + 1)


def expanded_box(box: tuple[int, int, int, int], width: int, height: int) -> tuple[int, int, int, int]:
    x0, y0, x1, y1 = box
    center_x, center_y = (x0 + x1) / 2.0, (y0 + y1) / 2.0
    target_width = max(x1 - x0, int(round((x1 - x0) * CONTEXT_SCALE)))
    target_height = max(y1 - y0, int(round((y1 - y0) * CONTEXT_SCALE)))
    return (
        max(0, int(round(center_x - target_width / 2))),
        max(0, int(round(center_y - target_height / 2))),
        min(width, int(round(center_x + target_width / 2))),
        min(height, int(round(center_y + target_height / 2))),
    )


def find_target(manifest: dict) -> dict:
    targets = [row for row in manifest["objects"] if row["class_id"] in set(LABELS.values())]
    if len(targets) != 1:
        raise ValueError(f"identifiability scene requires one target, found {len(targets)}")
    return targets[0]


def build_scene(scene: Path, split: str, output: Path) -> list[dict]:
    manifest = read(scene / "scene_manifest.json")
    diagnostic = manifest.get("trcrv10_identifiability_diagnostic", {})
    if diagnostic.get("enabled") is not True or diagnostic.get("production_runtime_eligible") is not False:
        raise ValueError(f"not an isolated identifiability diagnostic: {scene}")
    if diagnostic.get("GT_runtime_forbidden") is not True:
        raise ValueError(f"GT runtime prohibition missing: {scene}")
    report = read(scene / "capture_report.json")
    if report.get("capture_pass") is not True:
        raise ValueError(f"capture failed: {scene}")
    target = find_target(manifest)
    label = next(key for key, value in LABELS.items() if value == target["class_id"])
    rows = []
    for record in report["records"]:
        index = int(record["frame_index"])
        mask = np.load(scene / record["paths"]["semantic"])
        box = bbox(mask, label)
        if box is None:
            continue
        x0, y0, x1, y1 = box
        short = min(x1 - x0, y1 - y0)
        if short < 4:
            continue
        image = cv2.imread(str(scene / record["paths"]["rgb"]), cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError(f"unreadable RGB: {scene / record['paths']['rgb']}")
        for view, crop_box in (("tight", box), ("context", expanded_box(box, image.shape[1], image.shape[0]))):
            cx0, cy0, cx1, cy1 = crop_box
            crop = image[cy0:cy1, cx0:cx1]
            relative = Path(split.lower()) / target["class_id"] / view / f"{manifest['world_id']}__{scene.name}__f{index:03d}.png"
            path = output / "crops" / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            if not cv2.imwrite(str(path), crop):
                raise RuntimeError(f"failed to write crop: {path}")
            encoded = path.read_bytes()
            rows.append({
                "split": split,
                "class_id": target["class_id"],
                "view": view,
                "world_id": manifest["world_id"],
                "scene_seed": manifest["scene_seed"],
                "frame_index": index,
                "asset_id": target["asset_id"],
                "bbox_xyxy": list(box),
                "crop_bbox_xyxy": list(crop_box),
                "short_side_px": short,
                "size_bucket": bucket(short),
                "path": str(Path("crops") / relative).replace("\\", "/"),
                "sha256": sha256_bytes(encoded),
                "phash": phash(crop),
                "GT_role": "offline_diagnostic_evaluator_only",
                "production_runtime_eligible": False,
            })
    return rows


def overlaps(rows: list[dict], key: str) -> list[list[dict]]:
    seen, found = {}, []
    for row in rows:
        prior = seen.get(row[key])
        if prior and prior["split"] != row["split"]:
            found.append([prior, row])
        else:
            seen.setdefault(row[key], row)
    return found


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--capture", action="append", required=True, help="TRAIN_DIAG=path")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    rows = []
    inputs = {}
    for value in args.capture:
        split, raw_path = value.split("=", 1)
        if split not in ALLOWED_SPLITS:
            raise ValueError(f"sealed/product split denied: {split}")
        root = Path(raw_path)
        inputs[split] = str(root.resolve())
        scenes = root / "g4_screening_native" / "scenes"
        for scene in sorted(path for path in scenes.glob("scene_*") if (path / "capture_report.json").is_file()):
            rows.extend(build_scene(scene, split, args.output))
    exact, visual = overlaps(rows, "sha256"), overlaps(rows, "phash")
    counts = Counter((row["split"], row["class_id"], row["view"], row["size_bucket"]) for row in rows)
    count_table: dict[str, dict] = defaultdict(dict)
    for (split, class_id, view, size), count in sorted(counts.items()):
        count_table[split][f"{class_id}/{view}/{size}"] = count
    world_sets = {split: {row["world_id"] for row in rows if row["split"] == split} for split in ALLOWED_SPLITS}
    seed_sets = {split: {row["scene_seed"] for row in rows if row["split"] == split} for split in ALLOWED_SPLITS}
    asset_sets = {split: {row["asset_id"] for row in rows if row["split"] == split} for split in ALLOWED_SPLITS}
    gates = {
        "both_splits_present": all(any(row["split"] == split for row in rows) for split in ALLOWED_SPLITS),
        "world_overlap_zero": not world_sets["TRAIN_DIAG"] & world_sets["HOLDOUT_DIAG"],
        "seed_overlap_zero": not seed_sets["TRAIN_DIAG"] & seed_sets["HOLDOUT_DIAG"],
        "asset_overlap_zero": not asset_sets["TRAIN_DIAG"] & asset_sets["HOLDOUT_DIAG"],
        "exact_crop_overlap_zero": not exact,
        "phash_crop_overlap_zero": not visual,
        "GT_product_input_violation_zero": all(row["production_runtime_eligible"] is False for row in rows),
    }
    qa = {
        "schema_version": 1,
        "protocol": "TRCRV10-01",
        "inputs": inputs,
        "views": {"tight": 1.0, "context": CONTEXT_SCALE},
        "counts": count_table,
        "world_overlap": sorted(world_sets["TRAIN_DIAG"] & world_sets["HOLDOUT_DIAG"]),
        "seed_overlap": sorted(seed_sets["TRAIN_DIAG"] & seed_sets["HOLDOUT_DIAG"]),
        "asset_overlap": sorted(asset_sets["TRAIN_DIAG"] & asset_sets["HOLDOUT_DIAG"]),
        "cross_split_duplicate_counts": {"exact": len(exact), "phash": len(visual)},
        "gates": gates,
        "IDENTIFIABILITY_DATASET_PASS": bool(rows) and all(gates.values()),
    }
    write(args.output / "IDENTIFIABILITY_CROP_MANIFEST.json", {"schema_version": 1, "rows": rows})
    write(args.output / "IDENTIFIABILITY_DATASET_QA.json", qa)
    print(json.dumps(qa, indent=2))
    return 0 if qa["IDENTIFIABILITY_DATASET_PASS"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
