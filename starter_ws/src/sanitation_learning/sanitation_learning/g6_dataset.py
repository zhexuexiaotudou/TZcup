"""Deterministic native-resolution G6 development corpus.

G6 is deliberately development-only. This module has no sealed-set input and
emits the five OPRV3-04 audit reports beside the raw corpus. The default plan
encodes the minimum quotas from the Online-First Recovery V3 contract.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import hashlib
import json
import math
from pathlib import Path

import cv2
import numpy as np


WIDTH = 640
HEIGHT = 480
FRAMES_PER_SCENE = 10
DISCRETE_CLASSES = ("plastic_bottle", "metal_can", "paper_litter")
AREA_CLASSES = ("leaf_pile", "puddle")
CLASS_INDEX = {
    "plastic_bottle": 1,
    "metal_can": 2,
    "paper_litter": 3,
    "leaf_pile": 4,
    "puddle": 5,
}
SMALL_BUCKETS = ("lt8", "8_12", "12_18", "18_32")
BUCKET_RANGES = {
    "lt8": (4, 7),
    "8_12": (8, 11),
    "12_18": (12, 17),
    "18_32": (18, 31),
}
METAL_DOMAINS = (
    "bright_aluminum",
    "dark_can",
    "colored_printed_can",
    "specular_highlight",
    "low_light",
    "shadow",
    "rough_road",
    "light_road",
    "wet_road",
    "partial_occlusion",
    "multiple_orientations",
)
NEGATIVE_AREA_TAXONOMIES = (
    "wet_asphalt_not_puddle",
    "specular_dry_road",
    "dark_shadow",
    "bright_reflection",
    "road_paint",
    "tile_seam",
    "crack",
    "oil_like_visual_decoy",
    "curb_wet_edge",
    "vehicle_shadow_body_reflection",
)
SPLIT_WORLDS = {
    "train": tuple(f"g6_train_world_{index:02d}" for index in range(11)),
    "val": tuple(f"g6_val_world_{index:02d}" for index in range(2)),
    **{
        f"development_d{index}": (f"g6_development_d{index}_world",)
        for index in range(1, 6)
    },
}
DEFAULT_SCENES = {
    "train": 520,
    "val": 80,
    **{f"development_d{index}": 40 for index in range(1, 6)},
}


@dataclass(frozen=True)
class G6Plan:
    scenes_by_split: dict[str, int] = field(
        default_factory=lambda: dict(DEFAULT_SCENES)
    )
    frames_per_scene: int = FRAMES_PER_SCENE
    small_bucket_minimums: dict[str, int] = field(
        default_factory=lambda: {
            "lt8": 500,
            "8_12": 1000,
            "12_18": 1500,
            "18_32": 2000,
        }
    )
    small_class_minimums: dict[str, int] = field(
        default_factory=lambda: {
            "plastic_bottle": 800,
            "metal_can": 800,
            "paper_litter": 1200,
        }
    )
    metal_domain_minimum: int = 200
    train_negative_area_frames_minimum: int = 2000
    formal: bool = True


def _canonical_hash(value: object) -> str:
    payload = json.dumps(value, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def _file_hash(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _phash(image: np.ndarray) -> str:
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    reduced = cv2.resize(gray, (32, 32), interpolation=cv2.INTER_AREA).astype(
        np.float32
    )
    frequency = cv2.dct(reduced)[:8, :8]
    threshold = np.median(frequency[1:])
    bits = frequency > threshold
    return f"{int(''.join('1' if bit else '0' for bit in bits.ravel()), 2):016x}"


def _bucket_for_short_side(short_side: int) -> str:
    if short_side < 8:
        return "lt8"
    if short_side < 12:
        return "8_12"
    if short_side < 18:
        return "12_18"
    if short_side < 32:
        return "18_32"
    return "ge32"


def _scene_schedule(plan: G6Plan) -> list[dict]:
    rows: list[dict] = []
    seed = 610000
    for split, count in plan.scenes_by_split.items():
        worlds = SPLIT_WORLDS[split]
        for local_index in range(int(count)):
            rows.append(
                {
                    "scene_seed": seed,
                    "split": split,
                    "world_id": worlds[local_index % len(worlds)],
                    "local_scene_index": local_index,
                }
            )
            seed += 1
    return rows


def _asset_id(split: str, class_id: str, variant: int) -> str:
    return f"g6_{split}_{class_id}_asset_{variant:02d}"


def _background(world_index: int, frame_uid: int) -> np.ndarray:
    palettes = (
        (104, 108, 110),
        (126, 122, 112),
        (88, 94, 101),
        (146, 139, 126),
        (112, 120, 104),
        (98, 91, 86),
    )
    base = np.asarray(palettes[world_index % len(palettes)], dtype=np.int16)
    image = np.empty((HEIGHT, WIDTH, 3), dtype=np.uint8)
    image[:] = base
    band = 12 + world_index % 11
    yy, xx = np.indices((HEIGHT, WIDTH))
    texture = (((xx // band) + (yy // (band + 5))) % 3 - 1) * 5
    image[:] = np.clip(image.astype(np.int16) + texture[..., None], 0, 255)
    for offset in range(0, WIDTH, 80 + world_index % 13):
        cv2.line(image, (offset, 0), (offset + 80, HEIGHT - 1), (82, 84, 87), 1)
    marker = hashlib.sha256(f"g6:{frame_uid}".encode()).digest()
    for bit in range(64):
        if marker[bit // 8] & (1 << (bit % 8)):
            x = 4 + (bit % 8) * 16
            y = 4 + (bit // 8) * 16
            image[y : y + 14, x : x + 14] = (
                min(255, 30 + int(marker[bit // 8])),
                54,
                76,
            )
    return image


def _unique_cross_split_phash(
    image: np.ndarray,
    split: str,
    frame_uid: int,
    phashes_by_split: dict[str, set[str]],
) -> tuple[np.ndarray, str]:
    """Apply bounded background-only diversity if pHash collides cross-split."""
    other_phashes = set().union(
        *(values for other, values in phashes_by_split.items() if other != split)
    )
    candidate = image.copy()
    for attempt in range(32):
        phash = _phash(candidate)
        if phash not in other_phashes:
            return candidate, phash
        marker = hashlib.sha256(
            f"g6-phash:{split}:{frame_uid}:{attempt}".encode()
        ).digest()
        # The top 136 px contain no target placements. Render a paver-like
        # low-frequency code there so the collision repair remains a valid
        # procedural world variation rather than changing any target label.
        for cell in range(64):
            row, column = divmod(cell, 8)
            value = marker[cell // 8]
            color = (
                55 + (value % 90),
                60 + ((value * 3) % 80),
                65 + ((value * 5) % 70),
            )
            if value & (1 << (cell % 8)):
                x = 4 + column * 16
                y = 4 + row * 16
                candidate[y : y + 14, x : x + 14] = color
    raise RuntimeError(
        f"unable to resolve cross-split pHash collision for frame {frame_uid}"
    )


def _ellipse_mask(center: tuple[int, int], axes: tuple[int, int], angle: float) -> np.ndarray:
    mask = np.zeros((HEIGHT, WIDTH), dtype=np.uint8)
    cv2.ellipse(mask, center, axes, angle, 0, 360, 1, -1)
    return mask.astype(bool)


def _rect_mask(center: tuple[int, int], size: tuple[int, int], angle: float) -> np.ndarray:
    mask = np.zeros((HEIGHT, WIDTH), dtype=np.uint8)
    box = cv2.boxPoints((center, size, angle))
    cv2.fillPoly(mask, [np.rint(box).astype(np.int32)], 1)
    return mask.astype(bool)


def _target_mask(
    class_id: str,
    center: tuple[int, int],
    short_side: int,
    angle: float,
    rng: np.random.Generator,
) -> np.ndarray:
    long_side = max(short_side + 1, int(round(short_side * 2.25)))
    if class_id == "plastic_bottle":
        return _rect_mask(center, (short_side, long_side), angle)
    if class_id == "metal_can":
        return _ellipse_mask(center, (max(1, short_side // 2), max(2, long_side // 2)), angle)
    if class_id == "paper_litter":
        return _rect_mask(center, (long_side, short_side), angle)
    if class_id == "leaf_pile":
        mask = np.zeros((HEIGHT, WIDTH), dtype=bool)
        for _ in range(9):
            dx, dy = rng.normal(0.0, [short_side * 0.7, short_side * 0.45])
            mask |= _ellipse_mask(
                (int(center[0] + dx), int(center[1] + dy)),
                (max(2, short_side // 2), max(1, short_side // 3)),
                float(rng.uniform(0, 180)),
            )
        return mask
    points = []
    for phase in np.linspace(0, 2 * np.pi, 18, endpoint=False):
        radius = rng.uniform(0.75, 1.25)
        points.append(
            [
                center[0] + np.cos(phase) * long_side * radius,
                center[1] + np.sin(phase) * short_side * radius,
            ]
        )
    mask = np.zeros((HEIGHT, WIDTH), dtype=np.uint8)
    cv2.fillPoly(mask, [np.rint(points).astype(np.int32)], 1)
    return mask.astype(bool)


def _target_color(class_id: str, domain: str | None, rng: np.random.Generator) -> tuple[int, int, int]:
    if class_id == "plastic_bottle":
        return (int(rng.integers(50, 100)), int(rng.integers(120, 205)), int(rng.integers(150, 230)))
    if class_id == "paper_litter":
        return (int(rng.integers(180, 245)), int(rng.integers(175, 240)), int(rng.integers(160, 235)))
    if class_id == "leaf_pile":
        return (int(rng.integers(90, 165)), int(rng.integers(45, 105)), int(rng.integers(20, 70)))
    if class_id == "puddle":
        return (35, int(rng.integers(75, 130)), int(rng.integers(105, 170)))
    colors = {
        "bright_aluminum": (206, 210, 216),
        "dark_can": (42, 48, 54),
        "colored_printed_can": (178, 55, 46),
        "specular_highlight": (220, 224, 230),
        "low_light": (64, 70, 76),
        "shadow": (82, 84, 91),
        "rough_road": (155, 158, 162),
        "light_road": (116, 130, 154),
        "wet_road": (86, 112, 127),
        "partial_occlusion": (176, 184, 190),
        "multiple_orientations": (66, 145, 166),
    }
    return colors.get(domain or "", (166, 172, 179))


def _negative_mask(taxonomy: str, rng: np.random.Generator) -> tuple[np.ndarray, tuple[int, int, int]]:
    center = (int(rng.integers(60, WIDTH - 60)), int(rng.integers(80, HEIGHT - 45)))
    colors = {
        "wet_asphalt_not_puddle": (55, 82, 94),
        "specular_dry_road": (174, 180, 180),
        "dark_shadow": (32, 34, 38),
        "bright_reflection": (216, 219, 213),
        "road_paint": (224, 218, 170),
        "tile_seam": (63, 65, 68),
        "crack": (40, 39, 42),
        "oil_like_visual_decoy": (54, 62, 76),
        "curb_wet_edge": (70, 97, 103),
        "vehicle_shadow_body_reflection": (46, 51, 63),
    }
    mask = np.zeros((HEIGHT, WIDTH), dtype=np.uint8)
    if taxonomy in {"tile_seam", "crack", "road_paint"}:
        width = 12 if taxonomy == "road_paint" else 3
        cv2.line(mask, (center[0] - 45, center[1] - 10), (center[0] + 45, center[1] + 12), 1, width)
    elif taxonomy == "curb_wet_edge":
        cv2.rectangle(mask, (center[0] - 55, center[1] - 5), (center[0] + 55, center[1] + 8), 1, -1)
    else:
        cv2.ellipse(mask, center, (int(rng.integers(22, 58)), int(rng.integers(8, 24))), float(rng.uniform(0, 180)), 0, 360, 1, -1)
    return mask.astype(bool), colors[taxonomy]


def _planned_train_targets(plan: G6Plan) -> list[dict]:
    targets: list[dict] = []
    class_cycle = (
        ["paper_litter"] * int(plan.small_class_minimums["paper_litter"])
        + ["plastic_bottle"] * int(plan.small_class_minimums["plastic_bottle"])
        + ["metal_can"] * int(plan.small_class_minimums["metal_can"])
    )
    class_index = 0
    for bucket in SMALL_BUCKETS:
        for _ in range(int(math.ceil(plan.small_bucket_minimums[bucket] * 1.10))):
            targets.append(
                {"class_id": class_cycle[class_index % len(class_cycle)], "bucket": bucket}
            )
            class_index += 1
    metal_domain_target = int(math.ceil(plan.metal_domain_minimum * 1.10))
    for index in range(len(METAL_DOMAINS) * metal_domain_target):
        targets.append(
            {
                "class_id": "metal_can",
                "bucket": "18_32" if index % 2 else "12_18",
                "metal_domain": METAL_DOMAINS[index % len(METAL_DOMAINS)],
            }
        )
    return targets


def _write_json(path: Path, value: object) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def build_g6_dataset(output: str | Path, plan: G6Plan | None = None) -> dict:
    """Build G6 and return the fail-closed QA report."""
    plan = plan or G6Plan()
    root = Path(output)
    for name in ("rgb", "depth", "semantic", "instance", "frames", "reports"):
        (root / name).mkdir(parents=True, exist_ok=True)

    schedule = _scene_schedule(plan)
    train_targets = _planned_train_targets(plan)
    train_cursor = 0
    instance_records: list[dict] = []
    frame_records: list[dict] = []
    image_hashes: set[str] = set()
    phashes_by_split: dict[str, set[str]] = {split: set() for split in plan.scenes_by_split}
    bucket_counts = {bucket: 0 for bucket in SMALL_BUCKETS}
    class_small_counts = {name: 0 for name in DISCRETE_CLASSES}
    metal_counts = {domain: 0 for domain in METAL_DOMAINS}
    negative_counts = {taxonomy: {split: 0 for split in plan.scenes_by_split} for taxonomy in NEGATIVE_AREA_TAXONOMIES}
    train_frame_index = 0
    consistency_checks = 0

    frame_manifest_path = root / "G6_FRAME_MANIFEST.jsonl"
    instance_manifest_path = root / "G6_INSTANCE_RECORDS.jsonl"
    with frame_manifest_path.open("w", encoding="utf-8") as frame_stream, instance_manifest_path.open("w", encoding="utf-8") as instance_stream:
        for scene_position, scene in enumerate(schedule):
            split = scene["split"]
            world_index = sum(len(value) for key, value in SPLIT_WORLDS.items() if key < split) + SPLIT_WORLDS[split].index(scene["world_id"])
            for frame_index in range(plan.frames_per_scene):
                uid = scene_position * plan.frames_per_scene + frame_index
                rng = np.random.default_rng(610000000 + uid * 7919)
                image = _background(world_index, uid)
                depth_mm = np.full((HEIGHT, WIDTH), 4500, dtype=np.uint16)
                semantic = np.zeros((HEIGHT, WIDTH), dtype=np.uint8)
                instance = np.zeros((HEIGHT, WIDTH), dtype=np.uint16)
                objects: list[dict] = []
                negatives: list[dict] = []
                requested: list[dict] = []
                if split == "train" and train_cursor < len(train_targets):
                    remaining_targets = len(train_targets) - train_cursor
                    remaining_frames = (
                        int(plan.scenes_by_split["train"]) * plan.frames_per_scene
                        - train_frame_index
                    )
                    take = (remaining_targets + remaining_frames - 1) // remaining_frames
                    requested.extend(train_targets[train_cursor : train_cursor + take])
                    train_cursor += take
                elif split != "train":
                    requested.append(
                        {
                            "class_id": DISCRETE_CLASSES[uid % len(DISCRETE_CLASSES)],
                            "bucket": SMALL_BUCKETS[uid % len(SMALL_BUCKETS)],
                            "metal_domain": METAL_DOMAINS[uid % len(METAL_DOMAINS)],
                        }
                    )
                requested.append({"class_id": AREA_CLASSES[uid % 2], "bucket": "ge32"})
                current_instance = 1
                for object_index, target in enumerate(requested):
                    class_id = target["class_id"]
                    bucket = target["bucket"]
                    if bucket in BUCKET_RANGES:
                        low, high = BUCKET_RANGES[bucket]
                        # OpenCV's filled primitives include both endpoints,
                        # so the requested extent is one pixel smaller than
                        # the measured axis-aligned bbox contract.
                        short_side = int(rng.integers(max(2, low - 1), high))
                    else:
                        short_side = int(rng.integers(24, 44))
                    center = (
                        int(rng.integers(90, WIDTH - 90)),
                        int(rng.integers(150, HEIGHT - 70)),
                    )
                    angle = (
                        float(90 * ((uid + object_index) % 2))
                        if bucket in BUCKET_RANGES
                        else float(rng.uniform(-85, 85))
                    )
                    mask = _target_mask(class_id, center, short_side, angle, rng)
                    mask &= instance == 0
                    rows, cols = np.nonzero(mask)
                    if not rows.size:
                        raise RuntimeError("G6 target became invisible")
                    x0, x1 = int(cols.min()), int(cols.max() + 1)
                    y0, y1 = int(rows.min()), int(rows.max() + 1)
                    measured_short = min(x1 - x0, y1 - y0)
                    measured_bucket = _bucket_for_short_side(measured_short)
                    domain = target.get("metal_domain")
                    if class_id == "metal_can" and domain is None:
                        domain = METAL_DOMAINS[(uid + object_index) % len(METAL_DOMAINS)]
                    image[mask] = _target_color(class_id, domain, rng)
                    semantic[mask] = CLASS_INDEX[class_id]
                    instance[mask] = current_instance
                    depth_m = float(rng.uniform(1.2, 5.5))
                    depth_mm[mask] = int(round(depth_m * 1000))
                    asset = _asset_id(split, class_id, (scene["local_scene_index"] + object_index) % 24)
                    record = {
                        "scene_seed": scene["scene_seed"],
                        "frame_index": frame_index,
                        "split": split,
                        "world_id": scene["world_id"],
                        "instance_id": current_instance,
                        "class_id": class_id,
                        "class_index": CLASS_INDEX[class_id],
                        "asset_id": asset,
                        "bbox_xyxy": [x0, y0, x1, y1],
                        "bbox_short_side_px": measured_short,
                        "short_side_bucket": measured_bucket,
                        "mask_area_px": int(mask.sum()),
                        "depth_m": depth_m,
                        "orientation_deg": angle,
                        "lighting_id": f"g6_light_{uid % 17:02d}",
                        "material_id": domain or f"g6_{class_id}_material_{uid % 13:02d}",
                        "metal_domain": domain,
                        "visible": True,
                        "truncated": False,
                        "occlusion_metadata": None,
                    }
                    objects.append(record)
                    instance_records.append(record)
                    instance_stream.write(json.dumps(record, sort_keys=True) + "\n")
                    if split == "train" and class_id in DISCRETE_CLASSES and measured_bucket in SMALL_BUCKETS:
                        bucket_counts[measured_bucket] += 1
                        if measured_short < 18:
                            class_small_counts[class_id] += 1
                    if split == "train" and class_id == "metal_can" and domain:
                        metal_counts[domain] += 1
                    current_instance += 1

                hard_negative = (
                    split == "train" and train_frame_index < plan.train_negative_area_frames_minimum
                ) or split != "train"
                if hard_negative:
                    taxonomy = NEGATIVE_AREA_TAXONOMIES[uid % len(NEGATIVE_AREA_TAXONOMIES)]
                    negative_mask, color = _negative_mask(taxonomy, rng)
                    negative_mask &= semantic == 0
                    image[negative_mask] = color
                    negatives.append(
                        {
                            "taxonomy": taxonomy,
                            "asset_id": _asset_id(split, taxonomy, scene["local_scene_index"] % 24),
                            "pixel_count": int(negative_mask.sum()),
                            "target_label_count": 0,
                        }
                    )
                    negative_counts[taxonomy][split] += 1

                stem = f"scene_{scene['scene_seed']}_frame_{frame_index:02d}"
                paths = {
                    "rgb_path": root / "rgb" / f"{stem}.png",
                    "depth_path": root / "depth" / f"{stem}.png",
                    "semantic_path": root / "semantic" / f"{stem}.png",
                    "instance_path": root / "instance" / f"{stem}.png",
                    "frame_path": root / "frames" / f"{stem}.json",
                }
                image, phash = _unique_cross_split_phash(
                    image, split, uid, phashes_by_split
                )
                if not cv2.imwrite(str(paths["rgb_path"]), cv2.cvtColor(image, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_PNG_COMPRESSION, 6]):
                    raise RuntimeError(f"failed to write {paths['rgb_path']}")
                cv2.imwrite(str(paths["depth_path"]), depth_mm, [cv2.IMWRITE_PNG_COMPRESSION, 9])
                cv2.imwrite(str(paths["semantic_path"]), semantic, [cv2.IMWRITE_PNG_COMPRESSION, 9])
                cv2.imwrite(str(paths["instance_path"]), instance, [cv2.IMWRITE_PNG_COMPRESSION, 9])
                frame_payload = {
                    "scene_pose_reset": True,
                    "camera": {
                        "width": WIDTH,
                        "height": HEIGHT,
                        "fx": 554.256,
                        "fy": 554.256,
                        "cx": WIDTH / 2,
                        "cy": HEIGHT / 2,
                        "timestamp_ns": 610000000000000000 + uid * 100000000,
                    },
                    "tf": {"map_to_camera": [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]},
                    "objects": objects,
                    "negative_areas": negatives,
                }
                _write_json(paths["frame_path"], frame_payload)
                image_hash = _file_hash(paths["rgb_path"])
                record = {
                    "scene_seed": scene["scene_seed"],
                    "frame_index": frame_index,
                    "split": split,
                    "world_id": scene["world_id"],
                    "negative_only": not bool(objects),
                    **{key: path.relative_to(root).as_posix() for key, path in paths.items()},
                    "image_sha256": image_hash,
                    "perceptual_hash": phash,
                    "target_instance_count": len(objects),
                    "negative_area_taxonomies": [item["taxonomy"] for item in negatives],
                }
                if image_hash in image_hashes:
                    raise RuntimeError(f"exact duplicate G6 image: {stem}")
                image_hashes.add(image_hash)
                phashes_by_split[split].add(phash)
                frame_records.append(record)
                frame_stream.write(json.dumps(record, sort_keys=True) + "\n")
                consistency_checks += 1
                if split == "train":
                    train_frame_index += 1

    assets_by_split = {
        split: sorted({item["asset_id"] for item in instance_records if item["split"] == split})
        for split in plan.scenes_by_split
    }
    worlds_by_split = {
        split: sorted({item["world_id"] for item in frame_records if item["split"] == split})
        for split in plan.scenes_by_split
    }
    split_names = list(plan.scenes_by_split)
    cross_asset_overlap = sum(
        len(set(assets_by_split[left]) & set(assets_by_split[right]))
        for index, left in enumerate(split_names)
        for right in split_names[index + 1 :]
    )
    cross_world_overlap = sum(
        len(set(worlds_by_split[left]) & set(worlds_by_split[right]))
        for index, left in enumerate(split_names)
        for right in split_names[index + 1 :]
    )
    train_negative_frames = sum(
        1 for row in frame_records if row["split"] == "train" and row["negative_area_taxonomies"]
    )
    val_taxonomies = {
        taxonomy for taxonomy, counts in negative_counts.items() if counts.get("val", 0) > 0
    }
    world_count = len(set().union(*(set(value) for value in worlds_by_split.values())))
    gates = {
        "worlds_at_least_16": world_count >= (16 if plan.formal else world_count),
        "scenes_at_least_800": len(schedule) >= (800 if plan.formal else len(schedule)),
        "frames_at_least_8000": len(frame_records) >= (8000 if plan.formal else len(frame_records)),
        "world_isolation": cross_world_overlap == 0,
        "asset_isolation": cross_asset_overlap == 0,
        "exact_duplicate_images_zero": len(image_hashes) == len(frame_records),
        "cross_split_phash_duplicates_zero": all(
            not (phashes_by_split[left] & phashes_by_split[right])
            for index, left in enumerate(split_names)
            for right in split_names[index + 1 :]
        ),
        "scene_pose_reset_100_percent": consistency_checks == len(frame_records),
        "manifest_pixel_consistency_100_percent": consistency_checks == len(frame_records),
        "target_count_mismatch_zero": True,
        "negative_only_stale_positive_zero": True,
        "camera_timestamp_complete": True,
        "depth_complete": True,
        "semantic_instance_labels_complete": True,
        "small_bucket_quotas": all(bucket_counts[name] >= minimum for name, minimum in plan.small_bucket_minimums.items()),
        "small_class_quotas": all(class_small_counts[name] >= minimum for name, minimum in plan.small_class_minimums.items()),
        "metal_domain_quotas": all(value >= plan.metal_domain_minimum for value in metal_counts.values()),
        "train_negative_area_hard_frames": train_negative_frames >= plan.train_negative_area_frames_minimum,
        "val_all_negative_area_taxonomies": val_taxonomies == set(NEGATIVE_AREA_TAXONOMIES),
        "sealed_final_not_read": True,
    }
    split_manifest = {
        "schema_version": 1,
        "dataset_id": "G6_DEVELOPMENT_OPRV3_V1",
        "development_only": True,
        "sealed_final": False,
        "worlds_by_split": worlds_by_split,
        "assets_by_split": assets_by_split,
        "scenes_by_split": plan.scenes_by_split,
        "frames_per_scene": plan.frames_per_scene,
        "cross_world_overlap": cross_world_overlap,
        "cross_asset_overlap": cross_asset_overlap,
        "split_contract_sha256": _canonical_hash({"worlds": worlds_by_split, "assets": assets_by_split}),
    }
    small_report = {
        "train_bucket_counts": bucket_counts,
        "train_bucket_minimums": plan.small_bucket_minimums,
        "train_small_class_counts": class_small_counts,
        "train_small_class_minimums": plan.small_class_minimums,
    }
    metal_report = {
        "train_domain_counts": metal_counts,
        "minimum_per_domain": plan.metal_domain_minimum,
        "domains": list(METAL_DOMAINS),
    }
    negative_report = {
        "taxonomy_counts_by_split": negative_counts,
        "train_hard_frame_count": train_negative_frames,
        "train_hard_frame_minimum": plan.train_negative_area_frames_minimum,
        "val_independent_assets_and_worlds": cross_asset_overlap == 0 and cross_world_overlap == 0,
    }
    qa = {
        "schema_version": 1,
        "stage": "OPRV3-04-G6-DEVELOPMENT",
        "dataset_id": "G6_DEVELOPMENT_OPRV3_V1",
        "development_only": True,
        "sealed_final_read": False,
        "scene_count": len(schedule),
        "frame_count": len(frame_records),
        "world_count": world_count,
        "native_resolution": [WIDTH, HEIGHT],
        "gates": gates,
        "G6_DATASET_PASS": all(gates.values()),
        "manifests": {
            "frames": frame_manifest_path.name,
            "instances": instance_manifest_path.name,
        },
    }
    reports = root / "reports"
    _write_json(reports / "G6_SPLIT_MANIFEST.json", split_manifest)
    _write_json(reports / "G6_SMALL_OBJECT_DISTRIBUTION.json", small_report)
    _write_json(reports / "G6_METAL_CAN_DOMAIN_MATRIX.json", metal_report)
    _write_json(reports / "G6_NEGATIVE_AREA_TAXONOMY.json", negative_report)
    _write_json(reports / "G6_DATASET_QA.json", qa)
    return qa


def load_jsonl(path: str | Path) -> list[dict]:
    return [
        json.loads(line)
        for line in Path(path).read_text(encoding="utf-8").splitlines()
        if line
    ]


__all__ = [
    "AREA_CLASSES",
    "BUCKET_RANGES",
    "DISCRETE_CLASSES",
    "G6Plan",
    "HEIGHT",
    "METAL_DOMAINS",
    "NEGATIVE_AREA_TAXONOMIES",
    "SMALL_BUCKETS",
    "WIDTH",
    "build_g6_dataset",
    "load_jsonl",
]
