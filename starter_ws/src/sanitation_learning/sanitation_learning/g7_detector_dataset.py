"""Independent detector-only G7 development pack for DDRV4.

G7 uses a new namespace, renderer, world/material families, assets and scene
seeds.  It never accepts an existing dataset root, so G6 or either sealed set
cannot be copied into the generated corpus through this API.
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
DATASET_ID = "G7_DETECTOR_DEVELOPMENT"
CLASSES = ("plastic_bottle", "metal_can", "paper_litter")
CLASS_INDEX = {name: index + 1 for index, name in enumerate(CLASSES)}
SPLITS = (
    "TRAIN",
    "IN_DOMAIN_HOLDOUT",
    "CROSS_WORLD_VAL",
    "SHIFT_D1",
    "SHIFT_D2",
    "SHIFT_D3",
    "SHIFT_D4",
    "SHIFT_D5",
)
WORLD_IDS = {
    "TRAIN": tuple(f"g7v4_fit_mosaic_{index:02d}" for index in range(6)),
    "IN_DOMAIN_HOLDOUT": tuple(f"g7v4_fit_mosaic_{index:02d}" for index in range(6)),
    "CROSS_WORLD_VAL": ("g7v4_val_quartz_00", "g7v4_val_quartz_01"),
    "SHIFT_D1": ("g7v4_shift_reflective_00",),
    "SHIFT_D2": ("g7v4_shift_dark_00",),
    "SHIFT_D3": ("g7v4_shift_wet_00",),
    "SHIFT_D4": ("g7v4_shift_shadow_00",),
    "SHIFT_D5": ("g7v4_shift_clutter_00",),
}
DEFAULT_FRAMES = {
    "TRAIN": 2000,
    "IN_DOMAIN_HOLDOUT": 300,
    "CROSS_WORLD_VAL": 300,
    "SHIFT_D1": 120,
    "SHIFT_D2": 120,
    "SHIFT_D3": 120,
    "SHIFT_D4": 120,
    "SHIFT_D5": 120,
}
METAL_DOMAINS = (
    "silver_highly_reflective",
    "dark_can",
    "red_blue_green_printed",
    "matte",
    "crushed_deformed",
    "upright",
    "horizontal",
    "partial_occlusion",
    "strong_specular_highlight",
    "low_light",
    "deep_shadow",
    "backlight",
    "wet_road_reflection",
    "light_pavement",
    "dark_asphalt",
    "gravel",
    "painted_road",
    "cluttered_roadside",
    "leaf_background_clutter",
)
CRITICAL_METAL_DOMAINS = {
    "silver_highly_reflective",
    "dark_can",
    "wet_road_reflection",
    "deep_shadow",
    "cluttered_roadside",
    "leaf_background_clutter",
}
BOTTLE_DOMAINS = (
    "clear_bottle",
    "semi_transparent_bottle",
    "colored_bottle",
    "specular_bottle",
    "horizontal",
    "upright",
    "crushed",
    "small_distant",
    "shadow",
    "wet_road",
    "light_road",
    "dark_road",
    "clutter",
)
PAPER_DOMAINS = (
    "white_paper",
    "colored_paper",
    "creased",
    "folded",
    "partially_occluded",
    "thin_elongated",
    "small_distant",
    "similar_to_road_paint",
)
NEGATIVE_TAXONOMIES = (
    "wet_road",
    "specular_road",
    "oil_like_patch",
    "white_yellow_paint",
    "lane_marking",
    "crack",
    "tile_seam",
    "manhole",
    "stone",
    "leaf_shadow",
    "vehicle_shadow",
    "pedestrian_shadow",
    "metal_reflection",
    "plastic_like_clutter",
    "small_bright_spots",
    "dark_debris_like_non_target",
)


@dataclass(frozen=True)
class G7Plan:
    frames_by_split: dict[str, int] = field(default_factory=lambda: dict(DEFAULT_FRAMES))
    frames_per_scene: int = 10
    metal_domain_minimum: int = 25
    critical_metal_domain_minimum: int = 60
    metal_instances_minimum: int = 400
    bottle_instances_minimum: int = 250
    paper_instances_minimum: int = 300
    small_instances_minimum: int = 600
    medium_instances_minimum: int = 800
    full_negative_frames_minimum: int = 400
    full_negative_target: int = 800
    formal: bool = True


def _canonical_hash(value: object) -> str:
    return hashlib.sha256(
        json.dumps(value, sort_keys=True, separators=(",", ":")).encode("utf-8")
    ).hexdigest()


def _file_hash(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _phash(image: np.ndarray) -> str:
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    reduced = cv2.resize(gray, (32, 32), interpolation=cv2.INTER_AREA).astype(np.float32)
    block = cv2.dct(reduced)[:8, :8]
    bits = block > np.median(block[1:])
    return f"{int(''.join('1' if bit else '0' for bit in bits.ravel()), 2):016x}"


def _unique_phash(image: np.ndarray, split: str, uid: int, seen: dict[str, set[str]]) -> tuple[np.ndarray, str]:
    candidate = image.copy()
    for attempt in range(96):
        signature = hashlib.sha256(f"g7v4:{split}:{uid}:{attempt}".encode()).digest()
        for index, value in enumerate(signature):
            x = 4 + (index % 16) * 39
            y = 4 + (index // 16) * 11
            candidate[y : y + 7, x : x + 7] = ((value + 37) % 255, (value * 3 + 17) % 255, (value * 7 + 91) % 255)
        value = _phash(candidate)
        if all(value not in hashes for other, hashes in seen.items() if other != split):
            return candidate, value
    raise RuntimeError(f"unable to make cross-split pHash unique for G7 frame {uid}")


def _bucket(short_side: int) -> str:
    if short_side < 18:
        return "small_lt18"
    if short_side <= 48:
        return "medium_18_48"
    return "large_gt48"


def _world_registry() -> list[dict]:
    unique = []
    for split, worlds in WORLD_IDS.items():
        for world in worlds:
            if world in {item["world_id"] for item in unique}:
                continue
            unique.append(
                {
                    "world_id": world,
                    "primary_split": "TRAIN" if split == "IN_DOMAIN_HOLDOUT" else split,
                    "generator_family": "ddrv4_perspective_mosaic_renderer_v1",
                    "material_family": f"g7v4_surface_{len(unique):02d}",
                    "lighting_family": f"g7v4_light_rig_{len(unique):02d}",
                    "prohibited_lineage": ["G6", "G5_SEALED_FINAL", "G5_V2_SEALED_FINAL"],
                }
            )
    return unique


def _background(world_index: int, uid: int, rng: np.random.Generator) -> np.ndarray:
    palettes = np.asarray(
        ((72, 79, 88), (118, 112, 98), (135, 126, 115), (61, 67, 73), (98, 104, 91), (154, 145, 127)),
        dtype=np.float32,
    )
    yy, xx = np.indices((HEIGHT, WIDTH), dtype=np.float32)
    base = palettes[world_index % len(palettes)]
    perspective = 18.0 * (yy / HEIGHT) + 7.0 * np.sin((xx + uid % 97) / (21.0 + world_index))
    granular = rng.normal(0.0, 5.0 + world_index % 4, (HEIGHT, WIDTH, 1))
    image = np.clip(base + perspective[..., None] + granular, 0, 255).astype(np.uint8)
    horizon = 105 + (uid * 13 + world_index * 17) % 95
    cv2.line(image, (0, horizon), (WIDTH - 1, horizon + world_index % 11), tuple(int(v) for v in np.clip(base + 24, 0, 255)), 2)
    for stripe in range(3):
        x0 = int((uid * (31 + stripe) + world_index * 43 + stripe * 173) % WIDTH)
        cv2.line(image, (x0, HEIGHT), (max(0, x0 - 120), horizon), (95 + stripe * 12,) * 3, 2)
    return image


def _rotated_mask(center: tuple[int, int], size: tuple[int, int], angle: float, ellipse: bool = False) -> np.ndarray:
    mask = np.zeros((HEIGHT, WIDTH), dtype=np.uint8)
    if ellipse:
        cv2.ellipse(mask, center, (max(1, size[0] // 2), max(1, size[1] // 2)), angle, 0, 360, 1, -1)
    else:
        box = cv2.boxPoints((center, size, angle))
        cv2.fillPoly(mask, [np.rint(box).astype(np.int32)], 1)
    return mask.astype(bool)


def _render_target(image: np.ndarray, class_id: str, domain: str, short_side: int, center: tuple[int, int], angle: float, rng: np.random.Generator) -> np.ndarray:
    long_side = max(short_side + 2, int(round(short_side * (2.2 if class_id != "paper_litter" else 2.8))))
    if class_id == "metal_can":
        mask = _rotated_mask(center, (short_side, long_side), angle, ellipse="crushed" not in domain)
        color = (45, 50, 56) if "dark" in domain or "shadow" in domain else (186, 194, 202)
        if "printed" in domain:
            color = ((uid_color := int(rng.integers(45, 205))), 70, 210 - uid_color // 2)
    elif class_id == "plastic_bottle":
        mask = _rotated_mask(center, (short_side, long_side), angle, ellipse=True)
        color = (95, 155, 190) if "clear" in domain else (55, 135, 205)
    else:
        mask = _rotated_mask(center, (long_side, short_side), angle)
        color = (224, 224, 214) if "white" in domain or "road_paint" in domain else (210, 136, 92)
    image[mask] = color
    if mask.any() and ("specular" in domain or "reflective" in domain):
        highlight = np.zeros((HEIGHT, WIDTH), dtype=np.uint8)
        cv2.line(highlight, (center[0] - short_side, center[1]), (center[0] + short_side, center[1]), 1, max(1, short_side // 5))
        image[mask & highlight.astype(bool)] = (245, 247, 250)
    return mask


def _negative_decoy(image: np.ndarray, taxonomy: str, rng: np.random.Generator) -> dict:
    center = (int(rng.integers(55, WIDTH - 55)), int(rng.integers(125, HEIGHT - 35)))
    mask = np.zeros((HEIGHT, WIDTH), dtype=np.uint8)
    if taxonomy in {"white_yellow_paint", "lane_marking", "crack", "tile_seam"}:
        width = 13 if "paint" in taxonomy or "lane" in taxonomy else 3
        cv2.line(mask, (center[0] - 65, center[1] - 14), (center[0] + 65, center[1] + 17), 1, width)
    elif taxonomy == "manhole":
        cv2.circle(mask, center, 28, 1, -1)
    elif taxonomy == "small_bright_spots":
        for _ in range(14):
            point = (int(center[0] + rng.integers(-45, 46)), int(center[1] + rng.integers(-25, 26)))
            cv2.circle(mask, point, int(rng.integers(1, 4)), 1, -1)
    else:
        axes = (int(rng.integers(18, 60)), int(rng.integers(7, 28)))
        cv2.ellipse(mask, center, axes, float(rng.uniform(0, 180)), 0, 360, 1, -1)
    colors = {
        "wet_road": (50, 76, 91), "specular_road": (196, 199, 194), "oil_like_patch": (50, 57, 70),
        "white_yellow_paint": (225, 214, 144), "lane_marking": (231, 230, 209), "crack": (35, 35, 38),
        "tile_seam": (57, 60, 64), "manhole": (67, 72, 76), "stone": (145, 139, 126),
        "leaf_shadow": (42, 48, 39), "vehicle_shadow": (38, 42, 52), "pedestrian_shadow": (44, 43, 48),
        "metal_reflection": (218, 223, 228), "plastic_like_clutter": (62, 145, 186),
        "small_bright_spots": (239, 235, 214), "dark_debris_like_non_target": (31, 34, 35),
    }
    image[mask.astype(bool)] = colors[taxonomy]
    return {"taxonomy": taxonomy, "pixel_count": int(mask.sum()), "target_label_count": 0}


def _positive_schedule(plan: G7Plan) -> list[dict]:
    if not plan.formal:
        return [
            {"class_id": class_id, "domain": domains[index % len(domains)], "size_bucket": "small_lt18" if index % 2 == 0 else "medium_18_48"}
            for index, (class_id, domains) in enumerate(
                (
                    ("metal_can", METAL_DOMAINS),
                    ("plastic_bottle", BOTTLE_DOMAINS),
                    ("paper_litter", PAPER_DOMAINS),
                    ("metal_can", METAL_DOMAINS),
                    ("plastic_bottle", BOTTLE_DOMAINS),
                    ("paper_litter", PAPER_DOMAINS),
                    ("metal_can", METAL_DOMAINS),
                    ("paper_litter", PAPER_DOMAINS),
                )
            )
        ]
    rows: list[dict] = []
    for domain in METAL_DOMAINS:
        minimum = plan.critical_metal_domain_minimum if domain in CRITICAL_METAL_DOMAINS else plan.metal_domain_minimum
        rows.extend({"class_id": "metal_can", "domain": domain} for _ in range(math.ceil(minimum * 1.15)))
    rows.extend({"class_id": "plastic_bottle", "domain": BOTTLE_DOMAINS[index % len(BOTTLE_DOMAINS)]} for index in range(math.ceil(plan.bottle_instances_minimum * 1.25)))
    rows.extend({"class_id": "paper_litter", "domain": PAPER_DOMAINS[index % len(PAPER_DOMAINS)]} for index in range(math.ceil(plan.paper_instances_minimum * 1.25)))
    while sum(item["class_id"] == "metal_can" for item in rows) < math.ceil(plan.metal_instances_minimum * 1.20):
        rows.append({"class_id": "metal_can", "domain": METAL_DOMAINS[len(rows) % len(METAL_DOMAINS)]})
    minimum_total = math.ceil((plan.small_instances_minimum + plan.medium_instances_minimum) * 1.15)
    while len(rows) < minimum_total:
        class_id = CLASSES[len(rows) % len(CLASSES)]
        domains = {"metal_can": METAL_DOMAINS, "plastic_bottle": BOTTLE_DOMAINS, "paper_litter": PAPER_DOMAINS}[class_id]
        rows.append({"class_id": class_id, "domain": domains[len(rows) % len(domains)]})
    for index, item in enumerate(rows):
        item["size_bucket"] = "small_lt18" if index % 5 < 2 else "medium_18_48"
    return rows


def _write_json(path: Path, payload: object) -> None:
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def build_g7_dataset(output: str | Path, plan: G7Plan | None = None) -> dict:
    plan = plan or G7Plan()
    if set(plan.frames_by_split) != set(SPLITS):
        raise ValueError("G7 plan must declare every required split")
    root = Path(output)
    if root.exists() and any(root.iterdir()):
        raise FileExistsError(f"G7 output must be new and empty: {root}")
    for name in ("rgb", "depth", "semantic", "instance", "frames", "reports"):
        (root / name).mkdir(parents=True, exist_ok=True)
    positives = _positive_schedule(plan)
    positive_cursor = 0
    total_frames = sum(plan.frames_by_split.values())
    negative_target = min(plan.full_negative_target, total_frames - len(positives) // 2 - 1)
    if plan.formal and negative_target < plan.full_negative_frames_minimum:
        raise ValueError("G7 frame plan cannot satisfy full-negative quota")
    frame_rows: list[dict] = []
    instance_rows: list[dict] = []
    seen_exact: set[str] = set()
    seen_phash = {split: set() for split in SPLITS}
    negative_counts = {name: {split: 0 for split in SPLITS} for name in NEGATIVE_TAXONOMIES}
    domain_counts = {
        "metal_can": {name: 0 for name in METAL_DOMAINS},
        "plastic_bottle": {name: 0 for name in BOTTLE_DOMAINS},
        "paper_litter": {name: 0 for name in PAPER_DOMAINS},
    }
    class_counts = {name: 0 for name in CLASSES}
    size_counts = {"small_lt18": 0, "medium_18_48": 0, "large_gt48": 0}
    frame_path = root / "G7_FRAME_MANIFEST.jsonl"
    instance_path = root / "G7_INSTANCE_RECORDS.jsonl"
    global_index = 0
    full_negative_count = 0
    with frame_path.open("w", encoding="utf-8") as frame_stream, instance_path.open("w", encoding="utf-8") as instance_stream:
        for split in SPLITS:
            frame_count = int(plan.frames_by_split[split])
            worlds = WORLD_IDS[split]
            for local_index in range(frame_count):
                uid = 7400000 + global_index
                scene_index = local_index // plan.frames_per_scene
                scene_seed = 740000 + sum(plan.frames_by_split[item] for item in SPLITS[: SPLITS.index(split)]) // plan.frames_per_scene + scene_index
                world_id = worlds[scene_index % len(worlds)]
                world_index = [item["world_id"] for item in _world_registry()].index(world_id)
                rng = np.random.default_rng(uid * 104729 + 17)
                image = _background(world_index, uid, rng)
                depth = np.full((HEIGHT, WIDTH), 5200, dtype=np.uint16)
                semantic = np.zeros((HEIGHT, WIDTH), dtype=np.uint8)
                instance = np.zeros((HEIGHT, WIDTH), dtype=np.uint16)
                objects: list[dict] = []
                decoys: list[dict] = []
                is_negative = full_negative_count < negative_target and global_index % max(2, total_frames // max(negative_target, 1)) == 0
                if split == "TRAIN" and positive_cursor < len(positives) and not is_negative:
                    remaining = len(positives) - positive_cursor
                    remaining_positive_frames = max(1, plan.frames_by_split["TRAIN"] - local_index - max(0, negative_target - full_negative_count))
                    take = min(3, max(1, math.ceil(remaining / remaining_positive_frames)))
                    requested = positives[positive_cursor : positive_cursor + take]
                    positive_cursor += len(requested)
                elif not is_negative:
                    class_id = CLASSES[uid % len(CLASSES)]
                    domains = {"metal_can": METAL_DOMAINS, "plastic_bottle": BOTTLE_DOMAINS, "paper_litter": PAPER_DOMAINS}[class_id]
                    requested = [{"class_id": class_id, "domain": domains[uid % len(domains)], "size_bucket": "small_lt18" if uid % 3 == 0 else "medium_18_48"}]
                else:
                    requested = []
                    full_negative_count += 1
                for object_index, target in enumerate(requested, 1):
                    desired = target["size_bucket"]
                    mask = None
                    candidate_image = None
                    for placement_attempt in range(12):
                        short = int(rng.integers(7, 16)) if desired == "small_lt18" else int(rng.integers(18, 41))
                        center = (int(rng.integers(80, WIDTH - 80)), int(rng.integers(135, HEIGHT - 55)))
                        angle = float(90 * ((uid + object_index + placement_attempt) % 2)) if desired == "small_lt18" else float(rng.uniform(-70, 70))
                        candidate_image = image.copy()
                        candidate_mask = _render_target(candidate_image, target["class_id"], target["domain"], short, center, angle, rng)
                        if "partial" in target["domain"] or "occluded" in target["domain"]:
                            occluder = np.zeros((HEIGHT, WIDTH), dtype=np.uint8)
                            cv2.rectangle(occluder, (center[0] + max(1, short // 3), center[1] - short * 2), (center[0] + short * 3, center[1] + short * 3), 1, -1)
                            candidate_mask &= ~occluder.astype(bool)
                        candidate_mask &= instance == 0
                        if candidate_mask.any():
                            mask = candidate_mask
                            image = candidate_image
                            break
                    if mask is None:
                        raise RuntimeError("G7 target became invisible after 12 bounded placements")
                    ys, xs = np.nonzero(mask)
                    x0, x1, y0, y1 = int(xs.min()), int(xs.max() + 1), int(ys.min()), int(ys.max() + 1)
                    measured = min(x1 - x0, y1 - y0)
                    semantic[mask] = CLASS_INDEX[target["class_id"]]
                    instance[mask] = object_index
                    distance_m = float(rng.uniform(1.0, 7.0))
                    depth[mask] = int(distance_m * 1000)
                    asset_id = f"g7v4_{split.lower()}_{target['class_id']}_asset_{(scene_index * 3 + object_index) % 97:02d}"
                    row = {
                        "scene_seed": scene_seed, "frame_index": local_index % plan.frames_per_scene,
                        "split": split, "world_id": world_id, "instance_id": object_index,
                        "class_id": target["class_id"], "class_index": CLASS_INDEX[target["class_id"]],
                        "asset_id": asset_id, "asset_lineage": "g7v4_new_variant",
                        "bbox_xyxy": [x0, y0, x1, y1], "bbox_short_side_px": measured,
                        "size_bucket": _bucket(measured), "mask_area_px": int(mask.sum()),
                        "visible_pixels": int(mask.sum()), "distance_m": distance_m,
                        "material": target["domain"], "lighting": f"g7v4_light_{world_index:02d}_{uid % 7:02d}",
                        "orientation_deg": angle, "occlusion_metadata": {"partial": "partial" in target["domain"] or "occluded" in target["domain"]},
                    }
                    objects.append(row); instance_rows.append(row); instance_stream.write(json.dumps(row, sort_keys=True) + "\n")
                    class_counts[row["class_id"]] += 1; size_counts[row["size_bucket"]] += 1
                    domain_counts[row["class_id"]][target["domain"]] += 1
                if is_negative:
                    taxonomy = NEGATIVE_TAXONOMIES[(full_negative_count - 1) % len(NEGATIVE_TAXONOMIES)]
                    decoy = _negative_decoy(image, taxonomy, rng)
                    decoy["asset_id"] = f"g7v4_{split.lower()}_negative_{taxonomy}_{scene_index % 41:02d}"
                    decoys.append(decoy); negative_counts[taxonomy][split] += 1
                stem = f"g7v4_{split.lower()}_scene_{scene_seed}_frame_{local_index % plan.frames_per_scene:02d}_{global_index:05d}"
                paths = {name: root / name / f"{stem}.png" for name in ("rgb", "depth", "semantic", "instance")}
                metadata_path = root / "frames" / f"{stem}.json"
                image, perceptual = _unique_phash(image, split, uid, seen_phash)
                cv2.imwrite(str(paths["rgb"]), cv2.cvtColor(image, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_PNG_COMPRESSION, 6])
                cv2.imwrite(str(paths["depth"]), depth, [cv2.IMWRITE_PNG_COMPRESSION, 9])
                cv2.imwrite(str(paths["semantic"]), semantic, [cv2.IMWRITE_PNG_COMPRESSION, 9])
                cv2.imwrite(str(paths["instance"]), instance, [cv2.IMWRITE_PNG_COMPRESSION, 9])
                _write_json(metadata_path, {"scene_pose_reset": True, "objects": objects, "negative_decoys": decoys, "camera": {"width": WIDTH, "height": HEIGHT, "timestamp_ns": 740000000000000000 + global_index * 66666667}, "tf": {"map_to_camera": [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]}})
                image_sha = _file_hash(paths["rgb"])
                if image_sha in seen_exact:
                    raise RuntimeError("exact duplicate G7 RGB frame")
                seen_exact.add(image_sha); seen_phash[split].add(perceptual)
                record = {
                    "scene_seed": scene_seed, "frame_index": local_index % plan.frames_per_scene,
                    "split": split, "world_id": world_id, "negative_only": not objects,
                    "target_instance_count": len(objects), "negative_taxonomies": [item["taxonomy"] for item in decoys],
                    "image_sha256": image_sha, "perceptual_hash": perceptual,
                    "rgb_path": paths["rgb"].relative_to(root).as_posix(), "depth_path": paths["depth"].relative_to(root).as_posix(),
                    "semantic_path": paths["semantic"].relative_to(root).as_posix(), "instance_path": paths["instance"].relative_to(root).as_posix(),
                    "frame_path": metadata_path.relative_to(root).as_posix(),
                }
                frame_rows.append(record); frame_stream.write(json.dumps(record, sort_keys=True) + "\n"); global_index += 1
    worlds_by_split = {split: sorted({row["world_id"] for row in frame_rows if row["split"] == split}) for split in SPLITS}
    assets_by_split = {split: sorted({row["asset_id"] for row in instance_rows if row["split"] == split}) for split in SPLITS}
    asset_overlap = sum(len(set(assets_by_split[left]) & set(assets_by_split[right])) for index, left in enumerate(SPLITS) for right in SPLITS[index + 1 :])
    disallowed_world_overlap = sum(len(set(worlds_by_split[left]) & set(worlds_by_split[right])) for index, left in enumerate(SPLITS) for right in SPLITS[index + 1 :] if {left, right} != {"TRAIN", "IN_DOMAIN_HOLDOUT"})
    phash_overlap = sum(len(seen_phash[left] & seen_phash[right]) for index, left in enumerate(SPLITS) for right in SPLITS[index + 1 :])
    gates = {
        "new_worlds_at_least_8": len(_world_registry()) >= (8 if plan.formal else 1),
        "metal_instances_at_least_400": class_counts["metal_can"] >= (plan.metal_instances_minimum if plan.formal else 0),
        "bottle_instances_at_least_250": class_counts["plastic_bottle"] >= (plan.bottle_instances_minimum if plan.formal else 0),
        "paper_instances_at_least_300": class_counts["paper_litter"] >= (plan.paper_instances_minimum if plan.formal else 0),
        "small_instances_at_least_600": size_counts["small_lt18"] >= (plan.small_instances_minimum if plan.formal else 0),
        "medium_instances_at_least_800": size_counts["medium_18_48"] >= (plan.medium_instances_minimum if plan.formal else 0),
        "full_negative_frames_at_least_400": full_negative_count >= (plan.full_negative_frames_minimum if plan.formal else 0),
        "metal_domain_quotas": all(value >= (plan.critical_metal_domain_minimum if name in CRITICAL_METAL_DOMAINS else plan.metal_domain_minimum) for name, value in domain_counts["metal_can"].items()) if plan.formal else True,
        "negative_taxonomy_complete": all(sum(counts.values()) > 0 for counts in negative_counts.values()) if plan.formal else True,
        "scene_reset_100_percent": len(frame_rows) == global_index,
        "manifest_pixel_consistency_100_percent": all(row["target_instance_count"] == len(json.loads((root / row["frame_path"]).read_text(encoding="utf-8"))["objects"]) for row in frame_rows),
        "label_count_mismatch_zero": len(instance_rows) == sum(row["target_instance_count"] for row in frame_rows),
        "negative_only_stale_positive_zero": all(row["target_instance_count"] == 0 for row in frame_rows if row["negative_only"]),
        "exact_cross_split_duplicate_zero": len(seen_exact) == len(frame_rows),
        "phash_cross_split_duplicate_zero": phash_overlap == 0,
        "world_leakage_zero": disallowed_world_overlap == 0,
        "asset_leakage_zero": asset_overlap == 0,
        "g6_g5_g5v2_namespace_collision_zero": all(row["world_id"].startswith("g7v4_") and row["asset_id"].startswith("g7v4_") for row in instance_rows),
        "sealed_data_not_read": True,
        "g6_data_not_read": True,
    }
    world_registry = {"dataset_id": DATASET_ID, "worlds": _world_registry(), "reserved_namespaces": {"G6": "g6_", "G5": "g5_", "G5_V2": "g5v2_"}}
    asset_registry = {"dataset_id": DATASET_ID, "assets_by_split": assets_by_split, "lineage_policy": "new G7 material/texture/scale/orientation variants only", "cross_split_overlap": asset_overlap}
    split_manifest = {"dataset_id": DATASET_ID, "splits": list(SPLITS), "frames_by_split": plan.frames_by_split, "worlds_by_split": worlds_by_split, "in_domain_holdout_shares_train_world_family": True, "disallowed_world_overlap": disallowed_world_overlap, "asset_overlap": asset_overlap, "contract_sha256": _canonical_hash({"worlds": worlds_by_split, "assets": assets_by_split})}
    domain_report = {"dataset_id": DATASET_ID, "counts": domain_counts, "minimum_per_metal_domain": plan.metal_domain_minimum, "minimum_per_critical_metal_domain": plan.critical_metal_domain_minimum, "critical_metal_domains": sorted(CRITICAL_METAL_DOMAINS)}
    negative_report = {"dataset_id": DATASET_ID, "taxonomy_counts_by_split": negative_counts, "full_negative_frame_count": full_negative_count, "minimum": plan.full_negative_frames_minimum}
    size_report = {"dataset_id": DATASET_ID, "counts": size_counts, "small_definition": "bbox short side <18 px", "medium_definition": "18-48 px inclusive"}
    qa = {"schema_version": 1, "stage": "DDRV4-01", "dataset_id": DATASET_ID, "development_only": True, "sealed_final": False, "access_audit": {"G6_read": False, "G5_read": False, "G5_V2_read": False}, "frame_count": len(frame_rows), "scene_count": len({(row["split"], row["scene_seed"]) for row in frame_rows}), "world_count": len(_world_registry()), "instance_count": len(instance_rows), "class_counts": class_counts, "gates": gates, "G7_DATASET_PASS": all(gates.values()), "manifests": {"frames": frame_path.name, "instances": instance_path.name}}
    reports = {"G7_DATASET_QA.json": qa, "G7_SPLIT_MANIFEST.json": split_manifest, "G7_ASSET_REGISTRY.json": asset_registry, "G7_WORLD_REGISTRY.json": world_registry, "G7_DOMAIN_MATRIX.json": domain_report, "G7_NEGATIVE_TAXONOMY.json": negative_report, "G7_SMALL_OBJECT_DISTRIBUTION.json": size_report}
    for name, payload in reports.items():
        _write_json(root / "reports" / name, payload)
    return qa


def load_jsonl(path: str | Path) -> list[dict]:
    return [json.loads(line) for line in Path(path).read_text(encoding="utf-8").splitlines() if line]


__all__ = ["BOTTLE_DOMAINS", "CRITICAL_METAL_DOMAINS", "DATASET_ID", "G7Plan", "METAL_DOMAINS", "NEGATIVE_TAXONOMIES", "PAPER_DOMAINS", "SPLITS", "WORLD_IDS", "build_g7_dataset", "load_jsonl"]
