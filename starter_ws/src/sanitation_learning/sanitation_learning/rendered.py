from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path

import cv2
import numpy as np

from .assets import load_asset_registry


WIDTH = 128
HEIGHT = 96
CLASS_ORDER = (
    "background",
    "plastic_bottle",
    "metal_can",
    "paper_litter",
    "leaf_pile",
    "puddle",
)
CLASS_INDEX = {name: index for index, name in enumerate(CLASS_ORDER)}


@dataclass(frozen=True)
class RenderedFrame:
    scene_seed: int
    frame_index: int
    split: str
    world_id: str
    image_rgb: np.ndarray
    depth_m: np.ndarray
    semantic_labels: np.ndarray
    instance_labels: np.ndarray
    objects: tuple[dict, ...]
    negatives: tuple[dict, ...]
    camera: dict
    transform_map_camera: tuple[tuple[float, ...], ...]
    scene_config: dict


def split_for_seed(seed: int) -> str:
    slot = int(seed) % 10
    if slot <= 6:
        return "train"
    if slot == 7:
        return "val"
    return "test"


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _phash(image: np.ndarray) -> str:
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    reduced = cv2.resize(gray, (32, 32), interpolation=cv2.INTER_AREA).astype(np.float32)
    frequency = cv2.dct(reduced)[:8, :8]
    bits = frequency > np.median(frequency[1:])
    return f"{int(''.join('1' if value else '0' for value in bits.ravel()), 2):016x}"


def _polygon_mask(points: np.ndarray) -> np.ndarray:
    mask = np.zeros((HEIGHT, WIDTH), dtype=np.uint8)
    cv2.fillPoly(mask, [np.rint(points).astype(np.int32)], 1)
    return mask.astype(bool)


def _rotated_rectangle(center, width, height, angle_deg) -> np.ndarray:
    box = cv2.boxPoints((tuple(float(v) for v in center), (float(width), float(height)), float(angle_deg)))
    return _polygon_mask(box)


def _target_mask(class_id: str, center: tuple[int, int], scale: float, yaw: float, variant: int, rng) -> np.ndarray:
    cx, cy = center
    mask = np.zeros((HEIGHT, WIDTH), dtype=np.uint8)
    if class_id == "plastic_bottle":
        width = max(5, int(7 * scale)); height = max(12, int((18 + variant) * scale))
        body = _rotated_rectangle(center, width, height, yaw)
        neck_center = (int(cx - math.sin(math.radians(yaw)) * height * 0.55), int(cy + math.cos(math.radians(yaw)) * height * 0.55))
        cv2.circle(mask, neck_center, max(2, width // 4), 1, -1)
        mask |= body.astype(np.uint8)
    elif class_id == "metal_can":
        axes = (max(4, int((7 + variant % 2) * scale)), max(6, int((10 + variant) * scale)))
        cv2.ellipse(mask, center, axes, yaw, 0, 360, 1, -1)
    elif class_id == "paper_litter":
        points = cv2.boxPoints((center, (max(12, int((18 + variant) * scale)), max(7, int((10 + variant % 3) * scale))), yaw))
        points += rng.normal(0, 1.2, points.shape)
        mask = _polygon_mask(points).astype(np.uint8)
    elif class_id == "leaf_pile":
        for _ in range(5 + variant):
            offset = rng.normal(0, [8 * scale, 5 * scale])
            leaf_center = (int(cx + offset[0]), int(cy + offset[1]))
            axes = (max(3, int(rng.uniform(4, 8) * scale)), max(2, int(rng.uniform(2, 4) * scale)))
            cv2.ellipse(mask, leaf_center, axes, float(rng.uniform(0, 180)), 0, 360, 1, -1)
    else:
        count = 14
        angles = np.linspace(0, 2 * np.pi, count, endpoint=False)
        radii_x = rng.uniform(12, 21, count) * scale
        radii_y = rng.uniform(6, 12, count) * scale
        points = np.column_stack((cx + np.cos(angles) * radii_x, cy + np.sin(angles) * radii_y))
        mask = _polygon_mask(points).astype(np.uint8)
        if variant == 5:
            cv2.circle(mask, (cx, cy), max(2, int(3 * scale)), 0, -1)
    return mask.astype(bool)


def _texture(mask: np.ndarray, colors: list[np.ndarray], texture_id: str, rng) -> np.ndarray:
    canvas = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
    canvas[:] = colors[0]
    yy, xx = np.indices((HEIGHT, WIDTH))
    if texture_id in {"ribs", "rings", "bands"}:
        pattern = ((xx + 2 * yy) // 4) % 2 == 0
    elif texture_id in {"grid", "crosshatch"}:
        pattern = ((xx // 5 + yy // 5) % 2) == 0
    elif texture_id in {"speckle", "veins"}:
        pattern = rng.random((HEIGHT, WIDTH)) > 0.72
    elif texture_id in {"reflection", "ripple"}:
        pattern = np.sin(xx * 0.35 + np.sin(yy * 0.2) * 2) > 0.45
    else:
        pattern = ((xx + yy) % 9) < 3
    canvas[np.logical_and(mask, pattern)] = colors[1]
    highlight = np.logical_and(mask, ((xx - WIDTH // 2) ** 2 + (yy - HEIGHT // 2) ** 2) % 37 < 3)
    canvas[highlight] = np.clip(canvas[highlight].astype(np.int16) + 25, 0, 255).astype(np.uint8)
    return canvas


def _draw_negative(image, depth, semantic, instances, kind: str, rng) -> dict:
    x = int(rng.integers(5, WIDTH - 10)); y = int(rng.integers(8, HEIGHT - 8))
    color_by_kind = {
        "red_obstacle": (176, 54, 58), "green_obstacle": (68, 132, 74),
        "blue_obstacle": (54, 94, 168), "reflective_patch": (190, 190, 184),
        "wet_asphalt": (38, 142, 146), "shadow": (35, 35, 42),
    }
    color = color_by_kind.get(kind, tuple(int(v) for v in rng.integers(45, 195, 3)))
    mask = np.zeros((HEIGHT, WIDTH), dtype=np.uint8)
    if "bottle_like" in kind:
        cv2.rectangle(mask, (x - 3, y - 8), (x + 3, y + 8), 1, -1)
    elif "can_like" in kind or "bollard" in kind:
        cv2.circle(mask, (x, y), 6, 1, -1)
    elif kind in {"reflective_patch", "wet_asphalt", "shadow", "leaves_outside_target_region"}:
        cv2.ellipse(mask, (x, y), (int(rng.integers(7, 17)), int(rng.integers(4, 10))), float(rng.integers(0, 180)), 0, 360, 1, -1)
    else:
        cv2.rectangle(mask, (x - 6, y - 6), (x + 6, y + 6), 1, -1)
    selected = mask.astype(bool)
    image[selected] = color
    semantic[selected] = 0
    instances[selected] = 0
    depth[selected] = float(rng.uniform(1.0, 3.4))
    return {"negative_id": kind, "pixel_count": int(selected.sum()), "target_label_count": 0}


def generate_frame(scene_seed: int, frame_index: int, registry: dict) -> RenderedFrame:
    split = split_for_seed(scene_seed)
    contract = registry["split_contract"]
    variant_indices = contract[f"{split}_variant_indices"]
    worlds = contract[f"{split}_worlds"]
    world_id = worlds[int(scene_seed) % len(worlds)]
    rng = np.random.default_rng(int(scene_seed) * 1009 + int(frame_index) * 37 + 2051)
    base = np.asarray(registry["palette_rgb"]["stone"], dtype=np.float32)
    gradient = np.linspace(-22, 18, WIDTH, dtype=np.float32)[None, :, None]
    texture_noise = rng.normal(0, 11, (HEIGHT, WIDTH, 1))
    image = np.clip(base + gradient + texture_noise, 0, 255).astype(np.uint8)
    depth = np.full((HEIGHT, WIDTH), 4.5, dtype=np.float32)
    semantic = np.zeros((HEIGHT, WIDTH), dtype=np.uint8)
    instances = np.zeros((HEIGHT, WIDTH), dtype=np.uint16)
    camera_x = float(rng.uniform(-0.15, 0.15)); camera_y = float(rng.uniform(-0.15, 0.15))
    fx = float(rng.uniform(72, 82)); fy = float(rng.uniform(70, 80))
    camera = {"width": WIDTH, "height": HEIGHT, "fx": fx, "fy": fy, "cx": WIDTH / 2, "cy": HEIGHT / 2, "camera_map_xy": [camera_x, camera_y], "pixel_sigma": 0.5, "depth_sigma_m": 0.025}
    objects_raw = []
    instance_id = 1
    class_ids = list(CLASS_ORDER[1:])
    rng.shuffle(class_ids)
    selected_classes = class_ids[: int(rng.integers(3, 6))]
    selected_classes.extend(rng.choice(selected_classes, size=int(rng.integers(0, 4)), replace=True).tolist())
    for class_id in selected_classes:
        variant_index = int(rng.choice(variant_indices))
        variant = registry["classes"][class_id]["variants"][variant_index]
        depth_m = float(rng.uniform(1.3, 3.4))
        center = (int(rng.integers(14, WIDTH - 14)), int(rng.integers(14, HEIGHT - 12)))
        scale = float(np.clip(2.4 / depth_m * rng.uniform(0.85, 1.2), 0.65, 1.8))
        yaw = float(rng.uniform(-85, 85))
        mask = _target_mask(class_id, center, scale, yaw, variant_index, rng)
        palette = [np.asarray(registry["palette_rgb"][name], dtype=np.uint8) for name in variant["palette"]]
        textured = _texture(mask, palette, variant["texture"], rng)
        image[mask] = textured[mask]
        semantic[mask] = CLASS_INDEX[class_id]
        instances[mask] = instance_id
        noisy_depth = depth_m + rng.normal(0, 0.018, int(mask.sum()))
        depth[mask] = noisy_depth.astype(np.float32)
        map_x = camera_x + (center[0] - camera["cx"]) * depth_m / fx
        map_y = camera_y + (center[1] - camera["cy"]) * depth_m / fy
        objects_raw.append({"instance_id": instance_id, "class_id": class_id, "class_index": CLASS_INDEX[class_id], "asset_id": variant["id"], "texture_id": variant["texture"], "variant_index": variant_index, "depth_m": depth_m, "map_pose": [map_x, map_y, 0.0], "target_type": registry["classes"][class_id]["target_type"], "cleaning_policy": registry["classes"][class_id]["policy"], "yaw_deg": yaw})
        instance_id += 1
    negatives = []
    negative_choices = list(registry["negative_assets"])
    rng.shuffle(negative_choices)
    for kind in negative_choices[: int(rng.integers(2, 6))]:
        negatives.append(_draw_negative(image, depth, semantic, instances, kind, rng))
    objects = []
    for raw in objects_raw:
        visible = instances == raw["instance_id"]
        rows, cols = np.nonzero(visible)
        if rows.size < 8:
            continue
        x0, x1 = int(cols.min()), int(cols.max() + 1)
        y0, y1 = int(rows.min()), int(rows.max() + 1)
        visible_area = int(rows.size)
        original_estimate = max(visible_area, int((x1 - x0) * (y1 - y0) * 0.72))
        objects.append({**raw, "bbox_xywh": [x0, y0, x1 - x0, y1 - y0], "pixel_area": visible_area, "center_uv": [float(cols.mean()), float(rows.mean())], "visibility": min(1.0, visible_area / max(original_estimate, 1)), "occlusion_ratio": max(0.0, 1.0 - visible_area / max(original_estimate, 1)), "truncation": float(x0 == 0 or y0 == 0 or x1 == WIDTH or y1 == HEIGHT)})
    exposure = float(rng.uniform(0.58, 1.42))
    white_balance = rng.uniform(0.82, 1.18, 3)
    image = np.clip(image.astype(np.float32) * exposure * white_balance, 0, 255)
    if rng.random() < 0.35:
        image = cv2.GaussianBlur(image.astype(np.uint8), (3, 3), float(rng.uniform(0.3, 1.0))).astype(np.float32)
    image = np.clip(image + rng.normal(0, rng.uniform(1, 7), image.shape), 0, 255).astype(np.uint8)
    depth_holes = rng.random(depth.shape) < 0.008
    depth[depth_holes] = np.nan
    transform = ((1.0, 0.0, 0.0, camera_x), (0.0, 1.0, 0.0, camera_y), (0.0, 0.0, 1.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    scene_config = {"world_id": world_id, "scene_seed": int(scene_seed), "frame_index": int(frame_index), "split": split, "target_count_requested": len(selected_classes), "target_count_visible": len(objects), "negative_count": len(negatives), "lighting": {"exposure": exposure, "white_balance": white_balance.tolist()}, "sensor": {"depth_hole_ratio": float(depth_holes.mean()), "motion_blur_applied": bool(rng.random() < 0.35)}}
    return RenderedFrame(int(scene_seed), int(frame_index), split, world_id, image, depth, semantic, instances, tuple(objects), tuple(negatives), camera, transform, scene_config)


def write_dataset(output: str | Path, registry_path: str | Path, scene_seeds: list[int], frames_per_scene: int = 10) -> dict:
    root = Path(output)
    registry = load_asset_registry(registry_path)
    for name in ("images", "depth", "semantic", "instances", "annotations", "camera", "tf", "scene_manifests", "splits"):
        (root / name).mkdir(parents=True, exist_ok=True)
    records = []
    split_scenes = {"train": [], "val": [], "test": []}
    per_class_instances = {name: 0 for name in CLASS_ORDER[1:]}
    perceptual_hashes = []
    annotation_id = 1
    coco_images = []
    coco_annotations = []
    for scene_seed in scene_seeds:
        split = split_for_seed(scene_seed)
        split_scenes[split].append(int(scene_seed))
        frame_records = []
        for frame_index in range(frames_per_scene):
            frame = generate_frame(scene_seed, frame_index, registry)
            stem = f"scene_{scene_seed:04d}_frame_{frame_index:02d}"
            paths = {"image": root / "images" / f"{stem}.png", "depth": root / "depth" / f"{stem}.npy", "semantic": root / "semantic" / f"{stem}.png", "instances": root / "instances" / f"{stem}.png", "annotation": root / "annotations" / f"{stem}.json", "camera": root / "camera" / f"{stem}.json", "tf": root / "tf" / f"{stem}.json"}
            cv2.imwrite(str(paths["image"]), cv2.cvtColor(frame.image_rgb, cv2.COLOR_RGB2BGR))
            np.save(paths["depth"], frame.depth_m, allow_pickle=False)
            cv2.imwrite(str(paths["semantic"]), frame.semantic_labels)
            cv2.imwrite(str(paths["instances"]), frame.instance_labels)
            paths["annotation"].write_text(json.dumps({"objects": frame.objects, "negatives": frame.negatives, "scene_config": frame.scene_config}, indent=2) + "\n", encoding="utf-8")
            paths["camera"].write_text(json.dumps(frame.camera, indent=2) + "\n", encoding="utf-8")
            paths["tf"].write_text(json.dumps({"T_map_camera": frame.transform_map_camera}, indent=2) + "\n", encoding="utf-8")
            image_id = len(coco_images) + 1
            coco_images.append({"id": image_id, "file_name": paths["image"].name, "width": WIDTH, "height": HEIGHT, "scene_seed": int(scene_seed), "frame_index": frame_index, "split": split, "world_id": frame.world_id})
            for obj in frame.objects:
                per_class_instances[obj["class_id"]] += 1
                coco_annotations.append({"id": annotation_id, "image_id": image_id, "category_id": obj["class_index"], "bbox": obj["bbox_xywh"], "area": obj["pixel_area"], "iscrowd": 0, "instance_id": obj["instance_id"], "asset_id": obj["asset_id"], "texture_id": obj["texture_id"], "map_pose": obj["map_pose"], "depth_m": obj["depth_m"], "visibility": obj["visibility"], "occlusion_ratio": obj["occlusion_ratio"]})
                annotation_id += 1
            phash = _phash(frame.image_rgb)
            perceptual_hashes.append(phash)
            record = {"scene_seed": int(scene_seed), "frame_index": frame_index, "split": split, "world_id": frame.world_id, **{name: str(path.relative_to(root)).replace("\\", "/") for name, path in paths.items()}, "image_sha256": _sha256(paths["image"]), "semantic_sha256": _sha256(paths["semantic"]), "perceptual_hash": phash, "asset_ids": sorted({obj["asset_id"] for obj in frame.objects}), "texture_ids": sorted({obj["texture_id"] for obj in frame.objects}), "target_instance_count": len(frame.objects), "negative_count": len(frame.negatives)}
            frame_records.append(record); records.append(record)
        (root / "scene_manifests" / f"scene_{scene_seed:04d}.json").write_text(json.dumps({"scene_seed": int(scene_seed), "split": split, "frames": frame_records}, indent=2) + "\n", encoding="utf-8")
    for split, seeds in split_scenes.items():
        (root / "splits" / f"{split}.json").write_text(json.dumps({"scene_seeds": seeds}, indent=2) + "\n", encoding="utf-8")
    coco = {"images": coco_images, "annotations": coco_annotations, "categories": [{"id": index, "name": name} for index, name in enumerate(CLASS_ORDER[1:], 1)]}
    (root / "annotations" / "coco.json").write_text(json.dumps(coco, indent=2) + "\n", encoding="utf-8")
    split_assets = {split: sorted({asset for record in records if record["split"] == split for asset in record["asset_ids"]}) for split in split_scenes}
    split_textures = {split: sorted({texture for record in records if record["split"] == split for texture in record["texture_ids"]}) for split in split_scenes}
    split_worlds = {split: sorted({record["world_id"] for record in records if record["split"] == split}) for split in split_scenes}
    split_hash = hashlib.sha256(json.dumps({"scenes": split_scenes, "assets": split_assets, "textures": split_textures, "worlds": split_worlds}, sort_keys=True).encode()).hexdigest()
    near_duplicate_count = len(perceptual_hashes) - len(set(perceptual_hashes))
    manifest = {"schema_version": 1, "dataset_id": "stage5b_d1_procedural_rendered_v1", "domain": "D1_procedural_rendered_not_gazebo_camera", "scene_count": len(scene_seeds), "frame_count": len(records), "frames_per_scene": frames_per_scene, "class_order": list(CLASS_ORDER), "split_scene_seeds": split_scenes, "split_assets": split_assets, "split_textures": split_textures, "split_worlds": split_worlds, "split_hash": split_hash, "adjacent_frames_cross_split": False, "exact_duplicate_image_count": len(records) - len({record["image_sha256"] for record in records}), "perceptual_hash_duplicate_count": near_duplicate_count, "per_class_instance_count": per_class_instances, "records": records, "rendered_synthetic_perception_claim_only": True, "gazebo_camera_rendered": False, "competition_perception_pass": False}
    (root / "dataset_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    return manifest


def validate_annotations(root: str | Path) -> dict:
    root = Path(root)
    manifest = json.loads((root / "dataset_manifest.json").read_text(encoding="utf-8"))
    errors = []
    occluded = truncated = area_targets = negatives = 0
    for record in manifest["records"]:
        semantic = cv2.imread(str(root / record["semantic"]), cv2.IMREAD_UNCHANGED)
        instances = cv2.imread(str(root / record["instances"]), cv2.IMREAD_UNCHANGED)
        depth = np.load(root / record["depth"], allow_pickle=False)
        annotation = json.loads((root / record["annotation"]).read_text(encoding="utf-8"))
        negatives += len(annotation["negatives"])
        if any(item.get("target_label_count") != 0 for item in annotation["negatives"]):
            errors.append({"record": record["image"], "reason": "negative_has_target_label"})
        for obj in annotation["objects"]:
            mask = instances == obj["instance_id"]
            rows, cols = np.nonzero(mask)
            if rows.size != obj["pixel_area"] or not np.all(semantic[mask] == obj["class_index"]):
                errors.append({"record": record["image"], "instance": obj["instance_id"], "reason": "mask_semantic_mismatch"})
                continue
            x, y, width, height = obj["bbox_xywh"]
            if (int(cols.min()), int(rows.min()), int(cols.max() + 1), int(rows.max() + 1)) != (x, y, x + width, y + height):
                errors.append({"record": record["image"], "instance": obj["instance_id"], "reason": "mask_box_mismatch"})
            valid_depth = depth[mask][np.isfinite(depth[mask])]
            if valid_depth.size < 3 or abs(float(np.median(valid_depth)) - obj["depth_m"]) > 0.08:
                errors.append({"record": record["image"], "instance": obj["instance_id"], "reason": "depth_mask_mismatch"})
            if obj["occlusion_ratio"] > 0.05:
                occluded += 1
            if obj["truncation"] > 0:
                truncated += 1
            if obj["target_type"] == "area":
                area_targets += 1
    split_sets = {name: set(values) for name, values in manifest["split_assets"].items()}
    leakage = bool(split_sets["train"] & split_sets["val"] or split_sets["train"] & split_sets["test"] or split_sets["val"] & split_sets["test"])
    error_rate = len(errors) / max(sum(manifest["per_class_instance_count"].values()), 1)
    report = {"schema_version": 1, "checked_frame_count": manifest["frame_count"], "checked_instance_count": sum(manifest["per_class_instance_count"].values()), "checked_negative_count": negatives, "occluded_target_count": occluded, "truncated_target_count": truncated, "area_target_count": area_targets, "annotation_error_count": len(errors), "annotation_error_rate": error_rate, "asset_split_leakage": leakage, "errors": errors[:100], "gates": {"each_class_at_least_100_instances": all(value >= 100 for value in manifest["per_class_instance_count"].values()), "negative_samples_at_least_100": negatives >= 100, "occluded_targets_at_least_50": occluded >= 50, "truncated_targets_at_least_50": truncated >= 50, "area_targets_at_least_50": area_targets >= 50, "annotation_error_rate_at_most_0_01": error_rate <= 0.01, "asset_split_leakage_zero": not leakage}}
    report["annotation_qa_pass"] = all(report["gates"].values())
    return report
