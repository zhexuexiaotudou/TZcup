from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np


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
CLASS_COLORS_RGB = np.array(
    [
        [32, 32, 32],
        [230, 30, 30],
        [30, 230, 30],
        [30, 30, 230],
        [220, 185, 30],
        [30, 185, 220],
    ],
    dtype=np.uint8,
)
TARGET_TYPE = {
    "plastic_bottle": "discrete",
    "metal_can": "discrete",
    "paper_litter": "discrete",
    "leaf_pile": "area",
    "puddle": "area",
}
POLICY = {
    "plastic_bottle": "spot_clean",
    "metal_can": "spot_clean",
    "paper_litter": "spot_clean",
    "leaf_pile": "local_coverage",
    "puddle": "local_coverage",
}


@dataclass(frozen=True)
class SyntheticScene:
    seed: int
    image_rgb: np.ndarray
    depth_m: np.ndarray
    labels: np.ndarray
    objects: tuple[dict, ...]
    camera: dict
    transform_map_camera: tuple[tuple[float, ...], ...]


def split_for_seed(seed: int) -> str:
    slot = int(seed) % 20
    if slot < 12:
        return "train"
    if slot < 16:
        return "val"
    return "test"


def _object_metadata(class_id: str, class_index: int, mask: np.ndarray, depth_m: float) -> dict:
    rows, cols = np.nonzero(mask)
    x0, x1 = int(cols.min()), int(cols.max() + 1)
    y0, y1 = int(rows.min()), int(rows.max() + 1)
    center_u, center_v = float(cols.mean()), float(rows.mean())
    map_x = (center_u - WIDTH / 2.0) * 0.02
    map_y = (center_v - HEIGHT / 2.0) * 0.02
    return {
        "class_id": class_id,
        "class_index": class_index,
        "target_type": TARGET_TYPE[class_id],
        "cleaning_policy": POLICY[class_id],
        "bbox_xywh": [x0, y0, x1 - x0, y1 - y0],
        "segmentation": [[x0, y0, x1, y0, x1, y1, x0, y1]],
        "pixel_area": int(mask.sum()),
        "center_uv": [center_u, center_v],
        "map_pose": [map_x, map_y, 0.0],
        "depth_m": depth_m,
        "visibility": 1.0,
        "occlusion_ratio": 0.0,
        "truncation": 0.0,
    }


def generate_scene(seed: int) -> SyntheticScene:
    rng = np.random.default_rng(int(seed))
    labels = np.zeros((HEIGHT, WIDTH), dtype=np.uint8)
    image = np.broadcast_to(CLASS_COLORS_RGB[0], (HEIGHT, WIDTH, 3)).copy()
    depth = np.full((HEIGHT, WIDTH), 3.0, dtype=np.float32)
    jitter = rng.integers(-3, 4, size=(5, 2))
    centers = np.array([[22, 23], [63, 24], [103, 23], [38, 69], [91, 69]]) + jitter

    masks: list[np.ndarray] = []
    mask = np.zeros_like(labels); cv2.rectangle(mask, tuple(centers[0] - [4, 9]), tuple(centers[0] + [4, 9]), 1, -1); masks.append(mask)
    mask = np.zeros_like(labels); cv2.circle(mask, tuple(centers[1]), 7, 1, -1); masks.append(mask)
    mask = np.zeros_like(labels); cv2.rectangle(mask, tuple(centers[2] - [10, 5]), tuple(centers[2] + [10, 5]), 1, -1); masks.append(mask)
    mask = np.zeros_like(labels); cv2.ellipse(mask, tuple(centers[3]), (17, 10), float(rng.integers(-15, 16)), 0, 360, 1, -1); masks.append(mask)
    mask = np.zeros_like(labels); cv2.ellipse(mask, tuple(centers[4]), (20, 11), float(rng.integers(-12, 13)), 0, 360, 1, -1); masks.append(mask)

    objects = []
    for class_index, mask in enumerate(masks, 1):
        image[mask.astype(bool)] = CLASS_COLORS_RGB[class_index]
        labels[mask.astype(bool)] = class_index
        object_depth = 1.5 + 0.15 * class_index + float(rng.uniform(-0.02, 0.02))
        depth[mask.astype(bool)] = object_depth
        objects.append(_object_metadata(CLASS_ORDER[class_index], class_index, mask, object_depth))

    # Explicit negative sample: a neutral obstacle never represented in target labels.
    cv2.rectangle(image, (3, 40), (14, 61), (105, 95, 85), -1)
    depth[40:62, 3:15] = 1.25
    brightness = float(rng.uniform(0.97, 1.03))
    noise = rng.normal(0.0, 1.2, image.shape)
    image = np.clip(image.astype(np.float32) * brightness + noise, 0, 255).astype(np.uint8)
    camera = {
        "width": WIDTH,
        "height": HEIGHT,
        "fx": 50.0,
        "fy": 50.0,
        "cx": WIDTH / 2.0,
        "cy": HEIGHT / 2.0,
        "projection": "synthetic_top_down_metric_grid",
        "map_m_per_pixel": 0.02,
    }
    transform = (
        (1.0, 0.0, 0.0, 0.0),
        (0.0, 1.0, 0.0, 0.0),
        (0.0, 0.0, 1.0, 0.0),
        (0.0, 0.0, 0.0, 1.0),
    )
    return SyntheticScene(int(seed), image, depth, labels, tuple(objects), camera, transform)


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_dataset(output: str | Path, seeds: range | list[int]) -> dict:
    root = Path(output)
    for subdirectory in ("images", "depth", "annotations", "camera", "tf", "scene_manifests", "splits"):
        (root / subdirectory).mkdir(parents=True, exist_ok=True)
    coco_images = []
    coco_annotations = []
    split_seeds = {"train": [], "val": [], "test": []}
    scene_records = []
    annotation_id = 1
    for image_id, seed in enumerate(seeds, 1):
        scene = generate_scene(int(seed))
        stem = f"scene_{int(seed):04d}"
        split = split_for_seed(int(seed))
        split_seeds[split].append(int(seed))
        image_path = root / "images" / f"{stem}.png"
        depth_path = root / "depth" / f"{stem}.npy"
        annotation_path = root / "annotations" / f"{stem}.json"
        camera_path = root / "camera" / f"{stem}.json"
        tf_path = root / "tf" / f"{stem}.json"
        cv2.imwrite(str(image_path), cv2.cvtColor(scene.image_rgb, cv2.COLOR_RGB2BGR))
        np.save(depth_path, scene.depth_m, allow_pickle=False)
        annotation_path.write_text(json.dumps({"objects": scene.objects}, indent=2) + "\n", encoding="utf-8")
        camera_path.write_text(json.dumps(scene.camera, indent=2) + "\n", encoding="utf-8")
        tf_path.write_text(json.dumps({"T_map_camera": scene.transform_map_camera}, indent=2) + "\n", encoding="utf-8")
        coco_images.append({"id": image_id, "file_name": image_path.name, "width": WIDTH, "height": HEIGHT, "scene_seed": int(seed), "split": split})
        for obj in scene.objects:
            coco_annotations.append({
                "id": annotation_id,
                "image_id": image_id,
                "category_id": obj["class_index"],
                "bbox": obj["bbox_xywh"],
                "area": obj["pixel_area"],
                "segmentation": obj["segmentation"],
                "iscrowd": 0,
                "map_pose": obj["map_pose"],
            })
            annotation_id += 1
        record = {
            "scene_seed": int(seed),
            "split": split,
            "image": str(image_path.relative_to(root)).replace("\\", "/"),
            "depth": str(depth_path.relative_to(root)).replace("\\", "/"),
            "annotation": str(annotation_path.relative_to(root)).replace("\\", "/"),
            "camera": str(camera_path.relative_to(root)).replace("\\", "/"),
            "tf": str(tf_path.relative_to(root)).replace("\\", "/"),
            "image_sha256": _sha256(image_path),
            "depth_sha256": _sha256(depth_path),
        }
        scene_manifest_path = root / "scene_manifests" / f"{stem}.json"
        scene_manifest_path.write_text(json.dumps(record, indent=2) + "\n", encoding="utf-8")
        scene_records.append(record)
    categories = [{"id": i, "name": name} for i, name in enumerate(CLASS_ORDER[1:], 1)]
    coco = {"images": coco_images, "annotations": coco_annotations, "categories": categories}
    (root / "annotations" / "coco.json").write_text(json.dumps(coco, indent=2) + "\n", encoding="utf-8")
    for split, values in split_seeds.items():
        (root / "splits" / f"{split}.json").write_text(json.dumps({"scene_seeds": values}, indent=2) + "\n", encoding="utf-8")
    image_hashes = [record["image_sha256"] for record in scene_records]
    split_hash = hashlib.sha256(json.dumps(split_seeds, sort_keys=True).encode("utf-8")).hexdigest()
    manifest = {
        "schema_version": 1,
        "dataset_id": "stage5a_synthetic_smoke_v1",
        "scene_count": len(scene_records),
        "class_order": list(CLASS_ORDER),
        "split_scene_seeds": split_seeds,
        "split_hash": split_hash,
        "adjacent_frames_cross_split": False,
        "duplicate_image_count": len(image_hashes) - len(set(image_hashes)),
        "records": scene_records,
        "synthetic_only": True,
    }
    (root / "dataset_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    calibration = {
        "schema_version": 1,
        "dataset_id": manifest["dataset_id"],
        "representative_scene_seeds": split_seeds["train"][:10],
        "image_sha256": [record["image_sha256"] for record in scene_records if record["split"] == "train"][:10],
        "j6_quantization_executed": False,
    }
    (root / "calibration_dataset_manifest.json").write_text(json.dumps(calibration, indent=2) + "\n", encoding="utf-8")
    return manifest
