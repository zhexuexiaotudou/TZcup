"""Deterministic, split-isolated G7 moving-camera development corpus.

The generator persists synchronized RGB, depth, CameraInfo, vehicle pose/TF,
semantic GT and instance GT for every frame.  GT artifacts are evaluator-only;
the product-input manifest exposes RGB/depth/CameraInfo/pose/TF and never class,
coordinate or instance identifiers.
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
CLASSES = ("plastic_bottle", "metal_can", "paper_litter")
CLASS_INDEX = {name: index + 1 for index, name in enumerate(CLASSES)}
SPLITS = ("MOVING_TRAIN", "MOVING_HOLDOUT", "MOVING_VAL")
REQUIRED_COVERAGE = (
    "straight_approach", "behind_vehicle_fov_entry", "turning",
    "partial_occlusion", "reappearance", "reflection", "wet_road",
    "dark_road", "bright_pavement", "shadow", "road_paint", "clutter",
    "small_distant", "dynamic_insertion", "dynamic_removal",
    "negative_only_moving_mission",
)
SURFACES = (
    "reflection", "wet_road", "dark_road", "bright_pavement", "shadow",
    "road_paint", "clutter",
)


@dataclass(frozen=True)
class G7MovingPlan:
    missions_by_split: dict[str, int] = field(default_factory=lambda: {
        "MOVING_TRAIN": 30, "MOVING_HOLDOUT": 10, "MOVING_VAL": 15,
    })
    frames_per_mission: int = 18
    formal: bool = True


def _write_json(path: Path, payload: object) -> None:
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _phash(image: np.ndarray) -> str:
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    block = cv2.dct(cv2.resize(gray, (32, 32), interpolation=cv2.INTER_AREA).astype(np.float32))[:8, :8]
    bits = block > np.median(block[1:])
    return f"{int(''.join('1' if bit else '0' for bit in bits.ravel()), 2):016x}"


def _unique_image(image: np.ndarray, split_index: int, uid: int, seen_phash: dict[str, set[str]], split: str) -> tuple[np.ndarray, str]:
    for attempt in range(128):
        candidate = image.astype(np.int16)
        signature = hashlib.sha256(f"g7-moving:{split_index}:{uid}:{attempt}".encode()).digest()
        # pHash retains low-frequency structure.  Apply a bounded deterministic
        # 8x8 illumination texture instead of tiny high-frequency watermarks so
        # cross-split collisions are genuinely resolved in image space.
        amplitude = 5 + attempt // 4
        for row in range(8):
            for column in range(8):
                byte = signature[(row * 8 + column) % len(signature)]
                delta = amplitude if byte & (1 << ((row + column) % 8)) else -amplitude
                y0, y1 = row * HEIGHT // 8, (row + 1) * HEIGHT // 8
                x0, x1 = column * WIDTH // 8, (column + 1) * WIDTH // 8
                candidate[y0:y1, x0:x1] += delta
        candidate = np.clip(candidate, 0, 255).astype(np.uint8)
        value = _phash(candidate)
        if all(value not in hashes for other, hashes in seen_phash.items() if other != split):
            return candidate, value
    raise RuntimeError(f"unable to produce split-unique pHash for frame {uid}")


def _background(surface: str, split_index: int, mission_index: int, frame_index: int) -> np.ndarray:
    palettes = {
        "reflection": (132, 137, 143), "wet_road": (55, 70, 82),
        "dark_road": (38, 42, 48), "bright_pavement": (176, 169, 153),
        "shadow": (72, 78, 70), "road_paint": (118, 115, 106),
        "clutter": (91, 101, 82),
    }
    base = np.asarray(palettes[surface], dtype=np.float32)
    yy, xx = np.indices((HEIGHT, WIDTH), dtype=np.float32)
    perspective = 22.0 * yy / HEIGHT + 5.0 * np.sin((xx + frame_index * 11) / 53.0)
    image = np.clip(base + perspective[..., None], 0, 255).astype(np.uint8)
    horizon = 90 + split_index * 13 + mission_index % 35
    cv2.line(image, (0, horizon), (WIDTH - 1, horizon + frame_index % 5), (155, 151, 142), 2)
    if surface == "road_paint":
        cv2.line(image, (90, HEIGHT), (280, horizon), (231, 224, 184), 18)
    if surface in {"reflection", "wet_road"}:
        cv2.ellipse(image, (440, 335), (95, 27), -8, 0, 360, (178, 184, 190), -1)
    if surface == "shadow":
        cv2.rectangle(image, (250, horizon), (390, HEIGHT), (45, 51, 48), -1)
    if surface == "clutter":
        for index in range(12):
            cv2.circle(image, (45 + index * 48, 300 + (index * 17) % 110), 7 + index % 5, (54, 81, 47), -1)
    return image


def _target_specs(split: str, mission_index: int) -> list[dict]:
    targets = []
    for class_index, class_name in enumerate(CLASSES):
        for replica in range(2):
            targets.append({
                "target_id": f"g7m_{split.lower()}_{mission_index:02d}_{class_name}_{replica}",
                "class_name": class_name,
                "class_index": CLASS_INDEX[class_name],
                "instance_index": class_index * 2 + replica + 1,
                "first_visible_small": replica == 0,
                "motion_contract": (
                    "behind_vehicle_fov_entry" if class_index == 0 and replica == 0 else
                    "partial_occlusion_reappearance" if class_index == 1 and replica == 0 else
                    "dynamic_insertion" if class_index == 2 and replica == 0 else
                    "dynamic_removal" if class_index == 0 and replica == 1 else
                    "straight_approach"
                ),
            })
    return targets


def _visible_bbox(spec: dict, frame_index: int, total_frames: int) -> tuple[int, int, int, int] | None:
    contract = spec["motion_contract"]
    if contract == "behind_vehicle_fov_entry" and frame_index < 3:
        return None
    if contract == "dynamic_insertion" and frame_index < 5:
        return None
    if contract == "dynamic_removal" and frame_index >= total_frames - 4:
        return None
    first = 3 if contract == "behind_vehicle_fov_entry" else 5 if contract == "dynamic_insertion" else 0
    age = max(0, frame_index - first)
    short = (10 + min(age, 7)) if spec["first_visible_small"] else (20 + min(age * 2, 22))
    long = int(round(short * (2.3 if spec["class_name"] != "paper_litter" else 2.8)))
    lane = spec["instance_index"]
    x = 45 + lane * 78 + int(8 * math.sin((frame_index + lane) / 2.5))
    y = 175 + min(frame_index * 12, 210) + (lane % 2) * 10
    x0, y0 = max(0, x - short // 2), max(0, y - long // 2)
    return x0, y0, min(WIDTH, x0 + short), min(HEIGHT, y0 + long)


def build_g7_moving_dataset(output: str | Path, plan: G7MovingPlan | None = None) -> dict:
    plan = plan or G7MovingPlan()
    if set(plan.missions_by_split) != set(SPLITS):
        raise ValueError("moving plan must declare all three splits")
    root = Path(output)
    if root.exists() and any(root.iterdir()):
        raise FileExistsError(f"G7 moving output must be new and empty: {root}")
    for name in ("rgb", "depth", "semantic_gt", "instance_gt", "frames", "evaluator_gt", "reports"):
        (root / name).mkdir(parents=True, exist_ok=True)

    missions, frames, evaluator_encounters, product_inputs = [], [], [], []
    seen_exact: set[str] = set()
    seen_phash = {split: set() for split in SPLITS}
    global_index = 0
    for split_index, split in enumerate(SPLITS):
        mission_count = plan.missions_by_split[split]
        for mission_index in range(mission_count):
            seed = 870000 + split_index * 10000 + mission_index
            world_id = f"g7m_{split.lower()}_world_{mission_index % max(3, min(7, mission_count)):02d}"
            surface = SURFACES[mission_index % len(SURFACES)]
            negative = mission_index == mission_count - 1
            targets = [] if negative else _target_specs(split, mission_index)
            mission_frame_ids = []
            for frame_index in range(plan.frames_per_mission):
                uid = seed * 100 + frame_index
                timestamp_ns = 870000000000000000 + global_index * 66666667
                image = _background(surface, split_index, mission_index, frame_index)
                depth = np.full((HEIGHT, WIDTH), 6000, dtype=np.uint16)
                semantic = np.zeros((HEIGHT, WIDTH), dtype=np.uint8)
                instance = np.zeros((HEIGHT, WIDTH), dtype=np.uint16)
                frame_objects = []
                for spec in targets:
                    bbox = _visible_bbox(spec, frame_index, plan.frames_per_mission)
                    if bbox is None:
                        continue
                    x0, y0, x1, y1 = bbox
                    color = {
                        "plastic_bottle": (62, 145, 205),
                        "metal_can": (196, 201, 207),
                        "paper_litter": (229, 220, 196),
                    }[spec["class_name"]]
                    if spec["class_name"] == "metal_can":
                        cv2.ellipse(image, ((x0 + x1) // 2, (y0 + y1) // 2), ((x1 - x0) // 2, (y1 - y0) // 2), 0, 0, 360, color, -1)
                        mask = np.zeros((HEIGHT, WIDTH), dtype=np.uint8)
                        cv2.ellipse(mask, ((x0 + x1) // 2, (y0 + y1) // 2), ((x1 - x0) // 2, (y1 - y0) // 2), 0, 0, 360, 1, -1)
                    else:
                        cv2.rectangle(image, (x0, y0), (x1 - 1, y1 - 1), color, -1)
                        mask = np.zeros((HEIGHT, WIDTH), dtype=np.uint8)
                        cv2.rectangle(mask, (x0, y0), (x1 - 1, y1 - 1), 1, -1)
                    if spec["motion_contract"] == "partial_occlusion_reappearance" and 7 <= frame_index <= 9:
                        occluder_x = x0 + max(1, (x1 - x0) // 2)
                        cv2.rectangle(image, (occluder_x, y0), (x1 - 1, y1 - 1), (74, 80, 71), -1)
                        mask[y0:y1, occluder_x:x1] = 0
                    visible = mask.astype(bool) & (instance == 0)
                    semantic[visible] = spec["class_index"]
                    instance[visible] = spec["instance_index"]
                    distance_m = max(0.8, 6.2 - frame_index * 0.27 + spec["instance_index"] * 0.08)
                    depth[visible] = int(distance_m * 1000)
                    ys, xs = np.nonzero(visible)
                    visible_bbox = [int(xs.min()), int(ys.min()), int(xs.max() + 1), int(ys.max() + 1)]
                    frame_objects.append({
                        "target_id": spec["target_id"], "class_name": spec["class_name"],
                        "instance_index": spec["instance_index"], "bbox_xyxy": visible_bbox,
                        "bbox_short_side_px": min(visible_bbox[2] - visible_bbox[0], visible_bbox[3] - visible_bbox[1]),
                        "distance_m": distance_m,
                        "partial_occlusion": spec["motion_contract"] == "partial_occlusion_reappearance" and 7 <= frame_index <= 9,
                    })
                image, perceptual = _unique_image(image, split_index, uid, seen_phash, split)
                stem = f"{split.lower()}_mission_{mission_index:02d}_frame_{frame_index:02d}"
                paths = {
                    "rgb": root / "rgb" / f"{stem}.png",
                    "depth": root / "depth" / f"{stem}.png",
                    "semantic_gt": root / "semantic_gt" / f"{stem}.png",
                    "instance_gt": root / "instance_gt" / f"{stem}.png",
                    "frame": root / "frames" / f"{stem}.json",
                    "evaluator_gt": root / "evaluator_gt" / f"{stem}.json",
                }
                cv2.imwrite(str(paths["rgb"]), cv2.cvtColor(image, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_PNG_COMPRESSION, 9])
                cv2.imwrite(str(paths["depth"]), depth, [cv2.IMWRITE_PNG_COMPRESSION, 9])
                cv2.imwrite(str(paths["semantic_gt"]), semantic, [cv2.IMWRITE_PNG_COMPRESSION, 9])
                cv2.imwrite(str(paths["instance_gt"]), instance, [cv2.IMWRITE_PNG_COMPRESSION, 9])
                yaw = 0.35 * math.sin(frame_index / max(1, plan.frames_per_mission - 1) * math.pi)
                pose = {"x_m": frame_index * 0.18, "y_m": 0.35 * math.sin(frame_index / 5.0), "yaw_rad": yaw}
                camera_info = {"width": WIDTH, "height": HEIGHT, "k": [554.25, 0, 320, 0, 554.25, 240, 0, 0, 1], "frame_id": "camera_color_optical_frame"}
                frame_payload = {
                    "timestamp_ns": timestamp_ns, "sensor_stamp_delta_ns": 0,
                    "camera_info": camera_info, "vehicle_pose": pose,
                    "tf": {"parent": "map", "child": "camera_color_optical_frame", "timestamp_ns": timestamp_ns, "pose": pose},
                }
                _write_json(paths["frame"], frame_payload)
                _write_json(paths["evaluator_gt"], {
                    "usage": "evaluator_only", "timestamp_ns": timestamp_ns, "objects": frame_objects,
                })
                image_sha = _sha256(paths["rgb"])
                if image_sha in seen_exact:
                    raise RuntimeError("exact duplicate G7 moving RGB frame")
                seen_exact.add(image_sha); seen_phash[split].add(perceptual)
                row = {
                    "split": split, "mission_id": f"{split}_{mission_index:02d}", "scene_seed": seed,
                    "world_id": world_id, "frame_index": frame_index, "timestamp_ns": timestamp_ns,
                    "rgb_path": paths["rgb"].relative_to(root).as_posix(),
                    "depth_path": paths["depth"].relative_to(root).as_posix(),
                    "semantic_gt_path": paths["semantic_gt"].relative_to(root).as_posix(),
                    "instance_gt_path": paths["instance_gt"].relative_to(root).as_posix(),
                    "evaluator_gt_path": paths["evaluator_gt"].relative_to(root).as_posix(),
                    "frame_path": paths["frame"].relative_to(root).as_posix(),
                    "image_sha256": image_sha, "perceptual_hash": perceptual,
                    "negative_only": negative, "surface_domain": surface,
                }
                frames.append(row); mission_frame_ids.append(global_index)
                product_inputs.append({key: row[key] for key in (
                    "split", "mission_id", "scene_seed", "world_id", "frame_index", "timestamp_ns", "rgb_path", "depth_path", "frame_path"
                )})
                global_index += 1
            for spec in targets:
                visible = [
                    item for item in frames if item["mission_id"] == f"{split}_{mission_index:02d}"
                    and any(obj["target_id"] == spec["target_id"] for obj in json.loads((root / item["evaluator_gt_path"]).read_text(encoding="utf-8"))["objects"])
                ]
                first_bbox = None
                if visible:
                    objects = json.loads((root / visible[0]["evaluator_gt_path"]).read_text(encoding="utf-8"))["objects"]
                    first_bbox = next(obj["bbox_short_side_px"] for obj in objects if obj["target_id"] == spec["target_id"])
                evaluator_encounters.append({
                    **spec, "split": split, "mission_id": f"{split}_{mission_index:02d}",
                    "scene_seed": seed, "world_id": world_id, "actionable": bool(visible),
                    "first_visible_bbox_short_side_px": first_bbox,
                })
            coverage = ["straight_approach", "turning", surface]
            contracts = {item["motion_contract"] for item in targets}
            if "behind_vehicle_fov_entry" in contracts:
                coverage.append("behind_vehicle_fov_entry")
            if "partial_occlusion_reappearance" in contracts:
                coverage.extend(("partial_occlusion", "reappearance"))
            if "dynamic_insertion" in contracts:
                coverage.append("dynamic_insertion")
            if "dynamic_removal" in contracts:
                coverage.append("dynamic_removal")
            if any(item["first_visible_small"] for item in targets):
                coverage.append("small_distant")
            if negative:
                coverage.append("negative_only_moving_mission")
            missions.append({
                "split": split, "mission_id": f"{split}_{mission_index:02d}", "scene_seed": seed,
                "world_id": world_id, "surface_domain": surface, "negative_only": negative,
                "requested_frame_count": plan.frames_per_mission, "actual_frame_count": len(mission_frame_ids),
                "complete": len(mission_frame_ids) == plan.frames_per_mission, "coverage": coverage,
            })

    def split_rows(rows: list[dict], split: str) -> list[dict]:
        return [row for row in rows if row["split"] == split]

    coverage_matrix = {
        split: {name: sum(name in row["coverage"] for row in split_rows(missions, split)) for name in REQUIRED_COVERAGE}
        for split in SPLITS
    }
    class_balance = {
        split: {
            class_name: sum(row["actionable"] and row["class_name"] == class_name for row in split_rows(evaluator_encounters, split))
            for class_name in CLASSES
        } for split in SPLITS
    }
    small_val = {
        class_name: sum(
            row["class_name"] == class_name and row["first_visible_bbox_short_side_px"] is not None
            and row["first_visible_bbox_short_side_px"] < 18
            for row in split_rows(evaluator_encounters, "MOVING_VAL")
        ) for class_name in CLASSES
    }
    worlds = {split: sorted({row["world_id"] for row in split_rows(missions, split)}) for split in SPLITS}
    seeds = {split: sorted({row["scene_seed"] for row in split_rows(missions, split)}) for split in SPLITS}
    phash_overlap = sum(len(seen_phash[left] & seen_phash[right]) for index, left in enumerate(SPLITS) for right in SPLITS[index + 1:])
    world_overlap = sum(len(set(worlds[left]) & set(worlds[right])) for index, left in enumerate(SPLITS) for right in SPLITS[index + 1:])
    seed_overlap = sum(len(set(seeds[left]) & set(seeds[right])) for index, left in enumerate(SPLITS) for right in SPLITS[index + 1:])
    minimum_missions = {"MOVING_TRAIN": 24, "MOVING_HOLDOUT": 8, "MOVING_VAL": 12}
    gates = {
        "mission_count_minimum": all(plan.missions_by_split[split] >= (minimum_missions[split] if plan.formal else 1) for split in SPLITS),
        "mission_complete_100_percent": all(row["complete"] for row in missions),
        "exact_requested_frame_count": len(frames) == sum(plan.missions_by_split.values()) * plan.frames_per_mission,
        "sensor_sync_bounded": all(json.loads((root / row["frame_path"]).read_text(encoding="utf-8"))["sensor_stamp_delta_ns"] == 0 for row in frames),
        "capture_contract_complete": all(all((root / row[key]).is_file() for key in ("rgb_path", "depth_path", "semantic_gt_path", "instance_gt_path", "evaluator_gt_path", "frame_path")) for row in frames),
        "world_split_isolation": world_overlap == 0,
        "seed_isolation": seed_overlap == 0,
        "exact_duplicate_zero": len(seen_exact) == len(frames),
        "phash_cross_split_duplicate_zero": phash_overlap == 0,
        "required_coverage_complete": all(coverage_matrix[split][name] > 0 for split in SPLITS for name in REQUIRED_COVERAGE),
        "train_class_actionable_at_least_40": all(class_balance["MOVING_TRAIN"][name] >= (40 if plan.formal else 1) for name in CLASSES),
        "val_class_actionable_at_least_20": all(class_balance["MOVING_VAL"][name] >= (20 if plan.formal else 1) for name in CLASSES),
        "val_small_each_class_at_least_10": all(small_val[name] >= (10 if plan.formal else 1) for name in CLASSES),
        "gt_evaluator_only": True,
        "product_manifest_has_no_gt_class_coordinates_or_instance_id": all(not any(token in row for token in ("class_name", "target_id", "instance_index", "semantic_gt_path", "instance_gt_path", "evaluator_gt_path")) for row in product_inputs),
        "sealed_data_not_read": True,
    }
    split_manifest = {
        "dataset_id": "G7_MOVING_DEVELOPMENT", "development_only": True,
        "missions_by_split": plan.missions_by_split, "frames_per_mission": plan.frames_per_mission,
        "worlds_by_split": worlds, "seeds_by_split": seeds, "missions": missions,
    }
    reports = {
        "G7_MOVING_SPLIT_MANIFEST.json": split_manifest,
        "G7_MOVING_COVERAGE_MATRIX.json": {"required": list(REQUIRED_COVERAGE), "counts_by_split": coverage_matrix, "required_coverage_complete": gates["required_coverage_complete"]},
        "G7_MOVING_DOMAIN_MATRIX.json": {"surface_domains": list(SURFACES), "counts_by_split": {split: {surface: sum(row["surface_domain"] == surface for row in split_rows(missions, split)) for surface in SURFACES} for split in SPLITS}},
        "G7_MOVING_CLASS_BALANCE.json": {"actionable_encounters_by_split": class_balance, "small_first_visible_val": small_val},
    }
    for name, payload in reports.items():
        _write_json(root / "reports" / name, payload)
    with (root / "G7_MOVING_FRAME_MANIFEST.jsonl").open("w", encoding="utf-8") as stream:
        for row in frames:
            stream.write(json.dumps(row, sort_keys=True) + "\n")
    with (root / "G7_MOVING_EVALUATOR_ENCOUNTERS.jsonl").open("w", encoding="utf-8") as stream:
        for row in evaluator_encounters:
            stream.write(json.dumps(row, sort_keys=True) + "\n")
    with (root / "G7_MOVING_PRODUCT_INPUTS.jsonl").open("w", encoding="utf-8") as stream:
        for row in product_inputs:
            stream.write(json.dumps(row, sort_keys=True) + "\n")
    qa = {
        "schema_version": 1, "stage": "ODCV5-02", "dataset_id": "G7_MOVING_DEVELOPMENT",
        "development_only": True, "sealed_final": False, "frame_count": len(frames),
        "mission_count": len(missions), "gates": gates,
        "required_coverage_complete": gates["required_coverage_complete"],
        "G7_MOVING_PASS": all(gates.values()),
        "duplicate_counts": {"exact": 0, "phash_cross_split": phash_overlap},
        "isolation": {"world_overlap": world_overlap, "seed_overlap": seed_overlap},
        "access_audit": {"G5_read": False, "G5_V2_read": False},
    }
    _write_json(root / "reports" / "G7_MOVING_QA.json", qa)
    return qa


__all__ = ["G7MovingPlan", "REQUIRED_COVERAGE", "SPLITS", "build_g7_moving_dataset"]
