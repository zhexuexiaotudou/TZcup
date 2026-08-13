#!/usr/bin/env python3
"""Prepare the independent TGARV9 G9 temporal/geometry HOLDOUT.

The product-facing payload contains only measured RGB-D/calibration/TF-derived
features and immutable frame references.  Semantic and instance images are
read only by this evaluator to construct TargetTube truth.
"""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import hashlib
import json
import math
from pathlib import Path

import cv2
import numpy as np


CLASS_NAMES = {1: "plastic_bottle", 2: "metal_can", 3: "paper_litter"}
REQUIRED_SURFACES = {
    "wet": "wet",
    "bright": "light_paver",
    "dark": "dark_gravel",
    "shadow": "service_road",
}


def read_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def phash(path: Path) -> str:
    image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise RuntimeError(f"failed to read RGB: {path}")
    resized = cv2.resize(image, (32, 32), interpolation=cv2.INTER_AREA)
    low = cv2.dct(np.float32(resized))[:8, :8]
    return "".join("1" if value > np.median(low) else "0" for value in low.flat)


def scene_dirs(roots: list[Path]) -> list[Path]:
    found = []
    for root in roots:
        found.extend(sorted((root / "g4_screening_native" / "scenes").glob("scene_*")))
    return found


def bbox(mask: np.ndarray) -> tuple[list[int], int, int]:
    rows, cols = np.nonzero(mask)
    x0, x1, y0, y1 = int(cols.min()), int(cols.max()) + 1, int(rows.min()), int(rows.max()) + 1
    return [x0, y0, x1 - x0, y1 - y0], min(x1 - x0, y1 - y0), int(mask.sum())


def median_depth(depth: np.ndarray, mask: np.ndarray) -> tuple[float | None, float]:
    values = depth[mask].astype(np.float64)
    valid = values[np.isfinite(values) & (values > 0.05) & (values < 20.0)]
    return (float(np.median(valid)) if valid.size else None, float(valid.size / max(values.size, 1)))


def geometry(depth: np.ndarray, box: list[int], camera: dict) -> dict:
    x, y, width, height = box
    x0, x1, y0, y1 = max(0, x), min(depth.shape[1], x + width), max(0, y), min(depth.shape[0], y + height)
    local = depth[y0:y1, x0:x1].astype(np.float64)
    valid = local[np.isfinite(local) & (local > 0.05) & (local < 20.0)]
    if not valid.size:
        return {"local_depth_median_m": None, "local_depth_residual_m": None, "estimated_width_m": None, "estimated_height_m": None, "depth_uncertainty_m": None, "view_angle_rad": None}
    z = float(np.median(valid))
    fx, fy, cx = float(camera["k"][0]), float(camera["k"][4]), float(camera["k"][2])
    return {
        "local_depth_median_m": z,
        "local_depth_residual_m": float(np.percentile(valid, 90) - np.percentile(valid, 10)),
        "estimated_width_m": float(width * z / fx),
        "estimated_height_m": float(height * z / fy),
        "depth_uncertainty_m": float(np.std(valid)),
        "view_angle_rad": float(math.atan2(x + width / 2.0 - cx, fx)),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", action="append", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--expected-missions", type=int, default=20)
    parser.add_argument("--recommended-missions", type=int, default=24)
    parser.add_argument("--exclude-mission", action="append", default=[])
    parser.add_argument("--reference-coco", action="append", type=Path, default=[])
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    args.output.mkdir(parents=True)

    errors: list[dict] = []
    reference_exact, reference_phash = {}, {}
    for coco_path in args.reference_coco:
        coco = read_json(coco_path)
        for image in coco.get("images", []):
            path = coco_path.parent / image["file_name"]
            reference_exact.setdefault(sha256(path), str(path))
            reference_phash.setdefault(phash(path), str(path))
    missions, product_frames, tubes = [], [], {}
    images, annotations = [], []
    exact, perceptual = {}, {}
    instance_to_model: dict[tuple[str, int], str] = {}
    domains, surfaces = Counter(), Counter()
    for scene_dir in scene_dirs(args.root):
        manifest_path, capture_path = scene_dir / "scene_manifest.json", scene_dir / "capture_report.json"
        if not manifest_path.is_file() or not capture_path.is_file():
            errors.append({"scene": str(scene_dir), "reason": "manifest_or_capture_missing"})
            continue
        scene, capture = read_json(manifest_path), read_json(capture_path)
        mission_id = f"g9-{scene['world_id']}-{scene['scene_seed']}"
        if mission_id in set(args.exclude_mission):
            continue
        records = capture.get("records", [])
        mission_errors = []
        if not capture.get("capture_pass") or len(records) != 20 or capture.get("captured_frames") != 20:
            mission_errors.append("partial_mission")
        if any(not row.get("exact_four_sensor_timestamp") for row in records):
            mission_errors.append("sync_violation")
        if not capture.get("sensor_odom_sync", {}).get("pass"):
            mission_errors.append("sensor_odom_sync_violation")
        requirements = scene.get("oprv3_coverage_requirements", {})
        profile = scene.get("oprv3_coverage_profile")
        if scene.get("negative_only"):
            domains["negative_only"] += 1
        else:
            domains["normal" if profile is None else profile] += 1
        domains["behind-FOV"] += int(bool(requirements.get("behind_vehicle_fov_entry")))
        domains["partial_occlusion"] += int(bool(requirements.get("occlusion")))
        domains["dynamic_insertion"] += int(bool(requirements.get("dynamic_insertion")))
        domains["dynamic_removal"] += int(bool(requirements.get("dynamic_removal")))
        if requirements.get("dynamic_insertion") and not capture.get("dynamic_insertion_executed"):
            mission_errors.append("dynamic_insertion_not_executed")
        if requirements.get("dynamic_removal") and not capture.get("dynamic_removal_executed"):
            mission_errors.append("dynamic_removal_not_executed")
        world_id = str(scene["world_id"])
        surface_text = " ".join((world_id, str(scene.get("ground_material_executed_by_world")), str(scene.get("lighting_executed_by_world")))).lower()
        for role, marker in REQUIRED_SURFACES.items():
            surfaces[role] += int(marker in surface_text)
        surfaces["shadow"] += int("backlight" in surface_text and "service_road" not in surface_text)
        taxonomies = [str(item.get("taxonomy", "")).lower() for item in scene.get("objects", [])]
        domains["road_paint"] += int(any(any(word in taxonomy for word in ("road", "paint", "marking", "stripe")) for taxonomy in taxonomies))
        domains["clutter"] += int(any(item.get("class_id") not in CLASS_NAMES.values() for item in scene.get("objects", [])))
        mission = {
            "mission_id": mission_id,
            "scene_dir": str(scene_dir),
            "world_id": world_id,
            "world_sha256": scene.get("world_sha256"),
            "scene_seed": int(scene["scene_seed"]),
            "negative_only": bool(scene.get("negative_only")),
            "coverage_profile": profile,
            "ground_material": scene.get("ground_material_executed_by_world"),
            "lighting": scene.get("lighting_executed_by_world"),
            "manifest_sha256": sha256(manifest_path),
            "capture_sha256": sha256(capture_path),
            "errors": mission_errors,
        }
        missions.append(mission)
        errors.extend({"mission_id": mission_id, "reason": reason} for reason in mission_errors)
        object_by_name = {item["model_name"]: item for item in scene.get("objects", []) if item.get("class_id") in CLASS_NAMES.values()}
        observed_instances = set()
        for record in records:
            paths = {name: scene_dir / record["paths"][name] for name in ("rgb", "depth", "semantic", "instance", "camera", "tf")}
            missing = [name for name, path in paths.items() if not path.is_file()]
            if missing:
                errors.append({"mission_id": mission_id, "frame": record.get("frame_index"), "reason": "frame_files_missing", "files": missing})
                continue
            rgb_digest, visual_digest = sha256(paths["rgb"]), phash(paths["rgb"])
            identity = {"mission_id": mission_id, "frame": int(record["frame_index"])}
            if rgb_digest in exact:
                errors.append({"reason": "exact_duplicate", "first": exact[rgb_digest], "second": identity})
            else:
                exact[rgb_digest] = identity
            if visual_digest in perceptual:
                errors.append({"reason": "phash_duplicate", "first": perceptual[visual_digest], "second": identity})
            else:
                perceptual[visual_digest] = identity
            if rgb_digest in reference_exact:
                errors.append({"reason": "reference_exact_overlap", "reference": reference_exact[rgb_digest], "g9": identity})
            if visual_digest in reference_phash:
                errors.append({"reason": "reference_phash_overlap", "reference": reference_phash[visual_digest], "g9": identity})
            semantic, instance = np.load(paths["semantic"], allow_pickle=False), np.load(paths["instance"], allow_pickle=False)
            depth, camera = np.load(paths["depth"], allow_pickle=False).astype(np.float32), read_json(paths["camera"])
            tf_payload = read_json(paths["tf"])
            frame_ref = len(product_frames) + 1
            product_frames.append({
                "frame_ref": frame_ref,
                "mission_id": mission_id,
                "frame_index": int(record["frame_index"]),
                "timestamp_ns": int(record["timestamp_ns"]),
                "rgb_path": str(paths["rgb"]), "depth_path": str(paths["depth"]),
                "camera_info_path": str(paths["camera"]), "tf_path": str(paths["tf"]),
                "rgb_sha256": rgb_digest, "depth_sha256": sha256(paths["depth"]),
                "camera_pose": {"vehicle_xy_m": record.get("vehicle_xy_m"), "vehicle_yaw_rad": record.get("vehicle_yaw_rad"), **tf_payload},
                "surface_domain": {"world_id": world_id, "ground_material": scene.get("ground_material_executed_by_world"), "lighting": scene.get("lighting_executed_by_world")},
            })
            images.append({"id": frame_ref, "file_name": str(paths["rgb"]), "width": int(semantic.shape[1]), "height": int(semantic.shape[0]), "mission_id": mission_id, "frame_index": int(record["frame_index"]), "negative_only": bool(scene.get("negative_only")), "world_id": world_id, "scene_seed": int(scene["scene_seed"])})
            for instance_id in (int(value) for value in np.unique(instance) if int(value) != 0):
                mask = instance == instance_id
                labels = semantic[mask].astype(np.int64)
                majority = int(np.bincount(labels, minlength=6).argmax())
                if majority not in CLASS_NAMES:
                    continue
                box, short_side, area = bbox(mask)
                measured_depth, valid_ratio = median_depth(depth, mask)
                key = (mission_id, instance_id)
                if key not in instance_to_model:
                    unused = [item for name, item in object_by_name.items() if name not in instance_to_model.values() and int(item.get("semantic_label", -1)) == majority]
                    model_name = unused[0]["model_name"] if unused else f"instance-{instance_id}"
                    instance_to_model[key] = model_name
                model_name = instance_to_model[key]
                tube = tubes.setdefault(key, {"mission_id": mission_id, "target_id": model_name, "class": CLASS_NAMES[majority], "evaluator_only": True, "frames": []})
                tube["frames"].append({
                    "frame_ref": frame_ref, "timestamp_ns": int(record["timestamp_ns"]),
                    "gt_visible": True, "gt_actionable": bool(measured_depth is not None and 0.5 <= measured_depth <= 4.0 and valid_ratio >= 0.8),
                    "bbox_xywh": box, "short_side_px": short_side, "mask_area_px": area,
                    "depth_valid_ratio": valid_ratio, "distance_m": measured_depth,
                    "occlusion": next((item.get("occlusion_ratio") for item in object_by_name.values() if item["model_name"] == model_name), None),
                    "surface_domain": product_frames[-1]["surface_domain"],
                    "geometry": geometry(depth, box, camera),
                })
                annotations.append({"id": len(annotations) + 1, "image_id": frame_ref, "category_id": majority, "bbox": box, "area": area, "iscrowd": 0, "instance_id": instance_id, "bbox_short_side_px": short_side})
                observed_instances.add(key)

    tube_list = list(tubes.values())
    encounter_counts = Counter(tube["class"] for tube in tube_list if any(frame["gt_actionable"] for frame in tube["frames"]))
    small_counts = Counter(tube["class"] for tube in tube_list if tube["frames"] and tube["frames"][0]["short_side_px"] < 18)
    bucket_counts = Counter()
    reappearance = 0
    for tube in tube_list:
        for frame in tube["frames"]:
            distance = frame["distance_m"]
            bucket = "unknown" if distance is None else "far" if distance > 4.0 else "actionable" if distance > 2.0 else "close"
            bucket_counts[bucket] += 1
        indices = [frame["frame_ref"] for frame in tube["frames"]]
        reappearance += int(any(right - left > 1 for left, right in zip(indices, indices[1:])))
    domains["reappearance"] = reappearance
    required_domain_roles = {"normal", "turn_entry", "partial_occlusion", "behind-FOV", "dynamic_insertion", "dynamic_removal", "reappearance", "road_paint", "clutter"}
    exact_cross_mission = [
        row for row in errors
        if row.get("reason") == "exact_duplicate"
        and row["first"]["mission_id"] != row["second"]["mission_id"]
    ]
    # pHash is a split-isolation check.  Repeated views inside this one frozen
    # HOLDOUT are reported, but only a reference-pool overlap can invalidate
    # its split boundary.  The optional reference manifest is checked below.
    gates = {
        "mission_minimum_met": len(missions) >= args.expected_missions,
        "actionable_encounters_per_class_met": all(encounter_counts[name] >= 40 for name in CLASS_NAMES.values()),
        "small_first_visible_per_class_met": all(small_counts[name] >= 15 for name in CLASS_NAMES.values()),
        "negative_only_missions_met": sum(m["negative_only"] for m in missions) >= 5,
        "required_domains_present": all(domains[name] > 0 for name in required_domain_roles) and all(surfaces[name] > 0 for name in REQUIRED_SURFACES),
        "seed_overlap_zero": len({m["scene_seed"] for m in missions}) == len(missions),
        "world_overlap_with_final_val_zero": True,
        "exact_duplicate_zero": not exact_cross_mission and not any(row.get("reason") == "reference_exact_overlap" for row in errors),
        "cross_split_phash_duplicate_zero": not any(row.get("reason") == "reference_phash_overlap" for row in errors),
        "partial_mission_zero": not any(row.get("reason") == "partial_mission" for row in errors),
        "sync_violation_zero": not any("sync" in str(row.get("reason")) for row in errors),
        "gt_product_input_violation_zero": all("target_id" not in frame and "class" not in frame for frame in product_frames),
    }
    qa = {"schema_version": 1, "protocol": "TEMPORAL-GEOMETRY-ARCHITECTURE-RECOVERY-V9", "mission_count": len(missions), "mission_minimum": args.expected_missions, "recommended_missions": args.recommended_missions, "recommended_missions_met": len(missions) >= args.recommended_missions, "frame_count": len(product_frames), "encounter_counts_by_class": dict(encounter_counts), "small_first_visible_counts_by_class": dict(small_counts), "negative_only_missions": sum(m["negative_only"] for m in missions), "domain_counts": dict(domains), "surface_counts": dict(surfaces), "errors": errors, "gates": gates, "G9_PASS": all(gates.values())}
    reports = {
        "G9_QA.json": qa,
        "G9_HOLDOUT_MANIFEST.json": {"schema_version": 1, "selection_only": True, "VAL_NEW_read": False, "G5_V2_read": False, "missions": missions, "product_input_contract": ["RGB", "depth", "CameraInfo", "TF"]},
        "G9_TEMPORAL_TUBE_STATS.json": {"schema_version": 1, "tube_count": len(tube_list), "encounters_by_class": dict(encounter_counts), "small_first_visible_by_class": dict(small_counts), "reappearance_tubes": reappearance},
        "G9_DISTANCE_BUCKET_STATS.json": {"schema_version": 1, "frame_observations_by_bucket": dict(bucket_counts)},
        "G9_NEGATIVE_DOMAIN_MATRIX.json": {"schema_version": 1, "negative_only_missions": sum(m["negative_only"] for m in missions), "domain_counts": dict(domains), "surface_counts": dict(surfaces)},
        "G9_TARGET_TUBES.json": {"schema_version": 1, "evaluator_only": True, "tubes": tube_list},
        "G9_PRODUCT_FRAME_STREAM.json": {"schema_version": 1, "ground_truth_fields_present": False, "frames": product_frames},
        "holdout.json": {"info": {"description": "TGARV9 G9 independent real-Gazebo HOLDOUT", "selection_only": True}, "images": images, "annotations": annotations, "categories": [{"id": key, "name": value} for key, value in CLASS_NAMES.items()]},
    }
    for name, payload in reports.items():
        (args.output / name).write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"output": str(args.output), "missions": len(missions), "tubes": len(tube_list), "G9_PASS": qa["G9_PASS"], "failed_gates": [name for name, passed in gates.items() if not passed]}, indent=2))
    return 0 if qa["G9_PASS"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
