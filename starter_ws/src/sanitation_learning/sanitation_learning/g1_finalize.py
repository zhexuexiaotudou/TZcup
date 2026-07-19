from __future__ import annotations

import hashlib
import json
from pathlib import Path

import cv2
import numpy as np

from .rendered import CLASS_ORDER


def _phash(path: Path) -> str:
    image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    reduced = cv2.resize(image, (32, 32), interpolation=cv2.INTER_AREA).astype(np.float32)
    frequency = cv2.dct(reduced)[:8, :8]
    bits = frequency > np.median(frequency[1:])
    return f"{int(''.join('1' if value else '0' for value in bits.ravel()), 2):016x}"


def finalize(root_path: str | Path, expected_scenes: int, expected_frames: int) -> dict:
    root = Path(root_path)
    scenes = []
    records = []
    errors = []
    pixel_counts = {name: 0 for name in CLASS_ORDER}
    instance_areas = {name: [] for name in CLASS_ORDER[1:]}
    asset_area_observations = {}
    seen_hashes = {}
    seen_phashes = {}
    cross_split_exact = []
    cross_split_perceptual = []
    label_consistency_errors = 0
    label_consistency_samples = 0
    for scene_path in sorted((root / "scene_manifests").glob("scene_*.json")):
        scene = json.loads(scene_path.read_text(encoding="utf-8"))
        capture_path = root / "scenes" / f"scene_{scene['scene_seed']:04d}" / "capture_report.json"
        if not capture_path.is_file():
            errors.append({"scene_seed": scene["scene_seed"], "reason": "capture_report_missing"})
            continue
        capture = json.loads(capture_path.read_text(encoding="utf-8"))
        if not capture["capture_pass"]:
            errors.append({"scene_seed": scene["scene_seed"], "reason": "capture_failed"})
        class_asset = {
            item["semantic_label"]: item["model_name"] for item in scene["objects"]
            if item["semantic_label"] != 0
        }
        for record in capture["records"]:
            base = capture_path.parent
            image_path = base / record["paths"]["image"]
            depth = np.load(base / record["paths"]["depth"], allow_pickle=False)
            semantic = np.load(base / record["paths"]["semantic"], allow_pickle=False)
            instances = np.load(base / record["paths"]["instances"], allow_pickle=False)
            if semantic.shape != (96, 128) or depth.shape != semantic.shape or instances.shape != semantic.shape:
                errors.append({"scene_seed": scene["scene_seed"], "frame": record["frame_index"], "reason": "shape_mismatch"})
            labels = set(int(value) for value in np.unique(semantic))
            if not labels.issubset(set(range(len(CLASS_ORDER)))):
                errors.append({"scene_seed": scene["scene_seed"], "frame": record["frame_index"], "reason": "unknown_semantic_label", "labels": sorted(labels)})
            for index, class_id in enumerate(CLASS_ORDER):
                area = int((semantic == index).sum())
                pixel_counts[class_id] += area
                if index:
                    instance_areas[class_id].append(area)
                    asset_area_observations.setdefault(class_asset[index], []).append(area)
            for instance_id in np.unique(instances):
                if instance_id == 0:
                    continue
                values = semantic[instances == instance_id]
                label_consistency_samples += int(values.size)
                if values.size:
                    majority = int(np.bincount(values.astype(np.int64), minlength=len(CLASS_ORDER)).argmax())
                    label_consistency_errors += int((values != majority).sum())
            rgb_hash = record["rgb_sha256"]
            phash = _phash(image_path)
            prior = seen_hashes.get(rgb_hash)
            if prior and prior["split"] != scene["split"]:
                cross_split_exact.append([prior, {"scene_seed": scene["scene_seed"], "split": scene["split"]}])
            else:
                seen_hashes[rgb_hash] = {"scene_seed": scene["scene_seed"], "split": scene["split"]}
            prior_phash = seen_phashes.get(phash)
            if prior_phash and prior_phash["split"] != scene["split"]:
                cross_split_perceptual.append([prior_phash, {"scene_seed": scene["scene_seed"], "split": scene["split"]}])
            else:
                seen_phashes[phash] = {"scene_seed": scene["scene_seed"], "split": scene["split"]}
            records.append({
                **record, "split": scene["split"], "asset_ids": scene["asset_ids"],
                "texture_ids": scene["texture_ids"], "negative_ids": scene["negative_ids"],
                "world_sha256": scene["world_sha256"], "scene_manifest_sha256": scene["scene_manifest_sha256"],
                "perceptual_hash": phash,
            })
        scenes.append(scene)
    asset_max = {asset: max(areas) for asset, areas in asset_area_observations.items() if areas}
    occlusion_ratios = []
    for asset, areas in asset_area_observations.items():
        maximum = max(asset_max.get(asset, 1), 1)
        occlusion_ratios.extend(max(0.0, 1.0 - area / maximum) for area in areas)
    split_scenes = {name: sorted(scene["scene_seed"] for scene in scenes if scene["split"] == name) for name in ("train", "val", "test")}
    split_assets = {name: sorted({asset for scene in scenes if scene["split"] == name for asset in scene["asset_ids"]}) for name in split_scenes}
    split_textures = {name: sorted({texture for scene in scenes if scene["split"] == name for texture in scene["texture_ids"]}) for name in split_scenes}
    asset_leakage = sorted((set(split_assets["train"]) & set(split_assets["val"])) | (set(split_assets["train"]) & set(split_assets["test"])) | (set(split_assets["val"]) & set(split_assets["test"])))
    semantic_error_rate = label_consistency_errors / max(label_consistency_samples, 1)
    manifest = {
        "schema_version": 1, "dataset_id": "stage5br_g1_gazebo_camera_smoke_v1",
        "domain": "G1_actual_gazebo_camera_rendered_synthetic",
        "gazebo_camera_rendered": True, "scene_count": len(scenes), "frame_count": len(records),
        "frames_per_scene": sorted({scene.get("requested_frame_count", 10) for scene in []}) or [10],
        "class_order": list(CLASS_ORDER), "split_scene_seeds": split_scenes,
        "split_assets": split_assets, "split_textures": split_textures,
        "scene_or_trajectory_cross_split": False, "asset_leakage": asset_leakage,
        "cross_split_exact_duplicate_count": len(cross_split_exact),
        "cross_split_perceptual_duplicate_count": len(cross_split_perceptual),
        "records": records,
    }
    qa = {
        "schema_version": 1, "phase": "Stage5BR G1 smoke QA",
        "expected_scene_count": expected_scenes, "actual_scene_count": len(scenes),
        "expected_frame_count": expected_frames, "actual_frame_count": len(records),
        "annotation_completeness": len(records) / max(expected_frames, 1),
        "rgb_depth_semantic_instance_exact_timestamp_match": all(record["exact_sensor_timestamp_match"] for record in records),
        "tf_manifest_present": all((root / "scenes" / f"scene_{record['scene_seed']:04d}" / record["paths"]["tf"]).is_file() for record in records),
        "semantic_instance_consistency_error_rate": semantic_error_rate,
        "sampled_label_error_rate": semantic_error_rate,
        "sampled_label_error_semantics": "instance pixels whose semantic labels disagree with that instance majority",
        "class_pixel_distribution": pixel_counts,
        "instance_size_pixels": {
            name: {"min": min(values) if values else None, "median": float(np.median(values)) if values else None, "max": max(values) if values else None}
            for name, values in instance_areas.items()
        },
        "occlusion_estimate": {
            "method": "one minus visible pixel area divided by per-asset observed maximum at fixed camera",
            "count": len(occlusion_ratios), "mean": float(np.mean(occlusion_ratios)) if occlusion_ratios else None,
            "p95": float(np.percentile(occlusion_ratios, 95)) if occlusion_ratios else None,
        },
        "negative_asset_distribution": {
            negative: sum(negative in scene["negative_ids"] for scene in scenes)
            for negative in sorted({negative for scene in scenes for negative in scene["negative_ids"]})
        },
        "lighting_randomized": all(scene["lighting"].get("randomized") for scene in scenes),
        "ground_material_distribution": sorted({scene["ground_material"] for scene in scenes}),
        "asset_leakage": asset_leakage,
        "cross_split_exact_duplicate_count": len(cross_split_exact),
        "cross_split_perceptual_duplicate_count": len(cross_split_perceptual),
        "errors": errors,
    }
    qa["annotation_qa_pass"] = all([
        len(scenes) == expected_scenes, len(records) == expected_frames,
        qa["annotation_completeness"] == 1.0,
        qa["rgb_depth_semantic_instance_exact_timestamp_match"], qa["tf_manifest_present"],
        semantic_error_rate <= 0.01, not asset_leakage,
        len(cross_split_exact) == 0, len(cross_split_perceptual) == 0,
        all(pixel_counts[name] > 0 for name in CLASS_ORDER[1:]),
        qa["lighting_randomized"], not errors,
    ])
    (root / "g1_dataset_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    (root / "g1_annotation_qa.json").write_text(json.dumps(qa, indent=2) + "\n", encoding="utf-8")
    summary = {
        "schema_version": 1, "stage": "Stage5BR G1 smoke",
        "dataset_scale_gate": len(scenes) >= 50 and len(records) >= 500,
        "annotation_qa_pass": qa["annotation_qa_pass"],
        "G1_smoke_pass": len(scenes) >= 50 and len(records) >= 500 and qa["annotation_qa_pass"],
        "formal_G1_500_scene_gate_executed": False,
        "READY_FOR_GPT_REVIEW_STAGE5B": False, "READY_FOR_STAGE5C": False,
    }
    (root / "g1_smoke_summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    return summary


def main() -> None:
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", required=True)
    parser.add_argument("--expected-scenes", type=int, required=True)
    parser.add_argument("--expected-frames", type=int, required=True)
    args = parser.parse_args()
    report = finalize(args.root, args.expected_scenes, args.expected_frames)
    print(json.dumps(report, indent=2))
    if not report["G1_smoke_pass"]:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
