#!/usr/bin/env python3
from __future__ import annotations

import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
REVIEW = ROOT / "artifacts" / "stage5br2_20260720_review"


def load(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def main() -> int:
    world_manifest_path = REVIEW / "g2_worlds" / "g2_world_manifest.json"
    smoke_path = REVIEW / "g2_world_smoke" / "g2_world_smoke.json"
    world_manifest = load(world_manifest_path)
    smoke = load(smoke_path)
    distinct_worlds = len({item["sha256"] for item in world_manifest["worlds"]})
    status = {
        "schema_version": 1,
        "stage": "Stage5BR2",
        "metric_semantics_corrected": True,
        "historical_metric_correction": {
            "old_name": "cross_asset_world",
            "correct_name": "cross_asset_same_world",
            "cross_material": None,
            "cross_world": None,
            "reason": "Stage5BR G1 used one world; historical evidence files remain immutable for hash traceability",
        },
        "production_camera_contract": world_manifest["camera_contract"],
        "g2_foundation": {
            "distinct_world_count": distinct_worlds,
            "distinct_material_count": len({item["material_id"] for item in world_manifest["worlds"]}),
            "four_world_sensor_smoke_pass": smoke["four_world_sensor_smoke_pass"],
            "true_physical_asset_scale": all(item["scale_factor"] == 1.0 for item in world_manifest["assets"]),
            "training_only_ground_truth": world_manifest["training_only_ground_truth"],
            "production_launch_modified": world_manifest["production_launch_modified"],
        },
        "resolution_scan": {
            "candidates": [[256, 192], [384, 288], [512, 384], [640, 384]],
            "measured": False,
            "selected": None,
            "required_metrics": ["bbox_short_side_p10_p50_p90", "mask_area", "distance", "visibility", "throughput", "training_vram", "onnx_latency", "model_size", "small_object_recall"],
        },
        "g2_screening_dataset": {
            "required_scenes": 80,
            "required_frames": 800,
            "actual_scenes": 0,
            "actual_frames": 0,
            "annotation_qa_pass": False,
            "status": "not_executed",
        },
        "model_screening": {
            "architecture_attempt_limit": 3,
            "attempts_executed": 0,
            "discrete_detector": None,
            "area_segmenter": None,
            "status": "blocked_by_g2_screening_dataset_gate",
        },
        "formal_500_scene_5000_frame_gate_executed": False,
        "live_30_seed_10_min_gate_executed": False,
        "real_nav2_spot_clean_30_seed_gate_executed": False,
        "j6_gate_executed": False,
        "first_blocking_layer": "G2_screening_dataset_80_scene_800_frame_not_executed",
        "READY_FOR_GPT_REVIEW_STAGE5B": False,
        "READY_FOR_STAGE5C": False,
    }
    status_path = REVIEW / "stage5br2_status.json"
    status_path.write_text(json.dumps(status, indent=2) + "\n", encoding="utf-8")
    inventory = []
    for path in sorted(REVIEW.rglob("*")):
        if path.is_file() and path.name != "artifact_manifest.json":
            inventory.append({
                "path": str(path.relative_to(REVIEW)).replace("\\", "/"),
                "bytes": path.stat().st_size,
                "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            })
    artifact_manifest = {
        "schema_version": 1,
        "artifact": REVIEW.name,
        "files": inventory,
        "READY_FOR_GPT_REVIEW_STAGE5B": False,
        "READY_FOR_STAGE5C": False,
    }
    (REVIEW / "artifact_manifest.json").write_text(json.dumps(artifact_manifest, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(status, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
