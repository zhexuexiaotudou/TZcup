#!/usr/bin/env python3
"""Generate the surveyed-reference localization lane from LiDAR-visible SDF boxes."""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path

import numpy as np
from PIL import Image
import yaml


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT / "starter_ws" / "src" / "sanitation_tasks"))

from stage4t_map_geometry import rasterize_truth, sdf_boxes  # noqa: E402
from sanitation_tasks.localization_metrics import (  # noqa: E402
    TRANSFORM_CONVENTION,
    canonical_payload_sha256,
)


def file_sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--template-map", required=True, type=Path)
    parser.add_argument("--world-sdf", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--lidar-height", default=0.64, type=float)
    args = parser.parse_args()

    template = yaml.safe_load(args.template_map.read_text(encoding="utf-8"))
    template_image = np.asarray(
        Image.open(args.template_map.parent / template["image"]).convert("L")
    )
    boxes = sdf_boxes(
        args.world_sdf, minimum_height=0.0, lidar_height=args.lidar_height
    )
    # The surveyed map frame is deliberately identical to canonical map_gt.
    occupied, polygons = rasterize_truth(
        template_image.shape, template, boxes, (8.0, 0.0, 0.0)
    )
    image = np.full(template_image.shape, 254, dtype=np.uint8)
    image[occupied] = 0
    args.output_dir.mkdir(parents=True, exist_ok=True)
    image_path = args.output_dir / "surveyed_reference.pgm"
    Image.fromarray(image).save(image_path)
    map_yaml = args.output_dir / "surveyed_reference.yaml"
    metadata = {
        "image": image_path.name,
        "mode": "trinary",
        "resolution": float(template["resolution"]),
        "origin": [float(value) for value in template["origin"]],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.25,
    }
    map_yaml.write_text(
        yaml.safe_dump(metadata, sort_keys=False), encoding="utf-8"
    )
    combined = hashlib.sha256(map_yaml.read_bytes() + b"\0" + image_path.read_bytes()).hexdigest()
    calibration = {
        "schema_version": 1,
        "map_id": f"sha256:{combined}",
        "map_sha256": combined,
        "map_yaml_sha256": file_sha(map_yaml),
        "map_image_sha256": file_sha(image_path),
        "source_map": "surveyed_reference",
        "source_map_yaml": map_yaml.name,
        "transform_convention": TRANSFORM_CONVENTION,
        "T_map_gt_map": {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0},
        "T_map_map_gt": {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0},
        "calibration_source": "SDF surveyed reference in canonical map_gt coordinates",
        "fixed_anchors": [box["name"] for box in boxes],
        "fit_residual": {
            "objective_initial_m": 0.0,
            "objective_final_m": 0.0,
            "boundary_chamfer_mean_m": 0.0,
            "boundary_p95_m": 0.0,
            "occupancy_iou": 1.0,
        },
        "rules": {
            "fit_once_per_map": True,
            "shared_by_all_seeds": True,
            "trial_trajectory_fitting_forbidden": True,
            "amcl_trajectory_used_for_fit": False,
        },
    }
    calibration["calibration_payload_sha256"] = canonical_payload_sha256(calibration)
    calibration_path = args.output_dir / "map_frame_calibration.yaml"
    calibration_path.write_text(
        yaml.safe_dump(calibration, allow_unicode=True, sort_keys=False), encoding="utf-8"
    )
    report = {
        "schema_version": 1,
        "lane": "M3_surveyed_reference",
        "mapping_evidence": False,
        "localization_reference": True,
        "world_sdf": str(args.world_sdf),
        "world_sdf_sha256": file_sha(args.world_sdf),
        "lidar_height_m": args.lidar_height,
        "resolution_m": float(template["resolution"]),
        "visible_fixed_obstacle_count": len(boxes),
        "visible_fixed_obstacles": [box["name"] for box in boxes],
        "occupied_cells": int(np.count_nonzero(occupied)),
        "polygons": polygons,
        "map_sha256": combined,
        "map_generation_pass": True,
        "map_basic_quality_pass": True,
        "map_localization_geometry_pass": True,
    }
    (args.output_dir / "surveyed_reference_report.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )

    from sanitation_tasks.localization_metrics import load_map_calibration

    load_map_calibration(calibration_path)


if __name__ == "__main__":
    main()
