#!/usr/bin/env python3
"""Freeze a one-time map/map_gt calibration from map-geometry evidence."""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
TASKS = ROOT / "starter_ws" / "src" / "sanitation_tasks"
sys.path.insert(0, str(TASKS))

from sanitation_tasks.localization_metrics import (  # noqa: E402
    TRANSFORM_CONVENTION,
    canonical_payload_sha256,
    compose_se2,
    invert_se2,
)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def map_digest(map_yaml: Path) -> tuple[str, Path]:
    metadata = yaml.safe_load(map_yaml.read_text(encoding="utf-8"))
    map_image = map_yaml.parent / metadata["image"]
    digest = hashlib.sha256()
    digest.update(map_yaml.read_bytes())
    digest.update(b"\0")
    digest.update(map_image.read_bytes())
    return digest.hexdigest(), map_image


def build_calibration(map_yaml: Path, geometry_path: Path, source_map: str) -> dict:
    geometry = json.loads(geometry_path.read_text(encoding="utf-8"))
    alignment = geometry["rigid_alignment"]
    # Geometry fitting produced T_map_world. map_gt is the canonical Gazebo
    # truth frame T_map_gt_world=(+8, 0, 0), independent of every map trial.
    t_map_world = {
        "x_m": float(alignment["x_m"]),
        "y_m": float(alignment["y_m"]),
        "yaw_rad": float(alignment["yaw_rad"]),
    }
    t_map_gt_world = {"x_m": 8.0, "y_m": 0.0, "yaw_rad": 0.0}
    t_map_gt_map = compose_se2(t_map_gt_world, invert_se2(t_map_world))
    map_sha, map_image = map_digest(map_yaml)
    document = {
        "schema_version": 1,
        "map_id": f"sha256:{map_sha}",
        "map_sha256": map_sha,
        "map_yaml_sha256": sha256(map_yaml),
        "map_image_sha256": sha256(map_image),
        "source_map": source_map,
        "source_map_yaml": map_yaml.name,
        "transform_convention": TRANSFORM_CONVENTION,
        "T_map_gt_map": t_map_gt_map,
        "T_map_map_gt": invert_se2(t_map_gt_map),
        "calibration_source": "SDF fixed obstacles at the 2D LiDAR plane; one fit after mapping",
        "fixed_anchors": [box["name"] for box in geometry.get("truth_boxes", [])],
        "fit_residual": {
            "objective_initial_m": alignment.get("initial_objective_m"),
            "objective_final_m": alignment.get("final_objective_m"),
            "boundary_chamfer_mean_m": geometry.get("boundary_chamfer_distance_m"),
            "boundary_p95_m": geometry.get("boundary_p95_m"),
            "occupancy_iou": geometry.get("occupancy_iou"),
        },
        "rules": {
            "fit_once_per_map": True,
            "shared_by_all_seeds": True,
            "trial_trajectory_fitting_forbidden": True,
            "amcl_trajectory_used_for_fit": False,
        },
    }
    document["calibration_payload_sha256"] = canonical_payload_sha256(document)
    return document


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--map-yaml", required=True, type=Path)
    parser.add_argument("--geometry", required=True, type=Path)
    parser.add_argument("--source-map", required=True)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    calibration = build_calibration(args.map_yaml, args.geometry, args.source_map)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        yaml.safe_dump(calibration, allow_unicode=True, sort_keys=False),
        encoding="utf-8",
    )
    # Re-read through the production validator, including payload SHA and inverse.
    from sanitation_tasks.localization_metrics import load_map_calibration

    load_map_calibration(args.output)


if __name__ == "__main__":
    main()
