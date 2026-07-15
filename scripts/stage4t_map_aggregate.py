#!/usr/bin/env python3
"""Select the measured 0.05/0.02 m map; retain both reports."""

import argparse
import json
import math
import shutil
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(); parser.add_argument("output_dir", type=Path)
    args = parser.parse_args(); candidates = {}
    for label in ("005", "002"):
        root = args.output_dir / f"map_{label}"
        if not (root / "map_geometry.json").is_file():
            continue
        geometry = json.loads((root / "map_geometry.json").read_text(encoding="utf-8"))
        mapping = json.loads((root / "mapping_probe.json").read_text(encoding="utf-8"))
        quality = json.loads((root / "map_quality.json").read_text(encoding="utf-8"))
        candidates[label] = {"mapping": mapping, "quality": quality, "geometry": geometry}
    eligible = [label for label, item in candidates.items() if item["mapping"].get("success") and item["quality"].get("slam_quality_pass")]
    selected = max(eligible, key=lambda label: (candidates[label]["geometry"].get("occupancy_iou", 0.0), -float(candidates[label]["geometry"].get("boundary_rmse_m") or 1.0e9))) if eligible else None
    report = {
        "schema_version": 1,
        "comparison_resolutions_m": [candidates[label]["geometry"]["map_resolution_m"] for label in candidates],
        "rigid_registration_completed_before_metrics": True,
        "candidates": candidates,
        "selected_map": selected,
        "pass": selected is not None,
    }
    (args.output_dir / "map_geometry_report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    if selected:
        root = args.output_dir / f"map_{selected}"
        shutil.copyfile(root / "slam_map.yaml", args.output_dir / "selected_map.yaml")
        shutil.copyfile(root / "slam_map.pgm", args.output_dir / "selected_map.pgm")
        shutil.copyfile(root / "map_truth_overlay.png", args.output_dir / "map_truth_overlay.png")
        text = (args.output_dir / "selected_map.yaml").read_text(encoding="utf-8").replace("slam_map.pgm", "selected_map.pgm")
        (args.output_dir / "selected_map.yaml").write_text(text, encoding="utf-8")
        alignment = candidates[selected]["geometry"]["rigid_alignment"]
        yaw = float(alignment["yaw_rad"])
        initial_x = float(alignment["x_m"]) - 8.0 * math.cos(yaw)
        initial_y = float(alignment["y_m"]) - 8.0 * math.sin(yaw)
        (args.output_dir / "selected_map_alignment.env").write_text(
            f"WORLD_TO_MAP_X={alignment['x_m']:.12g}\n"
            f"WORLD_TO_MAP_Y={alignment['y_m']:.12g}\n"
            f"WORLD_TO_MAP_YAW={alignment['yaw_rad']:.12g}\n"
            f"INITIAL_POSE_X={initial_x:.12g}\n"
            f"INITIAL_POSE_Y={initial_y:.12g}\n"
            f"INITIAL_POSE_YAW={yaw:.12g}\n",
            encoding="utf-8",
        )


if __name__ == "__main__": main()
