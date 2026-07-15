#!/usr/bin/env python3
"""Assemble the Stage4R truth summary and complete SHA256 manifest."""

import argparse
import datetime as dt
import hashlib
import json
from pathlib import Path


def load(root, name):
    return json.loads((root / name).read_text(encoding="utf-8"))


def main():
    parser = argparse.ArgumentParser(); parser.add_argument("--artifact-dir", required=True)
    args = parser.parse_args(); root = Path(args.artifact_dir)
    localization = load(root, "localization_report.json")
    slam = load(root, "slam_quality_report.json")
    coverage = load(root, "coverage_report.json")
    safety = load(root, "safety_latency_report.json")
    filters = load(root, "filter_report.json")
    dynamic = load(root, "dynamic_obstacle_report.json")
    efficiency = load(root, "efficiency_scan.json")
    bag_metadata = root / "full_mission_bag" / "metadata.yaml"
    replay_ok = (root / "replay_ground_truth.txt").stat().st_size > 0
    image_names = ["gazebo_initial.png", "gazebo_mapping.png", "gazebo_coverage_attempt.png", "gazebo_dynamic_obstacle.png", "gazebo_final.png"]
    images_ok = all((root / name).is_file() and (root / name).stat().st_size > 0 for name in image_names)
    complete_bag = bag_metadata.is_file() and (root / "full_mission_bag" / "full_mission_bag_0.mcap").is_file() and replay_ok
    readiness = {
        "same_frame_localization": localization["competition_localization_pass"],
        "valid_slam_numeric_quality": slam["slam_quality_pass"],
        "full_coverage_execution": coverage["full_execution_success"],
        "empirical_coverage_at_least_90pct": coverage["empirical_metrics"]["coverage_rate"] >= 0.90,
        "no_collision": coverage["collision_count"] == 0,
        "estop_p95_at_most_1s": safety["competition_estop_pass"],
        "complete_rosbag_and_replay": complete_bag,
        "real_gazebo_images": images_ok,
    }
    summary = {
        "schema_version": 1,
        "generated_at_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
        "stage": "Stage4R",
        "ready_for_gpt_review_stage4r": all(readiness.values()),
        "ready_for_stage5a": all(readiness.values()),
        "old_stage3_localization_metric_valid": False,
        "old_metric_note": "The 1.805570 m Stage3 value directly subtracted map-frame AMCL and odom-frame wheel odometry and is invalid as localization error.",
        "readiness_gates": readiness,
        "slam": slam,
        "mapping_route_completion": load(root, "mapping_completion_probe.json"),
        "localization": localization,
        "coverage": coverage,
        "efficiency": efficiency,
        "filters": filters,
        "dynamic_obstacles": dynamic,
        "emergency_stop": safety,
        "complete_bag": {"metadata": "full_mission_bag/metadata.yaml", "replay_verified": replay_ok},
        "gazebo_render_images": image_names,
        "stop_boundary": "Stage4R failed localization/full-coverage gates; Stage5A and J6 work were not started.",
    }
    (root / "stage4r_summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    entries = []
    for path in sorted(item for item in root.rglob("*") if item.is_file() and item.name != "MANIFEST.json"):
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        entries.append({"path": path.relative_to(root).as_posix(), "size_bytes": path.stat().st_size, "sha256": digest})
    manifest = {"schema_version": 1, "generated_at_utc": dt.datetime.now(dt.timezone.utc).isoformat(), "root": root.name, "self_excluded": "MANIFEST.json is excluded because a file cannot contain its own stable hash", "file_count": len(entries), "files": entries}
    (root / "MANIFEST.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__": main()
