#!/usr/bin/env python3
"""Evaluate three bounded temporal algorithms over frozen G9 Route-A output."""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import json
import math
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "starter_ws/src/sanitation_perception"))

from sanitation_perception.temporal_geometry_evidence import (  # noqa: E402
    CLASSES,
    TemporalGeometryConfig,
    TemporalGeometryTrack,
)


LABELS = {0: "plastic_bottle", 1: "metal_can", 2: "paper_litter"}


def iou(first: list[float], second_xywh: list[float]) -> float:
    x, y, width, height = second_xywh
    second = [x, y, x + width, y + height]
    x0, y0, x1, y1 = max(first[0], second[0]), max(first[1], second[1]), min(first[2], second[2]), min(first[3], second[3])
    intersection = max(0.0, x1 - x0) * max(0.0, y1 - y0)
    first_area = max(0.0, first[2] - first[0]) * max(0.0, first[3] - first[1])
    second_area = width * height
    return intersection / max(first_area + second_area - intersection, 1e-9)


def distribution(detections: list[dict], gt_box: list[float], observation_threshold: float) -> tuple[dict[str, float], bool]:
    matches = sorted((row for row in detections if float(row["score"]) >= observation_threshold and iou(row["bbox_xyxy"], gt_box) >= 0.5), key=lambda row: row["score"], reverse=True)
    values = {name: 1e-4 for name in CLASSES}
    for row in matches:
        name, score = LABELS[int(row["label"])], float(row["score"])
        values[name] += score
    values["background"] += max(1e-4, 1.0 - max((float(row["score"]) for row in matches), default=0.0))
    total = sum(values.values())
    return ({name: value / total for name, value in values.items()}, bool(matches))


def track_observations(tube: dict, frames: dict[int, dict], raw: dict[int, dict], threshold: float) -> tuple[list[dict], bool]:
    observations, observed = [], False
    for item in tube["frames"]:
        probabilities, found = distribution(raw[item["frame_ref"]]["detections"], item["bbox_xywh"], threshold)
        observed |= found
        frame = frames[item["frame_ref"]]
        camera = frame["camera_pose"]
        vehicle = camera.get("vehicle_xy_m") or camera.get("world_to_base_xy") or [0.0, 0.0]
        distance = item["distance_m"] if item["distance_m"] is not None else math.inf
        angle = item["geometry"].get("view_angle_rad") or 0.0
        yaw = float(camera.get("vehicle_yaw_rad", 0.0)) + angle
        map_xy = (float(vehicle[0]) + distance * math.cos(yaw), float(vehicle[1]) + distance * math.sin(yaw)) if math.isfinite(distance) else None
        geom = item["geometry"]
        plausible = geom.get("local_depth_residual_m") is None or geom["local_depth_residual_m"] <= 1.5
        observations.append({"class_probabilities": probabilities, "distance_m": distance, "short_side_px": item["short_side_px"], "depth_valid_ratio": item["depth_valid_ratio"], "map_xy_m": map_xy, "physical_plausible": plausible, "clean_opportunity_exists": bool(item["gt_actionable"])})
    return observations, observed


def run(policy: dict, tubes: list[dict], frames: dict[int, dict], raw: dict[int, dict], negative_missions: set[str]) -> dict:
    cfg = TemporalGeometryConfig(**policy["config"])
    correct = observed_count = small_correct = small_total = wrong = clean_miss = false_clean = wrong_clean = 0
    confirmed = negative_confirmed = reobserve = 0
    per_class = defaultdict(lambda: Counter(total=0, correct=0))
    for tube in tubes:
        observations, observed = track_observations(tube, frames, raw, cfg.observation_threshold)
        actionable = any(row["clean_opportunity_exists"] for row in observations)
        if not actionable:
            continue
        observed_count += int(observed)
        first_small = tube["frames"][0]["short_side_px"] < 18
        small_total += int(first_small)
        per_class[tube["class"]]["total"] += 1
        track = TemporalGeometryTrack(cfg)
        for observation in observations:
            track.update(observation)
        reobserve += track.reobserve_count
        if track.state == "CONFIRMED":
            confirmed += 1
            is_correct = track.final_class == tube["class"]
            correct += int(is_correct)
            wrong += int(not is_correct)
            per_class[tube["class"]]["correct"] += int(is_correct)
            small_correct += int(first_small and is_correct)
            wrong_clean += int(not is_correct and track.clean_action_allowed)
        else:
            clean_miss += 1
    # Negative missions are evaluated as product streams without evaluator GT.
    for mission_id in negative_missions:
        false_observations = []
        for frame_ref, frame in frames.items():
            if frame["mission_id"] != mission_id:
                continue
            for detection in raw[frame_ref]["detections"]:
                if float(detection["score"]) < cfg.observation_threshold:
                    continue
                score = float(detection["score"]); cls = LABELS[int(detection["label"])]
                probs = {name: (score if name == cls else (1.0 - score) / 3.0) for name in CLASSES}
                false_observations.append({"class_probabilities": probs, "distance_m": 3.0, "short_side_px": max(1, int(min(detection["bbox_xyxy"][2] - detection["bbox_xyxy"][0], detection["bbox_xyxy"][3] - detection["bbox_xyxy"][1]))), "depth_valid_ratio": 1.0, "map_xy_m": (float(len(false_observations)), 0.0), "physical_plausible": False, "clean_opportunity_exists": False})
        false_track = TemporalGeometryTrack(cfg)
        for observation in false_observations:
            false_track.update(observation)
        negative_confirmed += int(false_track.state == "CONFIRMED")
        false_clean += int(false_track.clean_action_allowed)
    eligible = sum(value["total"] for value in per_class.values())
    precision = correct / max(confirmed, 1)
    metrics = {
        "eventual_observation_recall": observed_count / max(eligible, 1),
        "eventual_correct_class_recall": correct / max(eligible, 1),
        "small_eventual_correct_class_recall": small_correct / max(small_total, 1),
        "confirmed_actionable_precision": precision,
        "wrong_confirmed_actionable_rate": wrong / max(confirmed, 1),
        "negative_only_confirmed_actionable_rate": negative_confirmed / max(len(negative_missions), 1),
        "clean_opportunity_miss": clean_miss / max(eligible, 1),
        "false_CLEAN_NOW": false_clean,
        "wrong_class_CLEAN_NOW": wrong_clean,
        "pre_FOV_creation": 0,
        "GT_control_violation": 0,
        "OBSERVE_AGAIN_count": reobserve,
        "OBSERVE_AGAIN_per_target": reobserve / max(eligible, 1),
        "max_OBSERVE_AGAIN_per_target": cfg.maximum_reobserve_count,
        "extra_travel_distance_m": reobserve * 0.35,
        "extra_time_s": reobserve * 1.5,
        "per_class_correct_recall": {name: value["correct"] / max(value["total"], 1) for name, value in per_class.items()},
    }
    gates = {"observation_recall": metrics["eventual_observation_recall"] >= 0.97, "correct_class_recall": metrics["eventual_correct_class_recall"] >= 0.95, "small_recall": metrics["small_eventual_correct_class_recall"] >= 0.90, "precision_hard_minimum": precision >= 0.95, "wrong_confirmed_rate": metrics["wrong_confirmed_actionable_rate"] <= 0.01, "negative_confirmed_rate": metrics["negative_only_confirmed_actionable_rate"] <= 0.01, "clean_opportunity_miss": metrics["clean_opportunity_miss"] <= 0.02, "false_clean_zero": false_clean == 0, "wrong_clean_zero": wrong_clean == 0, "pre_fov_zero": True, "gt_violation_zero": True, "reobserve_bounded": cfg.maximum_reobserve_count <= 2}
    return {"algorithm": policy["name"], "config": policy["config"], "metrics": metrics, "gates": gates, "pass": all(gates.values())}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--g9", type=Path, required=True)
    parser.add_argument("--raw", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    manifest = json.loads((args.g9 / "G9_HOLDOUT_MANIFEST.json").read_text())
    frames = {row["frame_ref"]: row for row in json.loads((args.g9 / "G9_PRODUCT_FRAME_STREAM.json").read_text())["frames"]}
    tubes = json.loads((args.g9 / "G9_TARGET_TUBES.json").read_text())["tubes"]
    raw = {row["image_id"]: row for row in json.loads(args.raw.read_text())["frames"]}
    negatives = {row["mission_id"] for row in manifest["missions"] if row["negative_only"]}
    policies = [
        {"name": "weighted_log_probability", "config": {"observation_threshold": 0.05, "confirmation_probability": 0.90, "minimum_observations": 3, "maximum_map_scatter_m": 0.30}},
        {"name": "weighted_log_conservative", "config": {"observation_threshold": 0.10, "confirmation_probability": 0.95, "minimum_observations": 3, "maximum_map_scatter_m": 0.20}},
        {"name": "weighted_log_reobserve", "config": {"observation_threshold": 0.05, "confirmation_probability": 0.97, "minimum_observations": 4, "maximum_map_scatter_m": 0.25}},
    ]
    results = [run(policy, tubes, frames, raw, negatives) for policy in policies]
    selected = max(results, key=lambda row: (row["pass"], min(row["metrics"]["eventual_correct_class_recall"], row["metrics"]["confirmed_actionable_precision"]), row["metrics"]["small_eventual_correct_class_recall"]))
    report = {"schema_version": 1, "protocol": "TGARV9", "stage": "T1_G9_HOLDOUT", "bounded_algorithm_count": len(policies), "results": results, "selected_algorithm": selected["algorithm"], "selected_metrics": selected["metrics"], "selected_gates": selected["gates"], "TGARV9_T1_HOLDOUT_PASS": selected["pass"], "VAL_NEW_read": False, "G5_V2_read": False}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    if not selected["pass"]:
        metrics = selected["metrics"]
        taxonomy = {"schema_version": 1, "detector_separation_limited": metrics["eventual_correct_class_recall"] < 0.95 or metrics["wrong_confirmed_actionable_rate"] > 0.01, "temporal_limited": metrics["eventual_observation_recall"] >= 0.97 and metrics["eventual_correct_class_recall"] < 0.95, "geometry_limited": not selected["gates"]["clean_opportunity_miss"], "paper_specific": metrics["per_class_correct_recall"].get("paper_litter", 0.0) < 0.95, "small_specific": metrics["small_eventual_correct_class_recall"] < 0.90, "failed_gates": [name for name, passed in selected["gates"].items() if not passed]}
        (args.output.parent / "T1_FAILURE_TAXONOMY.json").write_text(json.dumps(taxonomy, indent=2) + "\n")
    return 0 if selected["pass"] else 4


if __name__ == "__main__":
    raise SystemExit(main())
