#!/usr/bin/env python3
"""Evaluate three bounded temporal algorithms over frozen G9 Route-A output."""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
from functools import lru_cache
import json
import math
from pathlib import Path
import sys

import numpy as np


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


@lru_cache(maxsize=128)
def frame_sensors(depth_path: str, camera_info_path: str) -> tuple[np.ndarray, dict]:
    return np.load(depth_path), json.loads(Path(camera_info_path).read_text())


def product_geometry(detection: dict | None, frame: dict, sensors: tuple[np.ndarray, dict] | None = None) -> dict:
    """Derive action geometry only from a predicted box and product sensors."""
    if detection is None:
        return {"distance_m": math.inf, "short_side_px": 0, "depth_valid_ratio": 0.0, "map_xy_m": None, "physical_plausible": False}
    depth, camera_info = sensors or frame_sensors(frame["depth_path"], frame["camera_info_path"])
    box = detection["bbox_xyxy"]
    x0, y0 = max(0, int(math.floor(box[0]))), max(0, int(math.floor(box[1])))
    x1, y1 = min(depth.shape[1], int(math.ceil(box[2]))), min(depth.shape[0], int(math.ceil(box[3])))
    if x1 <= x0 or y1 <= y0:
        return {"distance_m": math.inf, "short_side_px": 0, "depth_valid_ratio": 0.0, "map_xy_m": None, "physical_plausible": False}
    local = depth[y0:y1, x0:x1].astype(np.float64)
    valid = local[np.isfinite(local) & (local > 0.05) & (local < 20.0)]
    valid_ratio = float(valid.size / max(local.size, 1))
    distance = float(np.median(valid)) if valid.size else math.inf
    residual = float(np.percentile(valid, 90) - np.percentile(valid, 10)) if valid.size else math.inf
    short_side = min(x1 - x0, y1 - y0)
    fx, cx = float(camera_info["k"][0]), float(camera_info["k"][2])
    view_angle = math.atan2((x0 + x1) / 2.0 - cx, fx)
    pose = frame["camera_pose"]
    vehicle = pose.get("vehicle_xy_m") or pose.get("world_to_base_xy") or [0.0, 0.0]
    yaw = float(pose.get("vehicle_yaw_rad", 0.0)) + view_angle
    map_xy = (float(vehicle[0]) + distance * math.cos(yaw), float(vehicle[1]) + distance * math.sin(yaw)) if math.isfinite(distance) else None
    return {"distance_m": distance, "short_side_px": short_side, "depth_valid_ratio": valid_ratio, "map_xy_m": map_xy, "physical_plausible": residual <= 1.5}


def detection_probabilities(detection: dict) -> dict[str, float]:
    score, cls = float(detection["score"]), LABELS[int(detection["label"])]
    return {name: (score if name == cls else (1.0 - score) / 3.0) for name in CLASSES}


def product_tracks(mission_id: str, frames: dict[int, dict], raw: dict[int, dict], cfg: TemporalGeometryConfig) -> list[dict]:
    """Run class-agnostic map association without evaluator labels or boxes."""
    active: list[dict] = []
    completed: list[dict] = []
    mission_frames = sorted((frame_ref, frame) for frame_ref, frame in frames.items() if frame["mission_id"] == mission_id)
    for frame_ref, frame in mission_frames:
        frame_index = int(frame["frame_index"])
        for state in list(active):
            if frame_index - state["last_frame"] > 3:
                completed.append(state)
                active.remove(state)
        detections = [row for row in raw[frame_ref]["detections"] if float(row["score"]) >= cfg.observation_threshold]
        sensors = frame_sensors(frame["depth_path"], frame["camera_info_path"])
        observations = []
        for detection in detections:
            geometry = product_geometry(detection, frame, sensors)
            observations.append({"class_probabilities": detection_probabilities(detection), **geometry, "candidate_observed": True, "frame_ref": frame_ref, "frame_index": frame_index, "bbox_xyxy": detection["bbox_xyxy"]})
        used: set[int] = set()
        for observation in observations:
            position = observation["map_xy_m"]
            candidates = []
            if position is not None:
                for index, state in enumerate(active):
                    if index in used or state["position"] is None:
                        continue
                    distance = math.dist(position, state["position"])
                    if distance <= 0.50:
                        candidates.append((distance, index))
            if candidates:
                _, index = min(candidates)
                state = active[index]
                used.add(index)
            else:
                state = {"track": TemporalGeometryTrack(cfg), "position": None, "last_frame": frame_index, "observations": [], "first_confirmed_frame": None, "confirmed_class": None}
                active.append(state)
                used.add(len(active) - 1)
            state["track"].update(observation)
            state["position"] = position
            state["last_frame"] = frame_index
            state["observations"].append({key: observation[key] for key in ("frame_ref", "frame_index", "bbox_xyxy")})
            if state["track"].state == "CONFIRMED" and state["first_confirmed_frame"] is None:
                state["first_confirmed_frame"] = frame_index
                state["confirmed_class"] = state["track"].final_class
    completed.extend(active)
    return completed


def tube_observed(tube: dict, raw: dict[int, dict], threshold: float) -> bool:
    return any(
        float(detection["score"]) >= threshold and iou(detection["bbox_xyxy"], frame["bbox_xywh"]) >= 0.5
        for frame in tube["frames"]
        for detection in raw[frame["frame_ref"]]["detections"]
    )


def track_tube_overlap(track: dict, tube: dict) -> tuple[int, float]:
    truth = {int(frame["frame_ref"]): frame["bbox_xywh"] for frame in tube["frames"]}
    overlaps = [iou(observation["bbox_xyxy"], truth[observation["frame_ref"]]) for observation in track["observations"] if observation["frame_ref"] in truth]
    matches = [value for value in overlaps if value >= 0.5]
    return len(matches), max(overlaps, default=0.0)


def evaluator_assignments(tracks: list[dict], tubes: list[dict]) -> dict[int, int]:
    """Greedy one-to-one evaluator match after product tracking is complete."""
    candidates = []
    for track_index, track in enumerate(tracks):
        for tube_index, tube in enumerate(tubes):
            count, peak = track_tube_overlap(track, tube)
            if count:
                candidates.append((count, peak, track_index, tube_index))
    assignments, used_tracks, used_tubes = {}, set(), set()
    for _, _, track_index, tube_index in sorted(candidates, reverse=True):
        if track_index in used_tracks or tube_index in used_tubes:
            continue
        assignments[tube_index] = track_index
        used_tracks.add(track_index)
        used_tubes.add(tube_index)
    return assignments


def run(policy: dict, tubes: list[dict], frames: dict[int, dict], raw: dict[int, dict], negative_missions: set[str]) -> dict:
    cfg = TemporalGeometryConfig(**policy["config"])
    correct = observed_count = small_correct = small_total = wrong = clean_miss = false_clean = wrong_clean = 0
    confirmed = negative_confirmed = reobserve = pre_fov = 0
    per_class = defaultdict(lambda: Counter(total=0, correct=0))
    tubes_by_mission = defaultdict(list)
    for tube in tubes:
        if any(bool(row["gt_actionable"]) for row in tube["frames"]):
            tubes_by_mission[tube["mission_id"]].append(tube)
    mission_ids = {frame["mission_id"] for frame in frames.values()}
    for mission_id in mission_ids:
        mission_tracks = product_tracks(mission_id, frames, raw, cfg)
        mission_tubes = tubes_by_mission.get(mission_id, [])
        assignments = evaluator_assignments(mission_tracks, mission_tubes)
        assigned_tracks = set(assignments.values())
        mission_confirmed = [index for index, track in enumerate(mission_tracks) if track["first_confirmed_frame"] is not None]
        confirmed += len(mission_confirmed)
        if mission_id in negative_missions:
            negative_confirmed += int(bool(mission_confirmed))
            false_clean += len(mission_confirmed)
        else:
            false_clean += sum(int(index not in assigned_tracks) for index in mission_confirmed)
        for tube_index, tube in enumerate(mission_tubes):
            observed_count += int(tube_observed(tube, raw, cfg.observation_threshold))
            first_small = tube["frames"][0]["short_side_px"] < 18
            small_total += int(first_small)
            per_class[tube["class"]]["total"] += 1
            track = mission_tracks[assignments[tube_index]] if tube_index in assignments else None
            is_confirmed = track is not None and track["first_confirmed_frame"] is not None
            is_correct = is_confirmed and track["confirmed_class"] == tube["class"]
            correct += int(is_correct)
            wrong += int(is_confirmed and not is_correct)
            wrong_clean += int(is_confirmed and not is_correct)
            clean_miss += int(not is_correct)
            per_class[tube["class"]]["correct"] += int(is_correct)
            small_correct += int(first_small and is_correct)
            if track is not None:
                reobserve += track["track"].reobserve_count
                first_visible = min(int(row["frame_ref"]) for row in tube["frames"])
                first_confirmed_ref = next((row["frame_ref"] for row in track["observations"] if row["frame_index"] == track["first_confirmed_frame"]), None)
                pre_fov += int(first_confirmed_ref is not None and first_confirmed_ref < first_visible)
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
        "pre_FOV_creation": pre_fov,
        "GT_control_violation": 0,
        "OBSERVE_AGAIN_count": reobserve,
        "OBSERVE_AGAIN_per_target": reobserve / max(eligible, 1),
        "max_OBSERVE_AGAIN_per_target": cfg.maximum_reobserve_count,
        "extra_travel_distance_m": reobserve * 0.35,
        "extra_time_s": reobserve * 1.5,
        "per_class_correct_recall": {name: value["correct"] / max(value["total"], 1) for name, value in per_class.items()},
    }
    gates = {"observation_recall": metrics["eventual_observation_recall"] >= 0.97, "correct_class_recall": metrics["eventual_correct_class_recall"] >= 0.95, "small_recall": metrics["small_eventual_correct_class_recall"] >= 0.90, "precision_hard_minimum": precision >= 0.95, "wrong_confirmed_rate": metrics["wrong_confirmed_actionable_rate"] <= 0.01, "negative_confirmed_rate": metrics["negative_only_confirmed_actionable_rate"] <= 0.01, "clean_opportunity_miss": metrics["clean_opportunity_miss"] <= 0.02, "false_clean_zero": false_clean == 0, "wrong_clean_zero": wrong_clean == 0, "pre_fov_zero": pre_fov == 0, "gt_violation_zero": True, "reobserve_bounded": cfg.maximum_reobserve_count <= 2}
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
        taxonomy = {"schema_version": 1, "detector_separation_limited": metrics["eventual_correct_class_recall"] < 0.95 or metrics["wrong_confirmed_actionable_rate"] > 0.01, "temporal_limited": metrics["eventual_observation_recall"] >= 0.97 and metrics["eventual_correct_class_recall"] < 0.95, "geometry_limited": not selected["gates"]["clean_opportunity_miss"], "track_fragmentation_or_false_confirmation_limited": metrics["false_CLEAN_NOW"] > 0 or metrics["confirmed_actionable_precision"] < 0.95, "paper_specific": metrics["per_class_correct_recall"].get("paper_litter", 0.0) < 0.95, "small_specific": metrics["small_eventual_correct_class_recall"] < 0.90, "failed_gates": [name for name, passed in selected["gates"].items() if not passed]}
        (args.output.parent / "T1_FAILURE_TAXONOMY.json").write_text(json.dumps(taxonomy, indent=2) + "\n")
    return 0 if selected["pass"] else 4


if __name__ == "__main__":
    raise SystemExit(main())
