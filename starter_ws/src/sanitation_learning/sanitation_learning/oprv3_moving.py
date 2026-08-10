"""Pure evaluator helpers for OPRV3 moving-camera evidence."""

from __future__ import annotations

import math
import numpy as np


CLASS_NAMES = (
    "plastic_bottle",
    "metal_can",
    "paper_litter",
    "leaf_pile",
    "puddle",
)


def actionable_window_eligible(
    *, visible_bbox, distance_m: float, scene_visibility_ratio: float,
    depth_valid_ratio: float, frozen_window: dict,
) -> bool:
    """Evaluator-only eligibility; deliberately accepts no model result."""
    return bool(
        visible_bbox is not None
        and frozen_window["minimum_actionable_range_m"] <= distance_m
        <= frozen_window["maximum_actionable_range_m"]
        and scene_visibility_ratio >= frozen_window["minimum_visibility_ratio"]
        and depth_valid_ratio >= frozen_window["minimum_depth_valid_ratio"]
    )


def bbox_from_mask(mask: np.ndarray) -> list[float] | None:
    ys, xs = np.nonzero(mask)
    if not len(xs):
        return None
    return [float(xs.min()), float(ys.min()), float(xs.max() + 1), float(ys.max() + 1)]


def bbox_iou(first: list[float], second: list[float]) -> float:
    left = max(first[0], second[0])
    top = max(first[1], second[1])
    right = min(first[2], second[2])
    bottom = min(first[3], second[3])
    intersection = max(right - left, 0.0) * max(bottom - top, 0.0)
    first_area = max(first[2] - first[0], 0.0) * max(first[3] - first[1], 0.0)
    second_area = max(second[2] - second[0], 0.0) * max(second[3] - second[1], 0.0)
    return intersection / max(first_area + second_area - intersection, 1e-12)


def percentile(values: list[float], q: float) -> float | None:
    return float(np.percentile(values, q)) if values else None


def summarize_encounter(
    target, frames, confirmation_count: int, minimum_visible_frames: int | None = None
) -> dict:
    sampled_actionable = [frame for frame in frames if frame["actionable_window"]]
    required_frames = (
        confirmation_count if minimum_visible_frames is None else minimum_visible_frames
    )
    entered_actionable = len(sampled_actionable) >= required_frames
    actionable = sampled_actionable if entered_actionable else []
    action_hits = [frame for frame in actionable if frame["action_detection"]]
    correct_hits = [
        frame for frame in actionable if frame["correct_action_detection"]
    ]
    consecutive = 0
    confirmation = None
    for frame in actionable:
        consecutive = consecutive + 1 if frame["correct_action_detection"] else 0
        if consecutive >= confirmation_count:
            confirmation = frame
            break
    first = correct_hits[0] if correct_hits else None
    window_start_stamp = actionable[0]["frame_stamp_ns"] if actionable else None
    return {
        "target_id": target["model_name"],
        "class_name": target["class_id"],
        "asset_id": target.get("asset_id"),
        "world_xyz_m": target["xyz_m"],
        "occlusion_bucket": target.get("occlusion_bucket"),
        "visible_fraction_bucket": target.get("visible_fraction_bucket"),
        "ever_in_camera_frustum": any(frame["visible"] for frame in frames),
        "entered_actionable_window": entered_actionable,
        "sampled_actionable_frame_count": len(sampled_actionable),
        "minimum_required_actionable_frames": required_frames,
        "insufficient_sampled_actionable_frames": bool(sampled_actionable) and not entered_actionable,
        "actionable_frame_count": len(actionable),
        "eventual_detection": bool(action_hits),
        "eventual_correct_class": bool(correct_hits),
        "eventual_track_confirmation": confirmation is not None,
        "first_detection_frame": first["frame_index"] if first else None,
        "distance_to_first_detection_m": first["distance_m"] if first else None,
        "time_to_first_detection_s": (
            (first["frame_stamp_ns"] - window_start_stamp) / 1e9
            if first and window_start_stamp is not None
            else None
        ),
        "confirmation_frame": confirmation["frame_index"] if confirmation else None,
        "missed_in_window": bool(actionable) and not bool(correct_hits),
        "frames": frames,
    }


def summarize_route(encounters, false_actions) -> dict:
    eligible = [item for item in encounters if item["entered_actionable_window"]]
    per_class = {}
    for class_name in CLASS_NAMES:
        selected = [item for item in eligible if item["class_name"] == class_name]
        per_class[class_name] = {
            "eligible_targets": len(selected),
            "eventual_detection_recall": sum(item["eventual_detection"] for item in selected) / max(len(selected), 1),
            "eventual_correct_class_recall": sum(item["eventual_correct_class"] for item in selected) / max(len(selected), 1),
            "eventual_track_confirmation_recall": sum(item["eventual_track_confirmation"] for item in selected) / max(len(selected), 1),
            "actionable_window_miss_rate": sum(item["missed_in_window"] for item in selected) / max(len(selected), 1),
        }
    distances = [
        item["distance_to_first_detection_m"] for item in eligible
        if item["distance_to_first_detection_m"] is not None
    ]
    times = [
        item["time_to_first_detection_s"] for item in eligible
        if item["time_to_first_detection_s"] is not None
    ]
    missed = sum(item["missed_in_window"] for item in eligible)
    return {
        "all_gt_targets": len(encounters),
        "never_in_camera_frustum": sum(not item["ever_in_camera_frustum"] for item in encounters),
        "insufficient_sampled_actionable_frames": sum(
            item.get("insufficient_sampled_actionable_frames", False)
            for item in encounters
        ),
        "occluded_entirely": sum(
            not item["ever_in_camera_frustum"]
            and item.get("occlusion_bucket") not in (None, "none")
            for item in encounters
        ),
        "entered_actionable_window": len(eligible),
        "detected_in_window": sum(item["eventual_detection"] for item in eligible),
        "correctly_classified_in_window": sum(item["eventual_correct_class"] for item in eligible),
        "track_confirmed_in_window": sum(item["eventual_track_confirmation"] for item in eligible),
        "missed_in_window": missed,
        "clean_opportunity_missed": missed,
        "eventual_detection_recall": sum(item["eventual_detection"] for item in eligible) / max(len(eligible), 1),
        "eventual_correct_class_recall": sum(item["eventual_correct_class"] for item in eligible) / max(len(eligible), 1),
        "eventual_track_confirmation_recall": sum(item["eventual_track_confirmation"] for item in eligible) / max(len(eligible), 1),
        "actionable_window_miss_rate": missed / max(len(eligible), 1),
        "clean_opportunity_miss_rate": missed / max(len(eligible), 1),
        "median_distance_to_first_detection_m": percentile(distances, 50),
        "p95_time_to_first_detection_s": percentile(times, 95),
        "per_class": per_class,
        **false_actions,
    }


def wrapped_yaw_change(first_rad: float, second_rad: float) -> float:
    return abs((second_rad - first_rad + math.pi) % (2.0 * math.pi) - math.pi)


def empirical_special_coverage(context: dict, routes: dict) -> dict:
    """Require declared scene intent plus captured GT facts; never model output."""
    requirements = {
        seed: scene.get("oprv3_coverage_requirements", {})
        for seed, scene in context["scenes"].items()
    }
    reports = context["capture_reports"]
    representative = next(iter(routes.values()))["encounters"] if routes else []
    turning_seeds = {
        seed for seed, item in requirements.items() if item.get("turning")
    }
    behind_seeds = {
        seed
        for seed, item in requirements.items()
        if item.get("behind_vehicle_fov_entry")
    }
    occlusion_seeds = {
        seed for seed, item in requirements.items() if item.get("occlusion")
    }
    reflection_seeds = {
        seed for seed, item in requirements.items() if item.get("reflection")
    }
    turning = bool(turning_seeds) and all(
        float(reports[seed].get("observed_absolute_yaw_change_rad", 0.0)) >= 1.20
        and {record.get("motion_phase") for record in reports[seed]["records"]}
        >= {"turn_into_target_fov", "straight_approach"}
        for seed in turning_seeds
    )
    behind_entry = False
    for encounter in representative:
        if encounter["scene_seed"] not in behind_seeds:
            continue
        frames = encounter["frames"]
        visible_indices = [index for index, frame in enumerate(frames) if frame["visible"]]
        if not visible_indices or visible_indices[0] == 0:
            continue
        first_visible = visible_indices[0]
        yaw_change = wrapped_yaw_change(
            frames[0]["vehicle_yaw_rad"], frames[first_visible]["vehicle_yaw_rad"]
        )
        if yaw_change >= 0.35 and any(
            not frame["visible"] for frame in frames[:first_visible]
        ):
            behind_entry = True
            break
    occlusion = False
    for encounter in representative:
        if encounter["scene_seed"] not in occlusion_seeds:
            continue
        if encounter.get("occlusion_bucket") in (None, "none"):
            continue
        overlaps = [
            float(frame.get("declared_occluder_bbox_iou", 0.0))
            for frame in encounter["frames"]
            if frame["visible"]
        ]
        if max(overlaps, default=0.0) >= 0.02:
            occlusion = True
            break
    reflection = bool(reflection_seeds) and all(
        reports[seed]["capture_pass"]
        and "wet" in context["scenes"][seed]["world_id"].lower()
        for seed in reflection_seeds
    )
    return {
        "behind_vehicle_fov_entry": behind_entry,
        "turning": turning,
        "occlusion": occlusion,
        "reflection": reflection,
    }


__all__ = [
    "CLASS_NAMES",
    "actionable_window_eligible",
    "bbox_from_mask",
    "bbox_iou",
    "empirical_special_coverage",
    "percentile",
    "summarize_encounter",
    "summarize_route",
    "wrapped_yaw_change",
]
