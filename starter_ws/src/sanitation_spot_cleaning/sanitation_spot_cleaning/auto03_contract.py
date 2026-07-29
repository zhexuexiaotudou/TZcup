from __future__ import annotations

from collections import Counter, defaultdict
import math
from statistics import median
from typing import Iterable


ORACLE_FIELDS = {
    "candidate_id",
    "x_m",
    "y_m",
    "target_size_m",
    "class_id",
    "covariance_trace",
    "timestamp_s",
}
DISCRETE_CLASSES = ("plastic_bottle", "metal_can", "paper_litter")


def validate_oracle_candidate(payload: dict) -> dict:
    """Validate the only payload that may cross the GT-to-runtime boundary."""
    unexpected = sorted(set(payload) - ORACLE_FIELDS)
    missing = sorted(ORACLE_FIELDS - set(payload))
    if unexpected or missing:
        raise ValueError(
            f"oracle boundary mismatch: missing={missing}, unexpected={unexpected}"
        )
    candidate = {
        "candidate_id": str(payload["candidate_id"]),
        "x_m": float(payload["x_m"]),
        "y_m": float(payload["y_m"]),
        "target_size_m": float(payload["target_size_m"]),
        "class_id": str(payload["class_id"]),
        "covariance_trace": float(payload["covariance_trace"]),
        "timestamp_s": float(payload["timestamp_s"]),
    }
    numeric = (
        candidate["x_m"],
        candidate["y_m"],
        candidate["target_size_m"],
        candidate["covariance_trace"],
        candidate["timestamp_s"],
    )
    if not all(math.isfinite(value) for value in numeric):
        raise ValueError("oracle candidate contains non-finite numeric data")
    if candidate["target_size_m"] <= 0.0:
        raise ValueError("target_size_m must be positive")
    if candidate["covariance_trace"] < 0.0:
        raise ValueError("covariance_trace must be non-negative")
    return candidate


def percentile(values: Iterable[float], probability: float) -> float | None:
    ordered = sorted(float(value) for value in values)
    if not ordered:
        return None
    position = (len(ordered) - 1) * probability
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def projection_measurement(
    predicted_roi_xyxy: list[float] | tuple[float, float, float, float],
    actual_bbox_xywh: list[int] | tuple[int, int, int, int],
    *,
    predicted_target_short_side_px: float | None = None,
) -> dict:
    px0, py0, px1, py1 = (float(value) for value in predicted_roi_xyxy)
    ax, ay, aw, ah = (float(value) for value in actual_bbox_xywh)
    predicted_center = ((px0 + px1) / 2.0, (py0 + py1) / 2.0)
    actual_center = (ax + aw / 2.0, ay + ah / 2.0)
    predicted_short = (
        min(px1 - px0, py1 - py0)
        if predicted_target_short_side_px is None
        else float(predicted_target_short_side_px)
    )
    actual_short = min(aw, ah)
    return {
        "actual_target_center_inside_predicted_roi": (
            px0 <= actual_center[0] <= px1 and py0 <= actual_center[1] <= py1
        ),
        "center_pixel_error": math.dist(predicted_center, actual_center),
        "predicted_short_side_px": predicted_short,
        "actual_short_side_px": actual_short,
        "short_side_relative_error": (
            abs(predicted_short - actual_short) / max(actual_short, 1.0)
        ),
        "search_roi_short_side_px": min(px1 - px0, py1 - py0),
    }


def summarize_auto03(
    truth_trials: list[dict],
    runtime_trials: list[dict],
    *,
    baseline_coverage_duration_s: float = 333.36516484299864,
    configured_navigation_speed_m_s: float = 0.45,
) -> dict:
    truth_by_id = {str(item["candidate_id"]): item for item in truth_trials}
    runtime_by_id = {str(item["candidate_id"]): item for item in runtime_trials}
    rows = []
    for candidate_id, truth in truth_by_id.items():
        row = dict(truth)
        row.update(runtime_by_id.get(candidate_id, {}))
        row["candidate_id"] = candidate_id
        rows.append(row)

    target_rows = [row for row in rows if row["case_type"] in {"reachable", "unreachable_keepout"}]
    reachable = [row for row in rows if row["case_type"] == "reachable"]
    attempted_preflight = [
        row for row in rows
        if row["case_type"] in {"reachable", "unreachable_keepout", "false_candidate"}
    ]
    navigated = [row for row in rows if row.get("navigation_attempted")]
    false_rows = [row for row in rows if row["case_type"] == "false_candidate"]
    stale_rows = [row for row in rows if row["case_type"] == "stale_dropout"]
    unreachable_rows = [row for row in rows if row["case_type"] == "unreachable_keepout"]
    captured_targets = [row for row in reachable if row.get("capture_completed")]
    ready_targets = [row for row in reachable if row.get("actual_ready")]
    projection_rows = [
        row for row in captured_targets
        if row.get("projection") and row["projection"].get("center_pixel_error") is not None
    ]

    def rate(group: list[dict], predicate) -> float:
        return sum(bool(predicate(row)) for row in group) / max(len(group), 1)

    class_counts = Counter(row["class_id"] for row in target_rows)
    per_class_conversion = {
        class_id: rate(
            [row for row in reachable if row["class_id"] == class_id],
            lambda row: row.get("actual_ready"),
        )
        for class_id in sorted({row["class_id"] for row in reachable})
    }
    distances = [row["extra_distance_m"] for row in ready_targets if row.get("extra_distance_m") is not None]
    times = [row["extra_time_s"] for row in ready_targets if row.get("extra_time_s") is not None]
    interruptions = [row["coverage_interruption_s"] for row in reachable if row.get("coverage_interruption_s") is not None]
    center_errors = [row["projection"]["center_pixel_error"] for row in projection_rows]
    short_errors = [row["projection"]["short_side_relative_error"] for row in projection_rows]
    self_pixels = [row["self_pixel_fraction"] for row in captured_targets if row.get("self_pixel_fraction") is not None]
    target_self = [row["target_self_overlap"] for row in captured_targets if row.get("target_self_overlap") is not None]
    prediction_agreement = rate(
        captured_targets,
        lambda row: bool(row.get("predicted_ready")) == bool(row.get("actual_ready")),
    )

    data_matrix = {
        "world_count": len({row["world_id"] for row in rows}),
        "scene_count": len({(row["world_id"], row["scene_id"]) for row in rows}),
        "valid_target_candidate_count": len(target_rows),
        "target_count_by_class": dict(sorted(class_counts.items())),
        "unreachable_keepout_count": len(unreachable_rows),
        "false_no_target_count": len(false_rows),
        "stale_dropout_count": len(stale_rows),
    }
    candidates_per_scene = len(rows) / max(data_matrix["scene_count"], 1)
    median_distance = median(distances) if distances else None
    median_time = median(times) if times else None
    measured_added_time = (
        median_time * candidates_per_scene if median_time is not None else None
    )
    theoretical_added_time = (
        median_distance / configured_navigation_speed_m_s * candidates_per_scene
        if median_distance is not None else None
    )
    metrics = {
        "safe_coverage_boundary_pause_rate": rate(rows, lambda row: row.get("coverage_boundary_pause_safe")),
        "preflight_path_success_rate": rate(
            [row for row in attempted_preflight if row["case_type"] != "unreachable_keepout"],
            lambda row: row.get("preflight_path_success"),
        ),
        "navigate_to_pose_success_rate_reachable": rate(
            [row for row in navigated if row["case_type"] in {"reachable", "false_candidate"}],
            lambda row: row.get("navigate_success"),
        ),
        "ready_conversion_overall": rate(reachable, lambda row: row.get("actual_ready")),
        "ready_conversion_by_class": per_class_conversion,
        "coverage_resume_rate": rate(
            [row for row in reachable + false_rows if row.get("navigation_attempted")],
            lambda row: row.get("coverage_resumed"),
        ),
        "unreachable_fail_closed_rate": rate(
            unreachable_rows,
            lambda row: row.get("terminal_state") == "UNREACHABLE" and not row.get("navigation_attempted"),
        ),
        "false_candidate_without_cleaning_rate": rate(
            false_rows,
            lambda row: row.get("terminal_state") == "REJECTED" and not row.get("cleaning_commanded"),
        ),
        "stale_fail_closed_rate": rate(
            stale_rows,
            lambda row: row.get("terminal_reason") == "sensor_observation_stale"
            and not row.get("navigation_attempted"),
        ),
        "collision_count": sum(int(row.get("collision_count", 0)) for row in rows),
        "keepout_violation_count": sum(int(row.get("keepout_violation_count", 0)) for row in rows),
        "gt_control_violation_count": sum(int(row.get("gt_control_violation_count", 0)) for row in rows),
        "projection": {
            "sample_count": len(projection_rows),
            "actual_center_inside_predicted_roi_rate": rate(
                projection_rows,
                lambda row: row["projection"]["actual_target_center_inside_predicted_roi"],
            ),
            "center_pixel_error_p50": percentile(center_errors, 0.50),
            "center_pixel_error_p95": percentile(center_errors, 0.95),
            "short_side_relative_error_p50": percentile(short_errors, 0.50),
            "short_side_relative_error_p95": percentile(short_errors, 0.95),
            "predicted_actual_ready_agreement": prediction_agreement,
            "self_pixel_fraction_p95": percentile(self_pixels, 0.95),
            "target_self_overlap_p95": percentile(target_self, 0.95),
        },
        "cost": {
            "median_extra_distance_per_confirmed_target_m": median_distance,
            "median_extra_time_per_confirmed_target_s": median_time,
            "median_coverage_interruption_s": median(interruptions) if interruptions else None,
            "candidate_count": len(rows),
            "candidates_per_scene": candidates_per_scene,
            "baseline_coverage_duration_s": baseline_coverage_duration_s,
            "theoretical_throughput_penalty": (
                theoretical_added_time / (baseline_coverage_duration_s + theoretical_added_time)
                if theoretical_added_time is not None else None
            ),
            "measured_throughput_penalty": (
                measured_added_time / (baseline_coverage_duration_s + measured_added_time)
                if measured_added_time is not None else None
            ),
            "throughput_method": (
                "AUTO-02 measured baseline duration plus AUTO-03 median per-confirmed-target "
                "interruption at the observed candidates-per-scene rate"
            ),
        },
    }
    projection = metrics["projection"]
    cost = metrics["cost"]
    checks = {
        "worlds_at_least_6": data_matrix["world_count"] >= 6,
        "scenes_at_least_60": data_matrix["scene_count"] >= 60,
        "valid_target_candidates_at_least_200": data_matrix["valid_target_candidate_count"] >= 200,
        "each_target_class_at_least_30": bool(class_counts) and min(class_counts.values()) >= 30,
        "unreachable_keepout_at_least_30": data_matrix["unreachable_keepout_count"] >= 30,
        "false_no_target_at_least_30": data_matrix["false_no_target_count"] >= 30,
        "stale_dropout_at_least_20": data_matrix["stale_dropout_count"] >= 20,
        "safe_coverage_boundary_pause_100_percent": metrics["safe_coverage_boundary_pause_rate"] == 1.0,
        "preflight_path_success_at_least_0_95": metrics["preflight_path_success_rate"] >= 0.95,
        "navigate_success_at_least_0_95": metrics["navigate_to_pose_success_rate_reachable"] >= 0.95,
        "ready_conversion_overall_at_least_0_90": metrics["ready_conversion_overall"] >= 0.90,
        "discrete_class_conversion_each_at_least_0_85": all(
            per_class_conversion.get(class_id, 0.0) >= 0.85 for class_id in DISCRETE_CLASSES
        ),
        "coverage_resume_at_least_0_95": metrics["coverage_resume_rate"] >= 0.95,
        "unreachable_fail_closed_100_percent": metrics["unreachable_fail_closed_rate"] == 1.0,
        "false_candidate_without_cleaning_100_percent": metrics["false_candidate_without_cleaning_rate"] == 1.0,
        "stale_fail_closed_100_percent": metrics["stale_fail_closed_rate"] == 1.0,
        "collision_zero": metrics["collision_count"] == 0,
        "keepout_zero": metrics["keepout_violation_count"] == 0,
        "gt_control_violation_zero": metrics["gt_control_violation_count"] == 0,
        "projection_center_inside_at_least_0_95": projection["actual_center_inside_predicted_roi_rate"] >= 0.95,
        "projection_center_error_p50_at_most_10_px": projection["center_pixel_error_p50"] is not None and projection["center_pixel_error_p50"] <= 10.0,
        "projection_center_error_p95_at_most_25_px": projection["center_pixel_error_p95"] is not None and projection["center_pixel_error_p95"] <= 25.0,
        "projection_short_error_p50_at_most_0_15": projection["short_side_relative_error_p50"] is not None and projection["short_side_relative_error_p50"] <= 0.15,
        "projection_short_error_p95_at_most_0_30": projection["short_side_relative_error_p95"] is not None and projection["short_side_relative_error_p95"] <= 0.30,
        "predicted_actual_ready_agreement_at_least_0_90": projection["predicted_actual_ready_agreement"] >= 0.90,
        "self_pixels_p95_at_most_0_05": projection["self_pixel_fraction_p95"] is not None and projection["self_pixel_fraction_p95"] <= 0.05,
        "target_self_overlap_at_most_0_05": projection["target_self_overlap_p95"] is not None and projection["target_self_overlap_p95"] <= 0.05,
        "median_extra_distance_at_most_8_m": (
            cost["median_extra_distance_per_confirmed_target_m"] is not None
            and cost["median_extra_distance_per_confirmed_target_m"] <= 8.0
        ),
        "median_extra_time_at_most_45_s": (
            cost["median_extra_time_per_confirmed_target_s"] is not None
            and cost["median_extra_time_per_confirmed_target_s"] <= 45.0
        ),
        "throughput_penalty_at_most_0_25": (
            cost["measured_throughput_penalty"] is not None
            and cost["measured_throughput_penalty"] <= 0.25
        ),
    }
    engineering_suggestions = {
        name: checks[name]
        for name in (
            "median_extra_distance_at_most_8_m",
            "median_extra_time_at_most_45_s",
            "throughput_penalty_at_most_0_25",
        )
    }
    return {
        "schema_version": 1,
        "stage": "AUTO-03",
        "oracle_boundary_fields": sorted(ORACLE_FIELDS),
        "data_matrix": data_matrix,
        "metrics": metrics,
        "checks": checks,
        "engineering_suggestions": engineering_suggestions,
        "auto03_gate_pass": all(checks.values()),
        "trial_count": len(rows),
    }
