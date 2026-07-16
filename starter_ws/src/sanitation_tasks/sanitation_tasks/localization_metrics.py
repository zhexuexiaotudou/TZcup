"""Pure Stage4U localization math shared by ROS nodes and unit tests.

SE(2) transforms use the explicit ``T_target_source`` convention:
``p_target = R(yaw) * p_source + translation``.
"""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path

import yaml


TRANSFORM_CONVENTION = (
    "T_target_source: p_target = R(yaw) * p_source + [x_m, y_m]"
)


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def apply_se2(transform: dict, pose: tuple[float, float, float]):
    """Apply a T_target_source transform to an (x, y, yaw) pose."""
    x, y, yaw = pose
    tx = float(transform["x_m"])
    ty = float(transform["y_m"])
    theta = float(transform["yaw_rad"])
    cosine, sine = math.cos(theta), math.sin(theta)
    return (
        tx + cosine * x - sine * y,
        ty + sine * x + cosine * y,
        normalize_angle(theta + yaw),
    )


def invert_se2(transform: dict) -> dict:
    """Return the inverse transform while preserving explicit units."""
    tx = float(transform["x_m"])
    ty = float(transform["y_m"])
    theta = float(transform["yaw_rad"])
    cosine, sine = math.cos(theta), math.sin(theta)
    return {
        "x_m": -cosine * tx - sine * ty,
        "y_m": sine * tx - cosine * ty,
        "yaw_rad": normalize_angle(-theta),
    }


def compose_se2(left: dict, right: dict) -> dict:
    """Compose T_a_b and T_b_c, returning T_a_c."""
    x, y, yaw = apply_se2(
        left,
        (float(right["x_m"]), float(right["y_m"]), float(right["yaw_rad"])),
    )
    return {"x_m": x, "y_m": y, "yaw_rad": yaw}


def pose_error(estimate: tuple[float, float, float], truth: tuple[float, float, float]):
    return {
        "xy_m": math.hypot(estimate[0] - truth[0], estimate[1] - truth[1]),
        "yaw_rad": abs(normalize_angle(estimate[2] - truth[2])),
    }


def canonical_payload_sha256(payload: dict) -> str:
    encoded = json.dumps(
        payload, ensure_ascii=False, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def load_map_calibration(path: str | Path) -> dict:
    """Load and fail closed on a malformed or directionally inconsistent file."""
    calibration_path = Path(path)
    document = yaml.safe_load(calibration_path.read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        raise ValueError("map calibration must be a YAML mapping")
    if document.get("transform_convention") != TRANSFORM_CONVENTION:
        raise ValueError("unsupported or ambiguous transform convention")
    for key in ("T_map_gt_map", "T_map_map_gt"):
        transform = document.get(key)
        if not isinstance(transform, dict):
            raise ValueError(f"missing {key}")
        for field in ("x_m", "y_m", "yaw_rad"):
            value = transform.get(field)
            if not isinstance(value, (int, float)) or not math.isfinite(float(value)):
                raise ValueError(f"invalid {key}.{field}")
    expected_inverse = invert_se2(document["T_map_gt_map"])
    actual_inverse = document["T_map_map_gt"]
    residual = pose_error(
        (
            float(actual_inverse["x_m"]),
            float(actual_inverse["y_m"]),
            float(actual_inverse["yaw_rad"]),
        ),
        (
            expected_inverse["x_m"],
            expected_inverse["y_m"],
            expected_inverse["yaw_rad"],
        ),
    )
    if residual["xy_m"] > 1.0e-9 or residual["yaw_rad"] > 1.0e-9:
        raise ValueError("T_map_gt_map and T_map_map_gt are not inverses")
    recorded_hash = document.get("calibration_payload_sha256")
    payload = dict(document)
    payload.pop("calibration_payload_sha256", None)
    if recorded_hash != canonical_payload_sha256(payload):
        raise ValueError("map calibration payload SHA-256 mismatch")
    return document


def particle_statistics(particles: list[tuple[float, float, float, float]]) -> dict:
    """Return weighted AMCL diagnostics for x, y, yaw, weight tuples."""
    count = len(particles)
    if count == 0:
        return {"valid": False, "reason": "empty_particle_cloud", "count": 0}
    if any(not all(math.isfinite(value) for value in particle) for particle in particles):
        return {"valid": False, "reason": "non_finite_particle", "count": count}
    weights = [particle[3] for particle in particles]
    if any(weight < 0.0 for weight in weights):
        return {"valid": False, "reason": "negative_weight", "count": count}
    weight_sum = sum(weights)
    if weight_sum <= 0.0:
        return {"valid": False, "reason": "zero_weight_sum", "count": count}
    normalized = [weight / weight_sum for weight in weights]
    mean_x = sum(weight * particle[0] for weight, particle in zip(normalized, particles))
    mean_y = sum(weight * particle[1] for weight, particle in zip(normalized, particles))
    mean_yaw = math.atan2(
        sum(weight * math.sin(particle[2]) for weight, particle in zip(normalized, particles)),
        sum(weight * math.cos(particle[2]) for weight, particle in zip(normalized, particles)),
    )
    deltas = [
        (particle[0] - mean_x, particle[1] - mean_y, normalize_angle(particle[2] - mean_yaw))
        for particle in particles
    ]
    covariance = [
        [
            sum(weight * delta[row] * delta[column] for weight, delta in zip(normalized, deltas))
            for column in range(3)
        ]
        for row in range(3)
    ]
    ess = 1.0 / sum(weight * weight for weight in normalized)
    entropy = -sum(weight * math.log(weight) for weight in normalized if weight > 0.0)
    normalized_entropy = entropy / math.log(count) if count > 1 else 0.0
    spread = math.sqrt(max(0.0, covariance[0][0] + covariance[1][1]))
    return {
        "valid": True,
        "reason": None,
        "count": count,
        "weight_sum": weight_sum,
        "weighted_mean": {"x_m": mean_x, "y_m": mean_y, "yaw_rad": mean_yaw},
        "weighted_covariance_xyyaw": covariance,
        "spread_m": spread,
        "effective_sample_size": ess,
        "effective_sample_ratio": ess / count,
        "max_normalized_weight": max(normalized),
        "entropy": entropy,
        "normalized_entropy": normalized_entropy,
        "degenerate": bool(ess / count < 0.05 or normalized_entropy < 0.10),
    }


MAP_GEOMETRY_GATE_VERSION = "stage4u-v1"
PARTICLE_CLOUD_TYPE = "nav2_msgs/msg/ParticleCloud"


def particle_topic_type_pass(observed_types) -> bool:
    return PARTICLE_CLOUD_TYPE in set(observed_types)


def trial_completion_reasons(trial):
    """Return fail-closed reasons that make a seed unusable as gate evidence."""
    reasons = []
    if int(trial.get("sample_count", 0) or 0) <= 0:
        reasons.append("no_synchronized_localization_samples")
    if int(trial.get("estimate_sample_count", 0) or 0) <= 0:
        reasons.append("no_estimate_samples")
    if int(trial.get("truth_sample_count", 0) or 0) <= 0:
        reasons.append("no_truth_samples")
    if trial.get("navigation_exit_code") != 0:
        reasons.append("navigation_nonzero_exit")
    if not trial.get("navigation", {}).get("success"):
        reasons.append("navigation_not_completed")
    if trial.get("map_relative_localization_error", {}).get("xy_m", {}).get("rmse") is None:
        reasons.append("missing_map_relative_xy_rmse")
    particle = trial.get("particle_filter", {})
    if particle.get("particle_instrumentation_required", True) and not particle.get(
        "particle_instrumentation_pass"
    ):
        reasons.append("particle_instrumentation_invalid")
    return reasons


def map_gate_result(mapping: dict, quality: dict, geometry: dict) -> dict:
    """Separate map production, basic coverage, and localization geometry gates."""
    resolution = float(geometry["map_resolution_m"])
    thresholds = {
        "version": MAP_GEOMETRY_GATE_VERSION,
        "boundary_chamfer_mean_max_m": 1.5 * resolution,
        "boundary_p95_max_m": 2.0 * resolution,
        "straight_line_angle_rmse_max_deg": 3.0,
        "loop_ghosting_ratio_max": 0.02,
        "occupancy_iou_min": 0.35,
    }
    checks = {
        "boundary_chamfer_mean": geometry.get("boundary_chamfer_distance_m") is not None
        and float(geometry["boundary_chamfer_distance_m"])
        <= thresholds["boundary_chamfer_mean_max_m"],
        "boundary_p95": geometry.get("boundary_p95_m") is not None
        and float(geometry["boundary_p95_m"]) <= thresholds["boundary_p95_max_m"],
        "straight_line_angle_rmse": geometry.get("straight_line_angle_error_deg", {}).get("rmse")
        is not None
        and float(geometry["straight_line_angle_error_deg"]["rmse"])
        <= thresholds["straight_line_angle_rmse_max_deg"],
        "loop_ghosting": geometry.get("loop_ghosting_ratio") is not None
        and float(geometry["loop_ghosting_ratio"])
        <= thresholds["loop_ghosting_ratio_max"],
        "occupancy_iou": geometry.get("occupancy_iou") is not None
        and float(geometry["occupancy_iou"]) >= thresholds["occupancy_iou_min"],
    }
    generation_pass = bool(mapping.get("success") and mapping.get("route_completed"))
    basic_pass = bool(generation_pass and quality.get("slam_quality_pass"))
    return {
        "map_generation_pass": generation_pass,
        "map_basic_quality_pass": basic_pass,
        "map_localization_geometry_pass": bool(basic_pass and all(checks.values())),
        "localization_geometry_thresholds": thresholds,
        "localization_geometry_checks": checks,
    }
