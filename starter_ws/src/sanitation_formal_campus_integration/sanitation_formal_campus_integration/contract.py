"""ROS-independent contract loading and Nav2 profile materialization."""

from __future__ import annotations

from copy import deepcopy
import json
import math
from pathlib import Path
from typing import Any

import yaml


FORMAL_KINEMATIC_MODEL = "four_wheel_skid_steer"
CANONICAL_PLANNING_KINEMATIC_CONSTRAINT = "curvature_limited_reference_path_for_skid_steer"
OLD_SMALL_FOOTPRINT = [
    [0.40, 0.36],
    [0.40, -0.36],
    [-0.40, -0.36],
    [-0.40, 0.36],
]


class IntegrationContractError(RuntimeError):
    """Raised when a campus integration input violates the formal contract."""


def load_yaml_mapping(path: str | Path) -> dict[str, Any]:
    value = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise IntegrationContractError(f"expected YAML mapping: {path}")
    return value


def footprint_string(points: list[list[float]]) -> str:
    """Return the Nav2 polygon string without changing source precision."""
    return json.dumps(points, separators=(",", ":"))


def formal_motion_values(
    motion_profile_path: str | Path,
    *,
    navigation_footprint_key: str = "transport_stowed",
) -> tuple[list[list[float]], float]:
    profile = load_yaml_mapping(motion_profile_path)
    drive = profile.get("drive", {})
    if drive.get("kinematic_model") != FORMAL_KINEMATIC_MODEL:
        raise IntegrationContractError(
            "formal vehicle must remain a four-wheel skid-steer platform"
        )
    if drive.get("steering_joint_names"):
        raise IntegrationContractError(
            "formal skid-steer profile cannot claim steering joints"
        )
    if drive.get("canonical_planning_kinematic_constraint") != CANONICAL_PLANNING_KINEMATIC_CONSTRAINT:
        raise IntegrationContractError(
            "formal skid-steer profile must use the canonical curvature-limited reference constraint"
        )
    canonical_claim = drive.get("canonical_constraint_claim", {})
    if canonical_claim.get("physical_steering_claim") is not False:
        raise IntegrationContractError(
            "formal skid-steer reference constraint cannot claim physical steering"
        )
    if canonical_claim.get("runtime_tracking_status") != "pending_skid_steer_tracking_validation":
        raise IntegrationContractError(
            "formal skid-steer reference tracking must remain pending runtime validation"
        )
    virtual_ackermann = drive.get("virtual_ackermann_constraint", {})
    if virtual_ackermann.get("compatibility_alias_for") != CANONICAL_PLANNING_KINEMATIC_CONSTRAINT:
        raise IntegrationContractError(
            "virtual Ackermann compatibility key must point to the canonical skid-steer constraint"
        )
    footprints = profile.get("motion_footprints", {})
    selected = footprints.get(navigation_footprint_key, {})
    points = selected.get("footprint_xy_m")
    if not isinstance(points, list) or len(points) < 3:
        raise IntegrationContractError("formal navigation footprint is missing or invalid")
    normalized = [[float(value) for value in point] for point in points]
    if normalized == OLD_SMALL_FOOTPRINT:
        raise IntegrationContractError(
            "legacy small footprint cannot represent the formal vehicle"
        )
    width = (
        profile.get("mechanism_sweeps", {})
        .get("transverse_union", {})
        .get("declared_effective_cleaning_width_m")
    )
    if not isinstance(width, (int, float)) or isinstance(width, bool) or width <= 0:
        raise IntegrationContractError("formal effective cleaning width is missing")
    return normalized, float(width)


def materialize_nav2_config(
    base_nav2_path: str | Path,
    motion_profile_path: str | Path,
    *,
    navigation_footprint_key: str = "transport_stowed",
    clean_path_speed_mps: float | None = None,
) -> tuple[dict[str, Any], float]:
    """Replace both costmap polygons from the canonical formal motion profile."""
    config = deepcopy(load_yaml_mapping(base_nav2_path))
    points, cleaning_width = formal_motion_values(
        motion_profile_path,
        navigation_footprint_key=navigation_footprint_key,
    )
    polygon = footprint_string(points)
    try:
        config["local_costmap"]["local_costmap"]["ros__parameters"]["footprint"] = polygon
        config["global_costmap"]["global_costmap"]["ros__parameters"]["footprint"] = polygon
        if clean_path_speed_mps is not None:
            speed = float(clean_path_speed_mps)
            if not math.isfinite(speed) or speed <= 0.0:
                raise IntegrationContractError("formal CleanPath speed is invalid")
            config["controller_server"]["ros__parameters"]["CleanPath"][
                "desired_linear_vel"
            ] = speed
            config["velocity_smoother"]["ros__parameters"]["max_velocity"][0] = speed
    except (KeyError, TypeError) as exc:
        raise IntegrationContractError("base Nav2 config has no formal runtime slots") from exc
    return config, cleaning_width


def write_materialized_nav2_config(
    base_nav2_path: str | Path,
    motion_profile_path: str | Path,
    output_path: str | Path,
) -> float:
    config, cleaning_width = materialize_nav2_config(
        base_nav2_path, motion_profile_path
    )
    Path(output_path).write_text(
        yaml.safe_dump(config, sort_keys=False), encoding="utf-8"
    )
    return cleaning_width


def resolve_spawn_pose(
    episode_manifest_path: str | Path,
    *,
    spawn_x: float | None = None,
    spawn_y: float | None = None,
    spawn_yaw: float | None = None,
) -> tuple[float, float, float]:
    try:
        manifest = json.loads(Path(episode_manifest_path).read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise IntegrationContractError("unable to read public spawn manifest") from exc
    source = manifest.get("vehicle_start_pose_source_world")
    legacy_source = manifest.get("vehicle_start_pose_map")
    if source is None:
        source = legacy_source
    if not isinstance(source, dict):
        raise IntegrationContractError("episode manifest has no source-world start pose")
    try:
        if isinstance(legacy_source, dict) and manifest.get("vehicle_start_pose_source_world") is not None:
            explicit_values = tuple(float(source[key]) for key in ("x_m", "y_m", "yaw_rad"))
            legacy_values = tuple(float(legacy_source[key]) for key in ("x_m", "y_m", "yaw_rad"))
            if explicit_values != legacy_values:
                raise IntegrationContractError("explicit and legacy source-world start poses disagree")
        values = (
            float(source["x_m"]) if spawn_x is None else float(spawn_x),
            float(source["y_m"]) if spawn_y is None else float(spawn_y),
            float(source.get("yaw_rad", 0.0)) if spawn_yaw is None else float(spawn_yaw),
        )
    except (KeyError, TypeError, ValueError) as exc:
        raise IntegrationContractError("vehicle start pose is invalid") from exc
    if not all(math.isfinite(value) for value in values):
        raise IntegrationContractError("vehicle start pose must be finite")
    return values
