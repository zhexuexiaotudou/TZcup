"""Repository-backed camera, target and vehicle geometry audit for OPRV3."""

from __future__ import annotations

import hashlib
import math
from pathlib import Path
from typing import Mapping

import yaml

from .g4_assets import GEOMETRY_PARAMS


PIXEL_LEVELS = (8, 12, 18, 24, 32)


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _load_yaml(path: Path) -> dict:
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def _physical_dimensions(kind: str, values: tuple[float, ...]) -> tuple[float, float, float]:
    if kind == "cylinder":
        radius, length = values
        return 2.0 * radius, 2.0 * radius, length
    if kind in {"box", "ellipsoid"}:
        return tuple(float(value) for value in values)
    raise ValueError(f"unsupported geometry kind {kind}")


def _class_geometry() -> dict[str, dict]:
    prefixes = {
        "plastic_bottle": "bottle_",
        "metal_can": "can_",
        "paper_litter": "paper_",
        "leaf_pile": "leaf_pile_",
        "puddle": "puddle_",
    }
    report: dict[str, dict] = {}
    for class_id, prefix in prefixes.items():
        variants = []
        for family, (kind, raw_values) in GEOMETRY_PARAMS.items():
            if not family.startswith(prefix):
                continue
            dimensions = _physical_dimensions(kind, raw_values)
            ground_sides = sorted(dimensions[:2])
            # The shortest projected extent is a conservative, orientation-
            # independent bbox-short-side proxy for the development audit.
            # G4 targets rest on the ground plane.  Thickness is not a valid
            # projected bbox-short-side proxy for flat paper/leaf/puddle
            # assets; use the two footprint axes frozen by the asset registry.
            short_side = min(dimensions[:2])
            face_area = ground_sides[0] * ground_sides[1]
            variants.append({
                "geometry_family": family,
                "kind": kind,
                "dimensions_m": list(dimensions),
                "conservative_short_side_m": short_side,
                "ground_face_area_m2": face_area,
            })
        if not variants:
            raise ValueError(f"no G4 geometry for {class_id}")
        conservative = min(item["conservative_short_side_m"] for item in variants)
        conservative_area = min(item["ground_face_area_m2"] for item in variants)
        report[class_id] = {
            "source": "sanitation_learning.g4_assets.GEOMETRY_PARAMS",
            "variant_count": len(variants),
            "variants": variants,
            "conservative_short_side_m": conservative,
            "conservative_projected_area_m2": conservative_area,
        }
    return report


def _ground_intersections(camera_x_m: float, camera_z_m: float, pitch_deg: float, vfov_rad: float, near_clip_m: float, far_clip_m: float) -> tuple[float, float]:
    pitch = math.radians(pitch_deg)
    top = pitch + vfov_rad / 2.0
    bottom = pitch - vfov_rad / 2.0

    def intersect(elevation: float) -> float | None:
        if elevation >= 0.0:
            return None
        return camera_x_m + camera_z_m / math.tan(-elevation)

    near = intersect(bottom)
    far = intersect(top)
    return (
        max(near_clip_m, near if near is not None else near_clip_m),
        min(far_clip_m, far if far is not None else far_clip_m),
    )


def derive_product_geometry(repo_root: str | Path) -> dict:
    root = Path(repo_root)
    camera_path = root / "starter_ws/src/sanitation_learning/config/auto05r_product_camera.yaml"
    nav2_path = root / "starter_ws/src/sanitation_navigation/config/nav2.yaml"
    spot_path = root / "starter_ws/src/sanitation_spot_cleaning/config/spot_cleaning.yaml"
    task_path = root / "starter_ws/src/sanitation_tasks/config/competition_demo_area_skid_steer_optimized.yaml"
    policy_path = root / "starter_ws/src/sanitation_learning/config/perception_evaluability_policy_v2_engineering.yaml"
    sources = [camera_path, nav2_path, spot_path, task_path, policy_path]
    for path in sources:
        if not path.is_file():
            raise FileNotFoundError(path)

    camera_doc = _load_yaml(camera_path)
    nav2_doc = _load_yaml(nav2_path)
    spot_doc = _load_yaml(spot_path)
    task_doc = _load_yaml(task_path)
    policy_doc = _load_yaml(policy_path)
    overrides = camera_doc["xacro_overrides"]
    camera_contract = camera_doc["observation_contract"]
    width, height = (int(value) for value in camera_contract["native_resolution"])
    hfov = float(camera_contract["horizontal_fov_rad"])
    vfov = 2.0 * math.atan(math.tan(hfov / 2.0) * height / width)
    focal_x = width / (2.0 * math.tan(hfov / 2.0))
    focal_y = height / (2.0 * math.tan(vfov / 2.0))
    pitch_deg = float(camera_doc["physical_mount_pitch_deg"])
    near_ground, far_ground = _ground_intersections(
        float(overrides["camera_x"]), float(overrides["camera_z"]), pitch_deg,
        vfov, 0.3, 100.0,
    )

    clean_controller = nav2_doc["controller_server"]["ros__parameters"]["CleanPath"]
    smoother = nav2_doc["velocity_smoother"]["ros__parameters"]
    speed = float(clean_controller["desired_linear_vel"])
    deceleration = abs(float(smoother["max_decel"][0]))
    control_latency_s = 0.15
    braking_distance = speed * control_latency_s + speed * speed / (2.0 * deceleration)
    brush_offset = float(task_doc["brush_forward_offset_m"])
    minimum_range = brush_offset + braking_distance
    confirmation_observations = int(spot_doc["confirmation_observations"])
    frame_rate = float(camera_contract["update_rate_hz"])

    class_geometry = _class_geometry()
    class_windows: dict[str, dict] = {}
    for class_id, geometry in class_geometry.items():
        short_side = float(geometry["conservative_short_side_m"])
        area = float(geometry["conservative_projected_area_m2"])
        distances = {str(px): focal_y * short_side / px for px in PIXEL_LEVELS}
        pixel_area = {
            str(round(distance, 3)): focal_x * focal_y * area / (distance * distance)
            for distance in (0.9, 1.2, 1.8, 2.4, 3.2)
        }
        maximum_range = min(far_ground, distances["8"])
        visibility = 0.70 if class_id in {"leaf_pile", "puddle"} else 0.60
        class_windows[class_id] = {
            "minimum_actionable_range_m": minimum_range,
            "maximum_actionable_range_m": maximum_range,
            "minimum_visible_frames": confirmation_observations,
            "minimum_visibility_ratio": visibility,
            "minimum_depth_valid_ratio": 0.80,
            "window_duration_at_normal_speed_s": max(0.0, maximum_range - minimum_range) / speed,
            "frames_in_actionable_window": max(
                0,
                math.floor((maximum_range - minimum_range) / speed * frame_rate + 1e-9),
            ),
            "distance_at_bbox_short_side_px_m": distances,
            "estimated_pixel_area_by_distance_px2": pixel_area,
            "window_nonempty": maximum_range > minimum_range,
        }

    source_records = [
        {"path": path.relative_to(root).as_posix(), "sha256": _sha256(path)}
        for path in sources
    ]
    return {
        "schema_version": 1,
        "protocol": "OPRV3-01",
        "frozen_before_moving_model_measurement": True,
        "camera": {
            "profile_id": camera_doc["profile_id"],
            "resolution": [width, height],
            "horizontal_fov_rad": hfov,
            "vertical_fov_rad": vfov,
            "focal_length_px": {"fx": focal_x, "fy": focal_y},
            "base_link_xyz_m": [float(overrides["camera_x"]), float(overrides["camera_y"]), float(overrides["camera_z"])],
            "pitch_deg": pitch_deg,
            "update_rate_hz": frame_rate,
            "near_ground_intersection_base_x_m": near_ground,
            "far_ground_intersection_base_x_m": far_ground,
        },
        "vehicle_and_action": {
            "normal_product_speed_m_s": speed,
            "maximum_deceleration_m_s2": deceleration,
            "control_latency_s": control_latency_s,
            "braking_distance_including_latency_m": braking_distance,
            "brush_forward_offset_m": brush_offset,
            "no_return_range_from_base_m": minimum_range,
            "spot_clean_confirmation_observations": confirmation_observations,
            "nav2_active_observation_standoff_m": {"minimum": 0.65, "maximum": 1.35},
        },
        "observation_semantics": {
            "low_confidence_observation_bbox_short_side_px": 8,
            "legacy_static_diagnostic_bbox_short_side_px": 18,
            "low_confidence_observation_is_actionable": False,
            "action_requires_confirmed_track": True,
            "actionable_window_eligibility_uses_model_result": False,
        },
        "target_geometry": class_geometry,
        "class_actionable_windows": class_windows,
        "all_classes_have_nonempty_window": all(item["window_nonempty"] for item in class_windows.values()),
        "source_files": source_records,
        "limitations": [
            "Analytic pinhole geometry is not empirical Gazebo evidence.",
            "Projected short-side and area estimates are conservative geometry proxies; empirical masks and bboxes remain required.",
            "The actionable-window contract is frozen before model output is inspected.",
        ],
    }


def class_window_kwargs(report: Mapping) -> dict[str, dict]:
    fields = {
        "minimum_actionable_range_m", "maximum_actionable_range_m",
        "minimum_visible_frames", "minimum_visibility_ratio",
        "minimum_depth_valid_ratio",
    }
    return {
        class_id: {name: values[name] for name in fields}
        for class_id, values in report["class_actionable_windows"].items()
    }
