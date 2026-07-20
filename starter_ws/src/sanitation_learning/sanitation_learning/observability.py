from __future__ import annotations

from collections import defaultdict
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import yaml


DISCRETE_CLASSES = {"plastic_bottle", "metal_can", "paper_litter"}
AREA_CLASSES = {"leaf_pile", "puddle"}


def sha256_file(path: str | Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_policy(path: str | Path) -> dict:
    payload = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    required = {"discrete_recognition_ready", "area_recognition_ready", "report_partitions"}
    if not required.issubset(payload):
        raise ValueError(f"evaluability policy missing {sorted(required - set(payload))}")
    if payload["report_partitions"] != ["all_visible", "recognition_ready", "non_ready"]:
        raise ValueError("all_visible/recognition_ready/non_ready reporting is mandatory")
    return payload


def recognition_ready(row: dict, policy: dict) -> bool:
    if row.get("visibility") != "visible":
        return False
    class_id = row.get("class_id") or row.get("semantic_class")
    if class_id in DISCRETE_CLASSES:
        rule = policy["discrete_recognition_ready"]
        distance = row.get("distance_m")
        return (
            float(row.get("bbox_shortest_side_px", 0)) >= float(rule["shortest_side_px"])
            and float(row.get("mask_area_px", 0)) >= float(rule["mask_area_px"])
            and distance is not None
            and math.isfinite(float(distance))
            and float(distance) <= float(rule["max_depth_m"])
        )
    if class_id in AREA_CLASSES:
        rule = policy["area_recognition_ready"]
        visible_fraction = row.get("visible_fraction")
        minimum = rule.get("min_visible_fraction")
        return float(row.get("mask_area_px", 0)) >= float(rule["mask_area_px"]) and (
            minimum is None or (visible_fraction is not None and float(visible_fraction) >= float(minimum))
        )
    return False


def camera_ground_geometry(config: dict, common: dict) -> dict:
    """Conservative pinhole ground intersection, in base_link coordinates."""
    width, height = common["resolution"]
    hfov = float(common["horizontal_fov_rad"])
    vfov = 2.0 * math.atan(math.tan(hfov / 2.0) * height / width)
    x, y, z = (float(v) for v in config["xyz_m"])
    pitch = math.radians(float(config["pitch_deg"]))
    # Optical axis is forward; downward rays have negative elevation.
    top = pitch + vfov / 2.0
    bottom = pitch - vfov / 2.0

    def hit(elevation: float) -> float | None:
        if elevation >= -1e-9:
            return None
        return x + z / math.tan(-elevation)

    near_x = hit(bottom)
    far_x = hit(top)
    far_limited = min(far_x if far_x is not None else float(common["far_clip_m"]), float(common["far_clip_m"]))
    near_limited = max(float(common["near_clip_m"]), near_x if near_x is not None else float(common["near_clip_m"]))
    half_near = max(0.0, near_limited - x) * math.tan(hfov / 2.0)
    half_far = max(0.0, far_limited - x) * math.tan(hfov / 2.0)
    polygon = [
        [near_limited, y - half_near], [near_limited, y + half_near],
        [far_limited, y + half_far], [far_limited, y - half_far],
    ]
    blind_zone = max(0.0, near_limited - 0.575)
    return {
        "vertical_fov_rad": vfov,
        "ground_coverage_polygon_base_xy_m": polygon,
        "near_field_blind_zone_m_from_front_bumper": blind_zone,
        "horizon_in_frame": top >= 0.0,
        "analytic_geometry_only": True,
    }


def summarize_rows(rows: list[dict], policy: dict) -> dict:
    enriched = []
    for source in rows:
        row = dict(source)
        row["class_id"] = row.get("class_id") or row.get("semantic_class")
        row["recognition_ready"] = recognition_ready(row, policy)
        enriched.append(row)

    def metrics(items: list[dict]) -> dict:
        result = {"count": len(items), "by_class": {}}
        grouped = defaultdict(list)
        for item in items:
            grouped[item.get("class_id", "unknown")].append(item)
        for class_id, group in sorted(grouped.items()):
            entry = {"count": len(group)}
            for key in ("bbox_shortest_side_px", "mask_area_px", "distance_m", "visible_fraction", "occlusion", "depth_valid_ratio"):
                values = [float(x[key]) for x in group if x.get(key) is not None and math.isfinite(float(x[key]))]
                entry[key] = {
                    "count": len(values),
                    "p10": float(np.percentile(values, 10)) if values else None,
                    "p50": float(np.percentile(values, 50)) if values else None,
                    "p90": float(np.percentile(values, 90)) if values else None,
                }
            result["by_class"][class_id] = entry
        return result

    visible = [row for row in enriched if row.get("visibility") == "visible"]
    ready = [row for row in visible if row["recognition_ready"]]
    non_ready = [row for row in visible if not row["recognition_ready"]]
    return {
        "all_visible": metrics(visible),
        "recognition_ready": metrics(ready),
        "non_ready": metrics(non_ready),
        "recognition_ready_fraction": len(ready) / len(visible) if visible else 0.0,
    }


def build_report(rows: list[dict], policy_path: str | Path, config_path: str | Path, camera_id: str) -> dict:
    policy = load_policy(policy_path)
    config_doc = yaml.safe_load(Path(config_path).read_text(encoding="utf-8"))
    camera = config_doc["camera_configs"][camera_id]
    if camera.get("mode") == "dual_camera":
        geometry = {
            role: camera_ground_geometry(spec, config_doc["camera_common"])
            for role, spec in (("discovery", camera["discovery"]), ("verification", camera["verification"]))
        }
        bandwidth = 2.0 * float(config_doc["camera_common"]["estimated_rgbd_mbps_per_camera"])
        maximum_z = max(float(camera["discovery"]["xyz_m"][2]), float(camera["verification"]["xyz_m"][2]))
    else:
        geometry = camera_ground_geometry(camera, config_doc["camera_common"])
        bandwidth = float(config_doc["camera_common"]["estimated_rgbd_mbps_per_camera"])
        maximum_z = float(camera["xyz_m"][2])
    mount_limit = float(config_doc["vehicle"]["maximum_mount_height_m"])
    return {
        "schema_version": 1,
        "stage": "Stage5BR4",
        "camera_config": camera_id,
        "camera_spec": camera,
        "evaluability_policy_sha256": sha256_file(policy_path),
        "partitions": summarize_rows(rows, policy),
        "ground_coverage": geometry,
        "estimated_uncompressed_rgbd_bandwidth_mbps": bandwidth,
        "mounting_and_collision_risk": {
            "maximum_camera_z_m": maximum_z,
            "allowed_mount_height_m": mount_limit,
            "height_envelope_pass": maximum_z <= mount_limit,
            "collision_box_m": config_doc["vehicle"]["camera_collision_box_m"],
            "physical_collision_test_executed": False,
        },
        "self_pixel_fraction": None,
        "self_pixel_measurement_status": "requires runtime vehicle self-mask; not inferred from background pixels",
    }


def write_report(report: dict, output: str | Path) -> None:
    path = Path(output)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
