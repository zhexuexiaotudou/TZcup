from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Aabb:
    center: tuple[float, float, float]
    half: tuple[float, float, float]

    @classmethod
    def from_center_size(cls, center, size) -> "Aabb":
        return cls(tuple(float(v) for v in center), tuple(float(v) / 2.0 for v in size))

    @property
    def minimum(self):
        return tuple(value - half for value, half in zip(self.center, self.half))

    @property
    def maximum(self):
        return tuple(value + half for value, half in zip(self.center, self.half))

    def intersects(self, other: "Aabb", tolerance: float = 1e-9) -> bool:
        return all(
            self.minimum[index] < other.maximum[index] - tolerance
            and self.maximum[index] > other.minimum[index] + tolerance
            for index in range(3)
        )


def pitched_camera_aabb(xyz_m, size_m, pitch_deg: float) -> Aabb:
    length, width, height = (float(v) for v in size_m)
    angle = math.radians(float(pitch_deg))
    half_x = abs(math.cos(angle)) * length / 2.0 + abs(math.sin(angle)) * height / 2.0
    half_z = abs(math.sin(angle)) * length / 2.0 + abs(math.cos(angle)) * height / 2.0
    return Aabb(tuple(float(v) for v in xyz_m), (half_x, width / 2.0, half_z))


def _inside_polygon(point, polygon) -> bool:
    x, y = point
    inside = False
    for index, first in enumerate(polygon):
        second = polygon[(index + 1) % len(polygon)]
        x1, y1 = first
        x2, y2 = second
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / (y2 - y1) + x1:
            inside = not inside
    return inside


def _aabb_intersects_cylinder(box: Aabb, cylinder: dict) -> bool:
    center = tuple(float(v) for v in cylinder["center_m"])
    radius = float(cylinder["radius_m"])
    half_height = float(cylinder["height_m"]) / 2.0
    if box.maximum[2] <= center[2] - half_height or box.minimum[2] >= center[2] + half_height:
        return False
    closest_x = min(max(center[0], box.minimum[0]), box.maximum[0])
    closest_y = min(max(center[1], box.minimum[1]), box.maximum[1])
    return math.hypot(closest_x - center[0], closest_y - center[1]) < radius


def evaluate_camera_mechanics(camera_id: str, camera: dict, document: dict) -> dict:
    vehicle = document["vehicle_collision_geometry"]
    box = pitched_camera_aabb(camera["base_link_xyz_m"], document["camera_common"]["collision_box_size_m"], camera["pitch_deg"])
    named_boxes = {
        "body": Aabb.from_center_size(vehicle["body_aabb"]["center_m"], vehicle["body_aabb"]["size_m"]),
        "front_bumper_reserved": Aabb.from_center_size(vehicle["front_bumper_reserved_aabb"]["center_m"], vehicle["front_bumper_reserved_aabb"]["size_m"]),
        "mechanical_arm_reserved": Aabb.from_center_size(vehicle["mechanical_arm_reserved_aabb"]["center_m"], vehicle["mechanical_arm_reserved_aabb"]["size_m"]),
    }
    collisions = [name for name, obstacle in named_boxes.items() if box.intersects(obstacle)]
    collisions.extend(
        f"brush_{index}" for index, cylinder in enumerate(vehicle["brush_cylinders"])
        if _aabb_intersects_cylinder(box, cylinder)
    )
    trial_polygon = [tuple(float(v) for v in point) for point in vehicle["stage5br5_trial_footprint_xy_m"]]
    production_polygon = [tuple(float(v) for v in point) for point in vehicle["production_nav2_footprint_xy_m"]]
    corners_xy = [
        (x, y)
        for x in (box.minimum[0], box.maximum[0])
        for y in (box.minimum[1], box.maximum[1])
    ]
    inside_trial = all(_inside_polygon(point, trial_polygon) for point in corners_xy)
    inside_production = all(_inside_polygon(point, production_polygon) for point in corners_xy)
    no_ground_penetration = box.minimum[2] > float(vehicle["ground_z_m"])
    height_pass = box.maximum[2] <= float(vehicle["maximum_mount_height_m"])
    collision_free = not collisions
    mechanical_pass = collision_free and no_ground_penetration and height_pass and inside_trial
    return {
        "camera_id": camera_id,
        "label": camera["label"],
        "base_link_xyz_m": camera["base_link_xyz_m"],
        "front_bumper_relative_xyz_m": camera["front_bumper_relative_xyz_m"],
        "pitch_deg": camera["pitch_deg"],
        "camera_aabb_min_m": list(box.minimum),
        "camera_aabb_max_m": list(box.maximum),
        "collision_geometry_sources": ["body_aabb", "front_bumper_reserved_aabb", "mechanical_arm_reserved_aabb", "brush_cylinders"],
        "collisions": collisions,
        "collision_free": collision_free,
        "no_ground_penetration": no_ground_penetration,
        "within_mount_height": height_pass,
        "inside_stage5br5_trial_footprint": inside_trial,
        "inside_current_production_nav2_footprint": inside_production,
        "production_footprint_change_applied": False,
        "mechanical_gate_pass": mechanical_pass,
    }


def evaluate_all(document: dict) -> dict:
    results = {camera_id: evaluate_camera_mechanics(camera_id, camera, document) for camera_id, camera in document["camera_configs"].items()}
    survivors = [camera_id for camera_id, item in results.items() if item["mechanical_gate_pass"]]
    pruned = [camera_id for camera_id, item in results.items() if not item["mechanical_gate_pass"]]
    return {
        "schema_version": 1,
        "stage": "Stage5BR5",
        "source_xacro": document["vehicle_collision_geometry"]["source_xacro"],
        "trial_footprint_only": bool(document["vehicle_collision_geometry"]["trial_footprint_only"]),
        "camera_results": results,
        "all_candidates_mechanical_gate_pass": all(item["mechanical_gate_pass"] for item in results.values()),
        "mechanically_viable_candidates": survivors,
        "mechanically_pruned_candidates": pruned,
        "mechanical_grid_executed": True,
        "mechanical_grid_has_viable_candidate": bool(survivors),
        "production_nav2_footprint_unchanged": True,
    }
