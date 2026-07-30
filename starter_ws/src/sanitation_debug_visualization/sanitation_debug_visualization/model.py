from __future__ import annotations

from dataclasses import dataclass, replace
import math
from pathlib import Path
import zlib

import yaml


Color = tuple[float, float, float, float]
Point = tuple[float, float, float]


CLASS_STYLES: dict[str, tuple[str, Color]] = {
    "plastic_bottle": ("BOTTLE", (0.10, 0.55, 1.00, 0.72)),
    "metal_can": ("CAN", (0.85, 0.20, 0.18, 0.72)),
    "paper_litter": ("PAPER", (1.00, 0.88, 0.20, 0.72)),
    "leaf_pile": ("LEAVES", (0.88, 0.40, 0.08, 0.50)),
    "puddle": ("PUDDLE", (0.08, 0.45, 1.00, 0.38)),
}

NEGATIVE_LABELS = {
    "trash_bin_obstacle": "OBSTACLE | FIXED BIN",
    "cardboard_box_obstacle": "OBSTACLE | BOX",
    "dynamic_pedestrian_box": "OBSTACLE | PEDESTRIAN",
    "structured_waste_bin": "OBSTACLE | WASTE BIN",
}

TARGET_LABEL_OFFSETS: dict[str, tuple[float, float]] = {
    "trash_bottle_01": (-0.45, -0.58),
    "trash_can_01": (-0.55, 0.28),
    "trash_paper_01": (0.10, 0.62),
    "leaf_pile_01": (0.72, -0.28),
    "puddle_zone": (0.00, 0.55),
}

OBSTACLE_LABEL_OFFSETS: dict[str, tuple[float, float]] = {
    "trash_bin_obstacle": (0.0, 0.72),
    "cardboard_box_obstacle": (0.0, 0.65),
    "dynamic_pedestrian_box": (0.0, 0.55),
    "structured_waste_bin": (1.30, -0.55),
}


@dataclass(frozen=True)
class MarkerSpec:
    namespace: str
    key: str
    kind: str
    position: Point = (0.0, 0.0, 0.0)
    yaw: float = 0.0
    scale: Point = (0.1, 0.1, 0.1)
    color: Color = (1.0, 1.0, 1.0, 1.0)
    text: str = ""
    points: tuple[Point, ...] = ()

    @property
    def marker_id(self) -> int:
        value = f"{self.namespace}:{self.key}".encode("utf-8")
        return zlib.crc32(value) & 0x7FFFFFFF


def load_yaml(path: str | Path) -> dict:
    return yaml.safe_load(Path(path).read_text(encoding="utf-8"))


def class_style(class_id: str) -> tuple[str, Color]:
    return CLASS_STYLES.get(class_id, (class_id, (0.80, 0.80, 0.80, 0.70)))


def closed_polyline(points: list[list[float]], z: float) -> tuple[Point, ...]:
    converted = tuple((float(point[0]), float(point[1]), z) for point in points)
    if not converted:
        return ()
    return (*converted, converted[0])


def target_specs(
    *,
    key: str,
    class_id: str,
    position: Point,
    size: Point,
    yaw: float = 0.0,
    prefix: str = "GT",
    confidence: float | None = None,
    cleaned: bool = False,
    label_offset: tuple[float, float] = (0.0, 0.0),
) -> list[MarkerSpec]:
    class_label, base_color = class_style(class_id)
    color = (0.12, 0.92, 0.30, 0.72) if cleaned else base_color
    shape = "cylinder" if class_id in {"plastic_bottle", "metal_can"} else "cube"
    safe_size = (
        max(float(size[0]), 0.04),
        max(float(size[1]), 0.04),
        max(float(size[2]), 0.025),
    )
    status = "CLEANED" if cleaned else prefix
    confidence_text = "" if confidence is None else f" {confidence:.2f}"
    label = f"{status} | {class_label}{confidence_text}"
    label_height = max(position[2] + safe_size[2] * 0.5 + 0.22, 0.34)
    return [
        MarkerSpec(
            namespace=f"{prefix}_halos",
            key=f"{key}_halo",
            kind="cylinder",
            position=(position[0], position[1], max(position[2] - 0.015, 0.012)),
            scale=(
                max(safe_size[0], 0.42),
                max(safe_size[1], 0.42),
                0.024,
            ),
            color=(color[0], color[1], color[2], min(color[3], 0.22)),
        ),
        MarkerSpec(
            namespace=f"{prefix}_targets",
            key=f"{key}_shape",
            kind=shape,
            position=position,
            yaw=yaw,
            scale=safe_size,
            color=color,
        ),
        MarkerSpec(
            namespace=f"{prefix}_labels",
            key=f"{key}_label",
            kind="text",
            position=(
                position[0] + label_offset[0],
                position[1] + label_offset[1],
                label_height,
            ),
            scale=(0.0, 0.0, 0.28),
            color=(1.0, 1.0, 1.0, 1.0),
            text=label,
        ),
    ]


def build_static_specs(
    registry: dict,
    scene: dict,
    mission: dict,
    cleaned_uuids: set[str] | None = None,
) -> list[MarkerSpec]:
    cleaned_uuids = cleaned_uuids or set()
    specs: list[MarkerSpec] = []
    tx, ty = (float(value) for value in scene["world_to_map_translation"])

    outer = mission.get("outer_polygon", [])
    if outer:
        specs.append(
            MarkerSpec(
                namespace="zones",
                key="coverage_boundary",
                kind="line_strip",
                scale=(0.065, 0.0, 0.0),
                color=(0.10, 0.85, 0.95, 0.95),
                points=closed_polyline(outer, 0.045),
            )
        )
        min_x = min(float(point[0]) for point in outer)
        max_y = max(float(point[1]) for point in outer)
        specs.append(
            MarkerSpec(
                namespace="zone_labels",
                key="coverage_boundary",
                kind="text",
                position=(min_x + 0.2, max_y + 0.35, 0.35),
                scale=(0.0, 0.0, 0.28),
                color=(0.10, 0.95, 1.00, 1.0),
                text="CYAN | CLEANING AREA",
            )
        )

    for index, polygon in enumerate(mission.get("keepout_polygons", [])):
        points = closed_polyline(polygon, 0.055)
        specs.append(
            MarkerSpec(
                namespace="zones",
                key=f"keepout_{index}",
                kind="line_strip",
                scale=(0.10, 0.0, 0.0),
                color=(1.0, 0.08, 0.08, 1.0),
                points=points,
            )
        )
        if polygon:
            center_x = sum(float(point[0]) for point in polygon) / len(polygon)
            center_y = sum(float(point[1]) for point in polygon) / len(polygon)
            specs.append(
                MarkerSpec(
                    namespace="zone_labels",
                    key=f"keepout_{index}",
                    kind="text",
                    position=(center_x, center_y, 0.30),
                    scale=(0.0, 0.0, 0.28),
                    color=(1.0, 0.18, 0.18, 1.0),
                    text="KEEP OUT",
                )
            )

    for model_name, scene_spec in scene.get("objects", {}).items():
        registry_spec = registry["models"][model_name]
        x, y, z, yaw = (float(value) for value in scene_spec["pose_world"])
        size = tuple(float(value) for value in registry_spec["size_m"])
        specs.extend(
            target_specs(
                key=model_name,
                class_id=str(registry_spec["class_id"]),
                position=(x + tx, y + ty, z),
                size=size,
                yaw=yaw,
                cleaned=str(registry_spec["uuid"]) in cleaned_uuids,
                label_offset=TARGET_LABEL_OFFSETS.get(model_name, (0.0, 0.0)),
            )
        )

    for model_name, scene_spec in scene.get("negative_objects", {}).items():
        x, y = (float(value) for value in scene_spec["pose_world"])
        radius = float(scene_spec["radius_m"])
        map_x, map_y = x + tx, y + ty
        label = NEGATIVE_LABELS.get(model_name, f"NON-TARGET | {model_name}")
        label_offset = OBSTACLE_LABEL_OFFSETS.get(model_name, (0.0, 0.55))
        specs.extend(
            [
                MarkerSpec(
                    namespace="obstacles",
                    key=f"{model_name}_halo",
                    kind="cylinder",
                    position=(map_x, map_y, 0.035),
                    scale=(2.0 * radius, 2.0 * radius, 0.07),
                    color=(0.92, 0.12, 0.75, 0.30),
                ),
                MarkerSpec(
                    namespace="obstacle_labels",
                    key=f"{model_name}_label",
                    kind="text",
                    position=(
                        map_x + label_offset[0],
                        map_y + label_offset[1],
                        0.42,
                    ),
                    scale=(0.0, 0.0, 0.26),
                    color=(1.0, 0.48, 0.88, 1.0),
                    text=label,
                ),
            ]
        )
    return specs


def predicted_specs(targets: list[dict], cleaned_uuids: set[str] | None = None) -> list[MarkerSpec]:
    cleaned_uuids = cleaned_uuids or set()
    specs: list[MarkerSpec] = []
    for target in targets:
        position = tuple(float(value) for value in target["position"])
        size = tuple(float(value) for value in target["size"])
        specs.extend(
            target_specs(
                key=str(target["uuid"]),
                class_id=str(target["class_id"]),
                position=position,
                size=size,
                yaw=float(target.get("yaw", 0.0)),
                prefix="PRED",
                confidence=float(target["confidence"]),
                cleaned=str(target["uuid"]) in cleaned_uuids,
            )
        )
    return specs


def status_text(
    *,
    prediction_count: int,
    truth_visible_count: int,
    cleaned_count: int,
    brush_enabled: bool,
    coverage_state: str,
    spot_state: str,
) -> str:
    brush = "ON" if brush_enabled else "OFF"
    return "\n".join(
        (
            "TZcup DEBUG",
            f"PRED: {prediction_count} | VISIBLE GT: {truth_visible_count}",
            f"CLEANED: {cleaned_count} | BRUSH: {brush}",
            f"COVERAGE: {coverage_state}",
            f"SPOT CLEAN: {spot_state}",
        )
    )


def vehicle_specs(x: float, y: float, yaw: float) -> list[MarkerSpec]:
    return [
        MarkerSpec(
            namespace="vehicle",
            key="footprint",
            kind="cube",
            position=(x, y, 0.045),
            yaw=yaw,
            scale=(0.80, 0.72, 0.09),
            color=(0.98, 0.72, 0.08, 0.45),
        ),
        MarkerSpec(
            namespace="vehicle",
            key="heading",
            kind="arrow",
            position=(x, y, 0.15),
            yaw=yaw,
            scale=(0.78, 0.11, 0.11),
            color=(1.0, 0.82, 0.12, 1.0),
        ),
        MarkerSpec(
            namespace="vehicle",
            key="label",
            kind="text",
            position=(x, y, 0.46),
            scale=(0.0, 0.0, 0.28),
            color=(1.0, 0.92, 0.35, 1.0),
            text="VEHICLE",
        ),
    ]


def transform_specs_to_vehicle(
    specs: list[MarkerSpec],
    vehicle_x: float,
    vehicle_y: float,
    vehicle_yaw: float,
) -> list[MarkerSpec]:
    cosine = math.cos(vehicle_yaw)
    sine = math.sin(vehicle_yaw)

    def transform_point(point: Point) -> Point:
        dx = point[0] - vehicle_x
        dy = point[1] - vehicle_y
        return (
            cosine * dx + sine * dy,
            -sine * dx + cosine * dy,
            point[2],
        )

    return [
        replace(
            spec,
            position=transform_point(spec.position),
            yaw=spec.yaw - vehicle_yaw,
            points=tuple(transform_point(point) for point in spec.points),
        )
        for spec in specs
    ]


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
