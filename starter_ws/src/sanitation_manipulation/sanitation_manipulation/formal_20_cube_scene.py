"""Strict evaluator-side scene contract for the formal 20-cube gate.

This module may read material and simulator model names because it is used only
by the launch/evaluator boundary.  Product ROS requests are separately reduced
to the truth-free v2 perception contract by the runtime probe.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Any


MATERIAL_MASS_KG = {
    "paperboard": 0.0189,
    "PP": 0.0243,
    "PET": 0.03726,
    "aluminum": 0.0729,
}


@dataclass(frozen=True)
class CubeSpawnSpec:
    target_id: str
    model_name: str
    material: str
    x_m: float
    y_m: float
    z_m: float
    yaw_rad: float
    color_rgb: tuple[float, float, float]
    mass_kg: float


def _number(value: Any, label: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} must be numeric") from exc
    if not math.isfinite(result):
        raise ValueError(f"{label} must be finite")
    return result


def load_scene_manifest(path: Path) -> tuple[dict[str, Any], tuple[CubeSpawnSpec, ...]]:
    try:
        manifest = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise ValueError(f"cannot read 20-cube manifest {path}: {exc}") from exc
    if not isinstance(manifest, dict):
        raise ValueError("20-cube manifest root must be an object")
    requests = manifest.get("requests")
    if manifest.get("task_count") != 20 or not isinstance(requests, list) or len(requests) != 20:
        raise ValueError("20-cube scene requires exactly 20 requests")
    scene = manifest.get("scene_contract")
    if not isinstance(scene, dict) or scene.get("world_name") != "formal_cube_manipulation":
        raise ValueError("20-cube scene world contract is missing or invalid")
    if scene.get("vehicle_model_name") != "tzcup_formal_sanitation_vehicle":
        raise ValueError("20-cube vehicle model contract is invalid")
    if scene.get("physical_rigid_bodies_retained_after_deposit") is not True:
        raise ValueError("20-cube rigid bodies must remain physical after deposit")
    accounting = manifest.get("dry_payload_accounting")
    if (
        not isinstance(accounting, dict)
        or accounting.get("mode") != "physical_resident"
        or accounting.get("aggregate_dry_mass_must_remain_kg") != 0.0
        or accounting.get("nonzero_aggregate_input_rejected") is not True
        or accounting.get("load_transfer") != "independent_rigid_bodies_contact"
    ):
        raise ValueError("20-cube dry payload accounting must be physical_resident")
    runtime = manifest.get("runtime_requirements")
    if not isinstance(runtime, dict):
        raise ValueError("20-cube runtime requirements are missing")
    if runtime.get("maximum_attempts_per_target") != 2:
        raise ValueError("20-cube targets must allow at most two attempts")
    if runtime.get("retry_requires_safe_transport_restored") is not True:
        raise ValueError("20-cube retry must require a safely restored transport pose")
    if runtime.get("retry_requires_unchanged_evaluator_payload") is not True:
        raise ValueError("20-cube retry must require an unchanged evaluator payload")
    if runtime.get("duplicate_payload_accounting_forbidden") is not True:
        raise ValueError("20-cube duplicate payload accounting must be forbidden")

    capacity = manifest.get("dry_bin_capacity_contract")
    if not isinstance(capacity, dict):
        raise ValueError("20-cube dry-bin capacity contract is missing")
    if (
        capacity.get("maximum_count") != 20
        or capacity.get("grid_rows") != 4
        or capacity.get("grid_columns") != 5
        or capacity.get("single_layer") is not True
        or capacity.get("stacking_allowed") is not False
        or abs(_number(capacity.get("cube_edge_m"), "cube edge") - 0.03) > 1.0e-9
    ):
        raise ValueError("20-cube scene must remain a 5x4 single-layer grid")
    grid_x_pitch = _number(capacity.get("grid_x_pitch_m"), "grid x pitch")
    grid_y_pitch = _number(capacity.get("grid_y_pitch_m"), "grid y pitch")

    specs: list[CubeSpawnSpec] = []
    positions: list[tuple[float, float, float]] = []
    slot_positions: dict[tuple[int, int], tuple[float, float]] = {}
    for index, request in enumerate(requests, start=1):
        if not isinstance(request, dict) or request.get("schema_version") != 2:
            raise ValueError(f"request {index} is not a v2 perception contract")
        if request.get("truth_used") is not False or request.get("material") != "unknown":
            raise ValueError(f"request {index} leaks evaluator truth")
        if request.get("size_m") != [0.03, 0.03, 0.03]:
            raise ValueError(f"request {index} is not a physical 30 mm cube")
        pose = request.get("pose")
        acceptance = request.get("acceptance")
        if not isinstance(pose, dict) or not isinstance(acceptance, dict):
            raise ValueError(f"request {index} lacks pose or evaluator acceptance data")
        model_name = acceptance.get("scene_model_name")
        expected_name = f"object_{index:02d}"
        if model_name != expected_name:
            raise ValueError(f"request {index} model name must equal {expected_name}")
        material = acceptance.get("actual_material_evaluator_only")
        if material not in MATERIAL_MASS_KG:
            raise ValueError(f"request {index} has unsupported material {material}")
        expected_mass = _number(acceptance.get("expected_increment_kg"), "expected mass")
        if abs(expected_mass - MATERIAL_MASS_KG[str(material)]) > 1.0e-9:
            raise ValueError(f"request {index} material mass is inconsistent")
        x_m = _number(pose.get("x_m"), "pose.x_m")
        y_m = _number(pose.get("y_m"), "pose.y_m")
        z_m = _number(pose.get("z_m"), "pose.z_m")
        qx = _number(pose.get("qx"), "pose.qx")
        qy = _number(pose.get("qy"), "pose.qy")
        qz = _number(pose.get("qz"), "pose.qz")
        qw = _number(pose.get("qw"), "pose.qw")
        if abs(qx) > 1.0e-9 or abs(qy) > 1.0e-9 or abs(qz * qz + qw * qw - 1.0) > 1.0e-6:
            raise ValueError(f"request {index} must have a normalized planar orientation")
        if abs(z_m - 0.015) > 1.0e-9:
            raise ValueError(f"request {index} is not in the single ground layer")
        raw_color = acceptance.get("random_color_rgb_evaluator_only")
        if not isinstance(raw_color, list) or len(raw_color) != 3:
            raise ValueError(f"request {index} has no independent random RGB colour")
        color = tuple(_number(value, "cube colour") for value in raw_color)
        if any(value < 0.0 or value > 1.0 for value in color):
            raise ValueError(f"request {index} colour is outside [0, 1]")
        slot = acceptance.get("single_layer_slot")
        if not isinstance(slot, dict):
            raise ValueError(f"request {index} has no 5x4 single-layer slot")
        row = slot.get("row")
        column = slot.get("column")
        if (
            isinstance(row, bool)
            or not isinstance(row, int)
            or isinstance(column, bool)
            or not isinstance(column, int)
            or not 0 <= row < 4
            or not 0 <= column < 5
            or (row, column) in slot_positions
        ):
            raise ValueError(f"request {index} has an invalid or duplicate 5x4 slot")
        slot_positions[(row, column)] = (x_m, y_m)
        positions.append((x_m, y_m, z_m))
        specs.append(
            CubeSpawnSpec(
                target_id=str(request.get("target_id", "")),
                model_name=str(model_name),
                material=str(material),
                x_m=x_m,
                y_m=y_m,
                z_m=z_m,
                yaw_rad=2.0 * math.atan2(qz, qw),
                color_rgb=color,  # type: ignore[arg-type]
                mass_kg=expected_mass,
            )
        )
    if len({spec.target_id for spec in specs}) != 20 or any(not spec.target_id for spec in specs):
        raise ValueError("20-cube target ids must be non-empty and unique")
    if len({spec.model_name for spec in specs}) != 20 or len(set(positions)) != 20:
        raise ValueError("20-cube model names and single-layer positions must be unique")
    if set(slot_positions) != {(row, column) for row in range(4) for column in range(5)}:
        raise ValueError("20-cube scene does not fill the complete 5x4 slot grid")
    for row in range(4):
        for column in range(5):
            x_m, y_m = slot_positions[(row, column)]
            if column < 4:
                next_x, next_y = slot_positions[(row, column + 1)]
                if abs(next_x - x_m - grid_x_pitch) > 1.0e-9 or abs(next_y - y_m) > 1.0e-9:
                    raise ValueError("20-cube x slots do not match the declared grid pitch")
            if row < 3:
                next_x, next_y = slot_positions[(row + 1, column)]
                if abs(next_x - x_m) > 1.0e-9 or abs(next_y - y_m - grid_y_pitch) > 1.0e-9:
                    raise ValueError("20-cube y slots do not match the declared grid pitch")
    minimum_clearance = _number(
        capacity.get("minimum_inter_cube_spacing_m"),
        "minimum inter-cube spacing",
    )
    minimum_center_distance = math.sqrt(2.0) * 0.03 + minimum_clearance
    for first_index, first in enumerate(positions):
        for second in positions[first_index + 1 :]:
            if math.hypot(first[0] - second[0], first[1] - second[1]) + 1.0e-12 < minimum_center_distance:
                raise ValueError("20-cube single-layer positions overlap their rotated envelopes")
    counts = {material: 0 for material in MATERIAL_MASS_KG}
    for spec in specs:
        counts[spec.material] += 1
    if set(counts.values()) != {5}:
        raise ValueError(f"20-cube material distribution must be 5 each: {counts}")
    expected_total = sum(spec.mass_kg for spec in specs)
    if abs(_number(manifest.get("expected_final_physical_resident_mass_kg"), "final mass") - expected_total) > 1.0e-9:
        raise ValueError("20-cube final physical-resident mass does not match rigid-body inertials")
    if _number(manifest.get("expected_final_aggregate_dry_mass_kg"), "aggregate dry mass") != 0.0:
        raise ValueError("20-cube physical-resident scene must not carry aggregate dry mass")
    return manifest, tuple(specs)
