"""Dependency-free mechanism-state to navigation-footprint selection."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Mapping, Sequence

import yaml


ARM_JOINTS = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)
ARM_STOWED = (-1.0, -1.0, 1.8, -1.5, -1.55, 0.25)


def select_profile(
    joints: Mapping[str, float],
    base_motion_inhibited: bool,
    *,
    arm_tolerance_rad: float = 0.08,
    cleaning_work_position_m: float = 0.100,
    cleaning_tolerance_m: float = 0.005,
) -> str:
    arm_known = all(name in joints for name in ARM_JOINTS)
    arm_stowed = arm_known and all(
        math.isfinite(joints[name])
        and abs(joints[name] - expected) <= arm_tolerance_rad
        for name, expected in zip(ARM_JOINTS, ARM_STOWED, strict=True)
    )
    if base_motion_inhibited or not arm_stowed:
        return "arm_deployed"
    lift = joints.get("cleaning_lift_joint")
    if (
        lift is not None
        and math.isfinite(lift)
        and abs(lift - cleaning_work_position_m) <= cleaning_tolerance_m
    ):
        return "cleaning_deployed"
    return "transport_stowed"


def load_footprints(path: Path) -> dict[str, list[list[float]]]:
    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    profiles = payload.get("motion_footprints", {}) if isinstance(payload, dict) else {}
    result: dict[str, list[list[float]]] = {}
    for name in ("transport_stowed", "cleaning_deployed", "arm_deployed"):
        row = profiles.get(name, {})
        points = row.get("footprint_xy_m", []) if isinstance(row, dict) else []
        if not isinstance(points, Sequence) or len(points) < 3:
            raise ValueError(f"motion footprint {name} is invalid")
        normalized = [[float(point[0]), float(point[1])] for point in points]
        if any(
            len(point) != 2 or not all(math.isfinite(value) for value in point)
            for point in normalized
        ):
            raise ValueError(f"motion footprint {name} contains invalid coordinates")
        result[name] = normalized
    return result
