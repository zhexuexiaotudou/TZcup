"""Dependency-free mechanism-state to navigation-footprint selection."""

from __future__ import annotations

import math
import struct
from collections.abc import Callable, Mapping, Sequence
from pathlib import Path
from typing import Any, TypeVar

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
_T = TypeVar("_T")


def run_with_fail_closed_cleanup(
    operation: Callable[[], _T], cleanup: Callable[[], None]
) -> _T:
    """Run cleanup once without replacing an earlier operation failure.

    A cleanup failure remains fatal after a successful operation.  When both
    fail, the primary operation exception keeps its traceback and receives the
    cleanup failure as an exception note for live-gate diagnosis.
    """
    try:
        result = operation()
    except BaseException as primary_error:
        try:
            cleanup()
        except BaseException as cleanup_error:
            primary_error.add_note(
                "fail-closed cleanup also failed: "
                f"{type(cleanup_error).__name__}: {cleanup_error}"
            )
        raise
    cleanup()
    return result


def profile_decision(
    joints: Mapping[str, float],
    base_motion_inhibited: bool,
    *,
    arm_tolerance_rad: float = 0.08,
    cleaning_work_position_m: float = 0.100,
    cleaning_tolerance_m: float = 0.005,
) -> tuple[str, str]:
    """Select the conservative envelope and retain the selection reason."""
    if base_motion_inhibited:
        return "arm_deployed", "base_motion_inhibited"
    arm_known = all(name in joints for name in ARM_JOINTS)
    arm_stowed = arm_known and all(
        math.isfinite(joints[name])
        and abs(joints[name] - expected) <= arm_tolerance_rad
        for name, expected in zip(ARM_JOINTS, ARM_STOWED, strict=True)
    )
    if not arm_stowed:
        return "arm_deployed", "arm_state_unknown_or_not_stowed"
    lift = joints.get("cleaning_lift_joint")
    if lift is None or not math.isfinite(lift):
        # The brush may be deployed even while the arm remains stowed.  Do not
        # shrink to transport until an actual finite lift observation arrives.
        return "cleaning_deployed", "cleaning_lift_unknown_or_nonfinite"
    if (
        abs(lift - cleaning_work_position_m) <= cleaning_tolerance_m
    ):
        return "cleaning_deployed", "cleaning_lift_at_work_position"
    return "transport_stowed", "transport_stowed"


def select_profile(
    joints: Mapping[str, float],
    base_motion_inhibited: bool,
    *,
    arm_tolerance_rad: float = 0.08,
    cleaning_work_position_m: float = 0.100,
    cleaning_tolerance_m: float = 0.005,
) -> str:
    return profile_decision(
        joints,
        base_motion_inhibited,
        arm_tolerance_rad=arm_tolerance_rad,
        cleaning_work_position_m=cleaning_work_position_m,
        cleaning_tolerance_m=cleaning_tolerance_m,
    )[0]


def normalize_exact_polygon(points: Sequence[Any]) -> tuple[tuple[float, float], ...]:
    """Return the exact float32 wire representation of a footprint polygon.

    Nav2 consumes ``geometry_msgs/msg/Polygon`` whose Point32 coordinates are
    serialized as IEEE-754 binary32 values.  Config YAML and readback messages
    therefore need this one shared normalization before an exact comparison;
    an arbitrary epsilon would hide a changed polygon.
    """
    if not isinstance(points, Sequence) or len(points) < 3:
        raise ValueError("footprint polygon must contain at least three points")
    normalized: list[tuple[float, float]] = []
    for point in points:
        if hasattr(point, "x") and hasattr(point, "y"):
            raw_x, raw_y = point.x, point.y
        elif isinstance(point, Sequence) and len(point) == 2:
            raw_x, raw_y = point
        else:
            raise ValueError("footprint polygon point must have exactly two coordinates")
        x, y = float(raw_x), float(raw_y)
        if not math.isfinite(x) or not math.isfinite(y):
            raise ValueError("footprint polygon contains non-finite coordinates")
        normalized.append(
            (
                struct.unpack("!f", struct.pack("!f", x))[0],
                struct.unpack("!f", struct.pack("!f", y))[0],
            )
        )
    return tuple(normalized)


def polygons_exactly_equal(actual: Sequence[Any], expected: Sequence[Any]) -> bool:
    """Compare polygons by ordered Point32 wire values, without an epsilon."""
    return normalize_exact_polygon(actual) == normalize_exact_polygon(expected)


def load_footprints(path: Path) -> dict[str, list[list[float]]]:
    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    profiles = payload.get("motion_footprints", {}) if isinstance(payload, dict) else {}
    result: dict[str, list[list[float]]] = {}
    for name in ("transport_stowed", "cleaning_deployed", "arm_deployed"):
        row = profiles.get(name, {})
        points = row.get("footprint_xy_m", []) if isinstance(row, dict) else []
        if not isinstance(points, Sequence) or len(points) < 3:
            raise ValueError(f"motion footprint {name} is invalid")
        # Validate through the same exact-comparison grammar used by the ROS
        # runtime gate, while retaining the frozen YAML values for publication.
        normalize_exact_polygon(points)
        result[name] = [[float(point[0]), float(point[1])] for point in points]
    return result
