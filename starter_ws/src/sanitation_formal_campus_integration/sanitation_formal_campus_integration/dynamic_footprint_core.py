"""Dependency-free mechanism-state to navigation-footprint selection."""

from __future__ import annotations

import json
import math
import os
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


def float32_ulp(value: float) -> float:
    """Return one IEEE-754 binary32 ULP at ``value`` (including zero)."""
    wire = struct.unpack("!f", struct.pack("!f", float(value)))[0]
    if not math.isfinite(wire):
        raise ValueError("Point32 ULP requires a finite coordinate")
    bits = struct.unpack("!I", struct.pack("!f", wire))[0]
    if bits == 0x80000000:
        bits = 0
    next_bits = bits + 1 if wire >= 0.0 else bits - 1
    next_wire = struct.unpack("!f", struct.pack("!I", next_bits))[0]
    return abs(next_wire - wire)


def float64_ulp(value: float) -> float:
    """Return one IEEE-754 binary64 ULP at ``value`` (including zero)."""
    wire = float(value)
    if not math.isfinite(wire):
        raise ValueError("float64 ULP requires a finite coordinate")
    bits = struct.unpack("!Q", struct.pack("!d", wire))[0]
    if bits == 0x8000000000000000:
        bits = 0
    next_bits = bits + 1 if wire >= 0.0 else bits - 1
    next_wire = struct.unpack("!d", struct.pack("!Q", next_bits))[0]
    return abs(next_wire - wire)


def float64_zero_ulp_bound() -> float:
    """Two binary64 ULPs at zero, for exact planar-frame equivalence."""
    return 2.0 * float64_ulp(0.0)


def point32_quantization_bound(*coordinates: float) -> float:
    """Bound two Point32 round trips by two ULPs at the largest magnitude."""
    if not coordinates:
        raise ValueError("Point32 bound requires coordinates")
    return 2.0 * max(float32_ulp(float(value)) for value in coordinates)


def point32_coordinate_quantization_bound(expected: float, observed: float) -> float:
    """Bound one coordinate only; polygon-wide maxima must not mask an error."""
    return 2.0 * max(float32_ulp(float(expected)), float32_ulp(float(observed)))


def fresh_nonzero_stamp(
    stamp_ns: int, baseline_stamp_ns: int, now_ros_ns: int, max_age_ns: int = 2_000_000_000
) -> tuple[bool, str]:
    """Accept only a nonzero, advancing, non-future ROS simulation stamp."""
    if not isinstance(stamp_ns, int) or stamp_ns <= 0:
        return False, "published_stamp_missing_or_zero"
    if not isinstance(baseline_stamp_ns, int) or stamp_ns <= baseline_stamp_ns:
        return False, "published_stamp_not_fresh"
    if not isinstance(now_ros_ns, int) or now_ros_ns <= 0 or stamp_ns > now_ros_ns:
        return False, "published_stamp_future_or_clock_unavailable"
    if not isinstance(max_age_ns, int) or max_age_ns <= 0 or now_ros_ns - stamp_ns > max_age_ns:
        return False, "published_stamp_too_old"
    return True, "ok"


def padded_polygon(points: Sequence[Any], padding_m: float) -> tuple[tuple[float, float], ...]:
    """Mirror Nav2's axis-sign footprint padding without changing point order."""
    padding = float(padding_m)
    if not math.isfinite(padding) or padding < 0.0:
        raise ValueError("footprint padding must be finite and non-negative")
    padded: list[tuple[float, float]] = []
    for x, y in normalize_exact_polygon(points):
        padded.append(
            (
                x + math.copysign(padding, x) if x else x,
                y + math.copysign(padding, y) if y else y,
            )
        )
    return tuple(padded)


def rigid_transform_polygon(
    points: Sequence[Any], translation_x_m: float, translation_y_m: float, yaw_rad: float
) -> tuple[tuple[float, float], ...]:
    """Apply the planar base-link-to-costmap rigid transform in point order."""
    tx, ty, yaw = float(translation_x_m), float(translation_y_m), float(yaw_rad)
    if not all(math.isfinite(value) for value in (tx, ty, yaw)):
        raise ValueError("rigid transform must be finite")
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return tuple(
        (tx + cosine * x - sine * y, ty + sine * x + cosine * y)
        for x, y in normalize_exact_polygon(points)
    )


def padded_rigid_point32_match(
    actual: Sequence[Any],
    profile: Sequence[Any],
    padding_m: float,
    translation_x_m: float,
    translation_y_m: float,
    yaw_rad: float,
) -> tuple[bool, float, str]:
    """Compare ordered Nav2 readback to the padded profile with a ULP-only bound.

    This deliberately has no caller-selected epsilon.  It rejects a changed
    point count/order, mirrors, shears, excess geometry, or a wrong padding.
    """
    expected = rigid_transform_polygon(
        padded_polygon(profile, padding_m), translation_x_m, translation_y_m, yaw_rad
    )
    try:
        observed = normalize_exact_polygon(actual)
    except ValueError as error:
        return False, 0.0, f"invalid_published_polygon:{error}"
    if len(observed) != len(expected):
        return False, 0.0, "published_point_count_mismatch"
    max_bound = 0.0
    for index, ((actual_x, actual_y), (expected_x, expected_y)) in enumerate(
        zip(observed, expected, strict=True)
    ):
        x_bound = point32_coordinate_quantization_bound(expected_x, actual_x)
        y_bound = point32_coordinate_quantization_bound(expected_y, actual_y)
        max_bound = max(max_bound, x_bound, y_bound)
        if abs(actual_x - expected_x) > x_bound or abs(actual_y - expected_y) > y_bound:
            return False, max_bound, f"published_rigid_padding_mismatch_point_{index}"
    return True, max_bound, "ok"


def atomic_write_fresh_json(path: Path, payload: Mapping[str, Any]) -> None:
    """Atomically create one fresh, non-link JSON evidence file."""
    if path.exists() or path.is_symlink() or not path.parent.is_dir() or path.parent.is_symlink():
        raise RuntimeError(f"output must be a fresh path in a regular directory: {path}")
    pending = path.with_name(f".{path.name}.{os.getpid()}.pending")
    if pending.exists() or pending.is_symlink():
        raise RuntimeError(f"refusing pre-existing output staging path: {pending}")
    descriptor = os.open(str(pending), os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(payload, stream, indent=2, sort_keys=True)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        if pending.is_symlink() or not pending.is_file():
            raise RuntimeError("output staging path became non-regular")
        os.replace(pending, path)
    except BaseException:
        if pending.exists() and pending.is_file() and not pending.is_symlink():
            pending.unlink()
        raise


def blocked_runtime_gate_shape(
    reason: str, input_topics: Sequence[str], published_topics: Sequence[str]
) -> dict[str, Any]:
    """Return the complete no-node BLOCKED evidence shape without ROS imports."""
    return {
        "result": "BLOCKED",
        "passed": False,
        "runtime_only": True,
        "reason": reason,
        "last_input": {topic: {"receipt": 0, "polygon": None} for topic in input_topics},
        "last_published": {
            topic: {"receipt": 0, "polygon": None} for topic in published_topics
        },
        "last_status": None,
        "last_safety": None,
        "receipt_counters": {
            "input": {topic: 0 for topic in input_topics},
            "published": {topic: 0 for topic in published_topics},
            "status": 0,
            "safety": 0,
        },
        "declared_footprint_padding_m": None,
        "live_footprint_padding_m": {},
        "footprint_padding_quantization_bound_m": 0.0,
        "profile_base_frame": None,
        "profile_to_robot_base_planar_equivalence": {},
        "point32_quantization_bound_m": 0.0,
        "fresh_readback_required_per_override": True,
    }


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


def load_nav2_footprint_padding(path: Path) -> float:
    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    value = payload.get("nav2_footprint_padding_m") if isinstance(payload, dict) else None
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        raise ValueError("formal profile lacks numeric nav2_footprint_padding_m")
    padding = float(value)
    if not math.isfinite(padding) or padding < 0.0:
        raise ValueError("formal profile nav2_footprint_padding_m is invalid")
    return padding


def load_profile_base_frame(path: Path) -> str:
    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    frame = payload.get("base_frame") if isinstance(payload, dict) else None
    if not isinstance(frame, str) or not frame or frame.startswith("/"):
        raise ValueError("formal profile lacks a relative base_frame")
    return frame
