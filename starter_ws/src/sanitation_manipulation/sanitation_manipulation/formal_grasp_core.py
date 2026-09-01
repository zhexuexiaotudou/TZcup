"""Dependency-free contracts for target-conditioned formal grasp planning.

The public request is a perception observation of one physical cube, including
its complete 3-D pose, measured dimensions and material class. Simulator entity
identities remain forbidden. ROS and MoveIt message construction stays in
``formal_grasp_executor`` so this module is testable without ROS.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from typing import Any, Mapping


ARM_JOINTS = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)
GRIPPER_JOINT = "robotiq_85_left_knuckle_joint"
STORAGE_JOINTS = ("dry_deposit_gate_joint",)

# This is only a collision-scanned safe configuration. Target approach, pick,
# lift, deposit and retreat are generated online through MoveIt.
TRANSPORT = (-1.0, -1.0, 1.8, -1.5, -1.55, 0.25)
PICK_WINDOW_X_M = 0.300
PICK_WINDOW_Y_M = -0.950

MATERIAL_MASS_KG = {
    "paperboard": 0.0189,
    "PP": 0.0243,
    "PET": 0.03726,
    "aluminum": 0.0729,
}
MATERIAL_MASSES_KG = tuple(MATERIAL_MASS_KG.values())


def _finite(value: Any, name: str) -> float:
    converted = float(value)
    if not math.isfinite(converted):
        raise ValueError(f"{name} must be finite")
    return converted


def _normalized_quaternion(
    values: tuple[float, float, float, float], *, name: str
) -> tuple[float, float, float, float]:
    if not all(math.isfinite(value) for value in values):
        raise ValueError(f"{name} must be finite")
    norm = math.sqrt(sum(value * value for value in values))
    if norm < 1.0e-9:
        raise ValueError(f"{name} must be non-zero")
    if abs(norm - 1.0) > 0.02:
        raise ValueError(f"{name} must be normalized")
    return tuple(value / norm for value in values)  # type: ignore[return-value]


@dataclass(frozen=True)
class TargetGeometry:
    frame_id: str
    x_m: float
    y_m: float
    z_m: float
    qx: float
    qy: float
    qz: float
    qw: float
    size_x_m: float
    size_y_m: float
    size_z_m: float
    material: str

    @property
    def size_m(self) -> tuple[float, float, float]:
        return (self.size_x_m, self.size_y_m, self.size_z_m)


@dataclass(frozen=True)
class GraspRequest:
    target_id: str
    geometry: TargetGeometry
    confidence: float

    @property
    def frame_id(self) -> str:
        return self.geometry.frame_id

    @property
    def x_m(self) -> float:
        return self.geometry.x_m

    @property
    def y_m(self) -> float:
        return self.geometry.y_m

    @classmethod
    def from_json(cls, encoded: str) -> "GraspRequest":
        try:
            raw = json.loads(encoded)
        except json.JSONDecodeError as exc:
            raise ValueError("grasp request is not valid JSON") from exc
        if not isinstance(raw, Mapping):
            raise ValueError("grasp request must be a JSON object")
        required = {
            "schema_version", "target_id", "frame_id", "pose", "size_m",
            "confidence", "truth_used",
        }
        if set(raw) not in (required, required | {"material"}):
            raise ValueError(
                "grasp request keys must equal the v2 perception contract, "
                "with optional material='unknown'"
            )
        if raw["schema_version"] != 2:
            raise ValueError("grasp request schema_version must equal 2")
        if raw["truth_used"] is not False:
            raise ValueError("truth-backed grasp requests are forbidden")
        target_id = raw["target_id"]
        frame_id = raw["frame_id"]
        if not isinstance(target_id, str) or not target_id.strip():
            raise ValueError("target_id must be a non-empty perception track id")
        if not isinstance(frame_id, str) or not frame_id.strip():
            raise ValueError("frame_id must be non-empty")
        if any(token in raw for token in ("model_name", "entity_name", "gazebo_id")):
            raise ValueError("simulator entity identity is forbidden in the product request")

        pose = raw["pose"]
        if not isinstance(pose, Mapping):
            raise ValueError("pose must be a JSON object")
        pose_keys = {"x_m", "y_m", "z_m", "qx", "qy", "qz", "qw"}
        if set(pose) != pose_keys:
            raise ValueError(f"pose keys must equal {sorted(pose_keys)}")
        position = tuple(_finite(pose[name], f"pose.{name}") for name in ("x_m", "y_m", "z_m"))
        quaternion = _normalized_quaternion(
            tuple(_finite(pose[name], f"pose.{name}") for name in ("qx", "qy", "qz", "qw")),
            name="pose quaternion",
        )
        size_raw = raw["size_m"]
        if not isinstance(size_raw, (list, tuple)) or len(size_raw) != 3:
            raise ValueError("size_m must contain exactly three dimensions")
        size = tuple(_finite(value, "size_m") for value in size_raw)
        if any(value < 0.020 or value > 0.040 for value in size):
            raise ValueError("size_m must remain within the physical 30 mm cube tolerance")
        # Random colour does not reveal paperboard/PP/PET/aluminum. Product
        # control therefore accepts only unknown; the bin load cell classifies
        # the allowed material mass after physical release.
        material = raw.get("material", "unknown")
        if material != "unknown":
            raise ValueError("pre-grasp material must be unknown, never inferred from colour")
        confidence = _finite(raw["confidence"], "confidence")
        if not 0.0 <= confidence <= 1.0:
            raise ValueError("confidence must be in [0, 1]")
        if confidence < 0.50:
            raise ValueError("perception confidence is below the grasp threshold")
        return cls(
            target_id=target_id,
            geometry=TargetGeometry(
                frame_id=frame_id,
                x_m=position[0], y_m=position[1], z_m=position[2],
                qx=quaternion[0], qy=quaternion[1], qz=quaternion[2], qw=quaternion[3],
                size_x_m=size[0], size_y_m=size[1], size_z_m=size[2],
                material=str(material),
            ),
            confidence=confidence,
        )


@dataclass(frozen=True)
class ToolPose:
    x_m: float
    y_m: float
    z_m: float
    qx: float
    qy: float
    qz: float
    qw: float


@dataclass(frozen=True)
class GraspTaskWaypoints:
    pregrasp: ToolPose
    pick: ToolPose
    lift: ToolPose
    deposit: ToolPose
    retreat: ToolPose


def target_yaw(geometry: TargetGeometry) -> float:
    return math.atan2(
        2.0 * (geometry.qw * geometry.qz + geometry.qx * geometry.qy),
        1.0 - 2.0 * (geometry.qy * geometry.qy + geometry.qz * geometry.qz),
    )


def build_target_conditioned_waypoints(
    geometry: TargetGeometry,
    *,
    tool_to_cube_center_m: float = 0.173,
    approach_distance_m: float = 0.260,
    lift_distance_m: float = 0.260,
) -> GraspTaskWaypoints:
    """Create top-down poses from the measured target, never fixed pick joints."""

    values = (*geometry.size_m, tool_to_cube_center_m, approach_distance_m, lift_distance_m)
    if not all(math.isfinite(value) and value > 0.0 for value in values):
        raise ValueError("grasp waypoint dimensions and offsets must be positive and finite")
    yaw = target_yaw(geometry)
    # Rz(yaw) * Rx(pi), matching the scanned formal UR5e top-down tool pose.
    qx, qy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    pick_z = geometry.z_m + tool_to_cube_center_m
    pick = ToolPose(geometry.x_m, geometry.y_m, pick_z, qx, qy, 0.0, 0.0)
    pregrasp = ToolPose(geometry.x_m, geometry.y_m, pick_z + approach_distance_m, qx, qy, 0.0, 0.0)
    lift = ToolPose(geometry.x_m, geometry.y_m, pick_z + lift_distance_m, qx, qy, 0.0, 0.0)
    # Bin geometry gives a fixed Cartesian goal, but MoveIt plans and checks
    # the path online from the target-conditioned lift.
    deposit = ToolPose(-0.205, 0.035, 1.260, 0.500, -0.500, 0.500, 0.500)
    retreat = ToolPose(-0.205, 0.035, 1.430, 0.500, -0.500, 0.500, 0.500)
    return GraspTaskWaypoints(pregrasp, pick, lift, deposit, retreat)


def validate_wrist_recheck(
    original: GraspRequest,
    refined: GraspRequest,
    *,
    maximum_position_delta_m: float = 0.080,
    maximum_size_delta_m: float = 0.008,
) -> tuple[bool, str]:
    if original.target_id != refined.target_id:
        return False, "wrist_recheck_target_mismatch"
    if original.geometry.material != refined.geometry.material:
        return False, "wrist_recheck_material_mismatch"
    position_delta = math.sqrt(
        (original.geometry.x_m - refined.geometry.x_m) ** 2
        + (original.geometry.y_m - refined.geometry.y_m) ** 2
        + (original.geometry.z_m - refined.geometry.z_m) ** 2
    )
    if position_delta > maximum_position_delta_m:
        return False, "wrist_recheck_position_disagreement"
    if any(abs(a - b) > maximum_size_delta_m for a, b in zip(original.geometry.size_m, refined.geometry.size_m)):
        return False, "wrist_recheck_size_disagreement"
    return True, "wrist_near_field_target_reconfirmed"


@dataclass(frozen=True)
class ParkingObservation:
    target_x_base_m: float
    target_y_base_m: float
    linear_speed_m_s: float
    angular_speed_rad_s: float
    transform_age_s: float
    odometry_age_s: float

    def validate(
        self,
        *,
        position_tolerance_m: float = 0.075,
        maximum_linear_speed_m_s: float = 0.015,
        maximum_angular_speed_rad_s: float = 0.025,
        maximum_age_s: float = 0.5,
    ) -> tuple[bool, str]:
        values = (
            self.target_x_base_m, self.target_y_base_m, self.linear_speed_m_s,
            self.angular_speed_rad_s, self.transform_age_s, self.odometry_age_s,
        )
        if not all(math.isfinite(value) for value in values):
            return False, "non_finite_parking_observation"
        if self.transform_age_s < 0.0 or self.odometry_age_s < 0.0:
            return False, "future_dated_parking_observation"
        if self.transform_age_s > maximum_age_s:
            return False, "target_transform_stale"
        if self.odometry_age_s > maximum_age_s:
            return False, "odometry_stale"
        if math.hypot(self.target_x_base_m - PICK_WINDOW_X_M, self.target_y_base_m - PICK_WINDOW_Y_M) > position_tolerance_m:
            return False, "target_outside_physical_pick_window"
        if abs(self.linear_speed_m_s) > maximum_linear_speed_m_s:
            return False, "base_not_stationary_linear"
        if abs(self.angular_speed_rad_s) > maximum_angular_speed_rad_s:
            return False, "base_not_stationary_angular"
        return True, "parked_in_physical_pick_window"


@dataclass(frozen=True)
class DryBinSample:
    sensor_ready: bool
    contained_object_count: int
    contained_mass_kg: float
    full: bool

    @classmethod
    def from_json(cls, encoded: str) -> "DryBinSample":
        try:
            raw: Any = json.loads(encoded)
        except json.JSONDecodeError as exc:
            raise ValueError("dry-bin status is not valid JSON") from exc
        if not isinstance(raw, Mapping):
            raise ValueError("dry-bin status must be a JSON object")
        try:
            sample = cls(
                sensor_ready=raw["sensor_ready"] is True,
                contained_object_count=int(raw["contained_object_count"]),
                contained_mass_kg=float(raw["contained_mass_kg"]),
                full=raw["full"] is True,
            )
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError("dry-bin status schema is incomplete") from exc
        if sample.contained_object_count < 0:
            raise ValueError("dry-bin count cannot be negative")
        if not math.isfinite(sample.contained_mass_kg) or sample.contained_mass_kg < 0.0:
            raise ValueError("dry-bin mass must be finite and non-negative")
        return sample


def verify_bin_increment(
    baseline: DryBinSample,
    samples: tuple[DryBinSample, ...],
    *,
    expected_material: str | None = None,
    mass_tolerance_kg: float = 1.0e-5,
    minimum_stable_samples: int = 8,
) -> tuple[bool, str, float | None]:
    """Require a stable physical increment matching the perceived material."""

    if not baseline.sensor_ready:
        return False, "dry_bin_sensor_not_ready_at_start", None
    if baseline.full:
        return False, "dry_bin_full_at_start", None
    if len(samples) < minimum_stable_samples:
        return False, "insufficient_post_release_bin_samples", None
    tail = samples[-minimum_stable_samples:]
    if any(not sample.sensor_ready for sample in tail):
        return False, "dry_bin_sensor_lost_after_release", None
    if any(sample.contained_object_count != baseline.contained_object_count + 1 for sample in tail):
        return False, "dry_bin_count_did_not_stably_increment_by_one", None
    if any(sample.full for sample in tail):
        return False, "dry_bin_full_after_release", None
    deltas = tuple(sample.contained_mass_kg - baseline.contained_mass_kg for sample in tail)
    if max(deltas) - min(deltas) > mass_tolerance_kg:
        return False, "dry_bin_mass_increment_not_stable", None
    measured = sum(deltas) / len(deltas)
    allowed = ((MATERIAL_MASS_KG[expected_material],) if expected_material in MATERIAL_MASS_KG else MATERIAL_MASSES_KG)
    if not any(abs(measured - expected) <= mass_tolerance_kg for expected in allowed):
        reason = (
            "dry_bin_mass_increment_does_not_match_perceived_material"
            if expected_material is not None
            else "dry_bin_mass_increment_not_a_permitted_material_cube"
        )
        return False, reason, measured
    return True, "physical_cube_stably_verified_in_dry_bin", measured


def material_for_measured_mass(
    measured_mass_kg: float, *, tolerance_kg: float = 1.0e-5
) -> str | None:
    """Classify material only from the post-deposit physical load increment."""

    for material, expected in MATERIAL_MASS_KG.items():
        if abs(measured_mass_kg - expected) <= tolerance_kg:
            return material
    return None
