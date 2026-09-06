#!/usr/bin/env python3
"""Validate the formal skid-steer and cleaning geometry profile against source truth."""

from __future__ import annotations

import argparse
import json
import math
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
PROFILE = ROOT / "config/high_fidelity_vehicle/formal_motion_cleaning_profile.yaml"
LAYOUT = ROOT / "config/high_fidelity_vehicle/formal_vehicle_layout.yaml"
CLEANING_XACRO = ROOT / (
    "starter_ws/src/sanitation_vehicle_description/urdf/high_fidelity/cleaning_mechanism.xacro"
)
CONTROL_XACRO = ROOT / (
    "starter_ws/src/sanitation_vehicle_description/urdf/high_fidelity/control_interfaces.xacro"
)
PLATFORM_XACRO = ROOT / (
    "starter_ws/src/sanitation_vehicle_description/urdf/high_fidelity/a300_platform.xacro"
)
STORAGE_XACRO = ROOT / (
    "starter_ws/src/sanitation_vehicle_description/urdf/high_fidelity/storage_system.xacro"
)
CONTROLLERS = ROOT / (
    "starter_ws/src/sanitation_vehicle_description/config/formal_vehicle_controllers.yaml"
)
A300_DRIVETRAIN = ROOT / "config/high_fidelity_vehicle/a300_drivetrain_realism_contract.yaml"
INTEGRATION_PACKAGE_ROOT = ROOT / "starter_ws/src/sanitation_formal_campus_integration"
if str(INTEGRATION_PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(INTEGRATION_PACKAGE_ROOT))

from sanitation_formal_campus_integration.contract import (  # noqa: E402
    IntegrationContractError,
    load_formal_motion_profile,
)


# The URDF has a DART-only release clearance past the P16 product stroke.  It
# is deliberately not commandable: product commands and the ground-tangent
# work pose remain at exactly 100 mm.
CLEANING_LIFT_PRODUCT_STROKE_M = 0.100
CLEANING_LIFT_SOLVER_RELEASE_CLEARANCE_M = 0.00002


class FormalMotionCleaningProfileError(RuntimeError):
    """Raised when the profile differs from the project-owned source geometry."""


def _load_yaml(path: Path) -> dict:
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise FormalMotionCleaningProfileError(f"{path} must contain a mapping")
    return data


def _root(path: Path) -> ET.Element:
    return ET.parse(path).getroot()


def _local_name(element: ET.Element) -> str:
    return element.tag.rsplit("}", 1)[-1]


def _named(root: ET.Element, local_name: str, name: str) -> ET.Element:
    for element in root.iter():
        if _local_name(element) == local_name and element.get("name") == name:
            return element
    raise FormalMotionCleaningProfileError(f"missing {local_name} {name}")


def _child(element: ET.Element, local_name: str) -> ET.Element:
    for candidate in element:
        if _local_name(candidate) == local_name:
            return candidate
    raise FormalMotionCleaningProfileError(
        f"{_local_name(element)} {element.get('name', '')} has no {local_name}"
    )


def _xyz(element: ET.Element) -> tuple[float, float, float]:
    raw = element.get("xyz", "0 0 0").split()
    if len(raw) != 3:
        raise FormalMotionCleaningProfileError(f"invalid xyz: {element.get('xyz')}")
    try:
        return tuple(float(value) for value in raw)  # type: ignore[return-value]
    except ValueError as exc:
        raise FormalMotionCleaningProfileError(f"non-numeric xyz: {element.get('xyz')}") from exc


def _joint_origin_z(root: ET.Element, name: str) -> float:
    origin = _child(_named(root, "joint", name), "origin")
    raw = origin.get("xyz", "0 0 0").rsplit(maxsplit=1)
    if len(raw) != 2:
        raise FormalMotionCleaningProfileError(f"invalid xyz: {origin.get('xyz')}")
    try:
        return float(raw[1])
    except ValueError as exc:
        raise FormalMotionCleaningProfileError(
            f"non-numeric joint z for {name}: {raw[1]}"
        ) from exc


def _joint_axis_z(root: ET.Element, name: str) -> float:
    return _xyz(_child(_named(root, "joint", name), "axis"))[2]


def _collision(link: ET.Element, name: str | None = None) -> ET.Element:
    collisions = [item for item in link if _local_name(item) == "collision"]
    if name is not None:
        collisions = [item for item in collisions if item.get("name") == name]
    if len(collisions) != 1:
        raise FormalMotionCleaningProfileError(
            f"link {link.get('name')} expected one collision {name or ''}, found {len(collisions)}"
        )
    return collisions[0]


def _collision_origin_z(collision: ET.Element) -> float:
    origins = [item for item in collision if _local_name(item) == "origin"]
    return _xyz(origins[0])[2] if origins else 0.0


def _geometry(collision: ET.Element, shape: str) -> ET.Element:
    geometry = _child(collision, "geometry")
    return _child(geometry, shape)


def _number(value: object, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise FormalMotionCleaningProfileError(f"{label} must be numeric")
    result = float(value)
    if not math.isfinite(result):
        raise FormalMotionCleaningProfileError(f"{label} must be finite")
    return result


def _same(actual: float, expected: float, label: str, tolerance: float = 1e-9) -> None:
    if not math.isclose(actual, expected, rel_tol=0.0, abs_tol=tolerance):
        raise FormalMotionCleaningProfileError(
            f"{label} differs: expected {expected:.9f}, found {actual:.9f}"
        )


def _polygon(value: object, label: str) -> list[list[float]]:
    if not isinstance(value, list) or len(value) < 3:
        raise FormalMotionCleaningProfileError(f"{label} must contain at least three points")
    result: list[list[float]] = []
    for index, point in enumerate(value):
        if not isinstance(point, list) or len(point) != 2:
            raise FormalMotionCleaningProfileError(f"{label}[{index}] must be an xy pair")
        result.append(
            [_number(point[0], f"{label}[{index}].x"), _number(point[1], f"{label}[{index}].y")]
        )
    return result


def _rectangle(min_x: float, min_y: float, max_x: float, max_y: float) -> list[list[float]]:
    return [[max_x, max_y], [max_x, min_y], [min_x, min_y], [min_x, max_y]]


def _assert_polygon(actual: object, expected: list[list[float]], label: str) -> None:
    points = _polygon(actual, label)
    if len(points) != len(expected):
        raise FormalMotionCleaningProfileError(f"{label} point count differs")
    for index, (point, expected_point) in enumerate(zip(points, expected, strict=True)):
        _same(point[0], expected_point[0], f"{label}[{index}].x")
        _same(point[1], expected_point[1], f"{label}[{index}].y")


def _joint_limit(root: ET.Element, name: str) -> dict[str, float]:
    limit = _child(_named(root, "joint", name), "limit")
    result: dict[str, float] = {}
    for key in ("lower", "upper", "velocity", "effort"):
        if limit.get(key) is not None:
            result[key] = float(limit.get(key, ""))
    return result


def _control_call(root: ET.Element, name: str) -> ET.Element:
    for element in root.iter():
        if _local_name(element) in {"hf_position_joint", "hf_velocity_joint"} and element.get("name") == name:
            return element
    raise FormalMotionCleaningProfileError(f"missing ros2_control declaration for {name}")


def _validate_joint_limits(cleaning: ET.Element, storage: ET.Element, control: ET.Element) -> list[str]:
    source_joint_names = {
        "left_side_brush_joint": (cleaning, "${side}_side_brush_joint"),
        "right_side_brush_joint": (cleaning, "${side}_side_brush_joint"),
        "central_roller_joint": (cleaning, "central_roller_joint"),
        "recovery_pump_joint": (cleaning, "recovery_pump_joint"),
        "cleaning_lift_joint": (cleaning, "cleaning_lift_joint"),
        "dry_deposit_gate_joint": (storage, "dry_deposit_gate_joint"),
    }
    checked: list[str] = []
    for control_name, (source_root, source_name) in source_joint_names.items():
        limits = _joint_limit(source_root, source_name)
        declaration = _control_call(control, control_name)
        if _local_name(declaration) == "hf_velocity_joint":
            _same(float(declaration.get("max_velocity", "nan")), limits["velocity"], f"{control_name} velocity")
            _same(float(declaration.get("max_effort", "nan")), limits["effort"], f"{control_name} effort")
        else:
            for attribute in ("lower", "upper", "velocity", "effort"):
                actual = float(declaration.get(attribute, "nan"))
                if control_name == "cleaning_lift_joint" and attribute == "upper":
                    _same(
                        actual,
                        CLEANING_LIFT_PRODUCT_STROKE_M,
                        "cleaning lift commandable product stroke",
                    )
                    _same(
                        limits[attribute],
                        CLEANING_LIFT_PRODUCT_STROKE_M
                        + CLEANING_LIFT_SOLVER_RELEASE_CLEARANCE_M,
                        "cleaning lift DART-only solver release bound",
                    )
                    continue
                _same(actual, limits[attribute], f"{control_name} {attribute}")
            initial = float(declaration.get("initial_position", "0.0"))
            if not limits["lower"] <= initial <= limits["upper"]:
                raise FormalMotionCleaningProfileError(f"{control_name} initial position is outside URDF limits")
        checked.append(control_name)
    return checked


def validate_profile(
    profile_path: Path = PROFILE,
    layout_path: Path = LAYOUT,
    cleaning_path: Path = CLEANING_XACRO,
    control_path: Path = CONTROL_XACRO,
    platform_path: Path = PLATFORM_XACRO,
    storage_path: Path = STORAGE_XACRO,
    controllers_path: Path = CONTROLLERS,
) -> dict:
    try:
        profile = load_formal_motion_profile(profile_path)
    except IntegrationContractError as exc:
        raise FormalMotionCleaningProfileError(str(exc)) from exc
    layout = _load_yaml(layout_path)
    controllers = _load_yaml(controllers_path)
    cleaning = _root(cleaning_path)
    control = _root(control_path)
    platform = _root(platform_path)
    storage = _root(storage_path)

    if profile.get("schema_version") != 1:
        raise FormalMotionCleaningProfileError("unsupported profile schema_version")
    claims = profile.get("claim_boundary", {})
    if claims.get("physical_ground_contact_status") != "pending_runtime_contact_and_normal_force_validation":
        raise FormalMotionCleaningProfileError("profile must keep physical ground contact pending")

    drive = profile.get("drive", {})
    if drive.get("kinematic_model") != "four_wheel_skid_steer" or drive.get("steering_joint_names") != []:
        raise FormalMotionCleaningProfileError("formal drive must remain four-wheel skid-steer with no steering joints")
    if drive.get("effective_skid_steer_separation_m") is not None:
        raise FormalMotionCleaningProfileError("effective skid-steer separation must remain unset before calibration")
    canonical_constraint = "curvature_limited_reference_path_for_skid_steer"
    if drive.get("canonical_planning_kinematic_constraint") != canonical_constraint:
        raise FormalMotionCleaningProfileError("formal planning constraint must use the skid-steer canonical name")
    canonical_claim = drive.get("canonical_constraint_claim", {})
    if canonical_claim.get("physical_steering_claim") is not False:
        raise FormalMotionCleaningProfileError("canonical skid-steer constraint cannot claim physical steering")
    if canonical_claim.get("runtime_tracking_status") != "pending_skid_steer_tracking_validation":
        raise FormalMotionCleaningProfileError("canonical skid-steer tracking must remain pending runtime validation")
    virtual_ackermann = drive.get("virtual_ackermann_constraint", {})
    if virtual_ackermann.get("compatibility_alias_for") != canonical_constraint:
        raise FormalMotionCleaningProfileError("virtual Ackermann compatibility key must point to the canonical skid-steer constraint")
    if virtual_ackermann.get("represents_physical_steering") is not False:
        raise FormalMotionCleaningProfileError("virtual Ackermann constraint cannot claim physical steering")

    arm_task = profile.get("arm_task_poses", {})
    arm_joint_names = arm_task.get("joint_names")
    transport_pose = arm_task.get("transport_rad")
    if not isinstance(arm_joint_names, list) or len(arm_joint_names) != 6:
        raise FormalMotionCleaningProfileError("arm transport contract must name exactly six joints")
    if not isinstance(transport_pose, list) or len(transport_pose) != len(arm_joint_names):
        raise FormalMotionCleaningProfileError("arm transport pose must contain one value per joint")
    for name, expected in zip(arm_joint_names, transport_pose, strict=True):
        declaration = _control_call(control, str(name))
        raw_initial = declaration.get("initial_position")
        if raw_initial is None:
            raise FormalMotionCleaningProfileError(
                f"{name} must explicitly start in the frozen transport pose"
            )
        _same(
            float(raw_initial),
            _number(expected, f"arm transport pose {name}"),
            f"{name} initial/transport position",
        )

    drivetrain = _load_yaml(A300_DRIVETRAIN)
    published_drivetrain = drivetrain["published_platform_boundaries"]
    controller_radius = _number(
        published_drivetrain["odometry_control_radius_from_locked_control_m"],
        "plant control wheel radius",
    )
    physical_radius = _number(drive.get("physical_wheel_radius_m"), "physical wheel radius")
    declared_controller_radius = _number(drive.get("controller_wheel_radius_m"), "declared controller radius")
    wheel_collision = _collision(_named(platform, "link", "${name}_wheel_link"))
    source_wheel_radius = float(_geometry(wheel_collision, "cylinder").get("radius", "nan"))
    _same(source_wheel_radius, physical_radius, "source/profile physical wheel radius")
    _same(
        physical_radius,
        _number(published_drivetrain["physical_wheel_radius_from_locked_description_m"], "published physical wheel radius"),
        "profile/published physical wheel radius",
    )
    _same(declared_controller_radius, controller_radius, "profile/plant control wheel radius")
    side_calls = {
        element.get("side"): element
        for element in platform.iter()
        if _local_name(element) == "hf_a300_drivetrain_side"
    }
    if set(side_calls) != {"left", "right"}:
        raise FormalMotionCleaningProfileError("platform must instantiate exactly two A300 drivetrain sides")
    motor_templates = {
        element.get("name")
        for element in platform.iter()
        if _local_name(element) == "hf_a300_motor"
    }
    if motor_templates != {"front_${side}", "rear_${side}"}:
        raise FormalMotionCleaningProfileError(
            "each A300 drivetrain side must instantiate named front and rear motors"
        )
    expanded_wheel_names = {
        template.replace("${side}", side)
        for side in side_calls
        for template in motor_templates
    }
    if expanded_wheel_names != {"front_left", "front_right", "rear_left", "rear_right"}:
        raise FormalMotionCleaningProfileError(
            "A300 side/motor macro chain must resolve to exactly four named wheels"
        )
    chain = drivetrain["future_rigid_body_chain"]
    source_separation = 2.0 * _number(
        chain["per_wheel_links"]["resulting_tire_center_y_abs_m"],
        "resolved tire center y",
    )
    source_wheelbase = 2.0 * abs(
        _number(chain["per_wheel_links"]["front"]["beam_motor_xyz_rule_m"][0], "front motor x")
    )
    _same(_number(drive["geometric_wheel_separation_m"], "profile wheel separation"), source_separation, "profile/source wheel separation")
    _same(
        _number(drive["controller_wheel_separation_m"], "profile controller wheel separation"),
        _number(published_drivetrain["tire_center_track_locked_description_m"], "plant controller wheel separation"),
        "profile/plant controller wheel separation",
    )
    _same(_number(drive["wheelbase_m"], "profile wheelbase"), source_wheelbase, "profile/source wheelbase")
    expected_plant_wheels = {
        "front_left_wheel_joint", "front_right_wheel_joint",
        "rear_left_wheel_joint", "rear_right_wheel_joint",
    }
    if set(drive["wheel_joint_names"]) != expected_plant_wheels:
        raise FormalMotionCleaningProfileError("profile wheel joints differ from A300 effort plant")

    joint_limits_checked = _validate_joint_limits(cleaning, storage, control)
    controlled_names = {
        element.get("name")
        for element in control.iter()
        if _local_name(element) in {"hf_position_joint", "hf_velocity_joint"}
    }
    forbidden_passive_controls = {
        "squeegee_pitch_joint",
        "squeegee_float_joint",
        "dry_bin_lid_joint",
        "wastewater_lid_joint",
    } & controlled_names
    if forbidden_passive_controls:
        raise FormalMotionCleaningProfileError(
            "passive joints cannot be exported as ros2_control actuators: "
            + ", ".join(sorted(forbidden_passive_controls))
        )

    layout_transport = layout["vehicle_envelopes"]["transport_stowed"]
    transport_min = [_number(item, "transport min") for item in layout_transport["min_xyz_m"]]
    transport_max = [_number(item, "transport max") for item in layout_transport["max_xyz_m"]]
    footprints = profile.get("motion_footprints", {})
    nav2_footprint_padding_m = _number(
        profile.get("nav2_footprint_padding_m"), "Nav2 footprint padding"
    )
    if nav2_footprint_padding_m < 0.0:
        raise FormalMotionCleaningProfileError(
            "Nav2 footprint padding must be nonnegative"
        )
    _assert_polygon(
        footprints["transport_stowed"]["footprint_xy_m"],
        _rectangle(transport_min[0], transport_min[1], transport_max[0], transport_max[1]),
        "transport footprint",
    )
    arm = layout["arm_envelopes"]["deployed"]
    arm_min = [_number(item, "arm min") for item in arm["min_xyz_m"]]
    arm_max = [_number(item, "arm max") for item in arm["max_xyz_m"]]
    _assert_polygon(
        footprints["arm_deployed"]["footprint_xy_m"],
        _rectangle(arm_min[0], arm_min[1], arm_max[0], arm_max[1]),
        "arm footprint",
    )
    if footprints["arm_deployed"].get("navigation_allowed") is not False:
        raise FormalMotionCleaningProfileError("arm-deployed navigation must remain inhibited")

    sweeps = profile.get("mechanism_sweeps", {})
    left = sweeps["left_side_brush"]
    right = sweeps["right_side_brush"]
    left_center = [_number(item, "left brush center") for item in left["center_xy_m"]]
    right_center = [_number(item, "right brush center") for item in right["center_xy_m"]]
    side_mount_origin = _child(
        _named(cleaning, "joint", "${side}_side_brush_motor_mount_joint"), "origin"
    ).get("xyz", "")
    side_mount_tokens = side_mount_origin.split()
    lateral_match = re.search(r"lateral_sign\s*\*\s*([0-9.]+)", side_mount_origin)
    if not side_mount_tokens or lateral_match is None:
        raise FormalMotionCleaningProfileError("side-brush mount must expose numeric x and lateral magnitude")
    source_side_x = float(side_mount_tokens[0])
    source_side_y = float(lateral_match.group(1))
    _same(left_center[0], source_side_x, "left-brush center x")
    _same(left_center[1], source_side_y, "left-brush center y")
    _same(right_center[0], source_side_x, "right-brush center x")
    _same(right_center[1], -source_side_y, "right-brush center y")
    brush_radius = _number(left["radius_m"], "left brush radius")
    _same(_number(right["radius_m"], "right brush radius"), brush_radius, "side brush radii")
    cleaning_min_y = min(transport_min[1], left_center[1] - brush_radius, right_center[1] - brush_radius)
    cleaning_max_y = max(transport_max[1], left_center[1] + brush_radius, right_center[1] + brush_radius)
    _assert_polygon(
        footprints["cleaning_deployed"]["footprint_xy_m"],
        _rectangle(transport_min[0], cleaning_min_y, transport_max[0], cleaning_max_y),
        "cleaning footprint",
    )

    pose = profile["cleaning_work_pose"]["joint_positions"]
    lift = _number(pose["cleaning_lift_joint_m"], "working lift")
    float_position = _number(pose["squeegee_float_joint_m"], "working squeegee float")
    pitch = _number(pose["squeegee_pitch_joint_rad"], "working squeegee pitch")
    _same(pitch, 0.0, "working squeegee pitch")
    pose_limits = {
        "cleaning_lift_joint": (lift, _joint_limit(cleaning, "cleaning_lift_joint")),
        "squeegee_float_joint": (float_position, _joint_limit(cleaning, "squeegee_float_joint")),
        "squeegee_pitch_joint": (pitch, _joint_limit(cleaning, "squeegee_pitch_joint")),
    }
    for joint_name, (position, limits) in pose_limits.items():
        if not limits["lower"] <= position <= limits["upper"]:
            raise FormalMotionCleaningProfileError(f"working pose is outside {joint_name} limits")
    interlock = profile["cleaning_work_pose"]["command_interlock"]
    _same(_number(interlock["minimum_cleaning_lift_joint_m"], "interlock minimum lift"), lift, "interlock minimum lift")
    _same(_number(interlock["maximum_cleaning_lift_joint_m"], "interlock maximum lift"), lift, "interlock maximum lift")
    ground = _number(profile["cleaning_work_pose"]["expected_ground_plane_z_m"], "ground plane")

    lift_semantics = profile["cleaning_work_pose"]["lift_coordinate_semantics"]
    if lift_semantics.get("zero_position") != "transport_safe_raised":
        raise FormalMotionCleaningProfileError("cleaning lift zero must be transport_safe_raised")
    if lift_semantics.get("positive_direction") != "downward":
        raise FormalMotionCleaningProfileError("cleaning lift positive direction must be downward")
    transport_lift = _number(
        lift_semantics["transport_joint_position_m"], "transport lift position"
    )
    full_down_lift = _number(
        lift_semantics["full_down_work_position_m"], "full-down work position"
    )
    safe_clearance = _number(
        lift_semantics["zero_pose_minimum_ground_clearance_m"],
        "zero-pose minimum ground clearance",
    )
    _same(transport_lift, 0.0, "transport-safe lift position")
    _same(full_down_lift, lift, "full-down/work lift position")
    _same(lift, CLEANING_LIFT_PRODUCT_STROKE_M, "cleaning lift product work stroke")
    lift_control = _control_call(control, "cleaning_lift_joint")
    _same(
        float(lift_control.get("upper", "nan")),
        lift,
        "cleaning lift commandable/work position",
    )
    _same(
        float(lift_control.get("initial_position", "nan")),
        transport_lift,
        "cleaning lift initial/transport position",
    )

    base_z = _joint_origin_z(platform, "base_footprint_joint")
    lift_origin_z = _joint_origin_z(cleaning, "cleaning_lift_joint")
    lift_axis_z = _joint_axis_z(cleaning, "cleaning_lift_joint")
    _same(lift_axis_z, -1.0, "cleaning lift downward axis")
    lift_work_displacement_z = lift_axis_z * lift
    side_motor_z = _joint_origin_z(cleaning, "${side}_side_brush_motor_mount_joint")
    side_gear_z = _joint_origin_z(cleaning, "${side}_side_brush_gearbox_mount_joint")
    side_joint_z = _joint_origin_z(cleaning, "${side}_side_brush_joint")
    bristle_collision = _collision(_named(cleaning, "link", "${side}_side_brush_link"))
    if bristle_collision.get("name") is not None:
        raise FormalMotionCleaningProfileError(
            "side-brush sweep collision must remain unnamed so sdformat 14.9 "
            "preserves its Gazebo surface parameters after fixed-joint lumping"
        )
    bristle_cylinder = _geometry(bristle_collision, "cylinder")
    source_brush_radius = float(bristle_cylinder.get("radius", "nan"))
    bristle_thickness = float(bristle_cylinder.get("length", "nan"))
    side_lowest_z = (
        base_z + lift_origin_z + lift_work_displacement_z + side_motor_z + side_gear_z + side_joint_z
        + _collision_origin_z(bristle_collision) - bristle_thickness / 2.0
    )
    _same(source_brush_radius, brush_radius, "side-brush profile/source radius")
    _same(bristle_thickness, _number(left["collision_thickness_m"], "left brush thickness"), "left-brush thickness")
    _same(bristle_thickness, _number(right["collision_thickness_m"], "right brush thickness"), "right-brush thickness")
    _same(_number(left["expected_lowest_z_at_work_pose_m"], "left brush expected z"), side_lowest_z, "left-brush declared z")
    _same(_number(right["expected_lowest_z_at_work_pose_m"], "right brush expected z"), side_lowest_z, "right-brush declared z")
    if abs(side_lowest_z - ground) > 1e-9:
        raise FormalMotionCleaningProfileError(
            "compressed side-brush collision envelope must remain tangent "
            f"to the ground plane, found {side_lowest_z:.9f}"
        )

    roller_collision = _collision(_named(cleaning, "link", "central_roller_link"))
    roller_cylinder = _geometry(roller_collision, "cylinder")
    roller_radius = float(roller_cylinder.get("radius", "nan"))
    roller_width = float(roller_cylinder.get("length", "nan"))
    roller_lowest_z = (
        base_z + lift_origin_z + lift_work_displacement_z + _joint_origin_z(cleaning, "central_roller_motor_mount_joint")
        - roller_radius
    )
    roller_center = [_number(item, "roller center") for item in sweeps["central_roller"]["center_xy_m"]]
    roller_mount_xyz = _xyz(_child(_named(cleaning, "joint", "central_roller_motor_mount_joint"), "origin"))
    _same(roller_center[0], roller_mount_xyz[0], "roller center x")
    _same(roller_center[1], roller_mount_xyz[1], "roller center y")
    _same(roller_radius, _number(sweeps["central_roller"]["radius_m"], "roller radius"), "roller radius")
    _same(roller_width, _number(sweeps["central_roller"]["width_m"], "roller width"), "roller width")
    _same(roller_lowest_z, ground, "central-roller expected lowest z")
    _same(
        _number(sweeps["central_roller"]["expected_lowest_z_at_work_pose_m"], "roller expected z"),
        roller_lowest_z,
        "roller declared z",
    )

    squeegee_collision = _collision(_named(cleaning, "link", "squeegee_link"))
    squeegee_box = [float(item) for item in _geometry(squeegee_collision, "box").get("size", "").split()]
    squeegee_frame_z = (
        base_z + lift_origin_z + lift_work_displacement_z + _joint_origin_z(cleaning, "squeegee_float_joint")
        + float_position + _joint_origin_z(cleaning, "squeegee_pitch_joint")
    )
    squeegee_center = [_number(item, "squeegee center") for item in sweeps["squeegee"]["center_xy_m"]]
    squeegee_float_xyz = _xyz(_child(_named(cleaning, "joint", "squeegee_float_joint"), "origin"))
    _same(squeegee_center[0], squeegee_float_xyz[0], "squeegee center x")
    _same(squeegee_center[1], squeegee_float_xyz[1], "squeegee center y")
    squeegee_lowest_z = squeegee_frame_z + _collision_origin_z(squeegee_collision) - squeegee_box[2] / 2.0
    declared_squeegee_box = [_number(item, "squeegee box") for item in sweeps["squeegee"]["collision_size_xyz_m"]]
    for index, value in enumerate(squeegee_box):
        _same(value, declared_squeegee_box[index], f"squeegee box[{index}]")
    _same(squeegee_lowest_z, ground, "squeegee expected lowest z")
    _same(
        _number(sweeps["squeegee"]["expected_lowest_z_at_work_pose_m"], "squeegee expected z"),
        squeegee_lowest_z,
        "squeegee declared z",
    )

    nozzle_collision = _collision(_named(cleaning, "link", "suction_nozzle_link"), "suction_nozzle_link_collision")
    nozzle_box = [float(item) for item in _geometry(nozzle_collision, "box").get("size", "").split()]
    nozzle_mount_xyz = _xyz(_child(_named(cleaning, "joint", "suction_nozzle_mount_joint"), "origin"))
    nozzle_center = [_number(item, "nozzle center") for item in sweeps["suction_nozzle"]["center_xy_m"]]
    _same(nozzle_center[0], squeegee_float_xyz[0] + nozzle_mount_xyz[0], "nozzle center x")
    _same(nozzle_center[1], squeegee_float_xyz[1] + nozzle_mount_xyz[1], "nozzle center y")
    nozzle_lowest_z = (
        squeegee_frame_z + _joint_origin_z(cleaning, "suction_nozzle_mount_joint")
        + _collision_origin_z(nozzle_collision) - nozzle_box[2] / 2.0
    )
    declared_nozzle_box = [_number(item, "nozzle box") for item in sweeps["suction_nozzle"]["collision_size_xyz_m"]]
    for index, value in enumerate(nozzle_box):
        _same(value, declared_nozzle_box[index], f"nozzle box[{index}]")
    _same(
        nozzle_lowest_z,
        _number(sweeps["suction_nozzle"]["expected_lowest_z_at_work_pose_m"], "nozzle clearance"),
        "suction-nozzle expected lowest z",
    )
    zero_pose_lowest_z = min(
        side_lowest_z - lift_work_displacement_z,
        roller_lowest_z - lift_work_displacement_z,
        squeegee_lowest_z - lift_work_displacement_z,
        nozzle_lowest_z - lift_work_displacement_z,
    )
    _same(zero_pose_lowest_z, safe_clearance, "zero-pose minimum ground clearance")

    transverse = sweeps["transverse_union"]
    _same(_number(transverse["minimum_y_m"], "sweep minimum y"), cleaning_min_y, "sweep minimum y")
    _same(_number(transverse["maximum_y_m"], "sweep maximum y"), cleaning_max_y, "sweep maximum y")
    _same(
        _number(transverse["geometric_sweep_width_m"], "geometric sweep width"),
        cleaning_max_y - cleaning_min_y,
        "geometric sweep width",
    )
    if transverse.get("effective_width_is_not_inferred_from_collision") is not True:
        raise FormalMotionCleaningProfileError("effective cleaning width must not be inferred from collision geometry")
    _same(
        _number(transverse["declared_effective_cleaning_width_m"], "declared effective width"),
        _number(layout["cleaning_geometry"]["effective_working_width_m"], "layout effective width"),
        "profile/layout effective cleaning width",
    )

    zero_pose_z = {
        "left_side_brush_link": base_z + lift_origin_z + side_motor_z + side_gear_z + side_joint_z,
        "right_side_brush_link": base_z + lift_origin_z + side_motor_z + side_gear_z + side_joint_z,
        "central_roller_link": base_z + lift_origin_z + _joint_origin_z(cleaning, "central_roller_motor_mount_joint"),
        "squeegee_link": base_z + lift_origin_z + _joint_origin_z(cleaning, "squeegee_float_joint")
        + _joint_origin_z(cleaning, "squeegee_pitch_joint"),
        "suction_nozzle_link": base_z + lift_origin_z + _joint_origin_z(cleaning, "squeegee_float_joint")
        + _joint_origin_z(cleaning, "squeegee_pitch_joint") + _joint_origin_z(cleaning, "suction_nozzle_mount_joint"),
    }
    for frame, source_z in zero_pose_z.items():
        layout_z = _number(layout["installation_frames"][frame]["xyz_m"][2], f"layout {frame} z")
        _same(source_z, layout_z, f"source/layout {frame} z")

    return {
        "status": "FORMAL_MOTION_CLEANING_PROFILE_SOURCE_CHECKS_PASSED_RUNTIME_CONTACT_PENDING",
        "profile_id": profile.get("profile_id"),
        "joint_limits_checked": joint_limits_checked,
        "arm_transport_start_positions_rad": dict(
            zip(arm_joint_names, transport_pose, strict=True)
        ),
        "wheel_radius_m": physical_radius,
        "control_wheel_radius_m": controller_radius,
        "planning_kinematic_constraint": canonical_constraint,
        "physical_steering_claim": canonical_claim["physical_steering_claim"],
        "runtime_tracking_status": canonical_claim["runtime_tracking_status"],
        "working_pose": {"lift_m": lift, "squeegee_float_m": float_position, "squeegee_pitch_rad": pitch},
        "lift_coordinate_semantics": {
            "zero_position": "transport_safe_raised",
            "positive_direction": "downward",
            "transport_lift_m": transport_lift,
            "work_lift_m": full_down_lift,
            "zero_pose_minimum_ground_clearance_m": zero_pose_lowest_z,
        },
        "expected_geometry_z_m": {
            "side_brush_lowest": side_lowest_z,
            "central_roller_lowest": roller_lowest_z,
            "squeegee_lowest": squeegee_lowest_z,
            "suction_nozzle_lowest": nozzle_lowest_z,
        },
        "physical_ground_contact_status": claims["physical_ground_contact_status"],
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--profile", type=Path, default=PROFILE)
    args = parser.parse_args()
    try:
        report = validate_profile(profile_path=args.profile)
    except (FormalMotionCleaningProfileError, KeyError, TypeError, ValueError, ET.ParseError) as exc:
        print(json.dumps({"status": "FAILED", "error": str(exc)}, indent=2))
        return 2
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
