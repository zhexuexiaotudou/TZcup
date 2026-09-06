"""ROS-independent contract for the persistent MoveIt ground collision box.

The core deliberately knows nothing about Gazebo models or perception target
identity.  It validates only the configured robot/world split and a read-only
MoveIt planning-scene snapshot, so unit tests cannot be mistaken for a live
MoveIt acceptance result.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any, Mapping, Sequence
from xml.etree import ElementTree

import yaml


_EPSILON = 1.0e-6


def _finite_numbers(value: Any, *, name: str, count: int) -> tuple[float, ...]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)) or len(value) != count:
        raise ValueError(f"{name} must contain exactly {count} finite numbers")
    numbers = tuple(float(item) for item in value)
    if not all(math.isfinite(item) for item in numbers):
        raise ValueError(f"{name} must contain exactly {count} finite numbers")
    return numbers


def _nonempty_strings(value: Any, *, name: str) -> tuple[str, ...]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
        raise ValueError(f"{name} must be a sequence of non-empty strings")
    result = tuple(str(item) for item in value)
    if not result or any(not item.strip() for item in result) or len(set(result)) != len(result):
        raise ValueError(f"{name} must be unique non-empty strings")
    return result


def planning_virtual_joint_from_srdf(semantic_xml: str) -> tuple[str, str, str]:
    """Return the one MoveIt planning frame declared by the SRDF virtual joint.

    The planning scene service normalizes world collision-object headers to
    this frame.  Reading a source string here keeps the runtime bootstrap
    independent from a guessed frame name and lets it reject a description
    that no longer matches the configured ground box.
    """

    try:
        root = ElementTree.fromstring(semantic_xml)
    except (TypeError, ValueError, ElementTree.ParseError) as exc:
        raise ValueError("move_group robot_description_semantic is invalid") from exc
    joints = root.findall("virtual_joint")
    if len(joints) != 1:
        raise ValueError("formal SRDF must declare exactly one virtual_joint planning frame")
    name = joints[0].attrib.get("name", "").strip()
    parent_frame = joints[0].attrib.get("parent_frame", "").strip()
    child_link = joints[0].attrib.get("child_link", "").strip()
    if not name or not parent_frame or not child_link:
        raise ValueError("formal SRDF virtual_joint name, parent_frame and child_link are required")
    return name, parent_frame, child_link


def planning_frame_from_srdf(semantic_xml: str) -> str:
    """Return only the planning-frame component of the formal virtual joint."""

    return planning_virtual_joint_from_srdf(semantic_xml)[1]


@dataclass(frozen=True)
class GroundBox:
    object_id: str
    frame_id: str
    size_m: tuple[float, float, float]
    pose_xyz_m: tuple[float, float, float]
    pose_xyzw: tuple[float, float, float, float]
    top_height_m: float
    source_world_frame_id: str
    source_world_bounds_xy_m: tuple[float, float, float, float]
    localization_start_xy_yaw: tuple[float, float, float]
    localization_map_bounds_xy_m: tuple[float, float, float, float]
    geofence_margin_xy_m: tuple[float, float]
    allowed_contact_links: tuple[str, ...]

    def __post_init__(self) -> None:
        if not self.object_id.strip() or not self.frame_id.strip():
            raise ValueError("ground id and frame_id must be non-empty")
        if any(value <= 0.0 or not math.isfinite(value) for value in self.size_m):
            raise ValueError("ground size_m must be finite and strictly positive")
        if self.size_m[2] <= _EPSILON:
            raise ValueError("ground must have non-zero thickness")
        if not all(math.isfinite(value) for value in (*self.pose_xyz_m, *self.pose_xyzw)):
            raise ValueError("ground pose must be finite")
        norm = math.sqrt(sum(value * value for value in self.pose_xyzw))
        if abs(norm - 1.0) > _EPSILON:
            raise ValueError("ground pose quaternion must be normalized")
        if abs(self.pose_xyzw[0]) > _EPSILON or abs(self.pose_xyzw[1]) > _EPSILON:
            raise ValueError("ground box must remain horizontal")
        derived_top = self.pose_xyz_m[2] + self.size_m[2] / 2.0
        if abs(derived_top - self.top_height_m) > _EPSILON:
            raise ValueError("ground top_height_m must equal pose z plus half thickness")
        if not self.source_world_frame_id.strip():
            raise ValueError("source_world_frame_id must be non-empty")
        source_xmin, source_xmax, source_ymin, source_ymax = self.source_world_bounds_xy_m
        xmin, xmax, ymin, ymax = self.localization_map_bounds_xy_m
        if not source_xmin < source_xmax or not source_ymin < source_ymax:
            raise ValueError("source_world_bounds_xy_m must be ordered")
        if not xmin < xmax or not ymin < ymax:
            raise ValueError("localization_map_bounds_xy_m must be ordered")
        start_x, start_y, start_yaw = self.localization_start_xy_yaw
        if abs(start_yaw) > _EPSILON:
            raise ValueError("formal localization start yaw must be zero for an axis-aligned geofence")
        translated = (
            source_xmin - start_x, source_xmax - start_x,
            source_ymin - start_y, source_ymax - start_y,
        )
        if any(abs(actual - expected) > _EPSILON for actual, expected in zip(
            (xmin, xmax, ymin, ymax), translated
        )):
            raise ValueError("localization map geofence must apply source-world transform exactly once")
        margin_x, margin_y = self.geofence_margin_xy_m
        if margin_x <= 0.0 or margin_y <= 0.0:
            raise ValueError("ground geofence margins must be finite and strictly positive")
        half_x, half_y = self.size_m[0] / 2.0, self.size_m[1] / 2.0
        expected_bounds = (xmin - margin_x, xmax + margin_x, ymin - margin_y, ymax + margin_y)
        actual_bounds = (
            self.pose_xyz_m[0] - half_x, self.pose_xyz_m[0] + half_x,
            self.pose_xyz_m[1] - half_y, self.pose_xyz_m[1] + half_y,
        )
        if any(abs(actual - expected) > _EPSILON for actual, expected in zip(actual_bounds, expected_bounds)):
            raise ValueError("ground box must cover formal geofence with the configured boundary margin")
        if not self.allowed_contact_links:
            raise ValueError("ground must allow contact with explicit wheel support links")


@dataclass(frozen=True)
class PlanningSceneConfig:
    required_robot_links: tuple[str, ...]
    required_world_objects: tuple[str, ...]
    reject_unknown_scene_revision: bool
    scene_revision_prefix: str
    planning_frame_id: str
    planning_virtual_joint_name: str
    planning_virtual_joint_child_link: str
    ground: GroundBox
    negative_joint_names: tuple[str, ...]
    negative_joint_positions: tuple[float, ...]
    min_arm_collision_z_m: float
    expected_arm_contact_links: tuple[str, ...]

    def __post_init__(self) -> None:
        if set(self.required_robot_links) & set(self.required_world_objects):
            raise ValueError("robot links and world collision objects must be disjoint")
        if self.ground.object_id not in self.required_world_objects:
            raise ValueError("ground must be a required world collision object")
        if self.ground.frame_id != self.planning_frame_id:
            raise ValueError("ground frame must equal the configured MoveIt planning frame")
        if not self.planning_virtual_joint_name or not self.planning_virtual_joint_child_link:
            raise ValueError("MoveIt virtual joint name and child link must be non-empty")
        if not self.scene_revision_prefix.strip() or ":" in self.scene_revision_prefix:
            raise ValueError("planning scene revision prefix must be known and colon-free")
        if len(self.negative_joint_names) != len(self.negative_joint_positions):
            raise ValueError("runtime negative joint names and positions must have equal length")
        if not set(self.ground.allowed_contact_links).issubset(self.required_robot_links):
            raise ValueError("allowed ground contacts must be declared robot links")
        if not set(self.expected_arm_contact_links).isdisjoint(self.ground.allowed_contact_links):
            raise ValueError("arm links cannot be allowed to contact ground")
        bottom = self.ground.pose_xyz_m[2] - self.ground.size_m[2] / 2.0
        if bottom > self.min_arm_collision_z_m + _EPSILON:
            raise ValueError("ground thickness does not cover the frozen negative arm collision depth")


@dataclass(frozen=True)
class SceneObjectReadback:
    object_id: str
    frame_id: str
    shape_type: str
    dimensions_m: tuple[float, float, float]
    pose_xyz_m: tuple[float, float, float]
    pose_xyzw: tuple[float, float, float, float]


@dataclass(frozen=True)
class PlanningSceneReadback:
    revision: str | None
    world_objects: tuple[SceneObjectReadback, ...]
    allowed_collision_pairs: tuple[tuple[str, str], ...] = ()


def load_planning_scene_config(path: str | Path) -> PlanningSceneConfig:
    raw = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(raw, Mapping) or not isinstance(raw.get("planning_scene"), Mapping):
        raise ValueError("bin_and_scene.yaml must contain a planning_scene mapping")
    scene = raw["planning_scene"]
    ground_raw = scene.get("ground")
    gate_raw = scene.get("runtime_gate")
    planning_raw = scene.get("planning_frame")
    if not isinstance(ground_raw, Mapping) or not isinstance(gate_raw, Mapping) or not isinstance(planning_raw, Mapping):
        raise ValueError("planning_scene must contain planning_frame, ground and runtime_gate mappings")
    ground = GroundBox(
        object_id=str(ground_raw.get("id", "")),
        frame_id=str(ground_raw.get("frame_id", "")),
        size_m=_finite_numbers(ground_raw.get("size_m"), name="ground.size_m", count=3),
        pose_xyz_m=_finite_numbers(ground_raw.get("pose_xyz_m"), name="ground.pose_xyz_m", count=3),
        pose_xyzw=_finite_numbers(ground_raw.get("pose_xyzw"), name="ground.pose_xyzw", count=4),
        top_height_m=float(ground_raw.get("top_height_m")),
        source_world_frame_id=str(ground_raw.get("source_world_frame_id", "")),
        source_world_bounds_xy_m=_finite_numbers(ground_raw.get("source_world_bounds_xy_m"), name="ground.source_world_bounds_xy_m", count=4),
        localization_start_xy_yaw=_finite_numbers(ground_raw.get("localization_start_xy_yaw"), name="ground.localization_start_xy_yaw", count=3),
        localization_map_bounds_xy_m=_finite_numbers(ground_raw.get("localization_map_bounds_xy_m"), name="ground.localization_map_bounds_xy_m", count=4),
        geofence_margin_xy_m=_finite_numbers(ground_raw.get("geofence_margin_xy_m"), name="ground.geofence_margin_xy_m", count=2),
        allowed_contact_links=_nonempty_strings(
            ground_raw.get("allowed_contact_links"), name="ground.allowed_contact_links"
        ),
    )
    return PlanningSceneConfig(
        required_robot_links=_nonempty_strings(scene.get("required_robot_links"), name="required_robot_links"),
        required_world_objects=_nonempty_strings(scene.get("required_world_objects"), name="required_world_objects"),
        reject_unknown_scene_revision=scene.get("reject_unknown_scene_revision") is True,
        scene_revision_prefix=str(scene.get("scene_revision_prefix", "")),
        planning_frame_id=str(planning_raw.get("frame_id", "")),
        planning_virtual_joint_name=str(planning_raw.get("virtual_joint_name", "")),
        planning_virtual_joint_child_link=str(planning_raw.get("child_link", "")),
        ground=ground,
        negative_joint_names=_nonempty_strings(
            gate_raw.get("negative_joint_names"), name="runtime_gate.negative_joint_names"
        ),
        negative_joint_positions=_finite_numbers(
            gate_raw.get("negative_joint_positions"),
            name="runtime_gate.negative_joint_positions",
            count=len(gate_raw.get("negative_joint_names", ())),
        ),
        min_arm_collision_z_m=float(gate_raw.get("min_arm_collision_z_m")),
        expected_arm_contact_links=_nonempty_strings(
            gate_raw.get("expected_arm_contact_links"),
            name="runtime_gate.expected_arm_contact_links",
        ),
    )


def parse_scene_revision(config: PlanningSceneConfig, revision: str | None) -> int | None:
    """Return a verified monotonically numbered revision, or ``None`` if foreign."""

    if not isinstance(revision, str):
        return None
    prefix = f"{config.scene_revision_prefix}:"
    if not revision.startswith(prefix):
        return None
    suffix = revision[len(prefix):]
    if not suffix.isdecimal():
        return None
    number = int(suffix)
    return number if number >= 1 else None


def next_scene_revision(config: PlanningSceneConfig, revision: str | None) -> str:
    """Produce the next revision without ever accepting an unknown predecessor."""

    if revision is None:
        return f"{config.scene_revision_prefix}:1"
    number = parse_scene_revision(config, revision)
    if number is None:
        raise ValueError("planning scene revision is missing, unknown, or mismatched")
    return f"{config.scene_revision_prefix}:{number + 1}"


def validate_scene_readback(config: PlanningSceneConfig, readback: PlanningSceneReadback) -> int:
    """Raise unless the GetPlanningScene snapshot proves the configured ground.

    A missing name is an unknown revision.  This is intentionally strict when
    ``reject_unknown_scene_revision`` is configured, rather than assuming an
    empty MoveIt scene name is safe.
    """

    revision = parse_scene_revision(config, readback.revision)
    if config.reject_unknown_scene_revision and revision is None:
        raise ValueError("planning scene revision is missing, unknown, or mismatched")
    by_id = {item.object_id: item for item in readback.world_objects}
    missing = sorted(set(config.required_world_objects) - set(by_id))
    if missing:
        raise ValueError(f"planning scene is missing required world objects: {missing}")
    ground = by_id[config.ground.object_id]
    expected = config.ground
    if ground.frame_id != expected.frame_id:
        raise ValueError("ground readback frame does not match configuration")
    if ground.shape_type != "BOX":
        raise ValueError("ground readback shape is not a box")
    actual_top = ground.pose_xyz_m[2] + ground.dimensions_m[2] / 2.0
    if abs(actual_top - expected.top_height_m) > _EPSILON:
        raise ValueError("ground readback top height does not match configuration")
    for label, actual, wanted in (
        ("dimensions", ground.dimensions_m, expected.size_m),
        ("position", ground.pose_xyz_m, expected.pose_xyz_m),
        ("orientation", ground.pose_xyzw, expected.pose_xyzw),
    ):
        if len(actual) != len(wanted) or any(abs(a - b) > _EPSILON for a, b in zip(actual, wanted)):
            raise ValueError(f"ground readback {label} does not match configuration")
    if revision is None:
        raise ValueError("planning scene revision is missing, unknown, or mismatched")
    ground_pairs = {
        other if one == expected.object_id else one
        for one, other in readback.allowed_collision_pairs
        if expected.object_id in (one, other)
    }
    if ground_pairs != set(expected.allowed_contact_links):
        raise ValueError("ground allowed-collision matrix does not equal the wheel-only contract")
    return revision
