"""Geometry gates for the formal vehicle storage and deposit passage.

This test expands the production Xacro and evaluates collision geometry in the
``base_link`` frame.  It intentionally checks collision primitives rather than
the decorative meshes: a visually open hopper or tank is not useful when a
hidden body collision blocks the physical volume.
"""

from __future__ import annotations

import copy
from dataclasses import dataclass
from functools import lru_cache
import math
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
MODEL = (
    ROOT
    / "starter_ws"
    / "src"
    / "sanitation_vehicle_description"
    / "urdf"
    / "formal_competition_vehicle.urdf.xacro"
)
MATERIALIZED_MODEL = ROOT / "reports" / "engineering" / "formal_competition_vehicle.urdf"
BODYWORK_XACRO = MODEL.parent / "high_fidelity" / "bodywork.xacro"
STORAGE_XACRO = MODEL.parent / "high_fidelity" / "storage_system.xacro"

# A 30 mm cube needs clearance for pose error and contact-solver tolerance.
CUBE_EDGE_M = 0.030
PASSAGE_SAFETY_MARGIN_M = 0.010
# Contacts at a designed common boundary are allowed; solid-volume overlap is not.
MATERIAL_OVERLAP_TOLERANCE_M = 0.001


@dataclass(frozen=True)
class Aabb:
    minimum: tuple[float, float, float]
    maximum: tuple[float, float, float]

    @property
    def size(self) -> tuple[float, float, float]:
        return tuple(self.maximum[i] - self.minimum[i] for i in range(3))


@dataclass(frozen=True)
class Collision:
    link: str
    name: str
    aabb: Aabb


def _matrix_mul(a: tuple[tuple[float, ...], ...], b: tuple[tuple[float, ...], ...]):
    return tuple(
        tuple(sum(a[row][k] * b[k][column] for k in range(4)) for column in range(4))
        for row in range(4)
    )


def _pose(xyz: str | None = None, rpy: str | None = None):
    x, y, z = (float(item) for item in (xyz or "0 0 0").split())
    roll, pitch, yaw = (float(item) for item in (rpy or "0 0 0").split())
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    # URDF specifies fixed-axis roll, pitch, yaw: Rz(yaw) * Ry(pitch) * Rx(roll).
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, x),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, y),
        (-sp, cp * sr, cp * cr, z),
        (0.0, 0.0, 0.0, 1.0),
    )


IDENTITY = _pose()


def _transform_point(matrix, point: tuple[float, float, float]):
    return tuple(
        sum(matrix[row][column] * point[column] for column in range(3)) + matrix[row][3]
        for row in range(3)
    )


def _origin(element: ET.Element):
    origin = element.find("origin")
    if origin is None:
        return IDENTITY
    return _pose(origin.get("xyz"), origin.get("rpy"))


def _run_checked(command: list[str], *, shell_command: str | None = None) -> str:
    completed = subprocess.run(
        command,
        check=False,
        capture_output=True,
        text=True,
        timeout=60,
    )
    assert completed.returncode == 0, (
        f"Xacro expansion failed ({shell_command or shlex.join(command)}):\n"
        f"stdout:\n{completed.stdout}\nstderr:\n{completed.stderr}"
    )
    return completed.stdout


@lru_cache(maxsize=1)
def _expanded_robot() -> ET.Element:
    source_only = os.environ.get("TZCUP_STORAGE_GEOMETRY_SOURCE_ONLY") == "1"
    xacro = None if source_only else shutil.which("xacro")
    if xacro:
        xml = _run_checked([xacro, str(MODEL), "use_sim:=false"])
        return ET.fromstring(xml)

    # The project is developed on Windows with ROS 2 Jazzy in WSL.  Keep the
    # same test executable from PowerShell instead of accepting a source-only
    # assertion that never materializes the robot tree.
    wsl = shutil.which("wsl.exe") if os.name == "nt" and not source_only else None
    if wsl:
        drive = MODEL.drive.rstrip(":").lower()
        assert len(drive) == 1 and MODEL.is_absolute(), f"unsupported Windows path: {MODEL}"
        linux_path = f"/mnt/{drive}/{MODEL.as_posix()[3:]}"
        bash = (
            "set -e; source /opt/ros/jazzy/setup.bash; "
            f"xacro {shlex.quote(linux_path)} use_sim:=false"
        )
        xml = _run_checked([wsl, "bash", "-lc", bash], shell_command=bash)
        return ET.fromstring(xml)

    # GitHub's fast gate is deliberately ROS-independent.  Its versioned
    # materialized URDF supplies the full joint tree, while the current Xacro
    # collision literals are overlaid below.  This keeps the CI gate sensitive
    # to the bodywork / chute edits under test without installing ROS merely to
    # evaluate numeric, argument-free geometry.
    return _ros_independent_geometry_robot()


def _ros_independent_geometry_robot() -> ET.Element:
    robot = ET.parse(MATERIALIZED_MODEL).getroot()
    overlays: list[tuple[Path, set[str] | None]] = [
        (BODYWORK_XACRO, {"bodywork_front_cowl_link", "bodywork_rear_shell_link"}),
        (STORAGE_XACRO, {"dry_deposit_chute_link"}),
    ]
    for path, selected in overlays:
        source = ET.parse(path).getroot()
        for source_link in source.findall(".//link"):
            name = source_link.get("name", "")
            if "${" in name:
                continue
            if selected is None:
                if not name.startswith("bodywork_"):
                    continue
            elif name not in selected:
                continue
            target_link = robot.find(f"./link[@name='{name}']")
            assert target_link is not None, f"materialized URDF is missing {name}"
            for collision in target_link.findall("collision"):
                target_link.remove(collision)
            for collision in source_link.findall("collision"):
                target_link.append(copy.deepcopy(collision))
    return robot


@lru_cache(maxsize=1)
def _link_transforms() -> dict[str, tuple[tuple[float, ...], ...]]:
    robot = _expanded_robot()
    links = {link.get("name") for link in robot.findall("link")}
    child_names = {
        joint.find("child").get("link")
        for joint in robot.findall("joint")
        if joint.find("child") is not None
    }
    roots = links - child_names
    assert len(roots) == 1, f"formal robot must have one root link, found {sorted(roots)}"
    transforms = {roots.pop(): IDENTITY}
    pending = list(robot.findall("joint"))
    while pending:
        progress = False
        for joint in pending[:]:
            parent = joint.find("parent").get("link")
            child = joint.find("child").get("link")
            if parent not in transforms:
                continue
            # All movable storage joints are evaluated at their declared zero
            # configuration.  This is the closed service state of the product.
            transforms[child] = _matrix_mul(transforms[parent], _origin(joint))
            pending.remove(joint)
            progress = True
        assert progress, "formal URDF contains a disconnected or cyclic joint tree"
    assert set(transforms) == links
    return transforms


def _box_aabb(transform, size: tuple[float, float, float]) -> Aabb:
    center = _transform_point(transform, (0.0, 0.0, 0.0))
    half = tuple(
        sum(abs(transform[axis][local]) * size[local] * 0.5 for local in range(3))
        for axis in range(3)
    )
    return Aabb(
        tuple(center[axis] - half[axis] for axis in range(3)),
        tuple(center[axis] + half[axis] for axis in range(3)),
    )


def _cylinder_aabb(transform, radius: float, length: float) -> Aabb:
    center = _transform_point(transform, (0.0, 0.0, 0.0))
    axis = tuple(transform[row][2] for row in range(3))
    half = tuple(
        abs(axis[row]) * length * 0.5
        + radius * math.sqrt(max(0.0, 1.0 - axis[row] * axis[row]))
        for row in range(3)
    )
    return Aabb(
        tuple(center[row] - half[row] for row in range(3)),
        tuple(center[row] + half[row] for row in range(3)),
    )


@lru_cache(maxsize=1)
def _collisions() -> tuple[Collision, ...]:
    collisions: list[Collision] = []
    transforms = _link_transforms()
    for link in _expanded_robot().findall("link"):
        link_name = link.get("name")
        for index, collision in enumerate(link.findall("collision")):
            name = collision.get("name") or f"unnamed_collision_{index}"
            transform = _matrix_mul(transforms[link_name], _origin(collision))
            geometry = collision.find("geometry")
            box = geometry.find("box") if geometry is not None else None
            cylinder = geometry.find("cylinder") if geometry is not None else None
            if box is not None:
                size = tuple(float(item) for item in box.get("size").split())
                aabb = _box_aabb(transform, size)
            elif cylinder is not None:
                aabb = _cylinder_aabb(
                    transform,
                    float(cylinder.get("radius")),
                    float(cylinder.get("length")),
                )
            else:
                continue
            collisions.append(Collision(link_name, name, aabb))
    return tuple(collisions)


def _collision(name: str) -> Collision:
    matches = [item for item in _collisions() if item.name == name]
    assert len(matches) == 1, f"expected one collision named {name}, found {len(matches)}"
    return matches[0]


def _visual_box(link_name: str, visual_name: str) -> Aabb:
    link = _expanded_robot().find(f"./link[@name='{link_name}']")
    assert link is not None
    visual = link.find(f"./visual[@name='{visual_name}']")
    assert visual is not None
    size = tuple(float(item) for item in visual.find("geometry/box").get("size").split())
    return _box_aabb(_matrix_mul(_link_transforms()[link_name], _origin(visual)), size)


def _overlap_depth(a: Aabb, b: Aabb) -> tuple[float, float, float]:
    return tuple(min(a.maximum[i], b.maximum[i]) - max(a.minimum[i], b.minimum[i]) for i in range(3))


def _materially_overlaps(a: Aabb, b: Aabb) -> bool:
    return all(depth > MATERIAL_OVERLAP_TOLERANCE_M for depth in _overlap_depth(a, b))


def _bodywork_intrusions(volume: Aabb) -> list[tuple[str, tuple[float, float, float]]]:
    return [
        (item.name, _overlap_depth(item.aabb, volume))
        for item in _collisions()
        if item.link.startswith("bodywork_") and _materially_overlaps(item.aabb, volume)
    ]


def _centered_box(link_name: str, size: tuple[float, float, float], z_offset: float = 0.0) -> Aabb:
    return _box_aabb(
        _matrix_mul(_link_transforms()[link_name], _pose(f"0 0 {z_offset}")),
        size,
    )


def test_dry_usable_volume_and_gravity_path_are_free_of_bodywork_collision() -> None:
    dry_usable = _visual_box("dry_bin_payload_reserve_link", "forty_litre_usable_envelope")
    assert math.prod(dry_usable.size) * 1000.0 >= 39.9
    assert _bodywork_intrusions(dry_usable) == []

    front = _collision("dry_deposit_chute_front_wall_collision").aabb
    rear = _collision("dry_deposit_chute_rear_wall_collision").aabb
    left = _collision("dry_deposit_chute_left_wall_collision").aabb
    right = _collision("dry_deposit_chute_right_wall_collision").aabb
    hopper_front = _collision("dry_deposit_hopper_front").aabb
    hopper_rear = _collision("dry_deposit_hopper_rear").aabb
    hopper_left = _collision("dry_deposit_hopper_left").aabb
    hopper_right = _collision("dry_deposit_hopper_right").aabb
    lid_rear = _collision("dry_lid_rear_collision").aabb
    lid_front = _collision("dry_lid_front_collision").aabb
    lid_left = _collision("dry_lid_left_collision").aabb
    lid_right = _collision("dry_lid_right_collision").aabb
    # Intersect the free cross-sections of the hopper, chute, service-lid
    # aperture and usable bin.  Extruding that common opening to the bin floor
    # proves one continuous vertical gravity path, not four unrelated openings.
    bore = Aabb(
        (
            max(dry_usable.minimum[0], rear.maximum[0], hopper_rear.maximum[0], lid_rear.maximum[0]),
            max(dry_usable.minimum[1], right.maximum[1], hopper_right.maximum[1], lid_right.maximum[1]),
            dry_usable.minimum[2],
        ),
        (
            min(dry_usable.maximum[0], front.minimum[0], hopper_front.minimum[0], lid_front.minimum[0]),
            min(dry_usable.maximum[1], left.minimum[1], hopper_left.minimum[1], lid_left.minimum[1]),
            max(hopper_front.maximum[2], hopper_rear.maximum[2], hopper_left.maximum[2], hopper_right.maximum[2]),
        ),
    )
    assert all(bore.minimum[i] < bore.maximum[i] for i in range(3))
    assert _bodywork_intrusions(bore) == []

    # The chute must land inside the dry usable footprint rather than beside it.
    for axis in (0, 1):
        assert dry_usable.minimum[axis] <= bore.minimum[axis]
        assert bore.maximum[axis] <= dry_usable.maximum[axis]


def test_wastewater_effective_fill_volume_is_free_of_bodywork_collision() -> None:
    # Frozen tank dimensions and competition payload cap from storage_system.xacro
    # / formal_competition_vehicle.urdf.xacro.  The effective volume is the
    # actual 9.7064 L accepted fill, not the unused 14 L installation envelope.
    inner_x, inner_y, inner_z = 0.350, 0.250, 0.160
    effective_l = 9.7064
    fill_height = effective_l / 1000.0 / (inner_x * inner_y)
    assert fill_height <= inner_z
    wet_effective = _centered_box(
        "wastewater_tank_link",
        (inner_x, inner_y, fill_height),
        z_offset=-inner_z * 0.5 + fill_height * 0.5,
    )
    assert math.isclose(math.prod(wet_effective.size) * 1000.0, effective_l, abs_tol=1e-6)
    assert _bodywork_intrusions(wet_effective) == []


def test_cube_passage_has_margin_through_chute_and_lid_aperture() -> None:
    front = _collision("dry_deposit_chute_front_wall_collision").aabb
    rear = _collision("dry_deposit_chute_rear_wall_collision").aabb
    left = _collision("dry_deposit_chute_left_wall_collision").aabb
    right = _collision("dry_deposit_chute_right_wall_collision").aabb
    chute_clearance = (front.minimum[0] - rear.maximum[0], left.minimum[1] - right.maximum[1])

    lid_rear = _collision("dry_lid_rear_collision").aabb
    lid_front = _collision("dry_lid_front_collision").aabb
    lid_left = _collision("dry_lid_left_collision").aabb
    lid_right = _collision("dry_lid_right_collision").aabb
    lid_clearance = (
        lid_front.minimum[0] - lid_rear.maximum[0],
        lid_left.minimum[1] - lid_right.maximum[1],
    )
    hopper_front = _collision("dry_deposit_hopper_front").aabb
    hopper_rear = _collision("dry_deposit_hopper_rear").aabb
    hopper_left = _collision("dry_deposit_hopper_left").aabb
    hopper_right = _collision("dry_deposit_hopper_right").aabb
    hopper_clearance = (
        hopper_front.minimum[0] - hopper_rear.maximum[0],
        hopper_left.minimum[1] - hopper_right.maximum[1],
    )
    required = CUBE_EDGE_M + PASSAGE_SAFETY_MARGIN_M
    assert min(*hopper_clearance, *chute_clearance, *lid_clearance) > required, (
        f"30 mm cube passage requires > {required:.3f} m; "
        f"hopper={hopper_clearance}, chute={chute_clearance}, lid={lid_clearance}"
    )


def test_bodywork_keeps_real_floor_side_and_rear_collision_structure() -> None:
    names = {item.name for item in _collisions() if item.link.startswith("bodywork_")}
    required = {
        "left_lower_tub_collision",
        "right_lower_tub_collision",
        "rear_shell_rear_wall_collision",
        "rear_shell_left_wall_collision",
        "rear_shell_right_wall_collision",
        "rear_shell_lower_left_skirt_collision",
        "rear_shell_lower_right_skirt_collision",
        "rear_shell_lower_rear_rail_collision",
    }
    assert required <= names
    for name in required:
        assert all(size > MATERIAL_OVERLAP_TOLERANCE_M for size in _collision(name).aabb.size)


def test_critical_storage_panels_do_not_materially_overlap_rear_shell() -> None:
    storage_names = {
        "dry_floor_collision",
        "dry_bin_front_panel_collision",
        "dry_bin_rear_panel_collision",
        "dry_bin_left_panel_collision",
        "dry_bin_right_panel_collision",
        "wet_floor_collision",
        "wastewater_front_panel_collision",
        "wastewater_rear_panel_collision",
        "wastewater_left_panel_collision",
        "wastewater_right_panel_collision",
    }
    shell_names = {
        "rear_shell_rear_wall_collision",
        "rear_shell_left_wall_collision",
        "rear_shell_right_wall_collision",
        "rear_shell_lower_left_skirt_collision",
        "rear_shell_lower_right_skirt_collision",
        "rear_shell_lower_rear_rail_collision",
    }
    overlaps = []
    for storage_name in storage_names:
        for shell_name in shell_names:
            depth = _overlap_depth(_collision(storage_name).aabb, _collision(shell_name).aabb)
            if all(item > MATERIAL_OVERLAP_TOLERANCE_M for item in depth):
                overlaps.append((storage_name, shell_name, depth))
    assert overlaps == []
