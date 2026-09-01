#!/usr/bin/env python3
"""Deterministic formal-vehicle mass, CoG and collision sweep audit.

The scanner consumes an expanded URDF.  It never substitutes visual geometry
for collision geometry: mesh bounds are calculated from every STL vertex and
primitive bounds are analytic.  The reported arm operating envelope is the
union of a declared transport pose, joint-limit anchors, production task
anchors and a deterministic Halton joint-space sample.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import heapq
import hashlib
import json
import math
from pathlib import Path
import struct
from typing import Any, Iterable
import xml.etree.ElementTree as ET

import numpy as np
import yaml

from formal_gripper_linkage_contract import resolve_mimic_relations


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_URDF = ROOT / "reports/engineering/formal_competition_vehicle.urdf"
DEFAULT_LAYOUT = ROOT / "config/high_fidelity_vehicle/formal_vehicle_layout.yaml"
ARM_JOINTS = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)
GRIPPER_MASTER = "robotiq_85_left_knuckle_joint"
TRANSPORT_POSE = (-1.0, -1.0, 1.8, -1.5, -1.55, 0.25)
TASK_ANCHORS = {
    "pregrasp": (-1.48278161, -0.44199397, 1.21471947, -2.34352182, -1.57079633, -3.05357794),
    "pick": (-1.48278161, 0.10260211, 0.80254082, -2.47593926, -1.57079633, -3.05357794),
    "deposit": (-0.30233498, -1.56960444, -0.73057657, -2.40638971, 1.56851193, 0.60324332),
}
TASK_AUXILIARY_POSITIONS = {
    # The physical runtime opens the powered robot-only gate before moving to
    # the release pose.  Collision geometry must be evaluated in that actual
    # mechanism state, not against the closed service state.
    "deposit": {"dry_deposit_gate_joint": 1.05},
}
HALTON_BASES = (2, 3, 5, 7, 11, 13)
DRY_CAPACITY_KG = 1.512
WASTEWATER_CAPACITY_KG = 8.30
WATER_DENSITY_KG_L = 1.0
A300_CURB_MASS_KG = 78.5
A300_RATED_PAYLOAD_KG = 101.5
A300_PAYLOAD_DESIGN_LIMIT_KG = 91.35
MINIMUM_STATIC_MARGIN_M = 0.03
KNOWN_ASSEMBLY_CONTACT_PAIRS = {
    tuple(sorted(pair))
    for pair in (
        ("ur5e_wrist_3_link", "ur_to_robotiq_adapter_link"),
        ("robotiq_85_left_finger_link", "robotiq_85_left_inner_knuckle_link"),
        ("robotiq_85_right_finger_link", "robotiq_85_right_inner_knuckle_link"),
        ("robotiq_85_left_inner_knuckle_link", "robotiq_85_left_finger_tip_link"),
        ("robotiq_85_right_inner_knuckle_link", "robotiq_85_right_finger_tip_link"),
    )
}


class ScanError(RuntimeError):
    """Raised when the URDF cannot support a fail-closed scan."""


def _push_bounded_worst_candidate(
    heap: list[tuple[float, int, dict[str, Any]]],
    candidate: dict[str, Any],
    sequence: int,
    *,
    limit: int = 50,
) -> None:
    """Retain the exact stable top-N collision candidates without O(events) RAM.

    The former implementation retained every candidate dictionary and sorted
    the complete list after all 1,000+ arm poses. Dense raw-joint collisions
    can create hundreds of thousands of dictionaries even though the report
    emits only 50. ``-sequence`` makes earlier equal-penetration events rank
    ahead of later ones, matching Python's previous stable descending sort.
    """

    if limit <= 0:
        return
    entry = (
        float(candidate["conservative_obb_penetration_m"]),
        -sequence,
        candidate,
    )
    if len(heap) < limit:
        heapq.heappush(heap, entry)
    elif entry[:2] > heap[0][:2]:
        heapq.heapreplace(heap, entry)


def _sorted_bounded_worst_candidates(
    heap: list[tuple[float, int, dict[str, Any]]],
) -> list[dict[str, Any]]:
    return [
        entry[2]
        for entry in sorted(heap, key=lambda entry: (-entry[0], -entry[1]))
    ]


def _vector(text: str | None, default: Iterable[float] = (0.0, 0.0, 0.0)) -> np.ndarray:
    if text is None:
        return np.asarray(tuple(default), dtype=float)
    values = tuple(float(token) for token in text.replace(",", " ").split())
    if len(values) != 3 or not all(math.isfinite(value) for value in values):
        raise ScanError(f"expected finite xyz/rpy vector, got {text!r}")
    return np.asarray(values, dtype=float)


def _rotation_rpy(rpy: np.ndarray) -> np.ndarray:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.asarray(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=float,
    )


def _transform(xyz: np.ndarray | None = None, rpy: np.ndarray | None = None) -> np.ndarray:
    result = np.eye(4, dtype=float)
    result[:3, :3] = _rotation_rpy(np.zeros(3) if rpy is None else rpy)
    result[:3, 3] = np.zeros(3) if xyz is None else xyz
    return result


def _axis_rotation(axis: np.ndarray, angle: float) -> np.ndarray:
    norm = float(np.linalg.norm(axis))
    if norm <= 1e-12:
        raise ScanError("movable joint has a zero axis")
    x, y, z = axis / norm
    c, s, one = math.cos(angle), math.sin(angle), 1.0 - math.cos(angle)
    result = np.eye(4, dtype=float)
    result[:3, :3] = np.asarray(
        [
            [c + x * x * one, x * y * one - z * s, x * z * one + y * s],
            [y * x * one + z * s, c + y * y * one, y * z * one - x * s],
            [z * x * one - y * s, z * y * one + x * s, c + z * z * one],
        ],
        dtype=float,
    )
    return result


def _translation(axis: np.ndarray, distance: float) -> np.ndarray:
    norm = float(np.linalg.norm(axis))
    if norm <= 1e-12:
        raise ScanError("prismatic joint has a zero axis")
    result = np.eye(4, dtype=float)
    result[:3, 3] = axis / norm * distance
    return result


def _origin(node: ET.Element | None) -> np.ndarray:
    if node is None:
        return np.eye(4, dtype=float)
    return _transform(_vector(node.get("xyz")), _vector(node.get("rpy")))


@dataclass(frozen=True)
class Joint:
    name: str
    kind: str
    parent: str
    child: str
    origin: np.ndarray
    axis: np.ndarray
    lower: float
    upper: float
    mimic_joint: str | None
    mimic_multiplier: float
    mimic_offset: float


@dataclass(frozen=True)
class Inertial:
    mass: float
    origin: np.ndarray


@dataclass(frozen=True)
class CollisionShape:
    link: str
    name: str
    origin: np.ndarray
    kind: str
    parameters: tuple[float, ...]
    vertices: np.ndarray | None
    source: str


@dataclass(frozen=True)
class CollisionObb:
    """Conservative oriented bound derived from one URDF collision element."""

    link: str
    name: str
    center: np.ndarray
    axes: np.ndarray
    half_size: np.ndarray
    source: str


@dataclass
class Bounds:
    lower: np.ndarray
    upper: np.ndarray

    @classmethod
    def empty(cls) -> "Bounds":
        return cls(np.full(3, np.inf), np.full(3, -np.inf))

    def include(self, lower: np.ndarray, upper: np.ndarray) -> None:
        self.lower = np.minimum(self.lower, lower)
        self.upper = np.maximum(self.upper, upper)

    def payload(self) -> dict[str, list[float]]:
        if not np.all(np.isfinite(self.lower)) or not np.all(np.isfinite(self.upper)):
            raise ScanError("empty collision envelope")
        return {
            "min_xyz_m": [round(float(value), 6) for value in self.lower],
            "max_xyz_m": [round(float(value), 6) for value in self.upper],
            "size_xyz_m": [round(float(value), 6) for value in self.upper - self.lower],
        }


class Model:
    def __init__(self, urdf_path: Path, root: Path = ROOT) -> None:
        self.urdf_path = urdf_path
        self.root = root
        document = ET.parse(urdf_path).getroot()
        if document.tag != "robot":
            raise ScanError("expanded URDF root must be <robot>")
        self.links = {node.get("name", ""): node for node in document.findall("link")}
        if "base_footprint" not in self.links:
            raise ScanError("expanded URDF is not rooted at base_footprint")
        self.joints: dict[str, Joint] = {}
        self.child_joints: dict[str, list[Joint]] = {}
        self.parent_joint: dict[str, Joint] = {}
        joint_node_list = document.findall("joint")
        joint_names = [node.get("name", "") for node in joint_node_list]
        if len(joint_names) != len(set(joint_names)):
            raise ScanError("URDF contains duplicate joint names")
        joint_nodes = {node.get("name", ""): node for node in joint_node_list}
        explicit_mimics = {}
        for name, node in joint_nodes.items():
            mimic = node.find("mimic")
            explicit_mimics[name] = None if mimic is None else (
                mimic.get("joint", ""),
                float(mimic.get("multiplier", "1")),
                float(mimic.get("offset", "0")),
            )
        resolved_mimics = resolve_mimic_relations(explicit_mimics)
        for name, node in joint_nodes.items():
            kind = node.get("type", "")
            parent_node, child_node = node.find("parent"), node.find("child")
            if parent_node is None or child_node is None:
                raise ScanError(f"joint {name} lacks parent or child")
            parent, child = parent_node.get("link", ""), child_node.get("link", "")
            limit = node.find("limit")
            if kind == "continuous":
                lower, upper = -math.pi, math.pi
            elif kind in {"revolute", "prismatic"}:
                if limit is None or limit.get("lower") is None or limit.get("upper") is None:
                    raise ScanError(f"joint {name} lacks finite limits")
                lower, upper = float(limit.get("lower")), float(limit.get("upper"))
            else:
                lower = upper = 0.0
            mimic = resolved_mimics.get(name)
            joint = Joint(
                name=name,
                kind=kind,
                parent=parent,
                child=child,
                origin=_origin(node.find("origin")),
                axis=_vector(node.find("axis").get("xyz") if node.find("axis") is not None else None, (1, 0, 0)),
                lower=lower,
                upper=upper,
                mimic_joint=mimic[0] if mimic is not None else None,
                mimic_multiplier=mimic[1] if mimic is not None else 1.0,
                mimic_offset=mimic[2] if mimic is not None else 0.0,
            )
            if name in self.joints or child in self.parent_joint:
                raise ScanError(f"duplicate joint or multiply-parented link: {name}/{child}")
            self.joints[name] = joint
            self.parent_joint[child] = joint
            self.child_joints.setdefault(parent, []).append(joint)
        self.inertials = self._parse_inertials()
        self.shapes = self._parse_collisions()
        self.visual_count = sum(len(link.findall("visual")) for link in self.links.values())

    def _parse_inertials(self) -> dict[str, Inertial]:
        result: dict[str, Inertial] = {}
        for name, link in self.links.items():
            inertial = link.find("inertial")
            if inertial is None:
                continue
            mass_node = inertial.find("mass")
            if mass_node is None:
                raise ScanError(f"link {name} inertial lacks mass")
            mass = float(mass_node.get("value", "nan"))
            if not math.isfinite(mass) or mass <= 0.0:
                raise ScanError(f"link {name} has non-positive mass")
            result[name] = Inertial(mass, _origin(inertial.find("origin")))
        return result

    def _resolve_mesh(self, uri: str) -> Path:
        prefix = "package://sanitation_vehicle_description/"
        if not uri.startswith(prefix):
            raise ScanError(f"unsupported collision mesh URI: {uri}")
        path = self.root / "starter_ws/src/sanitation_vehicle_description" / uri[len(prefix):]
        if not path.is_file():
            raise ScanError(f"collision mesh is missing: {path}")
        if path.suffix.lower() != ".stl":
            raise ScanError(f"collision mesh must be STL for deterministic vertex audit: {uri}")
        return path

    @staticmethod
    def _stl_vertices(path: Path) -> np.ndarray:
        raw = path.read_bytes()
        if len(raw) >= 84:
            triangle_count = struct.unpack_from("<I", raw, 80)[0]
            if 84 + triangle_count * 50 == len(raw):
                dtype = np.dtype(
                    [("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")]
                )
                return np.frombuffer(raw, dtype=dtype, count=triangle_count, offset=84)["vertices"].reshape(-1, 3).astype(float)
        vertices = []
        for line in raw.decode("utf-8", errors="strict").splitlines():
            tokens = line.strip().split()
            if len(tokens) == 4 and tokens[0].lower() == "vertex":
                vertices.append(tuple(float(token) for token in tokens[1:]))
        if not vertices:
            raise ScanError(f"STL contains no vertices: {path}")
        return np.asarray(vertices, dtype=float)

    def _parse_collisions(self) -> list[CollisionShape]:
        result: list[CollisionShape] = []
        for link_name, link in self.links.items():
            for index, collision in enumerate(link.findall("collision")):
                geometry = collision.find("geometry")
                if geometry is None or len(geometry) != 1:
                    raise ScanError(f"collision {link_name}/{index} lacks one geometry")
                node = geometry[0]
                kind = node.tag
                vertices: np.ndarray | None = None
                parameters: tuple[float, ...] = ()
                source = kind
                if kind == "mesh":
                    uri = node.get("filename", "")
                    path = self._resolve_mesh(uri)
                    vertices = self._stl_vertices(path)
                    scale = _vector(node.get("scale"), (1, 1, 1))
                    vertices = vertices * scale
                    source = path.relative_to(self.root).as_posix()
                elif kind == "box":
                    parameters = tuple(_vector(node.get("size")))
                elif kind == "cylinder":
                    parameters = (float(node.get("radius", "nan")), float(node.get("length", "nan")))
                elif kind == "sphere":
                    parameters = (float(node.get("radius", "nan")),)
                else:
                    raise ScanError(f"unsupported collision geometry {kind} on {link_name}")
                if parameters and (not all(math.isfinite(v) and v > 0 for v in parameters)):
                    raise ScanError(f"invalid {kind} collision dimensions on {link_name}")
                result.append(
                    CollisionShape(
                        link=link_name,
                        name=collision.get("name", f"collision_{index}"),
                        origin=_origin(collision.find("origin")),
                        kind=kind,
                        parameters=parameters,
                        vertices=vertices,
                        source=source,
                    )
                )
        if not result:
            raise ScanError("expanded URDF has no collision geometry")
        return result

    def transforms(self, positions: dict[str, float]) -> dict[str, np.ndarray]:
        result = {"base_footprint": np.eye(4, dtype=float)}
        queue = ["base_footprint"]
        while queue:
            parent = queue.pop(0)
            for joint in sorted(self.child_joints.get(parent, ()), key=lambda item: item.name):
                position = positions.get(joint.name, 0.0)
                if joint.mimic_joint:
                    position = positions.get(joint.mimic_joint, 0.0) * joint.mimic_multiplier + joint.mimic_offset
                motion = np.eye(4, dtype=float)
                if joint.kind in {"revolute", "continuous"}:
                    motion = _axis_rotation(joint.axis, position)
                elif joint.kind == "prismatic":
                    motion = _translation(joint.axis, position)
                result[joint.child] = result[parent] @ joint.origin @ motion
                queue.append(joint.child)
        missing = sorted(set(self.links) - set(result))
        if missing:
            raise ScanError("URDF is disconnected: " + ", ".join(missing))
        return result

    def descendants(self, root_link: str) -> set[str]:
        found = {root_link}
        queue = [root_link]
        while queue:
            current = queue.pop()
            for joint in self.child_joints.get(current, ()):
                if joint.child not in found:
                    found.add(joint.child)
                    queue.append(joint.child)
        return found


def _shape_bounds(shape: CollisionShape, link_transform: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    transform = link_transform @ shape.origin
    rotation, translation = transform[:3, :3], transform[:3, 3]
    if shape.kind == "mesh":
        assert shape.vertices is not None
        points = shape.vertices @ rotation.T + translation
        return points.min(axis=0), points.max(axis=0)
    if shape.kind == "box":
        half = np.asarray(shape.parameters) * 0.5
        extent = np.abs(rotation) @ half
    elif shape.kind == "cylinder":
        radius, length = shape.parameters
        extent = np.asarray(
            [radius * math.hypot(rotation[row, 0], rotation[row, 1]) + abs(rotation[row, 2]) * length * 0.5 for row in range(3)]
        )
    elif shape.kind == "sphere":
        extent = np.full(3, shape.parameters[0])
    else:  # pragma: no cover - parser rejects this
        raise ScanError(f"unhandled collision shape {shape.kind}")
    return translation - extent, translation + extent


def _shape_obb(shape: CollisionShape, link_transform: np.ndarray) -> CollisionObb:
    """Return a fail-closed OBB around the declared collision geometry.

    Primitive boxes are represented exactly.  Cylinders, spheres and STL
    collisions are conservatively bounded; therefore an overlap is a collision
    *candidate* and can never be used to claim a false clearance.
    """

    transform = link_transform @ shape.origin
    if shape.kind == "mesh":
        assert shape.vertices is not None
        local_lower = shape.vertices.min(axis=0)
        local_upper = shape.vertices.max(axis=0)
        local_center = (local_lower + local_upper) * 0.5
        half_size = (local_upper - local_lower) * 0.5
    elif shape.kind == "box":
        local_center = np.zeros(3)
        half_size = np.asarray(shape.parameters) * 0.5
    elif shape.kind == "cylinder":
        radius, length = shape.parameters
        local_center = np.zeros(3)
        half_size = np.asarray((radius, radius, length * 0.5))
    elif shape.kind == "sphere":
        local_center = np.zeros(3)
        half_size = np.full(3, shape.parameters[0])
    else:  # pragma: no cover - parser rejects this
        raise ScanError(f"unhandled collision shape {shape.kind}")
    center = (transform @ np.asarray((*local_center, 1.0)))[:3]
    return CollisionObb(
        link=shape.link,
        name=shape.name,
        center=center,
        axes=transform[:3, :3],
        half_size=half_size,
        source=shape.source,
    )


def _obb_material_overlap(left: CollisionObb, right: CollisionObb, tolerance_m: float) -> float | None:
    """Return conservative minimum OBB penetration, or ``None`` if clear."""

    delta = right.center - left.center
    left_aabb_half = np.abs(left.axes) @ left.half_size
    right_aabb_half = np.abs(right.axes) @ right.half_size
    if np.any(left_aabb_half + right_aabb_half - np.abs(delta) <= tolerance_m):
        return None
    candidate_axes = [left.axes[:, index] for index in range(3)]
    candidate_axes.extend(right.axes[:, index] for index in range(3))
    candidate_axes.extend(
        np.cross(left.axes[:, left_index], right.axes[:, right_index])
        for left_index in range(3)
        for right_index in range(3)
    )
    minimum_penetration = math.inf
    for axis in candidate_axes:
        norm = float(np.linalg.norm(axis))
        if norm <= 1e-10:
            continue
        unit = axis / norm
        left_radius = float(np.sum(left.half_size * np.abs(left.axes.T @ unit)))
        right_radius = float(np.sum(right.half_size * np.abs(right.axes.T @ unit)))
        penetration = left_radius + right_radius - abs(float(np.dot(delta, unit)))
        if penetration <= tolerance_m:
            return None
        minimum_penetration = min(minimum_penetration, penetration)
    if not math.isfinite(minimum_penetration):
        raise ScanError("OBB overlap test produced no usable separating axes")
    return minimum_penetration


def _mesh_vertices_strictly_inside_box(
    mesh: CollisionShape,
    mesh_link_transform: np.ndarray,
    box: CollisionShape,
    box_link_transform: np.ndarray,
    clearance_m: float,
) -> int:
    """Count unique mesh vertices lying strictly inside an exact URDF box."""

    if mesh.kind != "mesh" or box.kind != "box":
        return 0
    assert mesh.vertices is not None
    mesh_transform = mesh_link_transform @ mesh.origin
    box_inverse = np.linalg.inv(box_link_transform @ box.origin)
    mesh_to_box = box_inverse @ mesh_transform
    local = mesh.vertices @ mesh_to_box[:3, :3].T + mesh_to_box[:3, 3]
    half = np.asarray(box.parameters) * 0.5
    inside = np.all(np.abs(local) < half - clearance_m, axis=1)
    if not np.any(inside):
        return 0
    return len(np.unique(np.round(local[inside], decimals=9), axis=0))


def _mesh_triangles_intersect_box(
    mesh: CollisionShape,
    mesh_link_transform: np.ndarray,
    box: CollisionShape,
    box_link_transform: np.ndarray,
    clearance_m: float,
) -> int:
    """Count STL triangles intersecting the interior of an exact URDF box.

    The test is an exact convex SAT test in the box-local frame.  Shrinking
    the box by ``clearance_m`` means a mere face touch or sub-tolerance CAD
    seam is not reported as material penetration.
    """

    if mesh.kind != "mesh" or box.kind != "box":
        return 0
    assert mesh.vertices is not None
    if len(mesh.vertices) % 3:
        raise ScanError(f"STL triangle vertex count is not divisible by three: {mesh.source}")
    mesh_transform = mesh_link_transform @ mesh.origin
    box_inverse = np.linalg.inv(box_link_transform @ box.origin)
    mesh_to_box = box_inverse @ mesh_transform
    local = mesh.vertices @ mesh_to_box[:3, :3].T + mesh_to_box[:3, 3]
    triangles = local.reshape(-1, 3, 3)
    half = np.asarray(box.parameters) * 0.5 - clearance_m
    if np.any(half <= 0.0):
        return 0

    count = 0
    box_axes = np.eye(3)
    for triangle in triangles:
        # Exact box face axes, with a cheap AABB reject first.
        if np.any(triangle.min(axis=0) > half) or np.any(triangle.max(axis=0) < -half):
            continue
        edges = (triangle[1] - triangle[0], triangle[2] - triangle[1], triangle[0] - triangle[2])
        axes = [np.cross(edges[0], edges[1])]
        axes.extend(np.cross(edge, axis) for edge in edges for axis in box_axes)
        separated = False
        for axis in axes:
            norm = float(np.linalg.norm(axis))
            if norm <= 1e-12:
                continue
            unit = axis / norm
            projection = triangle @ unit
            box_radius = float(np.dot(half, np.abs(unit)))
            if float(projection.min()) > box_radius or float(projection.max()) < -box_radius:
                separated = True
                break
        if not separated:
            count += 1
    return count


def _mesh_triangles_intersect_mesh(
    left: CollisionShape,
    left_link_transform: np.ndarray,
    right: CollisionShape,
    right_link_transform: np.ndarray,
) -> int:
    """Count triangle-pair intersections between two closed STL meshes.

    Each candidate pair uses the complete triangle/triangle separating-axis
    set: both face normals, all edge cross-products, and in-plane edge normals
    for the coplanar case.  AABB filtering keeps this exact required-anchor
    refinement bounded; the full raw joint sweep remains OBB fail-closed.
    """

    if left.kind != "mesh" or right.kind != "mesh":
        return 0
    assert left.vertices is not None and right.vertices is not None

    def world_triangles(shape: CollisionShape, transform: np.ndarray) -> np.ndarray:
        if len(shape.vertices) % 3:
            raise ScanError(f"STL triangle vertex count is not divisible by three: {shape.source}")
        full = transform @ shape.origin
        return (shape.vertices @ full[:3, :3].T + full[:3, 3]).reshape(-1, 3, 3)

    left_triangles = world_triangles(left, left_link_transform)
    right_triangles = world_triangles(right, right_link_transform)
    right_lower = right_triangles.min(axis=1)
    right_upper = right_triangles.max(axis=1)
    count = 0
    epsilon = 1e-10
    for triangle in left_triangles:
        lower, upper = triangle.min(axis=0), triangle.max(axis=0)
        candidates = np.flatnonzero(
            np.all(right_upper >= lower - epsilon, axis=1)
            & np.all(right_lower <= upper + epsilon, axis=1)
        )
        left_edges = (
            triangle[1] - triangle[0],
            triangle[2] - triangle[1],
            triangle[0] - triangle[2],
        )
        left_normal = np.cross(left_edges[0], left_edges[1])
        for index in candidates:
            other = right_triangles[index]
            right_edges = (
                other[1] - other[0],
                other[2] - other[1],
                other[0] - other[2],
            )
            right_normal = np.cross(right_edges[0], right_edges[1])
            axes = [left_normal, right_normal]
            axes.extend(np.cross(a, b) for a in left_edges for b in right_edges)
            # These axes are redundant for non-coplanar pairs and essential
            # for separating disjoint coplanar triangles.
            axes.extend(np.cross(left_normal, edge) for edge in left_edges)
            axes.extend(np.cross(right_normal, edge) for edge in right_edges)
            separated = False
            for axis in axes:
                norm = float(np.linalg.norm(axis))
                if norm <= 1e-12:
                    continue
                projection_left = triangle @ axis
                projection_right = other @ axis
                if (
                    float(projection_left.max()) < float(projection_right.min()) - epsilon
                    or float(projection_right.max()) < float(projection_left.min()) - epsilon
                ):
                    separated = True
                    break
            if not separated:
                count += 1
    return count


def _bounds(model: Model, transforms: dict[str, np.ndarray], links: set[str] | None = None) -> Bounds:
    result = Bounds.empty()
    count = 0
    for shape in model.shapes:
        if links is not None and shape.link not in links:
            continue
        lower, upper = _shape_bounds(shape, transforms[shape.link])
        result.include(lower, upper)
        count += 1
    if count == 0:
        raise ScanError("selected link set contains no collision geometry")
    return result


def _halton(index: int, base: int) -> float:
    fraction, factor = 0.0, 1.0 / base
    while index:
        fraction += factor * (index % base)
        index //= base
        factor /= base
    return fraction


def arm_samples(model: Model, halton_count: int) -> list[tuple[str, dict[str, float]]]:
    limits = []
    for name in ARM_JOINTS:
        joint = model.joints.get(name)
        if joint is None or joint.kind not in {"revolute", "continuous"}:
            raise ScanError(f"required arm joint is missing or non-rotary: {name}")
        if not math.isfinite(joint.lower) or not math.isfinite(joint.upper) or joint.upper <= joint.lower:
            raise ScanError(f"arm joint lacks usable limits: {name}")
        limits.append((joint.lower, joint.upper))

    raw: list[tuple[str, tuple[float, ...]]] = [("transport", TRANSPORT_POSE), ("zero", (0.0,) * 6)]
    raw.extend((name, pose) for name, pose in TASK_ANCHORS.items())
    for mask in range(1 << len(ARM_JOINTS)):
        raw.append((f"limit_corner_{mask:02d}", tuple(limits[axis][(mask >> axis) & 1] for axis in range(6))))
    for axis, (lower, upper) in enumerate(limits):
        for label, value in (("lower", lower), ("upper", upper)):
            pose = [0.0] * 6
            pose[axis] = value
            raw.append((f"axis_{axis}_{label}", tuple(pose)))
    for index in range(1, halton_count + 1):
        raw.append(
            (
                f"halton_{index:05d}",
                tuple(lower + _halton(index, base) * (upper - lower) for base, (lower, upper) in zip(HALTON_BASES, limits)),
            )
        )

    samples: list[tuple[str, dict[str, float]]] = []
    seen: set[tuple[float, ...]] = set()
    gripper = model.joints.get(GRIPPER_MASTER)
    gripper_positions = (0.0,) if gripper is None else (gripper.lower, (gripper.lower + gripper.upper) * 0.5, gripper.upper)
    multi_gripper_labels = {"transport", *TASK_ANCHORS}
    for index, (label, pose) in enumerate(raw):
        if any(value < lower - 1e-9 or value > upper + 1e-9 for value, (lower, upper) in zip(pose, limits)):
            raise ScanError(f"anchor {label} violates final URDF joint limits")
        variants = (
            tuple(enumerate(gripper_positions))
            if label in multi_gripper_labels
            else ((index % len(gripper_positions), gripper_positions[index % len(gripper_positions)]),)
        )
        for variant_index, gripper_position in variants:
            key = tuple(round(value, 12) for value in pose) + (round(gripper_position, 12),)
            if key in seen:
                continue
            seen.add(key)
            positions = dict(zip(ARM_JOINTS, pose))
            positions[GRIPPER_MASTER] = gripper_position
            positions.update(TASK_AUXILIARY_POSITIONS.get(label, {}))
            variant_label = label
            if label in multi_gripper_labels and variant_index:
                variant_label += "_gripper_half_closed" if variant_index == 1 else "_gripper_closed"
            samples.append((variant_label, positions))
    return samples


def _joint_graph_distance(model: Model, start: str, goal: str, maximum: int = 2) -> int | None:
    if start == goal:
        return 0
    neighbours: dict[str, set[str]] = {name: set() for name in model.links}
    for joint in model.joints.values():
        neighbours[joint.parent].add(joint.child)
        neighbours[joint.child].add(joint.parent)
    frontier = {start}
    visited = {start}
    for distance in range(1, maximum + 1):
        frontier = {next_link for link in frontier for next_link in neighbours[link] if next_link not in visited}
        if goal in frontier:
            return distance
        visited.update(frontier)
    return None


def _collision_audit(
    model: Model,
    samples: list[tuple[str, dict[str, float]]],
    arm_links: set[str],
    *,
    tolerance_m: float = 0.002,
) -> dict[str, Any]:
    """Conservatively scan arm/self and arm/vehicle collision candidates.

    Direct and next-nearest kinematic neighbours are excluded because their
    collision meshes intentionally meet at the mechanical joint.  The only
    static mounting exclusion is the arm base against its dedicated adapter;
    bodywork, sensor, cleaning and storage geometry is never allow-listed.
    """

    static_links = set(model.links) - arm_links
    static_shapes = [shape for shape in model.shapes if shape.link in static_links]
    arm_pair_exclusions = {
        tuple(sorted((left, right)))
        for left in arm_links
        for right in arm_links
        if left != right and _joint_graph_distance(model, left, right, maximum=2) is not None
    }
    arm_pair_exclusions.update(KNOWN_ASSEMBLY_CONTACT_PAIRS)
    mounting_exclusions = {
        tuple(sorted(("ur5e_base_link_inertia", "arm_mount_link"))),
    }
    required_anchors = {"transport", *TASK_ANCHORS}
    sample_candidate_counts: dict[str, int] = {}
    sample_blocking_counts: dict[str, int] = {}
    sample_exact_disproved_counts: dict[str, int] = {}
    pair_hit_counts: dict[tuple[str, str, str, str, str], int] = {}
    worst_candidate_heap: list[tuple[float, int, dict[str, Any]]] = []
    candidate_sequence = 0
    confirmed_candidate_count = 0
    confirmed_candidate_examples: list[dict[str, Any]] = []
    confirmed_candidate_counts_by_anchor = {
        name: 0 for name in sorted(required_anchors)
    }
    maximum_penetration = 0.0

    def record_confirmed_candidate(candidate: dict[str, Any]) -> None:
        nonlocal confirmed_candidate_count
        confirmed_candidate_count += 1
        if len(confirmed_candidate_examples) < 50:
            confirmed_candidate_examples.append(candidate)
        sample = str(candidate["sample"])
        for anchor in confirmed_candidate_counts_by_anchor:
            if sample == anchor or sample.startswith(anchor + "_gripper_"):
                confirmed_candidate_counts_by_anchor[anchor] += 1

    for label, positions in samples:
        required_sample = any(
            label == name or label.startswith(name + "_gripper_")
            for name in required_anchors
        )
        transforms = model.transforms(positions)
        static_obbs = [
            (shape, _shape_obb(shape, transforms[shape.link]))
            for shape in static_shapes
        ]
        arm_obbs = [
            (shape, _shape_obb(shape, transforms[shape.link]))
            for shape in model.shapes
            if shape.link in arm_links
        ]
        sample_hits: list[dict[str, Any]] = []
        sample_blocking_count = 0
        sample_exact_disproved_count = 0

        for left_index, (left_shape, left) in enumerate(arm_obbs):
            for right_shape, right in arm_obbs[left_index + 1 :]:
                link_pair = tuple(sorted((left.link, right.link)))
                if left.link == right.link or link_pair in arm_pair_exclusions:
                    continue
                penetration = _obb_material_overlap(left, right, tolerance_m)
                if penetration is not None:
                    hit = {
                        "class": "arm_self",
                        "left_link": left.link,
                        "left_collision": left.name,
                        "right_link": right.link,
                        "right_collision": right.name,
                        "conservative_obb_penetration_m": penetration,
                    }
                    sample_hits.append(hit)
                    triangle_count = 0
                    if required_sample and left_shape.kind == "mesh" and right_shape.kind == "mesh":
                        triangle_count = _mesh_triangles_intersect_mesh(
                            left_shape,
                            transforms[left_shape.link],
                            right_shape,
                            transforms[right_shape.link],
                        )
                        if triangle_count == 0:
                            sample_exact_disproved_count += 1
                        else:
                            sample_blocking_count += 1
                            record_confirmed_candidate(
                                {
                                    "sample": label,
                                    **hit,
                                    "exact_mesh_triangle_pair_intersection_count": triangle_count,
                                }
                            )
                    elif required_sample and {left_shape.kind, right_shape.kind} == {"mesh", "box"}:
                        mesh_shape, mesh_transform, box_shape, box_transform = (
                            (left_shape, transforms[left_shape.link], right_shape, transforms[right_shape.link])
                            if left_shape.kind == "mesh"
                            else (right_shape, transforms[right_shape.link], left_shape, transforms[left_shape.link])
                        )
                        triangle_count = _mesh_triangles_intersect_box(
                            mesh_shape,
                            mesh_transform,
                            box_shape,
                            box_transform,
                            tolerance_m,
                        )
                        if triangle_count == 0:
                            sample_exact_disproved_count += 1
                        else:
                            sample_blocking_count += 1
                            record_confirmed_candidate(
                                {
                                    "sample": label,
                                    **hit,
                                    "exact_mesh_triangles_intersecting_box": triangle_count,
                                }
                            )
                    else:
                        sample_blocking_count += 1

        for left_shape, left in arm_obbs:
            for right_shape, right in static_obbs:
                link_pair = tuple(sorted((left.link, right.link)))
                if link_pair in mounting_exclusions:
                    continue
                penetration = _obb_material_overlap(left, right, tolerance_m)
                if penetration is not None:
                    hit = {
                        "class": "arm_vehicle",
                        "left_link": left.link,
                        "left_collision": left.name,
                        "right_link": right.link,
                        "right_collision": right.name,
                        "conservative_obb_penetration_m": penetration,
                    }
                    sample_hits.append(hit)
                    triangle_count = 0
                    if required_sample and left_shape.kind == "mesh" and right_shape.kind == "box":
                        triangle_count = _mesh_triangles_intersect_box(
                            left_shape,
                            transforms[left_shape.link],
                            right_shape,
                            transforms[right_shape.link],
                            tolerance_m,
                        )
                        if triangle_count == 0:
                            sample_exact_disproved_count += 1
                        else:
                            sample_blocking_count += 1
                    else:
                        sample_blocking_count += 1
                    inside_count = _mesh_vertices_strictly_inside_box(
                        left_shape,
                        transforms[left_shape.link],
                        right_shape,
                        transforms[right_shape.link],
                        tolerance_m,
                    )
                    if triangle_count or inside_count:
                        record_confirmed_candidate(
                            {
                                "sample": label,
                                **hit,
                                "unique_arm_mesh_vertices_strictly_inside_vehicle_box": inside_count,
                                "exact_mesh_triangles_intersecting_vehicle_box": triangle_count,
                            }
                        )

        sample_candidate_counts[label] = len(sample_hits)
        sample_blocking_counts[label] = sample_blocking_count
        sample_exact_disproved_counts[label] = sample_exact_disproved_count
        for hit in sample_hits:
            penetration = float(hit["conservative_obb_penetration_m"])
            maximum_penetration = max(maximum_penetration, penetration)
            key = (
                str(hit["class"]),
                str(hit["left_link"]),
                str(hit["left_collision"]),
                str(hit["right_link"]),
                str(hit["right_collision"]),
            )
            pair_hit_counts[key] = pair_hit_counts.get(key, 0) + 1
            _push_bounded_worst_candidate(
                worst_candidate_heap,
                {"sample": label, **hit},
                candidate_sequence,
            )
            candidate_sequence += 1

    worst_candidates = _sorted_bounded_worst_candidates(worst_candidate_heap)
    anchor_candidate_counts = {
        name: max(
            (
                count
                for label, count in sample_candidate_counts.items()
                if label == name or label.startswith(name + "_gripper_")
            ),
            default=-1,
        )
        for name in sorted(required_anchors)
    }
    anchor_counts = {
        name: max(
            (
                count
                for label, count in sample_blocking_counts.items()
                if label == name or label.startswith(name + "_gripper_")
            ),
            default=-1,
        )
        for name in sorted(required_anchors)
    }
    anchor_exact_disproved_counts = {
        name: sum(
            count
            for label, count in sample_exact_disproved_counts.items()
            if label == name or label.startswith(name + "_gripper_")
        )
        for name in sorted(required_anchors)
    }
    anchor_confirmed_counts = confirmed_candidate_counts_by_anchor
    unique_pairs = [
        {
            "class": key[0],
            "left_link": key[1],
            "left_collision": key[2],
            "right_link": key[3],
            "right_collision": key[4],
            "sample_hit_count": count,
        }
        for key, count in sorted(pair_hit_counts.items(), key=lambda item: (-item[1], item[0]))
    ]
    return {
        "method": "fail-closed collision-geometry OBB SAT broad phase",
        "material_overlap_tolerance_m": tolerance_m,
        "sample_count": len(samples),
        "samples_with_candidates": sum(count > 0 for count in sample_candidate_counts.values()),
        "candidate_event_count": sum(sample_candidate_counts.values()),
        "confirmed_mesh_vertex_inside_box_event_count": confirmed_candidate_count,
        "unique_candidate_pair_count": len(pair_hit_counts),
        "maximum_conservative_obb_penetration_m": round(maximum_penetration, 9),
        "required_anchor_candidate_counts": anchor_candidate_counts,
        "required_anchor_blocking_counts": anchor_counts,
        "required_anchor_exact_disproved_candidate_counts": anchor_exact_disproved_counts,
        "required_anchor_confirmed_penetration_counts": anchor_confirmed_counts,
        "required_anchors_clear": all(count == 0 for count in anchor_counts.values()),
        "all_sampled_configurations_clear": all(count == 0 for count in sample_blocking_counts.values()),
        "top_candidates": [
            {
                **item,
                "conservative_obb_penetration_m": round(
                    float(item["conservative_obb_penetration_m"]), 9
                ),
            }
            for item in worst_candidates[:50]
        ],
        "confirmed_penetration_examples": confirmed_candidate_examples,
        "unique_candidates": unique_pairs[:100],
        "excluded_pair_policy": {
            "arm_links_at_joint_graph_distance_le_2": len(arm_pair_exclusions),
            "dedicated_mounting_pairs": [list(pair) for pair in sorted(mounting_exclusions)],
            "bodywork_sensor_storage_cleaning_pairs_allowlisted": 0,
        },
        "interpretation": (
            "An overlap is conservative because cylinders, spheres and STL collision meshes are bounded "
            "by oriented boxes. Required-anchor STL-versus-box candidates are resolved by an exact "
            "triangle-box SAT test; exact-disproved broad-phase candidates do not block that anchor. "
            "All other candidates remain fail-closed and block a collision-free claim."
        ),
    }


def _convex_hull(points: list[tuple[float, float]]) -> list[tuple[float, float]]:
    points = sorted(set(points))
    if len(points) < 3:
        raise ScanError("support polygon requires at least three distinct wheel centres")

    def cross(origin, left, right):
        return (left[0] - origin[0]) * (right[1] - origin[1]) - (left[1] - origin[1]) * (right[0] - origin[0])

    lower: list[tuple[float, float]] = []
    for point in points:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], point) <= 0:
            lower.pop()
        lower.append(point)
    upper: list[tuple[float, float]] = []
    for point in reversed(points):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], point) <= 0:
            upper.pop()
        upper.append(point)
    return lower[:-1] + upper[:-1]


def _signed_margin(point: np.ndarray, polygon: list[tuple[float, float]]) -> float:
    margins = []
    for start, end in zip(polygon, polygon[1:] + polygon[:1]):
        dx, dy = end[0] - start[0], end[1] - start[1]
        margins.append((dx * (point[1] - start[1]) - dy * (point[0] - start[0])) / math.hypot(dx, dy))
    return min(margins)


def _load_override(load_case: str, wet_capacity_kg: float = WASTEWATER_CAPACITY_KG) -> dict[str, tuple[float, np.ndarray]]:
    dry = DRY_CAPACITY_KG if load_case in {"max_dry", "max_combined"} else 0.0
    wet = wet_capacity_kg if load_case in {"max_wastewater", "max_combined"} else 0.0
    fill_height = wet / (1000.0 * 0.350 * 0.250) if wet > 0 else 0.0
    return {
        "dry_bin_payload_reserve_link": (dry, np.asarray((0.0, 0.0, 0.0))),
        "wastewater_payload_reserve_link": (wet, np.asarray((0.0, 0.0, fill_height * 0.5))),
    }


def _mass_moment(
    model: Model,
    transforms: dict[str, np.ndarray],
    overrides: dict[str, tuple[float, np.ndarray]],
) -> tuple[float, np.ndarray]:
    total, moment = 0.0, np.zeros(3)
    for link, inertial in model.inertials.items():
        if link in overrides:
            mass, local = overrides[link]
        else:
            mass, local = inertial.mass, inertial.origin[:3, 3]
        if mass <= 0.0:
            continue
        world = transforms[link] @ np.asarray((local[0], local[1], local[2], 1.0))
        total += mass
        moment += mass * world[:3]
    if total <= 0.0:
        raise ScanError("total vehicle mass is not positive")
    return total, moment


def _rounded_vector(vector: np.ndarray) -> list[float]:
    return [round(float(value), 9) for value in vector]


def _static_sensor_structure_audit(model: Model, tolerance_m: float = 0.002) -> dict[str, Any]:
    """Verify the final mast/sensor/compute interfaces from collision geometry."""

    transforms = model.transforms({})
    groups = {
        "utm": {"lidar_2d_mount_link", "lidar_2d_link"},
        "mid360": {"lidar_3d_mount_link", "lidar_3d_link"},
        "compute": {"s100_compute_enclosure_link"},
        "control_cabinet": {"ur5e_control_box_link"},
    }
    shape_groups = {
        name: [shape for shape in model.shapes if shape.link in links]
        for name, links in groups.items()
    }
    shape_groups["pylon_fairing"] = [
        shape
        for shape in model.shapes
        if shape.link == "bodywork_rear_shell_link" and shape.name.startswith("pylon_fairing_")
    ]
    if any(not shapes for shapes in shape_groups.values()):
        missing = sorted(name for name, shapes in shape_groups.items() if not shapes)
        raise ScanError("static sensor structure audit lacks collision groups: " + ", ".join(missing))

    def candidates(left_name: str, right_name: str) -> list[dict[str, Any]]:
        result = []
        for left in shape_groups[left_name]:
            left_obb = _shape_obb(left, transforms[left.link])
            for right in shape_groups[right_name]:
                right_obb = _shape_obb(right, transforms[right.link])
                penetration = _obb_material_overlap(left_obb, right_obb, tolerance_m)
                if penetration is not None:
                    result.append(
                        {
                            "left_link": left.link,
                            "left_collision": left.name,
                            "right_link": right.link,
                            "right_collision": right.name,
                            "conservative_obb_penetration_m": round(float(penetration), 9),
                        }
                    )
        return result

    fairing_top = max(
        float(_shape_bounds(shape, transforms[shape.link])[1][2])
        for shape in shape_groups["pylon_fairing"]
    )
    utm_plane = float(transforms["lidar_2d_link"][2, 3])
    interfaces = {
        "utm_vs_mid360": candidates("utm", "mid360"),
        "utm_vs_pylon_fairing": candidates("utm", "pylon_fairing"),
        "compute_vs_control_cabinet": candidates("compute", "control_cabinet"),
    }
    passed = bool(
        utm_plane - fairing_top >= 0.04
        and all(not hits for hits in interfaces.values())
    )
    return {
        "passed": passed,
        "material_overlap_tolerance_m": tolerance_m,
        "utm_measurement_plane_z_m": round(utm_plane, 9),
        "maximum_pylon_fairing_z_m": round(fairing_top, 9),
        "utm_plane_above_fairing_clearance_m": round(utm_plane - fairing_top, 9),
        "minimum_required_scan_plane_clearance_m": 0.04,
        "interface_candidate_counts": {
            name: len(hits) for name, hits in interfaces.items()
        },
        "interface_candidates": interfaces,
        "interpretation": (
            "Zero conservative OBB candidates proves separation for the final UTM, MID-360, "
            "pylon-fairing and roof-mounted compute/control-cabinet interfaces."
        ),
    }


def scan(
    model: Model,
    layout: dict[str, Any],
    halton_count: int,
    *,
    layout_path: Path = DEFAULT_LAYOUT,
) -> dict[str, Any]:
    samples = arm_samples(model, halton_count)
    arm_links = model.descendants("ur5e_base_link_inertia")
    static_links = set(model.links) - arm_links
    transport_positions = dict(zip(ARM_JOINTS, TRANSPORT_POSE))
    transport_positions[GRIPPER_MASTER] = 0.20
    transport_transforms = model.transforms(transport_positions)
    zero_transforms = model.transforms({})

    wheel_points = []
    for name in ("front_left_wheel_joint", "front_right_wheel_joint", "rear_left_wheel_joint", "rear_right_wheel_joint"):
        joint = model.joints.get(name)
        if joint is None:
            raise ScanError(f"support wheel joint missing: {name}")
        centre = zero_transforms[joint.child][:3, 3]
        wheel_points.append((float(centre[0]), float(centre[1])))
    support_polygon = _convex_hull(wheel_points)

    transport_arm = _bounds(model, transport_transforms, arm_links)
    transport_vehicle = _bounds(model, transport_transforms)
    static_vehicle = _bounds(model, zero_transforms, static_links)
    deployed_arm = Bounds.empty()
    operating_vehicle = Bounds(static_vehicle.lower.copy(), static_vehicle.upper.copy())
    worst: dict[str, Any] | None = None
    load_cases = ("empty", "max_dry", "max_wastewater", "max_combined")
    load_summaries = {case: {"minimum_margin_m": math.inf, "maximum_cog_z_m": -math.inf, "minimum_cog_z_m": math.inf} for case in load_cases}
    base_moments: list[tuple[str, float, np.ndarray, dict[str, float]]] = []
    arm_ground_clearances: dict[str, float] = {}

    for label, positions in samples:
        transforms = model.transforms(positions)
        arm_bounds = _bounds(model, transforms, arm_links)
        arm_ground_clearances[label] = float(arm_bounds.lower[2])
        deployed_arm.include(arm_bounds.lower, arm_bounds.upper)
        operating_vehicle.include(arm_bounds.lower, arm_bounds.upper)
        for load_case in load_cases:
            total, moment = _mass_moment(model, transforms, _load_override(load_case))
            cog = moment / total
            margin = _signed_margin(cog[:2], support_polygon)
            summary = load_summaries[load_case]
            summary["minimum_margin_m"] = min(summary["minimum_margin_m"], margin)
            summary["maximum_cog_z_m"] = max(summary["maximum_cog_z_m"], float(cog[2]))
            summary["minimum_cog_z_m"] = min(summary["minimum_cog_z_m"], float(cog[2]))
            summary["total_mass_kg"] = total
            if worst is None or margin < worst["static_margin_m"]:
                worst = {
                    "sample": label,
                    "load_case": load_case,
                    "joint_positions_rad": {name: round(float(positions[name]), 9) for name in ARM_JOINTS},
                    "total_mass_kg": round(total, 9),
                    "cog_xyz_m": _rounded_vector(cog),
                    "static_margin_m": round(margin, 9),
                }
        total, moment = _mass_moment(model, transforms, _load_override("max_dry"))
        base_moments.append((label, total, moment, positions))

    wet_install_l = float(layout["storage"]["wastewater_tank"]["geometric_installation_limit_l"])
    wet_link_transform = zero_transforms["wastewater_payload_reserve_link"]

    def minimum_margin_for_water(volume_l: float) -> float:
        wet_mass = volume_l * WATER_DENSITY_KG_L
        fill_height = wet_mass / (1000.0 * 0.350 * 0.250) if wet_mass > 0 else 0.0
        local = np.asarray((0.0, 0.0, fill_height * 0.5, 1.0))
        wet_world = (wet_link_transform @ local)[:3]
        return min(_signed_margin(((moment + wet_mass * wet_world) / (total + wet_mass))[:2], support_polygon) for _, total, moment, _ in base_moments)

    if minimum_margin_for_water(wet_install_l) >= MINIMUM_STATIC_MARGIN_M:
        cog_limit_l = wet_install_l
    else:
        low, high = 0.0, wet_install_l
        for _ in range(60):
            middle = (low + high) * 0.5
            if minimum_margin_for_water(middle) >= MINIMUM_STATIC_MARGIN_M:
                low = middle
            else:
                high = middle
        cog_limit_l = low

    design_cap_l = 20.0
    usable_fraction = float(layout["storage"]["wastewater_tank"]["usable_fraction"])
    empty_total_mass_kg = float(load_summaries["empty"]["total_mass_kg"])
    fixed_payload_mass_kg = empty_total_mass_kg - A300_CURB_MASS_KG
    mass_limited_usable_l = max(
        0.0,
        (
            A300_PAYLOAD_DESIGN_LIMIT_KG
            - fixed_payload_mass_kg
            - DRY_CAPACITY_KG
        )
        / WATER_DENSITY_KG_L,
    )
    mass_limit_l = mass_limited_usable_l / usable_fraction
    nominal_limit_l = min(mass_limit_l, design_cap_l, wet_install_l, cog_limit_l)
    theoretical_usable_l = nominal_limit_l * usable_fraction
    configured_usable_l = float(
        layout["storage"]["wastewater_tank"]["final_usable_capacity_l"]
    )
    final_usable_l = min(theoretical_usable_l, configured_usable_l)
    max_combined_payload_kg = (
        float(load_summaries["max_combined"]["total_mass_kg"])
        - A300_CURB_MASS_KG
    )
    payload_design_margin_kg = (
        A300_PAYLOAD_DESIGN_LIMIT_KG - max_combined_payload_kg
    )
    final_payload_budget_pass = bool(
        configured_usable_l <= theoretical_usable_l + 1e-9
        and abs(configured_usable_l - WASTEWATER_CAPACITY_KG) <= 1e-9
        and payload_design_margin_kg >= -1e-9
    )
    min_margin = min(float(summary["minimum_margin_m"]) for summary in load_summaries.values())
    full_inertia_pass = min_margin >= MINIMUM_STATIC_MARGIN_M
    envelope_scan_complete = bool(
        len(samples) >= halton_count
        and all(shape.kind != "mesh" or shape.vertices is not None for shape in model.shapes)
        and transport_vehicle.upper[2] > transport_vehicle.lower[2]
    )
    collision_audit = _collision_audit(model, samples, arm_links)
    sensor_structure_audit = _static_sensor_structure_audit(model)
    sampled_collision_clear = bool(collision_audit["all_sampled_configurations_clear"])
    required_anchors_clear = bool(collision_audit["required_anchors_clear"])
    required_ground_clearances = {
        name: round(arm_ground_clearances[name], 9)
        for name in ("transport", *TASK_ANCHORS)
    }
    required_ground_clear = bool(
        all(clearance >= -0.002 for clearance in required_ground_clearances.values())
    )
    overall_pass = bool(
        full_inertia_pass
        and envelope_scan_complete
        and required_anchors_clear
        and required_ground_clear
        and sensor_structure_audit["passed"]
        and final_payload_budget_pass
    )
    assert worst is not None
    mesh_shapes = [shape for shape in model.shapes if shape.kind == "mesh"]
    return {
        "schema_version": 1,
        "report_id": "tzcup_formal_vehicle_inertia_collision_sweep_v1",
        "status": (
            "PRODUCTION_ANCHORS_PASSED_WITH_RAW_JOINT_SPACE_EXCLUSION_REGIONS"
            if overall_pass and not sampled_collision_clear
            else "FORMAL_INERTIA_COG_AND_COLLISION_MESH_SWEEP_PASSED"
            if overall_pass
            else "FAILED"
        ),
        "passed": overall_pass,
        "checks": {
            "full_inertia_and_cog_scan": bool(full_inertia_pass),
            "arm_sampled_collision_envelope_complete": envelope_scan_complete,
            "arm_sampled_self_and_vehicle_collision_clear": sampled_collision_clear,
            "raw_joint_space_exclusion_regions_detected": bool(not sampled_collision_clear),
            "transport_and_task_anchors_collision_clear": required_anchors_clear,
            "arm_exact_continuous_swept_volume": False,
            "all_raw_joint_samples_above_ground": bool(
                all(clearance >= -0.002 for clearance in arm_ground_clearances.values())
            ),
            "transport_and_task_anchors_above_ground": bool(
                required_ground_clear
            ),
            "static_sensor_structure_clear": bool(sensor_structure_audit["passed"]),
            "every_physical_inertial_used": bool(len(model.inertials) == len(model.links) - 2),
            "collision_geometry_only_no_visual_fallback": True,
            "minimum_static_margin_met": bool(full_inertia_pass),
            "wastewater_cog_limit_resolved": bool(cog_limit_l > 0.0),
            "final_payload_within_a300_design_limit": final_payload_budget_pass,
        },
        "inputs": {
            "expanded_urdf": model.urdf_path.resolve().as_posix(),
            "expanded_urdf_sha256": hashlib.sha256(model.urdf_path.read_bytes()).hexdigest(),
            "layout": layout_path.resolve().relative_to(ROOT).as_posix(),
            "layout_sha256": hashlib.sha256(layout_path.read_bytes()).hexdigest(),
            "scanner": Path(__file__).resolve().relative_to(ROOT).as_posix(),
            "scanner_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            "support_polygon_source": "final URDF wheel-joint centres",
            "load_source": "formal Xacro payload reserve links and final capacity clamps",
        },
        "scan_scale": {
            "link_count": len(model.links),
            "inertial_link_count": len(model.inertials),
            "collision_element_count": len(model.shapes),
            "collision_mesh_count": len(mesh_shapes),
            "collision_mesh_vertex_references": sum(len(shape.vertices) for shape in mesh_shapes if shape.vertices is not None),
            "visual_element_count_ignored": model.visual_count,
            "visual_fallback_count": 0,
            "halton_requested": halton_count,
            "arm_pose_count": len(samples),
            "load_case_count": len(load_cases),
            "cog_evaluation_count": len(samples) * len(load_cases),
        },
        "joint_space": {
            "arm_joints": list(ARM_JOINTS),
            "limits_rad": {name: [model.joints[name].lower, model.joints[name].upper] for name in ARM_JOINTS},
            "transport_pose_rad": dict(zip(ARM_JOINTS, TRANSPORT_POSE)),
            "task_anchor_names": sorted(TASK_ANCHORS),
            "sampling": "transport + zero + task anchors + 64 limit corners + one-axis limits + Halton",
        },
        "support_polygon_xy_m": [[round(x, 6), round(y, 6)] for x, y in support_polygon],
        "minimum_required_static_margin_m": MINIMUM_STATIC_MARGIN_M,
        "load_cases": {
            case: {key: round(float(value), 9) for key, value in summary.items()}
            for case, summary in load_summaries.items()
        },
        "payload_budget": {
            "a300_curb_mass_kg": A300_CURB_MASS_KG,
            "a300_rated_payload_kg": A300_RATED_PAYLOAD_KG,
            "a300_payload_design_limit_kg": A300_PAYLOAD_DESIGN_LIMIT_KG,
            "final_empty_vehicle_mass_kg": round(empty_total_mass_kg, 9),
            "final_fixed_payload_mass_kg": round(fixed_payload_mass_kg, 9),
            "maximum_dry_trash_mass_kg": DRY_CAPACITY_KG,
            "configured_wastewater_capacity_kg": WASTEWATER_CAPACITY_KG,
            "maximum_combined_payload_kg": round(max_combined_payload_kg, 9),
            "remaining_design_margin_kg": round(payload_design_margin_kg, 9),
            "remaining_rated_payload_margin_kg": round(
                A300_RATED_PAYLOAD_KG - max_combined_payload_kg, 9
            ),
        },
        "worst_case": worst,
        "collision_audit": collision_audit,
        "static_sensor_structure_audit": sensor_structure_audit,
        "ground_plane_audit": {
            "plane_z_m": 0.0,
            "material_penetration_tolerance_m": 0.002,
            "minimum_arm_collision_z_m": round(min(arm_ground_clearances.values()), 9),
            "samples_below_ground": sum(
                clearance < -0.002 for clearance in arm_ground_clearances.values()
            ),
            "required_anchor_minimum_z_m": required_ground_clearances,
        },
        "envelopes": {
            "arm_transport_collision": transport_arm.payload(),
            "vehicle_transport_collision": transport_vehicle.payload(),
            "arm_reachable_sampled_collision_union": deployed_arm.payload(),
            "vehicle_arm_operating_collision_union": operating_vehicle.payload(),
        },
        "wastewater_capacity": {
            "mass_limit_l": mass_limit_l,
            "installation_limit_l": wet_install_l,
            "cog_limit_l": round(cog_limit_l, 6),
            "design_cap_l": design_cap_l,
            "usable_fraction": usable_fraction,
            "theoretical_usable_capacity_l": round(theoretical_usable_l, 6),
            "configured_usable_capacity_l": round(configured_usable_l, 6),
            "final_usable_capacity_l": round(final_usable_l, 6),
            "minimum_margin_at_installation_limit_m": round(minimum_margin_for_water(wet_install_l), 9),
        },
        "claim_boundary": {
            "passed": (
                "All final-URDF inertial masses/CoGs and collision geometry were scanned at the reported "
                "deterministic joint samples. Transport, pregrasp, pick and deposit are exact-refined clear "
                "for open, half-closed and closed gripper states; raw collision regions remain planner exclusions."
            ),
            "not_proved": [
                "continuous unsampled joint-space extrema between samples",
                "triangle-level clearance for conservative OBB candidates outside production anchors",
                "MoveIt collision-free reachability or online collision avoidance",
                "dynamic tip-over during braking, turning, slopes or liquid slosh",
                "tyre compliance/contact-patch migration",
                "manufacturing tolerances and real weighed component CoGs",
            ],
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    parser.add_argument("--layout", type=Path, default=DEFAULT_LAYOUT)
    parser.add_argument("--halton-count", type=int, default=1024)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.halton_count < 64:
        parser.error("--halton-count must be at least 64")
    layout = yaml.safe_load(args.layout.read_text(encoding="utf-8"))
    result = scan(
        Model(args.urdf),
        layout,
        args.halton_count,
        layout_path=args.layout,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
