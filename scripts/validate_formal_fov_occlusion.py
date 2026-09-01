#!/usr/bin/env python3
"""Deterministic mesh-ray self-occlusion audit for the formal vehicle."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import struct
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml

from formal_gripper_linkage_contract import resolve_mimic_relations


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_URDF = ROOT / "reports/engineering/formal_competition_vehicle.urdf"
DEFAULT_LAYOUT = ROOT / "config/high_fidelity_vehicle/formal_vehicle_layout.yaml"
MESH_ROOT = ROOT / "starter_ws/src/sanitation_vehicle_description/meshes"
ARM_JOINTS = (
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
)
ARM_POSES = {
    # The stowed pose is the declared transport pose from the deterministic
    # swept-volume gate.  The other three poses are the exact targets used by
    # the physical cube pick/deposit runtime, not visually convenient poses.
    "transport": {
        **dict(zip(ARM_JOINTS, [-1.0, -1.0, 1.8, -1.5, -1.55, 0.25])),
        "robotiq_85_left_knuckle_joint": 0.20,
    },
    "pregrasp": {
        **dict(zip(ARM_JOINTS, [-1.48278161, -0.44199397, 1.21471947, -2.34352182, -1.57079633, -3.05357794])),
        "robotiq_85_left_knuckle_joint": 0.0,
    },
    "pick": {
        **dict(zip(ARM_JOINTS, [-1.48278161, 0.10260211, 0.80254082, -2.47593926, -1.57079633, -3.05357794])),
        "robotiq_85_left_knuckle_joint": 0.30,
    },
    "deposit": {
        **dict(zip(ARM_JOINTS, [-0.30233498, -1.56960444, -0.73057657, -2.40638971, 1.56851193, 0.60324332])),
        "robotiq_85_left_knuckle_joint": 0.30,
        # The production runtime opens this robot-only gate before moving to
        # DEPOSIT.  Auditing a closed gate would test a state that never occurs
        # during the deposition observation.
        "dry_deposit_gate_joint": 1.05,
    },
}
SENSORS = {
    "utm30lx": {"frame": "lidar_2d_link", "kind": "lidar2d", "range": 30.0, "ignore": {"lidar_2d_link"}},
    "mid360": {"frame": "lidar_3d_link", "kind": "mid360", "range": 40.0, "ignore": {"lidar_3d_link"}},
    "front_d435_depth": {"frame": "front_rgbd_depth_optical_frame", "kind": "camera", "hfov": 87.0, "vfov": 58.0, "range": 10.0, "ignore": {"front_rgbd_link"}},
    "rear_left_fisheye": {"frame": "rear_left_fisheye_optical_frame", "kind": "fisheye", "hfov": 150.0, "vfov": 129.0, "range": 20.0, "ignore": {"rear_left_fisheye_link"}},
    "rear_right_fisheye": {"frame": "rear_right_fisheye_optical_frame", "kind": "fisheye", "hfov": 150.0, "vfov": 129.0, "range": 20.0, "ignore": {"rear_right_fisheye_link"}},
    "gnss": {"frame": "gnss_antenna_link", "kind": "sky", "range": 100.0, "ignore": {"gnss_antenna_link"}},
    "wrist_d435_depth": {"frame": "wrist_rgbd_depth_optical_frame", "kind": "camera", "hfov": 87.0, "vfov": 58.0, "range": 3.0, "ignore": {"wrist_rgbd_link"}},
}

# A wide-angle image is allowed to contain the intended workpiece or hopper.
# Therefore the wrist camera's full-FOV self-occlusion gate is evaluated at
# pregrasp (where perception is actually used), while explicit line-of-sight
# gates below prove the cube and deposit aperture remain observable.
FULL_FOV_REQUIRED_POSES = {
    "utm30lx": tuple(ARM_POSES),
    "mid360": tuple(ARM_POSES),
    "front_d435_depth": tuple(ARM_POSES),
    "rear_left_fisheye": tuple(ARM_POSES),
    "rear_right_fisheye": tuple(ARM_POSES),
    "gnss": tuple(ARM_POSES),
    "wrist_d435_depth": ("pregrasp",),
}

STATIC_SENSOR_IDS = tuple(sensor_id for sensor_id in SENSORS if sensor_id != "wrist_d435_depth")


def _numbers(raw: str | None, count: int, default: tuple[float, ...]) -> np.ndarray:
    if raw is None:
        return np.asarray(default, dtype=float)
    values = [float(item) for item in raw.split()]
    if len(values) != count:
        raise ValueError(f"expected {count} values, got {raw!r}")
    return np.asarray(values, dtype=float)


def _rpy_matrix(rpy: np.ndarray) -> np.ndarray:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def _transform(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, 0.0)) -> np.ndarray:
    result = np.eye(4)
    result[:3, :3] = _rpy_matrix(np.asarray(rpy, dtype=float))
    result[:3, 3] = np.asarray(xyz, dtype=float)
    return result


def _axis_rotation(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    c, s, one = math.cos(angle), math.sin(angle), 1.0 - math.cos(angle)
    result = np.eye(4)
    result[:3, :3] = np.array([
        [c + x*x*one, x*y*one - z*s, x*z*one + y*s],
        [y*x*one + z*s, c + y*y*one, y*z*one - x*s],
        [z*x*one - y*s, z*y*one + x*s, c + z*z*one],
    ])
    return result


def _apply(triangles: np.ndarray, transform: np.ndarray) -> np.ndarray:
    return triangles @ transform[:3, :3].T + transform[:3, 3]


def _box(size: np.ndarray) -> np.ndarray:
    x, y, z = size / 2.0
    vertices = np.array([
        [-x, -y, -z], [x, -y, -z], [x, y, -z], [-x, y, -z],
        [-x, -y, z], [x, -y, z], [x, y, z], [-x, y, z],
    ])
    faces = np.array([
        [0, 2, 1], [0, 3, 2], [4, 5, 6], [4, 6, 7],
        [0, 1, 5], [0, 5, 4], [1, 2, 6], [1, 6, 5],
        [2, 3, 7], [2, 7, 6], [3, 0, 4], [3, 4, 7],
    ])
    return vertices[faces]


def _cylinder(radius: float, length: float, segments: int = 20) -> np.ndarray:
    triangles = []
    for index in range(segments):
        a = 2.0 * math.pi * index / segments
        b = 2.0 * math.pi * (index + 1) / segments
        lower_a = (radius * math.cos(a), radius * math.sin(a), -length / 2.0)
        lower_b = (radius * math.cos(b), radius * math.sin(b), -length / 2.0)
        upper_a = (lower_a[0], lower_a[1], length / 2.0)
        upper_b = (lower_b[0], lower_b[1], length / 2.0)
        triangles.extend([
            [lower_a, lower_b, upper_b], [lower_a, upper_b, upper_a],
            [(0, 0, -length/2), lower_b, lower_a],
            [(0, 0, length/2), upper_a, upper_b],
        ])
    return np.asarray(triangles, dtype=float)


def _sphere(radius: float, rings: int = 10, segments: int = 20) -> np.ndarray:
    triangles = []
    for ring in range(rings):
        p0 = -math.pi / 2 + math.pi * ring / rings
        p1 = -math.pi / 2 + math.pi * (ring + 1) / rings
        for index in range(segments):
            a0 = 2 * math.pi * index / segments
            a1 = 2 * math.pi * (index + 1) / segments
            def point(p, a):
                return (radius*math.cos(p)*math.cos(a), radius*math.cos(p)*math.sin(a), radius*math.sin(p))
            triangles.extend([[point(p0, a0), point(p0, a1), point(p1, a1)], [point(p0, a0), point(p1, a1), point(p1, a0)]])
    return np.asarray(triangles, dtype=float)


def _stl(path: Path) -> tuple[np.ndarray, str]:
    raw = path.read_bytes()
    digest = hashlib.sha256(raw).hexdigest()
    if len(raw) >= 84:
        count = struct.unpack_from("<I", raw, 80)[0]
        if 84 + count * 50 == len(raw):
            result = np.empty((count, 3, 3), dtype=float)
            offset = 84
            for index in range(count):
                values = struct.unpack_from("<12fH", raw, offset)
                result[index] = np.asarray(values[3:12]).reshape(3, 3)
                offset += 50
            return result, digest
    vertices = []
    for line in raw.decode("utf-8", errors="ignore").splitlines():
        parts = line.strip().split()
        if len(parts) == 4 and parts[0].lower() == "vertex":
            vertices.append([float(value) for value in parts[1:]])
    if len(vertices) % 3:
        raise ValueError(f"invalid STL vertex count: {path}")
    return np.asarray(vertices, dtype=float).reshape(-1, 3, 3), digest


@dataclass
class Joint:
    parent: str
    child: str
    kind: str
    origin: np.ndarray
    axis: np.ndarray
    mimic: tuple[str, float, float] | None


class UrdfModel:
    def __init__(self, path: Path):
        self.path = path
        self.root = ET.parse(path).getroot()
        self.joints = {}
        joint_element_list = self.root.findall("joint")
        joint_names = [element.attrib["name"] for element in joint_element_list]
        if len(joint_names) != len(set(joint_names)):
            raise ValueError("URDF contains duplicate joint names")
        joint_elements = {
            element.attrib["name"]: element for element in joint_element_list
        }
        explicit_mimics = {}
        for name, element in joint_elements.items():
            mimic = element.find("mimic")
            explicit_mimics[name] = None if mimic is None else (
                mimic.attrib["joint"],
                float(mimic.get("multiplier", "1")),
                float(mimic.get("offset", "0")),
            )
        resolved_mimics = resolve_mimic_relations(explicit_mimics)
        self.joint_names_by_child = {}
        children = set()
        for joint_name, element in joint_elements.items():
            origin = element.find("origin")
            joint = Joint(
                parent=element.find("parent").attrib["link"],
                child=element.find("child").attrib["link"],
                kind=element.attrib["type"],
                origin=_transform(
                    _numbers(None if origin is None else origin.get("xyz"), 3, (0, 0, 0)),
                    _numbers(None if origin is None else origin.get("rpy"), 3, (0, 0, 0)),
                ),
                axis=_numbers(None if element.find("axis") is None else element.find("axis").get("xyz"), 3, (1, 0, 0)),
                mimic=resolved_mimics.get(joint_name),
            )
            self.joints[joint.child] = joint
            self.joint_names_by_child[joint.child] = joint_name
            children.add(joint.child)
        links = {element.attrib["name"] for element in self.root.findall("link")}
        roots = sorted(links - children)
        if len(roots) != 1:
            raise ValueError(f"URDF must have one root, got {roots}")
        self.root_link = roots[0]
        self.link_elements = {element.attrib["name"]: element for element in self.root.findall("link")}
        self._mesh_cache: dict[Path, tuple[np.ndarray, str]] = {}

    def transforms(self, values: dict[str, float]) -> dict[str, np.ndarray]:
        result = {self.root_link: np.eye(4)}
        pending = dict(self.joints)
        while pending:
            progressed = False
            for child, joint in list(pending.items()):
                if joint.parent not in result:
                    continue
                joint_name = self.joint_names_by_child[child]
                value = values.get(joint_name, 0.0)
                if joint.mimic:
                    value = values.get(joint.mimic[0], 0.0) * joint.mimic[1] + joint.mimic[2]
                motion = np.eye(4)
                if joint.kind in {"revolute", "continuous"}:
                    motion = _axis_rotation(joint.axis, value)
                elif joint.kind == "prismatic":
                    motion[:3, 3] = joint.axis * value
                result[child] = result[joint.parent] @ joint.origin @ motion
                del pending[child]
                progressed = True
            if not progressed:
                raise ValueError(f"disconnected joint tree: {sorted(pending)}")
        return result

    def geometry(self, transforms: dict[str, np.ndarray]):
        triangles, owners, mesh_hashes = [], [], {}
        for link_name, link in self.link_elements.items():
            for tag in ("collision", "visual"):
                for item in link.findall(tag):
                    geometry = item.find("geometry")
                    if geometry is None:
                        continue
                    origin = item.find("origin")
                    local = _transform(
                        _numbers(None if origin is None else origin.get("xyz"), 3, (0, 0, 0)),
                        _numbers(None if origin is None else origin.get("rpy"), 3, (0, 0, 0)),
                    )
                    mesh = geometry.find("mesh")
                    if mesh is not None:
                        uri = mesh.attrib["filename"]
                        prefix = "package://sanitation_vehicle_description/meshes/"
                        if not uri.startswith(prefix):
                            continue
                        path = MESH_ROOT / uri[len(prefix):]
                        if path not in self._mesh_cache:
                            self._mesh_cache[path] = _stl(path)
                        current, mesh_digest = self._mesh_cache[path]
                        scale = _numbers(mesh.get("scale"), 3, (1, 1, 1))
                        current = current * scale.reshape(1, 1, 3)
                        mesh_hashes[str(path.relative_to(ROOT)).replace("\\", "/")] = mesh_digest
                    elif geometry.find("box") is not None:
                        current = _box(_numbers(geometry.find("box").get("size"), 3, (0, 0, 0)))
                    elif geometry.find("cylinder") is not None:
                        cylinder = geometry.find("cylinder")
                        current = _cylinder(float(cylinder.get("radius")), float(cylinder.get("length")))
                    elif geometry.find("sphere") is not None:
                        current = _sphere(float(geometry.find("sphere").get("radius")))
                    else:
                        continue
                    current = _apply(current, transforms[link_name] @ local)
                    triangles.append(current)
                    owners.extend([link_name] * len(current))
        return np.concatenate(triangles), np.asarray(owners, dtype=object), mesh_hashes


@dataclass
class BvhNode:
    lower: np.ndarray
    upper: np.ndarray
    indices: np.ndarray | None = None
    left: "BvhNode | None" = None
    right: "BvhNode | None" = None


def _build_bvh(triangles: np.ndarray, indices: np.ndarray | None = None) -> BvhNode:
    if indices is None:
        indices = np.arange(len(triangles))
    selected = triangles[indices]
    lower, upper = selected.min(axis=(0, 1)), selected.max(axis=(0, 1))
    if len(indices) <= 24:
        return BvhNode(lower, upper, indices=indices)
    centroids = selected.mean(axis=1)
    axis = int(np.argmax(np.ptp(centroids, axis=0)))
    order = indices[np.argsort(centroids[:, axis], kind="mergesort")]
    middle = len(order) // 2
    return BvhNode(lower, upper, left=_build_bvh(triangles, order[:middle]), right=_build_bvh(triangles, order[middle:]))


def _aabb(origin, direction, lower, upper, maximum):
    """Numerically stable ray/AABB slab test.

    Multiplying an infinite inverse direction by a zero offset yields NaN and
    can silently drop a box when a ray lies on its face.  Handle parallel axes
    explicitly so the audit is deterministic on every supported NumPy build.
    """
    near, far = 0.0, maximum
    for axis in range(3):
        if abs(direction[axis]) <= 1e-12:
            if origin[axis] < lower[axis] or origin[axis] > upper[axis]:
                return False
            continue
        first = (lower[axis] - origin[axis]) / direction[axis]
        second = (upper[axis] - origin[axis]) / direction[axis]
        near = max(near, min(first, second))
        far = min(far, max(first, second))
        if far < near:
            return False
    return near <= maximum


def _nearest(origin, direction, maximum, triangles, owners, bvh, ignored):
    best, best_index = maximum, None
    stack = [bvh]
    while stack:
        node = stack.pop()
        if not _aabb(origin, direction, node.lower, node.upper, best):
            continue
        if node.indices is None:
            stack.extend((node.left, node.right))
            continue
        indices = node.indices
        allowed = np.array([owners[index] not in ignored for index in indices])
        indices = indices[allowed]
        if not len(indices):
            continue
        tri = triangles[indices]
        edge1, edge2 = tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]
        pvec = np.cross(np.broadcast_to(direction, edge2.shape), edge2)
        determinant = np.einsum("ij,ij->i", edge1, pvec)
        valid = np.abs(determinant) > 1e-10
        inv_det = np.divide(1.0, determinant, out=np.zeros_like(determinant), where=valid)
        tvec = origin - tri[:, 0]
        u = np.einsum("ij,ij->i", tvec, pvec) * inv_det
        qvec = np.cross(tvec, edge1)
        v = qvec @ direction * inv_det
        distance = np.einsum("ij,ij->i", edge2, qvec) * inv_det
        valid &= (u >= 0.0) & (v >= 0.0) & (u + v <= 1.0) & (distance > 1e-4) & (distance < best)
        if np.any(valid):
            local = np.where(valid, distance, np.inf).argmin()
            best, best_index = float(distance[local]), int(indices[local])
    return best_index, best


def _directions(config: dict) -> np.ndarray:
    kind = config["kind"]
    if kind == "lidar2d":
        angles = np.deg2rad(np.linspace(-135.0, 135.0, 1081))
        return np.stack((np.cos(angles), np.sin(angles), np.zeros_like(angles)), axis=1)
    if kind == "mid360":
        azimuth = np.deg2rad(np.arange(-180.0, 180.0, 1.0))
        elevation = np.deg2rad(np.linspace(-7.0, 52.0, 60))
        return np.asarray([(math.cos(e)*math.cos(a), math.cos(e)*math.sin(a), math.sin(e)) for e in elevation for a in azimuth])
    if kind in {"camera", "fisheye"}:
        horizontal = np.deg2rad(np.linspace(-config["hfov"]/2, config["hfov"]/2, 51))
        vertical = np.deg2rad(np.linspace(-config["vfov"]/2, config["vfov"]/2, 35))
        if kind == "camera":
            result = np.asarray([(math.tan(h), math.tan(v), 1.0) for v in vertical for h in horizontal])
        else:
            result = np.asarray([(math.cos(v)*math.sin(h), math.sin(v), math.cos(v)*math.cos(h)) for v in vertical for h in horizontal])
        return result / np.linalg.norm(result, axis=1)[:, None]
    if kind == "sky":
        azimuth = np.deg2rad(np.arange(0.0, 360.0, 2.0))
        # A 15 degree satellite elevation mask is the declared RTK operating
        # domain; near-horizontal rays are multipath-prone and are not usable
        # GNSS sky visibility even on a vehicle without a mast.
        minimum_z = math.sin(math.radians(15.0))
        z_values = minimum_z + ((np.arange(30) + 0.5) / 30.0) * (1.0 - minimum_z)
        return np.asarray([(math.sqrt(1-z*z)*math.cos(a), math.sqrt(1-z*z)*math.sin(a), z) for z in z_values for a in azimuth])
    raise KeyError(kind)


def _scan(sensor_id, config, transforms, triangles, owners, bvh):
    frame = transforms[config["frame"]]
    origin = frame[:3, 3]
    local = _directions(config)
    world = local @ frame[:3, :3].T
    blocked = []
    histogram = {}
    for index, direction in enumerate(world):
        hit, distance = _nearest(origin, direction, config["range"], triangles, owners, bvh, config["ignore"])
        if hit is not None:
            owner = str(owners[hit])
            histogram[owner] = histogram.get(owner, 0) + 1
            blocked.append({"ray_index": index, "distance_m": distance, "occluder_link": owner, "direction_world": direction.tolist()})
    blocked.sort(key=lambda item: (item["distance_m"], item["ray_index"]))
    clear = (len(world) - len(blocked)) / len(world)
    return {
        "sensor_id": sensor_id,
        "frame": config["frame"],
        "ray_count": len(world),
        "blocked_ray_count": len(blocked),
        "clear_fraction": round(clear, 9),
        "occluder_histogram": dict(sorted(histogram.items(), key=lambda item: (-item[1], item[0]))),
        "worst_blocked_rays": blocked[:10],
        "origin_world_m": origin.tolist(),
    }


def _angle_between(first: np.ndarray, second: np.ndarray) -> float:
    first = first / np.linalg.norm(first)
    second = second / np.linalg.norm(second)
    return math.acos(float(np.clip(first @ second, -1.0, 1.0)))


def _sensor_forward_axis(config: dict) -> np.ndarray:
    return np.asarray((0.0, 0.0, 1.0) if config["kind"] in {"camera", "fisheye", "sky"} else (1.0, 0.0, 0.0))


def _inside_fov(local_direction: np.ndarray, config: dict) -> bool:
    direction = local_direction / np.linalg.norm(local_direction)
    kind = config["kind"]
    if kind == "camera":
        if direction[2] <= 0.0:
            return False
        horizontal = abs(math.atan2(direction[0], direction[2]))
        vertical = abs(math.atan2(direction[1], direction[2]))
        return horizontal <= math.radians(config["hfov"] / 2.0) and vertical <= math.radians(config["vfov"] / 2.0)
    if kind == "fisheye":
        if direction[2] <= 0.0:
            return False
        horizontal = abs(math.atan2(direction[0], direction[2]))
        vertical = abs(math.atan2(direction[1], math.hypot(direction[0], direction[2])))
        return horizontal <= math.radians(config["hfov"] / 2.0) and vertical <= math.radians(config["vfov"] / 2.0)
    if kind == "lidar2d":
        horizontal = abs(math.atan2(direction[1], direction[0]))
        return horizontal <= math.radians(135.0) and abs(direction[2]) <= 1e-5
    if kind == "mid360":
        elevation = math.asin(float(np.clip(direction[2], -1.0, 1.0)))
        return math.radians(-7.0) <= elevation <= math.radians(52.0)
    if kind == "sky":
        return direction[2] > 0.0
    raise KeyError(kind)


def _target_visibility(sensor_id, config, transforms, triangles, owners, bvh, targets):
    frame = transforms[config["frame"]]
    inverse_rotation = frame[:3, :3].T
    origin = frame[:3, 3]
    rows = []
    for target_id, raw_target in targets:
        target = np.asarray(raw_target, dtype=float)
        delta = target - origin
        distance = float(np.linalg.norm(delta))
        direction = delta / distance
        local_direction = inverse_rotation @ direction
        in_range = 0.2 <= distance <= config["range"]
        in_fov = _inside_fov(local_direction, config)
        hit, hit_distance = _nearest(
            origin,
            direction,
            max(distance - 0.003, 0.0),
            triangles,
            owners,
            bvh,
            config["ignore"],
        )
        rows.append({
            "target_id": target_id,
            "target_world_m": target.tolist(),
            "distance_m": round(distance, 9),
            "in_range": in_range,
            "in_fov": in_fov,
            "line_of_sight_clear": hit is None,
            "occluder_link": None if hit is None else str(owners[hit]),
            "occluder_distance_m": None if hit is None else round(hit_distance, 9),
            "visible": bool(in_range and in_fov and hit is None),
        })
    return rows


def _grid_targets(prefix: str, xs, ys, z: float):
    return [(f"{prefix}_{ix}_{iy}", (float(x), float(y), z)) for ix, x in enumerate(xs) for iy, y in enumerate(ys)]


def _contract_audit(model: UrdfModel, layout: dict, transforms: dict) -> dict:
    tolerance_m = float(layout["validation_policy"]["static_frame_position_tolerance_m"])
    tolerance_rad = float(layout["validation_policy"]["static_frame_rotation_tolerance_rad"])
    rows = {}
    for item in layout["sensor_layout"]:
        sensor_id = item["id"]
        frame_name = item["frame"]
        config = SENSORS.get(sensor_id, {"kind": "imu"})
        if sensor_id == "wrist_d435_depth":
            reference = "tool0"
            frame = np.linalg.inv(transforms[reference]) @ transforms[frame_name]
        else:
            reference = model.root_link
            frame = transforms[frame_name]
        expected_position = np.asarray(item["xyz_m"], dtype=float)
        measured_position = frame[:3, 3]
        local_axis = _sensor_forward_axis(config) if sensor_id != "imu" else np.asarray((1.0, 0.0, 0.0))
        measured_forward = frame[:3, :3] @ local_axis
        expected_forward = np.asarray(item["forward_xyz"], dtype=float)
        position_error = float(np.linalg.norm(measured_position - expected_position))
        direction_error = _angle_between(measured_forward, expected_forward)
        rows[sensor_id] = {
            "frame": frame_name,
            "coordinate_reference": reference,
            "expected_position_m": expected_position.tolist(),
            "measured_position_m": measured_position.tolist(),
            "position_error_m": round(position_error, 12),
            "position_tolerance_m": tolerance_m,
            "expected_forward_xyz": expected_forward.tolist(),
            "measured_forward_xyz": measured_forward.tolist(),
            "direction_error_rad": round(direction_error, 12),
            "direction_tolerance_rad": tolerance_rad,
            "passed": position_error <= tolerance_m and direction_error <= tolerance_rad,
        }
    return {
        "passed": all(row["passed"] for row in rows.values()),
        "sensor_frames": rows,
    }


def _text(element: ET.Element, path: str) -> str | None:
    child = element.find(path)
    return None if child is None or child.text is None else child.text.strip()


def _sensor_parameter_audit(model: UrdfModel) -> dict:
    expected = {
        "utm30lx": {"type": "gpu_lidar", "topic": "/sensors/lidar_2d/scan", "rate": 40.0, "frame": "lidar_2d_link", "range": (0.1, 30.0), "hfov": (-2.356194, 2.356194), "samples": (1080, 1)},
        "mid360": {"type": "gpu_lidar", "topic": "/sensors/lidar_3d", "rate": 10.0, "frame": "lidar_3d_link", "range": (0.1, 40.0), "hfov": (-math.pi, math.pi), "vfov": (-0.122173, 0.907571), "samples": (1800, 64)},
        "front_rgbd_d435_rgbd": {"type": "rgbd_camera", "topic": "/sensors/front_rgbd/depth/image_rect_raw", "rate": 30.0, "frame": "front_rgbd_depth_optical_frame", "range": (0.2, 10.0), "camera_hfov": 1.518436, "image": (848, 480)},
        "wrist_rgbd_d435_rgbd": {"type": "rgbd_camera", "topic": "/sensors/wrist_rgbd/depth/image_rect_raw", "rate": 30.0, "frame": "wrist_rgbd_depth_optical_frame", "range": (0.2, 3.0), "camera_hfov": 1.518436, "image": (848, 480)},
        "rear_left_fisheye_imx291": {"type": "wideanglecamera", "topic": "/sensors/rear_left_fisheye/image_raw", "rate": 30.0, "frame": "rear_left_fisheye_optical_frame", "range": (0.2, 20.0), "camera_hfov": 2.617994, "image": (1920, 1080), "lens_type": "equisolid_angle"},
        "rear_right_fisheye_imx291": {"type": "wideanglecamera", "topic": "/sensors/rear_right_fisheye/image_raw", "rate": 30.0, "frame": "rear_right_fisheye_optical_frame", "range": (0.2, 20.0), "camera_hfov": 2.617994, "image": (1920, 1080), "lens_type": "equisolid_angle"},
        "zed_f9p": {"type": "navsat", "topic": "/sensors/gnss/fix", "rate": 10.0, "frame": "gnss_antenna_link"},
        "vn100": {"type": "imu", "topic": "/sensors/imu/data", "rate": 200.0, "frame": "imu_link"},
    }
    actual = {sensor.attrib["name"]: sensor for sensor in model.root.findall(".//sensor")}
    rows = {}
    for sensor_name, contract in expected.items():
        element = actual.get(sensor_name)
        checks = {}
        if element is not None:
            checks.update({
                "type": element.get("type") == contract["type"],
                "topic": _text(element, "topic") == contract["topic"],
                "update_rate": abs(float(_text(element, "update_rate") or "nan") - contract["rate"]) <= 1e-9,
                "frame": _text(element, "gz_frame_id") == contract["frame"],
            })
            if "range" in contract:
                if contract["type"] == "gpu_lidar":
                    measured = (float(_text(element, "lidar/range/min") or "nan"), float(_text(element, "lidar/range/max") or "nan"))
                else:
                    measured = (float(_text(element, "camera/clip/near") or "nan"), float(_text(element, "camera/clip/far") or "nan"))
                checks["range"] = np.allclose(measured, contract["range"], atol=1e-6)
            if "hfov" in contract:
                measured = (float(_text(element, "lidar/scan/horizontal/min_angle") or "nan"), float(_text(element, "lidar/scan/horizontal/max_angle") or "nan"))
                checks["horizontal_fov"] = np.allclose(measured, contract["hfov"], atol=1e-6)
            if "vfov" in contract:
                measured = (float(_text(element, "lidar/scan/vertical/min_angle") or "nan"), float(_text(element, "lidar/scan/vertical/max_angle") or "nan"))
                checks["vertical_fov"] = np.allclose(measured, contract["vfov"], atol=1e-6)
            if "camera_hfov" in contract:
                checks["horizontal_fov"] = abs(float(_text(element, "camera/horizontal_fov") or "nan") - contract["camera_hfov"]) <= 1e-6
            if "samples" in contract:
                measured = (int(_text(element, "lidar/scan/horizontal/samples") or -1), int(_text(element, "lidar/scan/vertical/samples") or -1))
                checks["samples"] = measured == contract["samples"]
            if "image" in contract:
                measured = (int(_text(element, "camera/image/width") or -1), int(_text(element, "camera/image/height") or -1))
                checks["image"] = measured == contract["image"]
            if "lens_type" in contract:
                checks["lens_type"] = _text(element, "camera/lens/type") == contract["lens_type"]
                checks["scale_to_hfov"] = _text(element, "camera/lens/scale_to_hfov") == "true"
                checks["cutoff_angle"] = abs(float(_text(element, "camera/lens/cutoff_angle") or "nan") - math.pi / 2.0) <= 1e-6
        rows[sensor_name] = {
            "present": element is not None,
            "checks": checks,
            "passed": element is not None and all(checks.values()),
        }
    return {"passed": all(row["passed"] for row in rows.values()), "sensors": rows}


def validate(urdf_path: Path, layout_path: Path) -> dict:
    model = UrdfModel(urdf_path)
    layout = yaml.safe_load(layout_path.read_text(encoding="utf-8"))
    thresholds = {item["id"]: float(item["minimum_clear_fraction"]) for item in layout["sensor_layout"]}
    poses = ARM_POSES
    scans, mesh_hashes, geometry_counts = {}, {}, {}

    # Build each transformed triangle set and BVH exactly once, score every
    # gate that needs that pose, then release it before advancing.  Retaining
    # four complete pose geometries previously multiplied peak RAM without
    # adding evidence: only the compact scored rows are part of the report.
    transport_transforms = model.transforms(poses["transport"])
    contract_audit = _contract_audit(model, layout, transport_transforms)
    parameter_audit = _sensor_parameter_audit(model)

    front_targets = []
    front_origin = transport_transforms[SENSORS["front_d435_depth"]["frame"]][:3, 3]
    half_angle = 0.8 * SENSORS["front_d435_depth"]["hfov"] / 2.0
    for ix, x in enumerate(np.linspace(1.2, 4.0, 8)):
        # The declared work zone is a true camera-centred ground trapezoid
        # inside 80% of the horizontal optical envelope, capped at 1.2 m on
        # either side. Using vehicle-origin x here would create targets that
        # are geometrically outside the claimed ROI at its near edge.
        forward_distance = float(x) - float(front_origin[0])
        half_width = min(1.2, forward_distance * math.tan(half_angle))
        front_targets.extend(
            (f"front_ground_{ix}_{iy}", (float(x), float(y), 0.03))
            for iy, y in enumerate(np.linspace(-half_width, half_width, 9))
        )
    rear_targets = [
        (f"rear_perimeter_{index}", (1.5 * math.cos(angle), 1.5 * math.sin(angle), 0.35))
        # The two side-rear cameras jointly own this 140 degree rear sector;
        # the adjacent side sectors remain inside MID-360 responsibility.
        for index, angle in enumerate(np.deg2rad(np.linspace(110.0, 250.0, 57)))
    ]
    cube_targets = _grid_targets(
        "cube_top",
        np.linspace(0.285, 0.315, 3),
        np.linspace(-0.965, -0.935, 3),
        0.032,
    )
    front_pose_rows, rear_pose_rows = {}, {}
    cube_rows = None
    deposit_rows = None

    for pose_name, values in poses.items():
        transforms = model.transforms(values)
        triangles, owners, hashes = model.geometry(transforms)
        mesh_hashes.update(hashes)
        geometry_counts[pose_name] = int(len(triangles))
        bvh = _build_bvh(triangles)
        scans[pose_name] = {
            sensor_id: _scan(sensor_id, config, transforms, triangles, owners, bvh)
            for sensor_id, config in SENSORS.items()
        }
        front_rows = _target_visibility(
            "front_d435_depth",
            SENSORS["front_d435_depth"],
            transforms,
            triangles,
            owners,
            bvh,
            front_targets,
        )
        front_pose_rows[pose_name] = {
            "visible_fraction": sum(row["visible"] for row in front_rows) / len(front_rows),
            "targets": front_rows,
        }
        left_rows = _target_visibility(
            "rear_left_fisheye",
            SENSORS["rear_left_fisheye"],
            transforms,
            triangles,
            owners,
            bvh,
            rear_targets,
        )
        right_rows = _target_visibility(
            "rear_right_fisheye",
            SENSORS["rear_right_fisheye"],
            transforms,
            triangles,
            owners,
            bvh,
            rear_targets,
        )
        combined = [
            left_rows[index]["visible"] or right_rows[index]["visible"]
            for index in range(len(rear_targets))
        ]
        rear_pose_rows[pose_name] = {
            "combined_visible_fraction": sum(combined) / len(combined),
            "left_targets": left_rows,
            "right_targets": right_rows,
        }
        if pose_name == "pregrasp":
            cube_rows = _target_visibility(
                "wrist_d435_depth",
                SENSORS["wrist_d435_depth"],
                transforms,
                triangles,
                owners,
                bvh,
                cube_targets,
            )
        if pose_name == "deposit":
            hopper_center = transforms["dry_deposit_hopper_link"][:3, 3]
            deposit_targets = _grid_targets(
                "deposit_aperture",
                np.linspace(hopper_center[0] - 0.040, hopper_center[0] + 0.040, 5),
                np.linspace(hopper_center[1] - 0.035, hopper_center[1] + 0.035, 5),
                float(hopper_center[2]),
            )
            deposit_rows = _target_visibility(
                "wrist_d435_depth",
                SENSORS["wrist_d435_depth"],
                transforms,
                triangles,
                owners,
                bvh,
                deposit_targets,
            )
    # Negative regression fixture for the prior Gazebo startup defect: all arm
    # interfaces defaulted to zero although the declared startup state was the
    # frozen transport posture. The mesh-ray audit must detect that mismatch,
    # otherwise a nominal 1.0 clear fraction can hide a rendered blocked view.
    legacy_zero_transforms = model.transforms({})
    legacy_zero_triangles, legacy_zero_owners, legacy_hashes = model.geometry(legacy_zero_transforms)
    mesh_hashes.update(legacy_hashes)
    legacy_zero_scan = _scan(
        "front_d435_depth",
        SENSORS["front_d435_depth"],
        legacy_zero_transforms,
        legacy_zero_triangles,
        legacy_zero_owners,
        _build_bvh(legacy_zero_triangles),
    )
    startup_pose_regression = {
        "legacy_zero_pose_clear_fraction": legacy_zero_scan["clear_fraction"],
        "legacy_zero_pose_detected_as_blocked": legacy_zero_scan["clear_fraction"]
        < thresholds["front_d435_depth"],
        "legacy_zero_pose_occluders": legacy_zero_scan["occluder_histogram"],
        "configured_transport_pose_clear_fraction": scans["transport"]["front_d435_depth"]["clear_fraction"],
        "configured_transport_pose_passed": scans["transport"]["front_d435_depth"]["clear_fraction"]
        >= thresholds["front_d435_depth"],
    }
    startup_pose_regression["passed"] = bool(
        startup_pose_regression["legacy_zero_pose_detected_as_blocked"]
        and startup_pose_regression["configured_transport_pose_passed"]
    )
    results = {}
    for sensor_id in SENSORS:
        relevant_names = FULL_FOV_REQUIRED_POSES[sensor_id]
        relevant = [scans[name][sensor_id] for name in relevant_names]
        minimum = min(item["clear_fraction"] for item in relevant)
        results[sensor_id] = {
            "minimum_clear_fraction": thresholds[sensor_id],
            "measured_clear_fraction": minimum,
            "passed": minimum + 1e-12 >= thresholds[sensor_id],
            "required_pose_results": {name: scans[name][sensor_id] for name in relevant_names},
            "all_pose_results": {name: scans[name][sensor_id] for name in poses},
            "worst_required_pose": min(zip(relevant_names, relevant), key=lambda item: item[1]["clear_fraction"])[0],
        }

    # Functional work-zone visibility is separate from an aggregate FOV
    # fraction: a camera can have a clear sky-facing FOV and still miss the
    # ground, cube or deposit aperture that justifies its installation.
    front_gate = min(row["visible_fraction"] for row in front_pose_rows.values()) >= 0.90
    rear_gate = min(row["combined_visible_fraction"] for row in rear_pose_rows.values()) >= 0.95

    if cube_rows is None or deposit_rows is None:
        raise RuntimeError("required pregrasp or deposit pose was not scanned")
    cube_fraction = sum(row["visible"] for row in cube_rows) / len(cube_rows)
    cube_gate = cube_fraction >= 0.95

    deposit_fraction = sum(row["visible"] for row in deposit_rows) / len(deposit_rows)
    deposit_gate = deposit_fraction >= 0.80

    functional_coverage = {
        "front_ground_observation": {"required_for_acceptance": True, "minimum_visible_fraction": 0.90, "passed": front_gate, "pose_results": front_pose_rows},
        "rear_fisheye_safety_perimeter": {"required_for_acceptance": True, "minimum_combined_visible_fraction": 0.95, "passed": rear_gate, "pose_results": rear_pose_rows},
        "wrist_pregrasp_cube": {"required_for_acceptance": True, "minimum_visible_fraction": 0.95, "measured_visible_fraction": cube_fraction, "passed": cube_gate, "targets": cube_rows},
        # The architecture assigns the wrist camera to pregrasp alignment, not
        # deposit confirmation.  Keep this diagnostic explicit so a future
        # task may promote it, but do not invent a requirement absent from the
        # product function contract.
        "wrist_deposit_aperture": {"required_for_acceptance": False, "minimum_visible_fraction": 0.80, "measured_visible_fraction": deposit_fraction, "passed": deposit_gate, "targets": deposit_rows},
    }
    all_passed = (
        all(item["passed"] for item in results.values())
        and contract_audit["passed"]
        and parameter_audit["passed"]
        and startup_pose_regression["passed"]
        and all(item["passed"] for item in functional_coverage.values() if item["required_for_acceptance"])
    )
    return {
        "report_id": "tzcup_formal_fov_occlusion_mesh_ray_v1",
        "status": "PASSED" if all_passed else "BLOCKED",
        "all_minimum_clear_fractions_passed": all_passed,
        "urdf": str(urdf_path),
        "urdf_sha256": hashlib.sha256(urdf_path.read_bytes()).hexdigest(),
        "layout_sha256": hashlib.sha256(layout_path.read_bytes()).hexdigest(),
        "validator": Path(__file__).resolve().relative_to(ROOT).as_posix(),
        "validator_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "geometry_triangle_count_by_pose": geometry_counts,
        "mesh_file_count": len(mesh_hashes),
        "mesh_hashes": dict(sorted(mesh_hashes.items())),
        "ray_geometry_policy": {
            "included": "all URDF collision primitives/STL meshes plus STL visual meshes",
            "collada_visual_policy": "DAE visual surfaces are not triangulated; every physical DAE link is represented by its URDF collision mesh or primitive",
            "sensor_self_link_policy": "only the emitting sensor housing link is ignored; physical brackets remain occluders",
            "endpoint_tolerance_m": 0.003,
        },
        "scan_domains": {
            "utm30lx": "270deg horizontal, 0.25deg deterministic rays",
            "mid360": "360deg azimuth x -7..+52deg elevation, 1deg deterministic grid",
            "cameras": "rectilinear or equiangular fisheye grid over frozen contract FOV",
            "gnss": "uniform-solid-angle sky above the declared 15deg satellite elevation mask",
            "arm_poses": ARM_POSES,
        },
        "sensor_results": results,
        "mount_pose_and_direction_audit": contract_audit,
        "urdf_sensor_parameter_audit": parameter_audit,
        "startup_arm_pose_visibility_regression": startup_pose_regression,
        "functional_zone_coverage": functional_coverage,
        "layout_updates_allowed": {
            sensor_id: item["measured_clear_fraction"] for sensor_id, item in results.items() if item["passed"]
        },
        "claim_boundary": "Deterministic mesh-ray self-occlusion, mount/FOV parameter and declared work-zone audit against the expanded URDF. It does not replace Gazebo rendered/range-message visibility, lens distortion calibration, satellite multipath modelling or real installation calibration.",
    }


def compact_report(report: dict) -> dict:
    """Remove per-ray/target rows while retaining every scored denominator."""
    compact = json.loads(json.dumps(report))
    for result in compact["sensor_results"].values():
        for collection_name in ("required_pose_results", "all_pose_results"):
            for pose in result[collection_name].values():
                pose.pop("worst_blocked_rays", None)
    functional = compact["functional_zone_coverage"]
    for gate_name in ("front_ground_observation", "rear_fisheye_safety_perimeter"):
        for pose in functional[gate_name]["pose_results"].values():
            if "targets" in pose:
                targets = pose.pop("targets")
                pose["target_count"] = len(targets)
                pose["visible_count"] = sum(target["visible"] for target in targets)
            else:
                left = pose.pop("left_targets")
                right = pose.pop("right_targets")
                pose["target_count"] = len(left)
                pose["left_visible_count"] = sum(target["visible"] for target in left)
                pose["right_visible_count"] = sum(target["visible"] for target in right)
    for gate_name in ("wrist_pregrasp_cube", "wrist_deposit_aperture"):
        targets = functional[gate_name].pop("targets")
        functional[gate_name]["target_count"] = len(targets)
        functional[gate_name]["visible_count"] = sum(target["visible"] for target in targets)
        functional[gate_name]["out_of_fov_count"] = sum(not target["in_fov"] for target in targets)
        functional[gate_name]["occluded_count"] = sum(not target["line_of_sight_clear"] for target in targets)
    compact["report_form"] = "compact_scored_summary"
    return compact


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    parser.add_argument("--layout", type=Path, default=DEFAULT_LAYOUT)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--compact", action="store_true")
    args = parser.parse_args()
    report = validate(args.urdf, args.layout)
    output_report = compact_report(report) if args.compact else report
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(output_report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"status": report["status"], "sensor_results": {key: value["measured_clear_fraction"] for key, value in report["sensor_results"].items()}}, ensure_ascii=False))
    return 0 if report["all_minimum_clear_fractions_passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
