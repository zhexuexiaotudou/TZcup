#!/usr/bin/env python3
"""Render a deterministic engineering preview from an expanded mesh URDF."""

from __future__ import annotations

import argparse
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import trimesh
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_URDF = ROOT / "reports" / "engineering" / "formal_competition_vehicle.urdf"
DEFAULT_OUTPUT = ROOT / "reports" / "engineering" / "formal_vehicle_preview.png"
PACKAGE_ROOT = ROOT / "starter_ws" / "src" / "sanitation_vehicle_description"


def numbers(raw: str | None, default: tuple[float, ...]) -> np.ndarray:
    return np.array([float(value) for value in raw.split()] if raw else default, dtype=float)


def transform(xyz: np.ndarray, rpy: np.ndarray) -> np.ndarray:
    roll, pitch, yaw = rpy
    cr, sr, cp, sp, cy, sy = (
        math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    )
    rotation = np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )
    result = np.eye(4)
    result[:3, :3] = rotation
    result[:3, 3] = xyz
    return result


def apply(matrix: np.ndarray, points: np.ndarray) -> np.ndarray:
    homogeneous = np.c_[points, np.ones(len(points))]
    return (matrix @ homogeneous.T).T[:, :3]


def box_faces(size: np.ndarray) -> list[np.ndarray]:
    x, y, z = size * 0.5
    corners = np.array(
        [[-x, -y, -z], [x, -y, -z], [x, y, -z], [-x, y, -z],
         [-x, -y, z], [x, -y, z], [x, y, z], [-x, y, z]]
    )
    return [corners[index] for index in ([0, 1, 2, 3], [4, 5, 6, 7], [0, 1, 5, 4],
                                          [1, 2, 6, 5], [2, 3, 7, 6], [3, 0, 4, 7])]


def cylinder_faces(radius: float, length: float, segments: int = 16) -> list[np.ndarray]:
    angles = np.linspace(0.0, 2.0 * math.pi, segments, endpoint=False)
    low = np.c_[radius * np.cos(angles), radius * np.sin(angles), np.full(segments, -length / 2.0)]
    high = low.copy()
    high[:, 2] = length / 2.0
    faces = [low, high[::-1]]
    for index in range(segments):
        nxt = (index + 1) % segments
        faces.append(np.array([low[index], low[nxt], high[nxt], high[index]]))
    return faces


def sphere_faces(radius: float, segments: int = 12) -> list[np.ndarray]:
    faces: list[np.ndarray] = []
    polar = np.linspace(0.0, math.pi, segments // 2 + 1)
    azimuth = np.linspace(0.0, 2.0 * math.pi, segments, endpoint=False)
    rings = [np.c_[radius * np.sin(p) * np.cos(azimuth), radius * np.sin(p) * np.sin(azimuth),
                   np.full(segments, radius * np.cos(p))] for p in polar]
    for row in range(len(rings) - 1):
        for index in range(segments):
            nxt = (index + 1) % segments
            faces.append(np.array([rings[row][index], rings[row][nxt], rings[row + 1][nxt], rings[row + 1][index]]))
    return faces


def material_rgba(visual: ET.Element) -> tuple[float, float, float, float]:
    color = visual.find("material/color")
    if color is None or not color.attrib.get("rgba"):
        return 0.52, 0.58, 0.62, 0.9
    rgba = tuple(float(value) for value in color.attrib["rgba"].split())
    return rgba[0], rgba[1], rgba[2], max(rgba[3], 0.18)


def mesh_faces(node: ET.Element, max_faces: int = 2400) -> list[np.ndarray]:
    filename = node.attrib["filename"]
    prefix = "package://sanitation_vehicle_description/"
    if not filename.startswith(prefix):
        raise ValueError(f"preview only accepts self-contained package meshes: {filename}")
    path = PACKAGE_ROOT / filename[len(prefix):]
    loaded = trimesh.load(path, force="scene")
    mesh = loaded.to_geometry()
    scale = numbers(node.attrib.get("scale"), (1.0, 1.0, 1.0))
    vertices = np.asarray(mesh.vertices, dtype=float) * scale
    faces = np.asarray(mesh.faces, dtype=int)
    if len(faces) > max_faces:
        faces = faces[:: max(1, math.ceil(len(faces) / max_faces))]
    return [vertices[index] for index in faces]


def link_transforms(root: ET.Element) -> dict[str, np.ndarray]:
    children: dict[str, list[tuple[str, np.ndarray]]] = {}
    child_names: set[str] = set()
    for joint in root.findall("joint"):
        parent = joint.find("parent").attrib["link"]
        child = joint.find("child").attrib["link"]
        origin = joint.find("origin")
        matrix = transform(
            numbers(origin.attrib.get("xyz") if origin is not None else None, (0.0, 0.0, 0.0)),
            numbers(origin.attrib.get("rpy") if origin is not None else None, (0.0, 0.0, 0.0)),
        )
        children.setdefault(parent, []).append((child, matrix))
        child_names.add(child)
    root_name = next(link.attrib["name"] for link in root.findall("link") if link.attrib["name"] not in child_names)
    poses = {root_name: np.eye(4)}
    stack = [root_name]
    while stack:
        parent = stack.pop()
        for child, relative in children.get(parent, []):
            poses[child] = poses[parent] @ relative
            stack.append(child)
    return poses


def render(urdf_path: Path, output_path: Path) -> None:
    root = ET.parse(urdf_path).getroot()
    poses = link_transforms(root)
    collections: list[tuple[list[np.ndarray], tuple[float, float, float, float]]] = []
    all_points: list[np.ndarray] = []
    for link in root.findall("link"):
        link_pose = poses[link.attrib["name"]]
        for visual in link.findall("visual"):
            origin = visual.find("origin")
            visual_pose = transform(
                numbers(origin.attrib.get("xyz") if origin is not None else None, (0.0, 0.0, 0.0)),
                numbers(origin.attrib.get("rpy") if origin is not None else None, (0.0, 0.0, 0.0)),
            )
            geometry = visual.find("geometry")
            box = geometry.find("box")
            cylinder = geometry.find("cylinder")
            sphere = geometry.find("sphere")
            mesh = geometry.find("mesh")
            if box is not None:
                faces = box_faces(numbers(box.attrib["size"], (1.0, 1.0, 1.0)))
            elif cylinder is not None:
                faces = cylinder_faces(float(cylinder.attrib["radius"]), float(cylinder.attrib["length"]))
            elif sphere is not None:
                faces = sphere_faces(float(sphere.attrib["radius"]))
            elif mesh is not None:
                faces = mesh_faces(mesh)
            else:
                continue
            world_faces = [apply(link_pose @ visual_pose, face) for face in faces]
            collections.append((world_faces, material_rgba(visual)))
            all_points.extend(world_faces)

    figure = plt.figure(figsize=(12, 9), facecolor="#f5f6f7")
    axis = figure.add_subplot(111, projection="3d", facecolor="#f5f6f7")
    for faces, rgba in collections:
        axis.add_collection3d(Poly3DCollection(faces, facecolors=[rgba], edgecolors="none", linewidths=0.0))
    points = np.vstack(all_points)
    lower, upper = points.min(axis=0), points.max(axis=0)
    centre = (lower + upper) * 0.5
    radius = max(upper - lower) * 0.58
    axis.set_xlim(centre[0] - radius, centre[0] + radius)
    axis.set_ylim(centre[1] - radius, centre[1] + radius)
    axis.set_zlim(max(0.0, centre[2] - radius), centre[2] + radius)
    axis.set_box_aspect((1, 1, 1))
    axis.view_init(elev=24, azim=-56)
    axis.set_xlabel("X forward (m)")
    axis.set_ylabel("Y left (m)")
    axis.set_zlabel("Z up (m)")
    axis.set_title("TZCup formal sanitation vehicle — zero-joint engineering preview", pad=18)
    figure.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output_path, dpi=180, bbox_inches="tight")
    plt.close(figure)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    render(args.urdf, args.output)
    print(args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
