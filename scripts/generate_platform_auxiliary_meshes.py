#!/usr/bin/env python3
"""Generate project-owned mechanical-detail meshes for platform auxiliaries.

The official A300, UR5e and 2F-85 geometry stays vendor supplied.  This script
only owns the vehicle-specific mast, adapter plate, control cabinet and compute
enclosure.  Dimensions are metres and match the corresponding URDF envelopes.
Meshes represent externally visible interface-level CAD; they do not pretend
to reproduce proprietary motor, PCB or cabinet internals.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import trimesh


def moved(mesh: trimesh.Trimesh, xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, 0.0)) -> trimesh.Trimesh:
    result = mesh.copy()
    matrix = trimesh.transformations.euler_matrix(*rpy, axes="sxyz")
    matrix[:3, 3] = xyz
    result.apply_transform(matrix)
    return result


def box(extents, xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, 0.0)) -> trimesh.Trimesh:
    return moved(trimesh.creation.box(extents=extents), xyz, rpy)


def cylinder(radius, height, xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, 0.0), sections=48):
    return moved(trimesh.creation.cylinder(radius=radius, height=height, sections=sections), xyz, rpy)


def rounded_box(extents, radius, xyz=(0.0, 0.0, 0.0)) -> trimesh.Trimesh:
    """Closed rounded planform enclosure assembled into a single mesh asset."""
    x, y, z = extents
    parts = [box((x - 2 * radius, y, z)), box((x, y - 2 * radius, z))]
    for sx in (-1, 1):
        for sy in (-1, 1):
            parts.append(cylinder(radius, z, (sx * (x / 2 - radius), sy * (y / 2 - radius), 0)))
    mesh = trimesh.util.concatenate(parts)
    mesh.apply_translation(xyz)
    return mesh


def triangular_gusset(length, height, thickness, xyz, flip=False):
    sign = -1.0 if flip else 1.0
    vertices = np.array([
        [0, -thickness / 2, 0], [sign * length, -thickness / 2, 0], [0, -thickness / 2, height],
        [0, thickness / 2, 0], [sign * length, thickness / 2, 0], [0, thickness / 2, height],
    ], dtype=float)
    faces = np.array([
        [0, 2, 1], [3, 4, 5], [0, 1, 4], [0, 4, 3],
        [1, 2, 5], [1, 5, 4], [2, 0, 3], [2, 3, 5],
    ])
    mesh = trimesh.Trimesh(vertices=vertices, faces=faces, process=True)
    mesh.apply_translation(xyz)
    return mesh


def sensor_mast() -> trimesh.Trimesh:
    parts = [
        rounded_box((0.060, 0.060, 0.720), 0.008, (0, 0, 0.360)),
        rounded_box((0.090, 0.480, 0.040), 0.010, (0, 0, 0.700)),
        box((0.130, 0.120, 0.010), (0, 0, 0.005)),
    ]
    # T-slot relief rails and corner gussets make the extrusion mechanically legible.
    for y in (-0.0275, 0.0275):
        parts.append(box((0.010, 0.006, 0.680), (0, y, 0.370)))
    for y in (-0.185, 0.0, 0.185):
        parts.append(box((0.070, 0.055, 0.008), (0, y, 0.725)))
    parts.extend([
        triangular_gusset(0.085, 0.120, 0.012, (0, 0.035, 0.020)),
        triangular_gusset(0.085, 0.120, 0.012, (0, -0.035, 0.020), flip=True),
    ])
    return trimesh.util.concatenate(parts)


def arm_mount() -> trimesh.Trimesh:
    parts = [cylinder(0.120, 0.030), cylinder(0.076, 0.046, (0, 0, 0.018))]
    for angle in np.linspace(0, 2 * np.pi, 6, endpoint=False):
        x, y = 0.096 * np.cos(angle), 0.096 * np.sin(angle)
        parts.append(cylinder(0.009, 0.008, (x, y, 0.019), sections=24))
        parts.append(box((0.060, 0.012, 0.020), (x * 0.55, y * 0.55, 0.015), (0, 0, angle)))
    return trimesh.util.concatenate(parts)


def control_box() -> trimesh.Trimesh:
    parts = [
        rounded_box((0.438, 0.240, 0.425), 0.018),
        rounded_box((0.010, 0.212, 0.382), 0.008, (0.224, 0, 0)),
        box((0.012, 0.040, 0.140), (0.228, -0.075, 0.020)),
    ]
    # Door hinges, latch, cable glands, feet and rear heat-sink ribs.
    for z in (-0.145, 0.145):
        parts.append(cylinder(0.008, 0.050, (0.222, 0.105, z), (np.pi / 2, 0, 0), 24))
    for y in (-0.075, -0.025, 0.025, 0.075):
        parts.append(cylinder(0.010, 0.014, (-0.222, y, -0.155), (0, np.pi / 2, 0), 24))
    for z in np.linspace(-0.150, 0.150, 9):
        parts.append(box((0.012, 0.200, 0.006), (-0.224, 0, z)))
    for y in (-0.095, 0.095):
        parts.append(box((0.050, 0.022, 0.018), (0, y, -0.220)))
    return trimesh.util.concatenate(parts)


def compute_enclosure() -> trimesh.Trimesh:
    parts = [
        rounded_box((0.198, 0.148, 0.078), 0.012),
        rounded_box((0.228, 0.178, 0.010), 0.008, (0, 0, -0.045)),
    ]
    # External heat-sink fins, connector bosses, fan guard and fasteners.
    for y in np.linspace(-0.058, 0.058, 9):
        parts.append(box((0.155, 0.004, 0.010), (-0.010, y, 0.044)))
    for y in (-0.045, 0, 0.045):
        parts.append(cylinder(0.010, 0.018, (0.108, y, 0), (0, np.pi / 2, 0), 24))
    parts.append(cylinder(0.036, 0.006, (-0.102, 0, 0), (0, np.pi / 2, 0), 48))
    for angle in np.linspace(0, np.pi, 5):
        parts.append(box((0.005, 0.065, 0.005), (-0.106, 0, 0), (angle, 0, 0)))
    for x in (-0.085, 0.085):
        for y in (-0.064, 0.064):
            parts.append(cylinder(0.0045, 0.006, (x, y, 0.042), sections=20))
    return trimesh.util.concatenate(parts)


def board_reference() -> trimesh.Trimesh:
    parts = [box((0.150, 0.100, 0.004))]
    for x, y, sx, sy, h in [
        (-0.030, 0.000, 0.045, 0.045, 0.008),
        (0.038, 0.027, 0.030, 0.018, 0.006),
        (0.038, -0.027, 0.030, 0.018, 0.006),
    ]:
        parts.append(box((sx, sy, h), (x, y, 0.002 + h / 2)))
    for y in np.linspace(-0.035, 0.035, 8):
        parts.append(box((0.014, 0.005, 0.006), (0.069, y, 0.005)))
    return trimesh.util.concatenate(parts)


GENERATORS = {
    "sensor_mast.stl": sensor_mast,
    "arm_mount_adapter.stl": arm_mount,
    "ur5e_control_cabinet.stl": control_box,
    "s100_compute_enclosure.stl": compute_enclosure,
    "s100_board_reference.stl": board_reference,
}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    for filename, factory in GENERATORS.items():
        mesh = factory()
        mesh.remove_unreferenced_vertices()
        mesh.export(args.output / filename)
        print(f"{filename}: {len(mesh.vertices)} vertices, {len(mesh.faces)} faces")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
