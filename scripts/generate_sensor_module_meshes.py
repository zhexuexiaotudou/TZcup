#!/usr/bin/env python3
"""Generate project-owned external CAD for SKU-pending sensor mounts/housings."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import trimesh


def moved(mesh, xyz=(0, 0, 0), rpy=(0, 0, 0)):
    result = mesh.copy()
    matrix = trimesh.transformations.euler_matrix(*rpy, axes="sxyz")
    matrix[:3, 3] = xyz
    result.apply_transform(matrix)
    return result


def box(extents, xyz=(0, 0, 0), rpy=(0, 0, 0)):
    return moved(trimesh.creation.box(extents=extents), xyz, rpy)


def cylinder(radius, height, xyz=(0, 0, 0), rpy=(0, 0, 0), sections=40):
    return moved(trimesh.creation.cylinder(radius=radius, height=height, sections=sections), xyz, rpy)


def fisheye_module():
    """Rugged 38 mm board-camera enclosure; optical axis is local +X."""
    parts = [
        box((0.020, 0.038, 0.038)),
        cylinder(0.014, 0.012, (0.016, 0, 0), (0, np.pi / 2, 0), 56),
        cylinder(0.0105, 0.010, (0.026, 0, 0), (0, np.pi / 2, 0), 56),
        cylinder(0.006, 0.010, (-0.015, 0, 0), (0, np.pi / 2, 0), 32),
        box((0.010, 0.056, 0.006), (-0.006, 0, -0.022)),
    ]
    for y in (-0.023, 0.023):
        parts.append(cylinder(0.0035, 0.008, (-0.006, y, -0.022), sections=20))
    for y in (-0.014, 0.014):
        for z in (-0.014, 0.014):
            parts.append(cylinder(0.0022, 0.0025, (0.011, y, z), (0, np.pi / 2, 0), 16))
    return trimesh.util.concatenate(parts)


def wrist_rgbd_bracket():
    """Side-mount L bracket with gussets and fastener heads, local to wrist camera."""
    parts = [
        box((0.120, 0.008, 0.050), (-0.0575, 0.035, 0)),
        box((0.055, 0.055, 0.006), (0, 0.010, -0.027)),
        box((0.008, 0.060, 0.072), (-0.115, 0.005, 0)),
        box((0.100, 0.006, 0.012), (-0.060, 0.005, -0.020), (0, -0.20, 0)),
    ]
    for x in (-0.018, 0.018):
        for z in (-0.027, 0.027):
            parts.append(cylinder(0.0035, 0.004, (x, 0.004, z), (np.pi / 2, 0, 0), 20))
    for y in (-0.018, 0.018):
        parts.append(cylinder(0.004, 0.005, (-0.119, y, 0.022), (0, np.pi / 2, 0), 20))
    return trimesh.util.concatenate(parts)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    for name, mesh in {
        "rugged_fisheye_module.stl": fisheye_module(),
        "wrist_rgbd_side_bracket.stl": wrist_rgbd_bracket(),
    }.items():
        mesh.remove_unreferenced_vertices()
        mesh.export(args.output / name)
        print(f"{name}: {len(mesh.vertices)} vertices, {len(mesh.faces)} faces")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
