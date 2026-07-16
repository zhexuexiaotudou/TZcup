#!/usr/bin/env python3
"""Compare a saved SLAM occupancy map with fixed SDF box geometry."""

from __future__ import annotations

import argparse
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
from scipy import ndimage
from scipy.optimize import minimize
from scipy.spatial import cKDTree
import yaml


IGNORED_MODELS = {"asphalt_ground", "north_sidewalk", "south_sidewalk", "puddle_zone", "world_overview_camera", "dynamic_pedestrian_box"}


def pose_values(element):
    values = [float(value) for value in (element.text or "0 0 0 0 0 0").split()]
    return (values + [0.0] * 6)[:6]


def sdf_boxes(path, minimum_height=0.20, lidar_height=None):
    root = ET.parse(path).getroot(); boxes = []
    for model in root.findall(".//world/model"):
        name = model.attrib.get("name", "")
        if name in IGNORED_MODELS or model.findtext("static", "false").lower() != "true": continue
        model_pose = pose_values(model.find("pose")) if model.find("pose") is not None else [0.0] * 6
        for collision in model.findall("./link/collision"):
            size = collision.find("./geometry/box/size")
            if size is None: continue
            sx, sy, sz = (float(value) for value in size.text.split())
            if sz < minimum_height: continue
            local = pose_values(collision.find("pose")) if collision.find("pose") is not None else [0.0] * 6
            center_z = model_pose[2] + local[2]
            if lidar_height is not None and not (
                center_z - sz / 2.0 <= lidar_height <= center_z + sz / 2.0
            ):
                continue
            boxes.append({"name": name, "x": model_pose[0] + local[0], "y": model_pose[1] + local[1], "z": center_z, "yaw": model_pose[5] + local[5], "size_x": sx, "size_y": sy, "size_z": sz})
    return boxes


def world_to_grid(x, y, metadata, height):
    origin = metadata["origin"]; resolution = float(metadata["resolution"])
    column = (x - float(origin[0])) / resolution
    row = height - 1 - (y - float(origin[1])) / resolution
    return row, column


def rasterize_truth(shape, metadata, boxes, alignment):
    mask = np.zeros(shape, dtype=bool); polygons = []
    for box in boxes:
        cosine, sine = math.cos(box["yaw"] + alignment[2]), math.sin(box["yaw"] + alignment[2])
        center_x = box["x"] * math.cos(alignment[2]) - box["y"] * math.sin(alignment[2]) + alignment[0]
        center_y = box["x"] * math.sin(alignment[2]) + box["y"] * math.cos(alignment[2]) + alignment[1]
        corners = []
        for dx, dy in ((-box["size_x"]/2, -box["size_y"]/2), (box["size_x"]/2, -box["size_y"]/2), (box["size_x"]/2, box["size_y"]/2), (-box["size_x"]/2, box["size_y"]/2)):
            x = center_x + dx * cosine - dy * sine; y = center_y + dx * sine + dy * cosine
            corners.append((x, y))
        rows_cols = [world_to_grid(x, y, metadata, shape[0]) for x, y in corners]
        rr = [point[0] for point in rows_cols]; cc = [point[1] for point in rows_cols]
        min_r, max_r = max(0, int(math.floor(min(rr)))), min(shape[0]-1, int(math.ceil(max(rr))))
        min_c, max_c = max(0, int(math.floor(min(cc)))), min(shape[1]-1, int(math.ceil(max(cc))))
        polygon_xy = np.asarray(corners)
        for row in range(min_r, max_r + 1):
            for column in range(min_c, max_c + 1):
                x = float(metadata["origin"][0]) + (column + 0.5) * float(metadata["resolution"])
                y = float(metadata["origin"][1]) + (shape[0] - row - 0.5) * float(metadata["resolution"])
                inside = False; previous = polygon_xy[-1]
                for current in polygon_xy:
                    if (current[1] > y) != (previous[1] > y) and x < (previous[0] - current[0]) * (y - current[1]) / (previous[1] - current[1]) + current[0]: inside = not inside
                    previous = current
                if inside: mask[row, column] = True
        polygons.append({**box, "map_corners": corners})
    return mask, polygons


def boundary(mask):
    return mask & ~ndimage.binary_erosion(mask)


def grid_points_to_world(points, metadata, height):
    resolution = float(metadata["resolution"]); origin = metadata["origin"]
    return np.column_stack((
        float(origin[0]) + (points[:, 1] + 0.5) * resolution,
        float(origin[1]) + (height - points[:, 0] - 0.5) * resolution,
    ))


def box_boundary_points(boxes, spacing):
    points = []
    for box in boxes:
        count_x = max(2, int(math.ceil(box["size_x"] / spacing)) + 1)
        count_y = max(2, int(math.ceil(box["size_y"] / spacing)) + 1)
        local = []
        for x in np.linspace(-box["size_x"] / 2, box["size_x"] / 2, count_x):
            local.extend(((x, -box["size_y"] / 2), (x, box["size_y"] / 2)))
        for y in np.linspace(-box["size_y"] / 2, box["size_y"] / 2, count_y):
            local.extend(((-box["size_x"] / 2, y), (box["size_x"] / 2, y)))
        c, s = math.cos(box["yaw"]), math.sin(box["yaw"])
        for x, y in local:
            points.append((box["x"] + c * x - s * y, box["y"] + s * x + c * y))
    return np.asarray(points, dtype=float)


def fit_rigid_alignment(observed, metadata, boxes, initial):
    observed_pixels = np.argwhere(boundary(observed))
    observed_world = grid_points_to_world(observed_pixels, metadata, observed.shape[0])
    tree = cKDTree(observed_world)
    truth_world = box_boundary_points(boxes, max(0.01, float(metadata["resolution"]) / 2.0))

    def objective(values):
        x, y, yaw = values; c, s = math.cos(yaw), math.sin(yaw)
        transformed = np.column_stack((
            c * truth_world[:, 0] - s * truth_world[:, 1] + x,
            s * truth_world[:, 0] + c * truth_world[:, 1] + y,
        ))
        distances = tree.query(transformed)[0]
        # Cap outliers from temporary scan artifacts while fitting the global
        # map frame; the full uncapped symmetric metrics are computed later.
        return float(np.mean(np.minimum(distances, 0.50)))

    bounds = ((initial[0] - 0.50, initial[0] + 0.50), (initial[1] - 0.50, initial[1] + 0.50), (initial[2] - 0.06, initial[2] + 0.06))
    result = minimize(objective, np.asarray(initial, dtype=float), method="Powell", bounds=bounds, options={"xtol": 1.0e-5, "ftol": 1.0e-6, "maxiter": 200})
    return tuple(float(value) for value in result.x), {
        "method": "Powell robust truth-boundary to SLAM-boundary registration",
        "initial_alignment": {"x_m": initial[0], "y_m": initial[1], "yaw_rad": initial[2]},
        "initial_objective_m": objective(initial),
        "final_objective_m": objective(result.x),
        "optimizer_success": bool(result.success),
        "optimizer_message": str(result.message),
    }


def metrics(observed, truth, resolution):
    intersection = int(np.count_nonzero(observed & truth)); union = int(np.count_nonzero(observed | truth))
    observed_boundary = np.argwhere(boundary(observed)); truth_boundary = np.argwhere(boundary(truth))
    distances = []
    if len(observed_boundary) and len(truth_boundary):
        tree_observed = cKDTree(observed_boundary); tree_truth = cKDTree(truth_boundary)
        distances.extend(tree_observed.query(truth_boundary)[0] * resolution)
        distances.extend(tree_truth.query(observed_boundary)[0] * resolution)
    distances = np.asarray(distances, dtype=float)
    dilation = ndimage.binary_dilation(truth, iterations=max(1, int(math.ceil(0.20 / resolution))))
    ghost = int(np.count_nonzero(observed & ~dilation))
    return {
        "occupancy_iou": intersection / union if union else 0.0,
        "boundary_chamfer_distance_m": float(distances.mean()) if len(distances) else None,
        "boundary_rmse_m": float(np.sqrt(np.mean(distances ** 2))) if len(distances) else None,
        "boundary_p95_m": float(np.percentile(distances, 95)) if len(distances) else None,
        "loop_ghosting_occupied_cells": ghost,
        "loop_ghosting_ratio": ghost / max(1, int(np.count_nonzero(observed))),
    }


def dominant_axis(points):
    if len(points) < 5: return None
    centered = points - points.mean(axis=0)
    values, vectors = np.linalg.eigh(np.cov(centered.T))
    vector = vectors[:, int(np.argmax(values))]
    return math.atan2(-float(vector[0]), float(vector[1]))


def straight_line_errors(observed, truth, resolution):
    labels, count = ndimage.label(truth); errors = []
    radius = max(1, int(math.ceil(0.30 / resolution)))
    for label in range(1, count + 1):
        component = labels == label
        if np.count_nonzero(component) < 10: continue
        nearby = observed & ndimage.binary_dilation(component, iterations=radius)
        expected = dominant_axis(np.argwhere(component)); measured = dominant_axis(np.argwhere(nearby))
        if expected is None or measured is None: continue
        difference = abs((measured - expected) % math.pi)
        errors.append(math.degrees(min(difference, math.pi - difference)))
    return {
        "sample_count": len(errors),
        "rmse": math.sqrt(sum(value * value for value in errors) / len(errors)) if errors else None,
        "p95": float(np.percentile(errors, 95)) if errors else None,
        "max": max(errors) if errors else None,
    }


def main():
    parser = argparse.ArgumentParser(); parser.add_argument("--map-yaml", required=True, type=Path); parser.add_argument("--world-sdf", required=True, type=Path); parser.add_argument("--output", required=True, type=Path); parser.add_argument("--overlay", required=True, type=Path); parser.add_argument("--alignment-x", type=float, default=8.0); parser.add_argument("--alignment-y", type=float, default=0.0); parser.add_argument("--alignment-yaw", type=float, default=0.0)
    args = parser.parse_args(); metadata = yaml.safe_load(args.map_yaml.read_text(encoding="utf-8")); image = np.asarray(Image.open(args.map_yaml.parent / metadata["image"]).convert("L"))
    negate = int(metadata.get("negate", 0)); probability = image / 255.0 if negate else (255.0 - image) / 255.0
    observed = probability >= float(metadata.get("occupied_thresh", 0.65))
    unknown = (image == 205) | ((probability > float(metadata.get("free_thresh", 0.25))) & ~observed)
    boxes = sdf_boxes(args.world_sdf)
    alignment, registration = fit_rigid_alignment(observed, metadata, boxes, (args.alignment_x, args.alignment_y, args.alignment_yaw))
    truth, polygons = rasterize_truth(observed.shape, metadata, boxes, alignment)
    report = {
        "schema_version": 1,
        "map_resolution_m": float(metadata["resolution"]),
        "rigid_alignment": {"x_m": alignment[0], "y_m": alignment[1], "yaw_rad": alignment[2], "source": "optimized SDF fixed-obstacle boundary to SLAM occupied boundary", **registration},
        "truth_source": str(args.world_sdf),
        "truth_box_count": len(boxes),
        "truth_boxes": boxes,
        "known_area_m2": float(np.count_nonzero(~unknown) * float(metadata["resolution"]) ** 2),
        "unknown_ratio": float(np.count_nonzero(unknown) / unknown.size),
        "straight_line_angle_error_deg": straight_line_errors(observed, truth, float(metadata["resolution"])),
        **metrics(observed, truth, float(metadata["resolution"])),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True); args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    figure, axes = plt.subplots(1, 3, figsize=(16, 6), constrained_layout=True)
    axes[0].imshow(observed, cmap="gray_r"); axes[0].set_title("SLAM occupied")
    axes[1].imshow(truth, cmap="gray_r"); axes[1].set_title("SDF fixed-obstacle truth")
    overlay = np.zeros((*observed.shape, 3), dtype=float); overlay[..., 0] = observed; overlay[..., 1] = truth; overlay[..., 2] = observed & truth
    axes[2].imshow(overlay); axes[2].set_title("red=SLAM, green=truth, white=overlap")
    for axis in axes: axis.axis("off")
    figure.savefig(args.overlay, dpi=160); plt.close(figure)


if __name__ == "__main__": main()
