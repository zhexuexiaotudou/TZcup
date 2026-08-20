#!/usr/bin/env python3
"""Generate a Nav2 keepout mask from the Ackermann outer turning apron."""

from __future__ import annotations

import argparse
from pathlib import Path

from PIL import Image, ImageDraw
import yaml


def world_to_pixel(x, y, origin, resolution, height):
    column = (float(x) - float(origin[0])) / float(resolution)
    row_from_bottom = (float(y) - float(origin[1])) / float(resolution)
    return int(round(column)), int(round(height - 1 - row_from_bottom))


def build_mask(map_yaml: Path, mission_yaml: Path, output_dir: Path) -> Path:
    map_metadata = yaml.safe_load(map_yaml.read_text(encoding="utf-8"))
    mission = yaml.safe_load(mission_yaml.read_text(encoding="utf-8"))
    polygon = mission.get("outer_polygon")
    if not polygon or len(polygon) < 3:
        raise ValueError("mission outer_polygon must contain at least 3 points")
    source_image = Image.open(map_yaml.parent / map_metadata["image"])
    mask = Image.new("L", source_image.size, 0)
    draw = ImageDraw.Draw(mask)
    pixels = [
        world_to_pixel(
            point[0], point[1], map_metadata["origin"],
            map_metadata["resolution"], source_image.height,
        )
        for point in polygon
    ]
    # This repository uses white=free/0 and black=keepout/100.
    draw.polygon(pixels, fill=255)
    output_dir.mkdir(parents=True, exist_ok=True)
    image_path = output_dir / "ackermann_keepout_mask.pgm"
    yaml_path = output_dir / "ackermann_keepout_mask.yaml"
    mask.save(image_path)
    metadata = {
        "image": image_path.name,
        "mode": "scale",
        "resolution": map_metadata["resolution"],
        "origin": map_metadata["origin"],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.25,
    }
    yaml_path.write_text(
        yaml.safe_dump(metadata, sort_keys=False), encoding="utf-8"
    )
    return yaml_path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--map-yaml", type=Path, required=True)
    parser.add_argument("--mission-yaml", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    print(build_mask(args.map_yaml, args.mission_yaml, args.output_dir))


if __name__ == "__main__":
    main()
