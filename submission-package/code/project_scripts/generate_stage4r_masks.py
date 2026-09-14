#!/usr/bin/env python3
"""Generate distinct keepout and speed masks aligned to a saved SLAM map."""

import argparse
from pathlib import Path

from PIL import Image
import yaml


def world_to_pixel(x, y, origin, resolution, height):
    column = int((x - origin[0]) / resolution)
    row_from_bottom = int((y - origin[1]) / resolution)
    return column, height - 1 - row_from_bottom


def fill_rectangle(image, bounds, value, origin, resolution):
    pixels = image.load(); width, height = image.size
    left, top = world_to_pixel(bounds[0], bounds[3], origin, resolution, height)
    right, bottom = world_to_pixel(bounds[2], bounds[1], origin, resolution, height)
    for row in range(max(0, top), min(height, bottom + 1)):
        for column in range(max(0, left), min(width, right + 1)):
            pixels[column, row] = value


def write_yaml(path, image_name, metadata):
    content = {
        "image": image_name, "mode": "scale", "resolution": metadata["resolution"],
        "origin": metadata["origin"], "negate": 0, "occupied_thresh": 0.65,
        "free_thresh": 0.25,
    }
    path.write_text(yaml.safe_dump(content, sort_keys=False), encoding="utf-8")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--map-yaml", required=True)
    parser.add_argument("--output-dir", required=True)
    args = parser.parse_args()
    source_yaml = Path(args.map_yaml); metadata = yaml.safe_load(source_yaml.read_text(encoding="utf-8"))
    source_image = Image.open(source_yaml.parent / metadata["image"]).convert("L")
    output = Path(args.output_dir); output.mkdir(parents=True, exist_ok=True)
    # Nav2 mask convention here: white=free/0, black=100.
    keepout = Image.new("L", source_image.size, 255)
    fill_rectangle(keepout, (2.0, 1.0, 4.0, 3.0), 0, metadata["origin"], metadata["resolution"])
    keepout.save(output / "keepout_mask.pgm")
    write_yaml(output / "keepout_mask.yaml", "keepout_mask.pgm", metadata)
    speed = Image.new("L", source_image.size, 255)
    # 50% gray creates a distinct speed-restricted region.
    fill_rectangle(speed, (-2.0, -2.0, 2.0, 2.0), 127, metadata["origin"], metadata["resolution"])
    speed.save(output / "speed_mask.pgm")
    write_yaml(output / "speed_mask.yaml", "speed_mask.pgm", metadata)


if __name__ == "__main__": main()
