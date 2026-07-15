import argparse
import json
from pathlib import Path

from PIL import Image
import yaml


def inspect_map(yaml_path):
    metadata = yaml.safe_load(Path(yaml_path).read_text(encoding="utf-8"))
    image_path = (Path(yaml_path).parent / metadata["image"]).resolve()
    image = Image.open(image_path).convert("L")
    pixels = list(image.getdata())
    negate = int(metadata.get("negate", 0))
    occupied_threshold = float(metadata.get("occupied_thresh", 0.65))
    free_threshold = float(metadata.get("free_thresh", 0.25))
    probabilities = [(value / 255.0 if negate else (255 - value) / 255.0) for value in pixels]
    explicit_unknown = [pixel == 205 for pixel in pixels]
    occupied = sum((not is_unknown) and value >= occupied_threshold for value, is_unknown in zip(probabilities, explicit_unknown))
    free = sum((not is_unknown) and value <= free_threshold for value, is_unknown in zip(probabilities, explicit_unknown))
    unknown = len(pixels) - occupied - free
    resolution = float(metadata["resolution"])
    width, height = image.size
    report = {
        "map_yaml": str(Path(yaml_path)), "map_image": str(image_path),
        "resolution_m": resolution, "width_cells": width, "height_cells": height,
        "span_x_m": width * resolution, "span_y_m": height * resolution,
        "occupied_cells": occupied, "free_cells": free, "unknown_cells": unknown,
        "known_cells": occupied + free, "known_area_m2": (occupied + free) * resolution ** 2,
    }
    report["resolution_pass"] = resolution <= 0.05
    report["span_pass"] = report["span_x_m"] >= 20.0 and report["span_y_m"] >= 10.0
    report["known_area_pass"] = report["known_area_m2"] >= 150.0
    report["slam_quality_pass"] = report["resolution_pass"] and report["span_pass"] and report["known_area_pass"]
    return report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--map-yaml", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--preview")
    args = parser.parse_args()
    report = inspect_map(args.map_yaml)
    if args.preview:
        Image.open(report["map_image"]).convert("L").save(args.preview)
    Path(args.output).write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    if not report["slam_quality_pass"]:
        raise SystemExit(2)
