#!/usr/bin/env python3
"""Add realistic fixed features to the sparse world; never add hidden markers."""

from __future__ import annotations

import argparse
import hashlib
import json
import xml.etree.ElementTree as ET
from pathlib import Path


FEATURES = [
    {"name": "structured_building_north", "pose": (2.0, 9.0, 1.0, 0.0), "size": (24.0, 0.30, 2.0), "kind": "building_boundary"},
    {"name": "structured_curb_south", "pose": (6.0, -9.0, 0.25, 0.0), "size": (28.0, 0.25, 0.50), "kind": "curb_face"},
    {"name": "structured_lamp_west", "pose": (-3.0, 5.0, 0.75, 0.0), "size": (0.25, 0.25, 1.50), "kind": "lamp_post"},
    {"name": "structured_lamp_east", "pose": (11.0, -5.0, 0.75, 0.0), "size": (0.25, 0.25, 1.50), "kind": "lamp_post"},
    {"name": "structured_tree_south", "pose": (0.0, -6.0, 0.90, 0.0), "size": (0.45, 0.45, 1.80), "kind": "tree_trunk"},
    {"name": "structured_tree_north", "pose": (12.0, 6.0, 0.90, 0.0), "size": (0.45, 0.45, 1.80), "kind": "tree_trunk"},
    {"name": "structured_waste_bin", "pose": (-4.0, -2.0, 0.50, 0.15), "size": (0.80, 0.65, 1.00), "kind": "waste_bin"},
]


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def add_box_model(world, feature):
    model = ET.SubElement(world, "model", {"name": feature["name"]})
    ET.SubElement(model, "static").text = "true"
    x, y, z, yaw = feature["pose"]
    ET.SubElement(model, "pose").text = f"{x} {y} {z} 0 0 {yaw}"
    link = ET.SubElement(model, "link", {"name": "link"})
    for tag in ("collision", "visual"):
        node = ET.SubElement(link, tag, {"name": tag})
        geometry = ET.SubElement(node, "geometry")
        box = ET.SubElement(geometry, "box")
        ET.SubElement(box, "size").text = " ".join(str(value) for value in feature["size"])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--report", required=True, type=Path)
    args = parser.parse_args()
    tree = ET.parse(args.source)
    world = tree.getroot().find("world")
    if world is None:
        raise ValueError("source SDF has no world")
    world.set("name", "sanitation_structured_world")
    for feature in FEATURES:
        add_box_model(world, feature)
    ET.indent(tree, space="  ")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    tree.write(args.output, encoding="utf-8", xml_declaration=True)
    report = {
        "schema_version": 1,
        "generator_revision": 2,
        "source_world": str(args.source),
        "source_world_sha256": sha(args.source),
        "structured_world": str(args.output),
        "structured_world_sha256": sha(args.output),
        "feature_count": len(FEATURES),
        "features": FEATURES,
        "hidden_markers": False,
        "sparse_world_preserved": True,
        "structured_world_does_not_override_sparse_failure": True,
        "route_clearance_review": "v2 removes the rejected west boundary that blocked the first navigation segment",
    }
    args.report.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )


if __name__ == "__main__":
    main()
