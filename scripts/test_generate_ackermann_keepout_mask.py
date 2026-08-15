#!/usr/bin/env python3
"""Contract tests for the Ackermann turning-apron keepout mask."""

from pathlib import Path
import sys

from PIL import Image
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from generate_ackermann_keepout_mask import build_mask, world_to_pixel


def test_outer_apron_is_free_and_outside_is_keepout(tmp_path):
    Image.new("L", (100, 100), 255).save(tmp_path / "map.pgm")
    map_yaml = tmp_path / "map.yaml"
    map_yaml.write_text(yaml.safe_dump({
        "image": "map.pgm",
        "resolution": 0.1,
        "origin": [-5.0, -5.0, 0.0],
    }), encoding="utf-8")
    mission_yaml = tmp_path / "mission.yaml"
    mission_yaml.write_text(yaml.safe_dump({
        "outer_polygon": [[-2.0, -2.0], [2.0, -2.0], [2.0, 2.0], [-2.0, 2.0]],
    }), encoding="utf-8")
    output_yaml = build_mask(map_yaml, mission_yaml, tmp_path / "filters")
    metadata = yaml.safe_load(output_yaml.read_text(encoding="utf-8"))
    mask = Image.open(output_yaml.parent / metadata["image"])
    inside = world_to_pixel(0.0, 0.0, [-5.0, -5.0], 0.1, 100)
    outside = world_to_pixel(4.0, 4.0, [-5.0, -5.0], 0.1, 100)
    assert mask.getpixel(inside) == 255
    assert mask.getpixel(outside) == 0


def test_launcher_routes_ackermann_to_generated_keepout():
    launcher = (ROOT / "scripts" / "run_visual_demo.sh").read_text(
        encoding="utf-8"
    )
    assert "generate_ackermann_keepout_mask.py" in launcher
    assert 'keepout_map="${ackermann_filter_dir}/ackermann_keepout_mask.yaml"' in launcher
