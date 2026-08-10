#!/usr/bin/env python3

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "starter_ws/src/sanitation_learning"))

from sanitation_learning.g4_tiled_fcos import ground_tile_specs, map_tile_box


def test_ground_tile_contracts_are_bounded_inside_native_frame():
    for mode, count in (("ground3", 3), ("ground2x2", 4)):
        tiles = ground_tile_specs(640, 480, mode)
        assert len(tiles) == count
        assert all(0 <= x1 < x2 <= 640 and 0 <= y1 < y2 <= 480 for x1, y1, x2, y2 in tiles)
        assert all(y1 >= 144 for _, y1, _, _ in tiles)


def test_tile_box_maps_back_to_full_model_coordinates():
    mapped = map_tile_box(
        [0, 0, 640, 480], (160, 144, 480, 480),
        (640, 480), (960, 720), (640, 480),
    )
    assert mapped == [240.0, 216.0, 720.0, 720.0]
