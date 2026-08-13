#!/usr/bin/env python3
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "starter_ws/src/sanitation_learning"))
from sanitation_learning.g6_small_specialist import map_tile_box_to_native, rgdrv8_ground_roi_tiles  # noqa: E402


def test_rgdrv8_fixed_tiles_cover_full_calibrated_camera_mask() -> None:
    tiles = rgdrv8_ground_roi_tiles()
    assert len(tiles) == 9
    assert tiles[0] == (0, 0, 320, 240)
    assert tiles[-1] == (320, 240, 640, 480)
    for x in range(640):
        for y in range(480):
            assert any(x0 <= x < x1 and y0 <= y < y1 for x0, y0, x1, y1 in tiles)


def test_rgdrv8_tile_native_roundtrip_is_exact() -> None:
    assert map_tile_box_to_native([0, 0, 640, 480], (160, 120, 480, 360)) == [160.0, 120.0, 480.0, 360.0]


def test_rgdrv8_roi_function_has_no_target_argument() -> None:
    assert rgdrv8_ground_roi_tiles.__code__.co_varnames[:2] == ("width", "height")
