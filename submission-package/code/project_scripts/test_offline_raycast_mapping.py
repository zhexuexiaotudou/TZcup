import math
from pathlib import Path
import sys

import numpy as np
from PIL import Image
import pytest


sys.path.insert(0, str(Path(__file__).resolve().parent))

from offline_raycast_mapping import (  # noqa: E402
    GridSpec,
    OfflineMappingError,
    Pose,
    cast_scan,
    classification_counts,
    quality_metrics,
    sample_polyline,
    write_map_bundle,
)
from verify_map_area import assess_map  # noqa: E402


def test_cast_scan_stops_at_first_occupied_cell_and_marks_free_prefix():
    grid = GridSpec(
        min_x_m=0.0,
        min_y_m=0.0,
        resolution_m=1.0,
        width=8,
        height=5,
    )
    occupancy = np.zeros((grid.height, grid.width), dtype=bool)
    occupancy[2, 5] = True
    scan = cast_scan(
        occupancy,
        grid,
        Pose(0.5, 2.5, 0.0),
        angles_rad=np.array([0.0, math.pi]),
        max_range_m=7.0,
        ray_step_m=0.25,
    )
    assert 2 * grid.width + 5 in set(scan.occupied_indices.tolist())
    assert 2 * grid.width + 4 in set(scan.free_indices.tolist())
    assert 2 * grid.width + 6 not in set(scan.free_indices.tolist())
    assert math.isclose(scan.ranges_m[0], 4.5, abs_tol=0.25)
    assert math.isinf(scan.ranges_m[1])


def test_sample_polyline_preserves_endpoints_and_cadence_deterministically():
    first = sample_polyline([(0.0, 0.0), (3.0, 0.0), (3.0, 4.0)], 1.0)
    second = sample_polyline([(0.0, 0.0), (3.0, 0.0), (3.0, 4.0)], 1.0)
    assert first == second
    assert (first[0].x_m, first[0].y_m) == (0.0, 0.0)
    assert (first[-1].x_m, first[-1].y_m) == (3.0, 4.0)
    assert all(
        math.hypot(right.x_m - left.x_m, right.y_m - left.y_m) <= 1.0 + 1e-12
        for left, right in zip(first, first[1:])
    )
    with pytest.raises(OfflineMappingError, match="spacing"):
        sample_polyline([(0.0, 0.0), (1.0, 0.0)], 0.0)


def test_area_accounting_matches_existing_verifier(tmp_path: Path):
    pixels = np.array(
        [
            [0, 254, 205],
            [254, 254, 0],
        ],
        dtype=np.uint8,
    )
    counts = classification_counts(pixels)
    assert counts == {
        "occupied_cells": 2,
        "free_cells": 3,
        "unknown_cells": 1,
        "known_cells": 5,
    }
    grid = GridSpec(
        min_x_m=0.0,
        min_y_m=0.0,
        resolution_m=0.5,
        width=3,
        height=2,
    )
    yaml_path, _ = write_map_bundle(tmp_path, pixels, grid)
    report = assess_map(yaml_path, root=tmp_path, minimum_area_m2=1.0)
    assert report["occupied_cells"] == 2
    assert report["free_cells"] == 3
    assert report["unknown_cells"] == 1
    assert report["known_area_m2"] == pytest.approx(1.25)
    assert report["area_gate"] is True
    assert np.asarray(
        Image.open(tmp_path / report["map_image"]).convert("L")
    ).tolist() == list(reversed(pixels.tolist()))


def test_quality_metrics_require_boundary_and_free_space_recall():
    grid = GridSpec(
        min_x_m=0.0,
        min_y_m=0.0,
        resolution_m=1.0,
        width=7,
        height=7,
    )
    source = np.zeros((7, 7), dtype=bool)
    source[2:5, 2:5] = True
    reconstructed = np.full((7, 7), 254, dtype=np.uint8)
    reconstructed[source] = 205
    reconstructed[2, 2:5] = 0
    metrics = quality_metrics(
        reconstructed,
        source,
        grid,
        field_bounds_m=(0.0, 0.0, 7.0, 7.0),
    )
    assert metrics["source_occupied_boundary_recall"] > 0.0
    assert metrics["source_free_recall"] == pytest.approx(1.0)
    assert metrics["quality_gate"] is False
