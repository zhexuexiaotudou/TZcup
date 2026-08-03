import math

from sanitation_coverage.residual_region_planner import (
    connected_residual_regions,
    plan_residual_regions,
    trim_swept_endcaps,
)


def test_separated_residual_islands_are_not_stitched_together():
    points = [(0, 0), (0.1, 0), (2, 2), (2.1, 2)]
    regions = connected_residual_regions(points, 0.1)
    assert len(regions) == 2
    plan = plan_residual_regions(points, 0.1, 0.4, 20.0)
    assert len(plan.regions) == 2
    assert all(abs(a[0] - b[0]) < 1 or abs(a[1] - b[1]) < 1 for a, b in plan.swaths)


def test_repair_length_is_capped_at_ten_percent():
    points = [(x / 10, 0) for x in range(20)]
    plan = plan_residual_regions(points, 0.1, 0.4, primary_length_m=5.0)
    assert plan.total_length_m <= 0.5
    assert plan.truncated


def test_adjacent_grid_rows_share_one_brush_width_repair_swath():
    points = [(x / 10, y / 10) for y in range(4) for x in range(11)]
    plan = plan_residual_regions(points, 0.1, 0.65, primary_length_m=20.0)
    assert len(plan.regions) == 1
    assert len(plan.swaths) == 1
    assert plan.total_length_m <= 2.0


def test_brush_centre_segment_does_not_double_extend_its_swept_endcaps():
    points = [(x / 10, 0.0) for x in range(13)]
    plan = plan_residual_regions(points, 0.1, 0.65, primary_length_m=20.0)

    assert plan.swaths == (((0.0, 0.0), (1.2, 0.0)),)
    assert plan.total_length_m == 1.2


def test_single_cell_residual_has_one_cell_physical_heading():
    plan = plan_residual_regions([(0.0, 0.0)], 0.1, 0.65, primary_length_m=5.0)

    assert len(plan.swaths) == 1
    assert math.isclose(plan.total_length_m, 0.1)
    assert plan.swaths[0][0] != plan.swaths[0][1]


def test_multicell_repair_trims_brush_swept_endcaps():
    trimmed = trim_swept_endcaps(((0.0, 0.0), (1.2, 0.0)), 0.65, 0.1)

    assert math.isclose(trimmed[0][0], 0.1625)
    assert math.isclose(trimmed[1][0], 1.0375)
    assert math.isclose(math.dist(*trimmed), 0.875)


def test_single_cell_repair_is_not_trimmed_to_zero():
    segment = ((-0.05, 0.0), (0.05, 0.0))

    assert trim_swept_endcaps(segment, 0.65, 0.1) == segment
