import math

import pytest

from sanitation_coverage.metrics import (
    empirical_swept_metrics,
    semantic_path_distances,
    path_length,
    projected_path_progress,
    raster_coverage_metrics,
    repair_degenerate_swaths,
    summarize_distances,
    swath_absolute_cross_track_errors,
    swath_lateral_errors,
    swath_straightness_errors,
    synchronized_pose_errors,
    synchronized_xy_errors,
    uncovered_cell_centers,
    horizontal_repair_segments,
)


def test_projected_path_progress_uses_arc_distance_not_callback_rate():
    points = [(0.0, 0.0), (1.0, 0.0), (1.0, 2.0)]
    assert projected_path_progress(points, (0.4, 0.2)) == pytest.approx(0.4)
    assert projected_path_progress(points, (1.2, 0.75)) == pytest.approx(1.75)


def test_summarize_distances_reports_rmse_and_tail():
    summary = summarize_distances([0.01, 0.02, 0.03, 0.04])
    assert summary['sample_count'] == 4
    assert summary['p95_m'] == 0.04
    assert summary['max_m'] == 0.04
    assert summary['mean_m'] == pytest.approx(0.025)
    assert summary['median_m'] == pytest.approx(0.025)
    assert 0.02 < summary['rmse_m'] < 0.04


def test_synchronized_xy_errors_use_ros_time_not_callback_arrival_order():
    estimates = [(1.00, 1.0, 0.0, 0.0), (1.10, 2.0, 0.0, 0.0)]
    truths = [
        (0.98, 0.98, 0.0, 0.0, False),
        (1.02, 1.02, 0.0, 0.0, False),
        (1.10, 2.01, 0.0, 0.0, False),
    ]
    errors, sync_errors, dropped = synchronized_xy_errors(
        estimates, truths, tolerance_sec=0.05
    )
    assert dropped == 0
    assert len(errors) == 2
    assert max(errors) <= 0.0200001
    assert max(sync_errors) <= 0.0200001


def test_synchronized_xy_errors_interpolate_fast_linear_motion():
    estimates = [(0.05, 0.05, 0.0, 0.0)]
    truths = [
        (0.00, 0.00, 0.0, 0.0, False),
        (0.10, 0.10, 0.0, 0.0, False),
    ]
    errors, sync_errors, dropped = synchronized_xy_errors(
        estimates, truths, tolerance_sec=0.05
    )
    assert dropped == 0
    assert errors == [0.0]
    assert sync_errors == [0.05]


def test_synchronized_xy_errors_drop_unsupported_truth_gap():
    estimates = [(0.10, 0.10, 0.0, 0.0)]
    truths = [
        (0.00, 0.00, 0.0, 0.0, False),
        (0.21, 0.21, 0.0, 0.0, False),
    ]
    errors, sync_errors, dropped = synchronized_xy_errors(
        estimates, truths, tolerance_sec=0.05
    )
    assert errors == []
    assert sync_errors == []
    assert dropped == 1


def test_synchronized_pose_errors_interpolate_wrapped_yaw():
    estimates = [(0.05, 0.05, 0.0, -math.pi)]
    truths = [
        (0.00, 0.00, 0.0, math.radians(179.0), False),
        (0.10, 0.10, 0.0, math.radians(-179.0), False),
    ]
    xy, yaw, sync, dropped = synchronized_pose_errors(estimates, truths, 0.05)
    assert dropped == 0
    assert xy == [0.0]
    assert yaw[0] == pytest.approx(0.0, abs=1e-12)
    assert sync == [0.05]


def test_empirical_metrics_use_brush_on_ground_truth_points():
    polygon = [(0.0, 0.0), (2.0, 0.0), (2.0, 1.0), (0.0, 1.0)]
    points = [(index * 0.1, index * 0.1, 0.5) for index in range(21)]
    metrics = empirical_swept_metrics(polygon, points, 1.0, resolution=0.1)
    assert metrics["coverage_rate"] > 0.9
    assert metrics["metric_basis"].startswith("gazebo_ground_truth")


def test_path_length():
    assert path_length([(0.0, 0.0), (3.0, 4.0)]) == 5.0


def test_semantic_path_distances_split_brush_motion_and_transitions():
    samples = [
        (0.0, 0.0, 0.0, 0.0, False, "TRANSIT"),
        (1.0, 1.0, 0.0, 0.0, False, "TRANSIT"),
        (2.0, 2.0, 0.0, 0.0, True, "EXECUTING_SWATH"),
        (3.0, 3.0, 0.0, 0.0, True, "EXECUTING_SWATH"),
    ]
    distances = semantic_path_distances(samples)
    assert distances["brush_off_distance_m"] == 1.0
    assert distances["brush_transition_distance_m"] == 1.0
    assert distances["brush_on_distance_m"] == 1.0
    assert distances["total_distance_m"] == 3.0


def test_swath_lateral_errors_use_brush_center_and_ignore_connectors():
    samples = [
        (0.0, 0.0, 0.04, 0.0, True, "EXECUTING_SWATH"),
        (1.0, 1.0, 0.50, 0.0, False, "EXECUTING_SHIFT"),
    ]
    errors = swath_lateral_errors(
        samples, [((0.0, 0.0), (2.0, 0.0))], brush_forward_offset_m=0.55
    )
    assert errors == [0.04]


def test_swath_straightness_separates_weave_from_constant_map_offset():
    samples = [
        (float(index), float(index) * 0.2, 0.10, 0.0, True, "EXECUTING_SWATH")
        for index in range(11)
    ]
    swaths = [((0.0, 0.0), (2.0, 0.0))]
    absolute = swath_absolute_cross_track_errors(
        samples, swaths, brush_forward_offset_m=0.0
    )
    straightness = swath_straightness_errors(
        samples, swaths, brush_forward_offset_m=0.0
    )
    assert min(absolute) == 0.10
    assert max(straightness) < 1.0e-12


def test_swath_straightness_detects_lateral_weave_and_splits_runs():
    samples = [
        (0.0, 0.0, 0.00, 0.0, True, "EXECUTING_SWATH"),
        (1.0, 0.5, 0.04, 0.0, True, "EXECUTING_SWATH"),
        (2.0, 1.0, -0.04, 0.0, True, "EXECUTING_SWATH"),
        (3.0, 1.0, 0.50, 0.0, False, "EXECUTING_SHIFT"),
        (4.0, 1.0, 1.02, 0.0, True, "EXECUTING_SWATH"),
        (5.0, 0.5, 0.98, 0.0, True, "EXECUTING_SWATH"),
        (6.0, 0.0, 1.00, 0.0, True, "EXECUTING_SWATH"),
    ]
    errors = swath_straightness_errors(
        samples,
        [((0.0, 0.0), (1.0, 0.0)), ((1.0, 1.0), (0.0, 1.0))],
        brush_forward_offset_m=0.0,
        endpoint_fraction=0.0,
    )
    assert len(errors) == 6
    assert max(errors) == 0.04


def test_uncovered_cells_form_bounded_horizontal_repair_segments():
    polygon = [(0.0, 0.0), (2.0, 0.0), (2.0, 1.0), (0.0, 1.0)]
    points = [(0.0, 0.5, 0.5)]
    missed = uncovered_cell_centers(polygon, points, 0.4, resolution=0.2)
    segments = horizontal_repair_segments(missed, polygon, 0.4)
    assert missed
    assert segments
    assert all(0.0 <= point[0] <= 2.0 for segment in segments for point in segment)
    assert all(segment[0][1] == segment[1][1] for segment in segments)


def test_single_swath_coverage_is_bounded():
    metrics = raster_coverage_metrics(
        [(0.0, 0.0), (2.0, 0.0), (2.0, 1.0), (0.0, 1.0)],
        [((0.0, 0.5), (2.0, 0.5))],
        width=1.0,
        resolution=0.1,
    )
    assert metrics["coverage_rate"] > 0.95
    assert metrics["coverage_rate"] <= 1.0
    assert metrics["repeat_rate"] == 0.0


def test_repair_degenerate_swaths_uses_turn_boundaries():
    swaths = [((0.0, 0.5), (0.0, 0.5)), ((2.0, 1.5), (2.0, 1.5))]
    turns = [[(2.0, 0.5), (2.0, 1.5)]]
    repaired, applied = repair_degenerate_swaths(
        swaths, turns, [(0.0, 0.5), (0.0, 1.5)]
    )
    assert applied
    assert repaired == [
        ((0.0, 0.5), (2.0, 0.5)),
        ((2.0, 1.5), (0.0, 1.5)),
    ]


def test_brush_center_swath_is_shifted_back_from_base_path():
    from sanitation_coverage.route_entry import brush_center_to_base_swath

    start, end = brush_center_to_base_swath(
        (0.0, 0.0), (4.0, 0.0), forward_offset_m=0.55, extension_m=0.20
    )
    assert start == (-0.75, 0.0)
    assert end == (3.65, 0.0)


def test_curvature_reversal_split_shares_primitive_boundaries():
    from sanitation_coverage.metrics import split_path_at_curvature_reversals

    points = [(float(index), 0.0) for index in range(7)]
    headings = [0.0, 0.1, 0.2, 0.1, 0.0, 0.1, 0.2]
    sections = split_path_at_curvature_reversals(points, headings)
    assert [section[0] for section in sections] == [
        points[0:3],
        points[2:5],
        points[4:7],
    ]
    assert sections[0][0][-1] == sections[1][0][0]
    assert sections[1][0][-1] == sections[2][0][0]


def test_curvature_reversal_split_keeps_straight_path_whole():
    from sanitation_coverage.metrics import split_path_at_curvature_reversals

    points = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
    headings = [0.0, 0.0, 0.0]
    assert split_path_at_curvature_reversals(points, headings) == [
        (points, headings)
    ]


def test_curvature_reversal_split_separates_curve_straight_curve():
    from sanitation_coverage.metrics import split_path_at_curvature_reversals

    points = [(float(index), 0.0) for index in range(7)]
    headings = [0.0, 0.1, 0.2, 0.2, 0.2, 0.3, 0.4]
    sections = split_path_at_curvature_reversals(points, headings)
    assert [section[0] for section in sections] == [
        points[0:3],
        points[2:5],
        points[4:7],
    ]


def test_path_heading_variation_accumulates_wrapped_turns():
    from sanitation_coverage.metrics import path_heading_variation

    assert math.isclose(
        path_heading_variation([0.0, math.pi / 2.0, math.pi]),
        math.pi,
    )
    assert math.isclose(
        path_heading_variation([3.1, -3.1]),
        2.0 * (math.pi - 3.1),
    )
