import math

import pytest

from sanitation_tasks.frontier_core import (
    frontier_goal_exclusion_centers,
    frontier_sweep_targets,
    frontier_sweep_target_axis,
    lane_shift_connector_goals,
    GridGeometry,
    frontier_clusters,
    grid_to_world,
    map_extent_metrics,
    mapping_completion_reached,
    next_adaptive_goal_distance,
    next_no_progress_frontier_state,
    prune_timed_exclusions,
    rank_frontiers,
    reverse_escape_goal,
    sweep_staging_goals,
    vertical_sweep_anchor_reached,
    world_disk_has_known_cell,
    world_disk_is_traversable,
)


def test_frontier_sweep_targets_cover_bounds_without_fixed_world_waypoints():
    targets = frontier_sweep_targets(
        (-100.0, -50.0, 100.0, 50.0),
        (0.0, 0.0),
        0.0,
        sensor_range_m=12.0,
        lane_overlap_m=2.0,
        boundary_margin_m=1.5,
    )
    assert len(targets) == 12
    assert targets[0] == pytest.approx((100.0, -10.0))
    assert targets[1] == pytest.approx((-100.0, -10.0))
    lane_y = sorted({target[1] for target in targets})
    assert lane_y == pytest.approx([-50.0, -30.0, -10.0, 10.0, 30.0, 50.0])
    assert max(right - left for left, right in zip(lane_y, lane_y[1:])) <= 20.0
    assert [
        frontier_sweep_target_axis(targets, index)
        for index in range(6)
    ] == [
        "horizontal", "horizontal", "vertical",
        "horizontal", "vertical", "horizontal",
    ]


def test_lane_shift_connector_goals_follow_online_pose_and_target_direction():
    goals = lane_shift_connector_goals(
        (-80.0, 8.0, math.pi),
        30.0,
        candidate_distances_m=(6.0, 4.0, 2.0),
        allowed_bounds_xyxy_m=(-100.0, -50.0, 100.0, 50.0),
        boundary_margin_m=1.5,
    )

    assert [(goal.world_x_m, goal.world_y_m) for goal in goals] == [
        (-80.0, 14.0),
        (-80.0, 12.0),
        (-80.0, 10.0),
    ]
    assert all(goal.yaw_rad == pytest.approx(math.pi / 2.0) for goal in goals)


def test_lane_shift_connector_goals_clip_to_target_and_bounds():
    goals = lane_shift_connector_goals(
        (0.0, -46.0, 0.0),
        -50.0,
        candidate_distances_m=(6.0, 4.0, 2.0),
        allowed_bounds_xyxy_m=(-100.0, -50.0, 100.0, 50.0),
        boundary_margin_m=1.5,
    )

    assert [(goal.world_x_m, goal.world_y_m) for goal in goals] == [
        (0.0, -48.0),
    ]
    assert goals[0].yaw_rad == pytest.approx(-math.pi / 2.0)


def test_frontier_preference_ranks_target_cluster_ahead_of_current_heading():
    geometry = GridGeometry(9, 9, 1.0, -4.5, -4.5)
    data = [-1] * (geometry.width * geometry.height)
    data[4 * geometry.width + 1] = 0
    data[4 * geometry.width + 7] = 0
    goals = rank_frontiers(
        data,
        geometry,
        (0.0, 0.0),
        robot_yaw_rad=math.pi,
        minimum_cells=1,
        minimum_goal_distance_m=0.5,
        preferred_world_xy=(10.0, 0.0),
    )
    assert goals[0].grid_x == 7
    assert goals[0].preference_distance_m < goals[1].preference_distance_m


def test_sweep_anchor_advances_only_after_live_map_observes_its_neighborhood():
    geometry = GridGeometry(9, 9, 1.0, -4.5, -4.5)
    data = [-1] * (geometry.width * geometry.height)
    assert not world_disk_has_known_cell(
        data, geometry, (2.0, 0.0), radius_m=1.0
    )
    data[4 * geometry.width + 6] = 0
    assert world_disk_has_known_cell(
        data, geometry, (2.0, 0.0), radius_m=1.0
    )


def test_vertical_sweep_anchor_uses_live_envelope_without_fixed_corner():
    envelope = (-102.0, -18.0, 98.0, 24.9)
    assert not vertical_sweep_anchor_reached(
        envelope, previous_y_m=-10.0, target_y_m=30.0, radius_m=5.0
    )
    assert vertical_sweep_anchor_reached(
        (-102.0, -18.0, 98.0, 25.0),
        previous_y_m=-10.0,
        target_y_m=30.0,
        radius_m=5.0,
    )
    assert vertical_sweep_anchor_reached(
        (-102.0, -25.0, 98.0, 40.0),
        previous_y_m=10.0,
        target_y_m=-30.0,
        radius_m=5.0,
    )


def test_failed_frontier_exclusions_expire_in_long_missions():
    records = [(1.0, 2.0, 9.0), (3.0, 4.0, 11.0)]
    active, points = prune_timed_exclusions(records, now_monotonic=10.0)
    assert active == [(3.0, 4.0, 11.0)]
    assert points == [(3.0, 4.0)]


def test_adaptive_frontier_stride_grows_after_success_and_contracts_on_failure():
    distance, streak = 2.0, 0
    for _ in range(3):
        distance, streak = next_adaptive_goal_distance(
            distance,
            streak,
            succeeded=True,
            minimum_distance_m=2.0,
            maximum_distance_m=6.0,
            successes_per_growth=3,
            growth_step_m=1.0,
        )
    assert (distance, streak) == (3.0, 0)
    distance, streak = next_adaptive_goal_distance(
        5.0,
        2,
        succeeded=False,
        minimum_distance_m=2.0,
        maximum_distance_m=6.0,
        successes_per_growth=3,
        growth_step_m=1.0,
    )
    assert (distance, streak) == (2.5, 0)


def test_successful_motion_stages_before_raw_frontier_exclusion():
    streak = 0
    actions = []
    for index in range(1, 13):
        streak, action, gain = next_no_progress_frontier_state(
            streak,
            map_area_before_m2=100.0,
            map_area_after_m2=100.4,
            minimum_gain_m2=2.0,
            successes_before_staging=3,
            successes_before_raw_exclusion=12,
        )
        assert gain == pytest.approx(0.4)
        if action is not None:
            actions.append((index, action))
    assert actions == [
        (3, "staging"),
        (6, "staging"),
        (9, "staging"),
        (12, "raw_and_staging"),
    ]
    assert streak == 0


def test_real_map_gain_resets_no_progress_streak():
    streak, action, gain = next_no_progress_frontier_state(
        2,
        map_area_before_m2=100.0,
        map_area_after_m2=102.0,
        minimum_gain_m2=2.0,
        successes_before_staging=3,
        successes_before_raw_exclusion=12,
    )
    assert streak == 0
    assert action is None
    assert gain == pytest.approx(2.0)


def test_no_progress_policy_rejects_invalid_contract_values():
    with pytest.raises(ValueError, match="positive"):
        next_no_progress_frontier_state(
            0,
            map_area_before_m2=0.0,
            map_area_after_m2=0.0,
            minimum_gain_m2=2.0,
            successes_before_staging=0,
            successes_before_raw_exclusion=12,
        )


def test_sweep_staging_goals_advance_toward_anchor_with_bounded_fallbacks():
    goals = sweep_staging_goals(
        (0.0, 0.0, math.pi / 2.0),
        (100.0, 10.0),
        candidate_distances_m=(8.0, 6.0, 4.0),
        allowed_bounds_xyxy_m=(-100.0, -50.0, 100.0, 50.0),
        boundary_margin_m=1.5,
    )
    assert [goal.distance_m for goal in goals] == [8.0, 6.0, 4.0]
    assert all(goal.grid_x == -1 and goal.grid_y == -1 for goal in goals)
    assert all(goal.world_x_m > 0.0 and goal.world_y_m > 0.0 for goal in goals)
    assert all(goal.yaw_rad == pytest.approx(math.atan2(10.0, 100.0)) for goal in goals)


def test_sweep_staging_goals_respect_boundary_margin():
    goals = sweep_staging_goals(
        (97.0, 10.0, 0.0),
        (100.0, 10.0),
        candidate_distances_m=(8.0, 2.0, 1.0),
        allowed_bounds_xyxy_m=(-100.0, -50.0, 100.0, 50.0),
        boundary_margin_m=1.5,
    )
    assert [goal.world_x_m for goal in goals] == pytest.approx([98.0])


def _grid(width=12, height=10):
    data = [-1] * (width * height)
    for y in range(2, 8):
        for x in range(2, 10):
            data[y * width + x] = 0
    return data


def test_frontier_clusters_trace_known_unknown_boundary():
    geometry = GridGeometry(12, 10, 0.5, -3.0, -2.5)
    clusters = frontier_clusters(_grid(), geometry, minimum_cells=4)
    assert len(clusters) == 1
    assert len(clusters[0]) == 24
    assert all(x in (2, 9) or y in (2, 7) for x, y in clusters[0])


def test_rank_frontiers_is_deterministic_and_respects_exclusion():
    geometry = GridGeometry(12, 10, 0.5, -3.0, -2.5)
    goals = rank_frontiers(_grid(), geometry, (0.0, 0.0), minimum_cells=4)
    assert len(goals) == 1
    excluded = [(goals[0].world_x_m, goals[0].world_y_m)]
    alternatives = rank_frontiers(
        _grid(), geometry, (0.0, 0.0), excluded_world_xy=excluded,
        minimum_cells=4,
    )
    assert alternatives
    assert all(
        math.hypot(
            goal.world_x_m - excluded[0][0],
            goal.world_y_m - excluded[0][1],
        ) > 2.0
        for goal in alternatives
    )


def test_grid_to_world_applies_origin_rotation():
    geometry = GridGeometry(2, 2, 1.0, 10.0, 20.0, math.pi / 2.0)
    x, y = grid_to_world(0, 0, geometry)
    assert x == pytest.approx(9.5)
    assert y == pytest.approx(20.5)


def test_map_extent_reports_required_20000m2_coverage_without_hiding_known_area():
    geometry = GridGeometry(200, 100, 1.0, -100.0, -50.0)
    data = [0] * (geometry.width * geometry.height)
    metrics = map_extent_metrics(
        data, geometry,
        required_bounds_xyxy_m=(-100.0, -50.0, 100.0, 50.0),
    )
    assert metrics["known_area_m2"] == 20_000.0
    assert metrics["mapped_envelope_area_m2"] == 20_000.0
    assert metrics["required_bounds_envelope_coverage_ratio"] == 1.0
    assert metrics["required_bounds_known_area_m2"] == 20_000.0
    assert metrics["required_bounds_known_coverage_ratio"] == 1.0


def test_sparse_corner_cells_do_not_turn_envelope_into_mapping_pass():
    geometry = GridGeometry(200, 100, 1.0, -100.0, -50.0)
    data = [-1] * (geometry.width * geometry.height)
    for x, y in ((0, 0), (199, 0), (0, 99), (199, 99)):
        data[y * geometry.width + x] = 0
    metrics = map_extent_metrics(
        data, geometry,
        required_bounds_xyxy_m=(-100.0, -50.0, 100.0, 50.0),
    )
    assert metrics["required_bounds_envelope_coverage_ratio"] == 1.0
    assert metrics["required_bounds_known_area_m2"] == 4.0
    assert metrics["required_bounds_known_coverage_ratio"] == 0.0002
    assert mapping_completion_reached(metrics) is False


def test_mapping_completion_allows_occlusion_pockets_after_continuous_area_pass():
    metrics = {
        "required_bounds_envelope_coverage_ratio": 1.0,
        "required_bounds_area_m2": 800.0,
        "known_area_m2": 1402.0,
        "required_bounds_known_coverage_ratio": 0.969,
    }
    assert mapping_completion_reached(metrics) is True


def test_map_extent_fails_closed_for_empty_map():
    geometry = GridGeometry(200, 100, 1.0, -100.0, -50.0)
    metrics = map_extent_metrics([-1] * 20_000, geometry)
    assert metrics["known_area_m2"] == 0.0
    assert metrics["mapped_envelope_area_m2"] == 0.0
    assert metrics["mapped_envelope_bounds_xyxy_m"] is None


def test_frontier_goals_respect_operating_bounds_and_vehicle_margin():
    geometry = GridGeometry(12, 10, 0.5, -3.0, -2.5)
    data = [-1] * (geometry.width * geometry.height)
    for y in range(1, 9):
        for x in range(1, 11):
            data[y * geometry.width + x] = 0

    goals = rank_frontiers(
        data,
        geometry,
        (0.0, 0.0),
        minimum_cells=1,
        allowed_bounds_xyxy_m=(-2.5, -2.0, 2.5, 2.0),
        boundary_margin_m=0.25,
    )

    assert goals
    assert all(-2.25 <= goal.world_x_m <= 2.25 for goal in goals)
    assert all(-1.75 <= goal.world_y_m <= 1.75 for goal in goals)
    assert rank_frontiers(
        data,
        geometry,
        (0.0, 0.0),
        minimum_cells=1,
        allowed_bounds_xyxy_m=(-1.0, -1.0, 1.0, 1.0),
        boundary_margin_m=0.25,
    ) == []


def test_frontier_connection_radius_bridges_lidar_sampling_gaps():
    geometry = GridGeometry(9, 9, 0.1, 0.0, 0.0)
    data = [-1] * (geometry.width * geometry.height)
    for x in (1, 3, 5, 7):
        data[4 * geometry.width + x] = 0
    assert frontier_clusters(
        data, geometry, minimum_cells=4, connection_radius_cells=1
    ) == []
    clusters = frontier_clusters(
        data, geometry, minimum_cells=4, connection_radius_cells=2
    )
    assert len(clusters) == 1
    assert len(clusters[0]) == 4


def test_sparse_ray_frontier_does_not_select_robot_center_as_goal():
    geometry = GridGeometry(21, 21, 1.0, -10.5, -10.5)
    data = [-1] * (geometry.width * geometry.height)
    for offset in range(-8, 9):
        data[10 * geometry.width + 10 + offset] = 0
        data[(10 + offset) * geometry.width + 10] = 0
    goals = rank_frontiers(
        data,
        geometry,
        (0.0, 0.0),
        minimum_cells=5,
        minimum_goal_distance_m=1.5,
    )
    assert goals
    assert goals[0].distance_m >= 0.65 * 8.0


def test_frontier_goal_backs_off_into_known_space_and_faces_outward():
    geometry = GridGeometry(25, 3, 1.0, -0.5, -1.5)
    data = [-1] * (geometry.width * geometry.height)
    for x in range(1, 21):
        data[geometry.width + x] = 0
    raw = rank_frontiers(
        data, geometry, (0.0, 0.0), minimum_cells=1,
        minimum_goal_distance_m=1.5,
    )[0]
    backed_off = rank_frontiers(
        data, geometry, (0.0, 0.0), minimum_cells=1,
        minimum_goal_distance_m=1.5, goal_backoff_m=1.5,
    )[0]
    assert backed_off.distance_m == pytest.approx(raw.distance_m - 1.5)
    assert backed_off.yaw_rad == pytest.approx(0.0)


def test_frontier_goal_distance_is_clamped_to_connected_sensor_neighborhood():
    geometry = GridGeometry(30, 3, 1.0, -0.5, -1.5)
    data = [-1] * (geometry.width * geometry.height)
    for x in range(1, 26):
        data[geometry.width + x] = 0
    goal = rank_frontiers(
        data, geometry, (0.0, 0.0), minimum_cells=1,
        minimum_goal_distance_m=1.5, maximum_goal_distance_m=4.0,
    )[0]
    assert goal.distance_m == pytest.approx(4.0)


def test_backoff_at_minimum_distance_survives_floating_point_roundoff():
    geometry = GridGeometry(7, 3, 1.0, -0.5, -1.5)
    data = [-1] * (geometry.width * geometry.height)
    for x in range(1, 4):
        data[geometry.width + x] = 0
    goals = rank_frontiers(
        data, geometry, (0.0, 0.0), minimum_cells=1,
        minimum_goal_distance_m=1.5, goal_backoff_m=10.0,
    )
    assert goals
    assert goals[0].distance_m == pytest.approx(1.5)


def test_frontier_goal_prefers_current_ackermann_heading():
    geometry = GridGeometry(9, 9, 1.0, -4.5, -4.5)
    data = [-1] * (geometry.width * geometry.height)
    for x, y in ((1, 4), (7, 4), (4, 1), (4, 7)):
        data[y * geometry.width + x] = 0
    east = rank_frontiers(
        data, geometry, (0.0, 0.0), robot_yaw_rad=0.0,
        minimum_cells=1, connection_radius_cells=8,
    )[0]
    north = rank_frontiers(
        data, geometry, (0.0, 0.0), robot_yaw_rad=math.pi / 2.0,
        minimum_cells=1, connection_radius_cells=8,
    )[0]
    assert east.world_x_m > 0.0
    assert abs(east.world_y_m) < 1.0e-9
    assert north.world_y_m > 0.0
    assert abs(north.world_x_m) < 1.0e-9


def test_frontier_goal_yaw_change_is_ackermann_bounded():
    geometry = GridGeometry(9, 9, 1.0, -4.5, -4.5)
    data = [-1] * (geometry.width * geometry.height)
    data[7 * geometry.width + 4] = 0
    goal = rank_frontiers(
        data, geometry, (0.0, 0.0), robot_yaw_rad=0.0,
        minimum_cells=1, maximum_goal_yaw_change_rad=0.35,
    )[0]
    assert goal.yaw_rad == pytest.approx(0.35)


def test_off_heading_frontier_becomes_forward_minimum_radius_arc_goal():
    geometry = GridGeometry(21, 21, 1.0, -10.5, -10.5)
    data = [-1] * (geometry.width * geometry.height)
    for y in range(10, 19):
        data[y * geometry.width + 10] = 0
    goal = rank_frontiers(
        data,
        geometry,
        (0.0, 0.0),
        robot_yaw_rad=0.0,
        minimum_cells=1,
        minimum_goal_distance_m=0.40,
        maximum_goal_distance_m=4.0,
        maximum_goal_yaw_change_rad=0.35,
        minimum_turning_radius_m=1.429,
    )[0]
    expected_x = 1.429 * math.sin(0.35)
    expected_y = 1.429 * (1.0 - math.cos(0.35))
    assert goal.world_x_m == pytest.approx(expected_x)
    assert goal.world_y_m == pytest.approx(expected_y)
    assert goal.yaw_rad == pytest.approx(0.35)
    assert goal.distance_m == pytest.approx(
        2.0 * 1.429 * math.sin(0.35 / 2.0)
    )


def test_frontier_exclusion_centers_preserve_raw_frontier_behind_local_arc():
    geometry = GridGeometry(21, 21, 1.0, -10.5, -10.5)
    data = [-1] * (geometry.width * geometry.height)
    for y in range(10, 19):
        data[y * geometry.width + 10] = 0
    goal = rank_frontiers(
        data,
        geometry,
        (0.0, 0.0),
        robot_yaw_rad=0.0,
        minimum_cells=1,
        minimum_goal_distance_m=0.40,
        maximum_goal_distance_m=4.0,
        maximum_goal_yaw_change_rad=0.35,
        minimum_turning_radius_m=1.429,
    )[0]

    endpoint, raw_frontier = frontier_goal_exclusion_centers(goal, geometry)
    assert endpoint == pytest.approx((goal.world_x_m, goal.world_y_m))
    assert raw_frontier == pytest.approx(
        grid_to_world(goal.grid_x, goal.grid_y, geometry)
    )
    assert math.dist(endpoint, raw_frontier) > 1.0


def test_frontier_exclusion_uses_dispatch_world_coordinate_after_map_expands():
    geometry = GridGeometry(21, 21, 1.0, -10.5, -10.5)
    data = [-1] * (geometry.width * geometry.height)
    for y in range(10, 19):
        data[y * geometry.width + 10] = 0
    goal = rank_frontiers(
        data,
        geometry,
        (0.0, 0.0),
        robot_yaw_rad=0.0,
        minimum_cells=1,
        minimum_goal_distance_m=0.40,
        maximum_goal_distance_m=4.0,
        maximum_goal_yaw_change_rad=0.35,
        minimum_turning_radius_m=1.429,
    )[0]
    expanded_geometry = GridGeometry(41, 41, 1.0, -20.5, -20.5)

    _, raw_frontier = frontier_goal_exclusion_centers(
        goal, expanded_geometry
    )

    assert raw_frontier == pytest.approx(
        (goal.raw_world_x_m, goal.raw_world_y_m)
    )
    assert raw_frontier != pytest.approx(
        grid_to_world(goal.grid_x, goal.grid_y, expanded_geometry)
    )


def test_raw_frontier_exclusion_prevents_reselection_with_new_local_arc():
    geometry = GridGeometry(9, 9, 1.0, -4.5, -4.5)
    data = [-1] * (geometry.width * geometry.height)
    data[4 * geometry.width + 7] = 0
    data[7 * geometry.width + 4] = 0
    kwargs = {
        "robot_yaw_rad": math.pi,
        "minimum_cells": 1,
        "minimum_goal_distance_m": 0.40,
        "maximum_goal_distance_m": 4.0,
        "maximum_goal_yaw_change_rad": 0.35,
        "minimum_turning_radius_m": 1.429,
        "preferred_world_xy": (10.0, 10.0),
    }
    selected = rank_frontiers(data, geometry, (0.0, 0.0), **kwargs)[0]
    raw_frontier = grid_to_world(
        selected.grid_x, selected.grid_y, geometry
    )

    alternatives = rank_frontiers(
        data,
        geometry,
        (0.0, 0.0),
        excluded_world_xy=[raw_frontier],
        exclusion_radius_m=0.25,
        **kwargs,
    )
    assert alternatives
    assert (alternatives[0].grid_x, alternatives[0].grid_y) != (
        selected.grid_x,
        selected.grid_y,
    )


def test_material_sub_limit_heading_change_still_becomes_ackermann_arc():
    geometry = GridGeometry(21, 21, 1.0, -10.5, -10.5)
    data = [-1] * (geometry.width * geometry.height)
    for x in range(11, 19):
        data[14 * geometry.width + x] = 0
    goal = rank_frontiers(
        data,
        geometry,
        (0.0, 0.0),
        robot_yaw_rad=0.0,
        minimum_cells=1,
        minimum_goal_distance_m=0.40,
        maximum_goal_distance_m=4.0,
        maximum_goal_yaw_change_rad=0.70,
        minimum_goal_arc_yaw_change_rad=0.15,
        minimum_turning_radius_m=1.429,
    )[0]
    assert 0.15 < goal.yaw_rad < 0.70
    expected_x = 1.429 * math.sin(goal.yaw_rad)
    expected_y = 1.429 * (1.0 - math.cos(goal.yaw_rad))
    assert goal.world_x_m == pytest.approx(expected_x)
    assert goal.world_y_m == pytest.approx(expected_y)


def test_reverse_escape_goal_is_straight_bounded_and_heading_preserving():
    goal = reverse_escape_goal(
        (4.0, -2.0),
        math.pi / 2.0,
        distance_m=1.0,
        allowed_bounds_xyxy_m=(-10.0, -10.0, 10.0, 10.0),
        boundary_margin_m=1.5,
    )
    assert goal is not None
    assert goal.world_x_m == pytest.approx(4.0)
    assert goal.world_y_m == pytest.approx(-3.0)
    assert goal.yaw_rad == pytest.approx(math.pi / 2.0)
    assert goal.distance_m == pytest.approx(1.0)
    assert goal.frontier_cell_count == 0
    assert reverse_escape_goal(
        (-8.4, 0.0),
        0.0,
        distance_m=1.0,
        allowed_bounds_xyxy_m=(-10.0, -10.0, 10.0, 10.0),
        boundary_margin_m=1.5,
    ) is None


def test_boundary_turn_buffer_starts_ackermann_turn_before_goal_envelope_edge():
    geometry = GridGeometry(21, 21, 1.0, -10.5, -10.5)
    data = [-1] * (geometry.width * geometry.height)
    for y in range(2, 19):
        for x in range(2, 19):
            data[y * geometry.width + x] = 0
    goal = rank_frontiers(
        data,
        geometry,
        (6.5, 0.0),
        robot_yaw_rad=0.0,
        minimum_cells=1,
        minimum_goal_distance_m=0.40,
        maximum_goal_distance_m=2.0,
        maximum_goal_yaw_change_rad=0.35,
        minimum_turning_radius_m=1.429,
        allowed_bounds_xyxy_m=(-10.0, -10.0, 10.0, 10.0),
        boundary_margin_m=1.5,
        boundary_turn_buffer_m=1.429,
    )[0]
    assert abs(goal.yaw_rad) == pytest.approx(0.35)
    assert goal.world_x_m < 8.5
    assert math.copysign(1.0, goal.world_y_m) == math.copysign(1.0, goal.yaw_rad)


def test_world_disk_traversability_rejects_unknown_and_inflated_costs():
    geometry = GridGeometry(9, 9, 0.5, -2.25, -2.25)
    data = [0] * (geometry.width * geometry.height)
    assert world_disk_is_traversable(
        data, geometry, (0.0, 0.0), radius_m=0.7, maximum_cost=50
    )
    data[4 * geometry.width + 5] = 80
    assert not world_disk_is_traversable(
        data, geometry, (0.0, 0.0), radius_m=0.7, maximum_cost=50
    )
    data[4 * geometry.width + 5] = -1
    assert not world_disk_is_traversable(
        data, geometry, (0.0, 0.0), radius_m=0.7, maximum_cost=50
    )


def test_boundary_buffer_converts_shallow_direction_change_to_short_arc():
    geometry = GridGeometry(41, 41, 1.0, -20.5, -20.5)
    data = [-1] * (geometry.width * geometry.height)
    for y in range(3, 38):
        for x in range(3, 38):
            data[y * geometry.width + x] = 0
    robot = (-18.0, 3.0)
    goal = rank_frontiers(
        data,
        geometry,
        robot,
        robot_yaw_rad=-1.92,
        minimum_cells=1,
        minimum_goal_distance_m=0.80,
        maximum_goal_distance_m=2.0,
        maximum_goal_yaw_change_rad=0.70,
        minimum_turning_radius_m=1.429,
        allowed_bounds_xyxy_m=(-20.0, -10.0, 20.0, 10.0),
        boundary_margin_m=1.5,
        boundary_turn_buffer_m=1.429,
    )[0]
    assert goal.distance_m < 1.0
    assert goal.world_x_m >= -18.5
    assert goal.world_y_m < robot[1]
