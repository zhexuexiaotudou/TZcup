from sanitation_coverage.oriented_swath_router import route_oriented_swaths


def test_router_sorts_adjacent_lanes_and_alternates_direction():
    # Upstream endpoint order is deliberately mixed.
    shuffled = [((4, 2), (0, 2)), ((0, 0), (4, 0)), ((4, 1), (0, 1))]
    result = route_oriented_swaths(shuffled, (-1, 0))
    centers = [(a[1] + b[1]) / 2 for a, b in result.swaths]
    assert centers in ([0, 1, 2], [2, 1, 0])
    assert result.swaths[0][1][0] != result.swaths[1][1][0]
    assert result.connector_distance_m == 2.0


def test_no_connector_crosses_the_field_when_upstream_is_already_alternating():
    swaths = [((0, 0), (4, 0)), ((4, 0.5), (0, 0.5)), ((0, 1), (4, 1))]
    result = route_oriented_swaths(swaths, (-1, 0))
    connector_lengths = [
        __import__("math").dist(left[1], right[0])
        for left, right in zip(result.swaths, result.swaths[1:])
    ]
    assert max(connector_lengths) == 0.5


def test_ackermann_skip_lane_order_avoids_repeated_sub_diameter_turns():
    swaths = [((0, y), (100, y)) for y in range(12)]
    result = route_oriented_swaths(swaths, (0, 0), lane_skip=3)
    centers = [round((a[1] + b[1]) / 2) for a, b in result.swaths]
    differences = [abs(b - a) for a, b in zip(centers, centers[1:])]
    assert sorted(centers) == list(range(12))
    assert differences.count(1) == 2
    assert all(value in (1, 3) for value in differences)
    assert "lane_skip_3" in result.ordering
