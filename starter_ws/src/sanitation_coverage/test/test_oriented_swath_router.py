from sanitation_coverage.oriented_swath_router import route_oriented_swaths


def test_router_sorts_adjacent_lanes_and_alternates_direction():
    shuffled = [((0, 2), (4, 2)), ((0, 0), (4, 0)), ((0, 1), (4, 1))]
    result = route_oriented_swaths(shuffled, (-1, 0))
    centers = [(a[1] + b[1]) / 2 for a, b in result.swaths]
    assert centers in ([0, 1, 2], [2, 1, 0])
    assert result.swaths[0][1][0] != result.swaths[1][1][0]
    assert result.connector_distance_m == 2.0
