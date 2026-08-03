from sanitation_coverage.swath_optimizer import optimize_swath_angle


def test_long_axis_is_selected_for_rectangular_demo_field():
    polygon = [(-3, -2), (3, -2), (3, 2), (-3, 2)]
    best, candidates = optimize_swath_angle(polygon, 0.5)
    assert best.angle_deg == 0.0
    assert len(candidates) == 36
    assert best.swath_count == 8


def test_rotated_long_axis_is_discovered_with_five_degree_search():
    # A 6 x 2 rectangle whose long axis is at 30 degrees.
    import math
    base = [(-3, -1), (3, -1), (3, 1), (-3, 1)]
    angle = math.radians(30)
    polygon = [(x * math.cos(angle) - y * math.sin(angle), x * math.sin(angle) + y * math.cos(angle)) for x, y in base]
    best, _ = optimize_swath_angle(polygon, 0.5)
    assert best.angle_deg == 30.0
