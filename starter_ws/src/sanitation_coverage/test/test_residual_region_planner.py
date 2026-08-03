from sanitation_coverage.residual_region_planner import connected_residual_regions, plan_residual_regions


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
