from sanitation_coverage.route_entry import apply_lateral_affine


def test_map_normal_affine_does_not_distort_swath_heading_or_length():
    swaths = [((0, -1), (4, -1)), ((4, -2), (0, -2))]
    calibrated = apply_lateral_affine(swaths, 0.0, scale=1.02, offset_m=0.02)
    assert calibrated[0] == ((0.0, -1.0), (4.0, -1.0))
    assert calibrated[1] == ((4.0, -2.02), (0.0, -2.02))
