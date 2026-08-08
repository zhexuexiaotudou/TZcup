import numpy as np

from sanitation_perception.map_projection_v2 import mask_regions_to_map


def test_all_predicted_regions_become_geometry_derived_polygons():
    mask = np.zeros((100, 120), dtype=np.uint8)
    mask[20:40, 20:40] = 1
    mask[60:85, 70:100] = 1
    probability = np.where(mask, 0.9, 0.05).astype(np.float32)
    depth = np.full(mask.shape, 2.0, dtype=np.float32)
    camera = {
        "fx": 100.0,
        "fy": 100.0,
        "cx": 60.0,
        "cy": 50.0,
        "pixel_sigma": 0.5,
        "depth_sigma_m": 0.01,
    }
    regions = mask_regions_to_map(mask, probability, depth, camera, np.eye(4))
    assert len(regions) == 2
    assert all(len(region.polygon_xy_m) >= 4 for region in regions)
    assert all(region.physical_area_m2 > 0.10 for region in regions)
    assert all(region.confidence > 0.89 for region in regions)
    assert {region.pixel_area for region in regions} == {400, 750}


def test_invalid_depth_region_is_dropped_fail_closed():
    mask = np.zeros((30, 30), dtype=np.uint8)
    mask[5:25, 5:25] = 1
    depth = np.full(mask.shape, np.nan, dtype=np.float32)
    regions = mask_regions_to_map(
        mask,
        mask.astype(np.float32),
        depth,
        {"fx": 100.0, "fy": 100.0, "cx": 15.0, "cy": 15.0},
        np.eye(4),
    )
    assert regions == []
