import math

import pytest

from sanitation_coverage.metrics import raster_coverage_metrics
from sanitation_coverage.mission_geometry import (
    compile_mission_geometry,
    exclusion_clearance,
    target_grid_viable,
)
from sanitation_coverage.route_entry import (
    entry_points,
    oriented_pose,
    reverse_components,
    route_candidates,
    segment_heading,
    transit_pose,
)


@pytest.mark.parametrize(
    ('start', 'end', 'expected'),
    [
        ((0, 0), (1, 0), 0.0),
        ((1, 0), (0, 0), math.pi),
        ((0, 0), (0, 1), math.pi / 2),
        ((0, 1), (0, 0), -math.pi / 2),
    ],
)
def test_explicit_cardinal_heading(start, end, expected):
    assert math.isclose(segment_heading(start, end), expected)
    assert oriented_pose(start, expected)['yaw'] == expected


def test_single_point_cannot_silently_become_zero_yaw():
    with pytest.raises(ValueError):
        segment_heading((1, 2), (1, 2))


def test_transit_yaw_follows_approach_not_implicit_zero():
    pose = transit_pose((0, 0), {'x': -2.2, 'y': -3.1, 'yaw': 0.0})
    assert math.isclose(pose['yaw'], math.atan2(-3.1, -2.2))
    assert pose['yaw'] != 0.0


def test_dense_entry_finishes_on_first_swath_heading():
    component = {'points': [(-1.45, -3.125), (-1.35, -3.125)]}
    points = entry_points((-2.09, -2.97), component)
    assert len(points) > 10
    assert points[0] == (-2.09, -2.97)
    assert math.isclose(segment_heading(points[-2], points[-1]), 0.0)


def test_staging_pose_outside_operation_area_keeps_exclusion_clearance():
    exclusion = [[2, 1], [4, 1], [4, 3], [2, 3]]
    assert exclusion_clearance((-2.2, -3.1), [exclusion]) > 5.0
    assert exclusion_clearance((3.0, 2.0), [exclusion]) == 0.0


def test_route_target_rejects_lethal_or_unavailable_costmap_cells():
    safe = {
        'global_costmap_received': True,
        'global_costmap_cost': 0,
        'keepout_mask_received': True,
        'keepout_mask_value': 0,
    }
    assert target_grid_viable(safe)
    assert not target_grid_viable({**safe, 'global_costmap_cost': 99})
    assert not target_grid_viable({**safe, 'global_costmap_received': False})
    assert not target_grid_viable({**safe, 'keepout_mask_value': 100})


def test_reverse_route_reverses_component_and_point_order():
    components = [
        {'kind': 'swath', 'points': [(0, 0), (1, 0)]},
        {'kind': 'turn', 'points': [(1, 0), (1, 1)]},
        {'kind': 'swath', 'points': [(1, 1), (0, 1)]},
    ]
    reversed_route = reverse_components(components)
    assert [item['kind'] for item in reversed_route] == ['swath', 'turn', 'swath']
    assert reversed_route[0]['points'] == [(0, 1), (1, 1)]
    assert {item['direction'] for item in route_candidates(components, 0.8)} == {
        'forward', 'reverse'
    }


def test_cleanable_geometry_excludes_keepout_and_static_obstacle():
    config = {
        'outer_polygon': [[0, 0], [8, 0], [8, 8], [0, 8]],
        'exclusion_polygons': [[[2, 2], [4, 2], [4, 4], [2, 4]]],
        'keepout_polygons': [],
        'static_obstacles': [{'center': [3.8, 4.0], 'size': [0.4, 0.4]}],
        'robot_footprint': [[0.4, 0.36], [0.4, -0.36], [-0.4, -0.36]],
        'safety_margin_m': 0.1,
        'operation_width_m': 0.65,
        'headland': {'width_m': 1.0},
    }
    geometry = compile_mission_geometry(config)
    assert geometry['outer_area_m2'] == 64.0
    assert len(geometry['exclusion_polygons']) == 1
    assert geometry['headland_clearance_valid'] is True
    metrics = raster_coverage_metrics(
        geometry['cleanable_outer_polygon'], [], 0.65, 0.1,
        geometry['cleanable_exclusion_polygons'],
    )
    assert math.isclose(
        metrics['target_area_m2'], geometry['cleanable_area_m2'], abs_tol=0.50
    )


def test_world_obstacles_are_transformed_and_outside_items_are_audited():
    config = {
        'outer_polygon': [[-2, -4], [6, -4], [6, 4], [-2, 4]],
        'exclusion_polygons': [],
        'keepout_polygons': [],
        'static_obstacles': [
            {
                'name': 'outside', 'frame_id': 'world',
                'center': [2, 3], 'size': [.7, .7],
            },
            {
                'name': 'inside', 'frame_id': 'world',
                'center': [-4, -2], 'size': [.8, .65],
            },
        ],
        'world_to_map_translation': [8, 0],
        'robot_footprint': [[.4, .36], [.4, -.36], [-.4, -.36]],
        'safety_margin_m': .1,
        'operation_width_m': .65,
        'headland': {'width_m': 1.0},
    }
    geometry = compile_mission_geometry(config)
    assert geometry['compiled_static_obstacles'][0]['map_center'] == (4.0, -2.0)
    assert geometry['ignored_static_obstacles'][0]['map_center'] == (10.0, 3.0)
