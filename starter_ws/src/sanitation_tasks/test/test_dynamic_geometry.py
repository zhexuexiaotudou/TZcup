import math

from sanitation_tasks.dynamic_geometry import (
    crossing_targets,
    path_heading,
    remaining_path_length,
)


def test_remaining_path_and_heading_follow_current_swath():
    points = [(0, 0), (1, 0), (2, 0), (3, 0)]
    assert math.isclose(remaining_path_length((1.2, 0), points), 2.2)
    assert path_heading((1.2, 0), points) == 0.0


def test_crossing_targets_move_across_corridor():
    targets = crossing_targets((0, 0), 0.0, 1.5, 1.0, 5)
    assert targets[0] == (1.5, -1.0)
    assert targets[2] == (1.5, 0.0)
    assert targets[-1] == (1.5, 1.0)
