from dataclasses import dataclass, field

from sanitation_perception.grid_safety import (
    costmap_clear,
    footprint_costmap_clear,
    keepout_clear,
    sample_occupancy_grid,
)


@dataclass
class Vector:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass
class Quaternion:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    w: float = 1.0


@dataclass
class Pose:
    position: Vector = field(default_factory=Vector)
    orientation: Quaternion = field(default_factory=Quaternion)


@dataclass
class Info:
    resolution: float = 1.0
    width: int = 3
    height: int = 2
    origin: Pose = field(default_factory=Pose)


@dataclass
class Grid:
    info: Info = field(default_factory=Info)
    data: list[int] = field(default_factory=lambda: [0, 100, -1, 0, 0, 0])


def test_sampling_and_safety_are_fail_closed() -> None:
    grid = Grid()
    assert sample_occupancy_grid(grid, 0.5, 0.5) == 0
    assert sample_occupancy_grid(grid, 1.5, 0.5) == 100
    assert sample_occupancy_grid(grid, 2.5, 0.5) == -1
    assert sample_occupancy_grid(grid, 4.0, 0.5) is None
    assert keepout_clear(grid, 0.5, 0.5)
    assert not keepout_clear(grid, 1.5, 0.5)
    assert not keepout_clear(grid, 2.5, 0.5)
    assert not keepout_clear(None, 0.5, 0.5)
    assert costmap_clear(grid, 0.5, 0.5)
    assert not costmap_clear(grid, 1.5, 0.5)
    assert not costmap_clear(grid, 2.5, 0.5)


def test_sampling_respects_rotated_grid_origin() -> None:
    grid = Grid()
    grid.info.origin.position.x = 10.0
    grid.info.origin.position.y = 20.0
    grid.info.origin.orientation.z = 2**-0.5
    grid.info.origin.orientation.w = 2**-0.5
    assert sample_occupancy_grid(grid, 9.5, 20.5) == 0


def test_footprint_rejects_lethal_and_out_of_bounds_cells() -> None:
    grid = Grid(info=Info(width=6, height=6), data=[0] * 36)
    footprint = ((-0.4, -0.4), (0.4, -0.4), (0.4, 0.4), (-0.4, 0.4))
    assert footprint_costmap_clear(grid, 2.5, 2.5, 0.0, footprint)
    grid.data[2 * 6 + 2] = 100
    assert not footprint_costmap_clear(grid, 2.5, 2.5, 0.0, footprint)
    assert not footprint_costmap_clear(grid, 0.1, 0.1, 0.0, footprint)
