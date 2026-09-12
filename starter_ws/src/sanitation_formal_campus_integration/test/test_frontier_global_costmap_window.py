"""Exercise global-costmap admission without importing ROS on offline hosts."""

import ast
import math
from pathlib import Path
from types import SimpleNamespace as NS

import pytest


SOURCE = (
    Path(__file__).parents[1]
    / "sanitation_formal_campus_integration"
    / "frontier_explorer.py"
)


@pytest.fixture
def explorer():
    tree = ast.parse(SOURCE.read_text(encoding="utf-8"))
    tree.body = [node for node in tree.body if isinstance(node, ast.ClassDef)]
    namespace = {
        "Node": object,
        "OccupancyGrid": object,
        "Odometry": object,
        "math": math,
    }
    exec(compile(tree, str(SOURCE), "exec"), namespace)
    node = namespace["FormalFrontierExplorer"].__new__(
        namespace["FormalFrontierExplorer"]
    )
    node.get_parameter = lambda name: NS(
        value={"map_frame": "map", "planning_period_sec": 2.0}[name]
    )
    now_ns = 10_000_000_000
    node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=now_ns))
    node.statuses = []
    node._publish = lambda state, **values: node.statuses.append((state, values))
    node._global_costmap = None
    return node, now_ns


def _costmap(stamp_ns, *, frame="map", width=4, height=4, resolution=1.0,
             origin_x=0.0, origin_y=0.0, quaternion=(0.0, 0.0, 0.0, 1.0), data=None):
    if data is None:
        data = [0] * (width * height)
    sec, nanosec = divmod(stamp_ns, 1_000_000_000)
    x, y, z, w = quaternion
    return NS(
        header=NS(frame_id=frame, stamp=NS(sec=sec, nanosec=nanosec)),
        info=NS(
            width=width,
            height=height,
            resolution=resolution,
            origin=NS(
                position=NS(x=origin_x, y=origin_y),
                orientation=NS(x=x, y=y, z=z, w=w),
            ),
        ),
        data=data,
    )


def test_missing_global_costmap_waits(explorer):
    node, _ = explorer
    assert node._planning_window() is None
    assert node.statuses[-1][0] == "waiting_for_global_costmap"


@pytest.mark.parametrize("stamp_ns", (0, 10_000_000_001, 7_999_999_999))
def test_zero_future_or_stale_global_costmap_waits(explorer, stamp_ns):
    node, _ = explorer
    node._global_costmap = _costmap(stamp_ns)
    assert node._planning_window() is None
    assert node.statuses[-1][0] == "waiting_for_fresh_global_costmap"


def test_wrong_frame_global_costmap_waits(explorer):
    node, now_ns = explorer
    node._global_costmap = _costmap(now_ns, frame="odom")
    assert node._planning_window() is None
    assert node.statuses[-1][0] == "waiting_for_global_costmap_map_frame"


@pytest.mark.parametrize(
    "kwargs",
    (
        {"width": 0},
        {"resolution": 0.0},
        {"data": [0]},
        {"quaternion": (0.1, 0.0, 0.0, math.sqrt(0.99))},
    ),
)
def test_invalid_or_nonplanar_global_costmap_waits(explorer, kwargs):
    node, now_ns = explorer
    node._global_costmap = _costmap(now_ns, **kwargs)
    assert node._planning_window() is None
    assert node.statuses[-1][0] == "waiting_for_valid_global_costmap"


def test_fresh_global_costmap_returns_rotated_metadata_window(explorer):
    node, now_ns = explorer
    node._global_costmap = _costmap(
        now_ns - 1_000_000_000,
        width=600,
        height=600,
        resolution=0.05,
        origin_x=6.9,
        origin_y=-18.3,
    )
    assert node._planning_window() == pytest.approx((600, 600, 0.05, 6.9, -18.3, 0.0))
