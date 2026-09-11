"""Exercise production manager methods without importing ROS on offline hosts."""
import ast
import math
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

from sanitation_formal_campus_integration.map_lifecycle_core import (
    MapLifecycleError, assess_grid_observation,
)


@pytest.fixture
def manager(tmp_path):
    source = Path(__file__).parents[1] / "sanitation_formal_campus_integration/map_lifecycle_manager.py"
    tree = ast.parse(source.read_text(encoding="utf-8"))
    # Execute unchanged production class methods. Only ROS transport and time are stubs.
    tree.body = [node for node in tree.body if isinstance(node, (ast.ClassDef, ast.FunctionDef))]
    clock = NS(value=10.0)
    namespace = dict(Node=object, OccupancyGrid=object, Odometry=object, math=math,
                     time=NS(monotonic=lambda: clock.value),
                     MapLifecycleError=MapLifecycleError, assess_grid_observation=assess_grid_observation,
                     SaveMap=NS(Request=lambda: NS(name=NS(data=""))))
    exec(compile(tree, str(source), "exec"), namespace)
    cls = namespace["FormalMapLifecycleManager"]
    node = cls.__new__(cls)
    params = dict(quality_period_sec=5.0, map_max_age_sec=15.0,
                  odometry_max_age_sec=5.0, gnss_odometry_consistency_tolerance_m=2.0,
                  observation_threshold=.95, stable_samples_required=3)
    node.get_parameter = lambda name: NS(value=params[name])
    node._finished = node._saving = False
    node._latest_map = None
    node._map_received_at = node._odom_received_at = node._gps_received_at = None
    node._map_stamp_ns = node._consumed_map_stamp_ns = None
    node._odom_stamp_ns = node._gps_stamp_ns = None
    node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=int(clock.value*1e9)))
    node._stable = 0
    node._last_quality_monotonic = 0.0
    node._start_ok = node._start_checked = True
    node._contract = NS(geofence=((0,0),(1,0),(1,1),(0,1)))
    node._root = tmp_path
    node.statuses = []
    node._publish = lambda *args: node.statuses.append(args)
    node.saves = []
    node._save_client = NS(service_is_ready=lambda: True,
        call_async=lambda request: (node.saves.append(request) or NS(add_done_callback=lambda callback: None)))
    return node, clock


def grid(stamp, **changes):
    msg = NS(header=NS(frame_id="map", stamp=NS(sec=stamp,nanosec=0)),
             info=NS(width=10,height=10,resolution=.1,
                origin=NS(position=NS(x=0.,y=0.,z=0.),orientation=NS(x=0.,y=0.,z=0.,w=1.))),
             data=[0]*100)
    for key, value in changes.items():
        setattr(msg, key, value)
    return msg


def odometry(stamp):
    return NS(header=NS(frame_id="odom", stamp=NS(sec=stamp,nanosec=0)),
              pose=NS(pose=NS(position=NS(x=0.,y=0.))))


def fresh_inputs(node):
    stamp = node.get_clock().now().nanoseconds // 1_000_000_000
    node._on_odom(odometry(stamp))
    node._on_gps_odom(odometry(stamp))


def test_one_map_cannot_count_three_timer_ticks(manager):
    node, clock = manager
    fresh_inputs(node)
    node._on_map(grid(10))
    for now in (11.,15.,20.):
        clock.value = now
        fresh_inputs(node)
        node._evaluate()
    assert node._stable == 1
    assert not node.saves


def test_three_fresh_distinct_maps_save(manager):
    node, clock = manager
    for index in range(3):
        clock.value = 10. + index*5
        fresh_inputs(node)
        node._on_map(grid(10+index*5))
        node._evaluate()
    assert node._stable == 3
    assert len(node.saves) == 1


@pytest.mark.parametrize("stamp", [10, 0])
def test_duplicate_or_reordered_map_resets_window(manager, stamp):
    node, clock = manager
    fresh_inputs(node)
    node._on_map(grid(10)); node._evaluate()
    clock.value = 15.
    fresh_inputs(node)
    node._on_map(grid(stamp)); node._evaluate()
    assert node._stable == 0
    assert not node.saves


@pytest.mark.parametrize("bad", ["frame", "nan", "zero_quaternion", "tilted", "payload", "resolution"])
def test_invalid_map_resets_window(manager, bad):
    node, _ = manager
    fresh_inputs(node)
    node._on_map(grid(10)); node._evaluate()
    msg = grid(15)
    if bad == "frame": msg.header.frame_id = "odom"
    if bad == "nan": msg.info.origin.position.x = math.nan
    if bad == "zero_quaternion": msg.info.origin.orientation.w = 0.
    if bad == "tilted": msg.info.origin.orientation.x = 1.
    if bad == "payload": msg.data[0] = 255
    if bad == "resolution": msg.info.resolution = .5
    node._on_map(msg)
    assert node._stable == 0
    assert node._latest_map is None


@pytest.mark.parametrize("stream", ["map", "odom", "gps"])
def test_stale_input_resets_window(manager, stream):
    node, clock = manager
    fresh_inputs(node)
    node._on_map(grid(10)); node._evaluate()
    clock.value = 30.
    fresh_inputs(node)
    node._on_map(grid(30))
    setattr(node, {"map":"_map_received_at", "odom":"_odom_received_at", "gps":"_gps_received_at"}[stream], 10.)
    node._evaluate()
    assert node._stable == 0
    assert not node.saves


def test_below_95_percent_breaks_consecutive_window(manager):
    node, clock = manager
    fresh_inputs(node)
    node._on_map(grid(10)); node._evaluate()
    clock.value = 15.
    fresh_inputs(node)
    msg = grid(15); msg.data[:6] = [-1]*6
    node._on_map(msg); node._evaluate()
    assert node._stable == 0
    assert not node.saves


@pytest.mark.parametrize("name,value", [("observation_threshold", .94), ("stable_samples_required", 2)])
def test_runtime_parameter_changes_cannot_lower_gate(manager, name, value):
    node, _ = manager
    original = node.get_parameter
    node.get_parameter = lambda key: NS(value=value) if key == name else original(key)
    fresh_inputs(node)
    node._on_map(grid(10)); node._evaluate()
    assert node._stable == 0
    assert not node.saves


def test_save_response_after_inputs_expire_never_marks_ready(manager):
    node, clock = manager
    for index in range(3):
        clock.value = 10. + index*5
        fresh_inputs(node)
        node._on_map(grid(10+index*5)); node._evaluate()
    assert len(node.saves) == 1
    clock.value = 40.
    node._on_save(NS(result=lambda: NS(result=0)), {})
    assert not node._finished
    assert node._stable == 0
    assert node.statuses[-1][0] == "map_save_or_integrity_gate_failed"


@pytest.mark.parametrize("stream", ["map", "odom", "gps"])
@pytest.mark.parametrize("stamp", [0, 1, 31])
def test_source_stamp_rejected_despite_fresh_arrival(manager, stream, stamp):
    node, clock = manager
    clock.value = 30.
    fresh_inputs(node)
    node._on_map(grid(30))
    callback = {"map":node._on_map, "odom":node._on_odom, "gps":node._on_gps_odom}[stream]
    # Isolate source age rejection from duplicate/reordered-stamp rejection.
    setattr(node, f"_{stream}_stamp_ns", None)
    callback(grid(stamp) if stream == "map" else odometry(stamp))
    node._evaluate()
    assert node._stable == 0
    assert not node.saves


@pytest.mark.parametrize("stream", ["odom", "gps"])
def test_repeated_odometry_cannot_refresh_age(manager, stream):
    node, clock = manager
    fresh_inputs(node)
    node._on_map(grid(10)); node._evaluate()
    clock.value = 12.
    callback = node._on_odom if stream == "odom" else node._on_gps_odom
    callback(odometry(10))
    assert getattr(node, f"_{stream}_received_at") is None
    assert node._stable == 0


@pytest.mark.parametrize("stream", ["odom", "gps"])
def test_wrong_odometry_frame_is_rejected(manager, stream):
    node, _ = manager
    fresh_inputs(node)
    msg = odometry(11); msg.header.frame_id = "map"
    callback = node._on_odom if stream == "odom" else node._on_gps_odom
    callback(msg)
    assert getattr(node, f"_{stream}_received_at") is None


def test_ready_is_not_revoked_by_late_replayed_map(manager):
    node, _ = manager
    node._finished = True
    node._on_map(grid(0))
    assert not node.statuses


def test_quality_drop_during_save_never_finalizes_manifest(manager):
    node, clock = manager
    for index in range(3):
        clock.value = 10. + index*5
        fresh_inputs(node)
        node._on_map(grid(10+index*5)); node._evaluate()
    assert node._saving
    clock.value = 21.
    fresh_inputs(node)
    msg = grid(21); msg.data[:6] = [-1]*6
    node._on_map(msg)
    node._on_save(NS(result=lambda: NS(result=0)), {})
    assert not node._finished
    assert node._stable == 0
    assert "quality changed" in node.statuses[-1][2]["error"]
    assert not (node._root / "map_lifecycle_manifest.json").exists()
