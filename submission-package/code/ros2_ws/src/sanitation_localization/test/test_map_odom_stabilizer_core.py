from pathlib import Path
import math

import pytest

from sanitation_localization.map_odom_stabilizer_core import (
    CausalMapOdomStabilizer,
    Pose2D,
    StabilizerInputError,
)


ROOT = Path(__file__).resolve().parents[1]


def sample(t, x=0.0, y=0.0, yaw=0.0):
    return Pose2D(t, x, y, yaw)


def test_constant_transform_is_unchanged():
    stabilizer = CausalMapOdomStabilizer()
    rows = [sample(index * 0.02, x=0.25, y=-0.5, yaw=0.75) for index in range(20)]
    assert [stabilizer.update(row) for row in rows] == rows


def test_filter_is_causal_when_future_values_change():
    prefix = [sample(0.0), sample(0.1, x=1.0)]
    first = CausalMapOdomStabilizer()
    second = CausalMapOdomStabilizer()
    prefix_outputs = [first.update(row) for row in prefix]
    assert [second.update(row) for row in prefix] == prefix_outputs
    first.update(sample(0.2, x=10.0))
    second.update(sample(0.2, x=-10.0))
    assert prefix_outputs == [sample(0.0), sample(0.1, x=0.06449301496838222)]


def test_angle_uses_short_arc():
    stabilizer = CausalMapOdomStabilizer(tau_sec=0.75)
    stabilizer.update(sample(0.0, yaw=math.pi - 0.01))
    output = stabilizer.update(sample(0.1, yaw=-math.pi + 0.01))
    assert abs(abs(output.yaw_rad) - math.pi) < 0.1


def test_input_gap_latches_fail_closed():
    stabilizer = CausalMapOdomStabilizer(max_gap_sec=0.2)
    stabilizer.update(sample(0.0))
    with pytest.raises(StabilizerInputError, match="input_gap"):
        stabilizer.update(sample(0.3))
    with pytest.raises(StabilizerInputError, match="input_gap"):
        stabilizer.update(sample(0.32))


def test_duplicate_and_conflicting_timestamp_contract():
    stabilizer = CausalMapOdomStabilizer()
    row = sample(1.0, x=1.0)
    assert stabilizer.update(row) == row
    assert stabilizer.update(row) == row
    assert stabilizer.duplicate_updates == 1
    with pytest.raises(StabilizerInputError, match="conflicting"):
        stabilizer.update(sample(1.0, x=2.0))


@pytest.mark.parametrize(
    "kwargs",
    [
        {"tau_sec": 0.5},
        {"tau_sec": 3.1},
        {"max_dt_sec": 0.01},
        {"max_dt_sec": 0.21},
        {"max_gap_sec": 0.09},
        {"max_gap_sec": 1.01},
    ],
)
def test_parameter_bounds_fail_closed(kwargs):
    with pytest.raises(ValueError):
        CausalMapOdomStabilizer(**kwargs)


def test_launch_and_node_contract_are_truth_isolated():
    launch = (ROOT / "launch/formal_localization_fusion.launch.py").read_text()
    node = (ROOT / "sanitation_localization/map_odom_stabilizer.py").read_text()
    setup = (ROOT / "setup.py").read_text()
    assert 'DeclareLaunchArgument("map_odom_stabilizer", default_value="false")' in launch
    assert 'map_odom_stabilizer_tau_sec", default_value="1.5"' in launch
    assert '("/tf", raw_map_odom_topic)' in launch
    assert "UnlessCondition(map_odom_stabilizer)" in launch
    assert "IfCondition(map_odom_stabilizer)" in launch
    assert "map_odom_stabilizer = sanitation_localization.map_odom_stabilizer:main" in setup
    for forbidden in ("/ground_truth", "/world", "/gazebo", "/model"):
        assert forbidden not in node
