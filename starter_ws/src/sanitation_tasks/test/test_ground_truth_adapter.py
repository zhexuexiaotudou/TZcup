import math
from types import SimpleNamespace

import rclpy
from rclpy.parameter import Parameter

from nav_msgs.msg import Odometry

from sanitation_tasks.ground_truth_adapter import (
    identity_matches,
    transform_planar_odometry,
    GroundTruthAdapter,
)


def source_message():
    message = Odometry()
    message.header.frame_id = "world"
    message.child_frame_id = "sanitation_vehicle/base_footprint"
    message.header.stamp.sec = 12
    message.header.stamp.nanosec = 34
    message.pose.pose.position.x = -8.0
    message.pose.pose.position.y = 1.0
    message.pose.pose.orientation.z = math.sin(0.25)
    message.pose.pose.orientation.w = math.cos(0.25)
    return message


def test_exact_model_identity_is_required():
    message = source_message()
    assert identity_matches(message, "world", "sanitation_vehicle/base_footprint")
    message.child_frame_id = "some_link"
    assert not identity_matches(message, "world", "sanitation_vehicle/base_footprint")


def test_identity_rejects_zero_or_invalid_stamp_and_non_normalized_quaternion():
    message = source_message()
    message.header.stamp.sec = 0
    message.header.stamp.nanosec = 0
    assert not identity_matches(message, "world", "sanitation_vehicle/base_footprint")
    message = source_message()
    message.header.stamp.nanosec = 1_000_000_000
    assert not identity_matches(message, "world", "sanitation_vehicle/base_footprint")
    message = source_message()
    message.pose.pose.orientation.w = 0.0
    message.pose.pose.orientation.z = 0.0
    assert not identity_matches(message, "world", "sanitation_vehicle/base_footprint")


def test_world_to_map_gt_transform_preserves_timestamp():
    message = source_message()
    output = transform_planar_odometry(message, 8.0, 0.0, 0.0)
    assert output.header.frame_id == "map_gt"
    assert output.child_frame_id == "ground_truth/base_footprint"
    assert output.header.stamp == message.header.stamp
    assert output.pose.pose.position.x == 0.0
    assert output.pose.pose.position.y == 1.0


def test_world_to_map_gt_rotation_is_explicit():
    message = source_message()
    output = transform_planar_odometry(message, 0.0, 0.0, math.pi / 2.0)
    assert abs(output.pose.pose.position.x + 1.0) < 1e-12
    assert abs(output.pose.pose.position.y + 8.0) < 1e-12


def test_formal_identity_gate_is_explicit_and_legacy_transform_remains_usable():
    rclpy.init()
    node = GroundTruthAdapter()
    received, identities = [], []
    node._publisher = SimpleNamespace(publish=received.append)
    node._identity_publisher = SimpleNamespace(publish=identities.append)
    try:
        node._odom_callback(source_message())
        assert len(received) == 1 and received[-1].pose.pose.position.x == 0.0
        node.set_parameters([Parameter("require_episode_identity", value=True)])
        node._odom_callback(source_message())
        assert len(received) == 1 and identities[-1].data is False
        node.set_parameters([Parameter("source_episode_manifest_sha256", value="a" * 64)])
        node._odom_callback(source_message())
        assert len(received) == 2 and identities[-1].data is True
    finally:
        node.destroy_node()
        rclpy.shutdown()
