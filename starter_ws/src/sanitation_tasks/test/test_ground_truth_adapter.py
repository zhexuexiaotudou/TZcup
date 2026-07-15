import math

from nav_msgs.msg import Odometry

from sanitation_tasks.ground_truth_adapter import (
    identity_matches,
    transform_planar_odometry,
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
