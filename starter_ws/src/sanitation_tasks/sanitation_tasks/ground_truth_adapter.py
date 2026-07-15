"""Fail-closed adapter for the model-scoped Gazebo ground-truth odometry."""

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool

from .evaluation import yaw_from_quaternion


def transform_planar_odometry(message, offset_x, offset_y, offset_yaw):
    """Return map_gt odometry while preserving the source timestamp and twist."""
    c, s = math.cos(offset_yaw), math.sin(offset_yaw)
    source = message.pose.pose.position
    source_yaw = yaw_from_quaternion(message.pose.pose.orientation)

    output = Odometry()
    output.header.stamp = message.header.stamp
    output.header.frame_id = "map_gt"
    output.child_frame_id = "ground_truth/base_footprint"
    output.pose.pose.position.x = offset_x + c * source.x - s * source.y
    output.pose.pose.position.y = offset_y + s * source.x + c * source.y
    output.pose.pose.position.z = source.z
    output.pose.pose.orientation.z = math.sin((source_yaw + offset_yaw) / 2.0)
    output.pose.pose.orientation.w = math.cos((source_yaw + offset_yaw) / 2.0)
    output.pose.covariance = list(message.pose.covariance)
    output.twist = message.twist
    return output


def identity_matches(message, expected_source_frame, expected_child_frame):
    position = message.pose.pose.position
    orientation = message.pose.pose.orientation
    finite_pose = all(
        math.isfinite(value)
        for value in (
            position.x,
            position.y,
            position.z,
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
    )
    return (
        message.header.frame_id == expected_source_frame
        and message.child_frame_id == expected_child_frame
        and finite_pose
    )


class GroundTruthAdapter(Node):
    """Accept only the dedicated model odometry; never infer identity by index."""

    def __init__(self):
        super().__init__("ground_truth_adapter")
        self.declare_parameter("world_to_map_x", 8.0)
        self.declare_parameter("world_to_map_y", 0.0)
        self.declare_parameter("world_to_map_yaw", 0.0)
        self.declare_parameter("expected_source_frame", "world")
        self.declare_parameter(
            "expected_child_frame", "sanitation_vehicle/base_footprint"
        )
        self._publisher = self.create_publisher(Odometry, "/ground_truth/odom", 20)
        self._identity_publisher = self.create_publisher(
            Bool, "/ground_truth/identity_valid", 1
        )
        self._rejected = 0
        self.create_subscription(
            Odometry, "/ground_truth/model_odom_raw", self._odom_callback, 20
        )

    def _odom_callback(self, message):
        expected_source = str(self.get_parameter("expected_source_frame").value)
        expected_child = str(self.get_parameter("expected_child_frame").value)
        valid = identity_matches(message, expected_source, expected_child)
        self._identity_publisher.publish(Bool(data=valid))
        if not valid:
            self._rejected += 1
            self.get_logger().error(
                "ground truth rejected (fail-closed): "
                f"frame={message.header.frame_id!r}, child={message.child_frame_id!r}, "
                f"expected=({expected_source!r}, {expected_child!r}), "
                f"rejected_count={self._rejected}"
            )
            return

        output = transform_planar_odometry(
            message,
            float(self.get_parameter("world_to_map_x").value),
            float(self.get_parameter("world_to_map_y").value),
            float(self.get_parameter("world_to_map_yaw").value),
        )
        self._publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthAdapter()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
