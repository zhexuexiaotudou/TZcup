import math
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_msgs.msg import TFMessage

from .evaluation import yaw_from_quaternion


class GroundTruthAdapter(Node):
    """Extract the vehicle model pose from Gazebo's world pose stream."""

    def __init__(self):
        super().__init__("ground_truth_adapter")
        self.declare_parameter("world_to_map_x", 8.0)
        self.declare_parameter("world_to_map_y", 0.0)
        self.declare_parameter("world_to_map_yaw", 0.0)
        self.declare_parameter("model_name", "sanitation_vehicle")
        self._publisher = self.create_publisher(Odometry, "/ground_truth/odom", 20)
        self._last_publish_wall = 0.0
        self.create_subscription(
            TFMessage, "/ground_truth/dynamic_pose", self._pose_callback, 20
        )

    def _pose_callback(self, message):
        now_wall = time.monotonic()
        if now_wall - self._last_publish_wall < 0.02:
            return
        self._last_publish_wall = now_wall
        model_name = self.get_parameter("model_name").value
        candidates = [
            transform for transform in message.transforms
            if transform.child_frame_id == model_name
            or transform.header.frame_id == model_name
            or transform.child_frame_id.endswith("/" + model_name)
        ]
        # ros_gz_bridge's Pose_V -> TFMessage conversion in Jazzy does not
        # preserve Gazebo pose names. DynamicPose lists the model pose first,
        # followed by its links, so use index zero as the documented fallback.
        if not candidates and message.transforms:
            candidates = [message.transforms[0]]
        if not candidates:
            return
        transform = candidates[0]
        angle = float(self.get_parameter("world_to_map_yaw").value)
        c, s = math.cos(angle), math.sin(angle)
        world_x = transform.transform.translation.x
        world_y = transform.transform.translation.y
        output = Odometry()
        output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = "map_gt"
        output.child_frame_id = "ground_truth/base_footprint"
        output.pose.pose.position.x = (
            float(self.get_parameter("world_to_map_x").value) + c * world_x - s * world_y
        )
        output.pose.pose.position.y = (
            float(self.get_parameter("world_to_map_y").value) + s * world_x + c * world_y
        )
        output.pose.pose.position.z = transform.transform.translation.z
        output.pose.pose.orientation = transform.transform.rotation
        if angle:
            yaw = yaw_from_quaternion(transform.transform.rotation) + angle
            output.pose.pose.orientation.x = 0.0
            output.pose.pose.orientation.y = 0.0
            output.pose.pose.orientation.z = math.sin(yaw / 2.0)
            output.pose.pose.orientation.w = math.cos(yaw / 2.0)
        self._publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
