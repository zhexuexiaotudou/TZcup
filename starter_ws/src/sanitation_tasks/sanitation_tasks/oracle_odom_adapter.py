"""Oracle-only Gazebo truth odometry lane; never valid as competition evidence."""

import copy

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class OracleOdomAdapter(Node):
    def __init__(self):
        super().__init__("oracle_odom_adapter")
        self.publisher = self.create_publisher(Odometry, "/odom", 50)
        self.broadcaster = TransformBroadcaster(self)
        self.create_subscription(Odometry, "/ground_truth/odom", self.on_truth, 50)

    def on_truth(self, message):
        output = copy.deepcopy(message)
        output.header.frame_id = "odom"
        output.child_frame_id = "base_footprint"
        self.publisher.publish(output)
        transform = TransformStamped()
        transform.header = output.header
        transform.child_frame_id = output.child_frame_id
        transform.transform.translation.x = output.pose.pose.position.x
        transform.transform.translation.y = output.pose.pose.position.y
        transform.transform.translation.z = output.pose.pose.position.z
        transform.transform.rotation = output.pose.pose.orientation
        self.broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args); node = OracleOdomAdapter()
    try: rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException): pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
