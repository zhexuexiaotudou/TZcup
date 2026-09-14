"""Synthetic ROS graph used only to integration-test the acceptance tools.

The fixture publishes zero-valued product messages. It has no simulator,
Gazebo, model-state, reference-pose, or world-truth input and must not be used
as evidence that the vehicle localization stack itself passed acceptance.
"""

from __future__ import annotations

import argparse
import time

from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from tf2_msgs.msg import TFMessage


class PublishingNode(Node):
    """Publish a fixed set of zero-valued messages at 20 Hz."""

    def __init__(self, name: str, messages: dict[str, object]) -> None:
        """Create publishers from topic-to-message mappings."""
        super().__init__(name)
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self._outputs = [
            (self.create_publisher(type(message), topic, qos), message)
            for topic, message in messages.items()
        ]
        self._timer = self.create_timer(0.05, self._publish)

    def _publish(self) -> None:
        stamp = self.get_clock().now().to_msg()
        for publisher, message in self._outputs:
            if isinstance(message, TFMessage):
                for transform in message.transforms:
                    transform.header.stamp = stamp
            elif hasattr(message, "header"):
                message.header.stamp = stamp
            publisher.publish(message)


class SubscriptionNode(Node):
    """Create product-input subscriptions under an expected node name."""

    def __init__(self, name: str, topics: dict[str, type]) -> None:
        """Subscribe to the given product topics without altering messages."""
        super().__init__(name)
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self._fixture_subscriptions = [
            self.create_subscription(message_type, topic, self._ignore, qos)
            for topic, message_type in topics.items()
        ]

    @staticmethod
    def _ignore(message: object) -> None:
        del message


def _map_to_odom() -> TFMessage:
    transform = TransformStamped()
    transform.header.frame_id = "map"
    transform.child_frame_id = "odom"
    transform.transform.rotation.w = 1.0
    return TFMessage(transforms=[transform])


def _fixture_nodes(mode: str) -> list[Node]:
    local_inputs = SubscriptionNode(
        "local_ekf",
        {"/odom/unfiltered": Odometry, "/imu/data": Imu},
    )

    qos = QoSProfile(depth=10)
    qos.reliability = ReliabilityPolicy.BEST_EFFORT
    odom_publisher = local_inputs.create_publisher(Odometry, "/odom", qos)
    local_inputs._fixture_odom_publisher = odom_publisher
    local_inputs._fixture_odom_timer = local_inputs.create_timer(
        0.05, lambda: odom_publisher.publish(Odometry())
    )

    nodes: list[Node] = [
        local_inputs,
        PublishingNode(
            "wheel_driver", {"/odom/unfiltered": Odometry()}
        ),
        PublishingNode("imu_driver", {"/imu/data": Imu()}),
    ]

    if mode == "mapping":
        nodes.append(PublishingNode("slam_toolbox", {"/tf": _map_to_odom()}))
        return nodes

    global_ekf = SubscriptionNode(
        "global_ekf",
        {
            "/odom": Odometry,
            "/amcl_pose": PoseWithCovarianceStamped,
            "/odometry/gps": Odometry,
        },
    )
    fused_publisher = global_ekf.create_publisher(
        Odometry, "/localization/fused_odom", qos
    )
    tf_publisher = global_ekf.create_publisher(TFMessage, "/tf", qos)
    global_ekf._fixture_output_timer = global_ekf.create_timer(
        0.05,
        lambda: (
            fused_publisher.publish(Odometry()),
            tf_publisher.publish(_map_to_odom()),
        ),
    )

    navsat = SubscriptionNode(
        "navsat_transform", {"/gnss/fix": NavSatFix}
    )
    gps_publisher = navsat.create_publisher(Odometry, "/odometry/gps", qos)
    navsat._fixture_output_timer = navsat.create_timer(
        0.05, lambda: gps_publisher.publish(Odometry())
    )
    nodes.extend(
        [
            global_ekf,
            navsat,
            PublishingNode(
                "amcl", {"/amcl_pose": PoseWithCovarianceStamped()}
            ),
            PublishingNode("gnss_driver", {"/gnss/fix": NavSatFix()}),
        ]
    )
    return nodes


def main() -> int:
    """Run a bounded synthetic product-topic graph for tool self-testing."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--mode", choices=("mapping", "cleaning"), required=True
    )
    parser.add_argument("--duration-seconds", type=float, default=5.0)
    args = parser.parse_args()

    rclpy.init()
    nodes = _fixture_nodes(args.mode)
    executor = SingleThreadedExecutor()
    for node in nodes:
        executor.add_node(node)
    deadline = time.monotonic() + args.duration_seconds
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
    finally:
        for node in nodes:
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
