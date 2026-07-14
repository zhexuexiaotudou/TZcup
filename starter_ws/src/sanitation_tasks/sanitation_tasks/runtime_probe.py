import json
import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState, LaserScan, PointCloud2
from tf2_msgs.msg import TFMessage


class RuntimeProbe(Node):
    def __init__(self) -> None:
        super().__init__("sanitation_runtime_probe")
        self.declare_parameter("timeout_sec", 90.0)
        self.declare_parameter("motion_sec", 5.0)
        self.declare_parameter("output_path", "runtime_probe.json")

        regular_qos = QoSProfile(depth=10)
        static_tf_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.specifications = {
            "/clock": (Clock, regular_qos),
            "/odom/unfiltered": (Odometry, regular_qos),
            "/odom": (Odometry, regular_qos),
            "/imu/data": (Imu, qos_profile_sensor_data),
            "/joint_states": (JointState, qos_profile_sensor_data),
            "/scan": (LaserScan, qos_profile_sensor_data),
            "/camera/color/camera_info": (CameraInfo, qos_profile_sensor_data),
            "/camera/color/image_raw": (Image, qos_profile_sensor_data),
            "/camera/depth/image_rect_raw": (Image, qos_profile_sensor_data),
            "/camera/depth/color/points": (PointCloud2, qos_profile_sensor_data),
            "/tf": (TFMessage, regular_qos),
            "/tf_static": (TFMessage, static_tf_qos),
        }
        self.message_counts = {topic: 0 for topic in self.specifications}
        self.last_unfiltered_odom = None
        self._probe_subscriptions = []

        for topic, (message_type, qos) in self.specifications.items():
            self._probe_subscriptions.append(
                self.create_subscription(
                    message_type,
                    topic,
                    self._callback_for(topic),
                    qos,
                )
            )
        self.cmd_vel_publisher = self.create_publisher(Twist, "/cmd_vel", 10)

    def _callback_for(self, topic):
        def callback(message):
            self.message_counts[topic] += 1
            if topic == "/odom/unfiltered":
                self.last_unfiltered_odom = message

        return callback

    def run(self) -> int:
        timeout_sec = float(self.get_parameter("timeout_sec").value)
        motion_sec = float(self.get_parameter("motion_sec").value)
        output_path = Path(str(self.get_parameter("output_path").value)).expanduser()
        output_path.parent.mkdir(parents=True, exist_ok=True)

        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            if all(count > 0 for count in self.message_counts.values()):
                break

        missing = sorted(
            topic for topic, count in self.message_counts.items() if count == 0
        )
        start_xy = self._odom_xy()
        if not missing and start_xy is not None:
            command = Twist()
            command.linear.x = 0.25
            motion_deadline = time.monotonic() + motion_sec
            while rclpy.ok() and time.monotonic() < motion_deadline:
                self.cmd_vel_publisher.publish(command)
                rclpy.spin_once(self, timeout_sec=0.1)
            self.cmd_vel_publisher.publish(Twist())
            settle_deadline = time.monotonic() + 1.0
            while rclpy.ok() and time.monotonic() < settle_deadline:
                rclpy.spin_once(self, timeout_sec=0.1)

        end_xy = self._odom_xy()
        displacement_m = None
        if start_xy is not None and end_xy is not None:
            displacement_m = math.hypot(end_xy[0] - start_xy[0], end_xy[1] - start_xy[1])

        topics_ok = not missing
        motion_ok = displacement_m is not None and displacement_m >= 0.01
        report = {
            "success": topics_ok and motion_ok,
            "timeout_sec": timeout_sec,
            "motion_sec": motion_sec,
            "message_counts": self.message_counts,
            "missing_topics": missing,
            "start_xy_m": start_xy,
            "end_xy_m": end_xy,
            "displacement_m": displacement_m,
            "motion_threshold_m": 0.01,
            "topics_ok": topics_ok,
            "motion_ok": motion_ok,
        }
        output_path.write_text(
            json.dumps(report, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        self.get_logger().info(json.dumps(report, ensure_ascii=False))
        return 0 if report["success"] else 2

    def _odom_xy(self):
        if self.last_unfiltered_odom is None:
            return None
        position = self.last_unfiltered_odom.pose.pose.position
        return [position.x, position.y]


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RuntimeProbe()
    try:
        code = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(code)


if __name__ == "__main__":
    main()
