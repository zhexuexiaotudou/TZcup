import json
import math
import time
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateThroughPoses
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile


WAYPOINTS = [
    (0.8, 0.0, 0.0),
    (1.2, 0.8, 1.57),
    (0.5, 1.3, 3.14),
    (-0.5, 1.3, 3.14),
    (-1.2, 0.8, -1.57),
    (-1.2, -0.2, -1.57),
    (-0.8, -1.0, 0.0),
    (0.0, -1.3, 0.0),
    (0.8, -1.0, 1.57),
    (0.0, 0.0, 3.14),
]


class NavigationProbe(Node):
    def __init__(self) -> None:
        super().__init__("sanitation_navigation_probe")
        self.declare_parameter("timeout_sec", 300.0)
        self.declare_parameter("output_path", "navigation_probe.json")
        self.action_client = ActionClient(
            self, NavigateThroughPoses, "/navigate_through_poses"
        )
        self.lifecycle_client = self.create_client(
            GetState, "/bt_navigator/get_state"
        )
        self.initial_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", QoSProfile(depth=1)
        )
        self.last_odom = None
        self.last_amcl = None
        self.feedback_samples = []
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._on_amcl, 10
        )

    def _on_odom(self, message) -> None:
        self.last_odom = message

    def _on_amcl(self, message) -> None:
        self.last_amcl = message

    def _publish_initial_pose(self) -> None:
        message = PoseWithCovarianceStamped()
        message.header.frame_id = "map"
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.pose.orientation.w = 1.0
        message.pose.covariance[0] = 0.04
        message.pose.covariance[7] = 0.04
        message.pose.covariance[35] = 0.07
        self.initial_pose_publisher.publish(message)

    def _feedback(self, message) -> None:
        feedback = message.feedback
        if len(self.feedback_samples) < 200:
            self.feedback_samples.append(
                {
                    "poses_remaining": int(feedback.number_of_poses_remaining),
                    "recoveries": int(feedback.number_of_recoveries),
                    "distance_remaining_m": float(feedback.distance_remaining),
                }
            )

    def run(self) -> int:
        timeout_sec = float(self.get_parameter("timeout_sec").value)
        output_path = Path(str(self.get_parameter("output_path").value))
        output_path.parent.mkdir(parents=True, exist_ok=True)
        deadline = time.monotonic() + timeout_sec

        if not self.action_client.wait_for_server(timeout_sec=60.0):
            return self._write(output_path, {"success": False, "error": "action_timeout"})
        if not self._wait_for_active():
            return self._write(output_path, {"success": False, "error": "bt_inactive"})

        for _ in range(20):
            self._publish_initial_pose()
            rclpy.spin_once(self, timeout_sec=0.15)
            if self.last_amcl is not None:
                break

        goal = NavigateThroughPoses.Goal()
        goal.poses = [self._pose(x, y, yaw) for x, y, yaw in WAYPOINTS]
        sent = self.action_client.send_goal_async(goal, feedback_callback=self._feedback)
        rclpy.spin_until_future_complete(self, sent, timeout_sec=15.0)
        goal_handle = sent.result() if sent.done() else None
        if goal_handle is None or not goal_handle.accepted:
            return self._write(output_path, {"success": False, "error": "goal_rejected"})

        result_future = goal_handle.get_result_async()
        while rclpy.ok() and not result_future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)

        if not result_future.done():
            goal_handle.cancel_goal_async()
            return self._write(output_path, {"success": False, "error": "goal_timeout"})

        wrapped_result = result_future.result()
        status = int(wrapped_result.status)
        report = {
            "success": status == GoalStatus.STATUS_SUCCEEDED,
            "waypoint_count": len(WAYPOINTS),
            "status": status,
            "status_expected": GoalStatus.STATUS_SUCCEEDED,
            "feedback_samples": self.feedback_samples,
            "recoveries_max": max(
                (item["recoveries"] for item in self.feedback_samples), default=0
            ),
            "final_odom_xy_m": self._xy(self.last_odom),
            "final_amcl_xy_m": self._xy(self.last_amcl),
            "amcl_covariance_trace_xy": self._amcl_covariance_trace(),
        }
        return self._write(output_path, report)

    def _wait_for_active(self):
        if not self.lifecycle_client.wait_for_service(timeout_sec=60.0):
            return False
        deadline = time.monotonic() + 60.0
        while rclpy.ok() and time.monotonic() < deadline:
            future = self.lifecycle_client.call_async(GetState.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            if future.done() and future.result().current_state.label == "active":
                return True
            rclpy.spin_once(self, timeout_sec=0.2)
        return False

    def _pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    @staticmethod
    def _xy(message):
        if message is None:
            return None
        pose = message.pose.pose
        return [float(pose.position.x), float(pose.position.y)]

    def _amcl_covariance_trace(self):
        if self.last_amcl is None:
            return None
        covariance = self.last_amcl.pose.covariance
        return float(covariance[0] + covariance[7])

    def _write(self, output_path, report):
        output_path.write_text(
            json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
        self.get_logger().info(json.dumps(report, ensure_ascii=False))
        return 0 if report["success"] else 2


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NavigationProbe()
    try:
        code = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(code)


if __name__ == "__main__":
    main()
