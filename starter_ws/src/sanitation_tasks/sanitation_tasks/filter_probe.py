import json
import math
import statistics
import time
from pathlib import Path

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import SpeedLimit
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node


def rectangle_distance(x, y, bounds):
    dx = max(bounds[0] - x, 0.0, x - bounds[2])
    dy = max(bounds[1] - y, 0.0, y - bounds[3])
    return math.hypot(dx, dy)


class FilterProbe(Node):
    def __init__(self):
        super().__init__("filter_probe")
        self.declare_parameter("output_path", "/tmp/filter_report.json")
        self.client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.samples = []; self.speed_limits = []
        self.create_subscription(Odometry, "/ground_truth/odom", self._truth, 20)
        self.create_subscription(SpeedLimit, "/speed_limit", self._speed_limit, 10)

    def _truth(self, message):
        pose = message.pose.pose; stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        self.samples.append((stamp, pose.position.x, pose.position.y))

    def _speed_limit(self, message): self.speed_limits.append({"wall_time": time.monotonic(), "speed_limit": message.speed_limit, "percentage": message.percentage})

    def _navigate(self, x, y, timeout=180.0):
        goal = NavigateToPose.Goal(); goal.pose = PoseStamped(); goal.pose.header.frame_id = "map"
        goal.pose.pose.position.x = x; goal.pose.pose.position.y = y; goal.pose.pose.orientation.w = 1.0
        send = self.client.send_goal_async(goal); rclpy.spin_until_future_complete(self, send, timeout_sec=10.0)
        handle = send.result() if send.done() else None
        if handle is None or not handle.accepted: return False
        result = handle.get_result_async(); deadline = time.monotonic() + timeout
        while rclpy.ok() and not result.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not result.done():
            handle.cancel_goal_async(); return False
        wrapped = result.result(); return wrapped.status == GoalStatus.STATUS_SUCCEEDED and int(getattr(wrapped.result, "error_code", 0)) == 0

    def run(self):
        if not self.client.wait_for_server(timeout_sec=60.0): return self._write({"success": False, "error": "navigate_server_timeout"})
        self._navigate(0.0, 2.0)
        keepout_start = len(self.samples); keepout_goal_success = self._navigate(6.0, 2.0)
        keepout_samples = self.samples[keepout_start:]; bounds = (2.0, 1.0, 4.0, 3.0)
        violations = sum(bounds[0] <= x <= bounds[2] and bounds[1] <= y <= bounds[3] for _stamp, x, y in keepout_samples)
        minimum_distance = min((rectangle_distance(x, y, bounds) for _stamp, x, y in keepout_samples), default=None)
        self._navigate(-5.0, 0.0)
        speed_start = len(self.samples); speed_goal_success = self._navigate(5.0, 0.0)
        speed_samples = self.samples[speed_start:]
        buckets = {"before": [], "inside": [], "after": []}
        for previous, current in zip(speed_samples, speed_samples[1:]):
            dt = current[0] - previous[0]
            if dt <= 0: continue
            speed = math.hypot(current[1] - previous[1], current[2] - previous[2]) / dt
            bucket = "before" if current[1] < -2.0 else "after" if current[1] > 2.0 else "inside"
            buckets[bucket].append(speed)
        mean_speeds = {key: statistics.fmean(values) if values else None for key, values in buckets.items()}
        speed_compliance = bool(mean_speeds["inside"] is not None and mean_speeds["inside"] <= 0.30 and mean_speeds["before"] is not None and mean_speeds["after"] is not None)
        report = {
            "keepout": {"mask_polygon": list(bounds), "goal_crossed_polygon_direct_line": True, "navigation_succeeded": keepout_goal_success, "violation_sample_count": violations, "minimum_distance_m": minimum_distance, "keepout_pass": keepout_goal_success and violations == 0},
            "speed_zone": {"polygon": [-2.0, -2.0, 2.0, 2.0], "navigation_succeeded": speed_goal_success, "mean_speed_m_s": mean_speeds, "speed_limit_messages": self.speed_limits, "speed_compliance_pass": speed_goal_success and speed_compliance},
        }
        report["success"] = report["keepout"]["keepout_pass"] and report["speed_zone"]["speed_compliance_pass"]
        return self._write(report)

    def _write(self, report):
        output = Path(self.get_parameter("output_path").value); output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        return 0 if report.get("success") else 2


def main(args=None):
    rclpy.init(args=args); node = FilterProbe()
    try: code = node.run()
    finally: node.destroy_node(); rclpy.shutdown()
    raise SystemExit(code)
