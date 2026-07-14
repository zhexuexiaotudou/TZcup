import json
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool


class SafetyProbe(Node):
    def __init__(self) -> None:
        super().__init__("sanitation_safety_probe")
        self.declare_parameter("output_path", "safety_probe.json")
        self.command_publisher = self.create_publisher(Twist, "/cmd_vel_nav", 10)
        self.estop_publisher = self.create_publisher(Bool, "/emergency_stop", 10)
        self.samples = []
        self.create_subscription(Twist, "/cmd_vel", self._on_output, 10)

    def _on_output(self, message) -> None:
        self.samples.append((time.monotonic(), message.linear.x, message.angular.z))

    def _exercise(self, stopped, duration=1.0, publish_commands=True):
        start_index = len(self.samples)
        self.estop_publisher.publish(Bool(data=stopped))
        deadline = time.monotonic() + duration
        command = Twist()
        command.linear.x = 0.35
        command.angular.z = 0.15
        while rclpy.ok() and time.monotonic() < deadline:
            if publish_commands:
                self.command_publisher.publish(command)
            rclpy.spin_once(self, timeout_sec=0.05)
        return self.samples[start_index:]

    def run(self) -> int:
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.05)
        pass_samples = self._exercise(False)
        stop_samples = self._exercise(True)
        resume_samples = self._exercise(False)
        timeout_samples = self._exercise(False, duration=1.2, publish_commands=False)

        pass_ok = any(abs(linear) > 0.2 for _, linear, _ in pass_samples)
        stop_ok = bool(stop_samples) and all(
            abs(linear) < 1.0e-6 and abs(angular) < 1.0e-6
            for _, linear, angular in stop_samples[-5:]
        )
        resume_ok = any(abs(linear) > 0.2 for _, linear, _ in resume_samples)
        timeout_ok = bool(timeout_samples) and all(
            abs(linear) < 1.0e-6 and abs(angular) < 1.0e-6
            for _, linear, angular in timeout_samples[-5:]
        )
        report = {
            "success": pass_ok and stop_ok and resume_ok and timeout_ok,
            "command_passed": pass_ok,
            "emergency_stop_zeroed": stop_ok,
            "resume_after_release": resume_ok,
            "stale_command_zeroed": timeout_ok,
            "sample_counts": {
                "pass": len(pass_samples),
                "stop": len(stop_samples),
                "resume": len(resume_samples),
                "timeout": len(timeout_samples),
            },
        }
        output_path = Path(str(self.get_parameter("output_path").value))
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(
            json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
        self.get_logger().info(json.dumps(report, ensure_ascii=False))
        return 0 if report["success"] else 2


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SafetyProbe()
    try:
        code = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(code)


if __name__ == "__main__":
    main()
