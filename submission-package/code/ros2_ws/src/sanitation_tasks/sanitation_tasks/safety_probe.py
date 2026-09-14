import json
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool

from .evaluation import percentile


class SafetyProbe(Node):
    """Thirty-trial emergency-stop latency and release/timeout gate."""

    def __init__(self):
        super().__init__("sanitation_safety_probe")
        self.declare_parameter("output_path", "safety_probe.json")
        self.declare_parameter("trial_count", 30)
        self.command_publisher = self.create_publisher(Twist, "/cmd_vel_nav", 10)
        self.estop_publisher = self.create_publisher(Bool, "/emergency_stop", 10)
        self.samples = []
        self.create_subscription(Twist, "/cmd_vel", self._on_output, 50)

    def _on_output(self, message):
        self.samples.append((time.monotonic(), message.linear.x, message.angular.z))

    def _pump(self, duration, command=True):
        deadline = time.monotonic() + duration
        message = Twist(); message.linear.x = 0.35; message.angular.z = 0.15
        while rclpy.ok() and time.monotonic() < deadline:
            if command: self.command_publisher.publish(message)
            rclpy.spin_once(self, timeout_sec=0.01); time.sleep(0.01)

    def _wait_for(self, predicate, after, timeout=1.0):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.01)
            for sample in self.samples:
                if sample[0] >= after and predicate(sample): return sample
            time.sleep(0.005)
        return None

    def _set_estop(self, stopped, settle=0.12):
        deadline = time.monotonic() + settle
        while time.monotonic() < deadline:
            self.estop_publisher.publish(Bool(data=stopped))
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.01)

    def run(self):
        self._set_estop(False, 0.5); self._pump(1.0)
        command_passed = any(abs(sample[1]) > 0.2 for sample in self.samples)
        trials = []
        releases_ok = True
        for index in range(int(self.get_parameter("trial_count").value)):
            self._set_estop(False); self._pump(0.10)
            publish_time = time.monotonic(); self.estop_publisher.publish(Bool(data=True))
            zero = self._wait_for(lambda sample: abs(sample[1]) < 1e-6 and abs(sample[2]) < 1e-6, publish_time, 1.2)
            latency = zero[0] - publish_time if zero else None
            release_time = time.monotonic(); self._set_estop(False); self._pump(0.08)
            resumed = self._wait_for(lambda sample: abs(sample[1]) > 0.2, release_time, 1.0) is not None
            releases_ok = releases_ok and resumed
            trials.append({"trial": index + 1, "estop_publish_monotonic_sec": publish_time, "first_zero_monotonic_sec": zero[0] if zero else None, "latency_sec": latency, "resume_after_release": resumed})
        self._set_estop(False); self._pump(0.3)
        timeout_index = len(self.samples); self._pump(1.5, command=False)
        timeout_samples = self.samples[timeout_index:]
        timeout_zero = bool(timeout_samples) and all(
            abs(sample[1]) < 1e-6 and abs(sample[2]) < 1e-6
            for sample in timeout_samples[-5:]
        )
        latencies = [item["latency_sec"] for item in trials if item["latency_sec"] is not None]
        p50 = percentile(latencies, 0.50); p95 = percentile(latencies, 0.95); maximum = max(latencies) if latencies else None
        report = {
            "schema_version": 2, "trial_count": len(trials), "completed_trial_count": len(latencies),
            "command_passed": command_passed, "emergency_stop_zeroed": len(latencies) == len(trials),
            "resume_after_release": releases_ok, "stale_command_zeroed": timeout_zero,
            "latency_sec": {"p50": p50, "p95": p95, "max": maximum},
            "competition_estop_pass": bool(p95 is not None and p95 <= 1.0),
            "trials": trials,
        }
        report["success"] = all([report["command_passed"], report["emergency_stop_zeroed"], report["resume_after_release"], report["stale_command_zeroed"], report["competition_estop_pass"]])
        output = Path(self.get_parameter("output_path").value); output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        self.get_logger().info(json.dumps(report, ensure_ascii=False))
        return 0 if report["success"] else 2


def main(args=None):
    rclpy.init(args=args); node = SafetyProbe()
    try: code = node.run()
    finally: node.destroy_node(); rclpy.shutdown()
    raise SystemExit(code)


if __name__ == "__main__": main()
