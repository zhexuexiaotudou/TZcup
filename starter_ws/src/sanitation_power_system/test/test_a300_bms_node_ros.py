"""ROS regression for BMS safety cadence under frozen or very low RTF."""

from __future__ import annotations

import threading
import time

import pytest

rclpy = pytest.importorskip("rclpy")

from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Bool

from sanitation_power_system.a300_bms_node import A300BmsNode


class _Harness(Node):
    def __init__(self) -> None:
        super().__init__("a300_bms_low_rtf_test_harness")
        self.fault_samples: list[bool] = []
        self.traction_samples: list[bool] = []
        self.estop = self.create_publisher(Bool, "/emergency_stop", 10)
        self.main_power = self.create_publisher(
            Bool, "/formal_vehicle/power/main_power_requested", 10
        )
        self.create_subscription(
            Bool,
            "/formal_vehicle/power/bms_fault",
            lambda message: self.fault_samples.append(bool(message.data)),
            10,
        )
        self.create_subscription(
            Bool,
            "/formal_vehicle/power/traction_permitted",
            lambda message: self.traction_samples.append(bool(message.data)),
            10,
        )
        self.create_timer(0.02, self._publish_inputs)

    def _publish_inputs(self) -> None:
        self.estop.publish(Bool(data=False))
        self.main_power.publish(Bool(data=True))


def test_frozen_sim_clock_does_not_starve_bms_safety_telemetry() -> None:
    rclpy.init()
    harness = _Harness()
    bms = A300BmsNode()
    result = bms.set_parameters([Parameter("use_sim_time", value=True)])
    assert result[0].successful
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(harness)
    executor.add_node(bms)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        # No /clock publisher is created: ROS time stays frozen. A default ROS
        # timer would emit nothing, while the steady scheduler must stay well
        # inside the independent whole-vehicle 0.5 s watchdog.
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline and len(harness.traction_samples) < 8:
            time.sleep(0.02)
        assert len(harness.fault_samples) >= 8
        assert len(harness.traction_samples) >= 8
        assert harness.fault_samples[-1] is False
        assert harness.traction_samples[-1] is True
        assert bms.get_clock().now().nanoseconds == 0
    finally:
        executor.shutdown()
        thread.join(timeout=2.0)
        bms.destroy_node()
        harness.destroy_node()
        rclpy.shutdown()
