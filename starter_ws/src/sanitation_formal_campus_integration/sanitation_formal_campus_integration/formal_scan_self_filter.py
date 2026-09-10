"""Filter only mesh-proven formal-vehicle returns from the UTM scan."""

from __future__ import annotations

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

from .scan_self_filter_core import filter_ranges, parse_masks


class FormalScanSelfFilter(Node):
    """Publish one canonical scan shared by SLAM, Nav2 and collision monitor."""

    def __init__(self) -> None:
        super().__init__("formal_scan_self_filter")
        self.declare_parameter("input_topic", "/scan")
        self.declare_parameter("output_topic", "/scan/navigation")
        self.declare_parameter(
            "angular_range_masks_rad", Parameter.Type.DOUBLE_ARRAY
        )
        self.declare_parameter("mesh_ray_occluded_count", 0)
        self.declare_parameter("mesh_ray_total_count", 0)
        self.declare_parameter("geometry_source", "")
        self.declare_parameter("normalize_positive_infinity", False)
        self.declare_parameter("expected_sensor_range_max_m", 30.0)
        self.declare_parameter("no_return_replacement_m", 12.0)
        self._masks = parse_masks(
            self.get_parameter("angular_range_masks_rad").value
        )
        if not self._masks:
            raise ValueError("formal scan self filter requires a non-empty mask")
        # Offer reliable delivery on this internal one-hop topic so the SLAM
        # startup gate cannot remain matched-but-starved under a slow full-
        # campus simulation.  Depth one still drops superseded frames instead
        # of building a stale scan backlog.  Best-effort navigation consumers
        # remain DDS-compatible with a reliable publisher.
        filtered_scan_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._publisher = self.create_publisher(
            LaserScan,
            str(self.get_parameter("output_topic").value),
            filtered_scan_qos,
        )
        self.create_subscription(
            LaserScan,
            str(self.get_parameter("input_topic").value),
            self._scan,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE),
        )
        self._received_count = 0
        self._published_count = 0
        self.create_timer(5.0, self._report_health)

    def _scan(self, message: LaserScan) -> None:
        self._received_count += 1
        if self._received_count == 1 or self._received_count % 100 == 0:
            self.get_logger().info(
                "scan filter input active: "
                f"received={self._received_count} "
                f"output_subscriptions={self._publisher.get_subscription_count()}"
            )
        output = LaserScan()
        output.header = message.header
        output.angle_min = message.angle_min
        output.angle_max = message.angle_max
        output.angle_increment = message.angle_increment
        output.time_increment = message.time_increment
        output.scan_time = message.scan_time
        output.range_min = message.range_min
        output.range_max = message.range_max
        expected_range_max = float(
            self.get_parameter("expected_sensor_range_max_m").value
        )
        if abs(float(message.range_max) - expected_range_max) > 1e-3:
            raise ValueError(
                "formal lidar range_max does not match the sealed sensor contract"
            )
        output.ranges, _, _ = filter_ranges(
            angle_min=message.angle_min,
            angle_increment=message.angle_increment,
            ranges=message.ranges,
            masks=self._masks,
            range_max=message.range_max,
            normalize_positive_infinity=bool(
                self.get_parameter("normalize_positive_infinity").value
            ),
            no_return_replacement_m=float(
                self.get_parameter("no_return_replacement_m").value
            ),
        )
        output.intensities = list(message.intensities)
        self._publisher.publish(output)
        self._published_count += 1

    def _report_health(self) -> None:
        self.get_logger().info(
            "scan filter health: "
            f"received={self._received_count} "
            f"published={self._published_count} "
            f"output_subscriptions={self._publisher.get_subscription_count()}"
        )


def main() -> None:
    rclpy.init()
    node = FormalScanSelfFilter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
