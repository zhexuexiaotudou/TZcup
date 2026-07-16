from collections import deque
import math

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float64

from .model import GnssNoiseModel, PROFILES, local_xy_to_wgs84


def _yaw_from_quaternion(quaternion):
    siny_cosp = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cosy_cosp = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(siny_cosp, cosy_cosp)


class GnssSimNode(Node):
    def __init__(self):
        super().__init__("sanitation_gnss_sim")
        profile_name = self.declare_parameter("profile", "rtk_fixed").value
        if profile_name not in PROFILES:
            raise ValueError(f"Unknown GNSS profile: {profile_name}")
        self.profile = PROFILES[profile_name]
        self.model = GnssNoiseModel(
            self.profile, int(self.declare_parameter("random_seed", 0).value)
        )
        self.origin_latitude = float(
            self.declare_parameter("origin_latitude_deg", 31.2304).value
        )
        self.origin_longitude = float(
            self.declare_parameter("origin_longitude_deg", 121.4737).value
        )
        self.publish_heading = bool(
            self.declare_parameter("publish_heading", True).value
        )
        self.publish_velocity = bool(
            self.declare_parameter("publish_velocity", True).value
        )
        self.queue = deque()
        self.last_sample_time = None
        self.last_truth_stamp_ns = None
        self.fix_publisher = self.create_publisher(NavSatFix, "/gnss/fix", 20)
        self.heading_publisher = self.create_publisher(Float64, "/gnss/heading", 20)
        self.velocity_publisher = self.create_publisher(
            TwistStamped, "/gnss/velocity", 20
        )
        self.diagnostic_publisher = self.create_publisher(
            DiagnosticArray, "/gnss/diagnostics", 10
        )
        self.create_subscription(
            Odometry, "/ground_truth/odom", self._on_truth, 50
        )
        self.create_timer(0.01, self._flush_queue)
        self.create_timer(1.0, self._publish_diagnostic)

    def _on_truth(self, message):
        stamp_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        minimum_period_ns = int(1_000_000_000 / self.profile.rate_hz)
        if (
            self.last_truth_stamp_ns is not None
            and stamp_ns - self.last_truth_stamp_ns < minimum_period_ns
        ):
            return
        self.last_truth_stamp_ns = stamp_ns
        now_seconds = self.get_clock().now().nanoseconds / 1e9
        dt = 1.0 / self.profile.rate_hz
        if self.last_sample_time is not None:
            dt = max(0.0, now_seconds - self.last_sample_time)
        self.last_sample_time = now_seconds
        measurement = self.model.sample(
            message.pose.pose.position.x, message.pose.pose.position.y, dt
        )
        if not measurement.publish:
            return
        latitude, longitude = local_xy_to_wgs84(
            measurement.x_m,
            measurement.y_m,
            self.origin_latitude,
            self.origin_longitude,
        )
        fix = NavSatFix()
        fix.header = message.header
        fix.header.frame_id = "gnss_link"
        fix.status.status = NavSatStatus.STATUS_GBAS_FIX
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.latitude = latitude
        fix.longitude = longitude
        fix.altitude = 0.0
        fix.position_covariance[0] = measurement.variance_m2
        fix.position_covariance[4] = measurement.variance_m2
        fix.position_covariance[8] = max(0.01, measurement.variance_m2 * 4.0)
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        release_time = now_seconds + self.profile.latency_s
        heading = _yaw_from_quaternion(message.pose.pose.orientation)
        self.queue.append((release_time, fix, heading, message.twist.twist))

    def _flush_queue(self):
        now_seconds = self.get_clock().now().nanoseconds / 1e9
        while self.queue and self.queue[0][0] <= now_seconds:
            _, fix, heading, twist = self.queue.popleft()
            self.fix_publisher.publish(fix)
            if self.publish_heading:
                heading_message = Float64()
                heading_message.data = heading
                self.heading_publisher.publish(heading_message)
            if self.publish_velocity:
                velocity = TwistStamped()
                velocity.header = fix.header
                velocity.twist = twist
                self.velocity_publisher.publish(velocity)

    def _publish_diagnostic(self):
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = "sanitation_gnss_sim"
        status.hardware_id = "simulation_only"
        status.level = DiagnosticStatus.OK
        status.message = self.profile.name
        values = {
            "profile": self.profile.name,
            "simulated_sensor": "true",
            "ground_truth_direct_fusion": "false",
            "publish_enabled": str(self.profile.publish).lower(),
            "standard_deviation_m": self.profile.standard_deviation_m,
            "latency_s": self.profile.latency_s,
            "dropout_probability": self.profile.dropout_probability,
            "multipath_probability": self.profile.multipath_probability,
            "queued_measurements": len(self.queue),
        }
        status.values = [KeyValue(key=str(key), value=str(value)) for key, value in values.items()]
        array.status.append(status)
        self.diagnostic_publisher.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = GnssSimNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
