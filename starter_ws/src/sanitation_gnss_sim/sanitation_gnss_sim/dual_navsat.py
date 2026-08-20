"""Dual-antenna GNSS adapter driven only by Gazebo NavSat sensor messages."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import json
import math

from .model import GnssNoiseModel, PROFILES, local_xy_to_wgs84, wgs84_to_local_xy


@dataclass(frozen=True)
class RawNavSatSample:
    stamp_ns: int
    latitude_deg: float
    longitude_deg: float
    altitude_m: float
    velocity_x_mps: float = 0.0
    velocity_y_mps: float = 0.0
    velocity_z_mps: float = 0.0


@dataclass(frozen=True)
class DualNavSatSolution:
    stamp_ns: int
    center_x_m: float
    center_y_m: float
    altitude_m: float
    heading_rad: float
    baseline_m: float
    velocity_x_mps: float
    velocity_y_mps: float
    velocity_z_mps: float


def solve_dual_navsat(
    front: RawNavSatSample,
    rear: RawNavSatSample,
    *,
    origin_latitude_deg: float,
    origin_longitude_deg: float,
    minimum_baseline_m: float,
    maximum_baseline_m: float,
) -> DualNavSatSolution:
    front_x, front_y = wgs84_to_local_xy(
        front.latitude_deg,
        front.longitude_deg,
        origin_latitude_deg,
        origin_longitude_deg,
    )
    rear_x, rear_y = wgs84_to_local_xy(
        rear.latitude_deg,
        rear.longitude_deg,
        origin_latitude_deg,
        origin_longitude_deg,
    )
    delta_x = front_x - rear_x
    delta_y = front_y - rear_y
    baseline = math.hypot(delta_x, delta_y)
    if not math.isfinite(baseline):
        raise ValueError("dual NavSat baseline is non-finite")
    if baseline < minimum_baseline_m or baseline > maximum_baseline_m:
        raise ValueError(f"dual NavSat baseline out of range: {baseline:.6f} m")
    return DualNavSatSolution(
        stamp_ns=max(front.stamp_ns, rear.stamp_ns),
        center_x_m=0.5 * (front_x + rear_x),
        center_y_m=0.5 * (front_y + rear_y),
        altitude_m=0.5 * (front.altitude_m + rear.altitude_m),
        heading_rad=math.atan2(delta_y, delta_x),
        baseline_m=baseline,
        velocity_x_mps=0.5 * (front.velocity_x_mps + rear.velocity_x_mps),
        velocity_y_mps=0.5 * (front.velocity_y_mps + rear.velocity_y_mps),
        velocity_z_mps=0.5 * (front.velocity_z_mps + rear.velocity_z_mps),
    )


class NavSatPairBuffer:
    """Pair the latest front/rear fixes without crossing sensor epochs."""

    def __init__(self, maximum_skew_ns: int) -> None:
        if maximum_skew_ns < 0:
            raise ValueError("maximum NavSat skew must be non-negative")
        self.maximum_skew_ns = int(maximum_skew_ns)
        self.front: RawNavSatSample | None = None
        self.rear: RawNavSatSample | None = None

    def push(
        self, antenna: str, sample: RawNavSatSample
    ) -> tuple[RawNavSatSample, RawNavSatSample] | None:
        if antenna == "front":
            self.front = sample
        elif antenna == "rear":
            self.rear = sample
        else:
            raise ValueError(f"unknown antenna: {antenna}")
        if self.front is None or self.rear is None:
            return None
        skew = abs(self.front.stamp_ns - self.rear.stamp_ns)
        if skew <= self.maximum_skew_ns:
            pair = (self.front, self.rear)
            self.front = None
            self.rear = None
            return pair
        if self.front.stamp_ns < self.rear.stamp_ns:
            self.front = None
        else:
            self.rear = None
        return None


def main(args=None) -> None:
    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from geometry_msgs.msg import TwistStamped
    from gps_msgs.msg import GPSFix
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import NavSatFix, NavSatStatus
    from std_msgs.msg import Float64

    class DualNavSatAdapter(Node):
        def __init__(self) -> None:
            super().__init__("dual_navsat_adapter")
            profile_name = str(self.declare_parameter("profile", "rtk_fixed").value)
            if profile_name not in PROFILES:
                raise ValueError(f"unknown GNSS profile: {profile_name}")
            self.profile = PROFILES[profile_name]
            self.model = GnssNoiseModel(
                self.profile,
                int(self.declare_parameter("random_seed", 0).value),
            )
            self.origin_latitude = float(
                self.declare_parameter("origin_latitude_deg", 31.2304).value
            )
            self.origin_longitude = float(
                self.declare_parameter("origin_longitude_deg", 121.4737).value
            )
            minimum_baseline = float(
                self.declare_parameter("minimum_baseline_m", 0.75).value
            )
            maximum_baseline = float(
                self.declare_parameter("maximum_baseline_m", 0.85).value
            )
            self.minimum_baseline = minimum_baseline
            self.maximum_baseline = maximum_baseline
            maximum_skew_sec = float(
                self.declare_parameter("maximum_pair_skew_sec", 0.02).value
            )
            self.pairs = NavSatPairBuffer(int(maximum_skew_sec * 1e9))
            self.queue = deque()
            self.last_solution_time_sec = None
            self.last_baseline_m = None
            self.received = {"front": 0, "rear": 0}
            self.paired_count = 0
            self.rejected_count = 0
            self.fix_publisher = self.create_publisher(NavSatFix, "/gnss/fix", 20)
            self.heading_publisher = self.create_publisher(
                Float64, "/gnss/heading", 20
            )
            self.velocity_publisher = self.create_publisher(
                TwistStamped, "/gnss/velocity", 20
            )
            self.diagnostic_publisher = self.create_publisher(
                DiagnosticArray, "/gnss/diagnostics", 10
            )
            self.create_subscription(
                GPSFix,
                "/gnss/front/gps_raw",
                lambda message: self._on_raw("front", message),
                qos_profile_sensor_data,
            )
            self.create_subscription(
                GPSFix,
                "/gnss/rear/gps_raw",
                lambda message: self._on_raw("rear", message),
                qos_profile_sensor_data,
            )
            self.create_timer(0.01, self._flush_queue)
            self.create_timer(1.0, self._publish_diagnostic)

        @staticmethod
        def _sample(message) -> RawNavSatSample:
            stamp_ns = (
                int(message.header.stamp.sec) * 1_000_000_000
                + int(message.header.stamp.nanosec)
            )
            track_rad = math.radians(float(message.track))
            speed = float(message.speed)
            return RawNavSatSample(
                stamp_ns=stamp_ns,
                latitude_deg=float(message.latitude),
                longitude_deg=float(message.longitude),
                altitude_m=float(message.altitude),
                velocity_x_mps=speed * math.sin(track_rad),
                velocity_y_mps=speed * math.cos(track_rad),
                velocity_z_mps=float(message.climb),
            )

        def _on_raw(self, antenna: str, message) -> None:
            self.received[antenna] += 1
            pair = self.pairs.push(antenna, self._sample(message))
            if pair is None:
                return
            try:
                solution = solve_dual_navsat(
                    pair[0],
                    pair[1],
                    origin_latitude_deg=self.origin_latitude,
                    origin_longitude_deg=self.origin_longitude,
                    minimum_baseline_m=self.minimum_baseline,
                    maximum_baseline_m=self.maximum_baseline,
                )
            except ValueError as exc:
                self.rejected_count += 1
                self.get_logger().warning(str(exc))
                return
            self.paired_count += 1
            self.last_baseline_m = solution.baseline_m
            sample_time_sec = solution.stamp_ns / 1e9
            dt_sec = 1.0 / self.profile.rate_hz
            if self.last_solution_time_sec is not None:
                dt_sec = max(0.0, sample_time_sec - self.last_solution_time_sec)
            self.last_solution_time_sec = sample_time_sec
            measurement = self.model.sample(
                solution.center_x_m,
                solution.center_y_m,
                dt_sec,
                solution.heading_rad,
            )
            if not measurement.publish:
                return
            release_time = self.get_clock().now().nanoseconds / 1e9
            release_time += self.profile.latency_s
            self.queue.append(
                (
                    release_time,
                    solution,
                    measurement,
                )
            )

        def _flush_queue(self) -> None:
            now_seconds = self.get_clock().now().nanoseconds / 1e9
            while self.queue and self.queue[0][0] <= now_seconds:
                _, solution, measurement = self.queue.popleft()
                latitude, longitude = local_xy_to_wgs84(
                    measurement.x_m,
                    measurement.y_m,
                    self.origin_latitude,
                    self.origin_longitude,
                )
                fix = NavSatFix()
                fix.header.stamp.sec = solution.stamp_ns // 1_000_000_000
                fix.header.stamp.nanosec = solution.stamp_ns % 1_000_000_000
                fix.header.frame_id = "gnss_center_link"
                fix.status.status = NavSatStatus.STATUS_GBAS_FIX
                fix.status.service = NavSatStatus.SERVICE_GPS
                fix.latitude = latitude
                fix.longitude = longitude
                fix.altitude = solution.altitude_m
                fix.position_covariance[0] = measurement.variance_m2
                fix.position_covariance[4] = measurement.variance_m2
                fix.position_covariance[8] = max(0.01, measurement.variance_m2 * 4.0)
                fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
                self.fix_publisher.publish(fix)
                self.heading_publisher.publish(Float64(data=measurement.heading_rad))
                velocity = TwistStamped()
                velocity.header = fix.header
                velocity.header.frame_id = "map"
                velocity.twist.linear.x = solution.velocity_x_mps
                velocity.twist.linear.y = solution.velocity_y_mps
                velocity.twist.linear.z = solution.velocity_z_mps
                self.velocity_publisher.publish(velocity)

        def _publish_diagnostic(self) -> None:
            payload = {
                "profile": self.profile.name,
                "input_source": "gazebo_dual_navsat",
                "ground_truth_ros_subscription": False,
                "front_received": self.received["front"],
                "rear_received": self.received["rear"],
                "paired_count": self.paired_count,
                "rejected_count": self.rejected_count,
                "last_baseline_m": self.last_baseline_m,
                "queued_measurements": len(self.queue),
            }
            array = DiagnosticArray()
            array.header.stamp = self.get_clock().now().to_msg()
            status = DiagnosticStatus()
            status.name = "dual_navsat_adapter"
            status.hardware_id = "gazebo_navsat_sensor_pair"
            status.level = (
                DiagnosticStatus.OK
                if self.paired_count > 0 and self.rejected_count == 0
                else DiagnosticStatus.WARN
            )
            status.message = json.dumps(payload, sort_keys=True)
            status.values = [
                KeyValue(key=str(key), value=str(value))
                for key, value in payload.items()
            ]
            array.status.append(status)
            self.diagnostic_publisher.publish(array)

    rclpy.init(args=args)
    node = DualNavSatAdapter()
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
