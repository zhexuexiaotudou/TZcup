"""ROS adapter from formal product perception to active-cleaning belief topics."""

from __future__ import annotations

import math
import time

from .formal_observation_core import (
    FormalObservationBridgeCore,
    ProductTargetObservation,
    PublicPlanningMap,
    observation_stamp_reason,
)


CONTROL_INPUT_TOPICS = (
    "/perception/ground_dirt/masks",
    "/perception/garbage/targets",
)
PRODUCT_OUTPUT_TOPICS = (
    "/active_cleaning/ground_dirt_belief",
    "/active_cleaning/garbage_targets",
    "/active_cleaning/observation_ready",
)


def main() -> None:
    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from nav_msgs.msg import OccupancyGrid
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sanitation_perception_interfaces.msg import GarbageTargetArray
    from sensor_msgs.msg import Image
    from std_msgs.msg import Bool

    class FormalObservationBridge(Node):
        def __init__(self) -> None:
            super().__init__("formal_active_cleaning_observation_bridge")
            self.declare_parameter("occupancy_map", "")
            self.declare_parameter("mission_geometry", "")
            self.declare_parameter("materialization_contract", "")
            self.declare_parameter("mask_topic", CONTROL_INPUT_TOPICS[0])
            self.declare_parameter("targets_topic", CONTROL_INPUT_TOPICS[1])
            self.declare_parameter("belief_topic", PRODUCT_OUTPUT_TOPICS[0])
            self.declare_parameter("filtered_targets_topic", PRODUCT_OUTPUT_TOPICS[1])
            self.declare_parameter("ready_topic", PRODUCT_OUTPUT_TOPICS[2])
            self.declare_parameter(
                "status_topic", "/active_cleaning/observation_status"
            )
            self.declare_parameter("min_target_confidence", 0.50)
            self.declare_parameter("max_observation_age_sec", 1.50)
            self._max_age = float(self.get_parameter("max_observation_age_sec").value)
            if not math.isfinite(self._max_age) or self._max_age <= 0.0:
                raise RuntimeError("max_observation_age_sec must be finite and positive")
            planning_map = PublicPlanningMap.load(
                str(self.get_parameter("occupancy_map").value),
                str(self.get_parameter("mission_geometry").value),
                str(self.get_parameter("materialization_contract").value),
            )
            self._core = FormalObservationBridgeCore(
                planning_map,
                min_target_confidence=float(
                    self.get_parameter("min_target_confidence").value
                ),
            )
            self._last_mask_time: float | None = None
            self._last_targets_time: float | None = None
            self._last_mask_stamp: int | None = None
            self._last_targets_stamp: int | None = None
            self._last_mask_reason = "not_received"
            self._last_targets_reason = "not_received"
            self._target_count = 0
            latched = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self._belief_publisher = self.create_publisher(
                OccupancyGrid, str(self.get_parameter("belief_topic").value), latched
            )
            self._targets_publisher = self.create_publisher(
                GarbageTargetArray,
                str(self.get_parameter("filtered_targets_topic").value),
                10,
            )
            self._ready_publisher = self.create_publisher(
                Bool, str(self.get_parameter("ready_topic").value), latched
            )
            self._status_publisher = self.create_publisher(
                DiagnosticArray,
                str(self.get_parameter("status_topic").value),
                latched,
            )
            self.create_subscription(
                Image,
                str(self.get_parameter("mask_topic").value),
                self._on_mask,
                10,
            )
            self.create_subscription(
                GarbageTargetArray,
                str(self.get_parameter("targets_topic").value),
                self._on_targets,
                10,
            )
            self.create_timer(0.20, self._publish_status)
            self._publish_belief()
            self._publish_status()

        def _on_mask(self, message: Image) -> None:
            stamp = self._stamp_ns(message.header.stamp)
            self._last_mask_reason = observation_stamp_reason(
                stamp, self.get_clock().now().nanoseconds, self._max_age,
                self._last_mask_stamp,
            )
            if self._last_mask_reason != "accepted":
                self._last_mask_time = None
                self._publish_status()
                return
            update = self._core.update_projected_mask(
                frame_id=message.header.frame_id,
                width=int(message.width),
                height=int(message.height),
                encoding=message.encoding,
                step=int(message.step),
                data=message.data,
            )
            self._last_mask_reason = update.reason
            if not update.accepted:
                self._last_mask_time = None
                self.get_logger().error(
                    f"map-projected dirt mask rejected: {update.reason}"
                )
                self._publish_status()
                return
            self._last_mask_time = time.monotonic()
            self._last_mask_stamp = stamp
            self._publish_belief()
            self._publish_status()

        def _on_targets(self, message: GarbageTargetArray) -> None:
            stamp = self._stamp_ns(message.header.stamp)
            now_ns = self.get_clock().now().nanoseconds
            self._last_targets_reason = observation_stamp_reason(
                stamp, now_ns, self._max_age, self._last_targets_stamp,
            )
            if message.header.frame_id != self._core.map.frame_id:
                self._last_targets_reason = "frame_mismatch"
            for target in message.targets:
                if target.header.frame_id != self._core.map.frame_id:
                    self._last_targets_reason = "target_frame_mismatch"
                    break
                if any(observation_stamp_reason(self._stamp_ns(value), now_ns,
                                                self._max_age) != "accepted"
                       for value in (target.header.stamp, target.source_stamp,
                                     target.last_seen)):
                    self._last_targets_reason = "target_source_stamp_invalid"
                    break
            if self._last_targets_reason != "accepted":
                self._last_targets_time = None
                self._publish_status()
                return
            candidates = []
            by_id = {}
            for target in message.targets:
                position = target.map_pose.pose.position
                item = ProductTargetObservation(
                    target_id=target.uuid,
                    x=float(position.x),
                    y=float(position.y),
                    confidence=float(target.confidence),
                    source_backend=target.source_backend,
                    track_state=target.track_state,
                    in_keepout=bool(target.in_keepout),
                )
                candidates.append(item)
                by_id[item.target_id] = target
            accepted = self._core.replace_targets(candidates)
            output = GarbageTargetArray()
            output.header = message.header
            output.registry_sha256 = message.registry_sha256
            output.targets = [by_id[item.target_id] for item in accepted]
            self._targets_publisher.publish(output)
            self._target_count = len(output.targets)
            self._last_targets_time = time.monotonic()
            self._last_targets_stamp = stamp
            self._publish_status()

        @staticmethod
        def _stamp_ns(stamp) -> int:
            if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
                return -1
            return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

        def _publish_belief(self) -> None:
            planning_map = self._core.map
            message = OccupancyGrid()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = planning_map.frame_id
            message.info.resolution = planning_map.resolution
            message.info.width = planning_map.width
            message.info.height = planning_map.height
            message.info.origin.position.x = planning_map.origin_x
            message.info.origin.position.y = planning_map.origin_y
            message.info.origin.orientation.w = 1.0
            message.data = list(self._core.occupancy_grid_values())
            self._belief_publisher.publish(message)

        def _publish_status(self) -> None:
            now = time.monotonic()
            now_ns = self.get_clock().now().nanoseconds
            mask_fresh = (
                self._last_mask_time is not None
                and 0.0 <= now - self._last_mask_time <= self._max_age
                and observation_stamp_reason(self._last_mask_stamp or 0, now_ns,
                                             self._max_age) == "accepted"
            )
            targets_fresh = (
                self._last_targets_time is not None
                and 0.0 <= now - self._last_targets_time <= self._max_age
                and observation_stamp_reason(self._last_targets_stamp or 0, now_ns,
                                             self._max_age) == "accepted"
            )
            ready = mask_fresh and targets_fresh
            self._ready_publisher.publish(Bool(data=ready))
            status = DiagnosticStatus()
            status.name = "formal_active_cleaning_observation_bridge"
            status.hardware_id = "product_perception_public_map"
            status.level = (
                DiagnosticStatus.OK if ready else DiagnosticStatus.ERROR
            )
            status.message = "READY" if ready else "BLOCKED"
            status.values = [
                KeyValue(key="mask_fresh", value=str(mask_fresh).lower()),
                KeyValue(key="targets_fresh", value=str(targets_fresh).lower()),
                KeyValue(key="last_mask_reason", value=self._last_mask_reason),
                KeyValue(key="last_targets_reason", value=self._last_targets_reason),
                KeyValue(key="accepted_target_count", value=str(self._target_count)),
                KeyValue(key="control_input_contract", value="product_only"),
            ]
            message = DiagnosticArray()
            message.header.stamp = self.get_clock().now().to_msg()
            message.status = [status]
            self._status_publisher.publish(message)

    rclpy.init()
    node = None
    try:
        node = FormalObservationBridge()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None and rclpy.ok():
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
