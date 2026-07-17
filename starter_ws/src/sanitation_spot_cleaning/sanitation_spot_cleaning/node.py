from __future__ import annotations

import json


def main() -> None:
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from sanitation_perception_interfaces.msg import CleaningEvent, GarbageTargetArray
    from std_msgs.msg import Bool, String

    class SpotCleaningNode(Node):
        def __init__(self):
            super().__init__("spot_cleaning_coordinator")
            self.declare_parameter("mode", "deferred")
            self.coverage_state = "UNKNOWN"
            self.queued: dict[str, object] = {}
            self.event_publisher = self.create_publisher(CleaningEvent, "/garbage/cleaning_events", 20)
            self.state_publisher = self.create_publisher(String, "/spot_clean/state", 20)
            self.brush_publisher = self.create_publisher(Bool, "/brush_enabled", 20)
            self.create_subscription(GarbageTargetArray, "/perception/garbage/targets", self.on_targets, 20)
            self.create_subscription(String, "/coverage/state", self.on_coverage_state, 20)
            self.create_timer(0.2, self.publish_state)

        def on_coverage_state(self, message):
            self.coverage_state = message.data

        def on_targets(self, message):
            for target in message.targets:
                if target.source_backend == "ground_truth":
                    self.get_logger().error("GT control violation rejected")
                    continue
                if target.track_state in {"CONFIRMED", "QUEUED"} and not target.in_keepout:
                    self.queued[target.uuid] = target

        def publish_state(self):
            mode = str(self.get_parameter("mode").value)
            payload = {
                "mode": mode,
                "coverage_state": self.coverage_state,
                "queued_target_count": len(self.queued),
                "execution_policy": "component_boundary" if mode == "deferred" else mode,
                "ground_truth_control_allowed": False,
            }
            self.state_publisher.publish(String(data=json.dumps(payload, sort_keys=True)))
            self.brush_publisher.publish(Bool(data=False))

    rclpy.init()
    node = SpotCleaningNode()
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
