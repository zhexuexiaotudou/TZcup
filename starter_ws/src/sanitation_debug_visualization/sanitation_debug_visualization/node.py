from __future__ import annotations

import json
import math
from pathlib import Path

from sanitation_debug_visualization.model import (
    MarkerSpec,
    build_static_specs,
    load_yaml,
    predicted_specs,
    status_text,
    transform_specs_to_vehicle,
    vehicle_specs,
    yaw_from_quaternion,
)


def main() -> None:
    import rclpy
    from ament_index_python.packages import get_package_share_directory
    from geometry_msgs.msg import Point
    from nav_msgs.msg import Odometry
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sanitation_perception_interfaces.msg import (
        CleaningEvent,
        GarbageTargetArray,
    )
    from std_msgs.msg import Bool, String
    from visualization_msgs.msg import Marker, MarkerArray

    marker_types = {
        "cube": Marker.CUBE,
        "cylinder": Marker.CYLINDER,
        "line_strip": Marker.LINE_STRIP,
        "text": Marker.TEXT_VIEW_FACING,
        "arrow": Marker.ARROW,
    }

    class DebugVisualizationNode(Node):
        def __init__(self):
            super().__init__("sanitation_debug_visualization")
            perception_share = Path(get_package_share_directory("sanitation_perception"))
            truth_share = Path(get_package_share_directory("sanitation_ground_truth"))
            tasks_share = Path(get_package_share_directory("sanitation_tasks"))
            self.declare_parameter("frame_id", "base_link")
            self.declare_parameter(
                "registry_path",
                str(perception_share / "config" / "garbage_registry.yaml"),
            )
            self.declare_parameter(
                "scene_path",
                str(truth_share / "config" / "stage5a_scene.yaml"),
            )
            self.declare_parameter(
                "mission_path",
                str(tasks_share / "config" / "demo_area.yaml"),
            )
            self.declare_parameter("publish_rate_hz", 2.0)
            self.declare_parameter("show_static_targets", True)

            self.frame_id = str(self.get_parameter("frame_id").value)
            self.registry = load_yaml(str(self.get_parameter("registry_path").value))
            self.scene = load_yaml(str(self.get_parameter("scene_path").value))
            self.mission = load_yaml(str(self.get_parameter("mission_path").value))
            self.show_static_targets = bool(
                self.get_parameter("show_static_targets").value
            )
            self.predictions: list[dict] = []
            self.truth_visible_count = 0
            self.cleaned_uuids: set[str] = set()
            self.brush_enabled = False
            self.coverage_state = "NOT STARTED"
            self.spot_state = "NOT STARTED"
            self.vehicle_pose = (0.0, 0.0, 0.0)

            marker_qos = QoSProfile(depth=1)
            marker_qos.reliability = ReliabilityPolicy.RELIABLE
            marker_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self.publisher = self.create_publisher(
                MarkerArray, "/debug/markers", marker_qos
            )
            self.create_subscription(
                GarbageTargetArray,
                "/perception/garbage/targets",
                self.on_predictions,
                20,
            )
            self.create_subscription(
                GarbageTargetArray,
                "/garbage/ground_truth",
                self.on_truth,
                20,
            )
            self.create_subscription(
                CleaningEvent,
                "/garbage/cleaning_events",
                self.on_cleaning_event,
                20,
            )
            self.create_subscription(Bool, "/brush_enabled", self.on_brush, 20)
            self.create_subscription(Odometry, "/odom", self.on_odom, 20)
            self.create_subscription(
                String, "/coverage/state", self.on_coverage_state, 20
            )
            self.create_subscription(
                String, "/spot_clean/state", self.on_spot_state, 20
            )

            rate = max(float(self.get_parameter("publish_rate_hz").value), 0.2)
            self.create_timer(1.0 / rate, self.publish_markers)
            self.get_logger().info(
                f"debug markers ready: frame={self.frame_id}, "
                f"static_targets={self.show_static_targets}"
            )

        @staticmethod
        def target_dict(target) -> dict:
            pose = target.map_pose.pose
            orientation = pose.orientation
            return {
                "uuid": target.uuid,
                "class_id": target.class_id,
                "confidence": target.confidence,
                "position": (
                    pose.position.x,
                    pose.position.y,
                    pose.position.z,
                ),
                "size": (target.size.x, target.size.y, target.size.z),
                "yaw": yaw_from_quaternion(
                    orientation.x,
                    orientation.y,
                    orientation.z,
                    orientation.w,
                ),
            }

        def on_predictions(self, message):
            self.predictions = [self.target_dict(target) for target in message.targets]

        def on_truth(self, message):
            self.truth_visible_count = len(message.targets)

        def on_cleaning_event(self, message):
            if message.result == "cleaned":
                self.cleaned_uuids.add(message.target_uuid)

        def on_brush(self, message):
            self.brush_enabled = bool(message.data)

        def on_odom(self, message):
            pose = message.pose.pose
            orientation = pose.orientation
            self.vehicle_pose = (
                pose.position.x,
                pose.position.y,
                yaw_from_quaternion(
                    orientation.x,
                    orientation.y,
                    orientation.z,
                    orientation.w,
                ),
            )

        def on_coverage_state(self, message):
            self.coverage_state = message.data or "UNKNOWN"

        def on_spot_state(self, message):
            try:
                payload = json.loads(message.data)
                mode = payload.get("mode", "UNKNOWN")
                queued = payload.get("queued_target_count", "?")
                self.spot_state = f"{mode} | QUEUED {queued}"
            except (json.JSONDecodeError, TypeError):
                self.spot_state = message.data or "UNKNOWN"

        def to_marker(self, spec: MarkerSpec, stamp) -> Marker:
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = stamp
            marker.ns = spec.namespace
            marker.id = spec.marker_id
            marker.type = marker_types[spec.kind]
            marker.action = Marker.ADD
            marker.pose.position.x = spec.position[0]
            marker.pose.position.y = spec.position[1]
            marker.pose.position.z = spec.position[2]
            marker.pose.orientation.z = math.sin(spec.yaw * 0.5)
            marker.pose.orientation.w = math.cos(spec.yaw * 0.5)
            marker.scale.x = spec.scale[0]
            marker.scale.y = spec.scale[1]
            marker.scale.z = spec.scale[2]
            marker.color.r = spec.color[0]
            marker.color.g = spec.color[1]
            marker.color.b = spec.color[2]
            marker.color.a = spec.color[3]
            marker.text = spec.text
            marker.points = [Point(x=x, y=y, z=z) for x, y, z in spec.points]
            return marker

        def publish_markers(self):
            stamp = self.get_clock().now().to_msg()
            specs: list[MarkerSpec] = []
            if self.show_static_targets:
                specs.extend(
                    build_static_specs(
                        self.registry,
                        self.scene,
                        self.mission,
                        self.cleaned_uuids,
                    )
                )
            specs.extend(predicted_specs(self.predictions, self.cleaned_uuids))
            specs.extend(vehicle_specs(*self.vehicle_pose))

            outer = self.mission.get("outer_polygon", [[-2.0, -4.0], [6.0, 4.0]])
            max_x = max(float(point[0]) for point in outer)
            min_y = min(float(point[1]) for point in outer)
            specs.append(
                MarkerSpec(
                    namespace="status",
                    key="runtime",
                    kind="text",
                    position=(max_x + 1.7, min_y + 1.15, 0.55),
                    scale=(0.0, 0.0, 0.30),
                    color=(0.95, 0.98, 1.0, 1.0),
                    text=status_text(
                        prediction_count=len(self.predictions),
                        truth_visible_count=self.truth_visible_count,
                        cleaned_count=len(self.cleaned_uuids),
                        brush_enabled=self.brush_enabled,
                        coverage_state=self.coverage_state,
                        spot_state=self.spot_state,
                    ),
                )
            )
            if self.frame_id in {"base_link", "base_footprint"}:
                specs = transform_specs_to_vehicle(specs, *self.vehicle_pose)
            message = MarkerArray()
            clear = Marker()
            clear.header.frame_id = self.frame_id
            clear.header.stamp = stamp
            clear.action = Marker.DELETEALL
            message.markers.append(clear)
            message.markers.extend(self.to_marker(spec, stamp) for spec in specs)
            self.publisher.publish(message)

    rclpy.init()
    node = DebugVisualizationNode()
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
