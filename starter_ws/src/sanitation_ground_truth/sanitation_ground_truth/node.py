from __future__ import annotations

import json
from pathlib import Path
import math

import yaml


def main() -> None:
    import rclpy
    from ament_index_python.packages import get_package_share_directory
    from geometry_msgs.msg import Point32, PoseWithCovarianceStamped
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from sanitation_perception.registry import GarbageRegistry
    from sanitation_perception_interfaces.msg import GarbageTarget, GarbageTargetArray
    from sanitation_ground_truth.visibility import DiscObject, visible_targets
    from std_msgs.msg import String

    class GroundTruthNode(Node):
        def __init__(self):
            super().__init__("garbage_ground_truth")
            perception_share = Path(get_package_share_directory("sanitation_perception"))
            ground_truth_share = Path(get_package_share_directory("sanitation_ground_truth"))
            self.declare_parameter("registry_path", str(perception_share / "config" / "garbage_registry.yaml"))
            self.declare_parameter("scene_path", str(ground_truth_share / "config" / "stage5a_scene.yaml"))
            self.registry = GarbageRegistry.load(str(self.get_parameter("registry_path").value))
            self.scene = yaml.safe_load(Path(str(self.get_parameter("scene_path").value)).read_text(encoding="utf-8"))
            tx, ty = (float(value) for value in self.scene["world_to_map_translation"])
            spawn_x, spawn_y, _ = (float(value) for value in self.scene["vehicle_spawn_world"])
            self.camera_xy = (spawn_x + tx + 0.53, spawn_y + ty)
            self.publisher = self.create_publisher(GarbageTargetArray, "/garbage/ground_truth", 20)
            self.diagnostics = self.create_publisher(String, "/garbage/ground_truth/diagnostics", 20)
            self.create_subscription(PoseWithCovarianceStamped, "/localization/fused_pose", self.on_pose, 20)
            self.create_timer(0.2, self.publish_truth)

        def on_pose(self, message):
            position = message.pose.pose.position
            orientation = message.pose.pose.orientation
            yaw = math.atan2(
                2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
                1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
            )
            self.camera_xy = (
                position.x + 0.53 * math.cos(yaw),
                position.y + 0.53 * math.sin(yaw),
            )

        def publish_truth(self):
            now = self.get_clock().now().to_msg()
            message = GarbageTargetArray()
            message.header.stamp = now
            message.header.frame_id = "map"
            message.registry_sha256 = self.registry.sha256
            tx, ty = (float(v) for v in self.scene["world_to_map_translation"])
            occlusion_objects = []
            for model_name, scene_spec in self.scene["objects"].items():
                entry = self.registry.resolve(model_name)
                x, y = (float(value) for value in scene_spec["pose_world"][:2])
                occlusion_objects.append(DiscObject(
                    model_name, x + tx, y + ty, max(entry.size_m[0], entry.size_m[1]) / 2.0, True
                ))
            for model_name, scene_spec in self.scene["negative_objects"].items():
                x, y = (float(value) for value in scene_spec["pose_world"])
                occlusion_objects.append(DiscObject(
                    model_name, x + tx, y + ty, float(scene_spec["radius_m"]), False
                ))
            visible = visible_targets(self.camera_xy, occlusion_objects)
            for model_name, scene_spec in self.scene["objects"].items():
                entry = self.registry.resolve(model_name)
                if entry is None or model_name not in visible:
                    continue
                x, y, z, _ = (float(v) for v in scene_spec["pose_world"])
                target = GarbageTarget()
                target.header = message.header
                target.uuid = entry.uuid
                target.class_id = entry.class_id
                target.target_type = entry.target_type
                target.confidence = 1.0
                target.map_pose.pose.position.x = x + tx
                target.map_pose.pose.position.y = y + ty
                target.map_pose.pose.position.z = z
                target.map_pose.pose.orientation.w = 1.0
                target.map_pose.covariance[0] = 1e-8
                target.map_pose.covariance[7] = 1e-8
                target.map_pose.covariance[14] = 1e-8
                target.size.x, target.size.y, target.size.z = entry.size_m
                for px, py in ((-0.5, -0.5), (0.5, -0.5), (0.5, 0.5), (-0.5, 0.5)):
                    target.polygon.points.append(Point32(x=x + tx + px * entry.size_m[0], y=y + ty + py * entry.size_m[1], z=0.0))
                target.first_seen = now
                target.last_seen = now
                target.observation_count = 1
                target.track_state = "CONFIRMED"
                target.cleaning_policy = entry.policy
                target.source_backend = "ground_truth"
                target.source_stamp = now
                target.visibility = float(visible[model_name]["visibility"])
                target.occlusion_ratio = float(visible[model_name]["occlusion_ratio"])
                target.in_keepout = False
                message.targets.append(target)
            self.publisher.publish(message)
            diagnostic = {
                "backend": "ground_truth",
                "evaluation_only": True,
                "control_allowed": False,
                "registry_sha256": self.registry.sha256,
                "published_target_count": len(message.targets),
                "registry_target_count": len(self.registry.entries),
                "fully_occluded_target_count": len(self.registry.entries) - len(message.targets),
                "occlusion_filter": "geometry_disc_fallback",
                "negative_models_published_as_targets": 0,
            }
            self.diagnostics.publish(String(data=json.dumps(diagnostic, sort_keys=True)))

    rclpy.init()
    node = GroundTruthNode()
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
