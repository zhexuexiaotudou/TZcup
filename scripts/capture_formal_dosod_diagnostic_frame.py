#!/usr/bin/env python3
"""Stage one cube in front of the fixed vehicle and save a real Gazebo frame.

This is an evaluator-only diagnostic utility.  It does not start or feed the
product node and does not publish labels, detections, TF, or control commands.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import time

import cv2
import numpy as np


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--entity", required=True)
    parser.add_argument("--world", default="campus_formal")
    parser.add_argument("--distance-m", type=float, default=1.15)
    parser.add_argument("--edge-m", type=float, default=0.03)
    parser.add_argument("--episode-manifest", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()

    import rclpy
    from cv_bridge import CvBridge
    from geometry_msgs.msg import Pose
    from rclpy.duration import Duration
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from ros_gz_interfaces.srv import SetEntityPose
    from sensor_msgs.msg import CameraInfo, Image
    from tf2_ros import Buffer, TransformListener

    class CaptureNode(Node):
        def __init__(self) -> None:
            super().__init__("formal_dosod_evaluator_frame_capture")
            self.bridge = CvBridge()
            self.info = None
            self.image = None
            self.tf_buffer = Buffer(cache_time=Duration(seconds=20.0))
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.client = self.create_client(SetEntityPose, f"/world/{args.world}/set_pose")
            self.create_subscription(
                CameraInfo,
                "/sensors/front_rgbd/depth/image_rect_raw/camera_info",
                self._on_info,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                Image,
                "/sensors/front_rgbd/depth/image_rect_raw/image",
                self._on_image,
                qos_profile_sensor_data,
            )

        def _on_info(self, message) -> None:
            self.info = message

        def _on_image(self, message) -> None:
            self.image = message

    def spin_until(node, predicate, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if predicate():
                return True
        return False

    rclpy.init()
    node = CaptureNode()
    try:
        if not node.client.wait_for_service(timeout_sec=60.0):
            raise RuntimeError("Gazebo SetEntityPose service unavailable")
        if not spin_until(node, lambda: node.info is not None and node.image is not None, 60.0):
            raise RuntimeError("front RGB/CameraInfo unavailable")
        manifest = json.loads(args.episode_manifest.read_text(encoding="utf-8"))
        start = manifest.get("vehicle_start_pose_source_world") or manifest["vehicle_start_pose_map"]
        base_x, base_y, yaw = float(start["x_m"]), float(start["y_m"]), float(start["yaw_rad"])
        target_x = base_x + args.distance_m * math.cos(yaw)
        target_y = base_y + args.distance_m * math.sin(yaw)
        request = SetEntityPose.Request()
        request.entity.name = args.entity
        request.pose = Pose()
        request.pose.position.x = target_x
        request.pose.position.y = target_y
        request.pose.position.z = args.edge_m / 2.0
        request.pose.orientation.w = 1.0
        future = node.client.call_async(request)
        if not spin_until(node, future.done, 10.0) or future.result() is None or not future.result().success:
            raise RuntimeError(f"failed to stage cube entity {args.entity}")
        staged_at = time.monotonic()
        last_stamp = None

        def settled_frame_ready() -> bool:
            nonlocal last_stamp
            if time.monotonic() - staged_at < 2.0 or node.image is None:
                return False
            stamp = (int(node.image.header.stamp.sec), int(node.image.header.stamp.nanosec))
            if stamp == last_stamp:
                return False
            last_stamp = stamp
            return True

        if not spin_until(node, settled_frame_ready, 20.0):
            raise RuntimeError("no settled front RGB frame after staging cube")
        image_message = node.image
        info = node.info
        camera_tf = node.tf_buffer.lookup_transform(
            "base_link", image_message.header.frame_id, Time(), timeout=Duration(seconds=5.0)
        )
        translation, rotation = camera_tf.transform.translation, camera_tf.transform.rotation
        x, y, z, w = rotation.x, rotation.y, rotation.z, rotation.w
        scale = 2.0 / (x * x + y * y + z * z + w * w)
        base_from_camera = np.asarray(
            [
                [1 - scale * (y * y + z * z), scale * (x * y - z * w), scale * (x * z + y * w), translation.x],
                [scale * (x * y + z * w), 1 - scale * (x * x + z * z), scale * (y * z - x * w), translation.y],
                [scale * (x * z - y * w), scale * (y * z + x * w), 1 - scale * (x * x + y * y), translation.z],
                [0.0, 0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
        map_from_base = np.asarray(
            [
                [cos_yaw, -sin_yaw, 0.0, base_x],
                [sin_yaw, cos_yaw, 0.0, base_y],
                # The public spawn pose is the ground datum; base_link is
                # 0.1651 m above it in the frozen A300 platform contract.
                [0.0, 0.0, 1.0, 0.1701],
                [0.0, 0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )
        map_from_camera = map_from_base @ base_from_camera
        half = args.edge_m / 2.0
        corners = np.asarray(
            [
                [target_x + dx, target_y + dy, half + dz, 1.0]
                for dx in (-half, half)
                for dy in (-half, half)
                for dz in (-half, half)
            ],
            dtype=np.float64,
        ).T
        camera_points = np.linalg.inv(map_from_camera) @ corners
        fx, fy, cx, cy = float(info.k[0]), float(info.k[4]), float(info.k[2]), float(info.k[5])
        us = fx * camera_points[0] / camera_points[2] + cx
        vs = fy * camera_points[1] / camera_points[2] + cy
        truth_box = [float(us.min()), float(vs.min()), float(us.max()), float(vs.max())]
        rgb = np.asarray(node.bridge.imgmsg_to_cv2(image_message, desired_encoding="rgb8"), dtype=np.uint8)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(args.output), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)):
            raise RuntimeError(f"failed to save {args.output}")
        metadata = {
            "source_topic": "/sensors/front_rgbd/depth/image_rect_raw/image",
            "source_frame_id": image_message.header.frame_id,
            "source_stamp": {"sec": last_stamp[0], "nanosec": last_stamp[1]},
            "real_gazebo_camera_frame": True,
            "evaluator_truth_overlay_only": True,
            "staged_entity": args.entity,
            "vehicle_start_pose_source": str(args.episode_manifest),
            "staged_map_pose": {"x_m": target_x, "y_m": target_y, "z_m": args.edge_m / 2.0},
            "truth_boxes_xyxy": [{"object_id": args.entity, "class_id": "litter_cube", "xyxy": truth_box}],
            "image_path": str(args.output),
            "image_shape_hwc": list(rgb.shape),
        }
        args.output.with_suffix(".json").write_text(
            json.dumps(metadata, ensure_ascii=False, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        print(json.dumps({"output": str(args.output), "truth_box_xyxy": truth_box}))
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
