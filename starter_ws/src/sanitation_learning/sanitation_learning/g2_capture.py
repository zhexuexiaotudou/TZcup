from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import time

import cv2
import numpy as np

from .g2_scene import set_poses


DEFAULT_TOPICS = ("/camera/color/image_raw", "/camera/depth/image_rect_raw", "/ground_truth/semantic/image", "/ground_truth/instance/image")


def stamp_ns(message) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)


def decode_label(rgb: np.ndarray) -> np.ndarray:
    values = np.asarray(rgb, dtype=np.uint32)
    return values[:, :, 0] + (values[:, :, 1] << 8) + (values[:, :, 2] << 16)


def outside_start_envelope(
    current_xy: tuple[float, float],
    start_xyz: list[float],
    tolerance_m: float = 0.50,
) -> bool:
    return (
        math.hypot(
            current_xy[0] - float(start_xyz[0]),
            current_xy[1] - float(start_xyz[1]),
        )
        > tolerance_m
    )


def should_reapply_start(
    saved_count: int,
    current_xy: tuple[float, float],
    start_xyz: list[float],
) -> bool:
    """Gate only frame zero; normal scene motion must not trigger a reset."""
    return saved_count == 0 and outside_start_envelope(current_xy, start_xyz)


def adjacent_translation_gate(
    records: list[dict],
    requested_frames: int,
    minimum_translation_m: float = 0.25,
) -> bool:
    if len(records) != requested_frames:
        return False
    translations = [
        math.hypot(
            current["vehicle_xy_m"][0] - previous["vehicle_xy_m"][0],
            current["vehicle_xy_m"][1] - previous["vehicle_xy_m"][1],
        )
        for previous, current in zip(records, records[1:])
    ]
    return bool(translations) and min(translations) >= minimum_translation_m


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--scene-manifest", required=True)
    parser.add_argument("--frame-count", type=int, default=10)
    parser.add_argument("--timeout", type=float, default=45.0)
    parser.add_argument("--rgb-topic", default=DEFAULT_TOPICS[0])
    parser.add_argument("--depth-topic", default=DEFAULT_TOPICS[1])
    parser.add_argument("--semantic-topic", default=DEFAULT_TOPICS[2])
    parser.add_argument("--instance-topic", default=DEFAULT_TOPICS[3])
    parser.add_argument("--camera-info-topic", default="/camera/color/camera_info")
    parser.add_argument("--odom-topic", default="/ground_truth/model_odom_raw")
    parser.add_argument("--cmd-topic", default="/cmd_vel")
    parser.add_argument("--camera-xyz", nargs=3, type=float, default=[0.53, 0.0, 0.22])
    parser.add_argument("--optical-frame", default="camera_depth_link")
    parser.add_argument("--node-name", default="stage5br3_g2_collector")
    args = parser.parse_args()
    from cv_bridge import CvBridge
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image

    output = Path(args.output)
    scene = json.loads(Path(args.scene_manifest).read_text(encoding="utf-8"))
    topics = (args.rgb_topic, args.depth_topic, args.semantic_topic, args.instance_topic)
    for name in ("rgb", "depth", "semantic", "instance", "camera", "tf", "capture"):
        (output / name).mkdir(parents=True, exist_ok=True)

    class Collector(Node):
        def __init__(self):
            super().__init__(args.node_name)
            self.bridge = CvBridge(); self.buffers = {topic: {} for topic in topics}
            self.camera = None; self.odom = None; self.last_saved_pose = None; self.saved = []
            self.dynamic_positions = []
            self.motion_enabled = False
            self.vehicle_reset_applied = False
            self.vehicle_reset_started = None
            self.started = time.monotonic(); self.publisher = self.create_publisher(Twist, args.cmd_topic, 10)
            for topic in topics:
                self.create_subscription(Image, topic, lambda msg, key=topic: self.receive(key, msg), qos_profile_sensor_data)
            self.create_subscription(CameraInfo, args.camera_info_topic, lambda msg: setattr(self, "camera", msg), qos_profile_sensor_data)
            self.create_subscription(Odometry, args.odom_topic, lambda msg: setattr(self, "odom", msg), 20)
            self.create_timer(0.05, self.tick)

        def tick(self):
            command = Twist()
            if self.motion_enabled and len(self.saved) < args.frame_count:
                command.linear.x = 0.35
            self.publisher.publish(command)

        def receive(self, topic, message):
            self.buffers[topic][stamp_ns(message)] = message
            for bucket in self.buffers.values():
                while len(bucket) > 30: bucket.pop(next(iter(bucket)))
            common = set.intersection(*(set(bucket) for bucket in self.buffers.values()))
            if not common or self.camera is None or self.odom is None or len(self.saved) >= args.frame_count:
                return
            pose = self.odom.pose.pose
            current = (pose.position.x, pose.position.y, pose.orientation.z, pose.orientation.w)
            # A world-scoped DiffDrive controller can retain the prior scene's
            # velocity while RGB-D / segmentation sensors are warming up.
            # Re-apply the manifest start pose only after every required sensor
            # and odometry stream is live, then hold zero velocity for a full
            # second before accepting frame 0. This keeps the sensor entity
            # alive without allowing cross-scene motion state to leak.
            vehicle_start = scene.get("vehicle_start_xyz_m", [-8.0, 0.0, 0.18])
            if not self.vehicle_reset_applied:
                set_poses(
                    scene["world_id"],
                    [
                        {
                            "name": "sanitation_vehicle",
                            "xyz": vehicle_start,
                            "yaw": 0.0,
                        }
                    ],
                )
                self.vehicle_reset_applied = True
                self.vehicle_reset_started = time.monotonic()
                self.odom = None
                for bucket in self.buffers.values():
                    bucket.clear()
                return
            if time.monotonic() - self.vehicle_reset_started < 1.0:
                for bucket in self.buffers.values():
                    bucket.clear()
                return
            if should_reapply_start(len(self.saved), current[:2], vehicle_start):
                set_poses(
                    scene["world_id"],
                    [
                        {
                            "name": "sanitation_vehicle",
                            "xyz": vehicle_start,
                            "yaw": 0.0,
                        }
                    ],
                )
                self.vehicle_reset_started = time.monotonic()
                self.odom = None
                for bucket in self.buffers.values():
                    bucket.clear()
                return
            if self.last_saved_pose is not None:
                translation = math.hypot(current[0] - self.last_saved_pose[0], current[1] - self.last_saved_pose[1])
                if translation < 0.25:
                    return
            stamp = min(common); messages = [self.buffers[topic].pop(stamp) for topic in topics]
            self.save(stamp, messages, current)
            self.apply_dynamic_step(len(self.saved))
            self.last_saved_pose = current
            self.motion_enabled = True

        def apply_dynamic_step(self, frame_count):
            plan = scene.get("dynamic_motion_plan")
            if not plan:
                return
            start = plan["start_xyz_m"]
            delta = plan["delta_per_frame_m"]
            xyz = [
                float(start[index]) + float(delta[index]) * frame_count
                for index in range(3)
            ]
            set_poses(
                scene["world_id"],
                [{"name": plan["model_name"], "xyz": xyz, "yaw": 0.0}],
            )
            self.dynamic_positions.append(
                {"after_frame": frame_count - 1, "xyz_m": xyz}
            )

        def save(self, stamp, messages, pose):
            index = len(self.saved); stem = f"frame_{index:02d}"
            rgb = self.bridge.imgmsg_to_cv2(messages[0], "rgb8")
            depth = self.bridge.imgmsg_to_cv2(messages[1], "passthrough").astype(np.float32)
            semantic_rgb = self.bridge.imgmsg_to_cv2(messages[2], "rgb8")
            instance_rgb = self.bridge.imgmsg_to_cv2(messages[3], "rgb8")
            if not np.all(semantic_rgb[:, :, 0] == semantic_rgb[:, :, 1]) or not np.all(semantic_rgb[:, :, 1] == semantic_rgb[:, :, 2]):
                raise RuntimeError("semantic labels are not repeated-channel IDs")
            paths = {"rgb": output/"rgb"/f"{stem}.png", "depth": output/"depth"/f"{stem}.npy", "semantic": output/"semantic"/f"{stem}.npy", "instance": output/"instance"/f"{stem}.npy", "camera": output/"camera"/f"{stem}.json", "tf": output/"tf"/f"{stem}.json", "capture": output/"capture"/f"{stem}.json"}
            cv2.imwrite(str(paths["rgb"]), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)); np.save(paths["depth"], depth, allow_pickle=False)
            np.save(paths["semantic"], semantic_rgb[:, :, 0], allow_pickle=False); np.save(paths["instance"], decode_label(instance_rgb), allow_pickle=False)
            paths["camera"].write_text(json.dumps({"width": self.camera.width, "height": self.camera.height, "k": list(self.camera.k), "p": list(self.camera.p), "frame_id": self.camera.header.frame_id}, indent=2)+"\n")
            paths["tf"].write_text(json.dumps({"world_to_base_xy": list(pose[:2]), "base_to_camera_xyz_m": list(args.camera_xyz), "optical_frame": args.optical_frame}, indent=2)+"\n")
            record = {"frame_index": index, "timestamp_ns": stamp, "vehicle_xy_m": list(pose[:2]), "exact_four_sensor_timestamp": len({stamp_ns(msg) for msg in messages}) == 1, "paths": {key: str(path.relative_to(output)).replace("\\", "/") for key, path in paths.items()}, "rgb_sha256": hashlib.sha256(paths["rgb"].read_bytes()).hexdigest()}
            paths["capture"].write_text(json.dumps(record, indent=2)+"\n"); self.saved.append(record)

    rclpy.init(); node = Collector()
    while rclpy.ok() and len(node.saved) < args.frame_count and time.monotonic() - node.started < args.timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.publisher.publish(Twist())
    dynamic_plan = scene.get("dynamic_motion_plan")
    dynamic_executed = (
        dynamic_plan is None
        or len(node.dynamic_positions) == args.frame_count
        and len({tuple(item["xyz_m"]) for item in node.dynamic_positions}) > 1
    )
    adjacent_motion_gate_pass = adjacent_translation_gate(
        node.saved, args.frame_count
    )
    report = {"schema_version": 1, "scene_seed": scene["scene_seed"], "world_id": scene["world_id"], "split": scene["split"], "requested_frames": args.frame_count, "captured_frames": len(node.saved), "topics": list(topics), "camera_xyz_m": list(args.camera_xyz), "optical_frame": args.optical_frame, "records": node.saved, "adjacent_motion_gate_pass": adjacent_motion_gate_pass, "dynamic_motion_requested": dynamic_plan is not None, "dynamic_motion_executed": dynamic_executed, "dynamic_positions": node.dynamic_positions, "capture_pass": len(node.saved) == args.frame_count and adjacent_motion_gate_pass and all(item["exact_four_sensor_timestamp"] for item in node.saved) and dynamic_executed}
    (output/"capture_report.json").write_text(json.dumps(report, indent=2)+"\n"); print(json.dumps(report, indent=2))
    node.destroy_node(); rclpy.shutdown(); raise SystemExit(0 if report["capture_pass"] else 2)


if __name__ == "__main__":
    main()
