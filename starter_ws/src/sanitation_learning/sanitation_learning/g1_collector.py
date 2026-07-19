from __future__ import annotations

import hashlib
import json
from pathlib import Path
import time

import cv2
import numpy as np


TOPICS = (
    "/g1/rgbd/image",
    "/g1/rgbd/depth_image",
    "/g1/semantic/labels_map",
    "/g1/instance/labels_map",
)


def _stamp(message) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)


def _decode_label_map(rgb: np.ndarray) -> np.ndarray:
    values = np.asarray(rgb, dtype=np.uint32)
    return values[:, :, 0] + (values[:, :, 1] << 8) + (values[:, :, 2] << 16)


def _decode_semantic_map(rgb: np.ndarray) -> np.ndarray:
    """Gazebo semantic labels repeat the uint8 class id in all RGB channels."""
    values = np.asarray(rgb, dtype=np.uint8)
    repeated = np.logical_and(values[:, :, 0] == values[:, :, 1], values[:, :, 1] == values[:, :, 2])
    if not bool(np.all(repeated)):
        raise ValueError("semantic labels_map is not a repeated-channel label image")
    return values[:, :, 0]


def main() -> None:
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--scene-seed", type=int, default=0)
    parser.add_argument("--frame-count", type=int, default=10)
    parser.add_argument("--timeout-sec", type=float, default=30.0)
    args = parser.parse_args()

    from cv_bridge import CvBridge
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image

    output = Path(args.output)
    for name in ("images", "depth", "semantic", "instances", "camera", "tf"):
        (output / name).mkdir(parents=True, exist_ok=True)

    class Collector(Node):
        def __init__(self):
            super().__init__("stage5br_g1_collector")
            self.bridge = CvBridge()
            self.buffers = {topic: {} for topic in TOPICS}
            self.camera_info = None
            self.saved = []
            self.started = time.monotonic()
            self.timed_out = False
            self.create_subscription(Image, TOPICS[0], lambda msg: self.receive(TOPICS[0], msg), qos_profile_sensor_data)
            self.create_subscription(Image, TOPICS[1], lambda msg: self.receive(TOPICS[1], msg), qos_profile_sensor_data)
            self.create_subscription(Image, TOPICS[2], lambda msg: self.receive(TOPICS[2], msg), qos_profile_sensor_data)
            self.create_subscription(Image, TOPICS[3], lambda msg: self.receive(TOPICS[3], msg), qos_profile_sensor_data)
            self.create_subscription(CameraInfo, "/g1/rgbd/camera_info", self.on_camera_info, qos_profile_sensor_data)
            self.create_timer(0.1, self.check_timeout)

        def on_camera_info(self, message):
            self.camera_info = message

        def receive(self, topic, message):
            if len(self.saved) >= args.frame_count:
                return
            stamp = _stamp(message)
            self.buffers[topic][stamp] = message
            for mapping in self.buffers.values():
                for old_stamp in sorted(mapping)[:-30]:
                    mapping.pop(old_stamp, None)
            common = set.intersection(*(set(mapping) for mapping in self.buffers.values()))
            if self.camera_info is None or not common:
                return
            for common_stamp in sorted(common):
                messages = [self.buffers[topic].pop(common_stamp) for topic in TOPICS]
                self.save_frame(common_stamp, messages)
                if len(self.saved) >= args.frame_count:
                    break

        def save_frame(self, stamp_ns, messages):
            index = len(self.saved)
            stem = f"scene_{args.scene_seed:04d}_frame_{index:02d}"
            rgb = self.bridge.imgmsg_to_cv2(messages[0], desired_encoding="rgb8")
            depth = self.bridge.imgmsg_to_cv2(messages[1], desired_encoding="passthrough")
            semantic_rgb = self.bridge.imgmsg_to_cv2(messages[2], desired_encoding="rgb8")
            instance_rgb = self.bridge.imgmsg_to_cv2(messages[3], desired_encoding="rgb8")
            semantic = _decode_semantic_map(semantic_rgb)
            instances = _decode_label_map(instance_rgb)
            paths = {
                "image": output / "images" / f"{stem}.png",
                "depth": output / "depth" / f"{stem}.npy",
                "semantic": output / "semantic" / f"{stem}.npy",
                "instances": output / "instances" / f"{stem}.npy",
                "camera": output / "camera" / f"{stem}.json",
                "tf": output / "tf" / f"{stem}.json",
            }
            cv2.imwrite(str(paths["image"]), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
            np.save(paths["depth"], np.asarray(depth, dtype=np.float32), allow_pickle=False)
            np.save(paths["semantic"], semantic, allow_pickle=False)
            np.save(paths["instances"], instances, allow_pickle=False)
            camera = {
                "width": int(self.camera_info.width), "height": int(self.camera_info.height),
                "distortion_model": self.camera_info.distortion_model,
                "d": list(self.camera_info.d), "k": list(self.camera_info.k),
                "p": list(self.camera_info.p), "frame_id": self.camera_info.header.frame_id,
            }
            paths["camera"].write_text(json.dumps(camera, indent=2) + "\n", encoding="utf-8")
            transform = {
                "parent_frame": "world", "child_frame": "g1_camera_optical",
                "source": "fixed Gazebo camera rig pose from world manifest",
                "translation_xyz_m": [0.0, 0.0, 2.6],
                "rpy_rad": [0.0, 1.57079632679, 0.0],
            }
            paths["tf"].write_text(json.dumps(transform, indent=2) + "\n", encoding="utf-8")
            record = {
                "scene_seed": args.scene_seed, "frame_index": index,
                "timestamp_ns": stamp_ns,
                "paths": {name: str(path.relative_to(output)).replace("\\", "/") for name, path in paths.items()},
                "source_encodings": [message.encoding for message in messages],
                "semantic_labels": sorted(int(value) for value in np.unique(semantic)),
                "instance_ids": sorted(int(value) for value in np.unique(instances)),
                "rgb_sha256": hashlib.sha256(paths["image"].read_bytes()).hexdigest(),
                "depth_finite_ratio": float(np.isfinite(depth).mean()),
                "exact_sensor_timestamp_match": True,
            }
            self.saved.append(record)

        def check_timeout(self):
            if time.monotonic() - self.started > args.timeout_sec:
                self.timed_out = True

    rclpy.init()
    node = Collector()
    try:
        while rclpy.ok() and len(node.saved) < args.frame_count and not node.timed_out:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        report = {
            "schema_version": 1, "dataset_domain": "G1_actual_gazebo_camera_rendered_synthetic",
            "scene_seed": args.scene_seed, "requested_frame_count": args.frame_count,
            "captured_frame_count": len(node.saved), "timed_out": node.timed_out,
            "topics": list(TOPICS), "camera_info_received": node.camera_info is not None,
            "records": node.saved,
            "capture_pass": len(node.saved) == args.frame_count and all(
                record["exact_sensor_timestamp_match"] for record in node.saved
            ),
        }
        (output / "capture_report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps(report, indent=2))
    if not report["capture_pass"]:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
