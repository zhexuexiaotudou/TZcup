from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import struct
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage


IMAGE_TOPICS = {
    "rgb": "/camera/color/image_raw",
    "depth": "/camera/depth/image_rect_raw",
    "semantic": "/ground_truth/semantic/image",
    "instance": "/ground_truth/instance/image",
}


def stamp_ns(message) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)


class ContractProbe(Node):
    def __init__(self):
        super().__init__("stage5br3_runtime_contract")
        self.samples = {name: {} for name in IMAGE_TOPICS}
        self.camera_info = None
        self.transforms = []
        self.odom = []
        for name, topic in IMAGE_TOPICS.items():
            self.create_subscription(Image, topic, lambda msg, key=name: self._image(key, msg), 10)
        self.create_subscription(CameraInfo, "/camera/color/camera_info", self._camera_info, 10)
        tf_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(TFMessage, "/tf_static", self._tf, tf_qos)
        self.create_subscription(Odometry, "/ground_truth/model_odom_raw", self.odom.append, 20)
        self.cmd = self.create_publisher(Twist, "/cmd_vel", 10)

    def _camera_info(self, message):
        self.camera_info = self.camera_info or message

    def _image(self, key, message):
        bucket = self.samples[key]
        bucket[stamp_ns(message)] = message
        while len(bucket) > 30:
            bucket.pop(next(iter(bucket)))

    def common_stamp(self):
        return max(set.intersection(*(set(bucket) for bucket in self.samples.values())), default=None)

    def _tf(self, message):
        self.transforms.extend(message.transforms)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--timeout", type=float, default=35.0)
    parser.add_argument("--world-id", required=True)
    args = parser.parse_args()
    rclpy.init()
    node = ContractProbe()
    started = time.monotonic()
    while time.monotonic() - started < args.timeout and (node.common_stamp() is None or node.camera_info is None or not node.transforms or not node.odom):
        rclpy.spin_once(node, timeout_sec=0.1)
    initial_odom = node.odom[-1] if node.odom else None
    command = Twist()
    command.linear.x = 0.35
    command_started = time.monotonic()
    while time.monotonic() - command_started < 2.0:
        node.cmd.publish(command)
        rclpy.spin_once(node, timeout_sec=0.05)
    node.cmd.publish(Twist())
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.05)
    final_odom = node.odom[-1] if node.odom else None

    common_stamp = node.common_stamp()
    images = ({name: bucket[common_stamp] for name, bucket in node.samples.items()} if common_stamp is not None else {name: next(reversed(bucket.values())) for name, bucket in node.samples.items() if bucket})
    checks = {}
    checks["all_four_images_nonempty"] = len(images) == 4 and all(msg.data for msg in images.values())
    checks["camera_info_nonempty"] = node.camera_info is not None and len(node.camera_info.k) == 9
    if len(images) == 4:
        checks["resolution_640x480"] = all((msg.width, msg.height) == (640, 480) for msg in images.values())
        checks["exact_sensor_timestamps"] = common_stamp is not None
        checks["optical_frame"] = all(msg.header.frame_id == "camera_depth_link" for msg in images.values())
    else:
        checks.update(resolution_640x480=False, exact_sensor_timestamps=False, optical_frame=False)
    depth = images.get("depth")
    checks["depth_encoding_32FC1"] = depth is not None and depth.encoding == "32FC1"
    depth_stats = None
    if checks["depth_encoding_32FC1"] and len(depth.data) >= 4:
        values = struct.unpack(f"<{len(depth.data)//4}f", bytes(depth.data))
        finite = [value for value in values if math.isfinite(value)]
        depth_stats = {"finite_count": len(finite), "nan_count": sum(math.isnan(v) for v in values), "inf_count": sum(math.isinf(v) for v in values), "min_m": min(finite) if finite else None, "max_m": max(finite) if finite else None}
        checks["depth_units_and_range"] = bool(finite) and min(finite) >= 0.3 and max(finite) <= 100.0
    else:
        checks["depth_units_and_range"] = False
    transform = next((item for item in node.transforms if item.header.frame_id == "base_link" and item.child_frame_id == "camera_link"), None)
    checks["camera_to_base_tf_matches_production"] = transform is not None and all(abs(actual - expected) <= 1e-6 for actual, expected in zip((transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z), (0.53, 0.0, 0.22)))
    motion_m = None
    if initial_odom and final_odom:
        dx = final_odom.pose.pose.position.x - initial_odom.pose.pose.position.x
        dy = final_odom.pose.pose.position.y - initial_odom.pose.pose.position.y
        motion_m = math.hypot(dx, dy)
    checks["actual_vehicle_reproducible_motion"] = motion_m is not None and motion_m >= 0.25
    checks["gt_ids_are_registry_bounded"] = all(
        key in images and images[key].encoding in {"rgb8", "bgr8", "8UC3"}
        for key in ("semantic", "instance")
    )
    report = {
        "schema_version": 1, "world_id": args.world_id,
        "topics": IMAGE_TOPICS, "checks": checks,
        "common_stamp_ns": common_stamp,
        "frames": {name: message.header.frame_id for name, message in images.items()},
        "encodings": {name: message.encoding for name, message in images.items()},
        "depth": depth_stats, "vehicle_motion_m": motion_m,
        "runtime_contract_pass": all(checks.values()),
    }
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(0 if report["runtime_contract_pass"] else 2)


if __name__ == "__main__":
    main()
