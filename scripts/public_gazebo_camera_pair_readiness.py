#!/usr/bin/env python3
"""Read one exact public RGB/CameraInfo pair without saving a dataset frame."""
from __future__ import annotations

import argparse
import json
import os
import time
from pathlib import Path


def stamp_ns(message: object) -> int:
    stamp = message.header.stamp
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image-topic", required=True)
    parser.add_argument("--camera-info-topic", required=True)
    parser.add_argument("--timeout", type=float, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.timeout <= 0 or args.output.exists() or args.output.is_symlink():
        return 2

    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image

    rclpy.init()
    node = rclpy.create_node("public_gazebo_camera_pair_readiness")
    images: dict[tuple[str, int], Image] = {}
    infos: dict[tuple[str, int], CameraInfo] = {}
    exact_pair: dict[str, object] | None = None

    def observe(key: tuple[str, int]) -> None:
        nonlocal exact_pair
        image, info = images.get(key), infos.get(key)
        if image is None or info is None or exact_pair is not None:
            return
        if (int(image.width) != int(info.width) or int(image.height) != int(info.height)
                or not str(image.encoding) or int(image.step) <= 0 or not image.data
                or len(info.k) != 9 or float(info.k[0]) <= 0 or float(info.k[4]) <= 0):
            return
        exact_pair = {
            "frame_id": key[0], "stamp_ns": key[1], "width": int(image.width),
            "height": int(image.height), "encoding": str(image.encoding),
            "image_bytes": len(image.data),
        }

    def on_image(message: Image) -> None:
        key = (str(message.header.frame_id), stamp_ns(message))
        if key[1] > 0:
            images[key] = message
            observe(key)

    def on_info(message: CameraInfo) -> None:
        key = (str(message.header.frame_id), stamp_ns(message))
        if key[1] > 0:
            infos[key] = message
            observe(key)

    node.create_subscription(Image, args.image_topic, on_image, qos_profile_sensor_data)
    node.create_subscription(CameraInfo, args.camera_info_topic, on_info, qos_profile_sensor_data)
    started = time.monotonic()
    publishers = {"image": [], "camera_info": []}
    try:
        while time.monotonic() - started < args.timeout and exact_pair is None:
            publishers = {
                "image": sorted({f"{item.node_name}:{item.topic_type}" for item in node.get_publishers_info_by_topic(args.image_topic)}),
                "camera_info": sorted({f"{item.node_name}:{item.topic_type}" for item in node.get_publishers_info_by_topic(args.camera_info_topic)}),
            }
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        payload = {
            "report_id": "tzcup_public_gazebo_camera_pair_readiness_v1",
            "status": "READY" if exact_pair else "BLOCKED",
            "formal_passed": False,
            "classification": "NON_FORMAL",
            "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", ""),
            "image_topic": args.image_topic,
            "camera_info_topic": args.camera_info_topic,
            "required_types": {"image": "sensor_msgs/msg/Image", "camera_info": "sensor_msgs/msg/CameraInfo"},
            "publisher_types": publishers,
            "exact_fresh_pair_count": int(exact_pair is not None),
            "first_exact_pair": exact_pair,
            "elapsed_s": round(time.monotonic() - started, 3),
            "claim_boundary": "One paired readiness observation only; no tensor, GT, pilot, full calibration, navigation, coverage, or control output.",
        }
        pending = args.output.with_name(f".{args.output.name}.pending.{os.getpid()}")
        pending.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        os.replace(pending, args.output)
        node.destroy_node()
        rclpy.shutdown()
    required = {
        "image": any(item.endswith(":sensor_msgs/msg/Image") for item in publishers["image"]),
        "camera_info": any(item.endswith(":sensor_msgs/msg/CameraInfo") for item in publishers["camera_info"]),
    }
    return 0 if exact_pair and all(required.values()) else 2


if __name__ == "__main__":
    raise SystemExit(main())
