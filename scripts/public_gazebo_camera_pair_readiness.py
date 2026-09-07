#!/usr/bin/env python3
"""Read one exact public RGB/CameraInfo pair without saving a dataset frame."""
from __future__ import annotations

import argparse
import json
import math
import os
import time
from collections import OrderedDict
from pathlib import Path

RGB_CHANNELS = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4}
REQUIRED_TYPES = {"image": "sensor_msgs/msg/Image", "camera_info": "sensor_msgs/msg/CameraInfo"}
MAX_PENDING_PER_TOPIC = 4


def stamp_ns(message: object) -> int:
    stamp = message.header.stamp
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def publisher_evidence(node: object, topic: str) -> list[dict[str, str]]:
    rows = []
    for item in node.get_publishers_info_by_topic(topic):
        gid = getattr(item, "endpoint_gid", b"")
        rows.append({"node_name": str(item.node_name), "node_namespace": str(item.node_namespace), "topic_type": str(item.topic_type),
                     "endpoint_gid": bytes(gid).hex()})
    return sorted(rows, key=lambda row: (row["node_name"], row["topic_type"], row["endpoint_gid"]))


def exact_publishers(evidence: dict[str, list[dict[str, str]]]) -> bool:
    import re
    return all(len(evidence[name]) == 1 and evidence[name][0].get("node_name") == "formal_legacy_topic_adapter" and evidence[name][0].get("node_namespace") == "/" and evidence[name][0].get("topic_type") == expected and re.fullmatch(r"(?!0{32})[0-9a-f]{32}", evidence[name][0].get("endpoint_gid", "")) for name, expected in REQUIRED_TYPES.items())


def valid_image(message: object) -> bool:
    channels = RGB_CHANNELS.get(str(message.encoding))
    width, height, step = int(message.width), int(message.height), int(message.step)
    return bool(channels and width > 0 and height > 0 and step == width * channels and len(message.data) == step * height)


def validate_report(path: Path, image_topic: str, camera_info_topic: str) -> int:
    try:
        if not path.is_file() or path.is_symlink(): return 2
        report = json.loads(path.read_text(encoding="utf-8"))
        pair, evidence = report["first_exact_pair"], report["publisher_evidence"]
        if not (report["report_id"] == "tzcup_public_gazebo_camera_pair_readiness_v1" and report["status"] == "READY" and report["formal_passed"] is False and report["classification"] == "NON_FORMAL" and report["ros_domain_id"] == os.environ.get("ROS_DOMAIN_ID", "") and report["image_topic"] == image_topic and report["camera_info_topic"] == camera_info_topic and report["required_types"] == REQUIRED_TYPES and report["max_pending_per_topic"] == MAX_PENDING_PER_TOPIC and report["exact_fresh_pair_count"] == 1): return 2
        if not (isinstance(pair["frame_id"], str) and pair["frame_id"] and int(pair["stamp_ns"]) > 0 and int(pair["width"]) > 0 and int(pair["height"]) > 0 and pair["encoding"] in RGB_CHANNELS and int(pair["step"]) == int(pair["width"]) * RGB_CHANNELS[pair["encoding"]] and int(pair["image_bytes"]) == int(pair["step"]) * int(pair["height"])): return 2
        if not (exact_publishers(evidence) and report["pre_pair_publisher_evidence"] == evidence and report["post_pair_publisher_evidence"] == evidence): return 2
    except (KeyError, TypeError, ValueError, json.JSONDecodeError, OSError): return 2
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image-topic"); parser.add_argument("--camera-info-topic")
    parser.add_argument("--timeout", type=float); parser.add_argument("--output", type=Path)
    parser.add_argument("--validate-report", type=Path)
    args = parser.parse_args()
    if args.validate_report: return validate_report(args.validate_report, args.image_topic, args.camera_info_topic)
    if not args.image_topic or not args.camera_info_topic or args.timeout is None or args.output is None or args.timeout <= 0 or args.output.exists() or args.output.is_symlink(): return 2

    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image
    rclpy.init(); node = rclpy.create_node("public_gazebo_camera_pair_readiness")
    images: OrderedDict[tuple[str, int], Image] = OrderedDict()
    infos: OrderedDict[tuple[str, int], CameraInfo] = OrderedDict(); exact_pair: dict[str, object] | None = None; gate = False; accepted_evidence: dict[str, list[dict[str, str]]] | None = None; active_endpoint_evidence: dict[str, list[dict[str, str]]] | None = None

    def remember(cache: OrderedDict, key: tuple[str, int], value: object) -> None:
        cache[key] = value; cache.move_to_end(key)
        while len(cache) > MAX_PENDING_PER_TOPIC: cache.popitem(last=False)

    def observe(key: tuple[str, int]) -> None:
        nonlocal exact_pair, accepted_evidence
        image, info = images.get(key), infos.get(key)
        if image is None or info is None or exact_pair is not None: return
        if valid_image(image) and int(image.width) == int(info.width) and int(image.height) == int(info.height) and len(info.k) == 9 and all(math.isfinite(float(v)) for v in info.k) and float(info.k[0]) > 0 and float(info.k[4]) > 0:
            exact_pair = {"frame_id": key[0], "stamp_ns": key[1], "width": int(image.width), "height": int(image.height), "encoding": str(image.encoding), "step": int(image.step), "image_bytes": len(image.data)}
            accepted_evidence = evidence
            images.clear(); infos.clear()

    def on_image(message: Image) -> None:
        key = (str(message.header.frame_id), stamp_ns(message))
        if gate and key[1] > 0: remember(images, key, message); observe(key)

    def on_info(message: CameraInfo) -> None:
        key = (str(message.header.frame_id), stamp_ns(message))
        if gate and key[1] > 0: remember(infos, key, message); observe(key)

    node.create_subscription(Image, args.image_topic, on_image, qos_profile_sensor_data)
    node.create_subscription(CameraInfo, args.camera_info_topic, on_info, qos_profile_sensor_data)
    started = time.monotonic(); evidence = {"image": [], "camera_info": []}
    try:
        while time.monotonic() - started < args.timeout and exact_pair is None:
            evidence = {"image": publisher_evidence(node, args.image_topic), "camera_info": publisher_evidence(node, args.camera_info_topic)}
            gate = exact_publishers(evidence)
            if not gate or evidence != active_endpoint_evidence:
                images.clear(); infos.clear()
            active_endpoint_evidence = evidence if gate else None
            rclpy.spin_once(node, timeout_sec=0.2)
        evidence = {"image": publisher_evidence(node, args.image_topic), "camera_info": publisher_evidence(node, args.camera_info_topic)}
    finally:
        payload = {"report_id": "tzcup_public_gazebo_camera_pair_readiness_v1", "status": "READY" if exact_pair and accepted_evidence == evidence else "BLOCKED", "formal_passed": False, "classification": "NON_FORMAL", "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", ""), "image_topic": args.image_topic, "camera_info_topic": args.camera_info_topic, "required_types": REQUIRED_TYPES, "publisher_evidence": evidence, "pre_pair_publisher_evidence": accepted_evidence, "post_pair_publisher_evidence": evidence, "exact_fresh_pair_count": int(exact_pair is not None and accepted_evidence == evidence), "first_exact_pair": exact_pair, "max_pending_per_topic": MAX_PENDING_PER_TOPIC, "elapsed_s": round(time.monotonic() - started, 3), "claim_boundary": "One paired readiness observation only; no tensor, GT, pilot, full calibration, navigation, coverage, or control output."}
        pending = args.output.with_name(f".{args.output.name}.pending.{os.getpid()}")
        pending.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8"); os.replace(pending, args.output)
        node.destroy_node(); rclpy.shutdown()
    return validate_report(args.output, args.image_topic, args.camera_info_topic)


if __name__ == "__main__": raise SystemExit(main())
