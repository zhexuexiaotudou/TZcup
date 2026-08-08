#!/usr/bin/env python3
"""Consent-gated ROS RGB-D capture and independent placement validation."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import time


CLASSES = {
    "plastic_bottle", "metal_can", "paper_litter", "leaf_pile", "puddle"
}
INDEPENDENT_METHODS = {"fiducial", "surveyed_fixture", "total_station", "motion_capture"}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def validate_placement_protocol(payload: dict) -> dict:
    errors = []
    if payload.get("schema_version") != 1:
        errors.append("schema_version must be 1")
    if payload.get("coordinate_frame") != "map":
        errors.append("coordinate_frame must be map")
    rows = payload.get("placements")
    if not isinstance(rows, list) or not rows:
        errors.append("placements must be a non-empty list")
        rows = []
    identities = set()
    for index, row in enumerate(rows):
        prefix = f"placements[{index}]"
        identity = (row.get("frame_id"), row.get("object_id"))
        if not all(identity) or identity in identities:
            errors.append(f"{prefix} frame_id/object_id missing or duplicated")
        identities.add(identity)
        if row.get("class_id") not in CLASSES:
            errors.append(f"{prefix} class_id unsupported")
        position = row.get("position_map_m")
        if not isinstance(position, list) or len(position) != 3 or not all(
            isinstance(value, (int, float)) for value in position
        ):
            errors.append(f"{prefix} position_map_m must be numeric xyz")
        if row.get("measurement_method") not in INDEPENDENT_METHODS:
            errors.append(f"{prefix} measurement_method is not independent")
        uncertainty = row.get("uncertainty_m")
        if not isinstance(uncertainty, (int, float)) or not 0 <= uncertainty <= 0.05:
            errors.append(f"{prefix} uncertainty_m must be in [0,0.05]")
        if row.get("independent_of_perception") is not True:
            errors.append(f"{prefix} must be independent_of_perception=true")
    return {
        "schema_version": 1,
        "placement_count": len(rows),
        "errors": errors,
        "independent_placement_gate_pass": not errors,
    }


def validate_command(args) -> int:
    source = Path(args.protocol).resolve()
    report = validate_placement_protocol(json.loads(source.read_text(encoding="utf-8")))
    report["protocol_sha256"] = sha256(source)
    target = Path(args.output).resolve()
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0 if report["independent_placement_gate_pass"] else 2


def capture_command(args) -> int:
    if not args.consent:
        raise RuntimeError("real RGB-D capture requires explicit --consent")
    import cv2
    from cv_bridge import CvBridge
    import message_filters
    import numpy as np
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image

    from sanitation_perception.frame_synchronizer import StrictFrameSynchronizer

    output = Path(args.output).resolve()
    for name in ("rgb", "depth", "camera_info"):
        (output / name).mkdir(parents=True, exist_ok=True)
    bridge = CvBridge()

    class CaptureNode(Node):
        def __init__(self):
            super().__init__("real_rgbd_capture")
            self.sync = StrictFrameSynchronizer(args.sync_tolerance_ms, 2)
            self.rows = []
            self.done = False
            self.last_receive_s = time.monotonic()
            self.sensor_subscribers = []
            for stream, message_type, topic in (
                ("rgb", Image, args.rgb_topic),
                ("depth", Image, args.depth_topic),
                ("camera_info", CameraInfo, args.camera_info_topic),
            ):
                subscriber = message_filters.Subscriber(
                    self, message_type, topic, qos_profile=qos_profile_sensor_data
                )
                subscriber.registerCallback(
                    lambda message, stream=stream: self.receive(stream, message)
                )
                self.sensor_subscribers.append(subscriber)

        def receive(self, stream, message):
            self.last_receive_s = time.monotonic()
            stamp = int(message.header.stamp.sec) * 1_000_000_000 + int(
                message.header.stamp.nanosec
            )
            frame = self.sync.add(stream, stamp, message)
            if frame is None or self.done:
                return
            index = len(self.rows)
            rgb = bridge.imgmsg_to_cv2(frame.rgb.payload, desired_encoding="rgb8")
            depth = bridge.imgmsg_to_cv2(frame.depth.payload, desired_encoding="passthrough")
            rgb_path = output / "rgb" / f"frame_{index:06d}.png"
            depth_path = output / "depth" / f"frame_{index:06d}.npy"
            info_path = output / "camera_info" / f"frame_{index:06d}.json"
            cv2.imwrite(str(rgb_path), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
            np.save(depth_path, depth, allow_pickle=False)
            info = frame.camera_info.payload
            info_path.write_text(
                json.dumps({"width": info.width, "height": info.height,
                            "distortion_model": info.distortion_model,
                            "d": list(info.d), "k": list(info.k), "r": list(info.r),
                            "p": list(info.p), "frame_id": info.header.frame_id}, indent=2)
                + "\n", encoding="utf-8",
            )
            self.rows.append(
                {"frame_id": f"frame_{index:06d}", "scene_id": args.scene_id,
                 "timestamp_ns": frame.rgb_stamp_ns,
                 "maximum_sync_delta_ns": frame.maximum_delta_ns,
                 "rgb": rgb_path.relative_to(output).as_posix(),
                 "depth": depth_path.relative_to(output).as_posix(),
                 "camera_info": info_path.relative_to(output).as_posix(),
                 "rgb_sha256": sha256(rgb_path), "depth_sha256": sha256(depth_path),
                 "camera_info_sha256": sha256(info_path)}
            )
            self.done = len(self.rows) >= args.frames

    rclpy.init()
    node = CaptureNode()
    started = time.monotonic()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.monotonic() - started > args.timeout_s:
                raise RuntimeError(
                    f"RGB-D capture timed out at {len(node.rows)}/{args.frames} frames"
                )
    finally:
        node.destroy_node()
        rclpy.shutdown()
    manifest = {
        "schema_version": 1, "domain": "real_rgbd", "scene_id": args.scene_id,
        "capture_consent_recorded": True, "frame_count": len(node.rows),
        "sync_tolerance_ms": args.sync_tolerance_ms, "queue_depth": 2,
        "topics": {"rgb": args.rgb_topic, "depth": args.depth_topic,
                   "camera_info": args.camera_info_topic},
        "frames": node.rows, "ground_truth_status": "UNANNOTATED",
        "independent_map_ground_truth": False,
    }
    (output / "real_rgbd_capture_manifest.json").write_text(
        json.dumps(manifest, indent=2) + "\n", encoding="utf-8"
    )
    return 0


def build_parser():
    parser = argparse.ArgumentParser()
    commands = parser.add_subparsers(dest="command", required=True)
    capture = commands.add_parser("capture")
    capture.add_argument("--output", required=True)
    capture.add_argument("--scene-id", required=True)
    capture.add_argument("--frames", type=int, required=True)
    capture.add_argument("--rgb-topic", default="/camera/color/image_raw")
    capture.add_argument("--depth-topic", default="/camera/depth/image_rect_raw")
    capture.add_argument("--camera-info-topic", default="/camera/color/camera_info")
    capture.add_argument("--sync-tolerance-ms", type=float, default=20.0)
    capture.add_argument("--timeout-s", type=float, default=300.0)
    capture.add_argument("--consent", action="store_true")
    capture.set_defaults(handler=capture_command)
    validate = commands.add_parser("validate-placement")
    validate.add_argument("--protocol", required=True)
    validate.add_argument("--output", required=True)
    validate.set_defaults(handler=validate_command)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    return args.handler(args)


if __name__ == "__main__":
    raise SystemExit(main())
