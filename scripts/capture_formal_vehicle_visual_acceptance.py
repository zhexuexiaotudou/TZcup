#!/usr/bin/env python3
"""Capture six deterministic Gazebo product/service acceptance camera topics."""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import numpy as np
from builtin_interfaces.msg import Duration
from PIL import Image as PillowImage

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


TOPICS = {
    "front_left": "/formal_visual/front_left",
    "rear_right": "/formal_visual/rear_right",
    "top_cleaning": "/formal_visual/top_cleaning",
    "sensor_tower_detail": "/formal_visual/sensor_tower_detail",
    "front_sensor_detail": "/formal_visual/front_sensor_detail",
    "arm_mount_detail": "/formal_visual/arm_mount_detail",
}

FOLDED_ARM_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]
FOLDED_ARM_POSITIONS = [-0.55, -1.75, 1.95, -1.78, -1.55, 0.25]


def decode(message: Image) -> np.ndarray:
    encoding = message.encoding.lower()
    channels_by_encoding = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4}
    if encoding not in channels_by_encoding:
        raise ValueError(f"unsupported visual-acceptance image encoding: {message.encoding}")
    channels = channels_by_encoding[encoding]
    row = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.step)
    pixels = row[:, : message.width * channels].reshape(message.height, message.width, channels)
    if encoding in {"bgr8", "bgra8"}:
        pixels = pixels[:, :, [2, 1, 0] + ([3] if channels == 4 else [])]
    if channels == 4:
        pixels = pixels[:, :, :3]
    return np.ascontiguousarray(pixels)


class CaptureNode(Node):
    def __init__(self) -> None:
        super().__init__("formal_vehicle_visual_acceptance_capture")
        self.frames: dict[str, tuple[Image, np.ndarray]] = {}
        self.arm_command = self.create_publisher(
            JointTrajectory, "/arm_controller/joint_trajectory", 1
        )
        self.gripper_command = self.create_publisher(
            JointTrajectory, "/gripper_controller/joint_trajectory", 1
        )
        self._camera_subscriptions = []
        for name, topic in TOPICS.items():
            self._camera_subscriptions.append(
                self.create_subscription(Image, topic, lambda message, key=name: self._on_image(key, message), 2)
            )

    def _on_image(self, name: str, message: Image) -> None:
        try:
            self.frames[name] = (message, decode(message))
        except ValueError as error:
            self.get_logger().error(str(error))

    def command_folded_arm(self) -> None:
        trajectory = JointTrajectory()
        trajectory.joint_names = FOLDED_ARM_JOINTS
        point = JointTrajectoryPoint()
        point.positions = FOLDED_ARM_POSITIONS
        point.time_from_start = Duration(sec=2)
        trajectory.points = [point]
        self.arm_command.publish(trajectory)
        gripper = JointTrajectory()
        gripper.joint_names = ["robotiq_85_left_knuckle_joint"]
        gripper_point = JointTrajectoryPoint()
        gripper_point.positions = [0.20]
        gripper_point.time_from_start = Duration(sec=2)
        gripper.points = [gripper_point]
        self.gripper_command.publish(gripper)


def frame_metrics(pixels: np.ndarray) -> dict[str, float | int]:
    luminance = pixels.astype(np.float32).mean(axis=2)
    dark_fraction = float((luminance < 8.0).mean())
    variation = float(luminance.std())
    return {
        "width": int(pixels.shape[1]),
        "height": int(pixels.shape[0]),
        "mean_luminance": float(luminance.mean()),
        "luminance_stddev": variation,
        "near_black_fraction": dark_fraction,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--timeout", type=float, default=60.0)
    parser.add_argument("--settle-seconds", type=float, default=18.0)
    parser.add_argument("--no-fold-arm", action="store_true")
    parser.add_argument("--bodywork-profile", choices=("product", "service"), default="product")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = CaptureNode()
    if not args.no_fold_arm:
        subscription_deadline = time.monotonic() + min(args.timeout, 20.0)
        while rclpy.ok() and (
            node.arm_command.get_subscription_count() == 0
            or node.gripper_command.get_subscription_count() == 0
        ):
            if time.monotonic() >= subscription_deadline:
                raise SystemExit("arm controller command subscription did not become available")
            rclpy.spin_once(node, timeout_sec=0.25)
        node.command_folded_arm()
        settle_deadline = time.monotonic() + args.settle_seconds
        while rclpy.ok() and time.monotonic() < settle_deadline:
            rclpy.spin_once(node, timeout_sec=0.25)
        # Discard frames received during motion so every saved image shows the
        # same product/transport pose.
        node.frames.clear()
    deadline = time.monotonic() + args.timeout
    try:
        while rclpy.ok() and set(node.frames) != set(TOPICS) and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.5)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    missing = sorted(set(TOPICS) - set(node.frames))
    if missing:
        raise SystemExit("visual acceptance cameras timed out: " + ", ".join(missing))

    reports = {}
    for name, (message, pixels) in node.frames.items():
        path = args.output / f"{name}.png"
        PillowImage.fromarray(pixels, "RGB").save(path)
        metrics = frame_metrics(pixels)
        if metrics["width"] < 1280 or metrics["height"] < 720:
            raise SystemExit(f"{name} image resolution is below acceptance minimum")
        if metrics["near_black_fraction"] > 0.95 or metrics["luminance_stddev"] < 8.0:
            raise SystemExit(f"{name} image is black or visually empty")
        reports[name] = {
            "topic": TOPICS[name],
            "encoding": message.encoding,
            "stamp_sec": message.header.stamp.sec,
            "stamp_nanosec": message.header.stamp.nanosec,
            "path": path.name,
            **metrics,
        }
    manifest = {
        "report_id": "tzcup_formal_vehicle_visual_acceptance_v1",
        "status": "GAZEBO_OGRE2_SIX_CAMERA_CAPTURE_PASSED",
        "render_source": "Gazebo Harmonic Sensors system / Ogre2",
        "bodywork_profile": args.bodywork_profile,
        "arm_pose": "folded_visual_candidate" if not args.no_fold_arm else "uncommanded",
        "camera_count": len(reports),
        "frames": reports,
        "claim_boundary": (
            f"The images prove that the committed {args.bodywork_profile} profile renders in Gazebo. "
            "They do not replace arm swept-volume, sensor self-occlusion, cleaning-contact or real-vehicle validation."
        ),
    }
    (args.output / "manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(manifest, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
