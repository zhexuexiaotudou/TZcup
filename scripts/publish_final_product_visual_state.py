#!/usr/bin/env python3
"""Publish a runner-owned, atomically-written final-product visual state at 1 Hz."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


STAGES = {
    "MAPPING", "MAP_SAVED", "HARD_RESTART", "RELOAD_LOCALIZE", "COVERAGE", "PRODUCT_TERMINAL",
}
PROVIDERS = {"pc", "unavailable", "s100p"}


def _validated_payload(path: Path) -> str:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError("state must be a JSON object")
    if payload.get("field_dimensions_m") != [200, 100] or payload.get("vehicle") != "A300":
        raise ValueError("state must identify the 200x100m A300 final product")
    if payload.get("stage") not in STAGES or payload.get("perception_provider") not in PROVIDERS:
        raise ValueError("state has an unknown lifecycle stage or perception provider")
    if payload.get("formal_product_acceptance") is not False:
        raise ValueError("visual state must not claim formal product acceptance")
    digest = payload.get("map_sha256")
    if digest is not None and (
        not isinstance(digest, str)
        or len(digest) != 64
        or any(character not in "0123456789abcdef" for character in digest.lower())
    ):
        raise ValueError("map_sha256 must be null or a SHA-256 digest")
    return json.dumps(payload, ensure_ascii=False, sort_keys=True)


class FinalProductVisualStatePublisher(Node):
    def __init__(self, state_file: Path, period_sec: float) -> None:
        super().__init__("final_product_visual_state_publisher")
        self._state_file = state_file
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._publisher = self.create_publisher(String, "/final_demo/state", qos)
        self.create_timer(period_sec, self._publish)

    def _publish(self) -> None:
        try:
            rendered = _validated_payload(self._state_file)
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().warning(f"not publishing invalid final visual state: {exc}")
            return
        message = String()
        message.data = rendered
        self._publisher.publish(message)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--state-file", type=Path, required=True)
    parser.add_argument("--period-sec", type=float, default=1.0)
    args = parser.parse_args()
    if not math.isfinite(args.period_sec) or args.period_sec != 1.0:
        raise SystemExit("--period-sec must be exactly 1.0")
    rclpy.init()
    node = FinalProductVisualStatePublisher(args.state_file, args.period_sec)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
