#!/usr/bin/env python3
"""Publish a bounded false emergency-stop pulse and verify HMI observation."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time


def dashboard_observed(path: Path) -> bool:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return False
    return "/emergency_stop" in payload.get("topics_seen", [])


def run(telemetry_path: Path, output_path: Path, timeout_sec: float) -> int:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Bool

    rclpy.init()
    node = Node("auto17_emergency_stop_availability")
    publisher = node.create_publisher(
        Bool, "/safety/operator_estop_command", 10
    )
    deadline = time.monotonic() + timeout_sec
    publish_count = 0
    subscription_count = 0
    observed = False
    last_publish = 0.0
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            subscription_count = publisher.get_subscription_count()
            now = time.monotonic()
            # DDS discovery can report a matched reader slightly before that
            # reader is ready to consume data. Keep sending a bounded false
            # availability heartbeat until the dashboard confirms receipt.
            if (
                subscription_count >= 1
                and publish_count < 50
                and now - last_publish >= 0.25
            ):
                publisher.publish(Bool(data=False))
                publish_count += 1
                last_publish = now
            observed = dashboard_observed(telemetry_path)
            if publish_count >= 5 and observed:
                break
            time.sleep(0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    result = {
        "success": bool(publish_count >= 5 and observed),
        "published_false_count": publish_count,
        "matching_subscription_count": subscription_count,
        "dashboard_observed_topic": observed,
    }
    output_path.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    return 0 if result["success"] else 4


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--telemetry", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--timeout", type=float, default=15.0)
    args = parser.parse_args()
    return run(args.telemetry, args.output, args.timeout)


if __name__ == "__main__":
    raise SystemExit(main())
