#!/usr/bin/env python3
"""Wait for the AUTO-17 ROS graph without restarting discovery each poll."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time
from urllib.request import urlopen


REQUIRED_TOPICS = {
    "/localization/fused_pose", "/map", "/scan", "/cmd_vel",
}
REQUIRED_SERVICES = {
    "/compute_coverage_path/_action/send_goal",
    "/follow_path/_action/send_goal",
    "/navigate_to_pose/_action/send_goal",
    "/controller_server/get_state",
    "/planner_server/get_state",
}


def readiness_decision(
    topics: set[str], services: set[str], controller_state: int | None,
    planner_state: int | None, dashboard_healthy: bool,
) -> bool:
    return bool(
        REQUIRED_TOPICS <= topics
        and REQUIRED_SERVICES <= services
        and controller_state == 3
        and planner_state == 3
        and dashboard_healthy
    )


def _dashboard_health(url: str) -> tuple[bool, dict | None]:
    try:
        with urlopen(url, timeout=1.0) as response:
            payload = json.loads(response.read().decode("utf-8"))
        return "mission_status" in payload, payload
    except (OSError, ValueError):
        return False, None


def run(timeout_sec: float, dashboard_url: str, output: Path) -> int:
    import rclpy
    from lifecycle_msgs.srv import GetState
    from rclpy.node import Node

    rclpy.init()
    node = Node("auto17_runtime_readiness")
    controller = node.create_client(GetState, "/controller_server/get_state")
    planner = node.create_client(GetState, "/planner_server/get_state")
    deadline = time.monotonic() + timeout_sec
    last: dict = {}
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            topics = {name for name, _types in node.get_topic_names_and_types()}
            services = {name for name, _types in node.get_service_names_and_types()}
            states: dict[str, int | None] = {"controller": None, "planner": None}
            for name, client in (("controller", controller), ("planner", planner)):
                if client.service_is_ready():
                    future = client.call_async(GetState.Request())
                    rclpy.spin_until_future_complete(node, future, timeout_sec=1.0)
                    if future.done() and future.result() is not None:
                        states[name] = int(future.result().current_state.id)
            dashboard_healthy, dashboard = _dashboard_health(dashboard_url)
            ready = readiness_decision(
                topics, services, states["controller"], states["planner"],
                dashboard_healthy,
            )
            last = {
                "ready": ready,
                "topics_present": sorted(REQUIRED_TOPICS & topics),
                "topics_missing": sorted(REQUIRED_TOPICS - topics),
                "services_present": sorted(REQUIRED_SERVICES & services),
                "services_missing": sorted(REQUIRED_SERVICES - services),
                "controller_state": states["controller"],
                "planner_state": states["planner"],
                "dashboard_healthy": dashboard_healthy,
                "dashboard": dashboard,
            }
            if ready:
                break
            time.sleep(0.2)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    output.write_text(json.dumps(last, indent=2) + "\n", encoding="utf-8")
    return 0 if last.get("ready") else 4


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=150.0)
    parser.add_argument("--dashboard-url", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    return run(args.timeout, args.dashboard_url, args.output)


if __name__ == "__main__":
    raise SystemExit(main())
